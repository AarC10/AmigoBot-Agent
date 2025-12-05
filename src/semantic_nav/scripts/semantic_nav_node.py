#!/usr/bin/env python3
import math
import threading

import actionlib
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from vlm_clip.srv import QueryTarget, QueryTargetRequest, QueryTargetResponse
from semantic_nav.msg import GoToTargetAction, GoToTargetFeedback, GoToTargetResult
from std_srvs.srv import Trigger, TriggerResponse

class SemanticNavigator(object):
    STATE_IDLE = "IDLE"
    STATE_ALIGN = "ALIGN"
    STATE_APPROACH = "APPROACH"
    STATE_STOPPED = "STOPPED"

    def __init__(self):
        # Parameters
        self.target_label = rospy.get_param("~target_label", "TODO")
        self.vlm_service_name = rospy.get_param("~vlm_service_name", "/query_target")

        # TODO: Need to remember the sonar topics used in Rosaria
        self.sonar_topic = rospy.get_param("~sonar_topic", "/RosAria/sonar")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        self.control_rate_hz = float(rospy.get_param("~control_rate_hz", 10.0))

        self.align_eps_deg = float(rospy.get_param("~align_eps_deg", 5.0))
        self.turn_gain = float(rospy.get_param("~turn_gain", 0.01))  # rad/s per deg
        self.max_turn_rate = float(rospy.get_param("~max_turn_rate", 0.6))  # rad/s

        self.forward_speed = float(rospy.get_param("~forward_speed", 0.1))  # m/s
        self.stop_distance = float(rospy.get_param("~stop_distance", 0.5))   # m
        self.front_sector_deg = float(rospy.get_param("~front_sector_deg", 30.0))

        # internal state
        self.state = self.STATE_IDLE
        self.last_bearing_deg = 0.0

        self.active = False
        self._current_goal_active = False
        self._goal_start_time = None

        # sonar buffer
        self._sonar_lock = threading.Lock()
        self._sonar_points = None

        # pubs and subs :)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.sonar_sub = rospy.Subscriber(
            self.sonar_topic, PointCloud, self.sonar_callback, queue_size=1
        )

        # Should wait for VLM service to be available
        rospy.loginfo("semantic_nav: waiting for VLM service '%s'", self.vlm_service_name)
        try:
            rospy.wait_for_service(self.vlm_service_name, timeout=10.0)
        except rospy.ROSException:
            rospy.logwarn("semantic_nav: VLM service '%s' not available at startup", self.vlm_service_name)
        self.vlm_query = rospy.ServiceProxy(self.vlm_service_name, QueryTarget)


        # start/stop services
        self.start_srv = rospy.Service("~start", Trigger, self.handle_start)
        self.stop_srv = rospy.Service("~stop", Trigger, self.handle_stop)

        # target action server
        self.action_server = actionlib.SimpleActionServer(
            "~go_to_target",
            GoToTargetAction,
            execute_cb=self.execute_goal,
            auto_start=False
        )
        self.action_server.start()

        # control loop timer
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate_hz), self.control_loop)

        rospy.loginfo("semantic_nav: initialized. Target='%s', sonar='%s', cmd_vel='%s'",
                      self.target_label, self.sonar_topic, self.cmd_vel_topic)

    # start/stop
    def handle_start(self, req):
        self.active = True
        self.state = self.STATE_IDLE
        rospy.loginfo("semantic_nav: start requested (service), state set to IDLE")
        return TriggerResponse(success=True, message="Semantic navigation started.")

    def handle_stop(self, req):
        self.active = False
        self._current_goal_active = False
        self.state = self.STATE_IDLE
        self._publish_stop()
        rospy.loginfo("semantic_nav: stop requested (service), navigation halted")
        return TriggerResponse(success=True, message="Semantic navigation stopped.")


    # Callbacks
    def sonar_callback(self, msg: PointCloud):
        with self._sonar_lock:
            self._sonar_points = msg.points

    # Core loop
    def control_loop(self, event):
        # Shouldn't do stuff if inactive
        if not self.active:
            self._publish_stop()
            return

        # Alright now we can do stuff
        try:
            resp = self._call_vlm()
        except Exception as e:
            rospy.logwarn_throttle(2.0, "semantic_nav: VLM call failed: %s", str(e))
            resp = None

        if self.state == self.STATE_IDLE:
            self._handle_idle(resp)
        elif self.state == self.STATE_ALIGN:
            self._handle_align(resp)
        elif self.state == self.STATE_APPROACH:
            self._handle_approach(resp)
        elif self.state == self.STATE_STOPPED:
            self._publish_stop()
        else:
            rospy.logwarn_throttle(5.0, "semantic_nav: unknown state '%s', resetting to IDLE", self.state)
            self.state = self.STATE_IDLE
            self._publish_stop()

    # State Handlers
    def _handle_idle(self, resp: QueryTargetResponse):
        # Attempts to acquire the target
        if resp is not None and resp.found:
            self.last_bearing_deg = resp.bearing_deg
            rospy.loginfo("semantic_nav: target '%s' acquired in IDLE (bearing=%.1f deg)",
                          self.target_label, self.last_bearing_deg)
            self.state = self.STATE_ALIGN
        else:
            # Remain stopped!!!
            self._publish_stop()

    def _handle_align(self, resp: QueryTargetResponse):
        # Check if we lost the target
        if resp is None or not resp.found:
            rospy.loginfo_throttle(5.0, "semantic_nav: lost target during ALIGN, returning to IDLE")
            self.state = self.STATE_IDLE
            self._publish_stop()
            return

        # calc turn rate
        bearing_deg = resp.bearing_deg
        self.last_bearing_deg = bearing_deg

        ang_z = self.turn_gain * bearing_deg  # rad/s
        ang_z = max(-self.max_turn_rate, min(self.max_turn_rate, ang_z))

        twist = Twist()
        twist.angular.z = ang_z

        # Check for alignment. If aligned, switch to APPROACH
        if abs(bearing_deg) < self.align_eps_deg:
            rospy.loginfo("semantic_nav: alignment achieved (bearing=%.1f deg), switching to APPROACH",
                          bearing_deg)
            twist.angular.z = 0.0
            self.state = self.STATE_APPROACH

        self.cmd_pub.publish(twist)

    def _handle_approach(self, resp: QueryTargetResponse):
        # Obstacle check
        front_dist = self._get_front_obstacle_distance()
        if front_dist is not None:
            self.last_front_distance = front_dist
        else:
            self.last_front_distance = float("nan")

        # Stop if too close
        if front_dist is not None and front_dist < self.stop_distance:
            rospy.loginfo("semantic_nav: obstacle within %.2f m (%.2f m), stopping",
                          self.stop_distance, front_dist)
            self.state = self.STATE_STOPPED
            self._publish_stop()
            return

        # slight heading correction if we still see the target
        bearing_deg = 0.0
        if resp is not None and resp.found:
            bearing_deg = resp.bearing_deg
            self.last_bearing_deg = bearing_deg

        ang_z = self.turn_gain * bearing_deg
        ang_z = max(-self.max_turn_rate, min(self.max_turn_rate, ang_z))

        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.angular.z = ang_z

        self.cmd_pub.publish(twist)

    # Helper funcs
    def _call_vlm(self):
        if self.vlm_query is None:
            return None
        req = QueryTargetRequest()
        req.query = self.target_label
        return self.vlm_query(req)

    def _get_front_obstacle_distance(self):
        with self._sonar_lock:
            points = list(self._sonar_points) if self._sonar_points is not None else None

        if not points:
            return None

        front_sector_rad = math.radians(self.front_sector_deg)
        min_dist = None

        for point in points:
            x = point.x
            y = point.y
            dist = math.hypot(x, y)
            if dist <= 0.0:
                continue

            angle = math.atan2(y, x)  # 0 is straight ahead
            if abs(angle) <= front_sector_rad:
                if (min_dist is None) or (dist < min_dist):
                    min_dist = dist

        return min_dist

    def _publish_stop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)


def main():
    rospy.init_node("semantic_nav")
    node = SemanticNavigator()
    rospy.spin()


if __name__ == "__main__":
    main()

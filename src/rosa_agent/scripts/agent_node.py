#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
# Ensure ROS/system site-packages are visible before any ROS imports
sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")
sys.path.insert(0, "/usr/lib/python3/dist-packages")

try:
    import rospkg  # noqa: F401
except Exception as e:
    raise ImportError("rospkg not found. Ensure ROS Python site-packages are on sys.path and apt package python3-rospkg is installed.") from e

import math
import asyncio
import rospy
import threading
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from langchain.agents import tool
from langchain_ollama import ChatOllama
from rosa import ROSA

class MotionHelper:
    def __init__(self):
        self._odom_lock = threading.Lock()
        self._last_odom = None

        self._odom_sub = rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=1)
        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def _odom_cb(self, msg):
        with self._odom_lock:
            self._last_odom = msg

    def _get_xy_yaw(self):
        with self._odom_lock:
            msg = self._last_odom
        if msg is None:
            return None, None, None

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        return x, y, yaw

    def _wait_for_odom(self, timeout=3.0):
        start = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            x, y, yaw = self._get_xy_yaw()
            if x is not None:
                return True
            if (rospy.Time.now() - start).to_sec() > timeout:
                return False
            rate.sleep()
        return False

    def drive(self, distance_m: float, speed=0.1, timeout=10.0):
        if distance_m == 0.0:
            return "Distance must be non-zero."

        speed = abs(speed) * (1 if distance_m > 0 else -1)

        if not self._wait_for_odom():
            return "No odometry available. Cannot move."

        x0, y0, _ = self._get_xy_yaw()

        twist = Twist()
        twist.linear.x = speed

        rate = rospy.Rate(20)
        start = rospy.Time.now()

        while not rospy.is_shutdown():
            x, y, _ = self._get_xy_yaw()
            dx = x - x0
            dy = y - y0
            traveled = math.hypot(dx, dy)

            if traveled >= abs(distance_m):
                self._cmd_pub.publish(Twist())
                return f"Moved {traveled:.2f} m."

            if (rospy.Time.now() - start).to_sec() > timeout:
                self._cmd_pub.publish(Twist())
                return f"Timeout after traveling {traveled:.2f} m."

            self._cmd_pub.publish(twist)
            rate.sleep()

        return "Aborted."

    def turn(self, angle_deg: float, turn_rate=0.6, timeout=8.0):
        if angle_deg == 0:
            return "Angle must be non-zero."

        if not self._wait_for_odom():
            return "No odometry available. Cannot turn."

        _, _, yaw0 = self._get_xy_yaw()

        target = yaw0 + math.radians(angle_deg)
        target = math.atan2(math.sin(target), math.cos(target))

        twist = Twist()
        twist.angular.z = turn_rate if angle_deg > 0 else -turn_rate

        rate = rospy.Rate(20)
        start = rospy.Time.now()

        while not rospy.is_shutdown():
            _, _, yaw = self._get_xy_yaw()

            err = math.atan2(math.sin(target - yaw), math.cos(target - yaw))
            if abs(err) < math.radians(3):  # within 3 degrees
                self._cmd_pub.publish(Twist())
                return f"Turned {angle_deg:.1f} deg."

            if (rospy.Time.now() - start).to_sec() > timeout:
                self._cmd_pub.publish(Twist())
                return f"Timeout while turning. Final error={math.degrees(err):.1f} deg."

            self._cmd_pub.publish(twist)
            rate.sleep()

        return "Aborted."


motion = MotionHelper()


@tool
def drive_forward(distance_m: float) -> str:
    return motion.drive(distance_m)


@tool
def drive_backward(distance_m: float) -> str:
    return motion.drive(-distance_m)


@tool
def turn(angle_deg: float) -> str:
    return motion.turn(angle_deg)


class SimpleMotionAgent(ROSA):
    def __init__(self):
        llm = ChatOllama(model="llama3.1", base_url="http://localhost:11434", temperature=0.0)

        super().__init__(
            ros_version=1,
            llm=llm,
            tools=[drive_forward, drive_backward, turn],
            prompts={"system": "You control a robot base. Use tools to move it."},
            verbose=True,
            streaming=False,
        )

    async def run(self):
        while True:
            query = input("> ")
            if query.lower() in ("exit", "quit"):
                break
            print(self.invoke(query))


def main():
    rospy.init_node("simple_motion_agent")
    agent = SimpleMotionAgent()

    loop = asyncio.get_event_loop()
    loop.run_until_complete(agent.run())


if __name__ == "__main__":
    main()

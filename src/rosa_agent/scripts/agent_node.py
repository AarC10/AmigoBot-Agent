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
import requests
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import importlib

# Attempt to import langchain tool decorator
try:
    _lc_agents = importlib.import_module('langchain.agents')
    tool = getattr(_lc_agents, 'tool', None)
    if not callable(tool):
        def tool(fn=None, *dargs, **dkwargs):
            if fn is None:
                return lambda f: f
            return fn
except Exception:
    def tool(fn=None, *dargs, **dkwargs):
        # allow using @tool without breaking
        if fn is None:
            return lambda f: f
        return fn

# Attempt to import ChatOllama and ROSA
try:
    _mod_llama = importlib.import_module('langchain_ollama')
    ChatOllama = getattr(_mod_llama, 'ChatOllama', None)
except Exception:
    ChatOllama = None

try:
    _mod_rosa = importlib.import_module('rosa')
    ROSA = getattr(_mod_rosa, 'ROSA', None)
except Exception:
    ROSA = None

# VLM service and pointcloud types
try:
    from vlm_clip.srv import QueryTarget, QueryTargetRequest
except Exception:
    QueryTarget = None
    QueryTargetRequest = None

try:
    from sensor_msgs.msg import PointCloud, PointCloud2
    from sensor_msgs import point_cloud2 as pc2
except Exception:
    PointCloud = None
    PointCloud2 = None
    pc2 = None


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


class SensorHelper:
    def __init__(self):
        self._sonar_lock = threading.Lock()
        self._sonar_points = None

        self.sonar_topic = rospy.get_param('~sonar_topic', '/rosaria/sonar_pointcloud2')
        self.legacy_sonar_topic = rospy.get_param('~legacy_sonar_topic', '/sonar')
        self.front_sector_deg = float(rospy.get_param('~front_sector_deg', 30.0))

        # Positive = left, Negative = right
        self.sonar_angle_mapping = {
            0: 90.0,
            1: 44.0,
            2: 12.0,
            3: -12.0,
            4: -44.0,
            5: -90.0,
            6: -144.0,
            7: 144.0,
        }

        if PointCloud2 is not None:
            try:
                self._pc2_sub = rospy.Subscriber(self.sonar_topic, PointCloud2, self._sonar_pc2_callback, queue_size=1)
            except Exception:
                self._pc2_sub = None
        else:
            self._pc2_sub = None

        if PointCloud is not None:
            try:
                self._pc_sub = rospy.Subscriber(self.legacy_sonar_topic, PointCloud, self._sonar_callback, queue_size=1)
            except Exception:
                self._pc_sub = None
        else:
            self._pc_sub = None

        # VLM service proxy
        self.vlm_service_name = rospy.get_param('~vlm_service_name', '/query_target')
        self.vlm_query = None
        if QueryTarget is not None and QueryTargetRequest is not None:
            try:
                rospy.loginfo("agent_node: waiting for VLM service '%s'", self.vlm_service_name)
                rospy.wait_for_service(self.vlm_service_name, timeout=5.0)
                self.vlm_query = rospy.ServiceProxy(self.vlm_service_name, QueryTarget)
            except Exception:
                rospy.logwarn("agent_node: VLM service not available (will attempt calls when needed)")

    def _sonar_callback(self, msg):
        with self._sonar_lock:
            self._sonar_points = list(msg.points) if msg and getattr(msg, 'points', None) is not None else None

    def _sonar_pc2_callback(self, msg):
        pts = None
        if pc2 is not None:
            try:
                pts_iter = pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)
                pts = [(float(x), float(y)) for (x,y,z) in pts_iter]
            except Exception:
                pts = None
        with self._sonar_lock:
            self._sonar_points = pts

    def get_front_obstacle_distance(self):
        with self._sonar_lock:
            pts = list(self._sonar_points) if self._sonar_points is not None else None

        if not pts:
            return float('nan')

        try:
            if self.sonar_angle_mapping and len(pts) == len(self.sonar_angle_mapping):
                min_dist = None
                for idx, (x, y) in enumerate(pts):
                    ang_deg = self.sonar_angle_mapping.get(idx, None)
                    if ang_deg is None:
                        continue
                    if isinstance(ang_deg, (int, float)):
                        ang_deg_f = float(ang_deg)
                    elif isinstance(ang_deg, str):
                        try:
                            ang_deg_f = float(ang_deg)
                        except Exception:
                            continue
                    else:
                        continue
                    if abs(ang_deg_f) <= self.front_sector_deg:
                        dist = math.hypot(x, y)
                        if dist <= 0.0:
                            continue
                        if (min_dist is None) or (dist < min_dist):
                            min_dist = dist
                return min_dist if min_dist is not None else float('nan')
        except Exception:
            # fallback to positional method below
            pass

        # Fallback: compute angle per point using atan2(y, x)
        front_rad = math.radians(self.front_sector_deg)
        min_dist = None
        for p in pts:
            try:
                x = p.x
                y = p.y
            except Exception:
                try:
                    x, y = p[0], p[1]
                except Exception:
                    continue
            dist = math.hypot(x, y)
            if dist <= 0.0:
                continue
            angle = math.atan2(y, x)
            if abs(angle) <= front_rad:
                if (min_dist is None) or (dist < min_dist):
                    min_dist = dist
        return min_dist if min_dist is not None else float('nan')

    def query_vlm(self, query_text: str):
        if self.vlm_query is None or QueryTargetRequest is None:
            raise RuntimeError('VLM service proxy not available or service type missing.')
        req = QueryTargetRequest()
        req.query = query_text
        resp = self.vlm_query(req)
        return {'found': bool(resp.found), 'bearing_deg': float(resp.bearing_deg), 'confidence': float(resp.confidence)}


sensors = SensorHelper()


@tool
def drive_forward(distance_m: float) -> str:
    """
    Move the robot forward by the specified distance in meters.
    :param distance_m: Distance to move forward in meters.
    :return: Status message.
    """
    return motion.drive(distance_m)


@tool
def drive_backward(distance_m: float) -> str:
    """
    Move the robot backward by the specified distance in meters.
    :param distance_m: Distance to move backward in meters.
    :return: Status message.
    """
    return motion.drive(-distance_m)


@tool
def turn(angle_deg: float) -> str:
    """
    Turn the robot by the specified angle in degrees.
    :param angle_deg: Angle to turn in degrees (positive for left, negative for right).
    :return: Status message.
    """
    return motion.turn(angle_deg)


# new tools for sensors/VLM
@tool
def front_distance() -> str:
    """Return front obstacle distance in meters as a string (or 'nan' if unknown)."""
    try:
        d = sensors.get_front_obstacle_distance()
        if math.isnan(d):
            return 'nan'
        return f"{d:.3f}"
    except Exception as e:
        return f"error: {e}"


@tool
def vlm_query(query: str) -> str:
    """Query the VLM for a bearing to the given query text."""
    try:
        res = sensors.query_vlm(query)
        return f"found={res['found']}, bearing_deg={res['bearing_deg']:.2f}, confidence={res['confidence']:.3f}"
    except Exception as e:
        return f"error: {e}"


class SimpleMotionAgent(ROSA if ROSA is not None else object):
    def __init__(self):
        # Read LLM config from ROS params with safe defaults
        model = rospy.get_param("~ollama_model", "llama3.1:8b")
        base_url = rospy.get_param("~ollama_base_url", "http://localhost:11434")
        temperature = float(rospy.get_param("~temperature", 0.0))

        llm = None

        def _init_llm_runtime(model, base_url, temperature):
            # Attempt runtime import/instantiation of ChatOllama (best-effort).
            try:
                resp = requests.get(f"{base_url.rstrip('/')}/api/tags", timeout=3)
                if resp.status_code != 200:
                    rospy.logwarn(f"Ollama at {base_url} returned {resp.status_code}. Ensure 'ollama serve' is reachable.")
            except Exception:
                rospy.logwarn("Ollama connectivity check failed.")

            try:
                mod = importlib.import_module('langchain_ollama')
                Cls = getattr(mod, 'ChatOllama', None)
                if Cls is None:
                    return None
                return Cls(model=model, base_url=base_url, temperature=temperature)
            except Exception as e:
                raise RuntimeError(f"Failed to initialize ChatOllama at runtime: {e}") from e

        try:
            llm = _init_llm_runtime(model, base_url, temperature)
            if llm is None:
                rospy.logwarn("ChatOllama not available; agent llm functionality disabled.")
        except Exception as e:
            raise

        if ROSA is None:
            rospy.logwarn("ROSA base class not available; creating a minimal agent object with tool invocation support.")

        tools = [drive_forward, drive_backward, turn, vlm_query, front_distance]

        # If ROSA is available, call its constructor or just set minimal attrs
        if ROSA is not None:
            super().__init__(
                ros_version=1,
                llm=llm,
                tools=tools,
                verbose=True,
                streaming=False,
            )
        else:
            self.tools = {t.__name__: t for t in tools}
            self.llm = llm
            self.verbose = True

    def invoke(self, query: str):
        if ROSA is not None:
            return super().invoke(query)

        # Minimal parsing: split by whitespace
        parts = query.strip().split()
        if not parts:
            return "empty query"

        cmd = parts[0]
        args = parts[1:]

        if cmd in self.tools:
            func = self.tools[cmd]
            # Try to coerce single numeric arg
            try:
                if len(args) == 0:
                    return func()
                if len(args) == 1:
                    try:
                        val = float(args[0])
                        return func(val)
                    except Exception:
                        return func(" ".join(args))
                return func(" ".join(args))
            except TypeError:
                return f"tool invocation failed for {cmd}"
        else:
            return f"unknown command '{cmd}'"


async def run_loop(agent: SimpleMotionAgent):
    while True:
        query = input("> ")
        if query.lower() in ("exit", "quit"):
            break
        try:
            result = agent.invoke(query)
            print(result)
        except Exception as e:
            print(f"Agent error: {e}")


def main():
    rospy.init_node("simple_motion_agent")
    agent = SimpleMotionAgent()

    loop = asyncio.get_event_loop()
    loop.run_until_complete(run_loop(agent))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys

# Ensure ROS/system site-packages are visible before any ROS imports
sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")
sys.path.insert(0, "/usr/lib/python3/dist-packages")

try:
    import rospkg  # noqa: F401
except Exception as e:
    raise ImportError(
        "rospkg not found. Ensure ROS Python site-packages are on sys.path and apt package python3-rospkg is installed.") from e

import math
import asyncio
import rospy
import threading
import requests
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import importlib

from langchain_core.tools import tool, Tool
from rosa import ROSA, RobotSystemPrompts

# Attempt to import ChatOllama and ROSA
try:
    _mod_llama = importlib.import_module('langchain_ollama')
    ChatOllama = getattr(_mod_llama, 'ChatOllama', None)
except Exception:
    ChatOllama = None

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
                pts_iter = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
                pts = [(float(x), float(y)) for (x, y, z) in pts_iter]
            except Exception:
                pts = None
        with self._sonar_lock:
            self._sonar_points = pts

    def get_front_obstacle_distance(self):
        """
        Get the distance to the nearest obstacle in front of the robot using sonar data.
        :return: Distance in meters, or float('nan') if unknown.
        """
        with self._sonar_lock:
            pts = list(self._sonar_points) if self._sonar_points is not None else None

        if not pts:
            try:
                use_sim = bool(rospy.get_param('/use_sim_time', False))
            except Exception:
                use_sim = False
            if use_sim:
                try:
                    return float(rospy.get_param('~sim_default_distance', 1.0))
                except Exception:
                    return 1.0
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
            rospy.logwarn("agent_node: Front obstacle distance not available")

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

    def get_index_for_angle(self, angle_deg: float):
        """Return the sonar channel index whose configured angle is closest to angle_deg.
        Returns None if mapping unavailable.
        """
        if not self.sonar_angle_mapping:
            return None
        best_idx = None
        best_diff = None
        for idx, ang in self.sonar_angle_mapping.items():
            try:
                diff = abs(float(ang) - float(angle_deg))
            except Exception:
                continue
            if (best_diff is None) or (diff < best_diff):
                best_diff = diff
                best_idx = idx
        return best_idx

    def get_distance_for_index(self, index: int):
        """Return measured distance (meters) for the given sonar index from the last PointCloud sample.
        Returns float('nan') if unavailable.
        """
        with self._sonar_lock:
            pts = list(self._sonar_points) if self._sonar_points is not None else None

        if not pts:
            # return default distance in sim mode
            try:
                use_sim = bool(rospy.get_param('/use_sim_time', False))
            except Exception:
                use_sim = False
            if use_sim:
                try:
                    return float(rospy.get_param('~sim_default_distance', 1.0))
                except Exception:
                    return 1.0
            return float('nan')

        if index is None:
            return float('nan')
        try:
            point = pts[index]
        except Exception:
            return float('nan')

        try:
            x = point.x
            y = point.y
        except Exception:
            try:
                x, y = point[0], point[1]
            except Exception:
                return float('nan')
        dist = math.hypot(x, y)
        return dist if dist > 0.0 else float('nan')


def make_tools(motion: MotionHelper, sensors: SensorHelper):
    @tool
    def drive_forward(distance_m: float) -> str:
        """
        Move the robot forward by the specified distance in meters.
        :param distance_m: Distance to move forward in meters.
        :return: Status message.
        """
        print("TOOL CALL: drive_forward", distance_m)
        rospy.loginfo("agent_node: drive_forward {distance_m=%.2f}", distance_m)
        return motion.drive(distance_m)

    @tool
    def drive_backward(distance_m: float) -> str:
        """
        Move the robot backward by the specified distance in meters.
        :param distance_m: Distance to move backward in meters.
        :return: Status message.
        """
        print("TOOL CALL: drive_backward", distance_m)
        rospy.loginfo("agent_node: drive_backward {distance_m=%.2f}", distance_m)
        return motion.drive(-distance_m)

    @tool
    def turn(angle_deg: float) -> str:
        """
        Turn the robot by the specified angle in degrees.
        :param angle_deg: Angle to turn in degrees (positive for left, negative for right).
        :return: Status message.
        """
        print("TOOL CALL: turn", angle_deg)
        rospy.loginfo("agent_node: turn {angle_deg=%.2f}", angle_deg)
        return motion.turn(angle_deg)

    @tool
    def front_distance() -> str:
        """
        Get the distance to the nearest obstacle in front of the robot.
        :return: Distance in meters as a string, or 'nan' if unknown.
        """
        print("TOOL CALL: front_distance")
        rospy.loginfo("agent_node: front_distance")
        try:
            d = sensors.get_front_obstacle_distance()
            if math.isnan(d):
                return 'nan'
            return f"{d:.3f}"
        except Exception as e:
            return f"error: {e}"

    @tool
    def vlm_query(query: str) -> str:
        """
        Query the Vision-Language Model (VLM) for a specified object to get its bearing and confidence.
        :param query: The object to look for with the camera
        :return: A string with found status, bearing in degrees, and confidence.
        """
        print("TOOL CALL: vlm_query", query)
        rospy.loginfo("agent_node: vlm_query {query='%s'}", query)
        try:
            res = sensors.query_vlm(query)
            return f"found={res['found']}, bearing_deg={res['bearing_deg']:.2f}, confidence={res['confidence']:.3f}"
        except Exception as e:
            return f"error: {e}"

    @tool
    def sonar_distance(index: int) -> str:
        """Return the distance for a particular sonar channel index (meters) as a string."""
        print("TOOL CALL: sonar_distance", index)
        rospy.loginfo("agent_node: sonar_distance {index=%d}", index)
        try:
            d = sensors.get_distance_for_index(int(index))
            if math.isnan(d):
                return 'nan'
            return f"{d:.3f}"
        except Exception as e:
            return f"error: {e}"

    @tool
    def sonar_distance_angle(angle_deg: float) -> str:
        """Find the sonar index closest to angle_deg and return its distance as a string."""
        print("TOOL CALL: sonar_distance_angle", angle_deg)
        try:
            idx = sensors.get_index_for_angle(float(angle_deg))
            if idx is None:
                return "error: no sonar mapping available"
            return sonar_distance(idx)
        except Exception as e:
            return f"error: {e}"

    @tool
    def approach_target_safe(query: str, safety_margin: float = 0.2, confidence_threshold: float = 0.4,
                             samples: int = 3, sample_delay: float = 0.1) -> str:
        """
        Robust helper to approach a VLM-detected object safely

        Steps:
          1. Query VLM for the `query` string.
          2. Verify `found` and `confidence` >= confidence_threshold.
          3. Map bearing to the nearest sonar index.
          4. Sample the sonar `samples` times (with `sample_delay`) and use the median reading.
          5. Turn the robot to the reported bearing.
          6. After turning, verify front clearance via `front_distance()`; abort if too close.
          7. Drive forward to (measured_distance - safety_margin).

        :param query: The object to look for with the camera
        :param safety_margin: The minimum distance to maintain from the target (meters)
        :param confidence_threshold: The minimum VLM confidence to accept the target
        :param samples: The number of sonar samples to take for distance estimation
        :param sample_delay: Delay (seconds) between sonar samples
        :return: Returns a concise status string. Uses simulation fallback distance when /use_sim_time=true.

        """
        import statistics

        print("TOOL CALL: approach_target_safe", query, safety_margin, confidence_threshold, samples, sample_delay)
        rospy.loginfo("agent_node: approach_target_safe {query='%s', safety_margin=%.2f, confidence_threshold=%.3f, samples=%d, sample_delay=%.2f}",
                      query, safety_margin, confidence_threshold, samples, sample_delay)

        # Query VLM
        try:
            res = sensors.query_vlm(query)
        except Exception as e:
            return f"vlm error: {e}"

        if not res.get('found', False):
            return f"target '{query}' not found by VLM"

        confidence = float(res.get('confidence', 0.0))
        bearing = float(res.get('bearing_deg', 0.0))

        if confidence < float(confidence_threshold):
            return f"vlm confidence too low ({confidence:.3f} < {confidence_threshold:.3f})"

        # Map bearing -> sonar index
        idx = sensors.get_index_for_angle(bearing)
        if idx is None:
            return f"no sonar mapping available to resolve bearing {bearing:.1f} deg"

        # Sample sonar readings
        readings = []
        for i in range(max(1, int(samples))):
            d = sensors.get_distance_for_index(idx)
            readings.append(d)
            # short delay to allow new data in real setups
            try:
                rospy.sleep(float(sample_delay))
            except Exception:
                # fallback to time.sleep if rospy.sleep unavailable
                import time
                time.sleep(float(sample_delay))

        # Filter NaNs
        valid = [r for r in readings if not math.isnan(r)]
        if not valid:
            # sim fallback handled in get_distance_for_index
            return f"no valid sonar readings for index {idx} (samples={readings})"

        # Use conservative est. median to be robust, also compute min for safety
        med = float(statistics.median(valid))
        mn = float(min(valid))

        # Turn to bearing
        turn_result = motion.turn(bearing)
        if isinstance(turn_result, str) and 'no odometry' in turn_result.lower():
            return f"turn failed: {turn_result}"

        # After turning, verify front clearance
        try:
            front = sensors.get_front_obstacle_distance()
        except Exception:
            front = float('nan')

        if not math.isnan(front) and front < float(safety_margin):
            return f"aborting after turn: front clearance too small ({front:.3f} m)"

        # Compute target drive distance (use conservative min reading)
        target_dist = max(0.0, mn - float(safety_margin))
        if target_dist <= 0.0:
            return f"already within safety margin (nearest reading {mn:.3f} m)"

        drive_result = motion.drive(target_dist)

        return (f"vlm:found={res['found']},bearing={bearing:.1f},conf={confidence:.3f}; "
                f"sonar_idx={idx},samples={len(readings)},median={med:.3f},min={mn:.3f}; "
                f"turn={turn_result}; drive={drive_result}")

    return [
        drive_forward,
        drive_backward,
        turn,
        front_distance,
        vlm_query,
        sonar_distance,
        sonar_distance_angle,
        approach_target_safe,
    ]


class AmigobotAgent(ROSA if ROSA is not None else object):
    def __init__(self, tools):
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
                    rospy.logwarn(
                        f"Ollama at {base_url} returned {resp.status_code}. Ensure 'ollama serve' is reachable.")
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
            rospy.logwarn(
                "ROSA base class not available; creating a minimal agent object with tool invocation support.")

        # If ROSA is available, call its constructor or just set minimal attrs
        if ROSA is not None:
            super().__init__(
                ros_version=1,
                llm=llm,
                tools=tools,
                verbose=True,
                streaming=False,
                prompts=get_prompts(),
            )
        else:
            self.tools = {t.__name__: t for t in tools}
            self.llm = llm
            self.verbose = True

    def invoke(self, query: str):
        if ROSA is not None:
            rospy.loginfo(f"Invoking ROSA agent for query: {query}")
            return super().invoke(query)

        rospy.loginfo(f"ROSA not available; minimal invoke for query: {query}")
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


async def run_loop(agent: AmigobotAgent):
    while True:
        query = input("> ")
        if query.lower() in ("exit", "quit"):
            break
        try:
            result = agent.invoke(query)
            print(result)
        except Exception as e:
            print(f"Agent error: {e}")


def get_prompts():
    return RobotSystemPrompts(
        embodiment_and_persona="You are an Amigobot mobile robot controlled through ROS.",
        about_your_operators="Operators will give high-level movement commands in plain English.",
        critical_instructions=(
            "TOOL USE REQUIRED:\n"
            "When the user requests movement/turning/sensing, you MUST call the appropriate tool.\n"
            "Do not describe actions without calling tools.\n"
            "Execute commands sequentially and wait for each tool to complete.\n"
            "After tool execution, respond with the tool's returned status string.\n"
        ),
        constraints_and_guardrails=(
            "SAFETY:\n"
            "If front_distance() is not 'nan' and is < 0.25m, do not drive forward.\n"
            "Prefer small motions for ambiguous requests.\n"
        ),
        about_your_environment="ROS1 system with /cmd_vel and /odom available when the robot stack is running.",
        about_your_capabilities=(
            "Available tools:\n"
            "- drive_forward(distance_m)\n"
            "- drive_backward(distance_m)\n"
            "- turn(angle_deg)\n"
            "- front_distance()\n"
            "- sonar_distance(index)\n"
            "- sonar_distance_angle(angle_deg)\n"
            "- vlm_query(query)\n"
            "- approach_target_safe(query, ...)\n"
        ),
        nuance_and_assumptions="Distances are meters, angles are degrees. Positive turn is left.",
        mission_and_objectives="Correctly execute robot motion and sensing requests using tools.",
    )

def main():
    rospy.init_node("amigo_agent")
    motion = MotionHelper()
    sensors = SensorHelper()
    tools = make_tools(motion, sensors)
    agent = AmigobotAgent(tools)

    rospy.loginfo(f"Agent started.")
    print("MANUAL TOOL INVOKE:", tools[0].invoke({"distance_m": 0.1}))

    loop = asyncio.get_event_loop()
    loop.run_until_complete(run_loop(agent))


if __name__ == "__main__":
    main()

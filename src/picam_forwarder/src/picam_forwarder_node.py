#!/usr/bin/env python3
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def main():
    rospy.init_node("picam_forwarder", anonymous=False)

    stream_url_param = rospy.get_param("~stream_url", "0")
    topic_name = rospy.get_param("~image_topic", "/camera/image_raw")
    frame_id = rospy.get_param("~frame_id", "camera_link")
    target_fps = rospy.get_param("~fps", 10.0)

    try:  # local camera index
        cam_index = int(stream_url_param)
        rospy.loginfo("picam_forwarder: opening local camera index %d", cam_index)
        cap = cv2.VideoCapture(cam_index)
    except ValueError:  # URL
        rospy.loginfo("picam_forwarder: opening stream URL %s", stream_url_param)
        cap = cv2.VideoCapture(stream_url_param)

    if not cap.isOpened():
        rospy.logerr("picam_forwarder: failed to open %s", stream_url_param)
        return

    bridge = CvBridge()
    pub = rospy.Publisher(topic_name, Image, queue_size=1)
    rate = rospy.Rate(target_fps)

    ok = True
    while not rospy.is_shutdown() and ok:
        ok, frame = cap.read()
        if not ok or frame is None:
            rospy.logwarn_throttle(5.0, "picam_forwarder: failed to read frame")
            rate.sleep()
            continue

        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        pub.publish(msg)

        rate.sleep()

    cap.release()
    rospy.loginfo("picam_forwarder: shutting down.")


if __name__ == "__main__":
    main()

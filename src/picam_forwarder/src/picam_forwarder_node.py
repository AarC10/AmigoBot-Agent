#!/usr/bin/env python3
import cv2 as cv
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def main():
    rospy.init_node("picam_forwarder", anonymous=False)

    stream_url = rospy.get_param("~stream_url", "")
    topic_name = rospy.get_param("~image_topic", "/camera/image_raw")
    frame_id   = rospy.get_param("~frame_id", "camera_link")
    target_fps = rospy.get_param("~fps", 10.0)

    if not stream_url:
        rospy.logerr("picam_forwarder: '~stream_url' parameter is empty.")
        return

    rospy.loginfo("picam_forwarder: opening stream %s", stream_url)
    videoCapture = cv.VideoCapture(stream_url)

    if not videoCapture.isOpened():
        rospy.logerr("picam_forwarder: failed to open stream: %s", stream_url)
        return

    bridge = CvBridge()
    pub = rospy.Publisher(topic_name, Image, queue_size=1)

    rate = rospy.Rate(target_fps)
    ok = True

    while not rospy.is_shutdown() and ok:
        ok, frame = videoCapture.read()
        if not ok or frame is None:
            rospy.logwarn_throttle(5.0, "picam_forwarder: failed to read frame, retrying...")
            # Attempt reopen
            videoCapture.release()
            videoCapture = cv.VideoCapture(stream_url)
            ok, frame = videoCapture.read()
            if not ok or frame is None:
                rate.sleep()
                continue

        # BGR to ROS Image MSG
        try:
            msg = bridge.cv_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frame_id
            pub.publish(msg)
        except Exception as e:
            rospy.logerr_throttle(5.0, "picam_forwarder: cv_bridge conversion failed: %s", str(e))

        rate.sleep()

    videoCapture.release()
    rospy.loginfo("picam_forwarder: shutting down.")


if __name__ == "__main__":
    main()

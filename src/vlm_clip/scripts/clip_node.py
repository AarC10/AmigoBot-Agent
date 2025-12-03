import rospy

class ClipNode(object):
    def __init__(self):
        # params
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.service_name = rospy.get_param("~service_name", "query_target")
        self.camera_fov_deg = float(rospy.get_param("~camera_fov_deg", 60.0))
        self.n_bins = int(rospy.get_param("~n_bins", 7))
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.5))
        self.max_image_age = float(rospy.get_param("~max_image_age", 1.0))  # secs

        # devices


        # load model

        # image buffer
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_stamp = None
        self.lock = threading.Lock()

        # subscriber

    def image_callback(self, msg):
        pass

    def handle_query(self, req):
        pass

    def _score_bins(self, image, text_features):
        pass

def main():
    rospy.init_node("vlm_clip_node")
    node = ClipNode()
    rospy.spin()


if __name__ == "__main__":
    main()
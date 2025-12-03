import threading

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch as nn
import clip
from vlm_clip.srv import QueryTarget, QueryTargetResponse

class ClipNode(object):
    def __init__(self):
        # params
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.service_name = rospy.get_param("~service_name", "query_target")
        self.camera_fov_deg = float(rospy.get_param("~camera_fov_deg", 60.0))
        self.n_bins = int(rospy.get_param("~n_bins", 7))
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.5))
        self.max_image_age = float(rospy.get_param("~max_image_age", 1.0))  # secs

        # cpu or gpu for cuda
        torch_device = rospy.get_param("~device", "cpu")
        if torch_device == "cuda" and not nn.cuda.is_available():
            rospy.logwarn("CUDA not available, using CPU instead")
            self.device = "cpu"
        self.device = nn.device(torch_device)

        # load model
        model_name = rospy.get_param("~model_name", "ViT-B/32")
        rospy.loginfo(f"Loading CLIP model '{model_name}' on device '{self.device}'")
        self.model, self.preprocess = clip.load(model_name, device=self.device)
        self.model.eval()

        # image buffer
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_stamp = None
        self.lock = threading.Lock()

        # subscriber
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1
        )

        # service
        self.service = rospy.Service(
            self.service_name,
            QueryTarget,
            self.handle_query
        )

        rospy.loginfo(
            f"vlm_clip: node initialized and listening on {self.image_topic} and writing to {self.service_name}")

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

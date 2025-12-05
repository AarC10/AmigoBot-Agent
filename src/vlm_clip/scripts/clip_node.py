import threading

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import clip
from vlm_clip.srv import QueryTarget, QueryTargetResponse
from PIL import Image as PILImage


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
        if torch_device == "cuda" and not torch.cuda.is_available():
            rospy.logwarn("CUDA not available, using CPU instead")
            torch_device = "cpu"
        self.device = torch.device(torch_device)

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
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return

        with self.lock:
            self.latest_image = PILImage.fromarray(cv_image[:, :, ::-1])
            self.latest_stamp = msg.header.stamp

    def handle_query(self, req):
        query = req.query.strip()
        if not query:
            rospy.logerr("vlm_clip: invalid empty query")
            return QueryTargetResponse(
                found=False,
                confidence=0.0,
                bearing_deg=0.0
            )

        with self.lock:
            image = self.latest_image
            stamp = self.latest_stamp

        if image is None:
            rospy.logerr("vlm_clip: no image available yet")
            return QueryTargetResponse(
                found=False,
                confidence=0.0,
                bearing_deg=0.0
            )

        if (rospy.Time.now() - stamp).to_sec() > self.max_image_age:
            rospy.logerr("vlm_clip: latest image is too old")
            return QueryTargetResponse(
                found=False,
                confidence=0.0,
                bearing_deg=0.0
            )

        with torch.no_grad():
            query_prompt = f"a photo of {query}"
            texts = [query_prompt, "background"]
            text_tokens = clip.tokenize(texts).to(self.device)
            text_features = self.model.encode_text(text_tokens)
            text_features = text_features / text_features.norm(dim=-1, keepdim=True)

        scores, bin_centers = self._score_bins(image, text_features)

        if scores is None or len(scores) == 0:
            rospy.logerr("vlm_clip: scoring failed")
            return QueryTargetResponse(
                found=False,
                confidence=0.0,
                bearing_deg=0.0
            )

        scores_tensor = torch.tensor(scores, device=self.device)
        best_index = int(torch.argmax(scores_tensor).item())
        best_probability = float(scores[best_index])

        if best_probability < self.min_confidence:
            rospy.loginfo(
                "vlm_clip: target '%s' not found (max P=%.3f)",
                query, best_probability
            )
            return QueryTargetResponse(
                found=False,
                confidence=best_probability,
                bearing_deg=0.0
            )

        center_norm = bin_centers[best_index]  # 0..1
        bearing_deg = (center_norm - 0.5) * self.camera_fov_deg
        rospy.loginfo(
            "vlm_clip: target '%s' found at bearing %.2f deg with P=%.3f (bin=%d)",
            query, bearing_deg, best_probability, best_index
        )

        return QueryTargetResponse(
            found=True,
            confidence=best_probability,
            bearing_deg=bearing_deg
        )

    def _score_bins(self, image, text_features):
        w, h = image.size

        if w <= 0 or h <= 0 or self.n_bins <= 0:
            rospy.logerr("vlm_clip: invalid image dimensions (w=%d, h=%d, n_bins=%d)",
                         w, h, self.n_bins)
            return None, None

        bin_width = float(w) / float(self.n_bins)
        scores = []
        centers = []

        with torch.no_grad():
            for i in range(self.n_bins):
                left = int(i * bin_width)
                right = int((i + 1) * bin_width) if i < self.n_bins - 1 else w
                left = max(0, min(left, w - 1))
                right = max(left + 1, min(right, w))

                crop = image.crop((left, 0, right, h))
                input_image = self.preprocess(crop).unsqueeze(0).to(self.device)

                image_features = self.model.encode_image(input_image)  # (1, D)
                image_features = image_features / image_features.norm(dim=-1, keepdim=True)

                logits = 100.0 * image_features @ text_features.T
                probs = torch.nn.functional.softmax(logits, dim=-1)

                p_query = float(probs[0, 0].item())
                scores.append(p_query)

                center_x = 0.5 * (left + right)
                center_norm = center_x / float(w)
                centers.append(center_norm)

        rospy.logdebug("vlm_clip: bin P(query) = %s", ["%.3f" % s for s in scores])

        return scores, centers

def main():
    rospy.init_node("vlm_clip_node")
    node = ClipNode()
    rospy.spin()


if __name__ == "__main__":
    main()

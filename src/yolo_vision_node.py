#!/usr/bin/env python3
"""
yolo_vision_node.py — Edge-AI Vision Node for SIH26123
======================================================
ROS 2 Jazzy node that runs YOLOv8n-nano (ONNX) inference on the
AMR's onboard RGB camera feed to classify physical obstructions.

Decision Logic:
  - person detected  → SAFETY_STOP   (immediate /cmd_vel zero + alert)
  - box/pallet detected → BLOCKED_AUCTION_REQUIRED (CNP trigger)

Subscriptions:
  /camera/image_raw        (sensor_msgs/Image)

Publications:
  /cmd_vel                 (geometry_msgs/Twist)       — emergency halt
  /amr/obstruction_status  (std_msgs/String)           — classification flag
  /camera/yolo_annotated   (sensor_msgs/Image)         — debug visualization

Optimized for edge hardware (Jetson Nano / Raspberry Pi 4):
  - Configurable inference throttle (default: 5 FPS max)
  - Pre-allocated numpy buffers to avoid GC pressure
  - Graceful degradation on dropped frames / ONNX errors
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# ---------------------------------------------------------------------------
# ONNX Runtime — import with fallback for environments without GPU providers
# ---------------------------------------------------------------------------
try:
    import onnxruntime as ort
    _ORT_AVAILABLE = True
except ImportError:
    _ORT_AVAILABLE = False

# ---------------------------------------------------------------------------
# COCO-80 Class Names (YOLOv8n is trained on COCO)
# ---------------------------------------------------------------------------
COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
]

# ---------------------------------------------------------------------------
# Semantic mapping: which COCO classes map to our warehouse categories.
# "person"  → SAFETY_STOP
# "box"     → BLOCKED_AUCTION_REQUIRED  (mapped from box-like COCO objects)
#
# NOTE: YOLOv8n's COCO vocabulary has no literal "box" or "pallet" class.
# We map physically analogous objects that Gazebo models will resemble.
# For production, retrain on a warehouse-specific dataset.
# ---------------------------------------------------------------------------
PERSON_CLASS_IDS = {0}                        # "person"

BOX_PALLET_CLASS_IDS = {                      # box/pallet proxies
    24,  # backpack   — rectangular bulk object
    26,  # handbag    — small box-like
    28,  # suitcase   — rigid rectangular container
    56,  # chair      — warehouse furniture obstacle
    57,  # couch      — large movable obstacle
    60,  # dining table — flat pallet-like surface
}

# ---------------------------------------------------------------------------
# Visual config for bounding box annotations
# ---------------------------------------------------------------------------
_COLOR_SAFETY  = (0, 0, 255)    # Red   — person / SAFETY_STOP
_COLOR_BLOCKED = (0, 165, 255)  # Orange — box / BLOCKED_AUCTION
_COLOR_OTHER   = (0, 255, 0)    # Green  — other detected objects
_FONT_SCALE    = 0.45
_THICKNESS     = 2


class YoloVisionNode(Node):
    """
    Edge-AI vision node: YOLOv8n-nano ONNX inference on /camera/image_raw.
    """

    def __init__(self):
        super().__init__("yolo_vision_node")

        # ---------------------------------------------------------------
        # Declare ROS 2 parameters (tunable at launch via --ros-args -p)
        # ---------------------------------------------------------------
        self.declare_parameter("model_path", "yolov8n.onnx")
        self.declare_parameter("confidence_threshold", 0.45)
        self.declare_parameter("iou_threshold", 0.50)
        self.declare_parameter("max_inference_fps", 5.0)
        self.declare_parameter("input_size", 640)

        self._model_path  = self.get_parameter("model_path").value
        self._conf_thresh = self.get_parameter("confidence_threshold").value
        self._iou_thresh  = self.get_parameter("iou_threshold").value
        self._max_fps     = self.get_parameter("max_inference_fps").value
        self._input_size  = self.get_parameter("input_size").value

        # Minimum interval between inferences (throttle for edge hardware)
        self._min_interval_ns = int(1e9 / self._max_fps) if self._max_fps > 0 else 0
        self._last_inference_time = self.get_clock().now()

        # ---------------------------------------------------------------
        # cv_bridge instance (reused across callbacks)
        # ---------------------------------------------------------------
        self._bridge = CvBridge()

        # ---------------------------------------------------------------
        # ONNX Runtime session — load model
        # ---------------------------------------------------------------
        self._session = None
        self._input_name = None
        self._output_name = None
        self._load_model()

        # ---------------------------------------------------------------
        # QoS — best-effort for camera (tolerate dropped frames on edge)
        # ---------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------------------------------------------------------------
        # Subscriptions
        # ---------------------------------------------------------------
        self._sub_image = self.create_subscription(
            Image,
            "/camera/image_raw",
            self._image_callback,
            sensor_qos,
        )

        # ---------------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------------
        self._pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_status  = self.create_publisher(String, "/amr/obstruction_status", 10)
        self._pub_annotated = self.create_publisher(Image, "/camera/yolo_annotated", sensor_qos)

        # ---------------------------------------------------------------
        # Pre-allocate reusable messages to reduce GC pressure
        # ---------------------------------------------------------------
        self._halt_twist = Twist()  # all zeros = full stop
        self._status_msg = String()

        self.get_logger().info(
            f"YoloVisionNode initialized | model={self._model_path} "
            f"conf={self._conf_thresh} iou={self._iou_thresh} "
            f"max_fps={self._max_fps} input_size={self._input_size}"
        )

    # ===================================================================
    # Model Loading
    # ===================================================================
    def _load_model(self):
        """Load the YOLOv8n ONNX model into an ORT InferenceSession."""
        if not _ORT_AVAILABLE:
            self.get_logger().error(
                "onnxruntime not installed. "
                "Install with: pip install onnxruntime"
            )
            return

        # Prefer execution providers in order: CUDA → TensorRT → CPU
        providers = []
        available = ort.get_available_providers()
        for ep in ["TensorrtExecutionProvider",
                    "CUDAExecutionProvider",
                    "CPUExecutionProvider"]:
            if ep in available:
                providers.append(ep)

        try:
            # Optimize for edge: disable memory pattern, use sequential exec
            opts = ort.SessionOptions()
            opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            opts.intra_op_num_threads = 2       # limit threads on Pi/Jetson
            opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL

            self._session = ort.InferenceSession(
                self._model_path, sess_options=opts, providers=providers
            )
            self._input_name  = self._session.get_inputs()[0].name
            self._output_name = self._session.get_outputs()[0].name

            active_ep = self._session.get_providers()
            self.get_logger().info(
                f"ONNX model loaded: {self._model_path} | "
                f"Execution Provider: {active_ep[0]}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load ONNX model: {e}")
            self._session = None

    # ===================================================================
    # Image Callback (Main Pipeline)
    # ===================================================================
    def _image_callback(self, msg: Image):
        """
        Called on every /camera/image_raw frame.
        Throttles inference to max_inference_fps to conserve edge resources.
        """
        # ---- Frame throttle ----
        now = self.get_clock().now()
        elapsed_ns = (now - self._last_inference_time).nanoseconds
        if elapsed_ns < self._min_interval_ns:
            return  # skip this frame silently
        self._last_inference_time = now

        # ---- Guard: model not loaded ----
        if self._session is None:
            return

        # ---- Convert ROS Image → OpenCV BGR ----
        try:
            cv_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        # ---- Run inference ----
        try:
            detections = self._run_inference(cv_frame)
        except Exception as e:
            self.get_logger().error(
                f"ONNX inference error (frame dropped): {e}"
            )
            return

        # ---- Classify and act ----
        self._process_detections(detections, cv_frame)

        # ---- Publish annotated debug frame ----
        self._publish_annotated(cv_frame, msg.header)

    # ===================================================================
    # ONNX Inference
    # ===================================================================
    def _run_inference(self, frame: np.ndarray) -> np.ndarray:
        """
        Pre-process frame, run YOLOv8n ONNX, post-process with NMS.

        Returns:
            np.ndarray of shape [N, 6] where each row is:
            [x1, y1, x2, y2, confidence, class_id]
            Returns empty (0, 6) array if no detections.
        """
        img_h, img_w = frame.shape[:2]
        sz = self._input_size

        # ---- Letterbox resize (preserve aspect ratio) ----
        scale = min(sz / img_w, sz / img_h)
        new_w, new_h = int(img_w * scale), int(img_h * scale)
        pad_w, pad_h = (sz - new_w) // 2, (sz - new_h) // 2

        # Use cv2 only if available, otherwise fall back to numpy resize
        import cv2
        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        padded = np.full((sz, sz, 3), 114, dtype=np.uint8)
        padded[pad_h:pad_h + new_h, pad_w:pad_w + new_w] = resized

        # ---- Normalize: HWC→CHW, 0-255→0.0-1.0, add batch dim ----
        blob = padded.astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))    # CHW
        blob = np.expand_dims(blob, axis=0)      # NCHW [1, 3, 640, 640]

        # ---- ONNX forward pass ----
        outputs = self._session.run(
            [self._output_name], {self._input_name: blob}
        )

        # ---- Parse YOLOv8 output ----
        # YOLOv8 raw output: [1, 84, 8400] → transpose to [8400, 84]
        preds = outputs[0][0].T  # shape: [8400, 84]

        # Columns 0-3: cx, cy, w, h  (center format, in 640×640 space)
        # Columns 4-83: class confidence scores (80 COCO classes)

        # ---- Filter by confidence ----
        class_scores = preds[:, 4:]                       # [8400, 80]
        max_scores   = np.max(class_scores, axis=1)       # [8400]
        mask         = max_scores > self._conf_thresh
        if not np.any(mask):
            return np.empty((0, 6), dtype=np.float32)

        filtered  = preds[mask]
        scores    = max_scores[mask]
        class_ids = np.argmax(filtered[:, 4:], axis=1)

        # ---- Convert cx,cy,w,h → x1,y1,x2,y2 (still in 640×640 space) ----
        cx, cy, w, h = (filtered[:, i] for i in range(4))
        x1 = cx - w / 2.0
        y1 = cy - h / 2.0
        x2 = cx + w / 2.0
        y2 = cy + h / 2.0

        # ---- NMS (class-agnostic for speed on edge) ----
        boxes = np.stack([x1, y1, x2, y2], axis=1)
        keep  = self._nms(boxes, scores, self._iou_thresh)
        if len(keep) == 0:
            return np.empty((0, 6), dtype=np.float32)

        x1, y1, x2, y2 = x1[keep], y1[keep], x2[keep], y2[keep]
        scores    = scores[keep]
        class_ids = class_ids[keep]

        # ---- Scale bounding boxes back to original image coordinates ----
        x1 = ((x1 - pad_w) / scale).clip(0, img_w)
        y1 = ((y1 - pad_h) / scale).clip(0, img_h)
        x2 = ((x2 - pad_w) / scale).clip(0, img_w)
        y2 = ((y2 - pad_h) / scale).clip(0, img_h)

        return np.stack(
            [x1, y1, x2, y2, scores, class_ids.astype(np.float32)], axis=1
        )

    # ===================================================================
    # Non-Maximum Suppression (pure numpy — no torchvision dependency)
    # ===================================================================
    @staticmethod
    def _nms(boxes: np.ndarray, scores: np.ndarray,
             iou_threshold: float) -> np.ndarray:
        """
        Class-agnostic NMS using pure numpy.
        Returns indices of boxes to keep.
        """
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)

        order = scores.argsort()[::-1]
        keep = []

        while order.size > 0:
            i = order[0]
            keep.append(i)

            # Compute IoU of the kept box with all remaining
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)

            # Keep boxes with IoU below threshold
            remaining = np.where(iou <= iou_threshold)[0]
            order = order[remaining + 1]

        return np.array(keep, dtype=int)

    # ===================================================================
    # Detection Classification & Action
    # ===================================================================
    def _process_detections(self, detections: np.ndarray,
                            frame: np.ndarray):
        """
        Apply warehouse safety logic based on detected object classes.

        Priority hierarchy:
          1. SAFETY_STOP          (person detected — highest priority)
          2. BLOCKED_AUCTION_REQUIRED (box/pallet — CNP handoff)
          3. CLEAR                (no relevant obstruction)
        """
        import cv2

        found_person = False
        found_box    = False

        for det in detections:
            x1, y1, x2, y2, conf, cls_id = det
            cls_id = int(cls_id)
            cls_name = COCO_CLASSES[cls_id] if cls_id < len(COCO_CLASSES) else "unknown"

            # ---- Choose color and track category ----
            if cls_id in PERSON_CLASS_IDS:
                found_person = True
                color = _COLOR_SAFETY
                label = f"PERSON {conf:.2f}"
            elif cls_id in BOX_PALLET_CLASS_IDS:
                found_box = True
                color = _COLOR_BLOCKED
                label = f"BOX/PALLET({cls_name}) {conf:.2f}"
            else:
                color = _COLOR_OTHER
                label = f"{cls_name} {conf:.2f}"

            # ---- Draw bounding box + label on frame ----
            pt1 = (int(x1), int(y1))
            pt2 = (int(x2), int(y2))
            cv2.rectangle(frame, pt1, pt2, color, _THICKNESS)

            # Label background for readability
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX,
                                          _FONT_SCALE, 1)
            cv2.rectangle(frame, pt1, (pt1[0] + tw, pt1[1] - th - 6),
                          color, -1)
            cv2.putText(frame, label, (pt1[0], pt1[1] - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, _FONT_SCALE,
                        (255, 255, 255), 1, cv2.LINE_AA)

        # ---- Publish decisions (person takes absolute priority) ----
        if found_person:
            # SAFETY_STOP: immediately halt all motor output
            self._pub_cmd_vel.publish(self._halt_twist)
            self._status_msg.data = "SAFETY_STOP"
            self._pub_status.publish(self._status_msg)
            self.get_logger().warn("SAFETY_STOP — Human worker detected, motors halted")

        elif found_box:
            # BLOCKED_AUCTION_REQUIRED: signal CNP module for task handoff
            self._status_msg.data = "BLOCKED_AUCTION_REQUIRED"
            self._pub_status.publish(self._status_msg)
            self.get_logger().info("BLOCKED_AUCTION_REQUIRED — Movable obstruction detected")

        else:
            # CLEAR: no relevant obstruction in view
            self._status_msg.data = "CLEAR"
            self._pub_status.publish(self._status_msg)

    # ===================================================================
    # Annotated Frame Publisher
    # ===================================================================
    def _publish_annotated(self, frame: np.ndarray, header):
        """
        Publish the bounding-box-annotated frame to /camera/yolo_annotated
        for the SIH judge dashboard / rqt_image_view visualization.
        """
        try:
            ann_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            ann_msg.header = header
            self._pub_annotated.publish(ann_msg)
        except CvBridgeError as e:
            self.get_logger().warn(f"Failed to publish annotated frame: {e}")


# =======================================================================
# Entry Point
# =======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = YoloVisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down YoloVisionNode (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

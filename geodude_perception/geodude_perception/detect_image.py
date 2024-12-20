import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class DetectImage(Node):

    def __init__(self):
        super().__init__('detect_image')

        self.image_sub = self.create_subscription(
            Image,
            'image',
            self.image_callback,
            10)

        self.d2d_publisher = self.create_publisher(
            Detection2DArray,
            'detections_output',
            10)

        self.overlay_publisher = self.create_publisher(
            Image,
            'overlay',
            10)
        
        self.declare_parameter('classes', ['cup', 'bottle'])

        self.model = YOLO("yolov8x-world.pt")
        self.classes = self.get_parameter('classes').value
        self.model.set_classes(self.classes)

        self.bridge = CvBridge()

    def populate_detection2d_msg(self, results, header):
        d2d_array = Detection2DArray()
        d2d_array.header = header

        bboxes = results[0].boxes.xyxy.cpu().numpy()
        scores = results[0].boxes.conf.cpu().numpy()
        classes = results[0].boxes.cls.cpu().numpy()

        if len(bboxes) == 0:
            return

        highest_conf_idx = scores.argmax()

        d2d = Detection2D()
        d2d.bbox.center.position.x = (bboxes[highest_conf_idx][0] + bboxes[highest_conf_idx][2]) / 2
        d2d.bbox.center.position.y = (bboxes[highest_conf_idx][1] + bboxes[highest_conf_idx][3]) / 2
        d2d.bbox.size_x = float(bboxes[highest_conf_idx][2] - bboxes[highest_conf_idx][0])
        d2d.bbox.size_y = float(bboxes[highest_conf_idx][3] - bboxes[highest_conf_idx][1])

        obj_hyp = ObjectHypothesisWithPose()
        obj_hyp.hypothesis.class_id = str(classes[highest_conf_idx])
        obj_hyp.hypothesis.score = float(scores[highest_conf_idx])

        d2d.results.append(obj_hyp)

        d2d_array.detections.append(d2d)

        self.d2d_publisher.publish(d2d_array)

    def image_callback(self, msg):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(e)
            return

        results = self.model.track(cv_image, persist=True, verbose=False)
        overlay = results[0].plot()

        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
        overlay_msg.header = msg.header

        self.overlay_publisher.publish(overlay_msg)
        self.populate_detection2d_msg(results, msg.header)


def main(args=None):
    rclpy.init(args=args)

    image_detector = DetectImage()

    rclpy.spin(image_detector)

    image_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
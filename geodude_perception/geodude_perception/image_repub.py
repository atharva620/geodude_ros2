import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import numpy as np

class ImageRepublishNode(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
        self.bridge = CvBridge()
        self.new_width = 1280
        self.new_height = 720

        # Subscribers
        self.image_sub = Subscriber(self, Image, 'image_topic')
        self.camera_info_sub = Subscriber(self, CameraInfo, 'camera_info_topic')
        self.depth_sub = Subscriber(self, Image, 'depth_topic')

        #params
        self.camera_info_distortion_model = self.declare_parameter('camera_info_distortion_model', 'plumb_bob').value

        self.sync = ApproximateTimeSynchronizer([self.image_sub, self.camera_info_sub, self.depth_sub], 10, 0.5)
        self.sync.registerCallback(self.callback)

        # Publishers
        self.re_image_pub = self.create_publisher(Image, 're_image_topic', 10)
        self.re_camera_info_pub = self.create_publisher(CameraInfo, 're_camera_info_topic', 10)
        self.re_depth_pub = self.create_publisher(Image, 're_depth_topic', 10)

    def callback(self, image_msg, camera_info_msg, depth_msg):

        header = Header()
        header.stamp = image_msg.header.stamp
        header.frame_id = image_msg.header.frame_id

        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        depth = depth.astype(np.float32) / 1000.0

        new_image_msg = self.bridge.cv2_to_imgmsg(image, encoding='rgb8')
        new_depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        new_image_msg.header = header
        new_depth_msg.header = header

        camera_info_msg.header = header
        if self.camera_info_distortion_model:
            camera_info_msg.distortion_model = self.camera_info_distortion_model

        self.re_image_pub.publish(new_image_msg)
        self.re_camera_info_pub.publish(camera_info_msg)
        self.re_depth_pub.publish(new_depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageRepublishNode("image_resize_node")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
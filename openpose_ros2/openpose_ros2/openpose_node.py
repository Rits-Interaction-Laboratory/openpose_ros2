import numpy as np
import datetime
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header

from openpose_ros2.wrapper import OpenPoseWrapper

from openpose_ros2_msgs.msg import PoseKeyPointsList, PoseKeyPoint, PoseKeyPoints


class OpenPosePreviewNode(Node):
    openpose_wrapper: OpenPoseWrapper

    def __init__(self):
        super().__init__('openpose_node')

        # QoS Settings
        shigure_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ros params
        is_debug_mode_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL,
                                                       description='If true, run debug mode.')
        self.declare_parameter('is_debug_mode', False, is_debug_mode_descriptor)
        self.is_debug_mode: bool = self.get_parameter("is_debug_mode").get_parameter_value().bool_value
        openpose_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                                  description='The path of openpose project root.')
        self.declare_parameter('openpose_root', '/openpose', openpose_descriptor)
        openpose_root: str = self.get_parameter("openpose_root").get_parameter_value().string_value
        is_image_compressed_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL,
                                                             description='Is input image compressed?')
        self.declare_parameter('is_image_compressed', False, is_image_compressed_descriptor)
        is_image_compressed: bool = self.get_parameter("is_image_compressed").get_parameter_value().bool_value
        image_node_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                                    description='The node name of input image.')
        self.declare_parameter('image_node', '/image', image_node_descriptor)
        image_node: str = self.get_parameter("image_node").get_parameter_value().string_value

        self.openpose_wrapper = OpenPoseWrapper(openpose_root)
        self.bridge = CvBridge()

        # show info
        self.get_logger().info('IsDebugMode : ' + str(self.is_debug_mode))
        self.get_logger().info('OpenposeRoot : ' + openpose_root)
        self.get_logger().info('ImageNode : ' + image_node)
        self.get_logger().info('IsImageCompressed : ' + str(is_image_compressed))

        # debug print settings
        self.received_flag = True

        if self.is_debug_mode:
            self._publisher = self.create_publisher(Image, '/openpose/preview', 10)
            self._publisher_compressed = self.create_publisher(CompressedImage, '/openpose/preview/compressed', 10)
        self._pose_publisher = self.create_publisher(PoseKeyPointsList, '/openpose/pose_key_points', 10)

        if is_image_compressed:
            self.subscription = self.create_subscription(CompressedImage, image_node,
                                                         self.get_img_compressed_callback, shigure_qos)
        else:
            self.subscription = self.create_subscription(Image, image_node,
                                                         self.get_img_callback, shigure_qos)

    def publish_from_img(self, img: np.ndarray, timestamp: Time, frame_id: str =""):
        result = self.openpose_wrapper.body_from_image(img)
        if self.is_debug_mode:
            result_image: Image = self.bridge.cv2_to_imgmsg(result.cvOutputData, "rgb8")
            result_image_compressed: CompressedImage = self.bridge.cv2_to_compressed_imgmsg(result.cvOutputData)
            result_image.header.stamp = timestamp
            result_image.header.frame_id = frame_id
            result_image_compressed.header.stamp = timestamp
            result_image_compressed.header.frame_id = frame_id
            self._publisher.publish(result_image)
            self._publisher_compressed.publish(result_image_compressed)

        # Convert to KeyPointsList
        pose_key_points_list_obj = PoseKeyPointsList()
        if isinstance(result.poseKeypoints, np.ndarray):
            pose_key_points_list = []
            for result_pose_key_points in result.poseKeypoints:
                pose_key_points = []
                for result_pose_key_point in result_pose_key_points:
                    x, y, score = result_pose_key_point
                    pose_key_points.append(PoseKeyPoint(x=x.item(), y=y.item(), score=score.item()))
                pose_key_points_list.append(PoseKeyPoints(pose_key_points=pose_key_points))
            pose_key_points_list_obj = PoseKeyPointsList(pose_key_points_list=pose_key_points_list)
        pose_key_points_list_obj.header.stamp = timestamp
        pose_key_points_list_obj.header.frame_id = frame_id
        self._pose_publisher.publish(pose_key_points_list_obj)

    def get_img_callback(self, image_raw: Image) -> None:
        try:
            if self.received_flag:
                print('[' + str(datetime.datetime.now()) + '] Image received')
                self.received_flag = False
            #print('[' + str(datetime.datetime.now()) + '] Image received', end='\r')
            image: np.ndarray = self.bridge.imgmsg_to_cv2(image_raw)
            self.publish_from_img(image, image_raw.header.stamp, image_raw.header.frame_id)
        except Exception as err:
            self.get_logger().error(err)

    def get_img_compressed_callback(self, image_raw: CompressedImage) -> None:
        try:
            if self.received_flag:
                print('[' + str(datetime.datetime.now()) + '] Compressed image received')
                self.received_flag = False
            #print('[' + str(datetime.datetime.now()) + '] Compressed image received', end='\r')
            image: np.ndarray = self.bridge.compressed_imgmsg_to_cv2(image_raw)
            self.publish_from_img(image, image_raw.header.stamp, image_raw.header.frame_id)

        except Exception as err:
            self.get_logger().error(err)


def main(args=None):
    rclpy.init(args=args)

    openpose_preview_node = OpenPosePreviewNode()

    try:
        rclpy.spin(openpose_preview_node)

    except KeyboardInterrupt:
        pass

    finally:
        # 終了処理
        print()
        openpose_preview_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

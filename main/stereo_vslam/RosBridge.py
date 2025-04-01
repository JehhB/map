import cv2
import numpy as np
import rosgraph
import rospy
from cv_bridge import CvBridge
from PIL.Image import Image
from rospy import Publisher, Time
from sensor_msgs.msg import CameraInfo as CameraInfoRos
from sensor_msgs.msg import Image as ImageRos

from stereo_vslam.Calibrator import CameraInfo


class RosBridge:
    bridge: CvBridge

    left_image_pub: Publisher
    right_image_pub: Publisher
    left_camera_info_pub: Publisher
    right_camera_info_pub: Publisher

    def __init__(self):
        if not rosgraph.is_master_online():
            raise RuntimeError("roscore is not running")

        rospy.init_node("main_map")
        self.bridge = CvBridge()

        self.left_image_pub = Publisher(
            "/stereo_camera/left/image_raw", ImageRos, queue_size=10
        )
        self.right_image_pub = Publisher(
            "/stereo_camera/right/image_raw", ImageRos, queue_size=10
        )
        self.left_camera_info_pub = Publisher(
            "/stereo_camera/left/camera_info", CameraInfoRos, queue_size=10
        )
        self.right_camera_info_pub = Publisher(
            "/stereo_camera/right/camera_info", CameraInfoRos, queue_size=10
        )

    def _image_to_imgmsg(self, img: Image) -> ImageRos:
        img_np = np.array(img)
        if len(img_np.shape) == 2:
            img_np = cv2.cvtColor(img_np, cv2.COLOR_GRAY2BGR)
        return self.bridge.cv2_to_imgmsg(img_np, encoding="bgr8")

    def _caminfo_to_caminfomsg(self, info: CameraInfo) -> CameraInfoRos:
        ret = CameraInfoRos()

        ret.width = info.width
        ret.height = info.height
        ret.distortion_model = info.distortion_model
        ret.D = info.distortion_parameters
        ret.K = info.intrinsic_matrix
        ret.R = info.rectification_matrix
        ret.P = info.projection_matrix

        return ret

    def send_stereo_image(
        self,
        left_image: Image,
        right_image: Image,
        left_camera_info: CameraInfo,
        right_camera_info: CameraInfo,
        timestamp: int,
    ):
        current_time = Time.from_sec(timestamp / 1e9)

        left_image_msg = self._image_to_imgmsg(left_image)
        right_image_msg = self._image_to_imgmsg(right_image)
        left_camera_info_msg = self._caminfo_to_caminfomsg(left_camera_info)
        right_camera_info_msg = self._caminfo_to_caminfomsg(right_camera_info)

        left_image_msg.header.stamp = current_time
        right_image_msg.header.stamp = current_time
        left_camera_info_msg.header.stamp = current_time
        right_camera_info_msg.header.stamp = current_time

        self.left_camera_info_pub.publish(left_camera_info_msg)
        self.right_camera_info_pub.publish(right_camera_info_msg)
        self.left_image_pub.publish(left_image_msg)
        self.right_image_pub.publish(right_image_msg)

    def destroy(self):
        rospy.signal_shutdown("Don't need anymore")
        self.left_image_pub.unregister()
        self.right_image_pub.unregister()
        self.left_camera_info_pub.unregister()
        self.right_camera_info_pub.unregister()

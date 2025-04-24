import traceback
from typing import Callable, Optional, Tuple

import cv2
import numpy as np
import rosgraph
import rospy
from cv2.typing import MatLike
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from PIL.Image import Image
from reactivex import Observable, operators
from reactivex.subject import BehaviorSubject
from rospy import Publisher, ServiceProxy, Subscriber, Time
from rtabmap_msgs.msg import MapGraph
from sensor_msgs.msg import CameraInfo as CameraInfoRos
from sensor_msgs.msg import Image as ImageRos
from std_srvs.srv import Empty
from stereo_msgs.msg import DisparityImage

from ratmap_common.EventTarget import EventTarget

from .CameraInfo import CameraInfo


class RosBridge(EventTarget):
    bridge: CvBridge

    left_image_pub: Publisher
    right_image_pub: Publisher
    left_camera_info_pub: Publisher
    right_camera_info_pub: Publisher
    occupancy_grid_sub: Subscriber
    map_graph_sub: Subscriber

    disparity_image_sub: Subscriber
    disparity_subject: BehaviorSubject[Optional[DisparityImage]]
    disparity_mat_observable: Observable[Optional[Tuple[DisparityImage, MatLike]]]
    disparity_image_observable: Observable[Optional[MatLike]]

    reset_service: ServiceProxy
    reset_odom_service: ServiceProxy

    def __init__(
        self,
        grid_callback: Optional[Callable[[OccupancyGrid], None]] = None,
        map_callback: Optional[Callable[[MapGraph], None]] = None,
    ):
        super().__init__()

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

        self.occupancy_grid_sub = Subscriber(
            "/grid_map",
            OccupancyGrid,
            lambda grid: grid_callback(grid) if grid_callback is not None else None,
            queue_size=10,
        )
        self.map_graph_sub = Subscriber(
            "/mapGraph",
            MapGraph,
            lambda map: map_callback(map) if map_callback is not None else None,
            queue_size=10,
        )

        self.disparity_subject = BehaviorSubject(None)
        self.disparity_image_sub = Subscriber(
            "/stereo_camera/disparity",
            DisparityImage,
            self.disparity_subject.on_next,
            queue_size=10,
        )

        self.disparity_mat_observable = self.disparity_subject.pipe(
            operators.map(self.convert_disparity_to_mat)
        )
        self.disparity_image_observable = self.disparity_mat_observable.pipe(
            operators.map(self.convert_mat_to_img)
        )

        self.reset_service = ServiceProxy("/reset", Empty)
        self.reset_odom_service = ServiceProxy("/reset_odom", Empty)

    def __image_to_imgmsg(self, img: Image) -> ImageRos:
        img_np = np.array(img)
        if len(img_np.shape) == 2:
            img_np = cv2.cvtColor(img_np, cv2.COLOR_GRAY2BGR)
        return self.bridge.cv2_to_imgmsg(img_np, encoding="bgr8")

    def __caminfo_to_caminfomsg(self, info: CameraInfo) -> CameraInfoRos:
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
    ):
        current_time = Time.now()

        left_image_msg = self.__image_to_imgmsg(left_image)
        right_image_msg = self.__image_to_imgmsg(right_image)
        left_camera_info_msg = self.__caminfo_to_caminfomsg(left_camera_info)
        right_camera_info_msg = self.__caminfo_to_caminfomsg(right_camera_info)

        left_image_msg.header.stamp = current_time
        left_image_msg.header.frame_id = "stereo_camera"
        right_image_msg.header.stamp = current_time
        right_image_msg.header.frame_id = "stereo_camera"

        left_camera_info_msg.header.stamp = current_time
        left_camera_info_msg.header.frame_id = "stereo_camera"
        right_camera_info_msg.header.stamp = current_time
        right_camera_info_msg.header.frame_id = "stereo_camera"

        self.left_camera_info_pub.publish(left_camera_info_msg)
        self.right_camera_info_pub.publish(right_camera_info_msg)
        self.left_image_pub.publish(left_image_msg)
        self.right_image_pub.publish(right_image_msg)

    def destroy(self):
        self.left_image_pub.unregister()
        self.right_image_pub.unregister()
        self.left_camera_info_pub.unregister()
        self.right_camera_info_pub.unregister()
        self.disparity_image_sub.unregister()
        self.occupancy_grid_sub.unregister()
        self.map_graph_sub.unregister()
        self.disparity_subject.on_completed()

        rospy.signal_shutdown("Don't need anymore")

    def convert_disparity_to_mat(
        self, disparity: Optional[DisparityImage]
    ) -> Optional[Tuple[DisparityImage, MatLike]]:
        if disparity is None:
            return None

        return (disparity, self.bridge.imgmsg_to_cv2(disparity.image))

    def convert_mat_to_img(
        self,
        inp: Optional[Tuple[DisparityImage, MatLike]],
    ) -> Optional[MatLike]:
        if inp is None:
            return None

        disparity, disp = inp
        min = disparity.min_disparity
        max = disparity.max_disparity

        mapped = (np.clip(disp, min, max) - min) / (max - min) * 255
        return mapped.astype(np.uint8)

    def inspect(
        self, absolute_x: float, absolute_y: float
    ) -> Tuple[float, float, float, float]:
        disparity = self.disparity_subject.value
        if disparity is None:
            return (absolute_x, absolute_y, 0, 0)

        width = disparity.image.width
        height = disparity.image.height

        disp = self.bridge.imgmsg_to_cv2(disparity.image)

        x = int(absolute_x * width)
        y = int(absolute_y * height)

        d: float = disp[y, x]

        if d > 0:
            depth = disparity.f * disparity.T / d
        else:
            depth = 0

        return (float(x), float(y), float(d), float(depth))

    def reset(self):
        try:
            self.reset_odom_service()
            self.reset_service()
        except:
            traceback.print_exc()

import os
from dataclasses import dataclass
from fileinput import filename
from typing import List, Optional, Tuple, TypedDict

import cv2
import numpy as np
import yaml
from cv2.typing import MatLike
from PIL import Image
from reactivex.subject import BehaviorSubject, Subject
from typing_extensions import TypeAlias


@dataclass
class CameraInfo:
    height: int
    width: int
    distortion_parameters: List[
        float
    ]  # For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
    intrinsic_matrix: List[float]  # 3x3 row-major matrix
    rectification_matrix: List[float]  # 3x3 row-major matrix
    projection_matrix: List[float]  # 3x4 row-major matrix
    error: float
    distortion_model: str = "plumb_bob"


@dataclass
class StereoInfo:
    disparity_to_depth_matrix: MatLike
    essential_matrix: MatLike
    fundamental_matrix: MatLike
    error: float


class CalibratorParams(TypedDict):
    chessboard_size: Tuple[int, int]
    square_size: float


StereoCameraInfo: TypeAlias = Tuple[CameraInfo, CameraInfo, StereoInfo]


class Calibrator:
    MINIMUM_NUMBER_SAMPLE: int = 5

    calibration_flags: int

    objpoints: List[MatLike]
    imgpoints_left: List[MatLike]
    imgpoints_right: List[MatLike]

    is_calibrating: BehaviorSubject[bool]

    number_of_samples: BehaviorSubject[int]
    left_image_with_drawing: Subject[Image.Image]
    right_image_with_drawing: Subject[Image.Image]
    stereo_camera_info: BehaviorSubject[Optional[StereoCameraInfo]]

    img_shape: Optional[Tuple[int, ...]]

    def __init__(
        self,
        calibration_flags: int = cv2.CALIB_FIX_ASPECT_RATIO
        + cv2.CALIB_ZERO_TANGENT_DIST
        + cv2.CALIB_SAME_FOCAL_LENGTH,
    ):
        """
        Initialize the reactive stereo calibrator.

        Args:
            chessboard_size: (width, height) of the chessboard pattern in terms of inner corners
            square_size: Real-world size of each chessboard square in meters
            calibration_flags: OpenCV calibration flags
        """
        self.calibration_flags = calibration_flags

        self.objpoints = []
        self.imgpoints_left = []
        self.imgpoints_right = []

        self.is_calibrating = BehaviorSubject(False)
        self.number_of_samples = BehaviorSubject(0)
        self.stereo_camera_info = BehaviorSubject(None)
        self.left_image_with_drawing = Subject()
        self.right_image_with_drawing = Subject()

        self.img_shape = None

        self.chessboard_size: Tuple[int, int]
        self.square_size: float
        self.objp: MatLike

    def start(self, params: CalibratorParams):
        self.chessboard_size = params["chessboard_size"]
        self.square_size = params["square_size"]
        self.objp = np.zeros(
            (self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32
        )
        self.objp[:, :2] = np.mgrid[
            0 : self.chessboard_size[0], 0 : self.chessboard_size[1]
        ].T.reshape(-1, 2)
        self.objp *= self.square_size

        self.is_calibrating.on_next(True)
        self.stereo_camera_info.on_next(None)

    def next(
        self, images: Tuple[Optional[Image.Image], Optional[Image.Image], int, bool]
    ):
        """
        Process a new pair of stereo images for calibration.

        Args:
            left_image: PIL.Image from left camera
            right_image: PIL.Image from right camera

        Returns:
            bool: True if corners found in both images, False otherwise
        """
        left_image, right_image, _timestamp, _is_calibrating = images

        if not self.is_calibrating.value:
            return

        if left_image is None or right_image is None:
            return

        left_cv = self._pil_to_cv(left_image)
        right_cv = self._pil_to_cv(right_image)

        if left_cv.shape[:2] != right_cv.shape[:2]:
            raise RuntimeError("Left and right image does not have the same shape")

        if self.img_shape is None:
            self.img_shape = left_cv.shape[:2]
        elif self.img_shape != left_cv.shape[:2]:
            raise RuntimeError("Img shape does not match previous shape")

        _ = self._process_image_pair(left_cv, right_cv)

    def _pil_to_cv(self, pil_image: Image.Image):
        """Convert PIL Image to OpenCV format."""
        if pil_image.mode != "RGB":
            pil_image = pil_image.convert("RGB")
        open_cv_image = np.array(pil_image)

        open_cv_image = open_cv_image[:, :, ::-1].copy()
        return open_cv_image

    def _cv_to_pil(self, cv_image: MatLike):
        """Convert OpenCV image to PIL Image."""

        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        return Image.fromarray(rgb_image)

    def _process_image_pair(self, left_cv: MatLike, right_cv: MatLike):
        """Process a pair of OpenCV format images."""
        gray_left = cv2.cvtColor(left_cv, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_cv, cv2.COLOR_BGR2GRAY)

        ret_left, corners_left = cv2.findChessboardCorners(
            gray_left, self.chessboard_size, None
        )
        ret_right, corners_right = cv2.findChessboardCorners(
            gray_right, self.chessboard_size, None
        )

        if ret_left and ret_right:

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_left = cv2.cornerSubPix(
                gray_left, corners_left, (11, 11), (-1, -1), criteria
            )
            corners_right = cv2.cornerSubPix(
                gray_right, corners_right, (11, 11), (-1, -1), criteria
            )

            self.objpoints.append(self.objp)
            self.imgpoints_left.append(corners_left)
            self.imgpoints_right.append(corners_right)

            img_left = cv2.drawChessboardCorners(
                left_cv.copy(), self.chessboard_size, corners_left, ret_left
            )
            img_right = cv2.drawChessboardCorners(
                right_cv.copy(), self.chessboard_size, corners_right, ret_right
            )

            self.number_of_samples.on_next(len(self.objpoints))
            self.left_image_with_drawing.on_next(self._cv_to_pil(img_left))
            self.right_image_with_drawing.on_next(self._cv_to_pil(img_right))
            return True

        self.left_image_with_drawing.on_next(self._cv_to_pil(left_cv))
        self.right_image_with_drawing.on_next(self._cv_to_pil(right_cv))

        return False

    def update_calibration(self):
        """Perform stereo calibration with current samples."""
        if len(self.objpoints) < Calibrator.MINIMUM_NUMBER_SAMPLE:
            raise RuntimeError("Too little samples to calibrate")

        if self.img_shape is None:
            raise RuntimeError("Cannot determin size")

        ret_left, KL, DL, _rvecs_left, _tvecs_left = cv2.calibrateCamera(
            self.objpoints,
            self.imgpoints_left,
            self.img_shape,
            None,
            None,
        )
        print("calibrated left camera")

        ret_right, KR, DR, _rvecs_right, _tvecs_right = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_right, self.img_shape, None, None
        )
        print("calibrated right camera")

        ret, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
            self.objpoints,
            self.imgpoints_left,
            self.imgpoints_right,
            KL,
            DL,
            KR,
            DR,
            self.img_shape,
            flags=self.calibration_flags,
            criteria=(
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                100,
                1e-5,
            ),
        )
        print("calibrated stereo camera")

        R1, R2, P1, P2, Q, _roi_left, _roi_right = cv2.stereoRectify(
            K1,
            D1,
            K2,
            D2,
            self.img_shape,
            R,
            T,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0,
        )
        print("calibrated stereo rectification")

        ret = (
            self._get_camera_info(D1, K1, R1, P1, ret_left),
            self._get_camera_info(D2, K2, R2, P2, ret_right),
            StereoInfo(
                disparity_to_depth_matrix=Q,
                essential_matrix=E,
                fundamental_matrix=F,
                error=ret,
            ),
        )
        self.stereo_camera_info.on_next(ret)
        print("calibrated done")

        return ret

    def _get_camera_info(
        self,
        distortion_params: MatLike,
        intrinsic_matrix: MatLike,
        rectification_matrix: MatLike,
        projection_matrix: MatLike,
        error: float,
    ):
        if self.img_shape is None:
            raise RuntimeError("Cannot determin size")

        dist_params_list: List[float] = distortion_params.flatten()[:5].tolist()
        intrinsic_list: List[float] = intrinsic_matrix.flatten().tolist()
        rect_list: List[float] = rectification_matrix.flatten().tolist()
        proj_list: List[float] = projection_matrix.flatten().tolist()

        height, width = self.img_shape

        return CameraInfo(
            height=height,
            width=width,
            distortion_parameters=dist_params_list,
            intrinsic_matrix=intrinsic_list,
            rectification_matrix=rect_list,
            projection_matrix=proj_list,
            error=error,
        )

    def resume(self):
        self.is_calibrating.on_next(True)

    def reset(self):
        """Reset the calibrator to start fresh."""
        self.objpoints = []
        self.imgpoints_left = []
        self.imgpoints_right = []
        self.img_shape = None
        self.number_of_samples.on_next(0)
        self.is_calibrating.on_next(False)
        self.stereo_camera_info.on_next(None)

    def pause(self):
        self.is_calibrating.on_next(False)

    def save(self, file_name: str) -> bool:
        yaml_dump = self.get_calibration_info()

        if yaml_dump is None:
            return False

        try:
            with open(file_name, "w") as f:
                _ = f.write(yaml_dump)

            return True
        except Exception as e:
            print(e.__repr__())
            return False

    def get_calibration_info(self) -> Optional[str]:
        if self.stereo_camera_info.value is None:
            return None

        left_camera_info, right_camera_info, stereo_info = self.stereo_camera_info.value

        # Convert numpy matrices to lists for YAML serialization
        serializable_stereo_info = {
            "disparity_to_depth_matrix": (
                stereo_info.disparity_to_depth_matrix.tolist()
            ),
            "essential_matrix": (stereo_info.essential_matrix.tolist()),
            "fundamental_matrix": (stereo_info.fundamental_matrix.tolist()),
            "error": stereo_info.error,
        }

        # Create dictionary for YAML serialization
        camera_data = {
            "left_camera_info": {
                "height": left_camera_info.height,
                "width": left_camera_info.width,
                "distortion_parameters": left_camera_info.distortion_parameters,
                "intrinsic_matrix": left_camera_info.intrinsic_matrix,
                "rectification_matrix": left_camera_info.rectification_matrix,
                "projection_matrix": left_camera_info.projection_matrix,
                "error": left_camera_info.error,
                "distortion_model": left_camera_info.distortion_model,
            },
            "right_camera_info": {
                "height": right_camera_info.height,
                "width": right_camera_info.width,
                "distortion_parameters": right_camera_info.distortion_parameters,
                "intrinsic_matrix": right_camera_info.intrinsic_matrix,
                "rectification_matrix": right_camera_info.rectification_matrix,
                "projection_matrix": right_camera_info.projection_matrix,
                "error": right_camera_info.error,
                "distortion_model": right_camera_info.distortion_model,
            },
            "stereo_info": serializable_stereo_info,
        }

        return yaml.dump(camera_data, default_flow_style=True)

    def load(self, file_name: str) -> bool:
        try:
            # Read YAML file
            with open(file_name, "r") as f:
                camera_data = yaml.safe_load(f)

            left_data = camera_data["left_camera_info"]
            left_camera_info = CameraInfo(
                height=left_data["height"],
                width=left_data["width"],
                distortion_parameters=left_data["distortion_parameters"],
                intrinsic_matrix=left_data["intrinsic_matrix"],
                rectification_matrix=left_data["rectification_matrix"],
                projection_matrix=left_data["projection_matrix"],
                error=left_data["error"],
                distortion_model=left_data.get("distortion_model", "plumb_bob"),
            )

            right_data = camera_data["right_camera_info"]
            right_camera_info = CameraInfo(
                height=right_data["height"],
                width=right_data["width"],
                distortion_parameters=right_data["distortion_parameters"],
                intrinsic_matrix=right_data["intrinsic_matrix"],
                rectification_matrix=right_data["rectification_matrix"],
                projection_matrix=right_data["projection_matrix"],
                error=right_data["error"],
                distortion_model=right_data.get("distortion_model", "plumb_bob"),
            )

            stereo_data = camera_data["stereo_info"]
            stereo_info = StereoInfo(
                disparity_to_depth_matrix=np.array(
                    stereo_data["disparity_to_depth_matrix"]
                ),
                essential_matrix=np.array(stereo_data["essential_matrix"]),
                fundamental_matrix=np.array(stereo_data["fundamental_matrix"]),
                error=stereo_data["error"],
            )

            # Update the stereo camera info
            self.stereo_camera_info.on_next(
                (left_camera_info, right_camera_info, stereo_info)
            )
            return True
        except:
            return False

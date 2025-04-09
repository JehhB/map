from dataclasses import dataclass
from typing import List, Tuple

from cv2.typing import MatLike
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
    translation_vector: MatLike
    error: float


StereoCameraInfo: TypeAlias = Tuple[CameraInfo, CameraInfo, StereoInfo]

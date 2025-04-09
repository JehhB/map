from __future__ import annotations

import tkinter as tk
from typing import Optional, Tuple, final

from cv2.typing import MatLike
from reactivex.abc import DisposableBase
from stereo_msgs.msg import DisparityImage as DisparityImageRos
from typing_extensions import override

from app.ui.ImageLabel import ImageLabel
from stereo_vslam.RosBridge import RosBridge


@final
class DisparityImage(ImageLabel):
    ros_bridge: Optional[RosBridge]
    current_disparity: Optional[Tuple[DisparityImageRos, MatLike]]
    disparity_disposer: Optional[DisposableBase]

    def __init__(
        self,
        master: tk.Misc,
        x_variable: tk.IntVar,
        y_variable: tk.IntVar,
        disparity_variable: tk.DoubleVar,
        depth_variable: tk.DoubleVar,
        width: int = 100,
        height: int = 100,
        ros_bridge: Optional[RosBridge] = None,
        **kwargs,
    ):
        super().__init__(master, width, height, **kwargs)
        self.ros_bridge = ros_bridge

        _ = self.bind("<Motion>", self.hover)
        self.x_variable = x_variable
        self.y_variable = y_variable
        self.disparity_variable = disparity_variable
        self.depth_variable = depth_variable
        self.current_disparity = None

        if ros_bridge is not None:
            self.disparity_disposer = ros_bridge.disparity_mat_observable.subscribe(
                self.set_current_disparity
            )

    def set_current_disparity(self, x: Optional[Tuple[DisparityImageRos, MatLike]]):
        self.current_disparity = x
        if self.ros_bridge is not None:
            self.update_from_cv2(self.ros_bridge.convert_mat_to_img(x))
        else:
            self.update_image(None)

    def hover(self, event: tk.Event["DisparityImage"]):
        if self.current_disparity is None:
            return

        disparity, disp = self.current_disparity
        if disparity is None:
            return

        width = disparity.image.width
        height = disparity.image.height

        x = int(event.x / self.width * width)
        y = int(event.y / self.height * height)

        self.x_variable.set(x)
        self.y_variable.set(y)

        d: float = disp[y, x]

        self.disparity_variable.set(d)
        if d > 0:
            self.depth_variable.set(disparity.f * disparity.T / d)

    @override
    def destroy(self) -> None:
        if self.disparity_disposer:
            self.disparity_disposer.dispose()
        return super().destroy()

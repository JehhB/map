from threading import Lock
from typing import Optional, Tuple

import reactivex as rx
from app.AbstractExtension import AbstractExtension, ExtensionMetadata
from app.Container import AbstractModule, ModuleDefinition
from app.events.AbstractEvent import AbstractEvent
from app.ExtensionManager import ExtensionManager
from app.ui.Main import Main as AppMain
from app.ui.MainGl import MainGl
from app.ui.MenuBar import MenuBar
from PIL.Image import Image
from reactivex.abc import DisposableBase
from reactivex.operators import throttle_first
from reactivex.subject import BehaviorSubject
from stereo_vslam.Calibrator import Calibrator, CalibratorParams, StereoCameraInfo
from stereo_vslam.RosBridge import RosBridge
from stereo_vslam.ui.Main import Main
from typing_extensions import Self, override


class StereoVslamExtension(AbstractModule, AbstractExtension):
    EXTENSION_LABEL: str = "Stereo VSLAM"
    TARGET_FPS: float = 20.0

    menu_bar: MenuBar
    main_gl: MainGl
    main_window: Optional[Main]
    ros_bridge: Optional[RosBridge]

    left_image_subject: rx.Subject[Optional[Image]]
    right_image_subject: rx.Subject[Optional[Image]]
    timestamp_subject: rx.Subject[int]
    stereo_image_observable: rx.Observable[
        Tuple[Optional[Image], Optional[Image], int, bool]
    ]

    _stereo_image_disposer: Optional[DisposableBase]
    _calibrator_disposer: Optional[DisposableBase]

    calibrator: Calibrator
    calibrator_params: BehaviorSubject[CalibratorParams]

    lock: Lock

    @property
    def metadata(self) -> ExtensionMetadata:
        return ExtensionMetadata(
            "Stereo VSLAM",
            "A Stereo VSLAM extension to calibrate and map stereo vision system",
            [],
        )

    @override
    @staticmethod
    def KEY() -> str:
        return "stereo_vslam.main"

    @override
    @classmethod
    def DEFINITION(cls) -> ModuleDefinition[Self]:
        return (
            ExtensionManager.defaultFactory("stereo_vslam", cls),
            [ExtensionManager, MenuBar, AppMain],
        )

    def __init__(self):
        super().__init__()

        self.main_window = None
        self.ros_bridge = None
        self.lock = Lock()
        self._stereo_image_disposer = None

        self.left_image_subject = BehaviorSubject(None)
        self.right_image_subject = BehaviorSubject(None)
        self.timestamp_subject = rx.Subject()

        self.calibrator = Calibrator()
        self.calibrator_params = BehaviorSubject(
            {
                "chessboard_size": (7, 6),
                "square_size": 3,
            }
        )

        self.stereo_image_observable = rx.combine_latest(
            self.left_image_subject,
            self.right_image_subject,
            self.timestamp_subject,
            self.calibrator.is_calibrating,
        ).pipe(throttle_first(1 / StereoVslamExtension.TARGET_FPS))

        self.add_event_handler("init", self.on_init)
        self.add_event_handler("deinit", self.on_deinit)

        self._stereo_image_disposer = None
        self._calibrator_disposer = None

    def on_init(self, event: AbstractEvent):
        if self.container is None:
            event.is_success = False
            return

        try:
            self.ros_bridge = RosBridge(self)
        except RuntimeError as e:
            event.detail = e
            event.is_success = False
            return

        self._stereo_image_disposer = rx.combine_latest(
            self.stereo_image_observable, self.calibrator.stereo_camera_info
        ).subscribe(on_next=self.process_images)

        self._calibrator_disposer = self.stereo_image_observable.pipe(
            throttle_first(0.8)
        ).subscribe(on_next=self.calibrator.next)

        self.menu_bar = self.container[MenuBar]
        self.menu_bar.extension_menu.add_command(
            label=StereoVslamExtension.EXTENSION_LABEL, command=self.open_window
        )
        self.main_gl = self.container[MainGl]

    def on_deinit(self, _event: AbstractEvent):
        if self.ros_bridge is not None:
            self.ros_bridge.destroy()
            self.ros_bridge = None

        if self._stereo_image_disposer is not None:
            self._stereo_image_disposer.dispose()

        if self._calibrator_disposer is not None:
            self._calibrator_disposer.dispose()

        try:
            _ = self.menu_bar.extension_menu.remove(
                StereoVslamExtension.EXTENSION_LABEL
            )
        except:
            pass

    def clear_ros(self):
        if self.ros_bridge:
            self.ros_bridge.reset_service()

    def open_window(self):
        if self.container is None:
            return

        if self.main_window is not None:
            self.main_window.lift()
            return

        self.main_window = Main(
            self,
            self.container[AppMain],
        )

        def on_close():
            if self.main_window is None:
                return

            self.main_window.destroy()
            self.main_window = None

        self.main_window.protocol("WM_DELETE_WINDOW", on_close)

    def process_images(
        self,
        images: Tuple[
            Tuple[Optional[Image], Optional[Image], int, bool],
            Optional[StereoCameraInfo],
        ],
    ):
        stereo_images, camera_info = images
        left_image, right_image, _timestamp, is_calibrating = stereo_images

        if (
            left_image is None
            or right_image is None
            or camera_info is None
            or self.ros_bridge is None
            or is_calibrating
        ):
            return

        left_camera_info, right_camera_info, _stereo_info = camera_info

        self.ros_bridge.send_stereo_image(
            left_image,
            right_image,
            left_camera_info,
            right_camera_info,
        )

    def start_calibration(self):
        self.calibrator.start(self.calibrator_params.value)

    def stop_calibration(self):
        self.calibrator.pause()

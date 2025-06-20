import os
import traceback
from tkinter import messagebox
from typing import TYPE_CHECKING, Optional, Tuple, cast, final

from reactivex import observable, operators
from reactivex.abc import DisposableBase
from reactivex.subject import BehaviorSubject
from typing_extensions import override

from ratmap_common.AbstractEvent import AbstractEvent
from ratmap_common.AbstractExtension import ExtensionMetadata
from ratmap_core import BaseExtension, JoystickEvent
from tkinter_rx.util import SetDisposer, safe_dispose

from .ImageTransformer import ImageTransformer
from .ui import StereoUmgvToolbar, StereoUmgvWindow
from .UmgvBridge import UmgvBridge

if TYPE_CHECKING:
    from ..stereo_vslam import StereoVslam


_EXTENSION_FOLDER = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_CAMERA_INFO = os.path.join(_EXTENSION_FOLDER, "cam_info_stereo_umgv.yaml")


@final
class StereoUmgv(BaseExtension):
    LABEL: str = "Ratmap UMGV"

    __extension_window: Optional[StereoUmgvWindow]
    __controler_disposable: Optional[DisposableBase]
    __transform_disposable: Optional[DisposableBase]

    __toolbar: Optional[StereoUmgvToolbar]

    @property
    @override
    def metadata(self) -> ExtensionMetadata:
        return {
            "id": "stereo_umgv",
            "title": "Ratmap UMGV",
            "description": "An extension to interface with ESP32-CAM based stereo UMGV",
            "dependency": ["stereo_vslam"],
        }

    def __init__(self) -> None:
        super().__init__()
        self.stereo_vslam: "StereoVslam"
        self.umgv_bridge: UmgvBridge
        self.__extension_window = None
        self.__toolbar = None

        _ = self.add_event_listener("stereo_umgv.connect", self.__connect_handler)

        _ = self.add_event_listener(
            "stereo_umgv.start_calibration",
            lambda e: self.stereo_vslam.start_calibration(),
        )

        _ = self.add_event_listener(
            "stereo_umgv.pause_calibration",
            lambda e: self.stereo_vslam.pause_calibration(e),
        )

        _ = self.add_event_listener(
            "stereo_umgv.reset_calibration",
            lambda e: self.stereo_vslam.reset_calibration(e),
        )

        _ = self.add_event_listener(
            "stereo_umgv.update_calibration",
            lambda e: self.stereo_vslam.update_calibration(e),
        )

        _ = self.add_event_listener(
            "stereo_umgv.start_mapping", lambda e: self.stereo_vslam.start_mapping()
        )

        self.__left_ip = BehaviorSubject("")
        self.__right_ip = BehaviorSubject("")

        self.__controler_disposable = None
        self.__right_transformer: ImageTransformer
        self.__left_transformer: ImageTransformer
        self.__transform_disposable = None

        self.__scale_disposable: Optional[DisposableBase] = None

    @override
    def start(self) -> None:
        self.ensure_deps()
        safe_dispose(self.__transform_disposable)
        safe_dispose(self.__controler_disposable)

        right_ip: str = self.context.config.get(
            f"{self.config_namespace}.right_ip", default=""
        )

        left_ip: str = self.context.config.get(
            f"{self.config_namespace}.left_ip", default=""
        )

        self.__right_ip.on_next(right_ip)
        self.__left_ip.on_next(left_ip)

        self.stereo_vslam = cast(
            "StereoVslam", self.extension_manager.get("stereo_vslam")
        )

        self.umgv_bridge = UmgvBridge(
            self.stereo_vslam.left_image_subject,
            self.stereo_vslam.right_image_subject,
            self.stereo_vslam.timestamp_subject,
        )
        scale: float = self.context.config.get(
            f"{self.config_namespace}.scaler", default=1.0
        )
        self.umgv_bridge.scaler.on_next(scale)
        safe_dispose(self.__scale_disposable)
        self.__scale_disposable = self.umgv_bridge.scaler.subscribe(
            lambda e: self.context.config.update(f"{self.config_namespace}.scaler", e)
        )

        if not self.stereo_vslam.extension_lock.acquire(True, 0.1):
            raise RuntimeError("Stereo VSLAM Extension currently inuse")

        if not self.stereo_vslam.load_calibration(_DEFAULT_CAMERA_INFO):
            raise RuntimeError("Stereo VSLAM Extension currently inuse")

        self.__right_transformer = ImageTransformer(
            f"{self.config_namespace}.right_transform", self.context.config
        )

        self.__left_transformer = ImageTransformer(
            f"{self.config_namespace}.left_transform", self.context.config
        )

        self.__transform_disposable = SetDisposer()
        self.__transform_disposable.add(
            self.__right_transformer.transformer_observable.subscribe(
                self.umgv_bridge.right_transform
            )
        )
        self.__transform_disposable.add(
            self.__left_transformer.transformer_observable.subscribe(
                self.umgv_bridge.left_transform
            )
        )

        self.stereo_vslam.calibrator_params.square_size.on_next(4.0)
        self.stereo_vslam.calibrator_params.chessboard_rows.on_next(4)
        self.stereo_vslam.calibrator_params.chessboard_cols.on_next(6)

        self.context.extension_menu.add_command(
            label=StereoUmgv.LABEL, command=self.__open_window
        )

        self.__controler_disposable = (
            self.context.observe("joystick.poll")
            .pipe(operators.pairwise())
            .subscribe(self.__handle_joystick)
        )

        if (
            self.stereo_vslam.left_image_subject is not None
            and self.stereo_vslam.right_image_subject is not None
        ):
            self.__toolbar = StereoUmgvToolbar(
                self.context.toolbar,
                left_image_observable=self.stereo_vslam.left_image_subject,
                right_image_observable=self.stereo_vslam.right_image_subject,
                x_subject=self.umgv_bridge.x_subject,
                y_subject=self.umgv_bridge.y_subject,
            )
            self.__toolbar.event_target.parent = self
            self.context.toolbar.add(self.__toolbar, text=StereoUmgv.LABEL)

        super().start()

    def __handle_joystick(self, events: Tuple[AbstractEvent, AbstractEvent]):
        prev, curr = events

        if not isinstance(curr, JoystickEvent) or not isinstance(prev, JoystickEvent):
            return

        detail = curr.detail
        self.umgv_bridge.x_subject.on_next(detail.right_stick[0])
        self.umgv_bridge.y_subject.on_next(detail.right_stick[1])

        x_prev_pressed = prev.detail.buttons[0]
        x_pressed = detail.buttons[0]

        if not x_prev_pressed and x_pressed:
            self.umgv_bridge.toggle_flashlight()

        left_trigger = (detail.axes[2] + 1.0) / 2.0
        right_trigger = (detail.axes[5] + 1.0) / 2.0

        if detail.left_bumper:
            self.umgv_bridge.flash.on_next(left_trigger)

        if detail.right_bumper:
            self.umgv_bridge.scaler.on_next(right_trigger)

    @override
    def stop(self) -> None:
        try:
            self.stereo_vslam.extension_lock.release()
        except:
            traceback.print_exc()

        self.__close_window()
        _ = self.context.extension_menu.remove(label=StereoUmgv.LABEL)

        try:
            _ = self.context.toolbar.remove(StereoUmgv.LABEL)
            if self.__toolbar is not None:
                self.__toolbar.destroy()
            self.__toolbar = None
        except:
            traceback.print_exc()

        self.__left_transformer.dispose()
        self.__right_transformer.dispose()

        safe_dispose(self.__scale_disposable)
        self.__scale_disposable = None

        safe_dispose(self.__controler_disposable)
        self.__controler_disposable = None

        safe_dispose(self.__transform_disposable)
        self.__transform_disposable = None

        self.umgv_bridge.dispose()

        super().stop()

    def __open_window(self):
        main_window = self.context.main_window

        if self.__extension_window is not None:
            self.__extension_window.lift()
            return

        calibrator = self.stereo_vslam.calibrator
        left_image_base = self.stereo_vslam.left_image_subject
        right_image_base = self.stereo_vslam.right_image_subject

        if calibrator is None or left_image_base is None or right_image_base is None:
            raise RuntimeError("Stereo VSLAM extension is not running properly")

        left_image = calibrator.is_calibrating.pipe(
            operators.map(
                lambda is_calibrating: (
                    calibrator.left_image_with_drawing
                    if is_calibrating
                    else left_image_base
                )
            ),
            operators.switch_latest(),
        )

        right_image = calibrator.is_calibrating.pipe(
            operators.map(
                lambda is_calibrating: (
                    calibrator.right_image_with_drawing
                    if is_calibrating
                    else right_image_base
                )
            ),
            operators.switch_latest(),
        )

        self.__extension_window = StereoUmgvWindow(
            main_window,
            self.umgv_bridge.x_subject,
            self.umgv_bridge.y_subject,
            self.umgv_bridge.scaler,
            self.umgv_bridge.flash,
            self.__right_transformer,
            self.__left_transformer,
            self.__left_ip,
            self.__right_ip,
        )
        self.__extension_window.protocol("WM_DELETE_WINDOW", self.__close_window)
        self.__extension_window.event_target.parent = self

        self.__extension_window.left_image_observable = left_image
        self.__extension_window.right_image_observable = right_image

        self.__extension_window.is_calibrating_observable = calibrator.is_calibrating

        self.__extension_window.calibration_count_observable = (
            calibrator.number_of_samples
        )

        calibrator_params = self.stereo_vslam.calibrator_params
        self.__extension_window.square_size_subject = calibrator_params.square_size
        self.__extension_window.chessboard_rows_subject = (
            calibrator_params.chessboard_rows
        )
        self.__extension_window.chessboard_cols_subject = (
            calibrator_params.chessboard_cols
        )

    def __close_window(self):
        if self.__extension_window is None:
            return

        self.__extension_window.destroy()
        self.__extension_window = None

    def __connect_handler(self, _e: AbstractEvent):
        right_ip = self.__right_ip.value
        left_ip = self.__left_ip.value

        res = self.umgv_bridge.connect(
            right_ip,
            left_ip,
        )

        if not res:
            _ = messagebox.showerror("Cannot connect", "Cannot connect to robot")
        else:
            self.context.config.update(f"{self.config_namespace}.right_ip", right_ip)
            self.context.config.update(f"{self.config_namespace}.left_ip", left_ip)

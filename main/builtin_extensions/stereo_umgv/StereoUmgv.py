import os
import traceback
from typing import TYPE_CHECKING, Optional, cast, final

from reactivex import operators
from reactivex.abc import DisposableBase
from typing_extensions import override

from ratmap_common.AbstractEvent import AbstractEvent
from ratmap_common.AbstractExtension import ExtensionMetadata
from ratmap_core import BaseExtension, JoystickEvent
from tkinter_rx.util import safe_dispose

from .ui import StereoUmgvWindow
from .UmgvBridge import UmgvBridge

if TYPE_CHECKING:
    from ..stereo_vslam import Calibrator, StereoVslam


_EXTENSION_FOLDER = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_CAMERA_INFO = os.path.join(_EXTENSION_FOLDER, "cam_info_stereo_umgv.yaml")


@final
class StereoUmgv(BaseExtension):
    LABEL: str = "Ratmap UMGV"

    __extension_window: Optional[StereoUmgvWindow]
    __controler_disposable: Optional[DisposableBase]

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

        _ = self.add_event_listener("stereo_umgv.connect", lambda e: print(e))

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
        self.__controler_disposable = None

    @override
    def start(self) -> None:
        super().start()

        self.stereo_vslam = cast(
            "StereoVslam", self.extension_manager.get("stereo_vslam")
        )
        self.umgv_bridge = UmgvBridge()

        if not self.stereo_vslam.lock.acquire(True, 0.1):
            raise RuntimeError("Stereo VSLAM Extension currently inuse")

        if not self.stereo_vslam.load_calibration(_DEFAULT_CAMERA_INFO):
            raise RuntimeError("Stereo VSLAM Extension currently inuse")

        self.context.extension_menu.add_command(
            label=StereoUmgv.LABEL, command=self.__open_window
        )

        self.__controler_disposable = self.context.add_event_listener(
            "joystick.poll", self.__handle_joystick
        )

    def __handle_joystick(self, event: AbstractEvent):
        if not isinstance(event, JoystickEvent):
            return

        detail = event.detail
        self.umgv_bridge.x_subject.on_next(detail.right_stick[0])
        self.umgv_bridge.y_subject.on_next(detail.right_stick[1])

    @override
    def stop(self) -> None:
        try:
            self.stereo_vslam.lock.release()
        except:
            traceback.print_exc()

        self.__close_window()
        _ = self.context.extension_menu.remove(label=StereoUmgv.LABEL)

        safe_dispose(self.__controler_disposable)
        self.__controler_disposable = None

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
            main_window, self.umgv_bridge.x_subject, self.umgv_bridge.y_subject
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

        master_ip: str = self.context.config.get(
            f"{self.config_namespace}.master_ip", default=""
        )
        slave_ip: str = self.context.config.get(
            f"{self.config_namespace}.slave_ip", default=""
        )

        def set_configs():
            if self.__extension_window is not None:
                self.__extension_window.master_ip.on_next(master_ip)
                self.__extension_window.slave_ip.on_next(slave_ip)

        _ = self.__extension_window.after(200, set_configs)

    def __close_window(self):
        if self.__extension_window is None:
            return

        self.__extension_window.destroy()
        self.__extension_window = None

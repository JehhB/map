import os
import traceback
from typing import TYPE_CHECKING, Optional, Tuple, cast, final

from cv2.typing import MatLike
from PIL.Image import fromarray
from reactivex import operators
from reactivex.abc import DisposableBase
from typing_extensions import override

from ratmap_common import AbstractEvent, ExtensionMetadata
from ratmap_core import BaseExtension
from tkinter_rx import TkinterEventDetail
from tkinter_rx.util import SetDisposer, safe_dispose

if TYPE_CHECKING:
    from ..stereo_vslam import StereoVslam

from .DatasetLoader import DatasetLoader, Metadata
from .Player import Player, PlayerState
from .ui import EurocMavLoaderWindow

_EXTENSION_FOLDER = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_CAMERA_INFO = os.path.join(_EXTENSION_FOLDER, "cam_info_euroc.yaml")


@final
class EurocMavLoader(BaseExtension):
    LABEL = "EuRoC MAV Loader"

    __loader: Optional[DatasetLoader]
    __player: Optional[Player[Tuple[MatLike, MatLike, Metadata]]]
    __extension_window: Optional[EurocMavLoaderWindow]

    __player_disposer: Optional[DisposableBase]

    @property
    @override
    def metadata(self) -> ExtensionMetadata:
        return {
            "id": "euroc_mav_loader",
            "title": "EuRoC MAV Loader",
            "description": "An extension to load EuRoC MAV Dataset to be used for Stereo VSLAM",
            "dependency": ["stereo_vslam"],
        }

    def __init__(self) -> None:
        super().__init__()

        self.stereo_vslam: "StereoVslam"

        self.__loader = None
        self.__player = None
        self.__extension_window = None

        self.__player_disposer = None

        _ = self.add_event_listener(
            "euroc_mav_loader.load_dataset", self.__load_dataset
        )
        _ = self.add_event_listener(
            "euroc_mav_loader.start_calibration", self.__start_calibration
        )
        _ = self.add_event_listener(
            "euroc_mav_loader.start_mapping", self.__start_mapping
        )
        _ = self.add_event_listener("euroc_mav_loader.set_fps", self.__set_fps)
        _ = self.add_event_listener("euroc_mav_loader.pause", self.__pause)
        _ = self.add_event_listener("euroc_mav_loader.play", self.__play)
        _ = self.add_event_listener("euroc_mav_loader.step", self.__step)
        _ = self.add_event_listener("euroc_mav_loader.stop", self.__stop)

    @override
    def start(self) -> None:
        self.ensure_deps()

        self.stereo_vslam = cast(
            "StereoVslam", self.extension_manager.get("stereo_vslam")
        )

        if not self.stereo_vslam.extension_lock.acquire(True, 0.1):
            raise RuntimeError("Stereo VSLAM Extension currently inuse")

        if not self.stereo_vslam.load_calibration(_DEFAULT_CAMERA_INFO):
            raise RuntimeError("Stereo VSLAM Extension currently inuse")

        self.context.extension_menu.add_command(
            label=EurocMavLoader.LABEL, command=self.__open_window
        )

        super().start()

    @override
    def stop(self) -> None:
        try:
            self.stereo_vslam.extension_lock.release()
        except:
            traceback.print_exc()

        if self.__player is not None:
            self.__player.dispose()

        self.__close_window()
        _ = self.context.extension_menu.remove(label=EurocMavLoader.LABEL)

        super().stop()

    def __open_window(self) -> None:
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

        self.__extension_window = EurocMavLoaderWindow(main_window)
        self.__extension_window.update_player_state(None)
        self.__extension_window.protocol("WM_DELETE_WINDOW", self.__close_window)
        self.__extension_window.event_target.parent = self

        self.__extension_window.left_image.image_observable = left_image
        self.__extension_window.right_image.image_observable = right_image

        path: str = self.context.config.get(f"{self.config_namespace}.path", default="")

        _ = self.__extension_window.after(
            200,
            lambda: (
                self.__extension_window.folder_path.on_next(path)
                if self.__extension_window
                else None
            ),
        )

    def __close_window(self) -> None:
        if self.__extension_window is None:
            return

        self.__extension_window.destroy()
        self.__extension_window = None

    def __load_dataset(self, _: AbstractEvent) -> None:
        if self.__extension_window is None:
            return
        path = self.__extension_window.folder_path.value
        self.context.config.update(f"{self.config_namespace}.path", path)
        self.__loader = DatasetLoader(path)

        if self.__player is not None:
            self.__player.dispose()

        images = iter(self.__loader)
        self.__player = Player(images)

        def update_images(inp: Tuple[MatLike, MatLike, Metadata]):
            left, right, meta = inp
            if self.stereo_vslam.left_image_subject is not None:
                self.stereo_vslam.left_image_subject.on_next(fromarray(left))

            if self.stereo_vslam.right_image_subject is not None:
                self.stereo_vslam.right_image_subject.on_next(fromarray(right))

            if self.stereo_vslam.timestamp_subject is not None:
                self.stereo_vslam.timestamp_subject.on_next(meta["timestamp"])

        def update_player_state(state: PlayerState):
            if self.__extension_window is not None:
                self.__extension_window.update_player_state(state)

        safe_dispose(self.__player_disposer)

        disposer = SetDisposer()
        disposer.add(self.__player.subject.subscribe(on_next=update_images))
        disposer.add(self.__player.state_subject.subscribe(on_next=update_player_state))
        self.__player_disposer = disposer

    def __start_mapping(self, _e: AbstractEvent) -> None:
        if self.__player is None:
            return

        self.stereo_vslam.start_mapping()

        _ = self.__player.reset()
        _ = self.__player.start()

    def __start_calibration(self, _e: AbstractEvent) -> None:
        if self.__player is None:
            return

        self.__start_mapping(_e)
        self.stereo_vslam.start_calibration(
            {"chessboard_size": (7, 6), "square_size": 6.0}
        )

    def __set_fps(self, ev: AbstractEvent) -> None:
        if not isinstance(ev.detail, TkinterEventDetail):
            return
        fps = int(ev.detail.additional)
        if self.__player:
            self.__player.set_frame_rate(fps)

    def __pause(self, _e: AbstractEvent) -> None:
        if self.__player:
            _ = self.__player.pause()

    def __step(self, _e: AbstractEvent) -> None:
        if self.__player:
            _ = self.__player.step()

    def __stop(self, _e: AbstractEvent) -> None:
        if self.__player:
            _ = self.__player.stop()

    def __play(self, _e: AbstractEvent) -> None:
        if self.__player:
            _ = self.__player.resume()

import os
import traceback
from typing import TYPE_CHECKING, Optional, Set, Tuple, cast, final

from cv2.typing import MatLike
from reactivex import operators
from reactivex.abc import DisposableBase
from typing_extensions import override

from ratmap_common import AbstractEvent, ExtensionMetadata
from ratmap_core import BaseExtension

if TYPE_CHECKING:
    from ..stereo_vslam import StereoVslam

from .DatasetLoader import DatasetLoader, Metadata
from .Player import Player
from .ui import EurocMavLoaderWindow

_EXTENSION_FOLDER = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_CAMERA_INFO = os.path.join(_EXTENSION_FOLDER, "cam_info_euroc.yaml")


@final
class EurocMavLoader(BaseExtension):
    LABEL = "EuRoC MAV Loader"

    __loader: Optional[DatasetLoader]
    __player: Optional[Player[Tuple[MatLike, MatLike, Metadata]]]
    __extension_window: Optional[EurocMavLoaderWindow]
    __disposers: Set[DisposableBase]

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

        self.__disposers = set()
        self.__disposers.add(
            self.add_event_listener(
                "euroc_mav_loader.load_dataset", self.__load_dataset
            )
        )

    @override
    def start(self) -> None:
        super().start()

        self.stereo_vslam = cast(
            "StereoVslam", self.extension_manager.get("stereo_vslam")
        )

        if not self.stereo_vslam.lock.acquire(True, 0.1):
            raise RuntimeError("Stereo VSLAM Extension currently inuse")

        if not self.stereo_vslam.load_calibration(_DEFAULT_CAMERA_INFO):
            raise RuntimeError("Stereo VSLAM Extension currently inuse")

        self.context.extension_menu.add_command(
            label=EurocMavLoader.LABEL, command=self.__open_window
        )

    @override
    def stop(self) -> None:
        try:
            self.stereo_vslam.lock.release()
        except:
            traceback.print_exc()

        self.__close_window()
        _ = self.context.extension_menu.remove(label=EurocMavLoader.LABEL)

        for disposer in self.__disposers:
            disposer.dispose()
        self.__disposers = set()

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
                    calibrator.left_image_with_drawing
                    if is_calibrating
                    else left_image_base
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
        print("loading")
        if self.__extension_window is None:
            return
        path = self.__extension_window.folder_path.value
        self.context.config.update(f"{self.config_namespace}.path", path)
        self.__loader = DatasetLoader(path)

    def __start_mapping(self) -> None:
        pass

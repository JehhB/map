import os
import traceback
from typing import TYPE_CHECKING, Optional, Tuple, cast, final

from cv2.typing import MatLike
from typing_extensions import override

from ratmap_common import ExtensionMetadata
from ratmap_core import BaseExtension

if TYPE_CHECKING:
    from ..stereo_vslam import StereoVslam

from .DatasetLoader import Metadata
from .Player import Player
from .ui import EurocMavLoaderWindow

_EXTENSION_FOLDER = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_CAMERA_INFO = os.path.join(_EXTENSION_FOLDER, "cam_info_euroc.yaml")


@final
class EurocMavLoader(BaseExtension):
    LABEL = "EuRoC MAV Loader"

    __player: Optional[Player[Tuple[MatLike, MatLike, Metadata]]]
    __extension_window: Optional[EurocMavLoaderWindow]

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

    @override
    def start(self) -> None:
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

        super().start()

    @override
    def stop(self) -> None:
        try:
            self.stereo_vslam.lock.release()
        except:
            traceback.print_exc()

        self.__close_window()
        _ = self.context.extension_menu.remove(label=EurocMavLoader.LABEL)

        super().stop()

    def __open_window(self) -> None:
        main_window = self.context.main_window

        if self.__extension_window is not None:
            self.__extension_window.lift()
            return

        self.__extension_window = EurocMavLoaderWindow(main_window)
        self.__extension_window.update_player_state(None)
        self.__extension_window.protocol("WM_DELETE_WINDOW", self.__close_window)

    def __close_window(self) -> None:
        if self.__extension_window is None:
            return

        self.__extension_window.destroy()
        self.__extension_window = None

    def __load_dataset(self) -> None:
        pass

    def __start_mapping(self) -> None:
        pass

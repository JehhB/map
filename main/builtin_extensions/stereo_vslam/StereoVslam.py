from typing import TYPE_CHECKING, cast, final

from typing_extensions import override

from ratmap_common import AbstractExtension, ExtensionMetadata

if TYPE_CHECKING:
    from ratmap_core import Application


@final
class StereoVslam(AbstractExtension):
    LABEL = "Stereo VSLAM"
    TARGET_FPS: float = 20.0

    @property
    @override
    def metadata(self) -> ExtensionMetadata:
        return {
            "id": "stereo_vslam",
            "title": "Stereo VSLAM",
            "description": "A Stereo VSLAM extension to calibrate and map stereo vision system",
        }

    def __init__(self) -> None:
        super().__init__()

    @override
    def start(self) -> None:
        application = cast("Application", self.context)
        application.extension_menu.add_command(
            label=StereoVslam.LABEL, command=self.__open_window
        )

        super().start()

    @override
    def stop(self) -> None:
        super().stop()

        application = cast("Application", self.context)
        _ = application.extension_menu.remove(label=StereoVslam.LABEL)

    def __close_window(self):
        pass

    def __open_window(self):
        print("open_window")

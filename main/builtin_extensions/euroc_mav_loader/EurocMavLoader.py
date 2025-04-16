from typing import TYPE_CHECKING, cast, final

from typing_extensions import override

from ratmap_common import ExtensionMetadata
from ratmap_core import BaseExtension

if TYPE_CHECKING:
    from stereo_vslam import StereoVslam


@final
class EurocMavLoader(BaseExtension):
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

    @override
    def start(self) -> None:
        self.stereo_vslam = cast(
            "StereoVslam", self.extension_manager.get("stereo_vslam")
        )

        return super().start()

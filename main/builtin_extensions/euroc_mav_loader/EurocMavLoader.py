from typing import final

from typing_extensions import override

from ratmap_common import AbstractExtension, ExtensionMetadata


@final
class EurocMavLoader(AbstractExtension):
    @property
    @override
    def metadata(self) -> ExtensionMetadata:
        return {
            "id": "euroc_mav_loader",
            "title": "EuRoC MAV Loader",
            "description": "An extension to load EuRoC MAV Dataset to be used for Stereo VSLAM",
        }

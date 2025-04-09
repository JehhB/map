from typing_extensions import override

from ratmap_common import AbstractExtension, ExtensionMetadata


class StereoVslam(AbstractExtension):
    @property
    @override
    def metadata(self) -> ExtensionMetadata:
        return {
            "title": "Stereo VSLAM",
            "description": "A Stereo VSLAM extension to calibrate and map stereo vision system",
        }

    def __init__(self) -> None:
        super().__init__()

    @override
    def start(self) -> None:
        return super().start()

import multiprocessing
import os
import tkinter as tk
from typing import TYPE_CHECKING, Optional, cast, final

import numpy as np
from cv2.typing import MatLike
from cvzone.PoseModule import PoseDetector
from PIL import Image
from reactivex import operators
from reactivex.scheduler import ThreadPoolScheduler
from typing_extensions import override

from ratmap_common import ExtensionMetadata
from ratmap_core import BaseExtension
from tkinter_rx import ImageLabel

if TYPE_CHECKING:
    from ....builtin_extensions.stereo_vslam import StereoVslam

_EXTENSION_FOLDER = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_MODEL = os.path.join(_EXTENSION_FOLDER, "yolov8n.pt")


@final
class Detector(BaseExtension):
    LABEL = "Detector"

    def __init__(self) -> None:
        super().__init__()
        self.__extension_window: Optional[tk.Toplevel] = None
        self.thread_pool_scheduler = ThreadPoolScheduler(multiprocessing.cpu_count())
        self.detector = PoseDetector()

    @property
    @override
    def metadata(self) -> ExtensionMetadata:
        return {
            "id": "detector",
            "title": "Object Detector",
            "dependency": ["stereo_vslam"],
            "description": "A object detector using YOLOv11",
        }

    @override
    def start(self) -> None:
        self.ensure_deps()

        self.context.extension_menu.add_command(
            label=Detector.LABEL, command=self.__open_window
        )

        super().start()

    @override
    def stop(self) -> None:
        super().stop()

    def __process_image(self, img: Optional[Image.Image]) -> Optional[MatLike]:
        if img is None:
            return None

        img_array = np.array(img)

        img_a = self.detector.findPose(img_array)
        bbox = self.detector.findPosition(img_a, draw=True)

        return img_a

    def __open_window(self):
        if self.__extension_window is not None:
            self.__extension_window.lift()
            return

        stereo_vslam = cast("StereoVslam", self.extension_manager.get("stereo_vslam"))
        if stereo_vslam.left_image_subject is None:
            return

        self.__extension_window = tk.Toplevel(self.context.main_window)
        self.__extension_window.title("Object Detector")

        image = stereo_vslam.left_image_subject.pipe(
            operators.throttle_first(0.5),
            operators.observe_on(self.thread_pool_scheduler),
            operators.map(self.__process_image),
        )

        process = ImageLabel(self.__extension_window, width=480, height=360)
        process.pack()

        process.image_observable = image

        self.__extension_window.protocol("WM_DELETE_WINDOW", self.__close_window)

    def __close_window(self):
        if self.__extension_window is None:
            return

        self.__extension_window.destroy()
        self.__extension_window = None

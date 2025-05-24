import multiprocessing
import tkinter as tk
import traceback
from typing import TYPE_CHECKING, List, Optional, Tuple, cast, final

import cv2
import numpy as np
import reactivex
from cv2.typing import MatLike
from cvzone.PoseModule import PoseDetector
from geometry_msgs.msg import Pose
from PIL import Image
from reactivex import Observable, operators
from reactivex.abc import DisposableBase
from reactivex.scheduler import ThreadPoolScheduler
from reactivex.subject import BehaviorSubject
from typing_extensions import override

from ratmap_common import ExtensionMetadata
from ratmap_core import BaseExtension
from ratmap_core.ui import Mesh
from tkinter_rx import Frame, ImageLabel, Label
from tkinter_rx.util import SetDisposer, safe_dispose

if TYPE_CHECKING:
    from ....builtin_extensions.stereo_vslam import StereoVslam


@final
class Detector(BaseExtension):
    LABEL = "Person Detector"

    def __init__(self) -> None:
        super().__init__()
        self.__extension_window: Optional[tk.Toplevel] = None
        self.thread_pool_scheduler = ThreadPoolScheduler(multiprocessing.cpu_count())
        self.detector = PoseDetector()
        self.stereo_vslam: "StereoVslam"
        self.detection: Optional[Observable[Tuple[Optional[MatLike], bool]]] = None
        self.last_detection = BehaviorSubject(-1)
        self.detection_disposable: Optional[DisposableBase] = None
        self.mesh = -1

        self.legends: Optional[Frame] = None

    @property
    @override
    def metadata(self) -> ExtensionMetadata:
        return {
            "id": "detector",
            "title": "Person Detector",
            "dependency": ["stereo_vslam"],
            "description": "A person detector using cvzones",
        }

    @override
    def start(self) -> None:
        self.ensure_deps()

        self.last_detection.on_next(-1)
        self.context.extension_menu.add_command(
            label=Detector.LABEL, command=self.__open_window
        )

        self.stereo_vslam = cast(
            "StereoVslam", self.extension_manager.get("stereo_vslam")
        )

        self.detection_disposable = SetDisposer()

        if self.stereo_vslam.left_image_subject is not None:
            self.detection = self.stereo_vslam.left_image_subject.pipe(
                operators.throttle_first(0.5),
                operators.observe_on(self.thread_pool_scheduler),
                operators.map(self.__process_image),
            )
            self.detection_disposable.add(
                self.detection.subscribe(self.__add_detection)
            )

        disposer = reactivex.combine_latest(
            self.last_detection, self.stereo_vslam.poses
        ).subscribe(self.__process_map)
        self.detection_disposable.add(disposer)

        self.mesh = self.context.main_gl.new_mesh("triangles")

        self.legends = Frame(self.context.main_window.legends)

        legend = Frame(self.legends)
        canvas = tk.Canvas(legend, width=16, height=16, bg="#FFA500")
        canvas.pack(side=tk.LEFT, expand=tk.NO)
        Label(legend, text="Last person").pack(side=tk.LEFT, expand=tk.NO, padx=2)
        legend.pack(side=tk.TOP, expand=tk.YES, fill=tk.X, padx=4)

        self.legends.pack(side=tk.TOP, expand=tk.YES, fill=tk.X)

        super().start()

    def __add_detection(self, data: Tuple[Optional[MatLike], bool]):
        _img, detected = data

        if detected:
            pose = len(self.stereo_vslam.poses.value) - 1
            self.last_detection.on_next(pose)

    def __process_map(self, data: Tuple[int, List[Pose]]):
        if self.mesh == -1:
            return

        detected, poses = data

        def update(mesh: Mesh):
            if detected < 0 or detected >= len(poses):
                mesh.vertices = np.array([], dtype=np.float32)
                mesh.indices = np.array([], dtype=np.uint32)
                return

            color = (1.0, 0.65, 0.0)

            pose = poses[detected]
            c = (pose.position.x, pose.position.y)

            s = 0.125
            min = (c[0] - s, c[1] - s)
            max = (c[0] + s, c[1] + s)

            v = [
                [min[0], min[1], 0, *color],
                [min[0], max[1], 0, *color],
                [max[0], max[1], 0, *color],
                [max[0], min[1], 0, *color],
            ]
            i = [[0, 1, 2], [0, 2, 3]]

            mesh.vertices = np.array(v, dtype=np.float32)
            mesh.indices = np.array(i, dtype=np.uint32)

        self.context.main_gl.update_mesh(self.mesh, update)

    @override
    def stop(self) -> None:
        if self.legends is not None:
            self.legends.pack_forget()
            self.legends.destroy()
        self.legends = None

        if self.mesh != -1:
            self.context.main_gl.remove_mesh(self.mesh)
        self.mesh = -1

        safe_dispose(self.detection_disposable)
        self.detection_disposable = None

        _ = self.context.extension_menu.remove(Detector.LABEL)
        super().stop()

    def __process_image(
        self, img: Optional[Image.Image]
    ) -> Tuple[Optional[MatLike], bool]:
        if img is None:
            return (None, False)

        try:
            img_array = np.array(img)

            if len(img_array.shape) == 2 or img_array.shape[2] == 1:
                img_array = cv2.cvtColor(img_array, cv2.COLOR_GRAY2BGR)

            img_array = cv2.resize(img_array, (640, 480))

            img_a = self.detector.findPose(
                img_array, draw=False
            )  # Try without drawing first

            lm = []
            if img_a is not None:
                lm, _bbox_inf = self.detector.findPosition(img_a, draw=True)

            return (img_a, len(lm) > 0)

        except Exception as e:
            traceback.print_exc()
            print(f"Error in pose detection: {e}")
            return (None, False)

    def __open_window(self):
        if self.__extension_window is not None:
            self.__extension_window.lift()
            return

        self.__extension_window = tk.Toplevel(self.context.main_window)
        self.__extension_window.title("Object Detector")

        process = ImageLabel(self.__extension_window, width=480, height=360)
        process.pack()

        if self.detection:
            process.image_observable = self.detection.pipe(
                operators.map(lambda v: v[0])
            )

        self.__extension_window.protocol("WM_DELETE_WINDOW", self.__close_window)

    def __close_window(self):
        if self.__extension_window is None:
            return

        self.__extension_window.destroy()
        self.__extension_window = None

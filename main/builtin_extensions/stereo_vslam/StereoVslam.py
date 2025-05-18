import math
import tkinter as tk
import traceback
from concurrent.futures import ThreadPoolExecutor
from threading import Lock
from tkinter import messagebox
from typing import List, Optional, Tuple, Union, cast, final

import numpy as np
import reactivex
from cv2.typing import MatLike
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from PIL.Image import Image
from pyrr.rectangle import width
from reactivex import Observable, Subject, empty, operators
from reactivex.abc import DisposableBase
from reactivex.subject import BehaviorSubject
from rtabmap_msgs.msg import MapGraph
from typing_extensions import override

from ratmap_common import AbstractEvent, ExtensionMetadata
from ratmap_core import BaseExtension
from ratmap_core.ui import MainGl
from ratmap_core.ui.Mesh import Mesh
from tkinter_rx import MenuEvent
from tkinter_rx.Label import LabelEvent
from tkinter_rx.TkinterEvent import TkinterEvent, TkinterEventDetail
from tkinter_rx.util import safe_dispose

from .Calibrator import Calibrator, CalibratorParams
from .CameraInfo import StereoCameraInfo
from .RosBridge import RosBridge
from .ui import StereoVslamToolbar, StereoVslamWindow


class CalibratorParamsHolder:
    chessboard_rows: BehaviorSubject[float]
    chessboard_cols: BehaviorSubject[float]
    square_size: BehaviorSubject[float]

    def __init__(self) -> None:
        self.chessboard_rows = BehaviorSubject(6)
        self.chessboard_cols = BehaviorSubject(7)
        self.square_size = BehaviorSubject(3)

    def getParams(self) -> CalibratorParams:
        return {
            "square_size": self.square_size.value,
            "chessboard_size": (
                int(self.chessboard_cols.value),
                int(self.chessboard_rows.value),
            ),
        }


@final
class StereoVslam(BaseExtension):
    LABEL = "Stereo VSLAM"
    TARGET_FPS: float = 20.0

    left_image_subject: Optional[Subject[Optional[Image]]]
    right_image_subject: Optional[Subject[Optional[Image]]]
    timestamp_subject: Optional[Subject[int]]
    stereo_image_observable: Optional[
        Observable[Tuple[Optional[Image], Optional[Image], int, bool]]
    ]

    __extension_lock: Lock

    __extension_window: Optional[StereoVslamWindow]
    __ros_bridge: Optional[RosBridge]
    __ros_bridge_disposer: Optional[DisposableBase]
    __calibrator: Optional[Calibrator]
    __calibrator_params: CalibratorParamsHolder
    __calibrator_disposer: Optional[DisposableBase]
    __inspect_subject: Subject[Tuple[float, float, float, float]]
    __maingl_click_disposer: Optional[DisposableBase]
    __toolbar: Optional[StereoVslamToolbar]

    __calibration_executor: ThreadPoolExecutor
    __graph_mesh: int
    __plan_mesh: int
    __grid_mesh: int
    __set_target_mesh: int
    __out_target_mesh: int

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
        self.__extension_window = None
        self.__ros_bridge = None
        self.__ros_bridge_disposer = None
        self.__calibrator = None
        self.__calibration_executor = ThreadPoolExecutor(max_workers=1)
        self.__calibrator_disposer = None
        self.__graph_mesh = -1
        self.__plan_mesh = -1
        self.__grid_mesh = -1
        self.__set_target_mesh = -1
        self.__out_target_mesh = -1
        self.__maingl_click_disposer = None
        self.__toolbar = None

        self.__extension_lock = Lock()

        self.left_image_subject = None
        self.right_image_subject = None
        self.timestamp_subject = None
        self.stereo_image_observable = None
        self.is_mapping = BehaviorSubject(False)

        self.__calibrator_params = CalibratorParamsHolder()
        self.__inspect_subject = Subject()

        self.__main_gl: MainGl

        _ = self.add_event_listener(
            "activate.stereo_vslam_menu.file.load_calibration",
            self.__load_calibration_handler,
        )

        _ = self.add_event_listener(
            "activate.stereo_vslam_menu.file.save_calibration",
            self.__save_calibration_handler,
        )

        _ = self.add_event_listener(
            "click.start_calibration", lambda _: self.start_calibration()
        )

        _ = self.add_event_listener("click.pause_calibration", self.pause_calibration)
        _ = self.add_event_listener("click.reset_calibration", self.reset_calibration)
        _ = self.add_event_listener("click.update_calibration", self.update_calibration)
        _ = self.add_event_listener("hover.disparity", self.__disparity_hover_handler)

    @override
    def start(self) -> None:
        super().start()

        self.left_image_subject = Subject()
        self.right_image_subject = Subject()
        self.timestamp_subject = Subject()
        self.is_mapping.on_next(False)

        self.__main_gl = self.context.main_gl

        if self.__grid_mesh != -1:
            self.__main_gl.remove_mesh(self.__grid_mesh)
        self.__grid_mesh = self.__main_gl.new_mesh("triangles")

        if self.__set_target_mesh != -1:
            self.__main_gl.remove_mesh(self.__set_target_mesh)
        self.__set_target_mesh = self.__main_gl.new_mesh("triangles")

        if self.__out_target_mesh != -1:
            self.__main_gl.remove_mesh(self.__out_target_mesh)
        self.__out_target_mesh = self.__main_gl.new_mesh("triangles")

        if self.__graph_mesh != -1:
            self.__main_gl.remove_mesh(self.__graph_mesh)
        self.__graph_mesh = self.__main_gl.new_mesh("lines")

        if self.__plan_mesh != -1:
            self.__main_gl.remove_mesh(self.__plan_mesh)
        self.__plan_mesh = self.__main_gl.new_mesh("lines")

        self.__ros_bridge = RosBridge(
            grid_callback=self.__process_occupancy_grid,
            map_callback=self.__process_map_graph,
            target_callback=self.__process_target,
            plan_callback=self.__process_plan,
        )
        self.__calibrator = Calibrator()

        self.stereo_image_observable = reactivex.combine_latest(
            self.left_image_subject,
            self.right_image_subject,
            self.timestamp_subject,
            self.__calibrator.is_calibrating,
        ).pipe(operators.throttle_first(1 / StereoVslam.TARGET_FPS))

        safe_dispose(self.__calibrator_disposer)
        self.__calibrator_disposer = self.stereo_image_observable.pipe(
            operators.throttle_first(0.8)
        ).subscribe(self.__calibrator.next)

        safe_dispose(self.__ros_bridge_disposer)
        self.__ros_bridge_disposer = reactivex.combine_latest(
            self.stereo_image_observable,
            self.__calibrator.stereo_camera_info,
            self.is_mapping,
        ).subscribe(on_next=self.process_images)

        self.context.extension_menu.add_command(
            label=StereoVslam.LABEL, command=self.__open_window
        )

        self.context.edit_menu.add_command(label="Reset map", command=self.reset_map)

        self.__maingl_click_disposer = self.__main_gl.event_target.add_event_listener(
            "main_gl.double_click", self.__add_target
        )

        self.__toolbar = StereoVslamToolbar(
            self.context.toolbar,
            self.left_image_subject,
            self.right_image_subject,
            self.__ros_bridge.disparity_image_observable,
        )
        self.context.toolbar.add(self.__toolbar, text=StereoVslam.LABEL)

        self.reset_map()

    def process_images(
        self,
        images: Tuple[
            Tuple[Optional[Image], Optional[Image], int, bool],
            Optional[StereoCameraInfo],
            bool,
        ],
    ):
        stereo_images, camera_info, is_mapping = images
        if not is_mapping:
            return

        left_image, right_image, _timestamp, is_calibrating = stereo_images

        if (
            left_image is None
            or right_image is None
            or camera_info is None
            or self.__ros_bridge is None
            or is_calibrating
        ):
            return

        left_camera_info, right_camera_info, _stereo_info = camera_info

        self.__ros_bridge.send_stereo_image(
            left_image,
            right_image,
            left_camera_info,
            right_camera_info,
        )

    @override
    def stop(self) -> None:

        safe_dispose(self.left_image_subject)
        self.left_image_subject = None

        safe_dispose(self.right_image_subject)
        self.right_image_subject = None

        safe_dispose(self.__calibrator_disposer)
        self.__calibrator_disposer = None

        safe_dispose(self.__ros_bridge_disposer)
        self.__ros_bridge_disposer = None

        safe_dispose(self.__maingl_click_disposer)
        self.__maingl_click_disposer = None

        self.is_mapping.on_next(False)

        if self.__ros_bridge:
            self.__ros_bridge.destroy()
            self.__ros_bridge = None

        if self.__graph_mesh != -1:
            self.__main_gl.remove_mesh(self.__graph_mesh)
        self.__graph_mesh = -1

        if self.__plan_mesh != -1:
            self.__main_gl.remove_mesh(self.__plan_mesh)
        self.__plan_mesh = -1

        if self.__grid_mesh != -1:
            self.__main_gl.remove_mesh(self.__grid_mesh)
        self.__grid_mesh = -1

        if self.__set_target_mesh != -1:
            self.__main_gl.remove_mesh(self.__set_target_mesh)
        self.__set_target_mesh = -1

        if self.__out_target_mesh != -1:
            self.__main_gl.remove_mesh(self.__out_target_mesh)
        self.__out_target_mesh = -1

        self.__calibrator = None

        self.__close_window()
        _ = self.context.extension_menu.remove(label=StereoVslam.LABEL)
        _ = self.context.edit_menu.remove(label="Reset map")

        try:
            _ = self.context.toolbar.remove(StereoVslam.LABEL)
            if self.__toolbar is not None:
                self.__toolbar.destroy()
            self.__toolbar = None
        except:
            traceback.print_exc()

        super().stop()

    def __close_window(self):
        if self.__extension_window is None:
            return

        self.__extension_window.destroy()
        self.__extension_window = None

    def __open_window(self):
        if self.__ros_bridge is None or self.__calibrator is None:
            return

        main_window = self.context.main_window

        if self.__extension_window is not None:
            self.__extension_window.lift()
            return

        self.__extension_window = StereoVslamWindow(main_window)
        self.__extension_window.protocol("WM_DELETE_WINDOW", self.__close_window)

        self.__extension_window.minimum_samples = (
            self.__calibrator.MINIMUM_NUMBER_SAMPLE
        )

        def preview_image(
            is_calibrating: bool,
        ) -> Observable[Union[MatLike, Image, None]]:
            if self.__ros_bridge is None or self.__calibrator is None:
                return empty()
            if is_calibrating:
                return self.__calibrator.left_image_with_drawing
            return self.__ros_bridge.disparity_image_observable

        self.__extension_window.disparity_image_observable = (
            self.__calibrator.is_calibrating.pipe(
                operators.map(preview_image), operators.switch_latest()
            )
        )
        self.__extension_window.left_image_observable = self.left_image_subject
        self.__extension_window.right_image_observable = self.right_image_subject

        self.__extension_window.is_calibrating_observable = (
            self.__calibrator.is_calibrating
        )

        self.__extension_window.calibration_count_observable = (
            self.__calibrator.number_of_samples
        )

        self.__extension_window.square_size_subject = (
            self.__calibrator_params.square_size
        )
        self.__extension_window.chessboard_rows_subject = (
            self.__calibrator_params.chessboard_rows
        )
        self.__extension_window.chessboard_cols_subject = (
            self.__calibrator_params.chessboard_cols
        )

        self.__extension_window.inspect_observable = self.__inspect_subject

        self.__extension_window.event_target.parent = self

    def load_calibration(self, filename: str):
        if self.__calibrator is None:
            return False

        return self.__calibrator.load(filename)

    def save_calibration(self, filename: str):
        if self.__calibrator is None:
            return False

        return self.__calibrator.save(filename)

    def __load_calibration_handler(self, event: AbstractEvent):
        if not isinstance(event, MenuEvent) or not isinstance(
            event.detail, TkinterEventDetail
        ):
            return

        filename: str = event.detail.additional
        print(filename)
        _ = self.load_calibration(filename)

    def __save_calibration_handler(self, event: AbstractEvent):
        if not isinstance(event, MenuEvent) or not isinstance(
            event.detail, TkinterEventDetail
        ):
            return

        filename: str = event.detail.additional
        _ = self.save_calibration(filename)

    def pause_calibration(self, _event: AbstractEvent):
        if self.__calibrator is None:
            return

        self.__calibrator.pause()

    def reset_calibration(self, _event: AbstractEvent):
        if self.__calibrator is None:
            return

        self.__calibrator.reset()

    def update_calibration(self, _event: AbstractEvent):
        if self.__calibrator is None:
            return

        response = messagebox.askyesno(
            "Update calibration",
            "This would overwrite previous calibration information. Are you sure you want to proceed?",
        )

        if response:
            _ = self.__calibration_executor.submit(self.__calibrator.update_calibration)

    def __disparity_hover_handler(self, event: AbstractEvent):
        if self.__calibrator is not None and self.__calibrator.is_calibrating.value:
            return

        if self.__ros_bridge is None:
            return

        if not isinstance(event, LabelEvent):
            return

        detail = event.detail

        if not isinstance(detail, tk.Event):
            return

        width = detail.widget.width
        height = detail.widget.height

        result = self.__ros_bridge.inspect(detail.x / width, detail.y / height)
        self.__inspect_subject.on_next(result)

    @property
    def calibrator(self):
        return self.__calibrator

    def start_calibration(self, params: Optional[CalibratorParams] = None):
        if params is not None:
            self.__calibrator_params.chessboard_rows.on_next(
                params["chessboard_size"][0]
            )
            self.__calibrator_params.chessboard_cols.on_next(
                params["chessboard_size"][1]
            )
            self.__calibrator_params.square_size.on_next(params["square_size"])
        if self.__calibrator:
            self.__calibrator.start(self.__calibrator_params.getParams())
        self.is_mapping.on_next(False)

    def reset_map(self):
        if self.__ros_bridge:
            self.__ros_bridge.reset()

        def clear_mesh(mesh: Mesh):
            mesh.vertices = np.array([], dtype=np.float32)
            mesh.indices = np.array([], dtype=np.uint32)

        def clear_path(mesh: Mesh):
            direction_pose = [
                # 1st point
                0,
                0,
                0,
                1.0,
                1.0,
                0.0,
                # 2nd point
                0.25,
                0,
                0,
                1.0,
                1.0,
                0.0,
            ]
            mesh.vertices = np.array(direction_pose, dtype=np.float32)
            mesh.indices = np.array([0, 1], dtype=np.uint32)

        if self.__grid_mesh != -1:
            self.__main_gl.update_mesh(self.__grid_mesh, clear_mesh)

        if self.__graph_mesh != -1:
            self.__main_gl.update_mesh(self.__graph_mesh, clear_path)

        if self.__out_target_mesh != -1:
            self.__main_gl.update_mesh(self.__out_target_mesh, clear_mesh)

        if self.__set_target_mesh != -1:
            self.__main_gl.update_mesh(self.__set_target_mesh, clear_mesh)

        if self.__plan_mesh != -1:
            self.__main_gl.update_mesh(self.__plan_mesh, clear_mesh)

    def __process_occupancy_grid(self, grid_msg: OccupancyGrid):
        if self.__grid_mesh == -1:
            return

        def update(mesh: Mesh):
            width = grid_msg.info.width
            height = grid_msg.info.height
            resolution = grid_msg.info.resolution
            origin_x = grid_msg.info.origin.position.x
            origin_y = grid_msg.info.origin.position.y

            # Reshape the data to 2D
            grid_data = np.array(grid_msg.data, dtype=np.int8).reshape(height, width)

            # Create vertex and index arrays more efficiently
            vertices = []
            indices = []
            vertex_count = 0

            def get_color(occupancy: int):
                if occupancy == 0:
                    return (0.8, 0.8, 0.8)
                elif occupancy == -1:
                    return (0.0, 0.0, 0.0)
                elif occupancy == 100:
                    return (0.8, 0.2, 0.2)
                else:
                    return (0, 0, 0.5)

            # Only process non-unknown cells
            valid_cells = np.where(grid_data >= 0)
            for y, x in zip(valid_cells[0], valid_cells[1]):
                occupancy = grid_data[y, x]

                # Calculate world coordinates
                world_x = origin_x + x * resolution
                world_y = origin_y + y * resolution

                # Get color
                r, g, b = get_color(occupancy)

                # Add vertices for this cell
                vertices.extend(
                    [
                        [world_x, world_y, -50, r, g, b],
                        [world_x + resolution, world_y, -50, r, g, b],
                        [world_x + resolution, world_y + resolution, -50, r, g, b],
                        [world_x, world_y + resolution, -50, r, g, b],
                    ]
                )

                # Add indices (two triangles per quad)
                indices.extend(
                    [
                        vertex_count,
                        vertex_count + 1,
                        vertex_count + 2,
                        vertex_count,
                        vertex_count + 2,
                        vertex_count + 3,
                    ]
                )

                vertex_count += 4

            # Convert to numpy arrays for better performance
            mesh.vertices = np.array(vertices, dtype=np.float32)
            mesh.indices = np.array(indices, dtype=np.uint32)

        self.__main_gl.update_mesh(self.__grid_mesh, update)

    def __process_map_graph(self, graph: MapGraph):
        if self.__graph_mesh == -1:
            return

        def draw_graph(mesh: Mesh):
            vertices = np.array(
                [
                    [p.position.x, p.position.y, p.position.z, 0.0, 0.0, 1.0]
                    for p in graph.poses
                ],
                dtype=np.float32,
            )

            if len(graph.poses) > 0:
                last_pose = graph.poses[-1]
                qx = last_pose.orientation.x
                qy = last_pose.orientation.y
                qz = last_pose.orientation.z
                qw = last_pose.orientation.w

                # Calculate forward vector (x-axis direction in local frame)
                forward_x = 1 - 2 * (qy * qy + qz * qz)
                forward_y = 2 * (qx * qy + qw * qz)
                forward_z = 2 * (qx * qz - qw * qy)

                # Project onto x-y plane by zeroing z component
                forward_z = 0

                # Recalculate magnitude after projection
                magnitude = np.sqrt(forward_x * forward_x + forward_y * forward_y)

                # Avoid division by zero
                if magnitude > 0.0001:
                    scale = 0.25
                    # Create a point in the direction the user is facing along x-y plane
                    direction_pose = [
                        [
                            last_pose.position.x,
                            last_pose.position.y,
                            last_pose.position.z,
                            1.0,
                            1.0,
                            0.0,
                        ],
                        [
                            last_pose.position.x + (forward_x / magnitude) * scale,
                            last_pose.position.y + (forward_y / magnitude) * scale,
                            last_pose.position.z,  # Keep same z to stay in x-y plane
                            1.0,
                            1.0,
                            0.0,
                        ],
                    ]
                    vertices = np.vstack((vertices, direction_pose), dtype=np.float32)

            mesh.vertices = vertices
            N = vertices.shape[0]
            mesh.indices = np.column_stack(
                (np.arange(N - 1, dtype=np.uint32), np.arange(1, N, dtype=np.uint32))
            )

        self.__main_gl.update_mesh(self.__graph_mesh, draw_graph)

    @property
    def calibrator_params(self):
        return self.__calibrator_params

    def start_mapping(self) -> None:
        self.reset_map()
        if self.calibrator is not None:
            self.calibrator.stop()
        self.is_mapping.on_next(True)

    @property
    def extension_lock(self):
        return self.__extension_lock

    def __add_target(self, event: AbstractEvent):
        if self.__set_target_mesh == -1:
            return

        if not isinstance(event, TkinterEvent) or self.__ros_bridge is None:
            return

        detail = event.detail
        target = event.target

        if not isinstance(detail, tk.Event) or not isinstance(target, MainGl):
            return

        x = detail.x / target.winfo_width()
        y = detail.y / target.winfo_height()

        res = self.__main_gl.camera.get_world_position((x, y))

        def update_target(mesh: Mesh):
            color = [0.2, 0.8, 0.8]
            vertices: List[List[float]] = []
            vertices.append([res[0], res[1], 0.0, *color])

            segments = 8
            r = 0.1

            # Add perimeter vertices
            for i in range(segments):
                angle = 2.0 * math.pi * i / segments
                vx = res[0] + r * math.cos(angle)
                vy = res[1] + r * math.sin(angle)
                vertices.append([vx, vy, 0.0, *color])

            # Create indices for triangle fan
            indices: List[int] = []
            for i in range(1, segments + 1):
                indices.append(0)  # Center vertex
                indices.append(i)  # Current perimeter vertex
                indices.append(1 if i == segments else i + 1)  # Next perimeter vertex

            mesh.vertices = np.array(vertices, dtype=np.float32)
            mesh.indices = np.array(indices, dtype=np.uint32)

        self.__main_gl.update_mesh(self.__set_target_mesh, update_target)
        self.__ros_bridge.set_target(res[0], res[1], 0)

    def __process_target(self, pose: PoseStamped):
        if self.__out_target_mesh == -1:
            return

        res = (pose.pose.position.x, pose.pose.position.y)

        def update_target(mesh: Mesh):
            color = [0.2, 0.8, 0.2]
            vertices: List[List[float]] = []
            vertices.append([res[0], res[1], 0.0, *color])

            segments = 8
            r = 0.15

            # Add perimeter vertices
            for i in range(segments):
                angle = 2.0 * math.pi * i / segments
                vx = res[0] + r * math.cos(angle)
                vy = res[1] + r * math.sin(angle)
                vertices.append([vx, vy, 0.2, *color])

            # Create indices for triangle fan
            indices: List[int] = []
            for i in range(1, segments + 1):
                indices.append(0)  # Center vertex
                indices.append(i)  # Current perimeter vertex
                indices.append(1 if i == segments else i + 1)  # Next perimeter vertex

            mesh.vertices = np.array(vertices, dtype=np.float32)
            mesh.indices = np.array(indices, dtype=np.uint32)

        self.__main_gl.update_mesh(self.__out_target_mesh, update_target)

    def __process_plan(self, plan: Path):
        if self.__plan_mesh == -1:
            return

        def draw_plan(mesh: Mesh):
            if plan.poses is None:
                mesh.vertices = np.array([], dtype=np.float32)
                mesh.indices = np.array([], dtype=np.uint32)
                return

            vertices = np.array(
                [
                    [
                        p.pose.position.x,
                        p.pose.position.y,
                        p.pose.position.z,
                        0.0,
                        1.0,
                        0.0,
                    ]
                    for p in plan.poses
                ],
                dtype=np.float32,
            )

            mesh.vertices = vertices
            N = vertices.shape[0]
            mesh.indices = np.column_stack(
                (np.arange(N - 1, dtype=np.uint32), np.arange(1, N, dtype=np.uint32))
            )

        self.__main_gl.update_mesh(self.__plan_mesh, draw_plan)

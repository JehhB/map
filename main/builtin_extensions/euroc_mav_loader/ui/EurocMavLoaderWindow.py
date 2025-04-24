import tkinter as tk
from tkinter import filedialog
from typing import Optional

from reactivex import Observable
from reactivex.subject import BehaviorSubject
from typing_extensions import override

from ratmap_common.EventTarget import EventTarget
from tkinter_rx import Button, Entry, Label, Scale
from tkinter_rx.ImageLabel import ImageLabel

from ..Player import PlayerState


class EurocMavLoaderWindow(tk.Toplevel):
    DEFAULT_VIEWER_WIDTH: int = 480
    DEFAULT_VIEWER_HEIGHT: int = 360

    left_image: ImageLabel
    right_image: ImageLabel
    folder_path: BehaviorSubject[str]

    def __init__(
        self,
        master: tk.Misc,
    ) -> None:
        super().__init__(master)

        self.title("EuRoC MAV Loader")

        self.folder_path = BehaviorSubject("")

        self.__event_target = EventTarget()
        self.__player_state: Optional[Observable[PlayerState]] = None

        file_selection_frame = tk.Frame(self)

        Label(file_selection_frame, text="Dataset Location").pack(side=tk.LEFT)

        folder_entry: Entry = Entry(file_selection_frame, textsubject=self.folder_path)
        folder_entry.pack(side=tk.LEFT, fill=tk.X, expand=tk.YES, padx=4)

        Button(file_selection_frame, text="Choose", command=self._choose_path).pack(
            side=tk.LEFT, padx=4
        )

        load_dataset_button = Button(
            file_selection_frame,
            text="Load",
            clickevent="euroc_mav_loader.load_dataset",
        )
        load_dataset_button.event_target.parent = self.__event_target
        load_dataset_button.pack(side=tk.LEFT, padx=4)

        file_selection_frame.pack(padx=8, pady=8, expand=tk.YES, fill=tk.X)

        preview_frame = tk.Frame(self)

        self.left_image = ImageLabel(
            preview_frame,
            width=EurocMavLoaderWindow.DEFAULT_VIEWER_WIDTH,
            height=EurocMavLoaderWindow.DEFAULT_VIEWER_HEIGHT,
            bg="black",
        )
        self.left_image.grid(row=0, column=0, padx=4, pady=4)

        self.right_image = ImageLabel(
            preview_frame,
            width=EurocMavLoaderWindow.DEFAULT_VIEWER_WIDTH,
            height=EurocMavLoaderWindow.DEFAULT_VIEWER_HEIGHT,
            bg="black",
        )
        self.right_image.grid(row=0, column=1, padx=4, pady=4)

        _ = preview_frame.rowconfigure(0, weight=1)
        _ = preview_frame.columnconfigure((0, 1), weight=1)
        preview_frame.pack(padx=8, pady=8, expand=tk.YES, fill=tk.BOTH)

        control_frame = tk.Frame(self)

        Label(control_frame, text="Speed").pack(side=tk.LEFT)
        fps_scale = Scale(
            control_frame,
            from_=5,
            to=40,
            value=30,
            orient=tk.HORIZONTAL,
            changeevent="euroc_mav_loader.set_fps",
        )
        fps_scale.event_target.parent = self.__event_target
        fps_scale.pack(side=tk.LEFT, expand=tk.YES, fill=tk.X, padx=4)

        self.__start_map_button = Button(
            control_frame,
            text="Start Mapping",
            clickevent="euroc_mav_loader.start_mapping",
        )
        self.__start_map_button.event_target.parent = self.__event_target
        self.__start_map_button.pack(side=tk.LEFT, padx=4)

        self.__start_calib_button = Button(
            control_frame,
            text="Start Calibration",
            clickevent="euroc_mav_loader.start_calibration",
        )
        self.__start_calib_button.event_target.parent = self.__event_target
        self.__start_calib_button.pack(side=tk.LEFT, padx=4)

        self.__pause_button = Button(
            control_frame, text="Pause", clickevent="euroc_mav_loader.pause"
        )
        self.__pause_button.event_target.parent = self.__event_target
        self.__pause_button.pack(side=tk.LEFT, padx=4)

        self.__step_button = Button(
            control_frame, text="Step", clickevent="euroc_mav_loader.step"
        )
        self.__step_button.event_target.parent = self.__event_target
        self.__step_button.pack(side=tk.LEFT, padx=4)

        self.__play_button = Button(
            control_frame, text="Play", clickevent="euroc_mav_loader.play"
        )
        self.__play_button.event_target.parent = self.__event_target
        self.__play_button.pack(side=tk.LEFT, padx=4)

        self.__stop_button = Button(
            control_frame, text="Stop", clickevent="euroc_mav_loader.stop"
        )
        self.__stop_button.event_target.parent = self.__event_target
        self.__stop_button.pack(side=tk.LEFT, padx=4)

        control_frame.pack(padx=8, pady=8, expand=tk.YES, fill=tk.X)

    def _choose_path(self):
        dir = filedialog.askdirectory(
            parent=self,
            mustexist=True,
            title="Choose EuRoC Mav Dataset directory",
            initialdir=self.folder_path.value,
        )
        if isinstance(dir, str):
            self.folder_path.on_next(dir)

    def update_player_state(self, state: Optional[PlayerState] = None):
        def set_state(button: Button, *states: Optional[PlayerState]):
            _ = button.config(state="normal" if state in states else "disabled")

        set_state(self.__start_map_button, "idle")
        set_state(self.__start_calib_button, "idle")
        set_state(self.__pause_button, "playing")
        set_state(self.__step_button, "playing", "paused")
        set_state(self.__play_button, "paused")
        set_state(self.__stop_button, "paused", "playing")

    @property
    def event_target(self):
        return self.__event_target

    @override
    def destroy(self) -> None:
        self.__event_target.dispose()
        return super().destroy()

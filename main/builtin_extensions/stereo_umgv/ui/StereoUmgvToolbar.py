import tkinter as tk

from reactivex import operators
from reactivex.subject import BehaviorSubject

from ratmap_common import EventTarget
from tkinter_rx import Button, ImageLabel, ImageObservable, Label, LabelFrame, Scale

from .Joystick import Joystick


class StereoUmgvToolbar(tk.Frame):

    def __init__(
        self,
        master: tk.Misc,
        left_image_observable: ImageObservable,
        right_image_observable: ImageObservable,
        x_subject: BehaviorSubject[float],
        y_subject: BehaviorSubject[float],
        flash_subject: BehaviorSubject[float],
    ) -> None:
        super().__init__(master)
        self.__event_target = EventTarget()

        left_frame = LabelFrame(self, text="Left image")
        self.__left_image = ImageLabel(
            left_frame, width=280, height=210, imageobservable=left_image_observable
        )
        self.__left_image.pack(expand=tk.NO)
        left_frame.grid(
            row=0, column=0, sticky="ew", ipadx=4, ipady=4, pady=4, padx=2, columnspan=2
        )

        right_frame = LabelFrame(self, text="Right image")
        self.__right_image = ImageLabel(
            right_frame, width=280, height=210, imageobservable=right_image_observable
        )
        self.__right_image.pack(expand=tk.NO)
        right_frame.grid(
            row=1, column=0, sticky="ew", ipadx=4, ipady=4, pady=4, padx=2, columnspan=2
        )

        flash_observable = flash_subject.pipe(
            operators.map(lambda v: "Flash (%0.2f)" % v)
        )
        flashlight_frame = tk.LabelFrame(
            self, labelwidget=Label(self, textobservable=flash_observable)
        )

        Scale(flashlight_frame, valuesubject=flash_subject).pack(
            expand=tk.YES, fill=tk.X
        )

        flashlight_frame.grid(
            row=2, column=0, sticky="ew", ipadx=4, ipady=4, pady=4, padx=2, columnspan=2
        )

        joystick_frame = tk.Frame(self)

        joystick = Joystick(joystick_frame, x_subject, y_subject, 150)
        joystick.grid(row=0, column=0, padx=4, pady=4)

        _ = joystick_frame.columnconfigure(0, weight=1)
        joystick_frame.grid(row=3, column=0, sticky="ew", pady=24, columnspan=2)

        connect = Button(self, text="Connect", clickevent="stereo_umgv.connect")
        connect.event_target.parent = self.event_target
        connect.grid(row=4, column=0, sticky="ew", pady=4, padx=2)

        start_mapping = Button(
            self, text="Start Mapping", clickevent="stereo_umgv.start_mapping"
        )
        start_mapping.event_target.parent = self.event_target
        start_mapping.grid(row=4, column=1, sticky="ew", pady=4, padx=2)

        _ = self.columnconfigure((0, 1), weight=1)

    @property
    def event_target(self):
        return self.__event_target

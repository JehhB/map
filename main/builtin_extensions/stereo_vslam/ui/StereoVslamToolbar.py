import tkinter as tk

from tkinter_rx import ImageLabel, ImageObservable, LabelFrame


class StereoVslamToolbar(tk.Frame):
    def __init__(
        self,
        master: tk.Misc,
        left_image_observable: ImageObservable,
        right_image_observable: ImageObservable,
        disparity_image_observable: ImageObservable,
    ) -> None:
        super().__init__(master)

        left_frame = LabelFrame(self, text="Left image")
        self.__left_image = ImageLabel(
            left_frame, width=280, height=210, imageobservable=left_image_observable
        )
        self.__left_image.pack(expand=tk.FALSE)
        left_frame.pack(side=tk.TOP, ipadx=4, ipady=4, pady=4, padx=2)

        right_frame = LabelFrame(self, text="Right image")
        self.__right_image = ImageLabel(
            right_frame, width=280, height=210, imageobservable=right_image_observable
        )
        self.__right_image.pack(expand=tk.FALSE)
        right_frame.pack(side=tk.TOP, ipadx=4, ipady=4, pady=4, padx=2)

        disparity_frame = LabelFrame(self, text="Disparity image")
        self.__disparity_image = ImageLabel(
            disparity_frame,
            width=280,
            height=210,
            imageobservable=disparity_image_observable,
        )
        self.__disparity_image.pack(expand=tk.FALSE)
        disparity_frame.pack(side=tk.TOP, ipadx=4, ipady=4, pady=4, padx=2)

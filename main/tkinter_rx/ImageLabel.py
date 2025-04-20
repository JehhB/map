import tkinter as tk
from typing import Optional, Union, cast

import cv2
from cv2.typing import MatLike
from PIL import Image, ImageTk
from reactivex import Observable
from reactivex.abc import DisposableBase
from typing_extensions import Unpack, override

from ratmap_common import EventTarget

from .typing import BaseImageLabelKwargs, ImageLabelKwargs
from .util import safe_callback


class ImageLabel(tk.Label):
    width: int
    height: int
    photo: Optional[ImageTk.PhotoImage]

    __image_observable: Optional[Observable[Union[Image.Image, MatLike, str, None]]]
    __disposer: Optional[DisposableBase]

    __event_target: EventTarget

    def __init__(
        self,
        master: Optional[tk.Misc] = None,
        **kwargs: Unpack[ImageLabelKwargs],
    ):
        self.__image_observable = None
        self.__disposer = None
        self.photo = None

        self.width = kwargs.pop("width", 100)
        self.height = kwargs.pop("height", 100)
        self.image_observable = kwargs.pop("imageobservable", None)

        super().__init__(master, **cast(BaseImageLabelKwargs, kwargs))
        self.__set_blank_image()
        self.__event_target = EventTarget()

    @property
    def image_observable(self):
        return self.__image_observable

    @image_observable.setter
    def image_observable(
        self,
        image_observable: Optional[
            Observable[Union[Image.Image, MatLike, str, None]]
        ] = None,
    ):
        del self.image_observable

        self.__image_observable = image_observable
        if image_observable is not None:
            self.__disposer = image_observable.subscribe(
                on_next=safe_callback(self.master, self.update_image)
            )

    @image_observable.deleter
    def image_observable(self):
        if self.__disposer is not None:
            self.__disposer.dispose()
            self.__disposer = None
        self.__image_observable = None

    def __set_blank_image(self):
        blank = Image.new("RGB", (self.width, self.height), color="black")
        self.photo = ImageTk.PhotoImage(blank)
        _ = self.configure(image=self.photo, width=self.width, height=self.height)

    def __resize_image(self, img: Image.Image):
        return img.resize((self.width, self.height))

    def update_image(self, image: Union[Image.Image, MatLike, str, None]):
        if image is None:
            self.__set_blank_image()
        elif isinstance(image, Image.Image):
            self.update_from_pil(image)
        elif isinstance(image, str):
            self.update_from_file(image)
        else:
            self.update_from_cv2(image)

    def update_from_cv2(self, cv_img: Optional[MatLike]):
        """
        Update the label with an OpenCV image (numpy array).

        Args:
            cv_img: OpenCV image (numpy array) in BGR format
        """
        if cv_img is None:
            self.__set_blank_image()
            return

        # Convert BGR to RGB
        if len(cv_img.shape) == 3 and cv_img.shape[2] == 3:
            rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        else:
            # Handle grayscale images
            rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2RGB)

        # Convert to PIL Image
        pil_img = Image.fromarray(rgb_img)

        # Update the label
        self.update_from_pil(pil_img)

    def update_from_pil(self, pil_img: Optional[Image.Image]):
        """
        Update the label with a PIL/Pillow image.

        Args:
            pil_img: PIL/Pillow Image object
        """
        if pil_img is None:
            self.__set_blank_image()
            return

        # Resize the image to fit the label
        resized_img = self.__resize_image(pil_img)

        # Update the PhotoImage
        self.photo = ImageTk.PhotoImage(resized_img)
        _ = self.configure(image=self.photo)

    def update_from_file(self, file_path: str):
        """
        Update the label with an image loaded from a file.

        Args:
            file_path: Path to the image file
        """
        try:
            pil_img = Image.open(file_path)
            self.update_from_pil(pil_img)
        except Exception as e:
            print(f"Error loading image from file: {e}")
            self.__set_blank_image()

    def clear(self):
        """Reset the label to display a blank image."""
        self.__set_blank_image()

    @override
    def destroy(self) -> None:
        del self.image_observable

        self.__event_target.dispose()
        return super().destroy()

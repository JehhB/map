import tkinter as tk
from typing import Optional

import cv2
from cv2.typing import MatLike
from PIL import Image, ImageTk


class ImageLabel(tk.Label):
    """
    A custom Tkinter Label that can display and update images from different formats:
    - OpenCV (cv2) images (numpy arrays)
    - PIL/Pillow images
    - File paths

    The label maintains the specified dimensions for all displayed images.
    """

    width: int
    height: int
    photo: Optional[ImageTk.PhotoImage]

    def __init__(
        self, master: Optional[tk.Misc] = None, width=100, height=100, **kwargs
    ):
        """
        Initialize the ImageLabel with specified dimensions.

        Args:
            master: The parent widget
            width: Width of the image display area
            height: Height of the image display area
            **kwargs: Additional arguments to pass to the Label constructor
        """
        super().__init__(master, **kwargs)
        self.width = width
        self.height = height
        self.photo = None

        # Set initial blank image
        self._set_blank_image()

    def _set_blank_image(self):
        blank = Image.new("RGB", (self.width, self.height), color="black")
        self.photo = ImageTk.PhotoImage(blank)
        self.configure(image=self.photo, width=self.width, height=self.height)

    def _resize_image(self, img: Image.Image):
        return img.resize((self.width, self.height))

    def update_from_cv2(self, cv_img: MatLike):
        """
        Update the label with an OpenCV image (numpy array).

        Args:
            cv_img: OpenCV image (numpy array) in BGR format
        """
        if cv_img is None:
            self._set_blank_image()
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
            self._set_blank_image()
            return

        # Resize the image to fit the label
        resized_img = self._resize_image(pil_img)

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
            self._set_blank_image()

    def clear(self):
        """Reset the label to display a blank image."""
        self._set_blank_image()

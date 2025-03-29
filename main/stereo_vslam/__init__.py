from threading import Lock
from typing import Optional, Tuple

import reactivex as rx
from PIL.Image import Image
from reactivex.abc import DisposableBase
from reactivex.operators import throttle_first
from reactivex.subject import BehaviorSubject
from typing_extensions import Self, override

from app.AbstractExtension import AbstractExtension, ExtensionMetadata
from app.Container import AbstractModule, ModuleDefinition
from app.events.AbstractEvent import AbstractEvent
from app.ExtensionManager import ExtensionManager
from app.ui.Main import Main as AppMain
from app.ui.MenuBar import MenuBar
from stereo_vslam.RosBridge import RosBridge
from stereo_vslam.ui.Main import Main


class StereoVslamExtension(AbstractModule, AbstractExtension):
    EXTENSION_LABEL: str = "Stereo VSLAM"
    TARGET_FPS: float = 10.0

    menu_bar: MenuBar
    main_window: Optional[Main]
    _ros_bridge: Optional[RosBridge]

    left_image_subject: rx.Subject[Optional[Image]]
    right_image_subject: rx.Subject[Optional[Image]]
    stereo_image_observable: rx.Observable[Tuple[Optional[Image], Optional[Image]]]
    _stereo_image_disposer: Optional[DisposableBase]

    lock: Lock

    @override
    @staticmethod
    def KEY() -> str:
        return "stereo_vslam.main"

    @override
    @classmethod
    def DEFINITION(cls) -> ModuleDefinition[Self]:
        return (
            ExtensionManager.defaultFactory("stereo-vslam", cls),
            [ExtensionManager, MenuBar, AppMain],
        )

    def __init__(self):
        super().__init__()

        self.main_window = None
        self._ros_bridge = None
        self.lock = Lock()
        self._stereo_image_disposer = None

        self.left_image_subject = BehaviorSubject(None)
        self.right_image_subject = BehaviorSubject(None)
        self.stereo_image_observable = rx.combine_latest(
            self.left_image_subject, self.right_image_subject
        ).pipe(throttle_first(1 / StereoVslamExtension.TARGET_FPS))

        self.add_event_handler("init", self.on_init)
        self.add_event_handler("deinit", self.on_deinit)

    def on_init(self, event: AbstractEvent):
        if self.container is None:
            event.is_success = False
            return

        try:
            self._ros_bridge = RosBridge()
        except RuntimeError as e:
            event.detail = e
            event.is_success = False
            return

        self._stereo_image_disposer = self.stereo_image_observable.subscribe(
            on_next=self.process_images
        )

        self.menu_bar = self.container[MenuBar]
        self.menu_bar.extension_menu.add_command(
            label=StereoVslamExtension.EXTENSION_LABEL, command=self.open_window
        )

    def on_deinit(self, _: AbstractEvent):
        if self._ros_bridge is not None:
            self._ros_bridge.destroy()
            self._ros_bridge = None

        if self._stereo_image_disposer is not None:
            self._stereo_image_disposer.dispose()

        menu = self.menu_bar.extension_menu
        for i in range(menu.index("end") + 1):
            try:
                if menu.entrycget(i, "label") == StereoVslamExtension.EXTENSION_LABEL:
                    menu.delete(i)
                    break
            except:
                pass

    def open_window(self):
        if self.container is None:
            return

        if self.main_window is not None:
            self.main_window.lift()
            return

        self.main_window = Main(
            self,
            self.container[AppMain],
        )

        def on_close():
            if self.main_window is None:
                return

            self.main_window.destroy()
            self.main_window = None

        self.main_window.protocol("WM_DELETE_WINDOW", on_close)

    def process_images(self, stereo_images: Tuple[Optional[Image], Optional[Image]]):
        left_image, right_image = stereo_images
        if left_image is None or right_image is None or self._ros_bridge is None:
            return

        self._ros_bridge.send_stereo_image(left_image, right_image)

    @property
    def metadata(self) -> ExtensionMetadata:
        return ExtensionMetadata(
            "Stereo VSLAM",
            "A Stereo VSLAM extension to calibrate and map stereo vision system",
            [],
        )

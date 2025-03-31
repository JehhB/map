from typing import Optional, Tuple

from cv2.typing import MatLike
from PIL.Image import Image, fromarray
from reactivex import Subject, operators
from reactivex.abc import DisposableBase
from typing_extensions import override

from app.AbstractExtension import AbstractExtension, ExtensionMetadata
from app.Container import AbstractModule, ModuleDefinition
from app.events.AbstractEvent import AbstractEvent
from app.ExtensionManager import ExtensionManager
from app.ui.Main import Main as AppMain
from app.ui.MenuBar import MenuBar
from euroc_mav_loader.DatasetLoader import DatasetLoader, Metadata
from euroc_mav_loader.Player import Player
from euroc_mav_loader.ui.Main import Main
from stereo_vslam import StereoVslamExtension


class EuRoCMAVLoaderExtension(AbstractModule, AbstractExtension):
    EXTENSION_LABEL = "EuRoC MAV Loader"

    menu_bar: MenuBar
    stereo_vslam: StereoVslamExtension
    main_window: Optional[Main]
    dataset_loader: Optional[DatasetLoader]
    player: Optional[Player[Tuple[MatLike, MatLike, Metadata]]]
    relay_dispose: Optional[DisposableBase]

    @override
    @staticmethod
    def KEY() -> str:
        return "euroc_mac_loader.main"

    @override
    @classmethod
    def DEFINITION(cls) -> ModuleDefinition["EuRoCMAVLoaderExtension"]:
        return ExtensionManager.defaultFactory("euroc_mac_loader", cls), [
            ExtensionManager,
            MenuBar,
            AppMain,
            StereoVslamExtension,
        ]

    def __init__(self) -> None:
        super().__init__()

        self.add_event_handler("init", self.on_init)
        self.add_event_handler("deinit", self.on_deinit)
        self.main_window = None

        self.player = None
        self.relay_dispose = None

    def on_init(self, event: AbstractEvent):
        if self.container is None:
            event.is_success = False
            event.detail = RuntimeError("Container is not set")
            return

        self.stereo_vslam = self.container[StereoVslamExtension]
        if not self.stereo_vslam.is_active:
            event.is_success = False
            event.detail = RuntimeError("Stereo VSLAM Extension is not active")
            return

        self.menu_bar = self.container[MenuBar]
        self.menu_bar.extension_menu.add_command(
            label=EuRoCMAVLoaderExtension.EXTENSION_LABEL, command=self.open_window
        )

    def on_deinit(self, _event: AbstractEvent):

        if self.player is not None:
            self.player.dispose()
            self.player = None

        if self.relay_dispose is not None:
            self.relay_dispose.dispose()
            self.relay_dispose = None

        self.dataset_loader = None

        try:
            _ = self.menu_bar.extension_menu.remove(
                EuRoCMAVLoaderExtension.EXTENSION_LABEL
            )
        except:
            pass

    def load_dataset(self, path: str):
        self.dataset_loader = DatasetLoader(path)

        images = iter(self.dataset_loader)

        transformer_subject: Subject[Tuple[MatLike, MatLike, Metadata]] = Subject()

        def transform(inp: Tuple[MatLike, MatLike, Metadata]):
            left, right, meta = inp
            return (
                fromarray(left),
                fromarray(right),
                meta["timestamp"],
            )

        relay_subject = transformer_subject.pipe(operators.map(transform))

        def relay(inp: Tuple[Optional[Image], Optional[Image], int]):
            self.stereo_vslam.left_image_subject.on_next(inp[0])
            self.stereo_vslam.right_image_subject.on_next(inp[1])
            self.stereo_vslam.timestamp_subject.on_next(inp[2])

        self.relay_dispose = relay_subject.subscribe(on_next=relay)

        self.player = Player(images, transformer_subject)

    def open_window(self):
        if self.container is None:
            return

        if self.main_window is not None:
            self.main_window.lift()
            return

        self.main_window = Main(
            self.container,
            self.container[AppMain],
        )

        def on_close():
            if self.main_window is None:
                return

            self.main_window.destroy()
            self.main_window = None

        self.main_window.protocol("WM_DELETE_WINDOW", on_close)

    @property
    @override
    def metadata(self) -> ExtensionMetadata:
        return ExtensionMetadata(
            "EuRoc MAV Loader",
            "An extension to load EuRoC MAV Dataset to be used for Stereo VSLAM",
            ["Stereo VSLAM"],
        )

from typing import Dict, Type, TypeVar

from typing_extensions import Self, override

from app.AbstractExtension import AbstractExtension
from app.Container import (
    AbstractModule,
    Container,
    ModuleDefinition,
    ModuleFactory,
    SetterInjectable,
)
from app.events.AbstractEvent import AbstractEvent


class InitEvent(AbstractEvent):
    pass


class DeinitEvent(AbstractEvent):
    pass


AbstractExtensionImpl = TypeVar("AbstractExtensionImpl", bound="AbstractExtension")


class ExtensionManager(AbstractModule, SetterInjectable):
    @staticmethod
    def defaultFactory(
        name: str, constructor: Type[AbstractExtensionImpl]
    ) -> ModuleFactory[AbstractExtensionImpl]:
        def factory(container: Container) -> AbstractExtensionImpl:
            extension = constructor()
            extension.set_container(container)

            extensionManager = container[ExtensionManager]
            extensionManager.add_extension(name, extension)

            return extension

        return factory

    _extensions: Dict[str, AbstractExtension]

    @override
    @staticmethod
    def KEY() -> str:
        return "main.extension_manager"

    @override
    @classmethod
    def DEFINITION(cls) -> ModuleDefinition[Self]:
        return Container.defaultSetterFactory(cls), None

    def __init__(self) -> None:
        super().__init__()
        self._extensions = dict()

    def enable_extension(self, key: str) -> bool:
        if key not in self._extensions:
            return False
        extension = self._extensions[key]

        event = InitEvent()
        extension.emit_event("init", event)

        if event.is_success:
            extension.active_subject.on_next(True)

        return event.is_success

    def disable_event(self, key: str) -> bool:
        if key not in self._extensions:
            return False

        extension = self._extensions[key]

        event = DeinitEvent()
        extension.emit_event("deinit", event)

        if event.is_success:
            extension.active_subject.on_next(False)

        return event.is_success

    def add_extension(self, key: str, extension: AbstractExtension) -> None:
        self._extensions[key] = extension

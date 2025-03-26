from typing import Self, override

from app.AbstractEvent import AbstractEvent
from app.AbstractExtension import AbstractExtension
from app.Container import (
    AbstractModule,
    Container,
    ModuleDefinition,
    ModuleFactory,
    SetterInjectable,
)


class InitEvent(AbstractEvent):
    pass


class ExtensionManager(AbstractModule, SetterInjectable):
    @staticmethod
    def defaultFactory[
        T: AbstractExtension
    ](name: str, constructor: type[T]) -> ModuleFactory[T]:
        def factory(container: Container) -> T:
            extension = constructor()
            extension.set_container(container)

            extensionManager = container[ExtensionManager]
            extensionManager.add_extension(name, extension)

            return extension

        return factory

    _extensions: dict[str, AbstractExtension]

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

    def add_extension(self, key: str, extension: AbstractExtension) -> None:
        self._extensions[key] = extension
        extension.emit_event("init", InitEvent())

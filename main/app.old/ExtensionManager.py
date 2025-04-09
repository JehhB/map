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

    def add_extension(self, key: str, extension: AbstractExtension) -> None:
        self._extensions[key] = extension

    def items(self):
        return self._extensions.items()

    def values(self):
        return self._extensions.values()

    def dispose(self):
        for _k, x in self._extensions.items():
            _ = x.disable()

from abc import ABC
from typing import List, Optional

from reactivex import operators
from reactivex.abc import DisposableBase
from typing_extensions import override

from ratmap_common import AbstractEvent, AbstractExtension
from ratmap_core.ExtensionManager import ExtensionManager

from .Application import Application


class BaseExtension(AbstractExtension, ABC):
    __context: Application

    __extension_manager: ExtensionManager
    __dependency_watch_disposer: Optional[DisposableBase]

    def __init__(self) -> None:
        super().__init__()
        self.__dependency_watch_disposer = None

    @property
    def context(self):
        return self.__context

    @context.setter
    def context(self, context: Application):
        self.__context = context

    @property
    def extension_manager(self):
        return self.__extension_manager

    @extension_manager.setter
    def extension_manager(self, extension_manager: ExtensionManager):
        self.__extension_manager = extension_manager

    def ensure_deps(self):
        deps = self.metadata.get("dependency", [])

        for dep in deps:
            extension = self.extension_manager.get(dep)
            if not extension.is_started:
                raise RuntimeError(f'Extension with id "{dep}" has not started yet')

        self.__watch_dependency(deps)

    @override
    def start(self) -> None:
        super().start()

        self.context.config.update(f"{self.config_namespace}.enabled", True)

    @override
    def stop(self) -> None:
        super().stop()

        if self.__dependency_watch_disposer is not None:
            self.__dependency_watch_disposer.dispose()

        self.context.config.update(f"{self.config_namespace}.enabled", False)

    def __watch_dependency(self, deps: List[str]):
        if self.__dependency_watch_disposer is not None:
            self.__dependency_watch_disposer.dispose()

        deps_stop_event = map(lambda id: "extension.stop." + id, deps)

        def close_dependency_handler(e: AbstractEvent):
            if e.type in deps_stop_event:
                self.stop()

        self.__dependency_watch_disposer = self.__extension_manager.add_event_listener(
            "extension.stop", close_dependency_handler
        )

    @property
    def config_namespace(self):
        id = self.metadata.get("id")
        return f"extensions.{id}"

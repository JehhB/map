from abc import ABC
from typing import Optional

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
        self.__watch_dependency()

    @override
    def start(self) -> None:
        deps = self.metadata.get("dependency", [])

        for dep in deps:
            extension = self.extension_manager.get(dep)
            if not extension.is_started:
                raise RuntimeError(f'Extension with id "{dep}" has not started yet')

        return super().start()

    def __watch_dependency(self):
        if self.__dependency_watch_disposer is not None:
            self.__dependency_watch_disposer.dispose()

        deps = self.metadata.get("dependency", [])
        deps_stop_event = map(lambda id: "extension.stop." + id, deps)

        def close_dependency_handler(e: AbstractEvent):
            if e.type in deps_stop_event:
                self.stop()

        self.__dependency_watch_disposer = self.__extension_manager.add_event_listener(
            "extension.stop", close_dependency_handler
        )

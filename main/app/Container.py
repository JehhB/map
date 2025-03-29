from abc import ABC, abstractmethod
from optparse import Option
from typing import Callable, Dict, List, Optional, Tuple, Type, TypeVar, Union, cast

from typing_extensions import Self

T = TypeVar("T")
ModuleDependency = Optional[List[Union[str, Type["AbstractModule"]]]]
ModuleFactory = Callable[["Container"], T]
ModuleDefinition = Tuple[ModuleFactory[T], ModuleDependency]

AbstractModuleImpl = TypeVar("AbstractModuleImpl", bound="AbstractModule")
SetterInjectableImpl = TypeVar("SetterInjectableImpl", bound="SetterInjectable")


class SetterInjectable:
    container: Union[None, "Container"]

    def __init__(self) -> None:
        self.container = None

    def set_container(self, container: "Container"):
        self.container = container


class AbstractModule(ABC):
    @classmethod
    @abstractmethod
    def DEFINITION(cls) -> ModuleDefinition[Self]:
        raise NotImplementedError()

    @staticmethod
    @abstractmethod
    def KEY() -> str:
        raise NotImplementedError()


class Container:
    @staticmethod
    def defaultSetterFactory(
        constructor: Type[SetterInjectableImpl],
    ) -> ModuleFactory[SetterInjectableImpl]:
        def factory(container: Container) -> SetterInjectableImpl:
            obj = constructor()
            obj.set_container(container)
            return obj

        return factory

    def __init__(self):
        self._uninitialized_modules: Dict[str, ModuleDefinition[object]] = dict()
        self._initialized_modules: Dict[str, object] = dict()

    def register_module(
        self, key: str, factory: ModuleFactory[T], dependency: ModuleDependency = None
    ) -> Optional[T]:
        self._uninitialized_modules[key] = (factory, dependency)

        new_entry = True
        while new_entry:
            new_entry = False

            for k, definition in list(self._uninitialized_modules.items()):
                new_entry = new_entry or self._initialize_module(k, definition)

        return cast(Optional[T], self._initialized_modules.get(key, None))

    def register(
        self, constructor: Type[AbstractModuleImpl]
    ) -> Optional[AbstractModuleImpl]:
        factory, dependency = constructor.DEFINITION()
        key = constructor.KEY()
        return self.register_module(key, factory, dependency)

    def _initialize_module(
        self, key: str, definition: ModuleDefinition[T]
    ) -> Optional[T]:
        factory, dependency = definition
        if not self._complete_dependencies(dependency):
            return None

        module = factory(self)
        self._initialized_modules[key] = module
        _ = self._uninitialized_modules.pop(key)

        return module

    def _complete_dependencies(self, dependency: ModuleDependency = None):
        dependency = dependency if dependency is not None else []
        dep_keys = [dep if isinstance(dep, str) else dep.KEY() for dep in dependency]
        return all([dep in self._initialized_modules for dep in dep_keys])

    def get_module(
        self, key: Union[str, Type[AbstractModuleImpl]]
    ) -> AbstractModuleImpl:
        keyStr = key if isinstance(key, str) else key.KEY()
        return cast(AbstractModuleImpl, self._initialized_modules[keyStr])

    def __getitem__(
        self, key: Union[str, Type[AbstractModuleImpl]]
    ) -> AbstractModuleImpl:
        return self.get_module(key)

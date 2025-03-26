from abc import ABC, abstractmethod
from typing import Callable, Self, TypeAlias, TypeVar, cast

T = TypeVar("T")
ModuleDependency: TypeAlias = list[str | type["AbstractModule"]] | None
ModuleFactory: TypeAlias = Callable[["Container"], T]
ModuleDefinition: TypeAlias = tuple[ModuleFactory[T], ModuleDependency]


class SetterInjectable:
    container: "None | Container"

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
    def defaultSetterFactory[
        T: SetterInjectable
    ](constructor: type[T]) -> ModuleFactory[T]:
        def factory(container: Container) -> T:
            obj = constructor()
            obj.set_container(container)
            return obj

        return factory

    def __init__(self):
        self._uninitialized_modules: dict[str, ModuleDefinition[object]] = dict()
        self._initialized_modules: dict[str, object] = dict()

    def register_module[
        T
    ](self, key: str, factory: ModuleFactory[T], dependency: ModuleDependency = None):
        self._uninitialized_modules[key] = (factory, dependency)

        new_entry = True
        while new_entry:
            new_entry = False

            for key, definition in list(self._uninitialized_modules.items()):
                new_entry = new_entry or self._initialize_module(key, definition)

    def register[T: AbstractModule](self, constructor: type[T]):
        factory, dependency = constructor.DEFINITION()
        key = constructor.KEY()
        self.register_module(key, factory, dependency)

    def _initialize_module[T](self, key: str, definition: ModuleDefinition[T]) -> bool:
        factory, dependency = definition
        if not self._complete_dependencies(dependency):
            return False

        module = factory(self)
        self._initialized_modules[key] = module
        _ = self._uninitialized_modules.pop(key)

        return True

    def _complete_dependencies(self, dependency: ModuleDependency = None):
        dependency = dependency if dependency is not None else []
        dep_keys = [dep if isinstance(dep, str) else dep.KEY() for dep in dependency]
        return all([dep in self._initialized_modules for dep in dep_keys])

    def get_module[T: AbstractModule](self, key: str | type[T]) -> T:
        keyStr = key if isinstance(key, str) else key.KEY()
        return cast(T, self._initialized_modules[keyStr])

    def __getitem__[T: AbstractModule](self, key: str | type[T]) -> T:
        return self.get_module(key)

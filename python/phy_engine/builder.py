from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

from .circuit import Circuit, Element, Wire


@dataclass(frozen=True)
class BuilderPinRef:
    element_index: int
    pin_index: int


@dataclass(frozen=True)
class BuilderElementRef:
    index: int
    name: str | None = None

    def pin(self, pin_index: int) -> BuilderPinRef:
        return BuilderPinRef(self.index, pin_index)


@dataclass(frozen=True)
class BuilderNodeRef:
    index: int
    name: str | None = None


class NetlistBuilder:
    def __init__(self) -> None:
        self._elements: list[Element | None] = []
        self._element_names: dict[str, int] = {}
        self._nodes: list[list[tuple[int, int]]] = []
        self._node_names: dict[str, int] = {}
        self._endpoint_to_node: dict[tuple[int, int], int] = {}

    def add_element(
        self,
        code: Element | int,
        properties: Sequence[float] = (),
        *,
        verilog_source: str | None = None,
        verilog_top: str | None = None,
        name: str | None = None,
    ) -> BuilderElementRef:
        if isinstance(code, Element):
            if properties or verilog_source is not None or verilog_top is not None:
                raise ValueError("properties/verilog_* cannot be provided when code is already an Element")
            element = code
        else:
            element = Element(code=code, properties=tuple(properties), verilog_source=verilog_source, verilog_top=verilog_top)

        index = len(self._elements)
        self._elements.append(element)
        if name is not None:
            if name in self._element_names:
                raise ValueError(f"duplicate element name: {name}")
            self._element_names[name] = index
        return BuilderElementRef(index=index, name=name)

    add_model = add_element

    def create_node(self, name: str | None = None) -> BuilderNodeRef:
        index = len(self._nodes)
        self._nodes.append([])
        if name is not None:
            if name in self._node_names:
                raise ValueError(f"duplicate node name: {name}")
            self._node_names[name] = index
        return BuilderNodeRef(index=index, name=name)

    def add_to_node(
        self,
        node: BuilderNodeRef | int | str,
        element: BuilderElementRef | int | str,
        pin_index: int,
    ) -> BuilderNodeRef:
        node_index = self._resolve_node_index(node)
        endpoint = (self._resolve_element_index(element), int(pin_index))
        existing = self._endpoint_to_node.get(endpoint)
        if existing is not None and existing != node_index:
            node_index = self._merge_node_indices(node_index, existing)
        self._add_endpoint(node_index, endpoint)
        return self._node_ref(node_index)

    def connect(
        self,
        element_a: BuilderElementRef | BuilderPinRef | int | str,
        pin_a: BuilderPinRef | int,
        element_b: BuilderElementRef | int | str | None = None,
        pin_b: int | None = None,
    ) -> BuilderNodeRef:
        if isinstance(element_a, BuilderPinRef) and isinstance(pin_a, BuilderPinRef):
            if element_b is not None or pin_b is not None:
                raise TypeError("connect(pin_a, pin_b) does not accept extra arguments")
            endpoint_a = (element_a.element_index, element_a.pin_index)
            endpoint_b = (pin_a.element_index, pin_a.pin_index)
        else:
            if element_b is None or pin_b is None:
                raise TypeError("connect() requires either (pin_a, pin_b) or (element_a, pin_a, element_b, pin_b)")
            endpoint_a = (self._resolve_element_index(element_a), int(pin_a))
            endpoint_b = (self._resolve_element_index(element_b), int(pin_b))
        node_a = self._endpoint_to_node.get(endpoint_a)
        node_b = self._endpoint_to_node.get(endpoint_b)

        if node_a is None and node_b is None:
            node = self.create_node()
            node_index = node.index
        elif node_a is None:
            node_index = node_b
        elif node_b is None:
            node_index = node_a
        else:
            node_index = self._merge_node_indices(node_a, node_b)

        self._add_endpoint(node_index, endpoint_a)
        self._add_endpoint(node_index, endpoint_b)
        return self._node_ref(node_index)

    def connect_pins(self, pin_a: BuilderPinRef, pin_b: BuilderPinRef) -> BuilderNodeRef:
        return self.connect(pin_a.element_index, pin_a.pin_index, pin_b.element_index, pin_b.pin_index)

    def merge_nodes(self, dst: BuilderNodeRef | int | str, src: BuilderNodeRef | int | str) -> BuilderNodeRef:
        return self._node_ref(self._merge_node_indices(self._resolve_node_index(dst), self._resolve_node_index(src)))

    def delete_model(self, element: BuilderElementRef | int | str) -> None:
        element_index = self._resolve_element_index(element)
        if self._elements[element_index] is None:
            return
        self._elements[element_index] = None

        doomed = [endpoint for endpoint in self._endpoint_to_node if endpoint[0] == element_index]
        for endpoint in doomed:
            node_index = self._endpoint_to_node.pop(endpoint)
            endpoints = self._nodes[node_index]
            self._nodes[node_index] = [item for item in endpoints if item != endpoint]

    delete_element = delete_model

    def build(self, *, library: str | None = None) -> Circuit:
        index_map: dict[int, int] = {}
        elements: list[Element] = []
        for old_index, element in enumerate(self._elements):
            if element is None:
                continue
            index_map[old_index] = len(elements)
            elements.append(element)

        wires: list[Wire] = []
        for endpoints in self._nodes:
            active = [(index_map[element_index], pin_index) for element_index, pin_index in endpoints if element_index in index_map]
            if len(active) < 2:
                continue
            root_element, root_pin = active[0]
            for element_index, pin_index in active[1:]:
                wires.append(Wire(root_element, root_pin, element_index, pin_index))

        return Circuit(elements=elements, wires=wires, library=library)

    def iter_elements(self) -> Iterable[BuilderElementRef]:
        for index, element in enumerate(self._elements):
            if element is not None:
                yield BuilderElementRef(index=index, name=self._name_for_index(self._element_names, index))

    def iter_nodes(self) -> Iterable[BuilderNodeRef]:
        for index, endpoints in enumerate(self._nodes):
            if endpoints:
                yield BuilderNodeRef(index=index, name=self._name_for_index(self._node_names, index))

    def _resolve_element_index(self, element: BuilderElementRef | int | str) -> int:
        if isinstance(element, BuilderElementRef):
            index = element.index
        elif isinstance(element, str):
            if element not in self._element_names:
                raise KeyError(f"unknown element name: {element}")
            index = self._element_names[element]
        else:
            index = int(element)

        if index < 0 or index >= len(self._elements) or self._elements[index] is None:
            raise KeyError(f"unknown element index: {index}")
        return index

    def _resolve_node_index(self, node: BuilderNodeRef | int | str) -> int:
        if isinstance(node, BuilderNodeRef):
            index = node.index
        elif isinstance(node, str):
            if node not in self._node_names:
                raise KeyError(f"unknown node name: {node}")
            index = self._node_names[node]
        else:
            index = int(node)

        if index < 0 or index >= len(self._nodes):
            raise KeyError(f"unknown node index: {index}")
        return index

    def _add_endpoint(self, node_index: int, endpoint: tuple[int, int]) -> None:
        existing = self._endpoint_to_node.get(endpoint)
        if existing is not None and existing != node_index:
            node_index = self._merge_node_indices(node_index, existing)
        if endpoint not in self._nodes[node_index]:
            self._nodes[node_index].append(endpoint)
        self._endpoint_to_node[endpoint] = node_index

    def _merge_node_indices(self, dst_index: int, src_index: int) -> int:
        if dst_index == src_index:
            return dst_index

        dst_endpoints = self._nodes[dst_index]
        src_endpoints = self._nodes[src_index]
        for endpoint in src_endpoints:
            if endpoint not in dst_endpoints:
                dst_endpoints.append(endpoint)
            self._endpoint_to_node[endpoint] = dst_index
        self._nodes[src_index] = []
        return dst_index

    def _node_ref(self, index: int) -> BuilderNodeRef:
        return BuilderNodeRef(index=index, name=self._name_for_index(self._node_names, index))

    @staticmethod
    def _name_for_index(table: dict[str, int], index: int) -> str | None:
        for name, value in table.items():
            if value == index:
                return name
        return None


__all__ = [
    "BuilderElementRef",
    "BuilderNodeRef",
    "BuilderPinRef",
    "NetlistBuilder",
]

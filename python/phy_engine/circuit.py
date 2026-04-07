from __future__ import annotations

import ctypes as ct
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Iterable, Sequence

from ._ffi import PhyEngineError, check_rc, load_library, raise_last_error


class AnalyzeType(IntEnum):
    OP = 0
    DC = 1
    AC = 2
    ACOP = 3
    TR = 4
    TROP = 5


class DigitalState(IntEnum):
    L = 0
    H = 1
    X = 2
    Z = 3


class ElementCode(IntEnum):
    GROUND = 0

    RESISTOR = 1
    CAPACITOR = 2
    INDUCTOR = 3
    VDC = 4
    VAC = 5
    IDC = 6
    IAC = 7
    VCCS = 8
    VCVS = 9
    CCCS = 10
    CCVS = 11
    SWITCH_SPST = 12
    PN_JUNCTION = 13
    TRANSFORMER = 14
    COUPLED_INDUCTORS = 15
    TRANSFORMER_CENTER_TAP = 16
    OP_AMP = 17
    RELAY = 18
    COMPARATOR = 19
    SAWTOOTH = 20
    SQUARE = 21
    PULSE = 22
    TRIANGLE = 23

    BJT_NPN = 50
    BJT_PNP = 51
    NMOSFET = 52
    PMOSFET = 53
    FULL_BRIDGE_RECTIFIER = 54
    BSIM3V32_NMOS = 55
    BSIM3V32_PMOS = 56

    DIGITAL_INPUT = 200
    DIGITAL_OUTPUT = 201
    DIGITAL_OR = 202
    DIGITAL_YES = 203
    DIGITAL_AND = 204
    DIGITAL_NOT = 205
    DIGITAL_XOR = 206
    DIGITAL_XNOR = 207
    DIGITAL_NAND = 208
    DIGITAL_NOR = 209
    DIGITAL_TRI = 210
    DIGITAL_IMP = 211
    DIGITAL_NIMP = 212

    DIGITAL_HALF_ADDER = 220
    DIGITAL_FULL_ADDER = 221
    DIGITAL_HALF_SUBTRACTOR = 222
    DIGITAL_FULL_SUBTRACTOR = 223
    DIGITAL_MUL2 = 224
    DIGITAL_DFF = 225
    DIGITAL_TFF = 226
    DIGITAL_T_BAR_FF = 227
    DIGITAL_JKFF = 228
    DIGITAL_COUNTER4 = 229
    DIGITAL_RANDOM_GENERATOR4 = 230
    DIGITAL_EIGHT_BIT_INPUT = 231
    DIGITAL_EIGHT_BIT_DISPLAY = 232
    DIGITAL_SCHMITT_TRIGGER = 233

    VERILOG_MODULE = 300
    VERILOG_NETLIST = 301


PROPERTY_ARITY: dict[int, int] = {
    ElementCode.GROUND: 0,
    ElementCode.RESISTOR: 1,
    ElementCode.CAPACITOR: 1,
    ElementCode.INDUCTOR: 1,
    ElementCode.VDC: 1,
    ElementCode.VAC: 3,
    ElementCode.IDC: 1,
    ElementCode.IAC: 3,
    ElementCode.VCCS: 1,
    ElementCode.VCVS: 1,
    ElementCode.CCCS: 1,
    ElementCode.CCVS: 1,
    ElementCode.SWITCH_SPST: 1,
    ElementCode.PN_JUNCTION: 9,
    ElementCode.TRANSFORMER: 1,
    ElementCode.COUPLED_INDUCTORS: 3,
    ElementCode.TRANSFORMER_CENTER_TAP: 1,
    ElementCode.OP_AMP: 1,
    ElementCode.RELAY: 2,
    ElementCode.COMPARATOR: 2,
    ElementCode.SAWTOOTH: 4,
    ElementCode.SQUARE: 5,
    ElementCode.PULSE: 7,
    ElementCode.TRIANGLE: 4,
    ElementCode.BJT_NPN: 5,
    ElementCode.BJT_PNP: 5,
    ElementCode.NMOSFET: 3,
    ElementCode.PMOSFET: 3,
    ElementCode.FULL_BRIDGE_RECTIFIER: 0,
    ElementCode.BSIM3V32_NMOS: 14,
    ElementCode.BSIM3V32_PMOS: 14,
    ElementCode.DIGITAL_INPUT: 1,
    ElementCode.DIGITAL_OUTPUT: 0,
    ElementCode.DIGITAL_OR: 0,
    ElementCode.DIGITAL_YES: 0,
    ElementCode.DIGITAL_AND: 0,
    ElementCode.DIGITAL_NOT: 0,
    ElementCode.DIGITAL_XOR: 0,
    ElementCode.DIGITAL_XNOR: 0,
    ElementCode.DIGITAL_NAND: 0,
    ElementCode.DIGITAL_NOR: 0,
    ElementCode.DIGITAL_TRI: 0,
    ElementCode.DIGITAL_IMP: 0,
    ElementCode.DIGITAL_NIMP: 0,
    ElementCode.DIGITAL_HALF_ADDER: 0,
    ElementCode.DIGITAL_FULL_ADDER: 0,
    ElementCode.DIGITAL_HALF_SUBTRACTOR: 0,
    ElementCode.DIGITAL_FULL_SUBTRACTOR: 0,
    ElementCode.DIGITAL_MUL2: 0,
    ElementCode.DIGITAL_DFF: 0,
    ElementCode.DIGITAL_TFF: 0,
    ElementCode.DIGITAL_T_BAR_FF: 0,
    ElementCode.DIGITAL_JKFF: 0,
    ElementCode.DIGITAL_COUNTER4: 1,
    ElementCode.DIGITAL_RANDOM_GENERATOR4: 1,
    ElementCode.DIGITAL_EIGHT_BIT_INPUT: 1,
    ElementCode.DIGITAL_EIGHT_BIT_DISPLAY: 0,
    ElementCode.DIGITAL_SCHMITT_TRIGGER: 5,
    ElementCode.VERILOG_MODULE: 0,
    ElementCode.VERILOG_NETLIST: 0,
}


@dataclass(frozen=True)
class Wire:
    element_a: int
    pin_a: int
    element_b: int
    pin_b: int


@dataclass(frozen=True)
class Element:
    code: ElementCode | int
    properties: Sequence[float] = field(default_factory=tuple)
    verilog_source: str | None = None
    verilog_top: str | None = None

    def normalized_code(self) -> int:
        return int(self.code)


@dataclass(frozen=True)
class ComponentSample:
    component_index: int
    element_index: int
    pin_voltages: tuple[float, ...]
    branch_currents: tuple[float, ...]
    pin_digital: tuple[DigitalState, ...]


@dataclass(frozen=True)
class CircuitSample:
    components: tuple[ComponentSample, ...]
    voltage_ord: tuple[int, ...]
    current_ord: tuple[int, ...]
    digital_ord: tuple[int, ...]


def _as_element(spec: Element | int) -> Element:
    if isinstance(spec, Element):
        return spec
    return Element(code=spec)


def _flatten_wires(wires: Sequence[Wire]) -> list[int]:
    raw: list[int] = []
    for wire in wires:
        raw.extend((wire.element_a, wire.pin_a, wire.element_b, wire.pin_b))
    return raw


def _non_null_double_array(values: Sequence[float]) -> ct.Array[ct.c_double]:
    data = list(values)
    if not data:
        data = [0.0]
    return (ct.c_double * len(data))(*data)


def _non_null_int_array(values: Sequence[int]) -> ct.Array[ct.c_int]:
    data = list(values)
    if not data:
        data = [0]
    return (ct.c_int * len(data))(*data)


def _validate_elements(elements: Sequence[Element]) -> None:
    if not elements:
        raise ValueError("Circuit requires at least one element")

    for index, element in enumerate(elements):
        code = element.normalized_code()
        expected = PROPERTY_ARITY.get(code)
        if expected is not None and len(element.properties) != expected:
            raise ValueError(
                f"element[{index}] code={code} expects {expected} properties, got {len(element.properties)}"
            )
        is_verilog = code in (ElementCode.VERILOG_MODULE, ElementCode.VERILOG_NETLIST)
        if is_verilog and not element.verilog_source:
            raise ValueError(f"element[{index}] requires verilog_source")
        if not is_verilog and (element.verilog_source is not None or element.verilog_top is not None):
            raise ValueError(f"element[{index}] has verilog_source/verilog_top but is not a Verilog element")


def _copy_size_array(ptr: ct.POINTER(ct.c_size_t), count: int) -> list[int]:
    return [int(ptr[index]) for index in range(count)]


class Circuit:
    def __init__(
        self,
        elements: Sequence[Element | int],
        wires: Sequence[Wire] = (),
        *,
        library: str | None = None,
    ) -> None:
        self._lib = load_library(library)
        self._elements = [_as_element(spec) for spec in elements]
        _validate_elements(self._elements)
        self._wires = list(wires)
        self._non_ground_element_indices = tuple(
            index for index, element in enumerate(self._elements) if element.normalized_code() != ElementCode.GROUND
        )

        codes = [element.normalized_code() for element in self._elements]
        properties = [value for element in self._elements for value in element.properties]
        flat_wires = _flatten_wires(self._wires)

        codes_arr = _non_null_int_array(codes)
        props_arr = _non_null_double_array(properties)
        wires_arr = _non_null_int_array(flat_wires)
        wires_ptr = wires_arr if flat_wires else ct.cast(None, ct.POINTER(ct.c_int))

        vec_pos = ct.POINTER(ct.c_size_t)()
        chunk_pos = ct.POINTER(ct.c_size_t)()
        comp_size = ct.c_size_t()

        use_ex = any(code in (ElementCode.VERILOG_MODULE, ElementCode.VERILOG_NETLIST) for code in codes)
        if use_ex:
            text_bytes: list[bytes] = []
            text_sizes: list[int] = []
            src_indices = [ct.c_size_t(-1).value] * len(self._elements)
            top_indices = [ct.c_size_t(-1).value] * len(self._elements)

            for index, element in enumerate(self._elements):
                if element.verilog_source is None:
                    continue
                src_bytes = element.verilog_source.encode("utf-8")
                src_indices[index] = len(text_bytes)
                text_bytes.append(src_bytes)
                text_sizes.append(len(src_bytes))
                if element.verilog_top is not None:
                    top_bytes = element.verilog_top.encode("utf-8")
                    top_indices[index] = len(text_bytes)
                    text_bytes.append(top_bytes)
                    text_sizes.append(len(top_bytes))

            text_arr = (ct.c_char_p * max(1, len(text_bytes)))(*text_bytes) if text_bytes else (ct.c_char_p * 1)()
            text_size_arr = (ct.c_size_t * max(1, len(text_sizes)))(*text_sizes) if text_sizes else (ct.c_size_t * 1)(0)
            src_index_arr = (ct.c_size_t * len(src_indices))(*src_indices)
            top_index_arr = (ct.c_size_t * len(top_indices))(*top_indices)

            handle = self._lib.create_circuit_ex(
                codes_arr,
                len(codes),
                wires_ptr,
                len(flat_wires),
                props_arr,
                text_arr,
                text_size_arr,
                len(text_bytes),
                src_index_arr,
                top_index_arr,
                ct.byref(vec_pos),
                ct.byref(chunk_pos),
                ct.byref(comp_size),
            )
        else:
            handle = self._lib.create_circuit(
                codes_arr,
                len(codes),
                wires_ptr,
                len(flat_wires),
                props_arr,
                ct.byref(vec_pos),
                ct.byref(chunk_pos),
                ct.byref(comp_size),
            )

        if not handle:
            raise_last_error("create_circuit/create_circuit_ex returned null", self._lib)

        self._handle = ct.c_void_p(handle)
        self._vec_pos = vec_pos
        self._chunk_pos = chunk_pos
        self.component_count = int(comp_size.value)

    def close(self) -> None:
        if getattr(self, "_handle", None):
            self._lib.destroy_circuit(self._handle, self._vec_pos, self._chunk_pos)
            self._handle = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    def __enter__(self) -> "Circuit":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _require_handle(self) -> ct.c_void_p:
        if not self._handle:
            raise PhyEngineError("Circuit is already closed")
        return self._handle

    def _component_locator(self, component_index: int) -> tuple[int, int]:
        if component_index < 0 or component_index >= self.component_count:
            raise IndexError(f"component_index out of range: {component_index}")
        return int(self._vec_pos[component_index]), int(self._chunk_pos[component_index])

    def set_analyze_type(self, analyze_type: AnalyzeType | int) -> None:
        check_rc(self._lib.circuit_set_analyze_type(self._require_handle(), int(analyze_type)), "circuit_set_analyze_type", self._lib)

    def set_tr(self, t_step: float, t_stop: float) -> None:
        check_rc(self._lib.circuit_set_tr(self._require_handle(), t_step, t_stop), "circuit_set_tr", self._lib)

    def set_ac_omega(self, omega: float) -> None:
        check_rc(self._lib.circuit_set_ac_omega(self._require_handle(), omega), "circuit_set_ac_omega", self._lib)

    def set_temperature(self, temp_c: float) -> None:
        check_rc(self._lib.circuit_set_temperature(self._require_handle(), temp_c), "circuit_set_temperature", self._lib)

    def set_tnom(self, tnom_c: float) -> None:
        check_rc(self._lib.circuit_set_tnom(self._require_handle(), tnom_c), "circuit_set_tnom", self._lib)

    def set_model_double_by_name(self, component_index: int, name: str, value: float) -> None:
        vec_pos, chunk_pos = self._component_locator(component_index)
        encoded = name.encode("utf-8")
        check_rc(
            self._lib.circuit_set_model_double_by_name(
                self._require_handle(), vec_pos, chunk_pos, encoded, len(encoded), value
            ),
            "circuit_set_model_double_by_name",
            self._lib,
        )

    def set_model_digital(
        self,
        component_index: int,
        attribute_index: int,
        state: DigitalState | int,
    ) -> None:
        vec_pos, chunk_pos = self._component_locator(component_index)
        check_rc(
            self._lib.circuit_set_model_digital(
                self._require_handle(), vec_pos, chunk_pos, attribute_index, int(state)
            ),
            "circuit_set_model_digital",
            self._lib,
        )

    def analyze(self) -> None:
        check_rc(self._lib.circuit_analyze(self._require_handle()), "circuit_analyze", self._lib)

    def digital_clk(self) -> None:
        check_rc(self._lib.circuit_digital_clk(self._require_handle()), "circuit_digital_clk", self._lib)

    def sample_layout(self) -> tuple[list[int], list[int], list[int]]:
        count = self.component_count + 1
        voltage_ord = (ct.c_size_t * count)()
        current_ord = (ct.c_size_t * count)()
        digital_ord = (ct.c_size_t * count)()
        check_rc(
            self._lib.circuit_sample_layout(
                self._require_handle(),
                self._vec_pos,
                self._chunk_pos,
                self.component_count,
                voltage_ord,
                current_ord,
                digital_ord,
            ),
            "circuit_sample_layout",
            self._lib,
        )
        return list(voltage_ord), list(current_ord), list(digital_ord)

    def sample(self) -> CircuitSample:
        voltage_ord, current_ord, digital_ord = self.sample_layout()
        total_pins = voltage_ord[-1]
        total_branches = current_ord[-1]

        voltage_buf = (ct.c_double * max(1, total_pins))()
        current_buf = (ct.c_double * max(1, total_branches))()
        digital_buf = (ct.c_uint8 * max(1, total_pins))()
        voltage_ord_arr = (ct.c_size_t * len(voltage_ord))(*voltage_ord)
        current_ord_arr = (ct.c_size_t * len(current_ord))(*current_ord)
        digital_ord_arr = (ct.c_size_t * len(digital_ord))(*digital_ord)

        check_rc(
            self._lib.circuit_sample_digital_state_u8(
                self._require_handle(),
                self._vec_pos,
                self._chunk_pos,
                self.component_count,
                voltage_buf,
                voltage_ord_arr,
                current_buf,
                current_ord_arr,
                digital_buf,
                digital_ord_arr,
            ),
            "circuit_sample_digital_state_u8",
            self._lib,
        )

        components: list[ComponentSample] = []
        for component_index, element_index in enumerate(self._non_ground_element_indices):
            v_begin, v_end = voltage_ord[component_index], voltage_ord[component_index + 1]
            i_begin, i_end = current_ord[component_index], current_ord[component_index + 1]
            d_begin, d_end = digital_ord[component_index], digital_ord[component_index + 1]
            components.append(
                ComponentSample(
                    component_index=component_index,
                    element_index=element_index,
                    pin_voltages=tuple(float(voltage_buf[i]) for i in range(v_begin, v_end)),
                    branch_currents=tuple(float(current_buf[i]) for i in range(i_begin, i_end)),
                    pin_digital=tuple(DigitalState(int(digital_buf[i])) for i in range(d_begin, d_end)),
                )
            )

        return CircuitSample(
            components=tuple(components),
            voltage_ord=tuple(voltage_ord),
            current_ord=tuple(current_ord),
            digital_ord=tuple(digital_ord),
        )

    def analyze_and_sample(self) -> CircuitSample:
        self.analyze()
        return self.sample()

    def to_experiment(self, **kwargs):
        from .physicslab import circuit_to_experiment

        return circuit_to_experiment(self, **kwargs)


__all__ = [
    "AnalyzeType",
    "Circuit",
    "CircuitSample",
    "ComponentSample",
    "DigitalState",
    "Element",
    "ElementCode",
    "PROPERTY_ARITY",
    "Wire",
]

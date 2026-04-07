from __future__ import annotations

import ctypes as ct
import json
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import TYPE_CHECKING, Mapping, Literal, Sequence

from ._ffi import PhyEngineError, check_rc, load_library, raise_last_error, take_owned_c_string
from .circuit import AnalyzeType, DigitalState

if TYPE_CHECKING:
    from .circuit import Circuit


class ExperimentType(IntEnum):
    CIRCUIT = 0
    CELESTIAL = 3
    ELECTROMAGNETISM = 4


class WireColor(IntEnum):
    BLACK = 0
    BLUE = 1
    RED = 2
    GREEN = 3
    YELLOW = 4


class AutoLayoutBackend(IntEnum):
    CPU = 0
    CUDA = 1


class AutoLayoutMode(IntEnum):
    FAST = 0
    CLUSTER = 1
    SPECTRAL = 2
    HIERARCHICAL = 3
    FORCE = 4


@dataclass(frozen=True)
class Position:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class AutoLayoutStats:
    grid_w: int
    grid_h: int
    fixed_obstacles: int
    placed: int
    skipped: int


@dataclass(frozen=True)
class WriteBackOptions:
    logic_output_low: float = 0.0
    logic_output_high: float = 1.0
    logic_output_x: float = 1.0
    logic_output_z: float = 1.0


@dataclass(frozen=True)
class ExperimentElement:
    identifier: str
    model_id: str
    label: str | None
    properties: dict[str, object]
    statistics: dict[str, object]
    raw: dict[str, object]


@dataclass(frozen=True)
class ExperimentComponentSample:
    component_index: int
    pin_voltages: tuple[float, ...]
    branch_currents: tuple[float, ...]
    pin_digital: tuple[DigitalState, ...]


@dataclass(frozen=True)
class ExperimentCircuitSample:
    components: tuple[ExperimentComponentSample, ...]
    voltage_ord: tuple[int, ...]
    current_ord: tuple[int, ...]
    digital_ord: tuple[int, ...]


ElementKeyMode = Literal["identifier", "label", "label_or_identifier"]


def _as_position(value: Position | Sequence[float]) -> Position:
    if isinstance(value, Position):
        return value
    if len(value) != 3:
        raise ValueError("position must contain exactly 3 numbers")
    return Position(float(value[0]), float(value[1]), float(value[2]))


def _encode_text(value: str) -> tuple[bytes, int]:
    encoded = value.encode("utf-8")
    return encoded, len(encoded)


def _require_handle(obj, attr: str, name: str) -> ct.c_void_p:
    handle = getattr(obj, attr, None)
    if not handle:
        raise PhyEngineError(f"{name} is already closed")
    return handle


def _resolve_write_back_options(
    options: WriteBackOptions | None,
    *,
    logic_output_low: float | None,
    logic_output_high: float | None,
    logic_output_x: float | None,
    logic_output_z: float | None,
) -> WriteBackOptions | None:
    has_overrides = (
        logic_output_low is not None
        or logic_output_high is not None
        or logic_output_x is not None
        or logic_output_z is not None
    )
    if options is not None and has_overrides:
        raise ValueError("write_back_to_pl accepts either options=WriteBackOptions(...) or individual logic_output_* overrides, not both")
    if options is not None:
        return options
    if not has_overrides:
        return None
    return WriteBackOptions(
        logic_output_low=0.0 if logic_output_low is None else float(logic_output_low),
        logic_output_high=1.0 if logic_output_high is None else float(logic_output_high),
        logic_output_x=1.0 if logic_output_x is None else float(logic_output_x),
        logic_output_z=1.0 if logic_output_z is None else float(logic_output_z),
    )


def _experiment_elements_from_dump(sav_json: str) -> list[ExperimentElement]:
    root = json.loads(sav_json)
    experiment = root.get("Experiment", {})
    status_raw = experiment.get("StatusSave", "")
    status = json.loads(status_raw) if isinstance(status_raw, str) and status_raw else {}
    elements_raw = status.get("Elements", [])
    if isinstance(elements_raw, dict):
        values = list(elements_raw.values())
    else:
        values = list(elements_raw)

    elements: list[ExperimentElement] = []
    for item in values:
        if not isinstance(item, dict):
            continue
        label = item.get("Label")
        elements.append(
            ExperimentElement(
                identifier=str(item.get("Identifier", "")),
                model_id=str(item.get("ModelID", "")),
                label=label if isinstance(label, str) else None,
                properties=dict(item.get("Properties", {})) if isinstance(item.get("Properties"), dict) else {},
                statistics=dict(item.get("Statistics", {})) if isinstance(item.get("Statistics"), dict) else {},
                raw=dict(item),
            )
        )
    return elements


def _element_key(element: ExperimentElement, by: ElementKeyMode) -> str:
    if by == "identifier":
        return element.identifier
    if by == "label":
        if not element.label:
            raise ValueError(f"element {element.identifier} has no label")
        return element.label
    return element.label or element.identifier


def _element_lookup(elements: Sequence[ExperimentElement], by: ElementKeyMode) -> dict[str, ExperimentElement]:
    lookup: dict[str, ExperimentElement] = {}
    for element in elements:
        key = _element_key(element, by)
        if key in lookup:
            raise ValueError(f"duplicate element key for by={by!r}: {key}")
        lookup[key] = element
    return lookup


class Experiment:
    def __init__(
        self,
        experiment_type: ExperimentType | int = ExperimentType.CIRCUIT,
        *,
        handle: int | ct.c_void_p | None = None,
        library: str | None = None,
    ) -> None:
        self._lib = load_library(library)
        if handle is None:
            raw_handle = self._lib.pl_experiment_create(int(experiment_type))
            if not raw_handle:
                raise_last_error("pl_experiment_create returned null", self._lib)
            handle = raw_handle

        self._handle = ct.c_void_p(handle)

    @classmethod
    def _from_handle(cls, handle: int | ct.c_void_p, lib: ct.CDLL) -> "Experiment":
        self = cls.__new__(cls)
        self._lib = lib
        self._handle = ct.c_void_p(handle)
        return self

    @classmethod
    def load_from_string(cls, sav_json: str, *, library: str | None = None) -> "Experiment":
        lib = load_library(library)
        encoded, size = _encode_text(sav_json)
        handle = lib.pl_experiment_load_from_string(encoded, size)
        if not handle:
            raise_last_error("pl_experiment_load_from_string returned null", lib)
        return cls._from_handle(handle, lib)

    @classmethod
    def load_from_file(cls, path: str | Path, *, library: str | None = None) -> "Experiment":
        lib = load_library(library)
        encoded, size = _encode_text(str(Path(path)))
        handle = lib.pl_experiment_load_from_file(encoded, size)
        if not handle:
            raise_last_error("pl_experiment_load_from_file returned null", lib)
        return cls._from_handle(handle, lib)

    def close(self) -> None:
        if getattr(self, "_handle", None):
            self._lib.pl_experiment_destroy(self._handle)
            self._handle = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    def __enter__(self) -> "Experiment":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _require_handle(self) -> ct.c_void_p:
        return _require_handle(self, "_handle", "Experiment")

    def dump(self, *, indent: int = 2) -> str:
        ptr = self._lib.pl_experiment_dump(self._require_handle(), indent)
        if not ptr:
            raise_last_error("pl_experiment_dump returned null", self._lib)
        return take_owned_c_string(ptr, self._lib)

    def elements(self, *, model_id: str | None = None) -> tuple[ExperimentElement, ...]:
        elements = _experiment_elements_from_dump(self.dump(indent=-1))
        if model_id is not None:
            elements = [element for element in elements if element.model_id == model_id]
        return tuple(elements)

    def logic_inputs(self, *, by: ElementKeyMode = "label_or_identifier") -> dict[str, float]:
        elements = self.elements(model_id="Logic Input")
        lookup = _element_lookup(elements, by)
        return {key: float(element.properties.get("开关", 0.0)) for key, element in lookup.items()}

    def logic_outputs(self, *, by: ElementKeyMode = "label_or_identifier") -> dict[str, float]:
        elements = self.elements(model_id="Logic Output")
        lookup = _element_lookup(elements, by)
        return {key: float(element.properties.get("状态", 0.0)) for key, element in lookup.items()}

    def set_logic_inputs(
        self,
        values: Mapping[str, float | int | bool],
        *,
        by: ElementKeyMode = "label_or_identifier",
    ) -> None:
        elements = self.elements(model_id="Logic Input")
        lookup = _element_lookup(elements, by)
        for key, value in values.items():
            if key not in lookup:
                raise KeyError(f"unknown Logic Input key for by={by!r}: {key}")
            self.set_element_property_number(lookup[key].identifier, "开关", float(value))

    def save(self, path: str | Path, *, indent: int = 2) -> None:
        encoded, size = _encode_text(str(Path(path)))
        check_rc(self._lib.pl_experiment_save(self._require_handle(), encoded, size, indent), "pl_experiment_save", self._lib)

    def add_circuit_element(
        self,
        model_id: str,
        position: Position | Sequence[float] = Position(0.0, 0.0, 0.0),
        *,
        element_xyz_coords: bool = False,
        is_big_element: bool = False,
        participate_in_layout: bool = True,
    ) -> str:
        pos = _as_position(position)
        model_id_bytes, model_id_size = _encode_text(model_id)
        ptr = self._lib.pl_experiment_add_circuit_element(
            self._require_handle(),
            model_id_bytes,
            model_id_size,
            pos.x,
            pos.y,
            pos.z,
            int(element_xyz_coords),
            int(is_big_element),
            int(participate_in_layout),
        )
        if not ptr:
            raise_last_error("pl_experiment_add_circuit_element returned null", self._lib)
        return take_owned_c_string(ptr, self._lib)

    def connect(
        self,
        src_id: str,
        src_pin: int,
        dst_id: str,
        dst_pin: int,
        *,
        color: WireColor | int = WireColor.BLUE,
    ) -> None:
        src_bytes, src_size = _encode_text(src_id)
        dst_bytes, dst_size = _encode_text(dst_id)
        check_rc(
            self._lib.pl_experiment_connect(
                self._require_handle(),
                src_bytes,
                src_size,
                src_pin,
                dst_bytes,
                dst_size,
                dst_pin,
                int(color),
            ),
            "pl_experiment_connect",
            self._lib,
        )

    def clear_wires(self) -> None:
        check_rc(self._lib.pl_experiment_clear_wires(self._require_handle()), "pl_experiment_clear_wires", self._lib)

    def set_xyz_precision(self, decimals: int) -> None:
        check_rc(self._lib.pl_experiment_set_xyz_precision(self._require_handle(), decimals), "pl_experiment_set_xyz_precision", self._lib)

    def set_element_xyz(self, enabled: bool, origin: Position | Sequence[float] = Position(0.0, 0.0, 0.0)) -> None:
        pos = _as_position(origin)
        check_rc(
            self._lib.pl_experiment_set_element_xyz(self._require_handle(), int(enabled), pos.x, pos.y, pos.z),
            "pl_experiment_set_element_xyz",
            self._lib,
        )

    def set_camera(
        self,
        vision_center: Position | Sequence[float],
        target_rotation: Position | Sequence[float],
    ) -> None:
        center = _as_position(vision_center)
        rotation = _as_position(target_rotation)
        check_rc(
            self._lib.pl_experiment_set_camera(
                self._require_handle(),
                center.x,
                center.y,
                center.z,
                rotation.x,
                rotation.y,
                rotation.z,
            ),
            "pl_experiment_set_camera",
            self._lib,
        )

    def set_element_property_number(self, element_id: str, key: str, value: float) -> None:
        element_bytes, element_size = _encode_text(element_id)
        key_bytes, key_size = _encode_text(key)
        check_rc(
            self._lib.pl_experiment_set_element_property_number(
                self._require_handle(),
                element_bytes,
                element_size,
                key_bytes,
                key_size,
                value,
            ),
            "pl_experiment_set_element_property_number",
            self._lib,
        )

    def set_element_label(self, element_id: str, label: str | None) -> None:
        element_bytes, element_size = _encode_text(element_id)
        label_bytes = None if label is None else label.encode("utf-8")
        label_size = 0 if label_bytes is None else len(label_bytes)
        check_rc(
            self._lib.pl_experiment_set_element_label(
                self._require_handle(),
                element_bytes,
                element_size,
                label_bytes,
                label_size,
            ),
            "pl_experiment_set_element_label",
            self._lib,
        )

    def set_element_position(
        self,
        element_id: str,
        position: Position | Sequence[float],
        *,
        element_xyz_coords: bool = False,
    ) -> None:
        pos = _as_position(position)
        element_bytes, element_size = _encode_text(element_id)
        check_rc(
            self._lib.pl_experiment_set_element_position(
                self._require_handle(),
                element_bytes,
                element_size,
                pos.x,
                pos.y,
                pos.z,
                int(element_xyz_coords),
            ),
            "pl_experiment_set_element_position",
            self._lib,
        )

    def merge(self, other: "Experiment", offset: Position | Sequence[float] = Position(0.0, 0.0, 0.0)) -> None:
        pos = _as_position(offset)
        check_rc(
            self._lib.pl_experiment_merge(self._require_handle(), other._require_handle(), pos.x, pos.y, pos.z),
            "pl_experiment_merge",
            self._lib,
        )

    def build_pe_circuit(self) -> "ExperimentCircuit":
        raw_handle = self._lib.pl_pe_circuit_build(self._require_handle())
        if not raw_handle:
            raise_last_error("pl_pe_circuit_build returned null", self._lib)
        return ExperimentCircuit._from_handle(raw_handle, self._lib)

    def auto_layout(
        self,
        corner0: Position | Sequence[float],
        corner1: Position | Sequence[float],
        *,
        z_fixed: float = 0.0,
        backend: AutoLayoutBackend | int = AutoLayoutBackend.CPU,
        mode: AutoLayoutMode | int = AutoLayoutMode.FAST,
        step: Sequence[float] = (0.16, 0.08),
        margin: Sequence[float] = (0.0, 0.0),
    ) -> AutoLayoutStats:
        if len(step) != 2:
            raise ValueError("step must contain exactly 2 numbers")
        if len(margin) != 2:
            raise ValueError("margin must contain exactly 2 numbers")

        p0 = _as_position(corner0)
        p1 = _as_position(corner1)
        grid_w = ct.c_size_t()
        grid_h = ct.c_size_t()
        fixed_obstacles = ct.c_size_t()
        placed = ct.c_size_t()
        skipped = ct.c_size_t()
        check_rc(
            self._lib.pl_experiment_auto_layout(
                self._require_handle(),
                p0.x,
                p0.y,
                p0.z,
                p1.x,
                p1.y,
                p1.z,
                z_fixed,
                int(backend),
                int(mode),
                float(step[0]),
                float(step[1]),
                float(margin[0]),
                float(margin[1]),
                ct.byref(grid_w),
                ct.byref(grid_h),
                ct.byref(fixed_obstacles),
                ct.byref(placed),
                ct.byref(skipped),
            ),
            "pl_experiment_auto_layout",
            self._lib,
        )
        return AutoLayoutStats(
            grid_w=int(grid_w.value),
            grid_h=int(grid_h.value),
            fixed_obstacles=int(fixed_obstacles.value),
            placed=int(placed.value),
            skipped=int(skipped.value),
        )


class ExperimentCircuit:
    def __init__(
        self,
        experiment: Experiment | None = None,
        *,
        handle: int | ct.c_void_p | None = None,
        library: str | None = None,
    ) -> None:
        self._lib = experiment._lib if library is None and experiment is not None else load_library(library)
        if handle is None:
            if experiment is None:
                raise ValueError("experiment is required when handle is not provided")
            raw_handle = self._lib.pl_pe_circuit_build(experiment._require_handle())
            if not raw_handle:
                raise_last_error("pl_pe_circuit_build returned null", self._lib)
            handle = raw_handle

        self._handle = ct.c_void_p(handle)
        self.component_count = int(self._lib.pl_pe_circuit_comp_size(self._handle))

    @classmethod
    def _from_handle(cls, handle: int | ct.c_void_p, lib: ct.CDLL) -> "ExperimentCircuit":
        self = cls.__new__(cls)
        self._lib = lib
        self._handle = ct.c_void_p(handle)
        self.component_count = int(self._lib.pl_pe_circuit_comp_size(self._handle))
        return self

    def close(self) -> None:
        if getattr(self, "_handle", None):
            self._lib.pl_pe_circuit_destroy(self._handle)
            self._handle = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    def __enter__(self) -> "ExperimentCircuit":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _require_handle(self) -> ct.c_void_p:
        return _require_handle(self, "_handle", "ExperimentCircuit")

    def set_analyze_type(self, analyze_type: AnalyzeType | int) -> None:
        check_rc(
            self._lib.pl_pe_circuit_set_analyze_type(self._require_handle(), int(analyze_type)),
            "pl_pe_circuit_set_analyze_type",
            self._lib,
        )

    def set_tr(self, t_step: float, t_stop: float) -> None:
        check_rc(self._lib.pl_pe_circuit_set_tr(self._require_handle(), t_step, t_stop), "pl_pe_circuit_set_tr", self._lib)

    def set_ac_omega(self, omega: float) -> None:
        check_rc(self._lib.pl_pe_circuit_set_ac_omega(self._require_handle(), omega), "pl_pe_circuit_set_ac_omega", self._lib)

    def analyze(self) -> None:
        check_rc(self._lib.pl_pe_circuit_analyze(self._require_handle()), "pl_pe_circuit_analyze", self._lib)

    def digital_clk(self) -> None:
        check_rc(self._lib.pl_pe_circuit_digital_clk(self._require_handle()), "pl_pe_circuit_digital_clk", self._lib)

    def sync_inputs_from_pl(self, experiment: Experiment) -> None:
        check_rc(
            self._lib.pl_pe_circuit_sync_inputs_from_pl(self._require_handle(), experiment._require_handle()),
            "pl_pe_circuit_sync_inputs_from_pl",
            self._lib,
        )

    def write_back_to_pl(
        self,
        experiment: Experiment,
        *,
        options: WriteBackOptions | None = None,
        logic_output_low: float | None = None,
        logic_output_high: float | None = None,
        logic_output_x: float | None = None,
        logic_output_z: float | None = None,
    ) -> None:
        resolved = _resolve_write_back_options(
            options,
            logic_output_low=logic_output_low,
            logic_output_high=logic_output_high,
            logic_output_x=logic_output_x,
            logic_output_z=logic_output_z,
        )
        if resolved is None:
            check_rc(
                self._lib.pl_pe_circuit_write_back_to_pl(self._require_handle(), experiment._require_handle()),
                "pl_pe_circuit_write_back_to_pl",
                self._lib,
            )
            return

        check_rc(
            self._lib.pl_pe_circuit_write_back_to_pl_ex(
                self._require_handle(),
                experiment._require_handle(),
                resolved.logic_output_low,
                resolved.logic_output_high,
                resolved.logic_output_x,
                resolved.logic_output_z,
            ),
            "pl_pe_circuit_write_back_to_pl_ex",
            self._lib,
        )

    def sample_and_write_back_to_pl(
        self,
        experiment: Experiment,
        *,
        options: WriteBackOptions | None = None,
        logic_output_low: float | None = None,
        logic_output_high: float | None = None,
        logic_output_x: float | None = None,
        logic_output_z: float | None = None,
    ) -> None:
        # `write_back_to_pl()` already samples the current PE state internally.
        self.write_back_to_pl(
            experiment,
            options=options,
            logic_output_low=logic_output_low,
            logic_output_high=logic_output_high,
            logic_output_x=logic_output_x,
            logic_output_z=logic_output_z,
        )

    def write_back_and_read_logic_outputs(
        self,
        experiment: Experiment,
        *,
        by: ElementKeyMode = "label_or_identifier",
        options: WriteBackOptions | None = None,
        logic_output_low: float | None = None,
        logic_output_high: float | None = None,
        logic_output_x: float | None = None,
        logic_output_z: float | None = None,
    ) -> dict[str, float]:
        self.write_back_to_pl(
            experiment,
            options=options,
            logic_output_low=logic_output_low,
            logic_output_high=logic_output_high,
            logic_output_x=logic_output_x,
            logic_output_z=logic_output_z,
        )
        return experiment.logic_outputs(by=by)

    def sample_layout(self) -> tuple[list[int], list[int], list[int]]:
        count = self.component_count + 1
        voltage_ord = (ct.c_size_t * count)()
        current_ord = (ct.c_size_t * count)()
        digital_ord = (ct.c_size_t * count)()
        check_rc(
            self._lib.pl_pe_circuit_sample_layout(self._require_handle(), voltage_ord, current_ord, digital_ord),
            "pl_pe_circuit_sample_layout",
            self._lib,
        )
        return list(voltage_ord), list(current_ord), list(digital_ord)

    def sample(self) -> ExperimentCircuitSample:
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
            self._lib.pl_pe_circuit_sample_digital_state_u8(
                self._require_handle(),
                voltage_buf,
                voltage_ord_arr,
                current_buf,
                current_ord_arr,
                digital_buf,
                digital_ord_arr,
            ),
            "pl_pe_circuit_sample_digital_state_u8",
            self._lib,
        )

        components: list[ExperimentComponentSample] = []
        for component_index in range(self.component_count):
            v_begin, v_end = voltage_ord[component_index], voltage_ord[component_index + 1]
            i_begin, i_end = current_ord[component_index], current_ord[component_index + 1]
            d_begin, d_end = digital_ord[component_index], digital_ord[component_index + 1]
            components.append(
                ExperimentComponentSample(
                    component_index=component_index,
                    pin_voltages=tuple(float(voltage_buf[i]) for i in range(v_begin, v_end)),
                    branch_currents=tuple(float(current_buf[i]) for i in range(i_begin, i_end)),
                    pin_digital=tuple(DigitalState(int(digital_buf[i])) for i in range(d_begin, d_end)),
                )
            )

        return ExperimentCircuitSample(
            components=tuple(components),
            voltage_ord=tuple(voltage_ord),
            current_ord=tuple(current_ord),
            digital_ord=tuple(digital_ord),
        )

    def analyze_and_sample(self) -> ExperimentCircuitSample:
        self.analyze()
        return self.sample()


def circuit_to_experiment(
    circuit: "Circuit",
    *,
    fixed_pos: Position | Sequence[float] = Position(0.0, 0.0, 0.0),
    element_xyz_coords: bool = False,
    keep_pl_macros: bool = True,
    include_linear: bool = False,
    include_ground: bool = False,
    generate_wires: bool = True,
    keep_unknown_as_placeholders: bool = False,
    drop_dangling_logic_inputs: bool = False,
    library: str | None = None,
) -> Experiment:
    lib = getattr(circuit, "_lib", None) if library is None else load_library(library)
    if lib is None:
        lib = load_library(library)
    pos = _as_position(fixed_pos)
    handle = circuit._require_handle()

    experiment_handle = lib.pe_to_pl_convert(
        handle,
        pos.x,
        pos.y,
        pos.z,
        int(element_xyz_coords),
        int(keep_pl_macros),
        int(include_linear),
        int(include_ground),
        int(generate_wires),
        int(keep_unknown_as_placeholders),
        int(drop_dangling_logic_inputs),
    )
    if not experiment_handle:
        raise_last_error("pe_to_pl_convert returned null", lib)
    return Experiment._from_handle(experiment_handle, lib)


convert_circuit_to_experiment = circuit_to_experiment


__all__ = [
    "AutoLayoutBackend",
    "AutoLayoutMode",
    "AutoLayoutStats",
    "ElementKeyMode",
    "ExperimentElement",
    "Experiment",
    "ExperimentCircuit",
    "ExperimentCircuitSample",
    "ExperimentComponentSample",
    "ExperimentType",
    "Position",
    "WriteBackOptions",
    "WireColor",
    "circuit_to_experiment",
    "convert_circuit_to_experiment",
]

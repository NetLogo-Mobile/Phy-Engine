from __future__ import annotations

import ctypes as ct
import os
import threading
from ctypes.util import find_library
from pathlib import Path
from typing import Iterable


class PhyEngineError(RuntimeError):
    pass


_LOCK = threading.Lock()
_LIB: ct.CDLL | None = None


def _candidate_library_paths() -> Iterable[Path]:
    env = os.environ.get("PHY_ENGINE_LIB")
    if env:
        yield Path(env).expanduser()

    root = Path(__file__).resolve().parents[2]
    names = ("libphyengine.dylib", "libphyengine.so", "phyengine.dll")
    dirs = (
        root / "build_py",
        root / "build",
        root / "src" / "build_py",
        root / "src" / "build",
    )
    subdirs = ("", "Release", "Debug", "RelWithDebInfo", "MinSizeRel")

    for directory in dirs:
        for subdir in subdirs:
            base = directory / subdir if subdir else directory
            for name in names:
                yield base / name


def _configure_library(lib: ct.CDLL) -> ct.CDLL:
    c_bool = ct.c_bool
    c_char_pp = ct.POINTER(ct.c_char_p)
    c_double_p = ct.POINTER(ct.c_double)
    c_int_p = ct.POINTER(ct.c_int)
    c_size_p = ct.POINTER(ct.c_size_t)
    c_u8_p = ct.POINTER(ct.c_uint8)

    lib.phy_engine_last_error.argtypes = []
    lib.phy_engine_last_error.restype = ct.c_char_p
    lib.phy_engine_clear_error.argtypes = []
    lib.phy_engine_clear_error.restype = None
    lib.phy_engine_string_free.argtypes = [ct.c_void_p]
    lib.phy_engine_string_free.restype = None

    lib.create_circuit.argtypes = [
        c_int_p,
        ct.c_size_t,
        c_int_p,
        ct.c_size_t,
        c_double_p,
        ct.POINTER(c_size_p),
        ct.POINTER(c_size_p),
        c_size_p,
    ]
    lib.create_circuit.restype = ct.c_void_p

    lib.create_circuit_ex.argtypes = [
        c_int_p,
        ct.c_size_t,
        c_int_p,
        ct.c_size_t,
        c_double_p,
        c_char_pp,
        c_size_p,
        ct.c_size_t,
        c_size_p,
        c_size_p,
        ct.POINTER(c_size_p),
        ct.POINTER(c_size_p),
        c_size_p,
    ]
    lib.create_circuit_ex.restype = ct.c_void_p

    lib.destroy_circuit.argtypes = [ct.c_void_p, c_size_p, c_size_p]
    lib.destroy_circuit.restype = None

    lib.circuit_set_analyze_type.argtypes = [ct.c_void_p, ct.c_uint32]
    lib.circuit_set_analyze_type.restype = ct.c_int
    lib.circuit_set_tr.argtypes = [ct.c_void_p, ct.c_double, ct.c_double]
    lib.circuit_set_tr.restype = ct.c_int
    lib.circuit_set_ac_omega.argtypes = [ct.c_void_p, ct.c_double]
    lib.circuit_set_ac_omega.restype = ct.c_int
    lib.circuit_set_temperature.argtypes = [ct.c_void_p, ct.c_double]
    lib.circuit_set_temperature.restype = ct.c_int
    lib.circuit_set_tnom.argtypes = [ct.c_void_p, ct.c_double]
    lib.circuit_set_tnom.restype = ct.c_int
    lib.circuit_set_model_double_by_name.argtypes = [
        ct.c_void_p,
        ct.c_size_t,
        ct.c_size_t,
        ct.c_char_p,
        ct.c_size_t,
        ct.c_double,
    ]
    lib.circuit_set_model_double_by_name.restype = ct.c_int
    lib.circuit_analyze.argtypes = [ct.c_void_p]
    lib.circuit_analyze.restype = ct.c_int
    lib.circuit_digital_clk.argtypes = [ct.c_void_p]
    lib.circuit_digital_clk.restype = ct.c_int
    lib.circuit_sample_layout.argtypes = [
        ct.c_void_p,
        c_size_p,
        c_size_p,
        ct.c_size_t,
        c_size_p,
        c_size_p,
        c_size_p,
    ]
    lib.circuit_sample_layout.restype = ct.c_int
    lib.circuit_sample_u8.argtypes = [
        ct.c_void_p,
        c_size_p,
        c_size_p,
        ct.c_size_t,
        c_double_p,
        c_size_p,
        c_double_p,
        c_size_p,
        c_u8_p,
        c_size_p,
    ]
    lib.circuit_sample_u8.restype = ct.c_int
    lib.circuit_sample_digital_state_u8.argtypes = [
        ct.c_void_p,
        c_size_p,
        c_size_p,
        ct.c_size_t,
        c_double_p,
        c_size_p,
        c_double_p,
        c_size_p,
        c_u8_p,
        c_size_p,
    ]
    lib.circuit_sample_digital_state_u8.restype = ct.c_int
    lib.circuit_set_model_digital.argtypes = [ct.c_void_p, ct.c_size_t, ct.c_size_t, ct.c_size_t, ct.c_uint8]
    lib.circuit_set_model_digital.restype = ct.c_int

    lib.verilog_synth_set_opt_level.argtypes = [ct.c_uint8]
    lib.verilog_synth_set_opt_level.restype = None
    lib.verilog_synth_get_opt_level.argtypes = []
    lib.verilog_synth_get_opt_level.restype = ct.c_uint8
    lib.verilog_synth_set_assume_binary_inputs.argtypes = [c_bool]
    lib.verilog_synth_set_assume_binary_inputs.restype = None
    lib.verilog_synth_get_assume_binary_inputs.argtypes = []
    lib.verilog_synth_get_assume_binary_inputs.restype = c_bool
    lib.verilog_synth_set_allow_inout.argtypes = [c_bool]
    lib.verilog_synth_set_allow_inout.restype = None
    lib.verilog_synth_get_allow_inout.argtypes = []
    lib.verilog_synth_get_allow_inout.restype = c_bool
    lib.verilog_synth_set_allow_multi_driver.argtypes = [c_bool]
    lib.verilog_synth_set_allow_multi_driver.restype = None
    lib.verilog_synth_get_allow_multi_driver.argtypes = []
    lib.verilog_synth_get_allow_multi_driver.restype = c_bool
    lib.verilog_synth_set_optimize_wires.argtypes = [c_bool]
    lib.verilog_synth_set_optimize_wires.restype = None
    lib.verilog_synth_get_optimize_wires.argtypes = []
    lib.verilog_synth_get_optimize_wires.restype = c_bool
    lib.verilog_synth_set_optimize_mul2.argtypes = [c_bool]
    lib.verilog_synth_set_optimize_mul2.restype = None
    lib.verilog_synth_get_optimize_mul2.argtypes = []
    lib.verilog_synth_get_optimize_mul2.restype = c_bool
    lib.verilog_synth_set_optimize_adders.argtypes = [c_bool]
    lib.verilog_synth_set_optimize_adders.restype = None
    lib.verilog_synth_get_optimize_adders.argtypes = []
    lib.verilog_synth_get_optimize_adders.restype = c_bool
    lib.verilog_synth_set_loop_unroll_limit.argtypes = [ct.c_size_t]
    lib.verilog_synth_set_loop_unroll_limit.restype = None
    lib.verilog_synth_get_loop_unroll_limit.argtypes = []
    lib.verilog_synth_get_loop_unroll_limit.restype = ct.c_size_t

    lib.verilog_runtime_create.argtypes = [
        ct.c_char_p,
        ct.c_size_t,
        ct.c_char_p,
        ct.c_size_t,
        c_char_pp,
        c_size_p,
        ct.c_size_t,
    ]
    lib.verilog_runtime_create.restype = ct.c_void_p
    lib.verilog_runtime_destroy.argtypes = [ct.c_void_p]
    lib.verilog_runtime_destroy.restype = None
    lib.verilog_runtime_get_tick.argtypes = [ct.c_void_p]
    lib.verilog_runtime_get_tick.restype = ct.c_uint64
    lib.verilog_runtime_reset.argtypes = [ct.c_void_p]
    lib.verilog_runtime_reset.restype = ct.c_int
    lib.verilog_runtime_step.argtypes = [ct.c_void_p, ct.c_uint64, ct.c_uint8]
    lib.verilog_runtime_step.restype = ct.c_int
    lib.verilog_runtime_tick.argtypes = [ct.c_void_p]
    lib.verilog_runtime_tick.restype = ct.c_int

    lib.verilog_runtime_module_count.argtypes = [ct.c_void_p]
    lib.verilog_runtime_module_count.restype = ct.c_size_t
    lib.verilog_runtime_port_count.argtypes = [ct.c_void_p]
    lib.verilog_runtime_port_count.restype = ct.c_size_t
    lib.verilog_runtime_signal_count.argtypes = [ct.c_void_p]
    lib.verilog_runtime_signal_count.restype = ct.c_size_t
    lib.verilog_runtime_preprocessed_size.argtypes = [ct.c_void_p]
    lib.verilog_runtime_preprocessed_size.restype = ct.c_size_t
    lib.verilog_runtime_copy_preprocessed.argtypes = [ct.c_void_p, ct.c_char_p, ct.c_size_t]
    lib.verilog_runtime_copy_preprocessed.restype = ct.c_int
    lib.verilog_runtime_top_module_name_size.argtypes = [ct.c_void_p]
    lib.verilog_runtime_top_module_name_size.restype = ct.c_size_t
    lib.verilog_runtime_copy_top_module_name.argtypes = [ct.c_void_p, ct.c_char_p, ct.c_size_t]
    lib.verilog_runtime_copy_top_module_name.restype = ct.c_int
    lib.verilog_runtime_module_name_size.argtypes = [ct.c_void_p, ct.c_size_t]
    lib.verilog_runtime_module_name_size.restype = ct.c_size_t
    lib.verilog_runtime_copy_module_name.argtypes = [ct.c_void_p, ct.c_size_t, ct.c_char_p, ct.c_size_t]
    lib.verilog_runtime_copy_module_name.restype = ct.c_int
    lib.verilog_runtime_port_name_size.argtypes = [ct.c_void_p, ct.c_size_t]
    lib.verilog_runtime_port_name_size.restype = ct.c_size_t
    lib.verilog_runtime_copy_port_name.argtypes = [ct.c_void_p, ct.c_size_t, ct.c_char_p, ct.c_size_t]
    lib.verilog_runtime_copy_port_name.restype = ct.c_int
    lib.verilog_runtime_port_dir.argtypes = [ct.c_void_p, ct.c_size_t]
    lib.verilog_runtime_port_dir.restype = ct.c_uint8
    lib.verilog_runtime_get_port_value.argtypes = [ct.c_void_p, ct.c_size_t]
    lib.verilog_runtime_get_port_value.restype = ct.c_uint8
    lib.verilog_runtime_set_port_value.argtypes = [ct.c_void_p, ct.c_size_t, ct.c_uint8]
    lib.verilog_runtime_set_port_value.restype = ct.c_int
    lib.verilog_runtime_signal_name_size.argtypes = [ct.c_void_p, ct.c_size_t]
    lib.verilog_runtime_signal_name_size.restype = ct.c_size_t
    lib.verilog_runtime_copy_signal_name.argtypes = [ct.c_void_p, ct.c_size_t, ct.c_char_p, ct.c_size_t]
    lib.verilog_runtime_copy_signal_name.restype = ct.c_int
    lib.verilog_runtime_get_signal_value.argtypes = [ct.c_void_p, ct.c_size_t]
    lib.verilog_runtime_get_signal_value.restype = ct.c_uint8
    lib.verilog_runtime_set_signal_value.argtypes = [ct.c_void_p, ct.c_size_t, ct.c_uint8]
    lib.verilog_runtime_set_signal_value.restype = ct.c_int

    lib.pl_experiment_create.argtypes = [ct.c_int]
    lib.pl_experiment_create.restype = ct.c_void_p
    lib.pl_experiment_load_from_string.argtypes = [ct.c_char_p, ct.c_size_t]
    lib.pl_experiment_load_from_string.restype = ct.c_void_p
    lib.pl_experiment_load_from_file.argtypes = [ct.c_char_p, ct.c_size_t]
    lib.pl_experiment_load_from_file.restype = ct.c_void_p
    lib.pl_experiment_destroy.argtypes = [ct.c_void_p]
    lib.pl_experiment_destroy.restype = None
    lib.pl_experiment_dump.argtypes = [ct.c_void_p, ct.c_int]
    lib.pl_experiment_dump.restype = ct.c_void_p
    lib.pl_experiment_save.argtypes = [ct.c_void_p, ct.c_char_p, ct.c_size_t, ct.c_int]
    lib.pl_experiment_save.restype = ct.c_int
    lib.pl_experiment_add_circuit_element.argtypes = [
        ct.c_void_p,
        ct.c_char_p,
        ct.c_size_t,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_uint8,
        ct.c_uint8,
        ct.c_uint8,
    ]
    lib.pl_experiment_add_circuit_element.restype = ct.c_void_p
    lib.pl_experiment_connect.argtypes = [
        ct.c_void_p,
        ct.c_char_p,
        ct.c_size_t,
        ct.c_int,
        ct.c_char_p,
        ct.c_size_t,
        ct.c_int,
        ct.c_int,
    ]
    lib.pl_experiment_connect.restype = ct.c_int
    lib.pl_experiment_clear_wires.argtypes = [ct.c_void_p]
    lib.pl_experiment_clear_wires.restype = ct.c_int
    lib.pl_experiment_set_xyz_precision.argtypes = [ct.c_void_p, ct.c_int]
    lib.pl_experiment_set_xyz_precision.restype = ct.c_int
    lib.pl_experiment_set_element_xyz.argtypes = [
        ct.c_void_p,
        ct.c_uint8,
        ct.c_double,
        ct.c_double,
        ct.c_double,
    ]
    lib.pl_experiment_set_element_xyz.restype = ct.c_int
    lib.pl_experiment_set_camera.argtypes = [
        ct.c_void_p,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_double,
    ]
    lib.pl_experiment_set_camera.restype = ct.c_int
    lib.pl_experiment_set_element_property_number.argtypes = [
        ct.c_void_p,
        ct.c_char_p,
        ct.c_size_t,
        ct.c_char_p,
        ct.c_size_t,
        ct.c_double,
    ]
    lib.pl_experiment_set_element_property_number.restype = ct.c_int
    lib.pl_experiment_set_element_label.argtypes = [
        ct.c_void_p,
        ct.c_char_p,
        ct.c_size_t,
        ct.c_char_p,
        ct.c_size_t,
    ]
    lib.pl_experiment_set_element_label.restype = ct.c_int
    lib.pl_experiment_set_element_position.argtypes = [
        ct.c_void_p,
        ct.c_char_p,
        ct.c_size_t,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_uint8,
    ]
    lib.pl_experiment_set_element_position.restype = ct.c_int
    lib.pl_experiment_merge.argtypes = [
        ct.c_void_p,
        ct.c_void_p,
        ct.c_double,
        ct.c_double,
        ct.c_double,
    ]
    lib.pl_experiment_merge.restype = ct.c_int

    lib.pl_pe_circuit_build.argtypes = [ct.c_void_p]
    lib.pl_pe_circuit_build.restype = ct.c_void_p
    lib.pl_pe_circuit_destroy.argtypes = [ct.c_void_p]
    lib.pl_pe_circuit_destroy.restype = None
    lib.pl_pe_circuit_comp_size.argtypes = [ct.c_void_p]
    lib.pl_pe_circuit_comp_size.restype = ct.c_size_t
    lib.pl_pe_circuit_set_analyze_type.argtypes = [ct.c_void_p, ct.c_uint32]
    lib.pl_pe_circuit_set_analyze_type.restype = ct.c_int
    lib.pl_pe_circuit_set_tr.argtypes = [ct.c_void_p, ct.c_double, ct.c_double]
    lib.pl_pe_circuit_set_tr.restype = ct.c_int
    lib.pl_pe_circuit_set_ac_omega.argtypes = [ct.c_void_p, ct.c_double]
    lib.pl_pe_circuit_set_ac_omega.restype = ct.c_int
    lib.pl_pe_circuit_analyze.argtypes = [ct.c_void_p]
    lib.pl_pe_circuit_analyze.restype = ct.c_int
    lib.pl_pe_circuit_digital_clk.argtypes = [ct.c_void_p]
    lib.pl_pe_circuit_digital_clk.restype = ct.c_int
    lib.pl_pe_circuit_sync_inputs_from_pl.argtypes = [ct.c_void_p, ct.c_void_p]
    lib.pl_pe_circuit_sync_inputs_from_pl.restype = ct.c_int
    lib.pl_pe_circuit_write_back_to_pl.argtypes = [ct.c_void_p, ct.c_void_p]
    lib.pl_pe_circuit_write_back_to_pl.restype = ct.c_int
    lib.pl_pe_circuit_write_back_to_pl_ex.argtypes = [
        ct.c_void_p,
        ct.c_void_p,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_double,
    ]
    lib.pl_pe_circuit_write_back_to_pl_ex.restype = ct.c_int
    lib.pl_pe_circuit_sample_layout.argtypes = [ct.c_void_p, c_size_p, c_size_p, c_size_p]
    lib.pl_pe_circuit_sample_layout.restype = ct.c_int
    lib.pl_pe_circuit_sample_u8.argtypes = [
        ct.c_void_p,
        c_double_p,
        c_size_p,
        c_double_p,
        c_size_p,
        c_u8_p,
        c_size_p,
    ]
    lib.pl_pe_circuit_sample_u8.restype = ct.c_int
    lib.pl_pe_circuit_sample_digital_state_u8.argtypes = [
        ct.c_void_p,
        c_double_p,
        c_size_p,
        c_double_p,
        c_size_p,
        c_u8_p,
        c_size_p,
    ]
    lib.pl_pe_circuit_sample_digital_state_u8.restype = ct.c_int

    lib.pe_to_pl_convert.argtypes = [
        ct.c_void_p,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_uint8,
        ct.c_uint8,
        ct.c_uint8,
        ct.c_uint8,
        ct.c_uint8,
        ct.c_uint8,
        ct.c_uint8,
    ]
    lib.pe_to_pl_convert.restype = ct.c_void_p

    lib.pl_experiment_auto_layout.argtypes = [
        ct.c_void_p,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_int,
        ct.c_int,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        ct.c_double,
        c_size_p,
        c_size_p,
        c_size_p,
        c_size_p,
        c_size_p,
    ]
    lib.pl_experiment_auto_layout.restype = ct.c_int

    return lib


def load_library(path: str | os.PathLike[str] | None = None) -> ct.CDLL:
    global _LIB
    with _LOCK:
        if path is None and _LIB is not None:
            return _LIB

        candidates: list[str] = []
        if path is not None:
            candidates.append(str(Path(path).expanduser()))
        else:
            candidates.extend(str(candidate) for candidate in _candidate_library_paths() if candidate.exists())
            found = find_library("phyengine")
            if found:
                candidates.append(found)

        last_exc: Exception | None = None
        for candidate in candidates:
            try:
                lib = _configure_library(ct.CDLL(candidate))
                if path is None:
                    _LIB = lib
                return lib
            except OSError as exc:
                last_exc = exc

        searched = "\n".join(candidates) if candidates else "  <none>"
        raise PhyEngineError(f"Unable to load phyengine shared library.\nSearched:\n{searched}") from last_exc


def last_error(lib: ct.CDLL | None = None) -> str:
    active = load_library() if lib is None else lib
    raw = active.phy_engine_last_error()
    return raw.decode("utf-8", errors="replace") if raw else ""


def clear_error(lib: ct.CDLL | None = None) -> None:
    active = load_library() if lib is None else lib
    active.phy_engine_clear_error()


def raise_last_error(prefix: str, lib: ct.CDLL | None = None) -> None:
    message = last_error(lib)
    if message:
        raise PhyEngineError(f"{prefix}: {message}")
    raise PhyEngineError(prefix)


def check_rc(rc: int, func: str, lib: ct.CDLL | None = None) -> None:
    if rc != 0:
        raise_last_error(f"{func} failed with rc={rc}", lib)


def take_owned_c_string(ptr: int | ct.c_void_p | None, lib: ct.CDLL | None = None) -> str:
    if not ptr:
        return ""
    active = load_library() if lib is None else lib
    raw_ptr = int(ptr)
    try:
        return ct.string_at(raw_ptr).decode("utf-8", errors="replace")
    finally:
        active.phy_engine_string_free(ct.c_void_p(raw_ptr))


__all__ = [
    "PhyEngineError",
    "check_rc",
    "clear_error",
    "last_error",
    "load_library",
    "raise_last_error",
    "take_owned_c_string",
]

from __future__ import annotations

import ctypes as ct
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import Sequence

from ._ffi import PhyEngineError, check_rc, load_library, raise_last_error
from .circuit import DigitalState


class VerilogPortDir(IntEnum):
    UNKNOWN = 0
    INPUT = 1
    OUTPUT = 2
    INOUT = 3


@dataclass(frozen=True)
class VerilogPort:
    index: int
    name: str
    direction: VerilogPortDir


@dataclass(frozen=True)
class VerilogSynthConfig:
    opt_level: int
    assume_binary_inputs: bool
    allow_inout: bool
    allow_multi_driver: bool
    optimize_wires: bool
    optimize_mul2: bool
    optimize_adders: bool
    loop_unroll_limit: int


def _copy_string(lib, size_fn, copy_fn, handle: ct.c_void_p, *args) -> str:
    size = int(size_fn(handle, *args))
    buf = ct.create_string_buffer(size + 1)
    check_rc(copy_fn(handle, *args, buf, len(buf)), copy_fn.__name__, lib)
    return buf.value.decode("utf-8", errors="replace")


def get_verilog_synth_config(*, library: str | None = None) -> VerilogSynthConfig:
    lib = load_library(library)
    return VerilogSynthConfig(
        opt_level=int(lib.verilog_synth_get_opt_level()),
        assume_binary_inputs=bool(lib.verilog_synth_get_assume_binary_inputs()),
        allow_inout=bool(lib.verilog_synth_get_allow_inout()),
        allow_multi_driver=bool(lib.verilog_synth_get_allow_multi_driver()),
        optimize_wires=bool(lib.verilog_synth_get_optimize_wires()),
        optimize_mul2=bool(lib.verilog_synth_get_optimize_mul2()),
        optimize_adders=bool(lib.verilog_synth_get_optimize_adders()),
        loop_unroll_limit=int(lib.verilog_synth_get_loop_unroll_limit()),
    )


def set_verilog_synth_config(
    *,
    opt_level: int | None = None,
    assume_binary_inputs: bool | None = None,
    allow_inout: bool | None = None,
    allow_multi_driver: bool | None = None,
    optimize_wires: bool | None = None,
    optimize_mul2: bool | None = None,
    optimize_adders: bool | None = None,
    loop_unroll_limit: int | None = None,
    library: str | None = None,
) -> None:
    lib = load_library(library)
    if opt_level is not None:
        lib.verilog_synth_set_opt_level(opt_level)
    if assume_binary_inputs is not None:
        lib.verilog_synth_set_assume_binary_inputs(assume_binary_inputs)
    if allow_inout is not None:
        lib.verilog_synth_set_allow_inout(allow_inout)
    if allow_multi_driver is not None:
        lib.verilog_synth_set_allow_multi_driver(allow_multi_driver)
    if optimize_wires is not None:
        lib.verilog_synth_set_optimize_wires(optimize_wires)
    if optimize_mul2 is not None:
        lib.verilog_synth_set_optimize_mul2(optimize_mul2)
    if optimize_adders is not None:
        lib.verilog_synth_set_optimize_adders(optimize_adders)
    if loop_unroll_limit is not None:
        lib.verilog_synth_set_loop_unroll_limit(loop_unroll_limit)


class VerilogRuntime:
    def __init__(
        self,
        source: str,
        *,
        top: str | None = None,
        include_dirs: Sequence[str | Path] = (),
        library: str | None = None,
    ) -> None:
        self._lib = load_library(library)
        source_bytes = source.encode("utf-8")
        top_bytes = top.encode("utf-8") if top is not None else b""
        include_dir_bytes = [str(Path(path)).encode("utf-8") for path in include_dirs]
        include_dir_arr = (ct.c_char_p * max(1, len(include_dir_bytes)))(*include_dir_bytes) if include_dir_bytes else (ct.c_char_p * 1)()
        include_dir_sizes = (ct.c_size_t * max(1, len(include_dir_bytes)))(*[len(item) for item in include_dir_bytes]) if include_dir_bytes else (ct.c_size_t * 1)(0)

        handle = self._lib.verilog_runtime_create(
            source_bytes,
            len(source_bytes),
            top_bytes if top is not None else None,
            len(top_bytes),
            include_dir_arr,
            include_dir_sizes,
            len(include_dir_bytes),
        )
        if not handle:
            raise_last_error("verilog_runtime_create returned null", self._lib)

        self._handle = ct.c_void_p(handle)
        self._ports_cache: tuple[VerilogPort, ...] | None = None
        self._signals_cache: tuple[str, ...] | None = None
        self._modules_cache: tuple[str, ...] | None = None

    @classmethod
    def from_file(
        cls,
        path: str | Path,
        *,
        top: str | None = None,
        include_dirs: Sequence[str | Path] = (),
        library: str | None = None,
    ) -> "VerilogRuntime":
        file_path = Path(path)
        source = file_path.read_text(encoding="utf-8")
        merged_include_dirs = (file_path.parent, *include_dirs)
        return cls(source, top=top, include_dirs=merged_include_dirs, library=library)

    def close(self) -> None:
        if getattr(self, "_handle", None):
            self._lib.verilog_runtime_destroy(self._handle)
            self._handle = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    def __enter__(self) -> "VerilogRuntime":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _require_handle(self) -> ct.c_void_p:
        if not self._handle:
            raise PhyEngineError("VerilogRuntime is already closed")
        return self._handle

    def _resolve_port_index(self, port: int | str) -> int:
        if isinstance(port, int):
            return port
        mapping = {item.name: item.index for item in self.ports}
        if port not in mapping:
            raise KeyError(f"unknown port: {port}")
        return mapping[port]

    def _resolve_signal_index(self, signal: int | str) -> int:
        if isinstance(signal, int):
            return signal
        mapping = {name: index for index, name in enumerate(self.signals)}
        if signal not in mapping:
            raise KeyError(f"unknown signal: {signal}")
        return mapping[signal]

    @property
    def tick(self) -> int:
        return int(self._lib.verilog_runtime_get_tick(self._require_handle()))

    @property
    def top_module_name(self) -> str:
        return _copy_string(
            self._lib,
            self._lib.verilog_runtime_top_module_name_size,
            self._lib.verilog_runtime_copy_top_module_name,
            self._require_handle(),
        )

    @property
    def preprocessed_source(self) -> str:
        return _copy_string(
            self._lib,
            self._lib.verilog_runtime_preprocessed_size,
            self._lib.verilog_runtime_copy_preprocessed,
            self._require_handle(),
        )

    @property
    def module_names(self) -> tuple[str, ...]:
        if self._modules_cache is None:
            handle = self._require_handle()
            count = int(self._lib.verilog_runtime_module_count(handle))
            self._modules_cache = tuple(
                _copy_string(
                    self._lib,
                    self._lib.verilog_runtime_module_name_size,
                    self._lib.verilog_runtime_copy_module_name,
                    handle,
                    index,
                )
                for index in range(count)
            )
        return self._modules_cache

    @property
    def ports(self) -> tuple[VerilogPort, ...]:
        if self._ports_cache is None:
            handle = self._require_handle()
            count = int(self._lib.verilog_runtime_port_count(handle))
            self._ports_cache = tuple(
                VerilogPort(
                    index=index,
                    name=_copy_string(
                        self._lib,
                        self._lib.verilog_runtime_port_name_size,
                        self._lib.verilog_runtime_copy_port_name,
                        handle,
                        index,
                    ),
                    direction=VerilogPortDir(int(self._lib.verilog_runtime_port_dir(handle, index))),
                )
                for index in range(count)
            )
        return self._ports_cache

    @property
    def signals(self) -> tuple[str, ...]:
        if self._signals_cache is None:
            handle = self._require_handle()
            count = int(self._lib.verilog_runtime_signal_count(handle))
            self._signals_cache = tuple(
                _copy_string(
                    self._lib,
                    self._lib.verilog_runtime_signal_name_size,
                    self._lib.verilog_runtime_copy_signal_name,
                    handle,
                    index,
                )
                for index in range(count)
            )
        return self._signals_cache

    def reset(self) -> None:
        check_rc(self._lib.verilog_runtime_reset(self._require_handle()), "verilog_runtime_reset", self._lib)

    def step(self, tick: int, *, process_sequential: bool = True) -> None:
        check_rc(
            self._lib.verilog_runtime_step(self._require_handle(), tick, int(process_sequential)),
            "verilog_runtime_step",
            self._lib,
        )

    def tick_once(self) -> None:
        check_rc(self._lib.verilog_runtime_tick(self._require_handle()), "verilog_runtime_tick", self._lib)

    def get_port(self, port: int | str) -> DigitalState:
        index = self._resolve_port_index(port)
        return DigitalState(int(self._lib.verilog_runtime_get_port_value(self._require_handle(), index)))

    def set_port(self, port: int | str, state: DigitalState | int) -> None:
        index = self._resolve_port_index(port)
        check_rc(
            self._lib.verilog_runtime_set_port_value(self._require_handle(), index, int(state)),
            "verilog_runtime_set_port_value",
            self._lib,
        )

    def get_signal(self, signal: int | str) -> DigitalState:
        index = self._resolve_signal_index(signal)
        return DigitalState(int(self._lib.verilog_runtime_get_signal_value(self._require_handle(), index)))

    def set_signal(self, signal: int | str, state: DigitalState | int) -> None:
        index = self._resolve_signal_index(signal)
        check_rc(
            self._lib.verilog_runtime_set_signal_value(self._require_handle(), index, int(state)),
            "verilog_runtime_set_signal_value",
            self._lib,
        )

    def snapshot_ports(self) -> dict[str, DigitalState]:
        return {port.name: self.get_port(port.index) for port in self.ports}

    def snapshot_signals(self) -> dict[str, DigitalState]:
        return {name: self.get_signal(index) for index, name in enumerate(self.signals)}


__all__ = [
    "VerilogPort",
    "VerilogPortDir",
    "VerilogRuntime",
    "VerilogSynthConfig",
    "get_verilog_synth_config",
    "set_verilog_synth_config",
]

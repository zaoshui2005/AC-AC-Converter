#!/usr/bin/env python3
"""Live SRAM waveform viewer for OpenOCD's TCL interface.

The tool intentionally uses the TCL port instead of the GDB port so it can run
beside Ozone without taking over the debug session.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import queue
import re
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


DEFAULT_CHANNELS = [
    "DebugWatchData.pfc.input_voltage_v",
    "DebugWatchData.pfc.input_current_a",
    "DebugWatchData.pfc.dc_bus_v",
    "DebugWatchData.pfc.modulation",
]

COLORS = ["#1677ff", "#d9485f", "#14866d", "#d97706", "#7c3aed", "#0891b2"]
TCL_EOF = b"\x1a"


class WaveError(RuntimeError):
    pass


@dataclass(frozen=True)
class Channel:
    expression: str
    address: int
    size: int
    type_name: str
    kind: str
    signed: bool


def find_gdb(explicit: Optional[str]) -> str:
    candidates = [
        explicit,
        shutil.which("arm-none-eabi-gdb"),
        r"D:\ST\STM32CubeCLT_1.18.0\GNU-tools-for-STM32\bin\arm-none-eabi-gdb.exe",
        r"D:\DevEnv\GNU-tools-for-STM32\bin\arm-none-eabi-gdb.exe",
    ]
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return str(Path(candidate))
    raise WaveError("arm-none-eabi-gdb.exe was not found. Use --gdb to specify it.")


def resolve_expression(gdb: str, elf: str, expression: str) -> Channel:
    if any(ch in expression for ch in "\r\n;"):
        raise WaveError("Expressions cannot contain newlines or semicolons.")

    commands = [
        "set language c",
        f'printf "OPENOCD_WAVE_ADDR=0x%lx\\n", (unsigned long)&({expression})',
        f'printf "OPENOCD_WAVE_SIZE=%u\\n", (unsigned int)sizeof({expression})',
        f"whatis {expression}",
    ]
    args = [gdb, "--batch", "-q", elf]
    for command in commands:
        args.extend(["-ex", command])

    result = subprocess.run(
        args,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
    )
    output = result.stdout + result.stderr
    address_match = re.search(r"OPENOCD_WAVE_ADDR=(0x[0-9a-fA-F]+)", output)
    size_match = re.search(r"OPENOCD_WAVE_SIZE=(\d+)", output)
    type_matches = re.findall(r"^type\s*=\s*(.+)$", output, flags=re.MULTILINE)
    if result.returncode or not address_match or not size_match or not type_matches:
        detail = output.strip().splitlines()[-1] if output.strip() else "unknown GDB error"
        raise WaveError(f"Cannot resolve '{expression}': {detail}")

    size = int(size_match.group(1))
    type_name = type_matches[-1].strip()
    normalized = re.sub(r"\b(const|volatile|restrict)\b", "", type_name)
    normalized = " ".join(normalized.split()).lower()

    if normalized == "float" and size == 4:
        kind, signed = "float", True
    elif normalized in {"double", "long double"} and size == 8:
        kind, signed = "double", True
    elif size in {1, 2, 4, 8}:
        kind = "integer"
        signed = "unsigned" not in normalized and not normalized.startswith("uint")
    else:
        raise WaveError(
            f"'{expression}' has unsupported type '{type_name}' ({size} bytes). "
            "Select a scalar member of 1, 2, 4, or 8 bytes."
        )

    return Channel(
        expression=expression,
        address=int(address_match.group(1), 16),
        size=size,
        type_name=type_name,
        kind=kind,
        signed=signed,
    )


class OpenOcdTcl:
    def __init__(self, host: str, port: int, timeout: float = 2.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None

    def connect(self) -> None:
        self.close()
        self.sock = socket.create_connection((self.host, self.port), self.timeout)
        self.sock.settimeout(self.timeout)

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def command(self, command: str) -> str:
        if self.sock is None:
            raise WaveError("OpenOCD TCL connection is not open.")
        payload = f'capture "{command}"'.encode("ascii") + TCL_EOF
        self.sock.sendall(payload)
        chunks = []
        while True:
            chunk = self.sock.recv(8192)
            if not chunk:
                raise WaveError("OpenOCD closed the TCL connection.")
            chunks.append(chunk)
            if TCL_EOF in chunk:
                break
        response = b"".join(chunks).split(TCL_EOF, 1)[0].decode("utf-8", errors="replace").strip()
        lower = response.lower()
        if "error" in lower or "invalid command" in lower or "failed" in lower:
            raise WaveError(response)
        return response

    def target_state(self) -> str:
        response = self.command("targets")
        match = re.search(r"\b(running|halted|reset|unknown)\b", response, re.IGNORECASE)
        return match.group(1).lower() if match else "unknown"

    @staticmethod
    def _decode(channel: Channel, raw: bytes) -> float:
        if channel.kind == "float":
            return float(struct.unpack("<f", raw)[0])
        if channel.kind == "double":
            return float(struct.unpack("<d", raw)[0])
        return float(int.from_bytes(raw, "little", signed=channel.signed))

    def read_channels(self, channels: list[Channel]) -> list[float]:
        if not channels:
            return []

        # Nearby scalar values are read in one SWD transaction. Limit each
        # group so unrelated symbols cannot cause a large SRAM transfer.
        indexed = sorted(enumerate(channels), key=lambda item: item[1].address)
        groups: list[tuple[int, int, list[tuple[int, Channel]]]] = []
        for original_index, channel in indexed:
            start = channel.address & ~3
            end = (channel.address + channel.size + 3) & ~3
            if groups and start - groups[-1][1] <= 16 and end - groups[-1][0] <= 256:
                group_start, group_end, members = groups[-1]
                groups[-1] = (group_start, max(group_end, end), members + [(original_index, channel)])
            else:
                groups.append((start, end, [(original_index, channel)]))

        values = [math.nan] * len(channels)
        for start, end, members in groups:
            count = (end - start) // 4
            response = self.command(f"read_memory 0x{start:x} 32 {count}")
            words = [int(value, 16) for value in re.findall(r"0x[0-9a-fA-F]+", response)]
            if len(words) < count:
                raise WaveError(f"Unexpected OpenOCD response: {response!r}")
            block = b"".join(word.to_bytes(4, "little") for word in words[:count])
            for original_index, channel in members:
                offset = channel.address - start
                raw = block[offset : offset + channel.size]
                if len(raw) != channel.size:
                    raise WaveError(f"Short memory block while reading {channel.expression}")
                values[original_index] = self._decode(channel, raw)
        return values

    def read_channel(self, channel: Channel) -> float:
        return self.read_channels([channel])[0]


def probe(args: argparse.Namespace, channels: list[Channel]) -> int:
    client = OpenOcdTcl(args.host, args.port)
    try:
        client.connect()
        print(f"OpenOCD: {args.host}:{args.port}")
        print(f"Target:  {client.target_state()}")
        values = client.read_channels(channels)
        for channel, value in zip(channels, values):
            print(
                f"{channel.expression} = {value:.9g} "
                f"({channel.type_name}, 0x{channel.address:08X})"
            )
        return 0
    except (OSError, WaveError) as exc:
        print(f"Probe failed: {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()


def benchmark(args: argparse.Namespace, channels: list[Channel]) -> int:
    client = OpenOcdTcl(args.host, args.port)
    try:
        client.connect()
        print(f"OpenOCD: {args.host}:{args.port}")
        print(f"Target:  {client.target_state()}")
        for label, selected in (("1 channel", channels[:1]), (f"{len(channels)} channels", channels)):
            started = time.perf_counter()
            deadline = started + args.benchmark
            count = 0
            while time.perf_counter() < deadline:
                client.read_channels(selected)
                count += 1
            elapsed = time.perf_counter() - started
            print(f"{label}: {count / elapsed:.1f} samples/s ({count} samples in {elapsed:.3f}s)")
        return 0
    except (OSError, WaveError) as exc:
        print(f"Benchmark failed: {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()


class OzoneGraphSync:
    """Import existing Data Sampling rows and listen for Graph actions."""

    ADD_PATTERN = re.compile(
        r'Window\.Add\s*\(\s*"Data Sampling"\s*,\s*"((?:\\.|[^"\\])*)"',
        re.IGNORECASE,
    )

    def __init__(self, host: str, port: int, events: queue.Queue[tuple]):
        self.host = host
        self.port = port
        self.events = events
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.export_path = Path(tempfile.gettempdir()) / f"openocd_wave_ozone_{os.getpid()}.csv"

    def start(self) -> None:
        self.stop_event.clear()
        if self.thread and self.thread.is_alive():
            return
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()

    @staticmethod
    def _unescape_expression(expression: str) -> str:
        return expression.replace(r'\"', '"').replace(r"\\", "\\")

    def _import_export(self) -> None:
        if not self.export_path.is_file() or self.export_path.stat().st_size == 0:
            return
        try:
            with self.export_path.open("r", newline="", encoding="utf-8-sig", errors="replace") as source:
                reader = csv.DictReader(source)
                fieldnames = reader.fieldnames or []
                if "Expression" in fieldnames:
                    expressions = [row.get("Expression", "").strip() for row in reader]
                else:
                    expressions = [
                        name.strip()
                        for name in fieldnames
                        if name.strip().lower() not in {"index", "time", "time_s"}
                    ]
            for expression in expressions:
                if expression:
                    self.events.put(("ozone_add", expression))
        except (OSError, csv.Error) as exc:
            self.events.put(("ozone_status", f"Ozone import failed: {exc}"))

    def _run(self) -> None:
        while not self.stop_event.is_set():
            sock: Optional[socket.socket] = None
            try:
                sock = socket.create_connection((self.host, self.port), 1.0)
                sock.settimeout(0.25)
                self.events.put(("ozone_status", "Ozone Graph sync connected."))
                try:
                    sock.recv(8192)
                except socket.timeout:
                    pass

                try:
                    self.export_path.unlink(missing_ok=True)
                    export_name = self.export_path.as_posix()
                    command = f'Window.Export("Data Sampling", "{export_name}");\n'
                    sock.sendall(command.encode("utf-8"))
                    deadline = time.perf_counter() + 1.0
                    while time.perf_counter() < deadline and not self.export_path.is_file():
                        self.stop_event.wait(0.05)
                    self._import_export()
                except OSError as exc:
                    self.events.put(("ozone_status", f"Ozone initial import failed: {exc}"))

                buffer = ""
                while not self.stop_event.is_set():
                    try:
                        data = sock.recv(32768)
                        if not data:
                            raise ConnectionError("Ozone closed the automation connection")
                        buffer += data.decode("utf-8", errors="replace")
                    except socket.timeout:
                        continue
                    lines = re.split(r"[\r\n]+", buffer)
                    buffer = lines.pop() if lines else ""
                    for line in lines:
                        for match in self.ADD_PATTERN.finditer(line):
                            expression = self._unescape_expression(match.group(1)).strip()
                            if expression:
                                self.events.put(("ozone_add", expression))
            except (OSError, ConnectionError) as exc:
                self.events.put(("ozone_status", f"Ozone Graph sync waiting: {exc}"))
                self.stop_event.wait(2.0)
            finally:
                if sock is not None:
                    sock.close()
        try:
            self.export_path.unlink(missing_ok=True)
        except OSError:
            pass


class WaveApp:
    def __init__(self, args: argparse.Namespace, gdb: str, channels: list[Channel]):
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk

        self.tk = tk
        self.ttk = ttk
        self.filedialog = filedialog
        self.messagebox = messagebox
        self.args = args
        self.gdb = gdb
        self.channels = channels
        self.channels_lock = threading.Lock()
        self.samples: deque[tuple[float, list[float]]] = deque(maxlen=20000)
        self.events: queue.Queue[tuple] = queue.Queue()
        self.stop_event = threading.Event()
        self.worker: Optional[threading.Thread] = None
        self.start_time = 0.0
        self.latest_values: list[float] = [math.nan] * len(channels)
        self.x_zoom = 1.0
        self.y_zoom = 1.0
        self.y_pan = 0.0
        self.x_view_end: Optional[float] = None
        self._drag_last: Optional[tuple[int, int]] = None
        self._plot_bounds = (112.0, 113.0, 0.0, 1.0)
        self._current_x_span = 1.0

        self.root = tk.Tk()
        self.root.title("OpenOCD Live Waveform")
        self.root.geometry("1180x760")
        self.root.minsize(820, 560)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.rate_var = tk.StringVar(value=str(args.rate))
        self.window_var = tk.StringVar(value=str(args.window))
        self.expression_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Ready. Start OpenOCD, then click Start.")
        self.state_var = tk.StringVar(value="Target: unknown")
        self.actual_rate_var = tk.StringVar(value="Actual: - Hz")
        self.view_var = tk.StringVar(value="overlay")
        self.normalize_var = tk.BooleanVar(value=True)
        self.ozone_sync_var = tk.BooleanVar(value=True)
        self.follow_var = tk.BooleanVar(value=True)
        self.view_status_var = tk.StringVar(value="View: X 1.00x  Y 1.00x")
        self.channel_visible = [tk.BooleanVar(value=True) for _ in channels]
        self.ozone_sync = OzoneGraphSync("127.0.0.1", 19200, self.events)

        self._build_ui()
        self._refresh_channel_table()
        self._rebuild_channel_menu()
        self.root.after(50, self._poll_events)
        self.root.after(100, self._draw)
        self.ozone_sync.start()
        if args.autostart:
            self.root.after(300, self.start)

    def _build_ui(self) -> None:
        tk, ttk = self.tk, self.ttk
        style = ttk.Style(self.root)
        if "vista" in style.theme_names():
            style.theme_use("vista")

        toolbar = ttk.Frame(self.root, padding=(10, 8))
        toolbar.pack(fill="x")
        self.start_button = ttk.Button(toolbar, text="Start", command=self.start)
        self.start_button.pack(side="left")
        self.stop_button = ttk.Button(toolbar, text="Stop", command=self.stop, state="disabled")
        self.stop_button.pack(side="left", padx=(6, 0))
        ttk.Button(toolbar, text="Clear", command=self.clear).pack(side="left", padx=(6, 0))
        ttk.Button(toolbar, text="Export CSV", command=self.export_csv).pack(side="left", padx=(6, 14))

        ttk.Label(toolbar, text="Rate").pack(side="left")
        rate = ttk.Combobox(
            toolbar,
            textvariable=self.rate_var,
            values=("10", "20", "50", "100", "200", "500", "1000", "2000", "5000", "Max"),
            width=7,
        )
        rate.pack(side="left", padx=(5, 3))
        ttk.Label(toolbar, text="Hz").pack(side="left")
        ttk.Label(toolbar, text="Window").pack(side="left", padx=(14, 0))
        ttk.Spinbox(toolbar, from_=1, to=60, textvariable=self.window_var, width=5).pack(
            side="left", padx=(5, 3)
        )
        ttk.Label(toolbar, text="s").pack(side="left")
        ttk.Label(toolbar, textvariable=self.actual_rate_var).pack(side="left", padx=(14, 0))
        ttk.Label(toolbar, textvariable=self.state_var).pack(side="right")
        self.channel_menu_button = ttk.Menubutton(toolbar, text="Channels")
        self.channel_menu = tk.Menu(self.channel_menu_button, tearoff=False)
        self.channel_menu_button.configure(menu=self.channel_menu)
        self.channel_menu_button.pack(side="right", padx=(6, 12))
        ttk.Checkbutton(
            toolbar,
            text="Sync Ozone Graph",
            variable=self.ozone_sync_var,
            command=self.toggle_ozone_sync,
        ).pack(side="right")

        channel_bar = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        channel_bar.pack(fill="x")
        ttk.Label(channel_bar, text="Expression").pack(side="left")
        entry = ttk.Entry(channel_bar, textvariable=self.expression_var)
        entry.pack(side="left", fill="x", expand=True, padx=(8, 6))
        entry.bind("<Return>", lambda _event: self.add_channel())
        ttk.Button(channel_bar, text="Add", command=self.add_channel).pack(side="left")
        ttk.Button(channel_bar, text="Remove", command=self.remove_channel).pack(side="left", padx=(6, 0))
        ttk.Checkbutton(channel_bar, text="Normalize", variable=self.normalize_var).pack(side="right", padx=(8, 0))
        ttk.Radiobutton(channel_bar, text="Stacked", variable=self.view_var, value="stacked").pack(side="right")
        ttk.Radiobutton(channel_bar, text="Overlay", variable=self.view_var, value="overlay").pack(side="right")

        table_frame = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        table_frame.pack(fill="x")
        self.channel_table = ttk.Treeview(
            table_frame,
            columns=("expression", "type", "address", "value"),
            show="headings",
            height=5,
            selectmode="browse",
        )
        self.channel_table.heading("expression", text="Expression")
        self.channel_table.heading("type", text="Type")
        self.channel_table.heading("address", text="Address")
        self.channel_table.heading("value", text="Latest")
        self.channel_table.column("expression", width=480, stretch=True)
        self.channel_table.column("type", width=130, stretch=False)
        self.channel_table.column("address", width=105, anchor="e", stretch=False)
        self.channel_table.column("value", width=130, anchor="e", stretch=False)
        channel_scroll = ttk.Scrollbar(table_frame, orient="vertical", command=self.channel_table.yview)
        self.channel_table.configure(yscrollcommand=channel_scroll.set)
        channel_scroll.pack(side="right", fill="y")
        self.channel_table.pack(side="left", fill="x", expand=True)

        nav = ttk.Frame(self.root, padding=(10, 0, 10, 6))
        nav.pack(fill="x")
        ttk.Label(nav, text="Horizontal").pack(side="left")
        ttk.Button(nav, text="X-", width=4, command=lambda: self._zoom_x(1 / 1.5)).pack(
            side="left", padx=(5, 2)
        )
        ttk.Button(nav, text="X+", width=4, command=lambda: self._zoom_x(1.5)).pack(side="left")
        ttk.Button(nav, text="Left", width=6, command=lambda: self._pan_x(-1)).pack(
            side="left", padx=(8, 2)
        )
        ttk.Button(nav, text="Right", width=6, command=lambda: self._pan_x(1)).pack(side="left")
        ttk.Label(nav, text="Vertical").pack(side="left", padx=(16, 0))
        ttk.Button(nav, text="Y-", width=4, command=lambda: self._zoom_y(1 / 1.5)).pack(
            side="left", padx=(5, 2)
        )
        ttk.Button(nav, text="Y+", width=4, command=lambda: self._zoom_y(1.5)).pack(side="left")
        ttk.Button(nav, text="Up", width=5, command=lambda: self._pan_y(1)).pack(
            side="left", padx=(8, 2)
        )
        ttk.Button(nav, text="Down", width=6, command=lambda: self._pan_y(-1)).pack(side="left")
        ttk.Button(nav, text="Reset View", command=self.reset_view).pack(side="left", padx=(16, 8))
        ttk.Checkbutton(
            nav,
            text="Follow",
            variable=self.follow_var,
            command=self._toggle_follow,
        ).pack(side="left")
        ttk.Label(nav, textvariable=self.view_status_var).pack(side="right")

        self.canvas = tk.Canvas(self.root, background="#ffffff", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=10)
        self.canvas.configure(cursor="fleur")
        self.canvas.bind("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind("<ButtonPress-1>", self._on_drag_start)
        self.canvas.bind("<B1-Motion>", self._on_drag_move)
        self.canvas.bind("<ButtonRelease-1>", self._on_drag_end)
        self.canvas.bind("<Double-Button-1>", lambda _event: self.reset_view())

        status = ttk.Label(self.root, textvariable=self.status_var, anchor="w", padding=(10, 6))
        status.pack(fill="x")

    def _refresh_channel_table(self) -> None:
        selected = self.channel_table.selection()
        self.channel_table.delete(*self.channel_table.get_children())
        with self.channels_lock:
            channels = list(self.channels)
        self.channel_table.configure(height=min(7, max(4, len(channels))))
        for index, channel in enumerate(channels):
            latest = self.latest_values[index] if index < len(self.latest_values) else math.nan
            value = "-" if math.isnan(latest) else f"{latest:.9g}"
            self.channel_table.insert(
                "",
                "end",
                iid=str(index),
                values=(channel.expression, channel.type_name, f"0x{channel.address:08X}", value),
            )
        if selected and self.channel_table.exists(selected[0]):
            self.channel_table.selection_set(selected[0])

    def _rebuild_channel_menu(self) -> None:
        self.channel_menu.delete(0, "end")
        self.channel_menu.add_command(label="Show all", command=lambda: self._set_all_visible(True))
        self.channel_menu.add_command(label="Hide all", command=lambda: self._set_all_visible(False))
        self.channel_menu.add_separator()
        with self.channels_lock:
            channels = list(self.channels)
        for index, channel in enumerate(channels):
            label = channel.expression if len(channel.expression) <= 54 else "..." + channel.expression[-51:]
            self.channel_menu.add_checkbutton(
                label=label,
                variable=self.channel_visible[index],
            )

    def _set_all_visible(self, visible: bool) -> None:
        for variable in self.channel_visible:
            variable.set(visible)

    def toggle_ozone_sync(self) -> None:
        if self.ozone_sync_var.get():
            self.ozone_sync.start()
        else:
            self.ozone_sync.stop()
            self.status_var.set("Ozone Graph sync disabled.")

    def _add_expression(self, expression: str, automatic: bool = False) -> bool:
        with self.channels_lock:
            if len(self.channels) >= 16:
                if not automatic:
                    self.messagebox.showwarning("Channel limit", "A maximum of 16 channels is supported.")
                return False
        try:
            channel = resolve_expression(self.gdb, self.args.elf, expression)
        except WaveError as exc:
            if automatic:
                self.status_var.set(f"Ozone Graph import skipped: {exc}")
            else:
                self.messagebox.showerror("Expression error", str(exc))
            return False

        with self.channels_lock:
            if any(item.address == channel.address and item.size == channel.size for item in self.channels):
                if automatic:
                    self.status_var.set(f"Ozone Graph already present: {expression}")
                return False
            self.channels.append(channel)
            self.latest_values.append(math.nan)
            self.channel_visible.append(self.tk.BooleanVar(value=True))
        self.clear()
        self._refresh_channel_table()
        self._rebuild_channel_menu()
        if automatic:
            self.status_var.set(f"Imported Ozone Graph: {expression}")
        return True

    def add_channel(self) -> None:
        expression = self.expression_var.get().strip()
        if not expression:
            return
        if self._add_expression(expression):
            self.expression_var.set("")

    def remove_channel(self) -> None:
        if self.worker and self.worker.is_alive():
            self.messagebox.showinfo("Stop sampling", "Stop sampling before removing channels.")
            return
        selected = self.channel_table.selection()
        if not selected:
            return
        index = int(selected[0])
        with self.channels_lock:
            self.channels.pop(index)
            self.latest_values.pop(index)
            self.channel_visible.pop(index)
        self.clear()
        self._refresh_channel_table()
        self._rebuild_channel_menu()

    def start(self) -> None:
        with self.channels_lock:
            has_channels = bool(self.channels)
        if not has_channels:
            self.messagebox.showwarning("No channels", "Add at least one scalar expression.")
            return
        if self.worker and self.worker.is_alive():
            return
        try:
            rate_text = self.rate_var.get().strip()
            rate = None if rate_text.lower() == "max" else float(rate_text)
            window = float(self.window_var.get())
            if (rate is not None and rate <= 0) or window <= 0:
                raise ValueError
        except ValueError:
            self.messagebox.showerror("Invalid settings", "Rate must be a positive number or Max.")
            return

        buffer_rate = rate if rate is not None else 5000.0
        required = min(500000, max(1000, int(buffer_rate * window * 8.0)))
        if self.samples.maxlen != required:
            self.samples = deque(self.samples, maxlen=required)
        self.stop_event.clear()
        self.start_time = time.perf_counter()
        self.worker = threading.Thread(target=self._sample_worker, args=(rate,), daemon=True)
        self.worker.start()
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.status_var.set(f"Connecting to OpenOCD at {self.args.host}:{self.args.port}...")

    def stop(self) -> None:
        self.stop_event.set()
        self.start_button.configure(state="normal")
        self.stop_button.configure(state="disabled")
        self.actual_rate_var.set("Actual: - Hz")
        self.status_var.set("Sampling stopped.")

    def clear(self) -> None:
        self.samples.clear()
        self.latest_values = [math.nan] * len(self.channels)
        self._refresh_channel_table()
        self.reset_view()

    def _sample_worker(self, rate: Optional[float]) -> None:
        client = OpenOcdTcl(self.args.host, self.args.port)
        interval = 0.0 if rate is None else 1.0 / rate
        next_sample = time.perf_counter()
        last_state_check = 0.0
        rate_started = next_sample
        rate_count = 0
        try:
            client.connect()
            self.events.put(("connected",))
            while not self.stop_event.is_set():
                now = time.perf_counter()
                if now - last_state_check >= 1.0:
                    self.events.put(("state", client.target_state()))
                    last_state_check = now
                with self.channels_lock:
                    channels = list(self.channels)
                values = client.read_channels(channels)
                self.events.put(("sample", now - self.start_time, values))
                rate_count += 1
                measured_at = time.perf_counter()
                if measured_at - rate_started >= 1.0:
                    self.events.put(("rate", rate_count / (measured_at - rate_started)))
                    rate_started = measured_at
                    rate_count = 0
                if interval > 0:
                    next_sample += interval
                    delay = next_sample - time.perf_counter()
                    if delay > 0:
                        self.stop_event.wait(delay)
                    else:
                        next_sample = time.perf_counter()
        except (OSError, WaveError) as exc:
            self.events.put(("error", str(exc)))
        finally:
            client.close()
            self.events.put(("stopped",))

    def _poll_events(self) -> None:
        table_dirty = False
        try:
            while True:
                event = self.events.get_nowait()
                if event[0] == "connected":
                    self.status_var.set("Sampling SRAM through OpenOCD TCL. Ozone may remain connected.")
                elif event[0] == "state":
                    state = event[1]
                    self.state_var.set(f"Target: {state}")
                    if state == "halted":
                        self.status_var.set("Target is halted. Resume it in Ozone to collect changing samples.")
                elif event[0] == "sample":
                    self.samples.append((event[1], event[2]))
                    self.latest_values = event[2]
                    table_dirty = True
                elif event[0] == "rate":
                    self.actual_rate_var.set(f"Actual: {event[1]:.0f} Hz")
                elif event[0] == "ozone_add":
                    self._add_expression(event[1], automatic=True)
                elif event[0] == "ozone_status":
                    self.status_var.set(event[1])
                elif event[0] == "error":
                    self.status_var.set(f"Sampling error: {event[1]}")
                    self.stop_event.set()
                elif event[0] == "stopped":
                    self.start_button.configure(state="normal")
                    self.stop_button.configure(state="disabled")
                    self.actual_rate_var.set("Actual: - Hz")
        except queue.Empty:
            pass
        if table_dirty:
            self._refresh_channel_table()
        self.root.after(50, self._poll_events)

    def reset_view(self) -> None:
        self.x_zoom = 1.0
        self.y_zoom = 1.0
        self.y_pan = 0.0
        self.x_view_end = None
        self.follow_var.set(True)
        self._update_view_status()

    def _latest_time(self) -> float:
        return self.samples[-1][0] if self.samples else 0.0

    def _update_view_status(self) -> None:
        self.view_status_var.set(f"View: X {self.x_zoom:.2f}x  Y {self.y_zoom:.2f}x")

    def _toggle_follow(self) -> None:
        if self.follow_var.get():
            self.x_view_end = None
        elif self.x_view_end is None:
            self.x_view_end = self._latest_time()

    def _freeze_x_view(self) -> None:
        if self.follow_var.get() or self.x_view_end is None:
            self.x_view_end = self._latest_time()
        self.follow_var.set(False)

    def _clamp_x_end(self, end_time: float, span: float) -> float:
        if not self.samples:
            return end_time
        earliest = self.samples[0][0]
        latest = self.samples[-1][0]
        if latest - earliest <= span:
            return latest
        return min(latest, max(earliest + span, end_time))

    def _zoom_x(self, factor: float, anchor_fraction: float = 0.5, mouse: bool = False) -> None:
        old_zoom = self.x_zoom
        new_zoom = min(256.0, max(0.125, old_zoom * factor))
        if math.isclose(new_zoom, old_zoom):
            return
        try:
            base_window = max(0.001, float(self.window_var.get()))
        except ValueError:
            base_window = 5.0
        latest = self._latest_time()
        old_span = base_window / old_zoom
        new_span = base_window / new_zoom
        old_end = latest if self.follow_var.get() else (self.x_view_end or latest)
        self.x_zoom = new_zoom
        if mouse:
            anchor = old_end - old_span + anchor_fraction * old_span
            new_end = anchor + (1.0 - anchor_fraction) * new_span
            self._freeze_x_view()
            self.x_view_end = self._clamp_x_end(new_end, new_span)
        elif not self.follow_var.get():
            center = old_end - old_span / 2.0
            self.x_view_end = self._clamp_x_end(center + new_span / 2.0, new_span)
        self._update_view_status()

    def _zoom_y(self, factor: float, anchor_fraction: float = 0.5) -> None:
        old_zoom = self.y_zoom
        new_zoom = min(256.0, max(0.125, old_zoom * factor))
        if math.isclose(new_zoom, old_zoom):
            return
        old_span = 1.0 / old_zoom
        new_span = 1.0 / new_zoom
        old_center = 0.5 + self.y_pan
        anchor_value = old_center + (0.5 - anchor_fraction) * old_span
        new_center = anchor_value - (0.5 - anchor_fraction) * new_span
        self.y_zoom = new_zoom
        self.y_pan = max(-20.0, min(20.0, new_center - 0.5))
        self._update_view_status()

    def _pan_x(self, direction: int) -> None:
        self._freeze_x_view()
        current = self.x_view_end if self.x_view_end is not None else self._latest_time()
        self.x_view_end = self._clamp_x_end(
            current + direction * self._current_x_span * 0.2,
            self._current_x_span,
        )

    def _pan_y(self, direction: int) -> None:
        self.y_pan = max(-20.0, min(20.0, self.y_pan + direction * 0.15 / self.y_zoom))

    def _on_mousewheel(self, event) -> str:
        left, right, top, bottom = self._plot_bounds
        if event.state & 0x0004:
            fraction = min(1.0, max(0.0, (event.y - top) / max(1.0, bottom - top)))
            self._zoom_y(1.25 if event.delta > 0 else 0.8, fraction)
        else:
            fraction = min(1.0, max(0.0, (event.x - left) / max(1.0, right - left)))
            self._zoom_x(1.25 if event.delta > 0 else 0.8, fraction, mouse=True)
        return "break"

    def _on_drag_start(self, event) -> None:
        self._drag_last = (event.x, event.y)
        self._freeze_x_view()

    def _on_drag_move(self, event) -> None:
        if self._drag_last is None:
            return
        old_x, old_y = self._drag_last
        dx, dy = event.x - old_x, event.y - old_y
        left, right, top, bottom = self._plot_bounds
        current = self.x_view_end if self.x_view_end is not None else self._latest_time()
        self.x_view_end = self._clamp_x_end(
            current - dx / max(1.0, right - left) * self._current_x_span,
            self._current_x_span,
        )
        self.y_pan = max(-20.0, min(20.0, self.y_pan + dy / max(1.0, bottom - top) / self.y_zoom))
        self._drag_last = (event.x, event.y)

    def _on_drag_end(self, _event) -> None:
        self._drag_last = None

    @staticmethod
    def _format_number(value: float) -> str:
        absolute = abs(value)
        if absolute >= 100000 or (0 < absolute < 0.001):
            return f"{value:.3e}"
        return f"{value:.5g}"

    @staticmethod
    def _format_time(delta: float, span: float) -> str:
        if abs(delta) < span / 10000:
            return "0 s"
        if span >= 20:
            return f"{delta:.1f} s"
        if span >= 2:
            return f"{delta:.2f} s"
        return f"{delta:.3f} s"

    def _view_y_range(self, low: float, high: float) -> tuple[float, float]:
        base_span = max(1e-15, high - low)
        center = (low + high) / 2.0 + self.y_pan * base_span
        span = base_span / self.y_zoom
        return center - span / 2.0, center + span / 2.0

    def _draw_y_grid(self, canvas, left, right, y0, y1, low, high, count: int) -> None:
        for tick in range(count):
            fraction = tick / max(1, count - 1)
            y = y1 - fraction * (y1 - y0)
            value = low + fraction * (high - low)
            canvas.create_line(left, y, right, y, fill="#e5e9ed")
            canvas.create_text(
                left - 7,
                y,
                anchor="e",
                text=self._format_number(value),
                fill="#5f6873",
                font=("Consolas", 8),
            )

    def _draw_x_grid(self, canvas, left, right, y0, y1, start, end, latest, labels: bool) -> None:
        span = max(1e-12, end - start)
        for tick in range(7):
            fraction = tick / 6.0
            x = left + fraction * (right - left)
            sample_time = start + fraction * span
            canvas.create_line(x, y0, x, y1, fill="#edf0f2")
            if labels:
                canvas.create_text(
                    x,
                    y1 + 7,
                    anchor="n",
                    text=self._format_time(sample_time - latest, span),
                    fill="#5f6873",
                    font=("Consolas", 8),
                )

    def _draw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width = max(1, canvas.winfo_width())
        height = max(1, canvas.winfo_height())
        with self.channels_lock:
            channels = list(self.channels)
        if not channels:
            canvas.create_text(width / 2, height / 2, text="Add a scalar expression to begin.", fill="#5f6368")
            self.root.after(100, self._draw)
            return

        visible_indices = [
            index
            for index in range(len(channels))
            if index < len(self.channel_visible) and self.channel_visible[index].get()
        ]
        if not visible_indices:
            canvas.create_text(
                width / 2,
                height / 2,
                text="No channels selected. Use the Channels menu to show signals.",
                fill="#5f6368",
            )
            self.root.after(100, self._draw)
            return

        try:
            base_window = max(0.001, float(self.window_var.get()))
        except ValueError:
            base_window = 5.0
        snapshot = list(self.samples)
        latest_time = snapshot[-1][0] if snapshot else 0.0
        window = base_window / self.x_zoom
        end_time = (
            latest_time
            if self.follow_var.get()
            else (self.x_view_end if self.x_view_end is not None else latest_time)
        )
        start_time = end_time - window
        visible = [sample for sample in snapshot if start_time <= sample[0] <= end_time]
        self._current_x_span = window

        left, right_margin, top, bottom = 112, 18, 8, 42
        plot_right = max(left + 1, width - right_margin)
        point_limit = max(1, int((plot_right - left) * 2))
        stride = max(1, len(visible) // point_limit)
        plot_samples = visible[::stride]
        if visible and plot_samples[-1] is not visible[-1]:
            plot_samples.append(visible[-1])

        def value_range(index: int) -> tuple[float, float]:
            values = [
                sample_values[index]
                for _, sample_values in plot_samples
                if index < len(sample_values) and math.isfinite(sample_values[index])
            ]
            if not values:
                return -1.0, 1.0
            low, high = min(values), max(values)
            if math.isclose(low, high, rel_tol=1e-9, abs_tol=1e-12):
                pad = max(abs(low) * 0.05, 1e-3)
            else:
                pad = (high - low) * 0.08
            return low - pad, high + pad

        ranges = {index: value_range(index) for index in visible_indices}

        def signal_points(index, y0, y1, low, high, normalizer=None) -> list[float]:
            points: list[float] = []
            for sample_time, sample_values in plot_samples:
                if index >= len(sample_values) or not math.isfinite(sample_values[index]):
                    continue
                value = sample_values[index]
                if normalizer is not None:
                    value = normalizer(value)
                x = left + (sample_time - start_time) / window * (plot_right - left)
                y = y1 - (value - low) / max(1e-15, high - low) * (y1 - y0)
                points.extend((x, min(y1, max(y0, y))))
            return points

        def draw_signal(points: list[float], color: str) -> None:
            if len(points) >= 4:
                canvas.create_line(*points, fill=color, width=2, smooth=False)
            elif len(points) == 2:
                canvas.create_oval(
                    points[0] - 2,
                    points[1] - 2,
                    points[0] + 2,
                    points[1] + 2,
                    fill=color,
                    outline="",
                )

        if self.view_var.get() == "overlay":
            legend_columns = 2 if len(visible_indices) > 4 else 1
            legend_rows = math.ceil(len(visible_indices) / legend_columns)
            y0 = top + 8 + legend_rows * 18
            y1 = max(y0 + 50, height - bottom)
            self._plot_bounds = (left, plot_right, y0, y1)
            if self.normalize_var.get():
                shown_low, shown_high = self._view_y_range(0.0, 1.0)
            else:
                common_low = min(ranges[index][0] for index in visible_indices)
                common_high = max(ranges[index][1] for index in visible_indices)
                shown_low, shown_high = self._view_y_range(common_low, common_high)

            canvas.create_rectangle(left, y0, plot_right, y1, fill="#ffffff", outline="#cbd2d9")
            self._draw_y_grid(canvas, left, plot_right, y0, y1, shown_low, shown_high, 5)
            self._draw_x_grid(
                canvas, left, plot_right, y0, y1, start_time, end_time, latest_time, True
            )

            column_width = max(180, (plot_right - left) / legend_columns)
            for position, index in enumerate(visible_indices):
                channel = channels[index]
                color = COLORS[index % len(COLORS)]
                latest = self.latest_values[index] if index < len(self.latest_values) else math.nan
                value_text = "-" if math.isnan(latest) else f"{latest:.6g}"
                column = position // legend_rows
                row = position % legend_rows
                short_name = re.split(r"[.)]", channel.expression)[-1] or channel.expression[-36:]
                canvas.create_text(
                    left + column * column_width + 8,
                    top + row * 18 + 2,
                    anchor="nw",
                    text=f"{short_name}  {value_text}",
                    fill=color,
                    font=("Segoe UI", 9, "bold"),
                )
                if self.normalize_var.get():
                    raw_low, raw_high = ranges[index]
                    raw_span = max(1e-15, raw_high - raw_low)
                    normalizer = lambda value, lo=raw_low, size=raw_span: (value - lo) / size
                else:
                    normalizer = None
                draw_signal(
                    signal_points(index, y0, y1, shown_low, shown_high, normalizer),
                    color,
                )
        else:
            lane_height = max(64, (height - top - bottom) / len(visible_indices))
            plot_bottom = min(height - bottom, top + lane_height * len(visible_indices) - 6)
            self._plot_bounds = (left, plot_right, top, plot_bottom)
            for position, index in enumerate(visible_indices):
                channel = channels[index]
                y0 = top + position * lane_height
                y1 = min(height - bottom, y0 + lane_height - 6)
                shown_low, shown_high = self._view_y_range(*ranges[index])
                color = COLORS[index % len(COLORS)]
                latest = self.latest_values[index] if index < len(self.latest_values) else math.nan
                value_text = "-" if math.isnan(latest) else f"{latest:.6g}"
                canvas.create_rectangle(left, y0, plot_right, y1, fill="#ffffff", outline="#cbd2d9")
                self._draw_y_grid(canvas, left, plot_right, y0, y1, shown_low, shown_high, 3)
                self._draw_x_grid(
                    canvas,
                    left,
                    plot_right,
                    y0,
                    y1,
                    start_time,
                    end_time,
                    latest_time,
                    position == len(visible_indices) - 1,
                )
                canvas.create_text(
                    left + 8,
                    y0 + 5,
                    anchor="nw",
                    text=channel.expression,
                    fill=color,
                    font=("Segoe UI", 9, "bold"),
                )
                canvas.create_text(
                    plot_right - 8,
                    y0 + 5,
                    anchor="ne",
                    text=value_text,
                    fill="#202124",
                    font=("Consolas", 9, "bold"),
                )
                draw_signal(signal_points(index, y0, y1, shown_low, shown_high), color)

        canvas.create_text(
            (left + plot_right) / 2,
            height - 4,
            anchor="s",
            text="Time relative to latest sample",
            fill="#5f6873",
            font=("Segoe UI", 8),
        )
        self.root.after(100, self._draw)

    def export_csv(self) -> None:
        snapshot = list(self.samples)
        if not snapshot:
            self.messagebox.showinfo("No samples", "There are no samples to export.")
            return
        default_name = time.strftime("openocd_wave_%Y%m%d_%H%M%S.csv")
        path = self.filedialog.asksaveasfilename(
            title="Export samples",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8-sig") as output:
            writer = csv.writer(output)
            writer.writerow(["time_s", *(channel.expression for channel in self.channels)])
            for sample_time, values in snapshot:
                writer.writerow([f"{sample_time:.9f}", *(f"{value:.9g}" for value in values)])
        self.status_var.set(f"Exported {len(snapshot)} samples to {path}")

    def close(self) -> None:
        self.stop_event.set()
        self.ozone_sync.stop()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    default_elf = script_dir.parent / "build" / "Debug" / "threenibian.elf"
    parser = argparse.ArgumentParser(description="Plot ELF expressions through OpenOCD TCL.")
    parser.add_argument("--elf", default=str(default_elf), help="Firmware ELF with debug information")
    parser.add_argument("--gdb", help="Path to arm-none-eabi-gdb.exe")
    parser.add_argument("--host", default="127.0.0.1", help="OpenOCD TCL host")
    parser.add_argument("--port", type=int, default=6666, help="OpenOCD TCL port")
    parser.add_argument("--rate", type=float, default=50.0, help="Initial sampling rate in Hz")
    parser.add_argument("--window", type=float, default=5.0, help="Initial plot window in seconds")
    parser.add_argument("--channel", action="append", help="Scalar C expression; repeat for multiple channels")
    parser.add_argument("--probe", action="store_true", help="Resolve and read each channel once, without GUI")
    parser.add_argument(
        "--benchmark",
        type=float,
        metavar="SECONDS",
        help="Measure maximum read rate for one and all configured channels",
    )
    parser.add_argument("--autostart", action="store_true", help="Start sampling after the GUI opens")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not Path(args.elf).is_file():
        print(f"ELF not found: {args.elf}", file=sys.stderr)
        return 2
    try:
        gdb = find_gdb(args.gdb)
        expressions = args.channel or DEFAULT_CHANNELS
        channels = [resolve_expression(gdb, args.elf, expression) for expression in expressions]
    except WaveError as exc:
        print(f"Setup failed: {exc}", file=sys.stderr)
        return 2

    if args.probe:
        return probe(args, channels)
    if args.benchmark:
        return benchmark(args, channels)

    app = WaveApp(args, gdb, channels)
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

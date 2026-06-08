#!/usr/bin/env python3
from __future__ import annotations

from collections import deque
from datetime import datetime
from pathlib import Path
from queue import Empty, Queue
from statistics import median
from threading import Thread
import time
import tkinter as tk
from tkinter import filedialog

import serial
from serial.tools import list_ports


# Optional manual override. Leave as None for auto-detection.
PREFERRED_SERIAL_PORT: str | None = None
LOG_DIR = Path("/home/vovanol/Downloads/ardu/logs")

PREFERRED_BAUD_RATE = 115200
BAUD_CANDIDATES = (115200, 230400, 1000000)
SERIAL_TIMEOUT_S = 1.0
RECONNECT_DELAY_S = 2.0
WINDOW_TITLE = "Arduino Due Telemetry"
AUTO_PORT_HINTS = ("arduino", "due", "ttyacm", "ttyusb", "usbmodem", "wchusbserial")
ARDUINO_VIDS = {0x2341, 0x2A03}

HISTORY_POINTS = 360
MEDIAN_WINDOW_S = 5.0
CANVAS_WIDTH = 780
CANVAS_HEIGHT = 180
UPDATE_PERIOD_MS = 20
SESSION_START_THRESHOLD_V = 5.0
LOG_RECORD_PERIOD_S = 0.2
PLOT_SMOOTHING_WINDOW = 5
PLOT_SPLINE_STEPS = 18
PLOT_OUTLIER_CLIP = 0.03
PLOT_Y_PADDING_RATIO = 0.08
PLOT_MIN_SPAN = 0.15

def parse_named_values(line: str) -> dict[str, str]:
    values: dict[str, str] = {}

    for chunk in line.split(","):
        part = chunk.strip()
        if ":" not in part:
            continue

        key, value = part.split(":", 1)
        values[key.strip()] = value.strip()

    return values


def parse_float(value: str | None) -> float | None:
    if value is None:
        return None

    try:
        return float(value)
    except ValueError:
        return None


def append_line(path: Path, line: str) -> None:
    with path.open("a", encoding="utf-8") as log_file:
        log_file.write(line + "\n")


def baud_candidates() -> tuple[int, ...]:
    ordered = [PREFERRED_BAUD_RATE]
    for baud_rate in BAUD_CANDIDATES:
        if baud_rate not in ordered:
            ordered.append(baud_rate)
    return tuple(ordered)


def open_serial_port(port: str, baud_rate: int) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud_rate
    ser.timeout = SERIAL_TIMEOUT_S
    ser.write_timeout = SERIAL_TIMEOUT_S
    ser.dtr = False
    ser.rts = False
    ser.open()
    time.sleep(1.5)
    ser.reset_input_buffer()
    return ser


def read_serial_sample(ser: serial.Serial, timeout_s: float) -> dict[str, object] | None:
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        if not line or line.startswith("#"):
            continue

        parsed = parse_named_values(line)
        if not parsed:
            continue

        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        return {
            "sample_time": time.monotonic(),
            "timestamp": timestamp,
            "raw_line": line,
            "weight_g": parse_float(parsed.get("weight_g")),
            "current_A": parse_float(parsed.get("current_A")),
            "voltage_V": parse_float(parsed.get("voltage_V")),
        }

    return None


def percentile(sorted_values: list[float], fraction: float) -> float:
    if not sorted_values:
        return 0.0

    if len(sorted_values) == 1:
        return sorted_values[0]

    position = fraction * (len(sorted_values) - 1)
    lower_index = int(position)
    upper_index = min(len(sorted_values) - 1, lower_index + 1)
    weight = position - lower_index

    lower_value = sorted_values[lower_index]
    upper_value = sorted_values[upper_index]
    return lower_value + (upper_value - lower_value) * weight


def detect_serial_port() -> str | None:
    ports = list(list_ports.comports())

    if PREFERRED_SERIAL_PORT:
        for port in ports:
            if port.device == PREFERRED_SERIAL_PORT:
                return port.device

    best_device: str | None = None
    best_score = -1

    for port in ports:
        text = " ".join(
            filter(
                None,
                (
                    port.device,
                    port.description,
                    port.manufacturer,
                    port.product,
                ),
            )
        ).lower()

        score = 0
        if port.vid in ARDUINO_VIDS:
            score += 100

        for hint in AUTO_PORT_HINTS:
            if hint in text:
                score += 10

        if score > best_score:
            best_score = score
            best_device = port.device

    if best_score <= 0:
        return None

    return best_device


class TelemetryPlotApp:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title(WINDOW_TITLE)
        self.root.geometry("1000x760")

        self.queue: Queue[dict[str, float | str | None]] = Queue()
        self.weight_data: deque[float] = deque(maxlen=HISTORY_POINTS)
        self.current_data: deque[float] = deque(maxlen=HISTORY_POINTS)
        self.voltage_data: deque[float] = deque(maxlen=HISTORY_POINTS)
        self.weight_window: deque[tuple[float, float]] = deque()
        self.current_window: deque[tuple[float, float]] = deque()
        self.voltage_window: deque[tuple[float, float]] = deque()
        self.session_samples: list[dict[str, object]] = []
        self.current_port: str | None = None
        self.session_active = False
        self.session_saved = False
        self.session_started_at: datetime | None = None
        self.last_logged_sample_time: float | None = None
        self.log_dir = LOG_DIR

        self.status_var = tk.StringVar(value="Searching for Arduino serial port...")
        self.session_var = tk.StringVar(
            value=f"Waiting for battery connection: voltage_V > {SESSION_START_THRESHOLD_V:.1f} V"
        )
        self.log_dir_var = tk.StringVar(value=f"Log path: {self.log_dir}")
        self.weight_var = tk.StringVar(value="weight_g: --")
        self.current_var = tk.StringVar(value="current_A: --")
        self.voltage_var = tk.StringVar(value="voltage_V: --")
        self.weight_median_var = tk.StringVar(value="--")
        self.current_median_var = tk.StringVar(value="--")
        self.voltage_median_var = tk.StringVar(value="--")

        self._build_ui()

        self.reader_thread = Thread(target=self._serial_reader_loop, daemon=True)
        self.reader_thread.start()

        self.root.after(UPDATE_PERIOD_MS, self._process_queue)

    def _build_ui(self) -> None:
        self.root.configure(bg="#f3f0ea")

        header = tk.Frame(self.root, bg="#f3f0ea")
        header.pack(fill="x", padx=12, pady=(12, 8))

        top_row = tk.Frame(header, bg="#f3f0ea")
        top_row.pack(fill="x")

        tk.Label(
            top_row,
            text="Arduino Due Telemetry",
            font=("TkDefaultFont", 16, "bold"),
            bg="#f3f0ea",
        ).pack(side="left", anchor="w")

        self.save_button = tk.Button(
            top_row,
            text="Save Log",
            command=self._save_log_button,
            state="disabled",
            padx=12,
            pady=6,
        )
        self.save_button.pack(side="right")

        self.log_dir_button = tk.Button(
            top_row,
            text="Log Folder",
            command=self._select_log_dir,
            padx=12,
            pady=6,
        )
        self.log_dir_button.pack(side="right", padx=(0, 8))

        tk.Label(
            header,
            textvariable=self.status_var,
            font=("TkDefaultFont", 10),
            bg="#f3f0ea",
            fg="#4d5a67",
        ).pack(anchor="w", pady=(4, 0))

        tk.Label(
            header,
            textvariable=self.session_var,
            font=("TkDefaultFont", 10),
            bg="#f3f0ea",
            fg="#7b5d1e",
        ).pack(anchor="w", pady=(2, 0))

        tk.Label(
            header,
            textvariable=self.log_dir_var,
            font=("TkDefaultFont", 10),
            bg="#f3f0ea",
            fg="#5a6672",
        ).pack(anchor="w", pady=(2, 0))

        values = tk.Frame(self.root, bg="#f3f0ea")
        values.pack(fill="x", padx=12, pady=(0, 8))

        for text_var in (self.weight_var, self.current_var, self.voltage_var):
            tk.Label(
                values,
                textvariable=text_var,
                font=("TkDefaultFont", 12, "bold"),
                bg="#f3f0ea",
                padx=12,
            ).pack(side="left")

        self.weight_canvas = self._make_plot_block("Weight, g", "#1b6ef3", self.weight_median_var, "g")
        self.current_canvas = self._make_plot_block("Current, A", "#d94841", self.current_median_var, "A")
        self.voltage_canvas = self._make_plot_block("Voltage, V", "#1e9b62", self.voltage_median_var, "V")

    def _make_plot_block(
        self,
        title: str,
        color: str,
        median_var: tk.StringVar,
        unit: str,
    ) -> tk.Canvas:
        frame = tk.Frame(self.root, bg="#f3f0ea")
        frame.pack(fill="x", padx=12, pady=8)

        tk.Label(
            frame,
            text=title,
            font=("TkDefaultFont", 11, "bold"),
            bg="#f3f0ea",
            fg=color,
        ).pack(anchor="w", pady=(0, 4))

        body = tk.Frame(frame, bg="#f3f0ea")
        body.pack(fill="x")

        canvas = tk.Canvas(
            body,
            width=CANVAS_WIDTH,
            height=CANVAS_HEIGHT,
            bg="#ffffff",
            highlightthickness=1,
            highlightbackground="#d8d2c8",
        )
        canvas.pack(side="left", fill="x", expand=True)

        side = tk.Frame(body, bg="#ece7de", width=170, padx=14, pady=14)
        side.pack(side="right", fill="y", padx=(10, 0))
        side.pack_propagate(False)

        tk.Label(
            side,
            text="Median 5s",
            font=("TkDefaultFont", 11, "bold"),
            bg="#ece7de",
            fg="#4d5a67",
        ).pack(anchor="w")

        tk.Label(
            side,
            textvariable=median_var,
            font=("TkDefaultFont", 20, "bold"),
            bg="#ece7de",
            fg=color,
        ).pack(anchor="w", pady=(10, 4))

        tk.Label(
            side,
            text=f"window: {MEDIAN_WINDOW_S:.0f}s\nunit: {unit}",
            justify="left",
            font=("TkDefaultFont", 10),
            bg="#ece7de",
            fg="#6a6a6a",
        ).pack(anchor="w")

        return canvas

    def _serial_reader_loop(self) -> None:
        self.log_dir.mkdir(parents=True, exist_ok=True)

        while True:
            port = detect_serial_port()
            if port is None:
                self.current_port = None
                self.queue.put({"status": "Arduino serial port not found. Waiting for connection..."})
                time.sleep(RECONNECT_DELAY_S)
                continue

            last_error: str | None = None

            for baud_rate in baud_candidates():
                try:
                    self.current_port = port
                    with open_serial_port(port, baud_rate) as ser:
                        first_sample = read_serial_sample(ser, 4.0)
                        if first_sample is None:
                            last_error = f"No telemetry at {baud_rate} baud"
                            continue

                        status = f"Connected to {port} @ {baud_rate} baud. Log folder: {self.log_dir}"
                        self.queue.put({"status": status})
                        self.queue.put({"status": status, **first_sample})

                        while True:
                            sample = read_serial_sample(ser, 2.0)
                            if sample is None:
                                continue

                            self.queue.put({"status": status, **sample})

                except (serial.SerialException, OSError) as exc:
                    last_error = f"Serial error on {port} @ {baud_rate}: {exc}"
                    continue

            self.current_port = None
            if last_error is None:
                last_error = f"No valid telemetry found on {port}"
            self.queue.put({"status": f"{last_error}. Retrying in {RECONNECT_DELAY_S:.0f}s..."})
            time.sleep(RECONNECT_DELAY_S)

    def _process_queue(self) -> None:
        changed = False

        while True:
            try:
                item = self.queue.get_nowait()
            except Empty:
                break

            status = item.get("status")
            if isinstance(status, str):
                self.status_var.set(status)

            sample_time = item.get("sample_time")
            timestamp = item.get("timestamp")
            raw_line = item.get("raw_line")
            weight = item.get("weight_g")
            current = item.get("current_A")
            voltage = item.get("voltage_V")

            if isinstance(sample_time, float) and isinstance(weight, float):
                self.weight_data.append(weight)
                self._push_window(self.weight_window, sample_time, weight)
                self.weight_var.set(f"weight_g: {weight:.2f}")
                self.weight_median_var.set(self._format_median(self.weight_window, 2))
                changed = True

            if isinstance(sample_time, float) and isinstance(current, float):
                self.current_data.append(current)
                self._push_window(self.current_window, sample_time, current)
                self.current_var.set(f"current_A: {current:.3f}")
                self.current_median_var.set(self._format_median(self.current_window, 3))
                changed = True

            if isinstance(sample_time, float) and isinstance(voltage, float):
                self.voltage_data.append(voltage)
                self._push_window(self.voltage_window, sample_time, voltage)
                self.voltage_var.set(f"voltage_V: {voltage:.3f}")
                self.voltage_median_var.set(self._format_median(self.voltage_window, 3))
                changed = True

            if isinstance(timestamp, str):
                self._handle_session_sample(sample_time, timestamp, raw_line, weight, current, voltage)

        if changed:
            self._draw_plot(self.weight_canvas, self.weight_data, "#1b6ef3")
            self._draw_plot(self.current_canvas, self.current_data, "#d94841")
            self._draw_plot(self.voltage_canvas, self.voltage_data, "#1e9b62")

        self.root.after(UPDATE_PERIOD_MS, self._process_queue)

    def _push_window(self, window: deque[tuple[float, float]], sample_time: float, value: float) -> None:
        window.append((sample_time, value))
        self._trim_window(window, sample_time)

    def _trim_window(self, window: deque[tuple[float, float]], current_time: float) -> None:
        cutoff = current_time - MEDIAN_WINDOW_S
        while window and window[0][0] < cutoff:
            window.popleft()

    def _format_median(self, window: deque[tuple[float, float]], digits: int) -> str:
        if not window:
            return "--"

        values = [value for _, value in window]
        return f"{median(values):.{digits}f}"

    def _update_save_button(self) -> None:
        state = "normal" if self.session_samples else "disabled"
        self.save_button.config(state=state)

    def _select_log_dir(self) -> None:
        chosen_dir = filedialog.askdirectory(
            title="Select log folder",
            initialdir=str(self.log_dir),
            mustexist=False,
        )
        if not chosen_dir:
            return

        self.log_dir = Path(chosen_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log_dir_var.set(f"Log path: {self.log_dir}")
        self.status_var.set(f"Log folder set to {self.log_dir}")

    def _make_session_dir(self, suffix: str = "") -> Path:
        started_at = self.session_started_at or datetime.now()
        base_name = started_at.strftime("session_%Y%m%d_%H%M%S")
        if suffix:
            base_name += suffix

        session_dir = self.log_dir / base_name
        index = 2
        while session_dir.exists():
            session_dir = self.log_dir / f"{base_name}_{index}"
            index += 1
        return session_dir

    def _save_session(self, suffix: str = "") -> Path | None:
        if not self.session_samples:
            return None

        session_dir = self._make_session_dir(suffix)
        session_dir.mkdir(parents=True, exist_ok=True)

        telemetry_path = session_dir / "telemetry.txt"
        weight_path = session_dir / "weight_g.txt"
        current_path = session_dir / "current_A.txt"
        voltage_path = session_dir / "voltage_V.txt"

        for sample in self.session_samples:
            timestamp = str(sample["timestamp"])
            raw_line = str(sample["raw_line"])
            weight = sample["weight_g"]
            current = sample["current_A"]
            voltage = sample["voltage_V"]

            telemetry_line = (
                f"{timestamp} | weight_g={weight} | current_A={current} | voltage_V={voltage} | raw={raw_line}"
            )
            append_line(telemetry_path, telemetry_line)

            if isinstance(weight, float):
                append_line(weight_path, f"{timestamp} {weight}")
            if isinstance(current, float):
                append_line(current_path, f"{timestamp} {current}")
            if isinstance(voltage, float):
                append_line(voltage_path, f"{timestamp} {voltage}")

        self.session_saved = True
        return session_dir

    def _save_log_button(self) -> None:
        session_dir = self._save_session()
        if session_dir is None:
            self.session_var.set("No captured battery session to save yet.")
            return

        self.session_var.set(f"Log saved: {session_dir}")
        self._update_save_button()

    def _handle_session_sample(
        self,
        sample_time: object,
        timestamp: str,
        raw_line: object,
        weight: object,
        current: object,
        voltage: object,
    ) -> None:
        voltage_value = voltage if isinstance(voltage, float) else None
        powered = voltage_value is not None and voltage_value > SESSION_START_THRESHOLD_V

        if powered and not self.session_active:
            if self.session_samples:
                if not self.session_saved:
                    auto_dir = self._save_session("_auto")
                    if auto_dir is not None:
                        self.session_var.set(f"Previous session auto-saved: {auto_dir}")
                self.session_samples = []
                self.session_saved = False

            self.session_active = True
            self.session_started_at = datetime.now()
            self.last_logged_sample_time = None
            self.session_var.set(
                f"Recording log: battery detected at {voltage_value:.2f} V"
            )

        if self.session_active and powered:
            if not isinstance(sample_time, float):
                return

            if (
                self.last_logged_sample_time is not None
                and sample_time - self.last_logged_sample_time < LOG_RECORD_PERIOD_S
            ):
                return

            self.session_samples.append(
                {
                    "timestamp": timestamp,
                    "raw_line": str(raw_line) if raw_line is not None else "",
                    "weight_g": weight if isinstance(weight, float) else None,
                    "current_A": current if isinstance(current, float) else None,
                    "voltage_V": voltage_value,
                }
            )
            self.last_logged_sample_time = sample_time
            self.session_saved = False
            self._update_save_button()
            return

        if self.session_active and not powered:
            self.session_active = False
            self.last_logged_sample_time = None
            self.session_var.set(
                f"Battery disconnected. Session ready to save: {len(self.session_samples)} samples"
            )
            self._update_save_button()

    def _smooth_values(self, data: list[float]) -> list[float]:
        if len(data) < 3 or PLOT_SMOOTHING_WINDOW <= 1:
            return data

        radius = PLOT_SMOOTHING_WINDOW // 2
        smoothed: list[float] = []

        for index in range(len(data)):
            start = max(0, index - radius)
            end = min(len(data), index + radius + 1)
            window = data[start:end]
            smoothed.append(sum(window) / len(window))

        return smoothed

    def _display_bounds(self, data: list[float]) -> tuple[float, float]:
        if not data:
            return -1.0, 1.0

        sorted_values = sorted(data)

        if len(sorted_values) >= 20:
            lower = percentile(sorted_values, PLOT_OUTLIER_CLIP)
            upper = percentile(sorted_values, 1.0 - PLOT_OUTLIER_CLIP)
        else:
            lower = sorted_values[0]
            upper = sorted_values[-1]

        if upper <= lower:
            lower = min(data)
            upper = max(data)

        span = max(upper - lower, PLOT_MIN_SPAN)
        padding = span * PLOT_Y_PADDING_RATIO
        center = (upper + lower) * 0.5
        half_span = (span * 0.5) + padding
        return center - half_span, center + half_span

    def _draw_plot(self, canvas: tk.Canvas, data: deque[float], color: str) -> None:
        canvas.delete("all")

        width = int(canvas["width"])
        height = int(canvas["height"])
        left = 46
        top = 12
        right = width - 12
        bottom = height - 24

        canvas.create_rectangle(left, top, right, bottom, outline="#d8d2c8")

        for step in range(1, 4):
            y = top + ((bottom - top) * step / 4.0)
            canvas.create_line(left, y, right, y, fill="#eee9e1", width=1)

        if len(data) < 2:
            canvas.create_text(
                width // 2,
                height // 2,
                text="Waiting for data...",
                fill="#7a7a7a",
                font=("TkDefaultFont", 11),
            )
            return

        raw_values = list(data)
        plot_values = self._smooth_values(raw_values)
        minimum, maximum = self._display_bounds(plot_values)

        span = maximum - minimum
        canvas.create_text(24, top, text=f"{maximum:.2f}", fill="#6a6a6a", font=("TkDefaultFont", 9))
        canvas.create_text(24, bottom, text=f"{minimum:.2f}", fill="#6a6a6a", font=("TkDefaultFont", 9))

        points: list[float] = []
        count = len(plot_values) - 1
        x_span = max(1, right - left)
        y_span = max(1, bottom - top)

        for index, value in enumerate(plot_values):
            x = left + (index * x_span / count)
            clamped = min(max(value, minimum), maximum)
            normalized = (clamped - minimum) / span
            y = bottom - (normalized * y_span)
            points.extend((x, y))

        canvas.create_line(
            points,
            fill=color,
            width=2,
            smooth=True,
            splinesteps=PLOT_SPLINE_STEPS,
        )

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    app = TelemetryPlotApp()
    app.run()


if __name__ == "__main__":
    main()

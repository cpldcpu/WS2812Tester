#!/usr/bin/env python3
"""WS2812 test log analyzer GUI.

Scans logs in ../testlogs, lets the user select one file, and renders
measurement summaries plus interpretations on a single page.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path
import math
import re
from typing import Dict, List, Optional, Tuple
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from tkinter import scrolledtext

try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
    from matplotlib.figure import Figure
    from matplotlib.ticker import AutoMinorLocator, MultipleLocator
    HAS_MATPLOTLIB = True
except Exception:  # pragma: no cover - optional dependency
    HAS_MATPLOTLIB = False

try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except Exception:  # pragma: no cover - optional dependency
    HAS_PIL = False


ALLOWED_SUFFIXES = {".txt", ".log", ".md", ""}
IMAGE_SUFFIX_PRIORITY = (".png", ".jpg", ".jpeg")
CLOCK_HZ = 48_000_000.0
CLOCK_NS = 1e9 / CLOCK_HZ


@dataclass
class Table:
    headers: List[str]
    rows: List[Dict[str, str]] = field(default_factory=list)


@dataclass
class Section:
    name: str
    kv: Dict[str, str] = field(default_factory=dict)
    tables: List[Table] = field(default_factory=list)


@dataclass
class ParsedLog:
    path: Path
    sections: Dict[str, Section]
    warnings: List[str] = field(default_factory=list)


@dataclass
class AnalysisResult:
    metrics: List[Tuple[str, str]]
    interpretation: List[str]
    plot_payload: Dict[str, object] = field(default_factory=dict)


def list_log_files(log_dir: Path) -> List[Path]:
    if not log_dir.exists():
        return []
    files = []
    for p in log_dir.iterdir():
        if p.is_file() and not p.name.startswith("."):
            if p.suffix.lower() in ALLOWED_SUFFIXES:
                files.append(p)
    return sorted(files, key=lambda p: p.name.lower())


def parse_number(value: str) -> Optional[float]:
    if value is None:
        return None
    v = value.strip()
    if not v or v.lower() == "na":
        return None
    try:
        return float(int(v, 10))
    except ValueError:
        try:
            return float(v)
        except ValueError:
            return None


def parse_ohms(value: str) -> Optional[float]:
    if value is None:
        return None
    m = re.search(r"-?\d+(?:\.\d+)?", value)
    if not m:
        return None
    return parse_number(m.group(0))


def current_ma_from_vdrop_mv(vdrop_mv: Optional[float], rsense_ohm: Optional[float]) -> Optional[float]:
    if vdrop_mv is None or rsense_ohm is None or rsense_ohm <= 0:
        return None
    return vdrop_mv / rsense_ohm


def format_number(value: Optional[float], decimals: int = 1) -> str:
    if value is None:
        return "n/a"
    if abs(value - round(value)) < 1e-9:
        return str(int(round(value)))
    return f"{value:.{decimals}f}"


def fit_linear_offset_scale(zs: List[float], ys: List[float]) -> Optional[Tuple[float, float]]:
    """Fit y ~= a + b*z via least squares."""
    if len(zs) != len(ys) or len(zs) < 2:
        return None

    n = float(len(zs))
    sz = sum(zs)
    sz2 = sum(z * z for z in zs)
    sy = sum(ys)
    szy = sum(z * y for z, y in zip(zs, ys))
    den = n * sz2 - sz * sz
    if abs(den) < 1e-15:
        return None

    b = (n * szy - sz * sy) / den
    a = (sy - b * sz) / n
    return a, b


def fit_power_law(xs: List[float], ys: List[float]) -> Optional[Tuple[float, float, float]]:
    """Fit y ~= a + b*x^c by scanning c and solving linear LS for a,b."""
    if len(xs) != len(ys) or len(xs) < 3:
        return None

    if not any(x > 0 for x in xs):
        return None

    best_sse = None
    best_params: Optional[Tuple[float, float, float]] = None

    def scan_c(c_start: float, c_stop: float, c_step: float) -> None:
        nonlocal best_sse, best_params
        count = int(round((c_stop - c_start) / c_step))
        for i in range(count + 1):
            c = c_start + i * c_step
            zs = [x ** c if x > 0 else 0.0 for x in xs]
            fit_ab = fit_linear_offset_scale(zs, ys)
            if fit_ab is None:
                continue
            a, b = fit_ab
            preds = [a + b * z for z in zs]
            sse = sum((y - p) * (y - p) for y, p in zip(ys, preds))
            if best_sse is None or sse < best_sse:
                best_sse = sse
                best_params = (a, b, c)

    # Coarse search first.
    scan_c(0.20, 4.00, 0.05)
    if best_params is None:
        return None

    # Fine search around coarse optimum.
    _, _, c0 = best_params
    c_lo = max(0.05, c0 - 0.10)
    c_hi = min(6.00, c0 + 0.10)
    scan_c(c_lo, c_hi, 0.002)
    return best_params


def r2_score(actual: List[float], predicted: List[float]) -> Optional[float]:
    if len(actual) != len(predicted) or len(actual) < 2:
        return None
    mean_a = sum(actual) / len(actual)
    ss_tot = sum((a - mean_a) ** 2 for a in actual)
    ss_res = sum((a - p) ** 2 for a, p in zip(actual, predicted))
    if ss_tot <= 0:
        return None
    return 1.0 - (ss_res / ss_tot)


def build_txh_plot_data(section: Optional[Section]) -> Dict[str, List[float]]:
    out: Dict[str, List[float]] = {
        "txh_clocks": [],
        "txh_ns": [],
        "prop_ns": [],
        "prop_std_ns": [],
        "dur_ns": [],
        "dur_std_ns": [],
    }
    table = first_table(section)
    if table is None:
        return out

    for row in table.rows:
        txh = parse_number(row.get("txh_clocks", ""))
        prop_x10 = parse_number(row.get("prop_x10", ""))
        dur_x10 = parse_number(row.get("dur_x10", ""))
        pvar_x10 = parse_number(row.get("p_var_x10", ""))
        dvar_x10 = parse_number(row.get("d_var_x10", ""))
        if txh is None:
            continue
        prop_ns = None if prop_x10 is None else ((prop_x10 / 10.0) * CLOCK_NS)
        dur_ns = None if dur_x10 is None else ((dur_x10 / 10.0) * CLOCK_NS)
        # var_x10 represents variance * 10 in clock^2 units.
        prop_std_ns = None
        dur_std_ns = None
        if pvar_x10 is not None and pvar_x10 >= 0:
            prop_std_ns = math.sqrt(pvar_x10 / 10.0) * CLOCK_NS
        if dvar_x10 is not None and dvar_x10 >= 0:
            dur_std_ns = math.sqrt(dvar_x10 / 10.0) * CLOCK_NS
        if prop_ns is None and dur_ns is None:
            continue
        out["txh_clocks"].append(txh)
        out["txh_ns"].append(txh * CLOCK_NS)
        out["prop_ns"].append(prop_ns if prop_ns is not None else math.nan)
        out["prop_std_ns"].append(prop_std_ns if prop_std_ns is not None else math.nan)
        out["dur_ns"].append(dur_ns if dur_ns is not None else math.nan)
        out["dur_std_ns"].append(dur_std_ns if dur_std_ns is not None else math.nan)
    return out


def build_led_channel_analysis(section: Optional[Section], rsense_ohm: Optional[float]) -> Dict[str, Dict[str, object]]:
    groups: Dict[str, List[Tuple[int, float]]] = {}
    table = first_table(section)
    if table is None:
        return {}

    for row in table.rows:
        ch = row.get("channel", "")
        duty = parse_number(row.get("duty", ""))
        vdrop = parse_number(row.get("vdrop_avg_mv", ""))
        if not ch or duty is None or vdrop is None:
            continue
        groups.setdefault(ch, []).append((int(duty), float(vdrop)))

    out: Dict[str, Dict[str, object]] = {}
    for ch in sorted(groups):
        pts = sorted(groups[ch], key=lambda x: x[0])
        duty = [d for d, _ in pts]
        vdrop_mv = [v for _, v in pts]
        if not duty:
            continue

        baseline_mv = None
        for d, v in pts:
            if d == 0:
                baseline_mv = v
                break
        if baseline_mv is None:
            baseline_mv = min(vdrop_mv)

        values = vdrop_mv
        unit = "mV"
        baseline = baseline_mv
        if rsense_ohm is not None and rsense_ohm > 0:
            values = [v / rsense_ohm for v in vdrop_mv]
            baseline = baseline_mv / rsense_ohm
            unit = "mA"

        xs_all: List[float] = []
        ys_all: List[float] = []
        for d, y in zip(duty, values):
            xs_all.append(d / 255.0)
            ys_all.append(y)

        fit_params = fit_power_law(xs_all, ys_all)
        fit_offset = None
        power_scale = None
        power_exp = None
        if fit_params is not None:
            fit_offset, power_scale, power_exp = fit_params

        fit_values: List[float] = []
        if fit_offset is not None and power_scale is not None and power_exp is not None:
            for d in duty:
                x = d / 255.0
                fit_values.append(fit_offset + power_scale * (x ** power_exp if x > 0 else 0.0))
        else:
            fit_values = [math.nan for _ in duty]

        actual = values
        pred = [p for p in fit_values if not math.isnan(p)]
        linearity_r2 = r2_score(actual, pred) if len(actual) == len(pred) else None

        on_est = None
        if fit_offset is not None and power_scale is not None:
            on_est = fit_offset + power_scale

        power_dev_pct = None
        if power_exp is not None:
            power_dev_pct = (power_exp - 1.0) * 100.0

        on_max = max(values) if values else None
        max_duty = max(duty)
        max_duty_val = values[duty.index(max_duty)]

        out[ch] = {
            "duty": duty,
            "values": values,
            "fit_values": fit_values,
            "unit": unit,
            "baseline": baseline,
            "fit_offset": fit_offset,
            "linearity_r2": linearity_r2,
            "power_scale": power_scale,
            "power_exp": power_exp,
            "power_dev_pct": power_dev_pct,
            "on_est": on_est,
            "on_max": on_max,
            "max_duty": max_duty,
            "max_duty_value": max_duty_val,
        }

    return out


def parse_log_file(path: Path) -> ParsedLog:
    text = path.read_text(encoding="utf-8", errors="replace")
    sections: Dict[str, Section] = {}
    warnings: List[str] = []
    current: Optional[Section] = None
    active_table: Optional[Table] = None

    for raw_line in text.splitlines():
        line = raw_line.strip("\r\n")
        stripped = line.strip()
        if not stripped:
            continue

        if stripped.startswith("## "):
            section_name = stripped[3:].strip()
            current = Section(name=section_name)
            sections[section_name] = current
            active_table = None
            continue

        if current is None:
            # Ignore preamble/tool noise before first section.
            continue

        if "\t" not in line:
            # Non-TSV content inside section; keep going, but record once.
            warnings.append(f"Ignored non-TSV line in section '{current.name}': {stripped[:40]}")
            continue

        cols = [c.strip() for c in line.split("\t")]
        ncols = len(cols)

        # key/value line
        if ncols == 2:
            current.kv[cols[0]] = cols[1]
            active_table = None
            continue

        # table-like content (>=3 columns)
        if active_table is None:
            active_table = Table(headers=cols)
            current.tables.append(active_table)
            continue

        if len(active_table.headers) == ncols:
            active_table.rows.append(dict(zip(active_table.headers, cols)))
        else:
            # New table header inside same section
            active_table = Table(headers=cols)
            current.tables.append(active_table)

    # Deduplicate repeated warning strings while preserving order.
    seen = set()
    deduped_warnings = []
    for w in warnings:
        if w not in seen:
            deduped_warnings.append(w)
            seen.add(w)

    return ParsedLog(path=path, sections=sections, warnings=deduped_warnings)


def find_section(parsed: ParsedLog, exact: str) -> Optional[Section]:
    return parsed.sections.get(exact)


def find_section_startswith(parsed: ParsedLog, prefix: str) -> Optional[Section]:
    lower_prefix = prefix.lower()
    for name, sec in parsed.sections.items():
        if name.lower().startswith(lower_prefix):
            return sec
    return None


def first_table(section: Optional[Section]) -> Optional[Table]:
    if section is None or not section.tables:
        return None
    return section.tables[0]


def analyze_txh(section: Optional[Section]) -> Tuple[List[Tuple[str, str]], List[str]]:
    metrics: List[Tuple[str, str]] = []
    notes: List[str] = []
    if section is None:
        notes.append("txh_sweep section missing.")
        return metrics, notes

    status = section.kv.get("status", "unknown")
    metrics.append(("txh status", status))

    table = first_table(section)
    if table is None:
        notes.append("txh_sweep has no table data.")
        return metrics, notes

    metrics.append(("txh rows", str(len(table.rows))))

    numeric_rows = []
    for row in table.rows:
        txh = parse_number(row.get("txh_clocks", ""))
        dur = parse_number(row.get("dur_x10", ""))
        if txh is not None and dur is not None:
            numeric_rows.append((int(txh), dur))

    metrics.append(("txh numeric rows", str(len(numeric_rows))))

    if len(numeric_rows) >= 10:
        early = [dur for _, dur in numeric_rows[:10]]
        early_avg = sum(early) / len(early)
        transition = None
        for txh, dur in numeric_rows:
            if dur > (early_avg * 1.5):
                transition = txh
                break
        metrics.append(("txh early avg dur_x10", format_number(early_avg, 1)))
        metrics.append(("txh transition estimate", str(transition) if transition else "n/a"))
        if transition is not None:
            metrics.append(("txh transition_ns", format_number(transition * CLOCK_NS, 1)))
        if transition is not None:
            notes.append(
                f"txh duration transition appears near txH={transition}, indicating timing knee behavior."
            )

    txh_values = [txh for txh, _ in numeric_rows]
    if txh_values:
        txh_min = min(txh_values)
        txh_max = max(txh_values)
        metrics.append(("txh_ns range", f"{format_number(txh_min * CLOCK_NS,1)}..{format_number(txh_max * CLOCK_NS,1)}"))

    prop_vals = []
    dur_vals = []
    pvar_vals = []
    dvar_vals = []
    for row in table.rows:
        prop_x10 = parse_number(row.get("prop_x10", ""))
        dur_x10 = parse_number(row.get("dur_x10", ""))
        pvar_x10 = parse_number(row.get("p_var_x10", ""))
        dvar_x10 = parse_number(row.get("d_var_x10", ""))
        if prop_x10 is not None:
            prop_vals.append(prop_x10)
        if dur_x10 is not None:
            dur_vals.append(dur_x10)
        if pvar_x10 is not None:
            pvar_vals.append(pvar_x10)
        if dvar_x10 is not None:
            dvar_vals.append(dvar_x10)

    if prop_vals:
        prop_clk = (sum(prop_vals) / len(prop_vals)) / 10.0
        metrics.append(("txh avg prop_ns", format_number(prop_clk * CLOCK_NS, 1)))
    if dur_vals:
        dur_clk = (sum(dur_vals) / len(dur_vals)) / 10.0
        metrics.append(("txh avg dur_ns", format_number(dur_clk * CLOCK_NS, 1)))
    if pvar_vals:
        metrics.append(("txh avg p_var_x10", format_number(sum(pvar_vals) / len(pvar_vals), 1)))
    if dvar_vals:
        metrics.append(("txh avg d_var_x10", format_number(sum(dvar_vals) / len(dvar_vals), 1)))

    if status == "ok" and len(table.rows) >= 64:
        notes.append("txh sweep completed with full row count.")
    elif status != "ok":
        notes.append("txh sweep reported failure status.")

    return metrics, notes


def analyze_reset(section: Optional[Section]) -> Tuple[List[Tuple[str, str]], List[str]]:
    metrics: List[Tuple[str, str]] = []
    notes: List[str] = []
    if section is None:
        notes.append("reset_time section missing.")
        return metrics, notes

    status = section.kv.get("status", "unknown")
    metrics.append(("reset status", status))

    thr_clocks = parse_number(section.kv.get("threshold_clocks", ""))
    if thr_clocks is not None:
        thr_us = thr_clocks / 48.0  # 48 MHz timer clocks
        metrics.append(("reset threshold_clocks", format_number(thr_clocks, 0)))
        metrics.append(("reset threshold_us", format_number(thr_us, 2)))
        if thr_us >= 50.0:
            notes.append("reset threshold is above 50 us, consistent with WS2812 reset behavior.")
        else:
            notes.append("reset threshold is below 50 us; verify capture and timing assumptions.")
    else:
        metrics.append(("reset threshold_clocks", section.kv.get("threshold_clocks", "n/a")))

    return metrics, notes


def analyze_pwm(section: Optional[Section], rsense_ohm: Optional[float]) -> Tuple[List[Tuple[str, str]], List[str]]:
    metrics: List[Tuple[str, str]] = []
    notes: List[str] = []
    if section is None:
        notes.append("pwm_rate section missing.")
        return metrics, notes

    status = section.kv.get("status", "unknown")
    metrics.append(("pwm status", status))

    pwm_hz = parse_number(section.kv.get("pwm_hz", ""))
    vdrop_off = parse_number(section.kv.get("vdrop_off_mv", ""))
    vdrop_on = parse_number(section.kv.get("vdrop_on_mv", ""))
    vdrop_led = parse_number(section.kv.get("vdrop_led_mv", ""))

    metrics.append(("pwm_hz", format_number(pwm_hz, 1)))
    metrics.append(("vdrop_off_mv", format_number(vdrop_off, 0)))
    metrics.append(("vdrop_on_mv", format_number(vdrop_on, 0)))
    metrics.append(("vdrop_led_mv", format_number(vdrop_led, 0)))
    if pwm_hz is not None and pwm_hz > 0:
        metrics.append(("pwm_period_us", format_number(1e6 / pwm_hz, 2)))

    i_off = current_ma_from_vdrop_mv(vdrop_off, rsense_ohm)
    i_on = current_ma_from_vdrop_mv(vdrop_on, rsense_ohm)
    i_led = current_ma_from_vdrop_mv(vdrop_led, rsense_ohm)
    if i_off is not None:
        metrics.append(("i_off_ma", format_number(i_off, 2)))
    if i_on is not None:
        metrics.append(("i_on_ma", format_number(i_on, 2)))
    if i_led is not None:
        metrics.append(("i_led_delta_ma", format_number(i_led, 2)))

    if pwm_hz is not None:
        if 400 <= pwm_hz <= 20000:
            notes.append("PWM frequency is inside the configured detection range (400..20000 Hz).")
        else:
            notes.append("PWM frequency is outside the configured detection range.")

    if vdrop_led is not None and vdrop_led > 0:
        notes.append("LED on/off drop delta is positive, indicating measurable load modulation.")

    if status != "ok":
        reason = section.kv.get("reason", "unknown")
        notes.append(f"PWM test reported failure reason: {reason}.")

    return metrics, notes


def analyze_led_current(
    section: Optional[Section], rsense_ohm: Optional[float]
) -> Tuple[List[Tuple[str, str]], List[str], Dict[str, Dict[str, object]]]:
    metrics: List[Tuple[str, str]] = []
    notes: List[str] = []
    if section is None:
        notes.append("led_current section missing.")
        return metrics, notes, {}

    status = section.kv.get("status", "unknown")
    metrics.append(("led_current status", status))

    channel_data = build_led_channel_analysis(section, rsense_ohm)
    if not channel_data:
        notes.append("led_current has no table rows.")
        return metrics, notes, {}

    metrics.append(("led channels", str(len(channel_data))))

    max_duty_values: List[float] = []
    standby_values: List[float] = []
    for ch in sorted(channel_data):
        info = channel_data[ch]
        duty = info["duty"]
        values = info["values"]
        unit = info["unit"]
        metrics.append((f"{ch} points", str(len(duty))))

        if values:
            min_v = min(values)
            max_v = max(values)
            metrics.append((f"{ch} range", f"{format_number(min_v,2)}..{format_number(max_v,2)} {unit}"))

            decreases = 0
            for i in range(1, len(values)):
                if values[i] + 1e-9 < values[i - 1]:
                    decreases += 1
            metrics.append((f"{ch} monotonic drops", str(decreases)))

            max_duty = info["max_duty"]
            max_duty_val = info["max_duty_value"]
            max_duty_values.append(max_duty_val)
            metrics.append((f"{ch} @duty{max_duty}", f"{format_number(max_duty_val,2)} {unit}"))

            baseline = info["baseline"]
            if baseline is not None:
                standby_values.append(baseline)
                metrics.append((f"{ch} standby", f"{format_number(baseline,2)} {unit}"))

            on_est = info["on_est"]
            on_max = info["on_max"]
            if on_est is not None:
                metrics.append((f"{ch} on-current est", f"{format_number(on_est,2)} {unit}"))
            if on_max is not None:
                metrics.append((f"{ch} on-current observed max", f"{format_number(on_max,2)} {unit}"))

            r2 = info["linearity_r2"]
            power_exp = info["power_exp"]
            power_dev = info["power_dev_pct"]
            metrics.append((f"{ch} linearity r2", format_number(r2, 4)))
            metrics.append((f"{ch} power exponent c", format_number(power_exp, 4)))
            metrics.append((f"{ch} power exponent dev", f"{format_number(power_dev,2)} %"))

            transformed = False
            if r2 is not None and r2 < 0.995:
                transformed = True
            if power_exp is not None and abs(power_exp - 1.0) > 0.08:
                transformed = True

            if transformed:
                notes.append(f"{ch} sweep appears non-linear (possible internal transfer shaping).")
            else:
                notes.append(f"{ch} sweep is close to linear across duty.")

            if power_exp is not None:
                if power_exp > 1.05:
                    notes.append(f"{ch} power exponent > 1 (compressed at low duty, expanded near full scale).")
                elif power_exp < 0.95:
                    notes.append(f"{ch} power exponent < 1 (expanded at low duty, compressed near full scale).")

            if decreases <= 2:
                notes.append(f"{ch} monotonicity is good.")
            else:
                notes.append(f"{ch} shows {decreases} downward steps; check noise or quantization.")

    if len(max_duty_values) >= 2:
        spread = max(max_duty_values) - min(max_duty_values)
        metrics.append(("channel spread @max duty", f"{format_number(spread,2)} {channel_data[sorted(channel_data)[0]]['unit']}"))
        if spread <= 1.0:
            notes.append("Channel full-scale mismatch is small.")
        else:
            notes.append("Channel full-scale mismatch is noticeable.")

    if standby_values:
        avg_standby = sum(standby_values) / len(standby_values)
        metrics.append(("standby current consumption", f"{format_number(avg_standby,2)} {channel_data[sorted(channel_data)[0]]['unit']}"))

    return metrics, notes, channel_data


def analyze_log(parsed: ParsedLog) -> AnalysisResult:
    metrics: List[Tuple[str, str]] = []
    interpretation: List[str] = []

    metrics.append(("File", parsed.path.name))
    metrics.append(("Sections found", str(len(parsed.sections))))

    boot = find_section(parsed, "boot")
    if boot:
        fw = boot.kv.get("fw")
        mcu = boot.kv.get("mcu")
        rsense = boot.kv.get("Rsense [Ohm]")
        if fw:
            metrics.append(("Firmware", fw))
        if mcu:
            metrics.append(("MCU", mcu))
        if rsense:
            metrics.append(("Rsense [Ohm]", rsense))
    rsense_ohm = parse_ohms(boot.kv.get("Rsense [Ohm]", "")) if boot else None

    device = find_section_startswith(parsed, "device")
    device_channel_order: Optional[str] = None
    if device:
        for key in ("Manufacturer", "Type", "IC", "Channel Order"):
            if key in device.kv:
                metrics.append((f"Device {key}", device.kv[key]))
        device_channel_order = device.kv.get("Channel Order")

    txh_section = find_section(parsed, "txh_sweep")
    reset_section = find_section(parsed, "reset_time")
    pwm_section = find_section(parsed, "pwm_rate")
    led_section = find_section(parsed, "led_current")

    txh_metrics, txh_notes = analyze_txh(txh_section)
    reset_metrics, reset_notes = analyze_reset(reset_section)
    pwm_metrics, pwm_notes = analyze_pwm(pwm_section, rsense_ohm)
    led_metrics, led_notes, led_channel_data = analyze_led_current(led_section, rsense_ohm)

    metrics.extend(txh_metrics)
    metrics.extend(reset_metrics)
    metrics.extend(pwm_metrics)
    metrics.extend(led_metrics)

    interpretation.append("Overview")
    for n in txh_notes + reset_notes + pwm_notes + led_notes:
        interpretation.append(f"- {n}")

    interpretation.append("")
    interpretation.append("Derived key information")
    if rsense_ohm is not None:
        interpretation.append(f"- Sense resistor used for current conversion: {format_number(rsense_ohm,3)} ohm.")
    else:
        interpretation.append("- Rsense value missing; current metrics are unavailable.")

    if reset_section:
        thr_clocks = parse_number(reset_section.kv.get("threshold_clocks", ""))
        if thr_clocks is not None:
            thr_us = thr_clocks / 48.0
            interpretation.append(
                f"- Reset threshold: {format_number(thr_clocks,0)} clocks ({format_number(thr_us,2)} us at 48 MHz)."
            )

    if pwm_section:
        pwm_hz = parse_number(pwm_section.kv.get("pwm_hz", ""))
        vdrop_led = parse_number(pwm_section.kv.get("vdrop_led_mv", ""))
        if pwm_hz is not None:
            interpretation.append(
                f"- PWM period estimate: {format_number(1e6 / pwm_hz,2)} us ({format_number(pwm_hz,1)} Hz)."
            )
        if vdrop_led is not None and rsense_ohm and rsense_ohm > 0:
            interpretation.append(
                f"- PWM LED delta current: {format_number(vdrop_led / rsense_ohm,2)} mA."
            )

    txh_table = first_table(txh_section)
    if txh_table and txh_table.rows:
        first = txh_table.rows[0]
        last = txh_table.rows[-1]
        t0 = parse_number(first.get("txh_clocks", ""))
        t1 = parse_number(last.get("txh_clocks", ""))
        if t0 is not None and t1 is not None:
            interpretation.append(
                f"- txH sweep coverage: {int(t0)}..{int(t1)} clocks "
                f"({format_number(t0 * CLOCK_NS,1)}..{format_number(t1 * CLOCK_NS,1)} ns)."
            )

    interpretation.append("")
    interpretation.append("Helpful plot ideas")
    if txh_table and txh_table.rows:
        interpretation.append(
            "- Propagation delay vs txH(ns) with stddev error bars derived from p_var_x10."
        )
        interpretation.append(
            "- Pulse length vs txH(ns) with stddev error bars derived from d_var_x10."
        )
    led_table = first_table(led_section)
    if led_table and led_table.rows:
        if rsense_ohm and rsense_ohm > 0:
            interpretation.append(
                "- Plot duty vs current (mA) per channel (CH1..CH3), with a dedicated low-duty zoom for 0..31."
            )
        else:
            interpretation.append(
                "- Plot duty vs vdrop_avg_mv per channel (CH1..CH3), with a dedicated low-duty zoom for 0..31."
            )

    # Per-channel maximum LED on-current and linearity summary from duty sweep.
    if led_channel_data:
        interpretation.append("")
        interpretation.append("LED channel on-current and linearity")
        for ch in sorted(led_channel_data):
            info = led_channel_data[ch]
            unit = info["unit"]
            on_est = info["on_est"]
            on_max = info["on_max"]
            r2 = info["linearity_r2"]
            power_exp = info["power_exp"]
            power_dev = info["power_dev_pct"]
            interpretation.append(
                f"- {ch}: on-current est={format_number(on_est,2)} {unit}, "
                f"observed max={format_number(on_max,2)} {unit}, "
                f"linearity r2={format_number(r2,4)}, "
                f"power exponent c={format_number(power_exp,4)}, "
                f"power exponent dev={format_number(power_dev,2)}%."
            )

    interpretation.append("")
    interpretation.append("Requested parameter list")
    if led_channel_data:
        channel_names = sorted(led_channel_data)
        for ch in channel_names:
            info = led_channel_data[ch]
            unit = info["unit"]
            on_value = info["on_est"] if info["on_est"] is not None else info["on_max"]
            interpretation.append(f"- {ch} on current: {format_number(on_value,2)} {unit}")
        baselines = [led_channel_data[ch]["baseline"] for ch in channel_names if led_channel_data[ch]["baseline"] is not None]
        if baselines:
            interpretation.append(
                f"- standby current consumption: {format_number(sum(baselines)/len(baselines),2)} {led_channel_data[channel_names[0]]['unit']}"
            )
        else:
            interpretation.append("- standby current consumption: n/a")
    else:
        interpretation.append("- CH1 on current: n/a")
        interpretation.append("- CH2 on current: n/a")
        interpretation.append("- CH3 on current: n/a")
        interpretation.append("- standby current consumption: n/a")

    pwm_freq = parse_number(pwm_section.kv.get("pwm_hz", "")) if pwm_section else None
    interpretation.append(f"- PWM frequency: {format_number(pwm_freq,1)} Hz")

    tests_complete = find_section(parsed, "tests_complete")
    if tests_complete is not None:
        overall = tests_complete.kv.get("status", "unknown")
        metrics.append(("Overall status", overall))
        if overall == "ok":
            interpretation.append("- tests_complete reports status=ok.")
        else:
            interpretation.append(f"- tests_complete reports status={overall}.")

    if parsed.warnings:
        interpretation.append("\nParser notes")
        for w in parsed.warnings[:8]:
            interpretation.append(f"- {w}")
        if len(parsed.warnings) > 8:
            interpretation.append(f"- ... and {len(parsed.warnings) - 8} more")

    plot_payload: Dict[str, object] = {
        "txh": build_txh_plot_data(txh_section),
        "led_channels": led_channel_data,
        "pwm": {
            "pwm_hz": parse_number(pwm_section.kv.get("pwm_hz", "")) if pwm_section else None,
            "vref_mv": parse_number(pwm_section.kv.get("vref_mv", "")) if pwm_section else None,
            "vdrop_off_mv": parse_number(pwm_section.kv.get("vdrop_off_mv", "")) if pwm_section else None,
            "vdrop_on_mv": parse_number(pwm_section.kv.get("vdrop_on_mv", "")) if pwm_section else None,
            "vdrop_led_mv": parse_number(pwm_section.kv.get("vdrop_led_mv", "")) if pwm_section else None,
            "i_off_ma": current_ma_from_vdrop_mv(
                parse_number(pwm_section.kv.get("vdrop_off_mv", "")) if pwm_section else None, rsense_ohm
            ),
            "i_on_ma": current_ma_from_vdrop_mv(
                parse_number(pwm_section.kv.get("vdrop_on_mv", "")) if pwm_section else None, rsense_ohm
            ),
            "i_led_delta_ma": current_ma_from_vdrop_mv(
                parse_number(pwm_section.kv.get("vdrop_led_mv", "")) if pwm_section else None, rsense_ohm
            ),
        },
        "reset_threshold_us": (
            (parse_number(reset_section.kv.get("threshold_clocks", "")) / 48.0)
            if (reset_section and parse_number(reset_section.kv.get("threshold_clocks", "")) is not None)
            else None
        ),
        "channel_order": device_channel_order,
    }

    return AnalysisResult(metrics=metrics, interpretation=interpretation, plot_payload=plot_payload)


class AnalyzerApp:
    def __init__(self, root: tk.Tk, logs_dir: Path) -> None:
        self.root = root
        self.logs_dir = logs_dir
        self.root.title("WS2812 Log Analyzer")
        self.root.geometry("1200x760")

        self.file_var = tk.StringVar()
        self.files: List[Path] = []
        self.current_log_path: Optional[Path] = None
        self.image_path: Optional[Path] = None
        self.image_src = None
        self.image_tk = None

        self._build_ui()
        self.refresh_file_list()

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        controls = ttk.Frame(self.root, padding=(10, 10, 10, 8))
        controls.grid(row=0, column=0, sticky="ew")
        controls.columnconfigure(1, weight=1)

        ttk.Label(controls, text="Log file:").grid(row=0, column=0, sticky="w")
        self.combo = ttk.Combobox(controls, textvariable=self.file_var, state="readonly")
        self.combo.grid(row=0, column=1, padx=(8, 8), sticky="ew")
        self.combo.bind("<<ComboboxSelected>>", self._on_file_selected)

        ttk.Button(controls, text="Refresh", command=self.refresh_file_list).grid(row=0, column=2, sticky="e")
        self.path_label = ttk.Label(controls, text=f"Source: {self.logs_dir}")
        self.path_label.grid(row=1, column=0, columnspan=3, sticky="w", pady=(6, 0))

        main = ttk.Panedwindow(self.root, orient="horizontal")
        main.grid(row=1, column=0, sticky="nsew", padx=10, pady=(0, 10))

        left = ttk.Labelframe(main, text="Measurement Summary", padding=8)
        right = ttk.Labelframe(main, text="Details", padding=8)
        main.add(left, weight=1)
        main.add(right, weight=1)

        left.columnconfigure(0, weight=1)
        left.rowconfigure(0, weight=1)

        self.tree = ttk.Treeview(left, columns=("metric", "value"), show="headings", height=28)
        self.tree.heading("metric", text="Metric")
        self.tree.heading("value", text="Value")
        self.tree.column("metric", width=300, anchor="w")
        self.tree.column("value", width=260, anchor="w")
        self.tree.grid(row=0, column=0, sticky="nsew")

        yscroll = ttk.Scrollbar(left, orient="vertical", command=self.tree.yview)
        yscroll.grid(row=0, column=1, sticky="ns")
        self.tree.configure(yscrollcommand=yscroll.set)

        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1)

        self.right_tabs = ttk.Notebook(right)
        self.right_tabs.grid(row=0, column=0, sticky="nsew")

        interp_tab = ttk.Frame(self.right_tabs)
        interp_tab.columnconfigure(0, weight=1)
        interp_tab.rowconfigure(0, weight=1)
        self.right_tabs.add(interp_tab, text="Interpretation")

        plots_tab = ttk.Frame(self.right_tabs)
        plots_tab.columnconfigure(0, weight=1)
        plots_tab.rowconfigure(0, weight=1)
        self.right_tabs.add(plots_tab, text="Plots")

        image_tab = ttk.Frame(self.right_tabs)
        image_tab.columnconfigure(0, weight=1)
        image_tab.rowconfigure(1, weight=1)
        self.right_tabs.add(image_tab, text="Image")

        self.text = scrolledtext.ScrolledText(interp_tab, wrap="word", font=("Consolas", 10))
        self.text.grid(row=0, column=0, sticky="nsew")
        self.text.configure(state="disabled")

        self.plot_message = ttk.Label(
            plots_tab,
            text="",
            anchor="center",
            justify="center",
        )
        self.plot_canvas = None
        self.plot_figure = None
        self.plot_toolbar = None

        if HAS_MATPLOTLIB:
            self.plot_figure = Figure(figsize=(10.0, 4.6), dpi=100, constrained_layout=True)
            self.plot_canvas = FigureCanvasTkAgg(self.plot_figure, master=plots_tab)
            self.plot_canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
            self.plot_toolbar = NavigationToolbar2Tk(self.plot_canvas, plots_tab, pack_toolbar=False)
            self.plot_toolbar.update()
            self.plot_toolbar.grid(row=1, column=0, sticky="ew")
        else:
            self.plot_message.configure(
                text=(
                    "Matplotlib is not available in this Python environment.\\n"
                    "Install matplotlib to enable embedded plots."
                )
            )
            self.plot_message.grid(row=0, column=0, sticky="nsew")

        self.image_info = ttk.Label(image_tab, text="", anchor="w", justify="left")
        self.image_info.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        self.image_label = ttk.Label(image_tab, text="No image loaded.", anchor="center", justify="center")
        self.image_label.grid(row=1, column=0, sticky="nsew")
        self.image_label.bind("<Configure>", self._on_image_resize)

    def refresh_file_list(self) -> None:
        self.files = list_log_files(self.logs_dir)
        names = [p.name for p in self.files]
        self.combo["values"] = names

        if not names:
            self.file_var.set("")
            self._render_empty("No log files found in testlogs.")
            return

        current = self.file_var.get()
        if current not in names:
            self.file_var.set(names[0])

        self._load_selected_file()

    def _on_file_selected(self, _event: object) -> None:
        self._load_selected_file()

    def _load_selected_file(self) -> None:
        name = self.file_var.get()
        target = None
        for p in self.files:
            if p.name == name:
                target = p
                break

        if target is None:
            self._render_empty("Selected file not found.")
            return

        self.current_log_path = target
        self._render_image_for_log(target)

        try:
            parsed = parse_log_file(target)
            analysis = analyze_log(parsed)
        except Exception as exc:  # pragma: no cover - GUI safety fallback
            messagebox.showerror("Parse Error", f"Failed to parse file:\n{target}\n\n{exc}")
            self._render_empty("Parse failed.")
            return

        self._render_analysis(analysis)

    def _render_empty(self, message: str) -> None:
        for item in self.tree.get_children():
            self.tree.delete(item)
        self.tree.insert("", "end", values=("Status", message))

        self.text.configure(state="normal")
        self.text.delete("1.0", tk.END)
        self.text.insert(tk.END, message)
        self.text.configure(state="disabled")
        self._render_plots({})
        self._clear_image(message)

    def _render_analysis(self, analysis: AnalysisResult) -> None:
        for item in self.tree.get_children():
            self.tree.delete(item)

        for metric, value in analysis.metrics:
            self.tree.insert("", "end", values=(metric, value))

        self.text.configure(state="normal")
        self.text.delete("1.0", tk.END)
        self.text.insert(tk.END, "\n".join(analysis.interpretation))
        self.text.configure(state="disabled")
        self._render_plots(analysis.plot_payload)

    def _find_matching_image(self, log_path: Path) -> Optional[Path]:
        log_stem = log_path.stem.lower()
        by_suffix: Dict[str, Path] = {}
        for p in log_path.parent.iterdir():
            if not p.is_file():
                continue
            if p.stem.lower() != log_stem:
                continue
            suffix = p.suffix.lower()
            if suffix in IMAGE_SUFFIX_PRIORITY and suffix not in by_suffix:
                by_suffix[suffix] = p

        for suffix in IMAGE_SUFFIX_PRIORITY:
            if suffix in by_suffix:
                return by_suffix[suffix]
        return None

    def _clear_image(self, message: str) -> None:
        self.image_path = None
        self.image_src = None
        self.image_tk = None
        self.image_info.configure(text=message)
        self.image_label.configure(image="", text="No image available.")

    def _render_image_for_log(self, log_path: Path) -> None:
        img_path = self._find_matching_image(log_path)
        if img_path is None:
            self._clear_image("No matching image found (expected: same-name .png/.jpg/.jpeg).")
            return

        self.image_path = img_path
        self.image_info.configure(text=f"Image: {img_path.name}")
        self.image_tk = None
        self.image_src = None

        suffix = img_path.suffix.lower()
        if HAS_PIL:
            try:
                with Image.open(img_path) as im:
                    self.image_src = im.convert("RGBA")
            except Exception as exc:
                self._clear_image(f"Failed to load image: {exc}")
                return
            self._refresh_image_widget()
            return

        if suffix == ".png":
            try:
                self.image_src = tk.PhotoImage(file=str(img_path))
            except Exception as exc:
                self._clear_image(f"Failed to load PNG image: {exc}")
                return
            self._refresh_image_widget()
            return

        self._clear_image("JPEG display requires Pillow (`pip install pillow`).")

    def _refresh_image_widget(self) -> None:
        if self.image_src is None:
            self.image_label.configure(image="", text="No image available.")
            return

        avail_w = max(self.image_label.winfo_width() - 8, 32)
        avail_h = max(self.image_label.winfo_height() - 8, 32)

        if HAS_PIL and hasattr(self.image_src, "size"):
            src = self.image_src
            width, height = src.size
            if width <= 0 or height <= 0:
                self.image_label.configure(image="", text="Invalid image dimensions.")
                return

            scale = min(float(avail_w) / float(width), float(avail_h) / float(height), 1.0)
            new_w = max(1, int(round(width * scale)))
            new_h = max(1, int(round(height * scale)))
            if new_w == width and new_h == height:
                resized = src
            else:
                resampling = getattr(Image, "Resampling", Image)
                resized = src.resize((new_w, new_h), resampling.LANCZOS)
            self.image_tk = ImageTk.PhotoImage(resized)
            self.image_label.configure(image=self.image_tk, text="")
            return

        if isinstance(self.image_src, tk.PhotoImage):
            src = self.image_src
            xsub = max(1, int(math.ceil(src.width() / float(avail_w))))
            ysub = max(1, int(math.ceil(src.height() / float(avail_h))))
            shown = src.subsample(xsub, ysub) if (xsub > 1 or ysub > 1) else src
            self.image_tk = shown
            self.image_label.configure(image=self.image_tk, text="")
            return

        self.image_label.configure(image="", text="Unsupported image type.")

    def _on_image_resize(self, _event: object) -> None:
        if self.image_src is not None:
            self._refresh_image_widget()

    def _render_plots(self, payload: Dict[str, object]) -> None:
        if not HAS_MATPLOTLIB:
            return
        if self.plot_figure is None or self.plot_canvas is None:
            return

        self.plot_figure.clear()

        ax_timing = self.plot_figure.add_subplot(1, 2, 1)
        ax_led = self.plot_figure.add_subplot(1, 2, 2)

        # 1) Combined Din->Dout pulse regeneration plot with stddev error bars.
        txh = payload.get("txh", {}) if payload else {}
        reset_threshold_us = payload.get("reset_threshold_us") if payload else None
        txh_x_ns = txh.get("txh_ns", []) if isinstance(txh, dict) else []
        prop_ns = txh.get("prop_ns", []) if isinstance(txh, dict) else []
        prop_std_ns = txh.get("prop_std_ns", []) if isinstance(txh, dict) else []
        dur_ns = txh.get("dur_ns", []) if isinstance(txh, dict) else []
        dur_std_ns = txh.get("dur_std_ns", []) if isinstance(txh, dict) else []

        if txh_x_ns:
            x_prop, y_prop, e_prop = [], [], []
            x_dur, y_dur, e_dur = [], [], []
            for x, y, e in zip(txh_x_ns, prop_ns, prop_std_ns):
                if all(math.isfinite(v) for v in (x, y)):
                    x_prop.append(x)
                    y_prop.append(y)
                    e_prop.append(e if math.isfinite(e) and e >= 0 else 0.0)
            for x, y, e in zip(txh_x_ns, dur_ns, dur_std_ns):
                if all(math.isfinite(v) for v in (x, y)):
                    x_dur.append(x)
                    y_dur.append(y)
                    e_dur.append(e if math.isfinite(e) and e >= 0 else 0.0)
            plotted = False
            if x_prop:
                ax_timing.errorbar(
                    x_prop,
                    y_prop,
                    yerr=e_prop,
                    fmt="-o",
                    markersize=2.5,
                    linewidth=1.2,
                    elinewidth=0.8,
                    capsize=2,
                    label="Propagation Delay",
                )
                plotted = True
            if x_dur:
                ax_timing.errorbar(
                    x_dur,
                    y_dur,
                    yerr=e_dur,
                    fmt="-o",
                    markersize=2.5,
                    linewidth=1.2,
                    elinewidth=0.8,
                    capsize=2,
                    color="#ff7f0e",
                    label="Pulse Length",
                )
                plotted = True
            if plotted:
                ax_timing.set_xlim(0.0, 1300.0)
                ax_timing.set_ylim(0.0, 1300.0)
                ax_timing.plot(
                    [0.0, 1300.0],
                    [0.0, 1300.0],
                    linestyle="--",
                    linewidth=1.0,
                    color="0.6",
                    zorder=0,
                    label="_nolegend_",
                )
                ax_timing.xaxis.set_major_locator(MultipleLocator(200))
                ax_timing.yaxis.set_major_locator(MultipleLocator(200))
                ax_timing.xaxis.set_minor_locator(MultipleLocator(100))
                ax_timing.yaxis.set_minor_locator(MultipleLocator(100))
                ax_timing.minorticks_on()
                ax_timing.set_xlabel("Din Pulse Duration [ns]")
                ax_timing.set_ylabel("Delay & Output Pulse Duration [ns]")
                ax_timing.grid(True, which="major", alpha=0.25)
                ax_timing.grid(True, which="minor", alpha=0.16, linestyle=":")
                if reset_threshold_us is None:
                    inset_text = "Reset threshold: n/a"
                else:
                    inset_text = f"Reset threshold: {format_number(reset_threshold_us,2)} us"
                ax_timing.text(
                    0.97,
                    0.03,
                    inset_text,
                    transform=ax_timing.transAxes,
                    ha="right",
                    va="bottom",
                    fontsize=8,
                    bbox={"facecolor": "white", "edgecolor": "0.6", "alpha": 0.85, "boxstyle": "round,pad=0.2"},
                )
                ax_timing.set_title("Pulse Regeneration Din-Dout")
                ax_timing.legend(fontsize=8, loc="upper left")
            else:
                ax_timing.set_title("Pulse Regeneration Din-Dout")
                ax_timing.text(0.5, 0.5, "No timing data", ha="center", va="center", transform=ax_timing.transAxes)
                ax_timing.set_xticks([])
                ax_timing.set_yticks([])
        else:
            ax_timing.set_title("Pulse Regeneration Din-Dout")
            ax_timing.text(0.5, 0.5, "No txh data", ha="center", va="center", transform=ax_timing.transAxes)
            ax_timing.set_xticks([])
            ax_timing.set_yticks([])

        # 2) PWM Sweep (baseline-subtracted LED current)
        led_channels = payload.get("led_channels", {}) if payload else {}
        pwm_info = payload.get("pwm", {}) if payload else {}
        channel_order_raw = payload.get("channel_order") if payload else None
        channel_order = "".join(c for c in str(channel_order_raw).upper() if c in "RGB")
        channel_to_led: Dict[str, str] = {}
        if len(channel_order) >= 3:
            channel_to_led["CH1"] = channel_order[0]
            channel_to_led["CH2"] = channel_order[1]
            channel_to_led["CH3"] = channel_order[2]
        led_color_map = {"R": "#d62728", "G": "#2ca02c", "B": "#1f77b4"}

        if isinstance(led_channels, dict) and led_channels:
            for ch in sorted(led_channels):
                info = led_channels[ch]
                duty = info.get("duty", [])
                vals = info.get("values", [])
                baseline = info.get("baseline")
                if baseline is not None:
                    vals = [v - baseline for v in vals]

                led_name = channel_to_led.get(ch)
                color = led_color_map.get(led_name) if led_name else None
                if led_name:
                    label = f"{ch} ({led_name})"
                else:
                    label = ch
                ax_led.plot(duty, vals, marker="o", markersize=2.5, linewidth=1.2, color=color, label=label)

            pwm_hz = pwm_info.get("pwm_hz") if isinstance(pwm_info, dict) else None
            i_off_ma = pwm_info.get("i_off_ma") if isinstance(pwm_info, dict) else None

            linearity_state = "n/a"
            if led_channels:
                non_linear = 0
                eval_n = 0
                for ch in led_channels:
                    info = led_channels[ch]
                    r2 = info.get("linearity_r2")
                    exp = info.get("power_exp")
                    if r2 is None and exp is None:
                        continue
                    eval_n += 1
                    if (r2 is not None and r2 < 0.995) or (exp is not None and abs(exp - 1.0) > 0.08):
                        non_linear += 1
                if eval_n > 0:
                    linearity_state = "non-linear" if non_linear > 0 else "linear"

            inset_lines = [
                f"PWM frequency: {format_number(pwm_hz,1)} Hz",
                f"Static current: {format_number(i_off_ma,2)} mA",
                f"Sweep linearity: {linearity_state}",
            ]

            ax_led.set_title("PWM Sweep")
            ax_led.set_xlabel("PWM setting [0..255]")
            ax_led.set_ylabel("Average LED Current [mA]")
            ax_led.xaxis.set_minor_locator(AutoMinorLocator(2))
            ax_led.yaxis.set_minor_locator(AutoMinorLocator(2))
            ax_led.minorticks_on()
            ax_led.grid(True, which="major", alpha=0.25)
            ax_led.grid(True, which="minor", alpha=0.16, linestyle=":")
            ax_led.text(
                0.97,
                0.03,
                "\n".join(inset_lines),
                transform=ax_led.transAxes,
                ha="right",
                va="bottom",
                fontsize=8,
                bbox={"facecolor": "white", "edgecolor": "0.6", "alpha": 0.85, "boxstyle": "round,pad=0.2"},
            )
            ax_led.legend(fontsize=7, ncol=2, loc="upper left")
        else:
            ax_led.set_title("PWM Sweep")
            ax_led.text(0.5, 0.5, "No led_current data", ha="center", va="center", transform=ax_led.transAxes)
            ax_led.set_xticks([])
            ax_led.set_yticks([])

        self.plot_canvas.draw_idle()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze WS2812 test logs with a GUI.")
    parser.add_argument(
        "--logs",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "testlogs",
        help="Directory containing log files (default: ../testlogs)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = tk.Tk()
    app = AnalyzerApp(root, args.logs)
    app.path_label.configure(text=f"Source: {args.logs}")
    root.mainloop()


if __name__ == "__main__":
    main()

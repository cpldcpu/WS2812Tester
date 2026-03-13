#!/usr/bin/env python3
"""Build static site from WS2812 testlogs.

Parses all testlog files, runs analysis, and produces a _site/ directory
with data.json and static assets for GitHub Pages deployment.
"""

from __future__ import annotations

import json
import math
import re
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
ALLOWED_SUFFIXES = {".txt", ".log", ".md", ""}
IMAGE_SUFFIX_PRIORITY = (".png", ".jpg", ".jpeg")
CLOCK_HZ = 48_000_000.0
CLOCK_NS = 1e9 / CLOCK_HZ

# ---------------------------------------------------------------------------
# Data classes (from analyzer/app.py)
# ---------------------------------------------------------------------------

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

# ---------------------------------------------------------------------------
# Numeric helpers
# ---------------------------------------------------------------------------

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

# ---------------------------------------------------------------------------
# Curve fitting
# ---------------------------------------------------------------------------

def fit_linear_offset_scale(zs: List[float], ys: List[float]) -> Optional[Tuple[float, float]]:
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

    scan_c(0.20, 4.00, 0.05)
    if best_params is None:
        return None
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

# ---------------------------------------------------------------------------
# Log parsing
# ---------------------------------------------------------------------------

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
            continue

        if "\t" not in line:
            warnings.append(f"Ignored non-TSV line in section '{current.name}': {stripped[:40]}")
            continue

        cols = [c.strip() for c in line.split("\t")]
        ncols = len(cols)

        if ncols == 2:
            current.kv[cols[0]] = cols[1]
            active_table = None
            continue

        if active_table is None:
            active_table = Table(headers=cols)
            current.tables.append(active_table)
            continue

        if len(active_table.headers) == ncols:
            active_table.rows.append(dict(zip(active_table.headers, cols)))
        else:
            active_table = Table(headers=cols)
            current.tables.append(active_table)

    seen = set()
    deduped = []
    for w in warnings:
        if w not in seen:
            deduped.append(w)
            seen.add(w)

    return ParsedLog(path=path, sections=sections, warnings=deduped)

# ---------------------------------------------------------------------------
# Section helpers
# ---------------------------------------------------------------------------

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

# ---------------------------------------------------------------------------
# Plot data builders
# ---------------------------------------------------------------------------

def build_txh_plot_data(section: Optional[Section]) -> Dict[str, List]:
    out: Dict[str, List] = {
        "txh_ns": [], "prop_ns": [], "prop_std_ns": [],
        "dur_ns": [], "dur_std_ns": [],
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
        prop_std_ns = None
        dur_std_ns = None
        if pvar_x10 is not None and pvar_x10 >= 0:
            prop_std_ns = math.sqrt(pvar_x10 / 10.0) * CLOCK_NS
        if dvar_x10 is not None and dvar_x10 >= 0:
            dur_std_ns = math.sqrt(dvar_x10 / 10.0) * CLOCK_NS
        if prop_ns is None and dur_ns is None:
            continue
        out["txh_ns"].append(txh * CLOCK_NS)
        out["prop_ns"].append(prop_ns)
        out["prop_std_ns"].append(prop_std_ns)
        out["dur_ns"].append(dur_ns)
        out["dur_std_ns"].append(dur_std_ns)
    return out


def build_led_channel_analysis(section: Optional[Section], rsense_ohm: Optional[float]) -> Dict[str, Dict]:
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

    out: Dict[str, Dict] = {}
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
        baseline = baseline_mv
        if rsense_ohm is not None and rsense_ohm > 0:
            values = [v / rsense_ohm for v in vdrop_mv]
            baseline = baseline_mv / rsense_ohm

        xs_all = [d / 255.0 for d in duty]
        ys_all = list(values)

        fit_params = fit_power_law(xs_all, ys_all)
        fit_offset = power_scale = power_exp = None
        if fit_params is not None:
            fit_offset, power_scale, power_exp = fit_params

        fit_values: List[float] = []
        if fit_offset is not None and power_scale is not None and power_exp is not None:
            for d in duty:
                x = d / 255.0
                fit_values.append(fit_offset + power_scale * (x ** power_exp if x > 0 else 0.0))
        else:
            fit_values = [None for _ in duty]

        actual = values
        pred = [p for p in fit_values if p is not None]
        linearity_r2 = r2_score(actual, pred) if len(actual) == len(pred) else None

        on_est = None
        if fit_offset is not None and power_scale is not None:
            on_est = fit_offset + power_scale

        power_dev_pct = None
        if power_exp is not None:
            power_dev_pct = (power_exp - 1.0) * 100.0

        on_max = max(values) if values else None

        out[ch] = {
            "duty": duty,
            "values": values,
            "fit_values": fit_values,
            "baseline": baseline,
            "linearity_r2": linearity_r2,
            "power_exp": power_exp,
            "power_dev_pct": power_dev_pct,
            "on_est": on_est,
            "on_max": on_max,
        }

    return out

# ---------------------------------------------------------------------------
# JSON-safe conversion (NaN → None)
# ---------------------------------------------------------------------------

def sanitize_for_json(obj):
    """Replace NaN/Inf floats with None for JSON serialization."""
    if isinstance(obj, float):
        if math.isnan(obj) or math.isinf(obj):
            return None
        return obj
    if isinstance(obj, dict):
        return {k: sanitize_for_json(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [sanitize_for_json(x) for x in obj]
    return obj

# ---------------------------------------------------------------------------
# Per-device data extraction
# ---------------------------------------------------------------------------

def extract_device_data(parsed: ParsedLog, rsense_ohm: Optional[float]) -> Dict:
    """Extract all data for a single device into a JSON-serializable dict."""
    boot = find_section(parsed, "boot")
    device = find_section_startswith(parsed, "device")
    txh_section = find_section(parsed, "txh_sweep")
    reset_section = find_section(parsed, "reset_time")
    pwm_section = find_section(parsed, "pwm_rate")
    led_section = find_section(parsed, "led_current")
    tests_complete = find_section(parsed, "tests_complete")

    # Device info
    device_info = {}
    if device:
        for key in ("Manufacturer", "Type", "IC", "LCSC", "Channel Order"):
            if key in device.kv:
                device_info[key] = device.kv[key]

    channel_order = device_info.get("Channel Order", "GRB")

    # Reset time
    reset_data = {}
    if reset_section:
        reset_data["status"] = reset_section.kv.get("status", "unknown")
        thr_clocks = parse_number(reset_section.kv.get("threshold_clocks", ""))
        if thr_clocks is not None:
            reset_data["threshold_clocks"] = thr_clocks
            reset_data["threshold_us"] = thr_clocks / 48.0

    # PWM rate
    pwm_data = {}
    if pwm_section:
        pwm_data["status"] = pwm_section.kv.get("status", "unknown")
        pwm_hz = parse_number(pwm_section.kv.get("pwm_hz", ""))
        if pwm_hz is not None:
            pwm_data["pwm_hz"] = pwm_hz
        for k in ("vdrop_off_mv", "vdrop_on_mv", "vdrop_led_mv"):
            v = parse_number(pwm_section.kv.get(k, ""))
            if v is not None:
                pwm_data[k] = v
        pwm_data["i_off_ma"] = current_ma_from_vdrop_mv(
            parse_number(pwm_section.kv.get("vdrop_off_mv", "")), rsense_ohm)
        pwm_data["i_on_ma"] = current_ma_from_vdrop_mv(
            parse_number(pwm_section.kv.get("vdrop_on_mv", "")), rsense_ohm)
        pwm_data["i_led_delta_ma"] = current_ma_from_vdrop_mv(
            parse_number(pwm_section.kv.get("vdrop_led_mv", "")), rsense_ohm)
        if pwm_data["status"] != "ok":
            reason = pwm_section.kv.get("reason", "")
            if reason:
                pwm_data["reason"] = reason

    # txH sweep
    txh_data = {}
    if txh_section:
        txh_data["status"] = txh_section.kv.get("status", "unknown")
        if txh_data["status"] != "ok":
            reason = txh_section.kv.get("reason", "")
            if reason:
                txh_data["reason"] = reason
        txh_data["plot"] = build_txh_plot_data(txh_section)

        # Detect transition point
        table = first_table(txh_section)
        if table and len(table.rows) >= 10:
            numeric_rows = []
            for row in table.rows:
                txh = parse_number(row.get("txh_clocks", ""))
                dur = parse_number(row.get("dur_x10", ""))
                if txh is not None and dur is not None:
                    numeric_rows.append((int(txh), dur))
            if len(numeric_rows) >= 10:
                early = [dur for _, dur in numeric_rows[:10]]
                early_avg = sum(early) / len(early)
                for txh_val, dur in numeric_rows:
                    if dur > (early_avg * 1.5):
                        txh_data["transition_clocks"] = txh_val
                        txh_data["transition_ns"] = txh_val * CLOCK_NS
                        break

    # LED current
    led_data = {}
    if led_section:
        led_data["status"] = led_section.kv.get("status", "unknown")
        led_data["channels"] = build_led_channel_analysis(led_section, rsense_ohm)

    # Overall status
    overall_status = "unknown"
    if tests_complete:
        overall_status = tests_complete.kv.get("status", "unknown")

    return {
        "device_info": device_info,
        "channel_order": channel_order,
        "rsense_ohm": rsense_ohm,
        "reset": reset_data,
        "pwm": pwm_data,
        "txh": txh_data,
        "led": led_data,
        "overall_status": overall_status,
    }

# ---------------------------------------------------------------------------
# Image discovery
# ---------------------------------------------------------------------------

def find_device_image(log_path: Path) -> Optional[Path]:
    stem = log_path.stem
    for suffix in IMAGE_SUFFIX_PRIORITY:
        candidate = log_path.parent / (stem + suffix)
        if candidate.exists():
            return candidate
    return None

# ---------------------------------------------------------------------------
# Main build
# ---------------------------------------------------------------------------

def main() -> None:
    repo_root = Path(__file__).resolve().parent.parent
    testlogs_dir = repo_root / "testlogs"
    web_dir = repo_root / "web"
    site_dir = repo_root / "_site"
    images_dir = site_dir / "images"

    # Clean and create output
    if site_dir.exists():
        shutil.rmtree(site_dir)
    site_dir.mkdir(parents=True)
    images_dir.mkdir()

    # Discover log files
    log_files = sorted(
        [p for p in testlogs_dir.iterdir()
         if p.is_file() and p.suffix.lower() in ALLOWED_SUFFIXES and not p.name.startswith(".")],
        key=lambda p: p.name.lower(),
    )

    devices = []
    for log_path in log_files:
        print(f"Processing {log_path.name}...")
        parsed = parse_log_file(log_path)

        boot = find_section(parsed, "boot")
        rsense_ohm = parse_ohms(boot.kv.get("Rsense [Ohm]", "")) if boot else None

        data = extract_device_data(parsed, rsense_ohm)
        data["name"] = log_path.stem
        data["log_file"] = log_path.name

        # Copy image if available
        img_path = find_device_image(log_path)
        if img_path is not None:
            dest = images_dir / img_path.name
            shutil.copy2(img_path, dest)
            data["image"] = f"images/{img_path.name}"
        else:
            data["image"] = None

        devices.append(data)

    # Write data.json
    output = sanitize_for_json({
        "clock_hz": CLOCK_HZ,
        "clock_ns": CLOCK_NS,
        "devices": devices,
    })
    data_path = site_dir / "data.json"
    data_path.write_text(json.dumps(output, indent=2), encoding="utf-8")
    print(f"Wrote {data_path} ({len(devices)} devices)")

    # Copy static assets
    for asset in ("index.html", "style.css", "app.js"):
        src = web_dir / asset
        if src.exists():
            shutil.copy2(src, site_dir / asset)
            print(f"Copied {asset}")
        else:
            print(f"WARNING: {asset} not found in web/")

    print("Build complete.")


if __name__ == "__main__":
    main()

# WS2812 Log Analyzer

Python GUI tool for inspecting and visualizing WS2812 tester firmware logs.

## Features

- Scans log files in `../testlogs` (or a custom directory)
- Dropdown to select and switch between log files
- Measurement summary table with all extracted metrics
- Interpretation panel with automated findings and derived values
- Image tab: if a file with the same basename exists in the log folder (`name.png`, `name.jpg`, `name.jpeg`), it is displayed and auto-scaled to fit
- Embedded plots (requires matplotlib):
  - **txH Timing**: Propagation delay and pulse length vs input high-time, with standard deviation error bars. Fixed 0-1300 ns axes with a diagonal reference line for easy comparison across devices.
  - **LED Sweep**: Per-channel current (or voltage drop) vs PWM duty, with power-law curve fits overlaid.
- Converts clock counts to ns/us at 48 MHz
- Converts shunt-drop (mV) to current (mA) using the `Rsense [Ohm]` value from the log
- Per-channel LED on-current estimation and linearity analysis (R^2, power exponent)

## Requirements

- Python 3.7+
- `tkinter` (included with most Python installations)
- `matplotlib` (optional, for embedded plots)
- `Pillow` (optional, needed for `.jpg/.jpeg` display in the Image tab; `.png` works without it)

## Usage

```bash
python3 analyzer/app.py
```

With a custom log directory:

```bash
python3 analyzer/app.py --logs /path/to/logs
```

## Log Format

The parser expects section-based logs emitted by the firmware:

```text
## section_name
key	value
col1	col2	col3
data1	data2	data3
...
```

- Lines before the first `## ` header are ignored (e.g. minichlink startup text)
- 2-column tab-separated lines are parsed as key/value pairs
- 3+ column tab-separated lines are parsed as table rows (first occurrence becomes the header)
- Accepted file extensions: `.txt`, `.log`, `.md`, or no extension

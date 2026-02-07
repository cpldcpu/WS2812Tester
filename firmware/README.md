# WS2812 Tester Firmware

Firmware for characterizing WS2812 LED timing and electrical parameters using a CH32V003 (SOIC-8, RISC-V).

## Hardware

| Signal | Pin (SOIC-8) | Port | Function |
|--------|-------------|------|----------|
| DiN | Pin 1 | PA1 | T1CH2 PWM output to WS2812 |
| Dout | Pin 7 | PC4 | T1CH4 input capture from WS2812 |
| Sense | Pin 3 | PA2 | ADC channel 0 (current sense on WS2812 VCC) |

A shunt resistor (e.g. 47 Ohm) on the WS2812 VCC rail provides the Sense signal. The firmware reports shunt-drop values (`Vdrop = VCC - Vsense`) which the analyzer converts to current using the `Rsense [Ohm]` value from the boot section.

## Build

Requires `riscv64-unknown-elf-gcc` toolchain. The ch32v003fun SDK is included as a git submodule.

```bash
git submodule update --init     # fetch SDK if not already present
make                            # build
make flash                      # build and flash via WCH-LinkE
make clean                      # remove build artifacts
```

Debug output via single-wire debug interface:
```bash
ch32v003fun/minichlink/minichlink -T
```

## Timer Architecture

TIM1 runs at 48 MHz (PSC=0) with a 96-tick period (2.0 us). Three DMA channels operate simultaneously:

- **CH2** (DMA1_Channel3): PWM output on PA1 (DiN). DMA feeds duty-cycle values from a TX buffer, generating WS2812-compatible waveforms.
- **CH3** (DMA1_Channel6): Input capture from TI4, rising edge. Records timestamps of rising edges on PC4 (Dout).
- **CH4** (DMA1_Channel4): Input capture from TI4, falling edge. Records timestamps of falling edges on PC4 (Dout).

Cross-channel routing (`CC3S=10`) lets CH3 and CH4 both capture from the same physical pin (PC4), giving simultaneous rising and falling edge timestamps in a single pass.

## Tests

**txH Sweep** — Sweeps the high-time of transmitted bits from 1 to 64 clock cycles (~21 to 1333 ns) and measures propagation delay and pulse duration on Dout. Each txH value is tested at 3 HSI trim settings (8, 16, 24) with 160 samples each (480 total), reporting average x10 and variance x10 in clock-cycle units.

**Reset Time** — Finds the minimum idle gap that triggers a WS2812 reset. Linear scan with majority-vote classification (7 probes per gap length) over gap durations from 10 to 500 us. Reports the threshold in timer clock cycles.

**PWM Rate** — Sends a WS2812 frame with 12.5% red duty (`R=32, G=0, B=0`), then captures the Sense waveform via continuous ADC+DMA. Uses a two-pass approach: first a fast pass (32:1 decimation, 501-20000 Hz detection range), then if no valid peak is found, a slow fallback pass (128:1 decimation, 250-2000 Hz range). Each pass decimates into 512 points and analyzes with autocorrelation + parabolic interpolation. VCC is calibrated from internal Vrefint (1200 mV nominal). Reports PWM frequency, shunt-drop levels (off/on/delta), effective sample rate, and decimation factor used.

**LED Current** — Sweeps each color channel (CH1/CH2/CH3, mapped to G/R/B) across duty values 0, 2, 4, ..., 62 then 64, 96, 128, 160, 192, 224, 255. For each point, applies the color setting and continuously samples Sense for 100 ms, reporting the average shunt-drop in mV.

## Log Format

Firmware output is structured text over the single-wire debug interface:

1. Section headers: `## section_name`
2. Tab-separated tables with a header row (for repeated measurements)
3. Tab-separated `key<TAB>value` lines (for single values)
4. A blank line between sections (not before the first)

```text
## boot
fw	ws2812_tester
mcu	ch32v003
Rsense [Ohm]	47.4

## Device (update from datasheet)
Manufacturer	Worldsemi
Type	WS2812D
IC	WS2812B
LCSC	C190565
Channel Order	GRB

## txh_sweep
txh_clocks	prop_x10	p_var_x10	dur_x10	d_var_x10
1	93	9	140	16
2	93	8	141	12
...
status	ok

## reset_time
threshold_clocks	5856
status	ok

## pwm_rate
vref_mv	4981
vrefint_cnt	246
vcal_cnt	511
pwm_hz	2032
vdrop_off_mv	53
vdrop_on_mv	843
vdrop_led_mv	790
fs_eff_hz	13386
fs_raw_hz	1713408
decim_factor	128
runs	2
status	ok

## led_current
vref_mv	4747
channel	duty	vdrop_avg_mv
CH1	0	46
CH1	2	46
...
CH3	255	849
status	ok

## tests_complete
status	ok
```

The `Device` section is a placeholder filled in manually from the LED datasheet before each test run. The `Channel Order` field records whether the device uses GRB (WS2812 standard) or RGB ordering.

# WS2812 tester based on CH32V003

The objective of this project is to implement a WS2812 tester based on an CH32V003. 

The CH32V003 connects to the DiN and Dout of the WS2812 to measure timing parameters using the CH32V003 timer with PWM and capture mode. In this setup the 8-pin version of the CH32V003 is used.

## Connections

DiN is connected to PA1/T1CH2 (pin 1 on SOIC8)
Dout is connected to PC4/T1CH4 (pin 7 on SOIC8)
Sense is connected to PA2/A0 (pin 3 on SOIC8)

Sense is an analog input used to measure the current on the WS2812 VCC line

## Test

A number of different tests are being implemented.

1) Reset time test: Measures how long the data in line must be left idle to reset the WS2812 state machine.

2) txH sweep: Measures the timing of 0 and 1 bits by sweeping the txH time and measuring the response on Dout.

3) PWM rate measurement: Sends WS2812 with fixed 12.5% red duty (`R=32, G=0, B=0`) and measures the sense waveform with ADC DMA on PA2/A0. The algorithm decimates samples and uses autocorrelation to extract the fundamental PWM frequency (target range 400 Hz to 20 kHz). VCC is calibrated from internal Vrefint (BG) and reported in shunt-drop form: `Vdrop = VCC - Vsense`, including min, max and ripple (p-p). RAM is shared with the timing tests via a common workspace buffer.

4) LED current measurement: Measures the average current of the logical channels CH1,CH2,CH3 separately at duty cycles `0..31` in steps of 1, then `32,64,96,128,160,192,224,255`. To ensure this works properly, continuously samples from sense while the LED is on for at least 100 ms, then average all samples to get a stable reading. Do this for all channels separately.

## Log format

Firmware output is structured for both human and script use:

1) Section headers: `## section_name`  
2) Tab-separated tables with headers for repeated measurements  
3) Tab-separated `key<TAB>value` lines for single-value reports
4) A blank line between sections (not before the first section)

Examples:
- `## txh_sweep` followed by table header `txh_clocks	prop_x10	p_var_x10	dur_x10	d_var_x10`
- `## reset_time` with lines such as `threshold_clocks	6048` and `status	ok`
- `## pwm_rate` with lines such as `pwm_hz	2035` and `vdrop_led_mv	73`
- `## led_current` with table header `channel	duty	vdrop_avg_mv`










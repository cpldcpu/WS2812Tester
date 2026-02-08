# High-Level Findings

A quick summary of findings after testing various ARGB LEDs using the [WS2812 tester](https://github.com/cpldcpu/WS2812Tester).

## WS2812 Protocol Recap

<p align="center">
<img width="50%" alt="grafik" src="https://github.com/user-attachments/assets/99241b20-8cff-4f79-b575-b2737e4cc463" />
</p>

In the WS2812 protocol, a short pulse represents a '0' bit, a long pulse represents a '1' bit. (see [in-depth investigation](https://cpldcpu.github.io/2014/01/14/light_ws2812-library-v2-0-part-i-understanding-the-ws2812/) and also my low [level re-implementation](https://cpldcpu.github.io/2021/02/27/the-transistorpixel/)). 

When the bus was idle for a certain time, the LED will self-reset. After reset, it will consume the first 24 bits and write them to the internal pulse-width-module (PWM) engine settings. After the first 24 bits have passed, any additional bits will be forwarded to the data out pin, on to the next LED in the chain. In a proper implementation, the pulse is re-shaped and re-times to prevent degradation along the chain. I found that this behavior varies significantly between different implementations of the data tranceiver.

### Worldsemi WS2812B (Original from 2014)
<p align="center">
<img src="ws2812B_original.png" />
</p>

To characterize the data interface, we enter pulses of different length and observe how long it takes until they appear on the output (propagation delay) and how long they are. The diagramm on the left side summarizes characterization results of the data interface. For the original WS2812, we can observe that pulses appear on the output after a little more than 200ns. This behavior is independent of input pulse length. Depending on the input pulse length, it is retimes into either a '0' of ~400ns or a '1' of ~800ns. There is a very sharp threshold between zero and one, when the input pulse duration transtions to beyond 600ns.

This looks like excellent and stable behavior and ensures that the pulses do not degenerate along the daisy chain.

In addition, the minimum duration of the reset is measured. For the original WS2812B, this is 10µs, which is rather short and causes issues when the data transmission is intererrupted.

A second subsytem of interest is th PWM engine. Measurement results are shown on the right side. This test is measureing the current consumption of the LED with a series resistor in the power supply and the integration ADC of the microcontroller. 

We can see that the PWM frequency of the original WS2812B is 432Hz, which often results in perceivable flicker when the LEDs are moving. Furthermore, there is a curious nonlinear relationshop between the PWM channel setting and the actual
LED current. I investigated this [in more detail here](https://cpldcpu.github.io/2022/08/15/does-the-ws2812-have-integrated-gamma-correction/)).

### WS2812B-V5
<p align="center">
<img src="ws2812B_v5.png"  /> 
<img src="../testlogs/WS2812B-B.png" width="50%" /> 
</p>

This variant is called "V5" by Worldsemi and appears to be an optimized, smaller version of the original die.

 - Data signal regeneration still works in the same way.
 - Reset timing is now increased to >200 µs for better system robustness.
 - Nonlinear PWM behavior remains, but PWM frequency has been increased to ~2 kHz to reduce flicker.
 - Power consumption is slightly reduced.
 - Oscilloscope measurements show significantly reduced emissions on the power lines for this variant.

## SK6812

![SK6812 analysis](SK6812.png) [Test Data](../testlogs/SK6812Gen1.txt)

This is a very common WS2812 clone. There are at least three generations of this controller IC, which were subsequently reduced in die size.

 - Linear PWM behavior.
 - Some differences in data regeneration.

The data regeneration works differently from the WS2812. There is a region for input pulses between 400–500 ns where signal regeneration is not stable. However, since the output timing of "LO" pulses never increases in duration, the system is ultimately still stable.

The photo below shows the DY-S505016, which appears to have one of the newest generations of the SK6812 controller, measuring 0.37 × 0.48 mm (≈ 0.18 mm²).

<p align="center">
<img src="../testlogs/DY-S505016.png" width="50%" /> 
</p> 


## TX1812 variants

![TX1812CXA analysis](TX1281CXA.png)

This controller appeared much later than the two above but seems to be widely used by smaller Chinese LED packagers such as TCWIN, TONYU, and others.

It behaves very similarly to the SK6812 but is noticeably noisier electrically. I believe it is based on a fully clocked digital transceiver; the WS2812 and, to some extent, the SK6812 use self-timed state machines.

It is notable for its extremely small size (0.350 × 0.370 mm ≈ 0.13 mm²) and its characteristic bond pad pattern.

<p align="center">
<img src="../testlogs/TX1812CXA.png" width="70%" /> 
</p> 

### SK6112 (Using AP6112 controller)
![SK6112 analysis](SK6112.png) 

This LED comes with a Gen2 ARGB controller, which I analyzed in detail [here](https://github.com/cpldcpu/Gen2-Addressable-RGB/blob/main/docs/Gen2_ARGB_protocol_analysis.md).

The PWM behavior is unremarkable. However, the transceiver exhibits several odd behaviors. There is only a very narrow region where it correctly regenerates the Lo and Hi pulses; in other regions it simply forwards the pulse from the input.

In addition, there is a region of indeterminate behavior between the Lo and Hi timings. Based on detailed oscilloscope investigations (some shown below), I believe this is caused by a combination of combinatorial and asynchronously clocked logic.

In any case, this does not look like a robust transceiver and is likely prone to glitches and EMI emissions due to oscillations.

![SK6112 analysis](SK6112_scope.png) 

<p align="center">
<img src="../testlogs/SK6112.png" width="50%" /> 
</p> 

## Lite-On LTST-G563

![LTST-G563 analysis](LTST-G563.png) 

This controller is found in an ARGB LED from Lite-On. It does not seem to be widely used, and the LEDs are comparatively more expensive without offering clear differentiation.

The transceiver does not appear to regenerate the signal; it forwards it to the output with some delay.

This is a poor design choice, as it is prone to EMI and stability issues. Considering that these LEDs appeared years after the original WS2812, it is puzzling how this design was adopted.

<p align="center">
<img src="../testlogs/LTST-G563.png" width="50%" /> 
</p> 

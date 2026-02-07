/*
 * WS2812 Tester — CH32V003 (SOIC-8)
 *
 * Characterizes WS2812 timing parameters using TIM1:
 *   - DMA-driven PWM output on CH2 (PA1 → WS2812 DiN)
 *   - Dual DMA input capture on CH3/CH4 (PC4 ← WS2812 Dout)
 *   - ADC current sense on PA2/A0 (WS2812 VCC)
 *
 * Pin mapping (SOIC-8, TIM1 default remap):
 *   PA1 (pin 1) = T1CH2  — data output to WS2812
 *   PC4 (pin 7) = T1CH4  — data input from WS2812
 *   PA2 (pin 3) = A0     — current sense (ADC)
 */

#include "ch32fun.h"
#include <limits.h>
#include <stdio.h>

#define LOG_LINE(fmt, ...) printf(fmt "\r\n", ##__VA_ARGS__)
#define LOG_KV_STR(key, value) LOG_LINE("%s\t%s", key, value)
#define LOG_KV_INT(key, value) LOG_LINE("%s\t%d", key, (int)(value))
#define LOG_KV_U32(key, value) LOG_LINE("%s\t%lu", key, (unsigned long)(value))

static void log_section(const char *name)
{
	static int first_section = 1;
	if (!first_section)
		printf("\r\n");
	first_section = 0;
	printf("## %s\r\n", name);
}

#define LOG_SECTION(name) log_section(name)

/* Timer / WS2812 timing at 48 MHz, PSC=0 */
#define WS2812_PERIOD    96   /* 96 ticks = 2.0 µs bit period */
#define WS2812_T0H       17   /* ~354 ns — standard "0" high time */
#define WS2812_T1H       34   /* ~708 ns — standard "1" high time */
#define RESET_ENTRIES    150   /* 150 × 2 µs = 300 µs reset pulse */

#define NUM_TEST_BITS    160   /* n=160 → avg_x10 = sum/16 (shift) */
#define TX_BUF_SIZE      512   /* max: 150+24+24+250gap+24+1 = 473 (reset test) */
#define RX_BUF_SIZE      160
#define TX_DMA_WAIT_SPIN_MAX 48000000UL
#define RESET_GAP_MIN_PERIODS 5
#define RESET_GAP_MAX_PERIODS 250
#define RESET_PROBE_REPEATS 7

/* PWM-rate test parameters */
#define PWM_FREQ_MIN_HZ          501
#define PWM_FREQ_MAX_HZ          20000
#define PWM_RUNTIME_BUDGET_MS    100
#define PWM_DECIM_FACTOR_FAST    32
#define PWM_DECIM_FACTOR_SLOW    128
#define PWM_FREQ_MIN_HZ_SLOW     250
#define PWM_FREQ_MAX_HZ_SLOW     2000
#define PWM_RUNTIME_BUDGET_MS_SLOW 160
#define PWM_DECIM_POINTS         512
#define PWM_DMA_CHUNK_SAMPLES    32
#define PWM_MIN_VALID_RUNS       2
#define PWM_MAX_RUNS             8
#define PWM_DMA_TIMEOUT_MS       30
#define PWM_QUALITY_MIN_PERMILLE 60
#define VREFINT_NOMINAL_MV       1200
#define WS2812_PWM_RED_VALUE     32
#define WS2812_PWM_GREEN_VALUE   0
#define WS2812_PWM_BLUE_VALUE    0
#define VREF_MIN_MV              2000
#define VREF_MAX_MV              5500
#define ADC_FULL_SCALE_COUNTS    1023
#define LED_MEAS_DURATION_MS     100

#ifndef PWM_CAPTURE_TRACE_DEBUG
#define PWM_CAPTURE_TRACE_DEBUG 0
#endif

/* Buffers */
typedef union
{
	struct
	{
		uint8_t tx_buf[TX_BUF_SIZE];
		uint16_t rx_rising[RX_BUF_SIZE];
		uint16_t rx_falling[RX_BUF_SIZE];
	} timing;
	struct
	{
		uint16_t adc_dma_chunk[PWM_DMA_CHUNK_SAMPLES];
		int16_t decim[PWM_DECIM_POINTS];
	} pwm;
} shared_workspace_t;

static shared_workspace_t ws;

#define tx_buf ws.timing.tx_buf
#define rx_rising ws.timing.rx_rising
#define rx_falling ws.timing.rx_falling
#define adc_dma_chunk ws.pwm.adc_dma_chunk
#define pwm_decim ws.pwm.decim

static int rx_rising_count;
static int rx_falling_count;

/* ---------- TX buffer helpers ---------- */

static int fill_tx_zeros(int offset, int n)
{
	for (int i = 0; i < n && offset < TX_BUF_SIZE; i++)
		tx_buf[offset++] = 0;
	return offset;
}

static int fill_tx_bits(int offset, int n, uint16_t txH)
{
	for (int i = 0; i < n && offset < TX_BUF_SIZE; i++)
		tx_buf[offset++] = (uint8_t)txH;
	return offset;
}

/* ---------- Hardware init ---------- */

static void init_adc(void)
{
	/* ADC clock = PCLK2 / 2 (clear prescaler bits) */
	RCC->CFGR0 &= ~(0x1F << 11);

	/* Enable ADC1 clock */
	RCC->APB2PCENR |= RCC_APB2Periph_ADC1;

	/* Reset ADC1 */
	RCC->APB2PRSTR |=  RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

	/* Single conversion on channel 0 (PA2) */
	ADC1->RSQR1 = 0;
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = 0;  /* channel 0 */

	/* Max sample time for channel 0 (241 cycles) */
	ADC1->SAMPTR2 = (ADC1->SAMPTR2 & ~ADC_SMP0) | (7 << (3*0));

	/* Turn on ADC, software trigger for regular group */
	ADC1->CTLR2 |= ADC_ADON | ADC_EXTSEL;

	/* Calibrate */
	ADC1->CTLR2 |= ADC_RSTCAL;
	while (ADC1->CTLR2 & ADC_RSTCAL);
	ADC1->CTLR2 |= ADC_CAL;
	while (ADC1->CTLR2 & ADC_CAL);
}

static uint16_t adc_read(void)
{
	ADC1->CTLR2 |= ADC_SWSTART;
	while (!(ADC1->STATR & ADC_EOC));
	return ADC1->RDATAR;
}

static void init_timer(void)
{
	/* Enable peripheral clocks */
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC |
	                   RCC_APB2Periph_TIM1   | RCC_APB2Periph_AFIO;
	RCC->AHBPCENR  |= RCC_AHBPeriph_DMA1;

	/* PA1: T1CH2 alternate-function push-pull (mode 9 = AF_PP 10MHz) */
	GPIOA->CFGLR = (GPIOA->CFGLR & ~(0xf << (4*1))) | (GPIO_CFGLR_OUT_10Mhz_AF_PP << (4*1));

	/* PC4: input with pull-down (mode 8 = IN_PUPD, ODR=0 selects pull-down) */
	GPIOC->CFGLR = (GPIOC->CFGLR & ~(0xf << (4*4))) | (GPIO_CFGLR_IN_PUPD << (4*4));
	GPIOC->BCR = (1 << 4);

	/* PA2: analog input (mode 0) for ADC */
	GPIOA->CFGLR &= ~(0xf << (4*2));

	/* Reset TIM1 */
	RCC->APB2PRSTR |=  RCC_APB2Periph_TIM1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

	/* Timer base: 48 MHz / 1 = 48 MHz tick, period = 2.0 µs */
	TIM1->PSC   = 0;
	TIM1->ATRLR = WS2812_PERIOD - 1;

	/*
	 * CHCTLR1 (bits 15:8 = CH2):
	 *   OC2M  = 110  (PWM mode 1)
	 *   OC2PE = 1    (preload enable — new duty latched on update)
	 */
	TIM1->CHCTLR1 = TIM_OC2M_2 | TIM_OC2M_1 | TIM_OC2PE;

	/*
	 * CHCTLR2 (bits 7:0 = CH3, bits 15:8 = CH4):
	 *   CC3S = 10  (CH3 captures from TI4 — same pin as CH4)
	 *   CC4S = 01  (CH4 captures from TI4 directly)
	 */
	TIM1->CHCTLR2 = TIM_CC3S_1 | TIM_CC4S_0;

	/*
	 * CCER:
	 *   CC2E  = 1  (enable CH2 PWM output on PA1)
	 *   CC3E  = 1  (enable CH3 capture, rising edge — CC3P=0)
	 *   CC4E  = 1  (enable CH4 capture, falling edge — CC4P=1)
	 *   CC2NE = 0  (MUST stay 0 — PA2 is T1CH2N, needed for ADC)
	 */
	TIM1->CCER = TIM_CC2E | TIM_CC3E | TIM_CC4E | TIM_CC4P;

	TIM1->CH2CVR = 0;

	/* Enable main output (required for advanced timer) */
	TIM1->BDTR = TIM_MOE;

	/* Initialize counter and latch preload values */
	TIM1->SWEVGR = TIM_UG;
}

/* ---------- Run a TX/RX cycle ---------- */

static int run_tx_rx(int num_tx, int num_rx)
{
	if (num_tx <= 0 || num_tx > TX_BUF_SIZE)
		return 0;

	/* Clamp RX to buffer size */
	if (num_rx > RX_BUF_SIZE)
		num_rx = RX_BUF_SIZE;

	/* Stop timer, disable DMA requests */
	TIM1->CTLR1    &= ~TIM_CEN;
	TIM1->DMAINTENR = 0;

	/* --- TX: DMA1_Channel3 (TIM1_CH2, CC2DE) --- */
	DMA1_Channel3->CFGR  = 0;
	DMA1_Channel3->PADDR = (uint32_t)&TIM1->CH2CVR;
	DMA1_Channel3->MADDR = (uint32_t)&tx_buf[1];
	DMA1_Channel3->CNTR  = (num_tx > 1) ? (num_tx - 1) : 0;
	DMA1_Channel3->CFGR  =
		DMA_CFGR1_DIR     |  /* Memory → Peripheral */
		DMA_CFGR1_PSIZE_0 |  /* 16-bit peripheral */
		DMA_CFGR1_MINC    |  /* Increment memory address */
		DMA_CFGR1_EN;

	/* --- RX rising: DMA1_Channel6 (TIM1_CH3 maps to Ch6, CC3DE) --- */
	DMA1_Channel6->CFGR  = 0;
	DMA1_Channel6->PADDR = (uint32_t)&TIM1->CH3CVR;
	DMA1_Channel6->MADDR = (uint32_t)rx_rising;
	DMA1_Channel6->CNTR  = num_rx;
	DMA1_Channel6->CFGR  =
		DMA_CFGR1_MSIZE_0 |  /* 16-bit memory */
		DMA_CFGR1_PSIZE_0 |  /* 16-bit peripheral */
		DMA_CFGR1_MINC    |  /* Increment memory address */
		DMA_CFGR1_EN;

	/* --- RX falling: DMA1_Channel4 (TIM1_CH4, CC4DE) --- */
	DMA1_Channel4->CFGR  = 0;
	DMA1_Channel4->PADDR = (uint32_t)&TIM1->CH4CVR;
	DMA1_Channel4->MADDR = (uint32_t)rx_falling;
	DMA1_Channel4->CNTR  = num_rx;
	DMA1_Channel4->CFGR  =
		DMA_CFGR1_MSIZE_0 |  /* 16-bit memory */
		DMA_CFGR1_PSIZE_0 |  /* 16-bit peripheral */
		DMA_CFGR1_MINC    |  /* Increment memory address */
		DMA_CFGR1_EN;

	/* Pre-load first TX value and latch into active register */
	TIM1->CH2CVR  = tx_buf[0];
	TIM1->SWEVGR  = TIM_UG;
	TIM1->INTFR   = 0;  /* Clear flags generated by UG */

	/* Enable DMA requests */
	TIM1->DMAINTENR = TIM_CC2DE | TIM_CC3DE | TIM_CC4DE;

	/* Start timer */
	TIM1->CTLR1 |= TIM_CEN;

	/* Wait for TX DMA to complete */
	uint32_t spins = TX_DMA_WAIT_SPIN_MAX;
	while (DMA1_Channel3->CNTR > 0 && spins > 0)
		spins--;

	if (spins == 0)
	{
		/* Stop everything if transfer hangs */
		TIM1->CTLR1    &= ~TIM_CEN;
		TIM1->DMAINTENR = 0;
		DMA1_Channel3->CFGR = 0;
		DMA1_Channel4->CFGR = 0;
		DMA1_Channel6->CFGR = 0;
		rx_rising_count = 0;
		rx_falling_count = 0;
		return 0;
	}

	/* Wait for final Dout edges to arrive */
	Delay_Us(10);

	/* Save capture counts before stopping */
	rx_rising_count  = num_rx - (int)DMA1_Channel6->CNTR;
	rx_falling_count = num_rx - (int)DMA1_Channel4->CNTR;

	/* Stop everything */
	TIM1->CTLR1    &= ~TIM_CEN;
	TIM1->DMAINTENR = 0;
	DMA1_Channel3->CFGR = 0;
	DMA1_Channel4->CFGR = 0;
	DMA1_Channel6->CFGR = 0;
	return 1;
}

/* ---------- Test 2: txH sweep ---------- */

static const uint8_t hsitrim_vals[] = { 8, 16, 24 };
#define NUM_TRIMS (sizeof(hsitrim_vals) / sizeof(hsitrim_vals[0]))

static void test_txh_sweep(void)
{
	LOG_SECTION("txh_sweep");
	LOG_LINE("txh_clocks\tprop_x10\tp_var_x10\tdur_x10\td_var_x10");

	uint32_t rcc_save = RCC->CTLR;
	int saw_timeout = 0;

	for (uint16_t txH = 1; txH <= 64; txH++)
	{
		/* Build TX buffer once (doesn't depend on trim) */
		int offset = 0;
		offset = fill_tx_zeros(offset, RESET_ENTRIES);
		offset = fill_tx_bits(offset, 24, WS2812_T0H);
		offset = fill_tx_bits(offset, NUM_TEST_BITS, txH);
		tx_buf[offset++] = 0;

		int32_t prop_sum_all = 0, dur_sum_all = 0;
		int32_t var_p_sum = 0, var_d_sum = 0;
		int total = 0;
		int sweep_ok = 1;

		for (int t = 0; t < (int)NUM_TRIMS; t++)
		{
			/* Shift HSI frequency — settles during 300 µs reset */
			RCC->CTLR = (rcc_save & ~(0x1F << 3)) |
			            ((uint32_t)hsitrim_vals[t] << 3);

			if (!run_tx_rx(offset, NUM_TEST_BITS))
			{
				sweep_ok = 0;
				break;
			}

			/* Per-trim accumulators for variance */
			int32_t ps = 0, ds = 0, psq = 0, dsq = 0;

			for (int i = 0; i < rx_rising_count; i++)
			{
				int16_t p = (int16_t)rx_rising[i];
				int16_t d = (int16_t)(rx_falling[i] - rx_rising[i]);
				if (d < 0)
					d += WS2812_PERIOD;
				prop_sum_all += p;
				dur_sum_all  += d;
				ps  += p;
				ds  += d;
				psq += (int32_t)p * p;
				dsq += (int32_t)d * d;
			}
			total += rx_rising_count;

			/* Within-trim var_x10 = sq/16 - (sum/16)²/10 */
			if (rx_rising_count == NUM_TEST_BITS)
			{
				int32_t ap_t = ps >> 4;
				int32_t ad_t = ds >> 4;
				int32_t vp = (psq >> 4) - ap_t * ap_t / 10;
				int32_t vd = (dsq >> 4) - ad_t * ad_t / 10;
				var_p_sum += (vp > 0) ? vp : 0;
				var_d_sum += (vd > 0) ? vd : 0;
			}
		}

		/* Restore default trim before printf */
		RCC->CTLR = rcc_save;

		/* avg_x10 from 480 samples = sum/48.
		 * var_x10 = average of per-trim variances / 3 */
		if (sweep_ok && total == (int)(NUM_TRIMS * NUM_TEST_BITS))
		{
			int32_t ap = prop_sum_all / 48;
			int32_t ad = dur_sum_all  / 48;
			int32_t vp = var_p_sum / (int)NUM_TRIMS;
			int32_t vd = var_d_sum / (int)NUM_TRIMS;

			LOG_LINE("%d\t%ld\t%ld\t%ld\t%ld",
			         (int)txH, (long)ap, (long)vp, (long)ad, (long)vd);
		}
		else
		{
			saw_timeout = 1;
			LOG_LINE("%d\tna\tna\tna\tna", (int)txH);
		}
	}

	if (saw_timeout)
	{
		LOG_KV_STR("status", "fail");
		LOG_KV_STR("reason", "dma_timeout");
	}
	else
	{
		LOG_KV_STR("status", "ok");
	}
}

/* ---------- Test 1: Reset time ---------- */

static int run_reset_probe(int gap_periods)
{
	int offset = 0;

	/* Reset pulse */
	offset = fill_tx_zeros(offset, RESET_ENTRIES);

	/* Init LED dark: 24× "0" */
	offset = fill_tx_bits(offset, 24, WS2812_T0H);

	/* First data packet: 24× "1" (passes through to Dout) */
	offset = fill_tx_bits(offset, 24, WS2812_T1H);

	/* Variable gap (each entry = one timer period = 96 clocks) */
	offset = fill_tx_zeros(offset, gap_periods);

	/* Second data packet: 24× "1" */
	offset = fill_tx_bits(offset, 24, WS2812_T1H);

	/* End */
	tx_buf[offset++] = 0;

	if (!run_tx_rx(offset, 48))
		return -1;

	return (rx_rising_count <= 24) ? 1 : 0;
}

static int classify_reset_gap(int gap_periods, int *hits_out)
{
	int hits = 0;

	for (int i = 0; i < RESET_PROBE_REPEATS; i++)
	{
		int probe = run_reset_probe(gap_periods);
		if (probe < 0)
			return -1;
		hits += probe;
	}

	if (hits_out)
		*hits_out = hits;

	/* Majority vote: robust against jitter near threshold */
	return (hits > (RESET_PROBE_REPEATS / 2)) ? 1 : 0;
}

static int find_reset_threshold_linear(void)
{
	for (int gap_periods = RESET_GAP_MIN_PERIODS;
	     gap_periods <= RESET_GAP_MAX_PERIODS; gap_periods++)
	{
		int cls = classify_reset_gap(gap_periods, 0);
		if (cls < 0)
			return -2;
		if (cls)
			return gap_periods;
	}

	return -1;
}

static void test_reset_time(void)
{
	LOG_SECTION("reset_time");

	/* Linear search for first reset-true gap using majority classification. */
	int threshold = find_reset_threshold_linear();

	if (threshold >= 0)
	{
		LOG_KV_INT("threshold_clocks", threshold * WS2812_PERIOD);
		LOG_KV_STR("status", "ok");
	}
	else if (threshold == -1)
	{
		LOG_KV_STR("threshold_clocks", "na");
		LOG_KV_INT("max_clocks", RESET_GAP_MAX_PERIODS * WS2812_PERIOD);
		LOG_KV_STR("status", "ok");
	}
	else
	{
		LOG_KV_STR("status", "fail");
		LOG_KV_STR("reason", "dma_timeout");
	}
}

/* ---------- Test 3: PWM rate measurement ---------- */

static int append_ws2812_byte(int offset, uint8_t value)
{
	for (int bit = 7; bit >= 0 && offset < TX_BUF_SIZE; bit--)
		tx_buf[offset++] = (value & (1 << bit)) ? WS2812_T1H : WS2812_T0H;
	return offset;
}

static int send_ws2812_frame(uint8_t g, uint8_t r, uint8_t b)
{
	int offset = 0;
	offset = fill_tx_zeros(offset, RESET_ENTRIES);
	offset = append_ws2812_byte(offset, g); /* G */
	offset = append_ws2812_byte(offset, r); /* R */
	offset = append_ws2812_byte(offset, b); /* B */
	tx_buf[offset++] = 0;
	if (!run_tx_rx(offset, 0))
		return 0;

	/* WS2812 needs >50 us low after data to latch a new color frame. */
	Delay_Us(80);
	return 1;
}

static void stop_adc_dma_capture(void)
{
	ADC1->CTLR2 &= ~(ADC_CONT | ADC_DMA | ADC_EXTTRIG);
	DMA1_Channel1->CFGR = 0;
	DMA1->INTFCR = DMA_CGIF1 | DMA_CTCIF1 | DMA_CHTIF1 | DMA_CTEIF1;
}

static uint32_t read_internal_channel_sum(uint8_t channel, int samples)
{
	uint32_t rsqr1_save = ADC1->RSQR1;
	uint32_t rsqr2_save = ADC1->RSQR2;
	uint32_t rsqr3_save = ADC1->RSQR3;
	uint32_t samptr2_save = ADC1->SAMPTR2;
	uint32_t ctlr1_save = ADC1->CTLR1;
	uint32_t sum = 0;

	stop_adc_dma_capture();
	ADC1->CTLR1 &= ~ADC_SCAN;
	ADC1->CTLR2 |= ADC_TSVREFE;

	ADC1->RSQR1 = 0;
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = channel;

	/* Use long sample time for stable Vrefint measurement. */
	ADC1->SAMPTR2 = (ADC1->SAMPTR2 & ~(7U << (3 * channel))) |
	                (7U << (3 * channel));
	Delay_Us(20);

	for (int i = 0; i < samples; i++)
		sum += adc_read();

	ADC1->CTLR2 &= ~ADC_TSVREFE;
	ADC1->CTLR1 = ctlr1_save;
	ADC1->RSQR1 = rsqr1_save;
	ADC1->RSQR2 = rsqr2_save;
	ADC1->RSQR3 = rsqr3_save;
	ADC1->SAMPTR2 = samptr2_save;
	return sum;
}

static int vref_mv_from_sum(uint32_t sum, int samples)
{
	if (sum == 0 || samples <= 0)
		return -1;

	uint64_t num = (uint64_t)VREFINT_NOMINAL_MV * ADC_FULL_SCALE_COUNTS * (uint64_t)samples;
	return (int)((num + (sum / 2)) / sum);
}

static int measure_vref_mv(int *vcal_counts_out, int *vrefint_counts_out)
{
	/*
	 * CH32V003 provides two internal channels:
	 *  - ADC_Channel_Vcalint (programmable fraction of AVDD)
	 *  - ADC_Channel_Vrefint
	 * Use Vrefint for absolute Vref calibration.
	 */
	const int samples = 64;
	uint32_t cfgr0_save = RCC->CFGR0;

	/* Use slower ADC clock for internal reference channels for better accuracy. */
	RCC->CFGR0 = (RCC->CFGR0 & ~RCC_ADCPRE) | RCC_ADCPRE_DIV8;

	uint32_t sum_ref = read_internal_channel_sum(ADC_Channel_Vrefint, samples);
	int vref_mv = vref_mv_from_sum(sum_ref, samples);
	if (vrefint_counts_out)
		*vrefint_counts_out = (int)((sum_ref + (samples / 2)) / samples);

	uint32_t sum_cal = read_internal_channel_sum(ADC_Channel_Vcalint, samples);
	if (vcal_counts_out)
		*vcal_counts_out = (int)((sum_cal + (samples / 2)) / samples);

	if (vref_mv >= VREF_MIN_MV && vref_mv <= VREF_MAX_MV)
	{
		RCC->CFGR0 = cfgr0_save;
		return vref_mv;
	}

	RCC->CFGR0 = cfgr0_save;
	return -1;
}

static int capture_vdrop_average_mv(int vref_mv, uint32_t duration_ms, int *vdrop_avg_mv_out)
{
	stop_adc_dma_capture();

	ADC1->CTLR1 &= ~ADC_SCAN;
	ADC1->RSQR1 = 0;
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = 0; /* channel 0 */
	ADC1->SAMPTR2 &= ~(7U << (3 * 0)); /* shortest sample time for dense sampling */
	ADC1->CTLR2 &= ~(ADC_EXTTRIG | ADC_CONT | ADC_DMA);
	ADC1->CTLR2 |= ADC_ADON | ADC_EXTSEL;

	uint64_t sum_adc = 0;
	uint32_t sample_count = 0;
	uint32_t start = funSysTick32();
	uint32_t budget = Ticks_from_Ms(duration_ms);

	while (TimeElapsed32u(funSysTick32(), start) < budget)
	{
		sum_adc += adc_read();
		sample_count++;
	}

	if (sample_count == 0)
		return 0;

	uint32_t avg_adc = (uint32_t)((sum_adc + (sample_count / 2)) / sample_count);
	int vsense_mv = (int)(((uint64_t)avg_adc * (uint32_t)vref_mv + (ADC_FULL_SCALE_COUNTS / 2)) / ADC_FULL_SCALE_COUNTS);
	int vdrop_mv = vref_mv - vsense_mv;
	if (vdrop_mv < 0)
		vdrop_mv = 0;

	*vdrop_avg_mv_out = vdrop_mv;
	return 1;
}

static int capture_pwm_decimated(int decim_factor, uint32_t *fs_eff_hz)
{
	stop_adc_dma_capture();

	/* Max ADC throughput on PA2/channel 0. */
	ADC1->CTLR1 &= ~ADC_SCAN;
	ADC1->RSQR1 = 0;
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = 0;
	ADC1->SAMPTR2 &= ~(7U << (3 * 0));

	DMA1->INTFCR = DMA_CGIF1 | DMA_CTCIF1 | DMA_CHTIF1 | DMA_CTEIF1;
	DMA1_Channel1->CFGR = 0;
	DMA1_Channel1->PADDR = (uint32_t)&ADC1->RDATAR;
	DMA1_Channel1->MADDR = (uint32_t)adc_dma_chunk;
	DMA1_Channel1->CNTR  = PWM_DMA_CHUNK_SAMPLES;
	DMA1_Channel1->CFGR  =
		DMA_DIR_PeripheralSRC |
		DMA_CFGR1_CIRC |
		DMA_CFGR1_MSIZE_0 |
		DMA_CFGR1_PSIZE_0 |
		DMA_CFGR1_MINC |
		DMA_CFGR1_EN;

	ADC1->CTLR2 &= ~(ADC_EXTTRIG | ADC_CONT | ADC_DMA);
	ADC1->CTLR2 |= ADC_EXTSEL | ADC_CONT | ADC_DMA | ADC_ADON;
	ADC1->CTLR2 |= ADC_SWSTART;

	uint32_t start_ticks = funSysTick32();
	if (decim_factor <= 0)
		return 0;

	uint32_t timeout_ms = (uint32_t)(((uint32_t)PWM_DMA_TIMEOUT_MS * (uint32_t)decim_factor +
	                                 (PWM_DECIM_FACTOR_FAST - 1U)) / PWM_DECIM_FACTOR_FAST);
	if (timeout_ms < PWM_DMA_TIMEOUT_MS)
		timeout_ms = PWM_DMA_TIMEOUT_MS;
	if (timeout_ms > 300U)
		timeout_ms = 300U;

	uint32_t timeout_ticks = Ticks_from_Ms(timeout_ms);
	int decim_idx = 0;
	uint32_t accum = 0;
	int accum_n = 0;

	while (decim_idx < PWM_DECIM_POINTS)
	{
		if (DMA1->INTFR & DMA1_FLAG_TC1)
		{
			DMA1->INTFCR = DMA_CTCIF1;
			for (int i = 0; i < PWM_DMA_CHUNK_SAMPLES; i++)
			{
				accum += adc_dma_chunk[i];
				accum_n++;
				if (accum_n == decim_factor)
				{
					pwm_decim[decim_idx++] = (int16_t)((accum + (decim_factor / 2)) / decim_factor);
					accum = 0;
					accum_n = 0;
					if (decim_idx >= PWM_DECIM_POINTS)
						break;
				}
			}
		}

		if (TimeElapsed32u(funSysTick32(), start_ticks) > timeout_ticks)
		{
			stop_adc_dma_capture();
			return 0;
		}
	}

	uint32_t elapsed_ticks = TimeElapsed32u(funSysTick32(), start_ticks);
	stop_adc_dma_capture();

	if (elapsed_ticks == 0)
		return 0;

	const uint32_t raw_samples = PWM_DECIM_POINTS * (uint32_t)decim_factor;
	const uint64_t tick_hz = (uint64_t)DELAY_US_TIME * 1000000ULL;
	uint64_t fs_raw = ((uint64_t)raw_samples * tick_hz + (elapsed_ticks / 2)) / elapsed_ticks;
	/* Analysis runs on decimated samples, so report effective decimated sample rate. */
	*fs_eff_hz = (uint32_t)((fs_raw + ((uint32_t)decim_factor / 2U)) / (uint32_t)decim_factor);
	return 1;
}

static int64_t autocorr_lag(const int16_t *x, int n, int lag, int64_t *e0_out, int64_t *e1_out)
{
	int m = n - lag;
	int64_t corr = 0;
	int64_t e0 = 0;
	int64_t e1 = 0;

	for (int i = 0; i < m; i++)
	{
		int32_t a = x[i];
		int32_t b = x[i + lag];
		corr += (int64_t)a * b;
		e0 += (int64_t)a * a;
		e1 += (int64_t)b * b;
	}

	if (e0_out)
		*e0_out = e0;
	if (e1_out)
		*e1_out = e1;
	return corr;
}

static int estimate_pwm_hz(uint32_t fs_eff_hz, int freq_min_hz, int freq_max_hz, uint32_t quality_min_permille,
                           uint32_t *out_hz, int *out_best_lag, uint32_t *out_quality_permille)
{
	int64_t sum = 0;
	for (int i = 0; i < PWM_DECIM_POINTS; i++)
		sum += pwm_decim[i];

	int32_t mean = (int32_t)(sum / PWM_DECIM_POINTS);
	for (int i = 0; i < PWM_DECIM_POINTS; i++)
		pwm_decim[i] = (int16_t)(pwm_decim[i] - mean);

	if (freq_min_hz <= 0 || freq_max_hz <= 0 || freq_min_hz > freq_max_hz)
		return 0;

	int lag_min = (int)(fs_eff_hz / (uint32_t)freq_max_hz);
	int lag_max = (int)((fs_eff_hz + (uint32_t)freq_min_hz - 1U) / (uint32_t)freq_min_hz);
	if (lag_min < 1)
		lag_min = 1;
	if (lag_max > (PWM_DECIM_POINTS - 3))
		lag_max = PWM_DECIM_POINTS - 3;
	if (lag_min > lag_max)
		return 0;

	int best_lag = -1;
	int64_t best_corr = LLONG_MIN;
	int64_t best_e0 = 0;
	int64_t best_e1 = 0;

	for (int lag = lag_min; lag <= lag_max; lag++)
	{
		int64_t e0 = 0, e1 = 0;
		int64_t corr = autocorr_lag(pwm_decim, PWM_DECIM_POINTS, lag, &e0, &e1);
		if (corr > best_corr)
		{
			best_corr = corr;
			best_lag = lag;
			best_e0 = e0;
			best_e1 = e1;
		}
	}

	if (best_lag < 0 || best_corr <= 0)
		return 0;

	int64_t qden = (best_e0 < best_e1) ? best_e0 : best_e1;
	if (qden <= 0)
		return 0;

	uint32_t quality_permille = (uint32_t)((best_corr * 1000) / qden);
	if (out_best_lag)
		*out_best_lag = best_lag;
	if (out_quality_permille)
		*out_quality_permille = quality_permille;

	if (quality_permille < quality_min_permille)
		return 0;

	int64_t y1 = autocorr_lag(pwm_decim, PWM_DECIM_POINTS, best_lag - 1, 0, 0);
	int64_t y2 = best_corr;
	int64_t y3 = autocorr_lag(pwm_decim, PWM_DECIM_POINTS, best_lag + 1, 0, 0);

	int32_t delta_q8 = 0;
	int64_t denom = (y1 - (2 * y2) + y3);
	if (denom != 0)
	{
		int64_t num = (y1 - y3) << 7; /* ((y1-y3)<<8)/(2*denom) */
		delta_q8 = (int32_t)(num / denom);
		if (delta_q8 > 128)
			delta_q8 = 128;
		if (delta_q8 < -128)
			delta_q8 = -128;
	}

	int32_t lag_q8 = (best_lag << 8) + delta_q8;
	if (lag_q8 <= 0)
		return 0;

	uint64_t hz = (((uint64_t)fs_eff_hz << 8) + (lag_q8 / 2)) / (uint32_t)lag_q8;
	if (hz < (uint32_t)freq_min_hz || hz > (uint32_t)freq_max_hz)
		return 0;

	*out_hz = (uint32_t)hz;
	return 1;
}

static int compute_vdrop_levels_from_trace(int vref_mv, int *vdrop_off_mv, int *vdrop_on_mv,
                                           int *vdrop_min_mv, int *vdrop_max_mv);

#if PWM_CAPTURE_TRACE_DEBUG
static void log_pwm_capture_trace(const char *pass_name, int attempt,
                                  int decim_factor, uint32_t fs_eff_hz)
{
	LOG_LINE("pwm_trace_begin\t%s\t%d\t%d\t%lu",
	         pass_name, attempt, decim_factor, (unsigned long)fs_eff_hz);
	LOG_LINE("sample\tadc_decim");
	for (int i = 0; i < PWM_DECIM_POINTS; i++)
		LOG_LINE("%d\t%d", i, (int)pwm_decim[i]);
	LOG_LINE("pwm_trace_end\t%s\t%d", pass_name, attempt);
}
#endif

static int run_pwm_measurement_pass(int vref_mv, int decim_factor,
                                    int freq_min_hz, int freq_max_hz,
                                    uint32_t runtime_budget_ms,
                                    const char *pass_name,
                                    int *valid_runs_out,
                                    uint64_t *hz_sum_out,
                                    int64_t *vdrop_off_sum_out,
                                    int64_t *vdrop_on_sum_out,
                                    uint32_t *fs_eff_hz_avg_out)
{
	uint32_t budget_start = funSysTick32();
	uint32_t budget_ticks = Ticks_from_Ms(runtime_budget_ms);
	int valid_runs = 0;
	int attempts = 0;
	uint64_t hz_sum = 0;
	int64_t vdrop_off_sum = 0;
	int64_t vdrop_on_sum = 0;
	uint64_t fs_eff_sum = 0;
	int fs_eff_n = 0;

	while (attempts < PWM_MAX_RUNS &&
	       (attempts < PWM_MIN_VALID_RUNS ||
	        TimeElapsed32u(funSysTick32(), budget_start) < budget_ticks))
	{
		uint32_t fs_eff_hz = 0;
		if (!capture_pwm_decimated(decim_factor, &fs_eff_hz))
			return 0;
		fs_eff_sum += fs_eff_hz;
		fs_eff_n++;

#if PWM_CAPTURE_TRACE_DEBUG
		log_pwm_capture_trace(pass_name, attempts + 1, decim_factor, fs_eff_hz);
#endif

		int run_vdrop_min = 0;
		int run_vdrop_max = 0;
		int run_vdrop_off = 0;
		int run_vdrop_on = 0;
		int levels_ok = compute_vdrop_levels_from_trace(vref_mv,
		                                                &run_vdrop_off,
		                                                &run_vdrop_on,
		                                                &run_vdrop_min,
		                                                &run_vdrop_max);

		uint32_t pwm_hz = 0;
		if (levels_ok &&
		    estimate_pwm_hz(fs_eff_hz, freq_min_hz, freq_max_hz, PWM_QUALITY_MIN_PERMILLE,
		                    &pwm_hz, 0, 0))
		{
			hz_sum += pwm_hz;
			vdrop_off_sum += run_vdrop_off;
			vdrop_on_sum += run_vdrop_on;
			valid_runs++;
		}

		attempts++;
	}

	*valid_runs_out = valid_runs;
	*hz_sum_out = hz_sum;
	*vdrop_off_sum_out = vdrop_off_sum;
	*vdrop_on_sum_out = vdrop_on_sum;
	if (fs_eff_hz_avg_out)
		*fs_eff_hz_avg_out = (fs_eff_n > 0) ? (uint32_t)((fs_eff_sum + (uint64_t)(fs_eff_n / 2)) / (uint64_t)fs_eff_n) : 0;
	return 1;
}

static int compute_vdrop_levels_from_trace(int vref_mv, int *vdrop_off_mv, int *vdrop_on_mv,
                                           int *vdrop_min_mv, int *vdrop_max_mv)
{
	int vmin = INT_MAX;
	int vmax = INT_MIN;

	for (int i = 0; i < PWM_DECIM_POINTS; i++)
	{
		uint16_t adc = (uint16_t)pwm_decim[i];
		int vsense_mv = (int)(((uint64_t)adc * (uint32_t)vref_mv +
		                      (ADC_FULL_SCALE_COUNTS / 2)) / ADC_FULL_SCALE_COUNTS);
		int vdrop_mv = vref_mv - vsense_mv;
		if (vdrop_mv < 0)
			vdrop_mv = 0;
		if (vdrop_mv < vmin)
			vmin = vdrop_mv;
		if (vdrop_mv > vmax)
			vmax = vdrop_mv;
	}

	if (vmax <= vmin)
		return 0;

	/* Duty is low (12.5%), so pick a threshold close to baseline and average both clusters. */
	int thr = vmin + ((vmax - vmin) / 3);
	int64_t off_sum = 0, on_sum = 0;
	int off_n = 0, on_n = 0;

	for (int i = 0; i < PWM_DECIM_POINTS; i++)
	{
		uint16_t adc = (uint16_t)pwm_decim[i];
		int vsense_mv = (int)(((uint64_t)adc * (uint32_t)vref_mv +
		                      (ADC_FULL_SCALE_COUNTS / 2)) / ADC_FULL_SCALE_COUNTS);
		int vdrop_mv = vref_mv - vsense_mv;
		if (vdrop_mv < 0)
			vdrop_mv = 0;

		if (vdrop_mv <= thr)
		{
			off_sum += vdrop_mv;
			off_n++;
		}
		else
		{
			on_sum += vdrop_mv;
			on_n++;
		}
	}

	if (off_n < 16 || on_n < 4)
		return 0;

	*vdrop_off_mv = (int)((off_sum + (off_n / 2)) / off_n);
	*vdrop_on_mv  = (int)((on_sum + (on_n / 2)) / on_n);
	*vdrop_min_mv = vmin;
	*vdrop_max_mv = vmax;
	return 1;
}

static void test_pwm_rate_measurement(void)
{
	LOG_SECTION("pwm_rate");

	int vcal_counts = -1;
	int vrefint_counts = -1;
	int vref_mv = measure_vref_mv(&vcal_counts, &vrefint_counts);
	if (vref_mv <= 0)
	{
		LOG_KV_STR("status", "fail");
		LOG_KV_STR("reason", "vref_cal");
		return;
	}
	LOG_KV_INT("vref_mv", vref_mv);
	LOG_KV_INT("vrefint_cnt", vrefint_counts);
	LOG_KV_INT("vcal_cnt", vcal_counts);

	/* PWM stimulus with LED on (red, 12.5%). */
	if (!send_ws2812_frame(WS2812_PWM_GREEN_VALUE, WS2812_PWM_RED_VALUE, WS2812_PWM_BLUE_VALUE))
	{
		LOG_KV_STR("status", "fail");
		LOG_KV_STR("reason", "adc_dma_timeout");
		return;
	}

	int valid_runs = 0;
	uint64_t hz_sum = 0;
	int64_t vdrop_off_sum = 0;
	int64_t vdrop_on_sum = 0;
	uint32_t fs_eff_hz_avg = 0;
	int used_decim_factor = PWM_DECIM_FACTOR_FAST;

	if (!run_pwm_measurement_pass(vref_mv,
	                              PWM_DECIM_FACTOR_FAST,
	                              PWM_FREQ_MIN_HZ,
	                              PWM_FREQ_MAX_HZ,
	                              PWM_RUNTIME_BUDGET_MS,
	                              "fast",
	                              &valid_runs,
	                              &hz_sum,
	                              &vdrop_off_sum,
	                              &vdrop_on_sum,
	                              &fs_eff_hz_avg))
	{
		LOG_KV_STR("status", "fail");
		LOG_KV_STR("reason", "adc_dma_timeout");
		return;
	}

	if (valid_runs < PWM_MIN_VALID_RUNS)
	{
		/* Fallback: higher decimation, low-frequency-focused search window. */
		int valid_runs_slow = 0;
		uint64_t hz_sum_slow = 0;
		int64_t vdrop_off_sum_slow = 0;
		int64_t vdrop_on_sum_slow = 0;
		uint32_t fs_eff_hz_avg_slow = 0;

		if (!run_pwm_measurement_pass(vref_mv,
		                              PWM_DECIM_FACTOR_SLOW,
		                              PWM_FREQ_MIN_HZ_SLOW,
		                              PWM_FREQ_MAX_HZ_SLOW,
		                              PWM_RUNTIME_BUDGET_MS_SLOW,
		                              "slow",
		                              &valid_runs_slow,
		                              &hz_sum_slow,
		                              &vdrop_off_sum_slow,
		                              &vdrop_on_sum_slow,
		                              &fs_eff_hz_avg_slow))
		{
			LOG_KV_STR("status", "fail");
			LOG_KV_STR("reason", "adc_dma_timeout");
			return;
		}

		if (valid_runs_slow >= PWM_MIN_VALID_RUNS)
		{
			valid_runs = valid_runs_slow;
				hz_sum = hz_sum_slow;
				vdrop_off_sum = vdrop_off_sum_slow;
				vdrop_on_sum = vdrop_on_sum_slow;
				fs_eff_hz_avg = fs_eff_hz_avg_slow;
				used_decim_factor = PWM_DECIM_FACTOR_SLOW;
			}
			else
			{
			LOG_KV_STR("status", "fail");
			LOG_KV_STR("reason", "no_valid_peak");
			return;
		}
	}

	int vdrop_off_mv = (int)((vdrop_off_sum + (valid_runs / 2)) / valid_runs);
	int vdrop_on_mv = (int)((vdrop_on_sum + (valid_runs / 2)) / valid_runs);
	int vdrop_led_mv = vdrop_on_mv - vdrop_off_mv;

	uint32_t pwm_hz_avg = (uint32_t)((hz_sum + (uint64_t)(valid_runs / 2)) / (uint64_t)valid_runs);
	LOG_KV_U32("pwm_hz", pwm_hz_avg);
	LOG_KV_INT("vdrop_off_mv", vdrop_off_mv);
	LOG_KV_INT("vdrop_on_mv", vdrop_on_mv);
	LOG_KV_INT("vdrop_led_mv", vdrop_led_mv);
	LOG_KV_U32("fs_eff_hz", fs_eff_hz_avg);
	LOG_KV_U32("fs_raw_hz", (uint32_t)((uint64_t)fs_eff_hz_avg * (uint64_t)used_decim_factor));
	LOG_KV_INT("decim_factor", used_decim_factor);
	LOG_KV_INT("runs", valid_runs);
	LOG_KV_STR("status", "ok");
}

/* ---------- Test 4: LED current measurement ---------- */

static int measure_led_current_point(int channel_index, const char *channel_label, uint8_t duty,
                                     int vref_mv, const char **reason_out)
{
	uint8_t r = 0, g = 0, b = 0;
	if (channel_index == 0)
		g = duty;
	else if (channel_index == 1)
		r = duty;
	else
		b = duty;

	if (!send_ws2812_frame(g, r, b))
	{
		if (reason_out)
			*reason_out = "dma_timeout";
		return 0;
	}

	int vdrop_avg_mv = 0;
	if (!capture_vdrop_average_mv(vref_mv, LED_MEAS_DURATION_MS, &vdrop_avg_mv))
	{
		if (reason_out)
			*reason_out = "adc_sample";
		return 0;
	}

	LOG_LINE("%s\t%u\t%d", channel_label, (unsigned int)duty, vdrop_avg_mv);
	return 1;
}

static void test_led_current_measurement(void)
{
	static const uint8_t coarse_steps[] = { 64, 96, 128, 160, 192, 224, 255 };
	static const char *channel_names[] = { "CH1", "CH2", "CH3" };

	LOG_SECTION("led_current");

	int vcal_counts = -1;
	int vrefint_counts = -1;
	int vref_mv = measure_vref_mv(&vcal_counts, &vrefint_counts);
	if (vref_mv <= 0)
	{
		LOG_KV_STR("status", "fail");
		LOG_KV_STR("reason", "vref_cal");
		return;
	}

	LOG_KV_INT("vref_mv", vref_mv);
	LOG_LINE("channel\tduty\tvdrop_avg_mv");

	for (int c = 0; c < 3; c++)
	{
		const char *reason = "adc_sample";
		for (uint8_t duty = 0; duty <= 63; duty+=2)
		{
			if (!measure_led_current_point(c, channel_names[c], duty, vref_mv, &reason))
			{
				LOG_KV_STR("status", "fail");
				LOG_KV_STR("reason", reason);
				return;
			}
		}

		for (int i = 0; i < (int)(sizeof(coarse_steps) / sizeof(coarse_steps[0])); i++)
		{
			if (!measure_led_current_point(c, channel_names[c], coarse_steps[i], vref_mv, &reason))
			{
				LOG_KV_STR("status", "fail");
				LOG_KV_STR("reason", reason);
				return;
			}
		}
	}

	/* Leave LED off after the sweep. */
	send_ws2812_frame(0, 0, 0);
	LOG_KV_STR("status", "ok");
}

/* ---------- Main ---------- */

int main(void)
{
	SystemInit();

	/* Enable GPIO port clocks (A, C for peripherals; D for debug SWIO) */
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC |
	                   RCC_APB2Periph_GPIOD;

	init_timer();
	init_adc();

	Delay_Ms(100);

	LOG_SECTION("boot");
	LOG_KV_STR("fw", "ws2812_tester");
	LOG_KV_STR("mcu", "ch32v003");
	LOG_KV_STR("Rsense [Ohm]", "47.7");

	LOG_SECTION("Device (update from datasheet)");
	LOG_KV_STR("Manufacturer", "Worldsemi");
	LOG_KV_STR("Type", "WS2812");
	LOG_KV_STR("IC", "WS2812");
	LOG_KV_STR("LCSC", "Cxxxxxx");
	LOG_KV_STR("Channel Order", "GRB");

	test_txh_sweep();
	test_reset_time();
	test_pwm_rate_measurement();
	test_led_current_measurement();

	LOG_SECTION("tests_complete");
	LOG_KV_STR("status", "ok");

	while (1)
		Delay_Ms(1000);
}

#include <msp430.h>
#include <stdint.h>

// ============================================================================
//  HARDWARE ABSTRACTION LAYER (HAL) - PIN DEFINITIONS
// ============================================================================

// --- SPI Port Configuration (P3) ---
#define SPI_PORT_DIR        P3DIR
#define SPI_PORT_OUT        P3OUT
#define SPI_PORT_SEL0       P3SEL0
#define SPI_PORT_SEL1       P3SEL1

// --- SPI Pin Assignments ---
// P3.4 <-- AD5722 Pin 7 (/SYNC)
#define PIN_SPI_CS          BIT4    // P3.4
// P3.5 <-- AD5722 Pin 8 (SCLK)
#define PIN_SPI_CLK         BIT5    // P3.5
// P3.6 <-- AD5722 Pin 9 (SDIN)
#define PIN_SPI_MOSI        BIT6    // P3.6

// --- ADC Inputs ---
// Channel A: P1.2 / A2
#define ADC_A_PORT_SEL0     P1SEL0
#define ADC_A_PORT_SEL1     P1SEL1
#define ADC_A_PIN           BIT2            // P1.2 (A2)
#define ADC_A_CHANNEL       ADC12INCH_2

// Channel B: P3.0 / A12
#define ADC_B_PORT_SEL0     P3SEL0
#define ADC_B_PORT_SEL1     P3SEL1
#define ADC_B_PIN           BIT0            // P3.0 (A12)
#define ADC_B_CHANNEL       ADC12INCH_12    // A12

// ============================================================================
//  AD5722 REGISTER DEFINITIONS
// ============================================================================
#define AD_REG_DAC          0x000000UL   // 0000 = Write DAC register
#define AD_REG_RANGE        0x080000UL   // 1000 = Output range select
#define AD_REG_POWER        0x100000UL   // 0001 = Power control

// ADDR field [19:16]
#define AD_ADDR_A           0x000000UL   // DAC A (VOUTA, pin 3)
#define AD_ADDR_B           0x020000UL   // DAC B (VOUTB, pin 23)
#define AD_ADDR_BOTH        0x040000UL   // Both DACs

// ============================================================================
//  DSP / SCALING SETTINGS
// ============================================================================

// ---------- EMA settings ----------
// Channel A: one-stage EMA (strong smoothing)
#define FAST_SHIFT_A        5       // alpha_A = 1/32

// Channel B: TWO-stage EMA -> very strong smoothing
#define FAST1_SHIFT_B       5       // first stage alpha = 1/32
#define FAST2_SHIFT_B       5       // second stage alpha = 1/32

// Baseline trackers (very slow EMAs) – same for both
#define BASE_SHIFT_A        9       // alpha_base ≈ 1/512
#define BASE_SHIFT_B        9

// ---------- Nonlinear delta / gain settings ----------
// Tuned for *smaller* dips (stronger analogue signal)

// Below this → treated as noise
#define DELTA_NOISE_THRESH  4U      // was 8

// From NOISE..SMALL_DELTA → small/medium-gain region
#define SMALL_DELTA_THRESH  16U     // was 30

// Gain for small/medium dips (helps 1–3 mm)
#define SMALL_GAIN          12U     // was 7

// Gain for big dips (4–10 mm etc.)
#define DROP_GAIN           20U     // was 10

// Scale 0–3.3 V ADC → 0–3.3 V DAC (AD5722 is 0–5 V)
#define ADC_TO_DAC_NUM      33U     // 3.3
#define ADC_TO_DAC_DEN      50U     // 5.0   => 33/50 ≈ 3.3/5
#define DAC_MAX_CODE        4095U

// ============================================================================
//  DROP-MIN DETECTION SETTINGS
// ============================================================================

// How far below baseline we consider "inside a drop" (in ADC codes)
#define DETECT_DELTA_CODES  10U     // was 20 – now more sensitive

// Maximum number of drops to record in RAM
#define MAX_DROPS           256

// ============================================================================
//  TinyML model settings (for Channel B only)
//  size_mm = a*ΔV^2 + b*ΔV + c, with ΔV in volts
// ============================================================================
#define ADC_MAX_CODE        4095U
#define ADC_REF_VOLT        3.3f
#define ADC_CODE_TO_VOLTS   (ADC_REF_VOLT / (float)ADC_MAX_CODE)

#define MIN_DV_FOR_EST      0.05f   // ignore ultratiny dips

const float DROP_A = -2.6851f;      // from your Python fit (older model)
const float DROP_B = 10.5871f;
const float DROP_C = -0.0380f;

const float DROP_DV_MIN = 0.55f;    // calibration range (V)
const float DROP_DV_MAX = 1.80f;

// ============================================================================
//  Filter accumulators
// ============================================================================

// Channel A: fast + baseline
volatile uint32_t g_fast_acc_A  = 0;
volatile uint32_t g_base_acc_A  = 0;

// Channel B: two fast stages + baseline
volatile uint32_t g_fast1_acc_B = 0;
volatile uint32_t g_fast2_acc_B = 0;
volatile uint32_t g_base_acc_B  = 0;

// ============================================================================
//  Drop-min detection state
// ============================================================================

// A-channel detection
volatile uint8_t  g_inAnomaly_A    = 0;
volatile uint16_t g_current_min_A  = 0xFFFF;
volatile uint16_t g_drop_mins_A[MAX_DROPS];
volatile uint16_t g_drop_count_A   = 0;

// B-channel detection
volatile uint8_t  g_inAnomaly_B    = 0;
volatile uint16_t g_current_min_B  = 0xFFFF;
volatile uint16_t g_drop_mins_B[MAX_DROPS];
volatile uint16_t g_drop_count_B   = 0;

// For TinyML (channel B)
volatile uint8_t  g_drop_ready_B   = 0;
volatile uint16_t g_last_min_B     = 0;
volatile uint16_t g_last_base_B    = 0;

// ============================================================================
//  Green LED multi-flash state (TinyML result)
//  We flash P1.1 N times per valid drop.
// ============================================================================
#define GREEN_ON_TICKS   40    // LED ON duration per flash (loop iterations)
#define GREEN_OFF_TICKS  40    // LED OFF gap between flashes

static uint8_t  g_green_flashes_remaining = 0;
static uint16_t g_green_tick              = 0;
static uint8_t  g_green_state             = 0; // 0=idle,1=on,2=off-between-flashes

// ============================================================================
//  Prototypes
// ============================================================================
void initClock(void);
void initGPIO(void);
void initSPI(void);
void initDAC(void);
void initADC(void);
void initTimer(void);
void DAC_write_register(uint32_t data24);

float tinyml_estimate_size_from_codes(uint16_t base_code, uint16_t min_code);
void  schedule_green_flashes(uint8_t flashes);

// ==================== MAIN ====================
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;       // Stop watchdog

    initClock();
    initGPIO();
    initSPI();
    initDAC();
    initADC();
    initTimer();

    // Initialise EMAs to mid-scale (2048)
    g_fast_acc_A  = 2048U << FAST_SHIFT_A;
    g_base_acc_A  = 2048U << BASE_SHIFT_A;

    g_fast1_acc_B = 2048U << FAST1_SHIFT_B;
    g_fast2_acc_B = 2048U << FAST2_SHIFT_B;
    g_base_acc_B  = 2048U << BASE_SHIFT_B;

    __enable_interrupt();

    while (1)
    {
        // --- TinyML processing for Channel B (outside ISR) ---
        if (g_drop_ready_B)
        {
            uint16_t base_code, min_code;

            __disable_interrupt();
            base_code      = g_last_base_B;
            min_code       = g_last_min_B;
            g_drop_ready_B = 0;
            __enable_interrupt();

            float size_mm = tinyml_estimate_size_from_codes(base_code, min_code);

            // Map size_mm to number of flashes:
            // 5mm->1, 6mm->2, ..., 10mm->6
            if (size_mm >= 5.0f && size_mm <= 10.0f)
            {
                uint8_t rounded = (uint8_t)(size_mm + 0.5f); // nearest integer mm
                uint8_t flashes = 0;
                if (rounded >= 5) flashes = (uint8_t)(rounded - 4); // 5->1, 10->6
                if (flashes > 0)
                    schedule_green_flashes(flashes);
            }
        }

        // --- Green LED flash state machine (P1.1) ---
        if (g_green_flashes_remaining == 0)
        {
            // No active sequence: ensure LED off
            P1OUT &= ~BIT1;
            g_green_state = 0;
        }
        else
        {
            if (g_green_state == 0)
            {
                // Start first flash: LED on
                P1OUT |= BIT1;
                g_green_state = 1;
                g_green_tick  = GREEN_ON_TICKS;
            }
            else if (g_green_state == 1)
            {
                // LED currently ON
                if (g_green_tick > 0)
                {
                    g_green_tick--;
                }
                else
                {
                    // End ON period
                    P1OUT &= ~BIT1;
                    g_green_state = 2;
                    g_green_tick  = GREEN_OFF_TICKS;
                }
            }
            else if (g_green_state == 2)
            {
                // OFF gap between flashes
                if (g_green_tick > 0)
                {
                    g_green_tick--;
                }
                else
                {
                    // One flash complete
                    if (g_green_flashes_remaining > 0)
                        g_green_flashes_remaining--;

                    if (g_green_flashes_remaining == 0)
                    {
                        g_green_state = 0;
                    }
                    else
                    {
                        // Start next flash
                        P1OUT |= BIT1;
                        g_green_state = 1;
                        g_green_tick  = GREEN_ON_TICKS;
                    }
                }
            }
        }

        __no_operation();           // All real work done in ISRs + above
    }
}

/* ---------------- CLOCK: SMCLK = 8 MHz ---------------- */
void initClock(void)
{
    FRCTL0 = FRCTLPW | NWAITS_1;
    CSCTL0_H = CSKEY >> 8;
    CSCTL1   = DCOFSEL_3;           // 8 MHz
    CSCTL2   = SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3   = DIVS__1 | DIVM__1;
    CSCTL0_H = 0;
}

/* ---------------- GPIO Setup ---------------- */
void initGPIO(void)
{
    PM5CTL0 &= ~LOCKLPM5;           // Unlock GPIO

    // LEDs on P1.0 (red) and P1.1 (green)
    P1DIR |= (BIT0 | BIT1);
    P1OUT &= ~(BIT0 | BIT1);

    // --- ADC pins ---
    // A2 on P1.2
    ADC_A_PORT_SEL0 |= ADC_A_PIN;
    ADC_A_PORT_SEL1 |= ADC_A_PIN;

    // A12 on P3.0
    ADC_B_PORT_SEL0 |= ADC_B_PIN;
    ADC_B_PORT_SEL1 |= ADC_B_PIN;

    // --- SPI pins as GPIO outputs (bit-banged) ---
    SPI_PORT_DIR |= (PIN_SPI_MOSI | PIN_SPI_CLK | PIN_SPI_CS);

    // Force them to GPIO (no alternate functions)
    SPI_PORT_SEL0 &= ~(PIN_SPI_MOSI | PIN_SPI_CLK | PIN_SPI_CS);
    SPI_PORT_SEL1 &= ~(PIN_SPI_MOSI | PIN_SPI_CLK | PIN_SPI_CS);

    // Initial SPI levels
    SPI_PORT_OUT |= PIN_SPI_CS;     // /SYNC high
    SPI_PORT_OUT |= PIN_SPI_CLK;    // SCLK high (idle)
    SPI_PORT_OUT &= ~PIN_SPI_MOSI;  // MOSI low
}

/* ---------------- SOFTWARE SPI Setup ---------------- */
void initSPI(void)
{
    // Already configured in initGPIO()
}

/* ---------------- AD5722 DAC Initialization ---------------- */
void initDAC(void)
{
    // 1. Power up DAC A and DAC B (PUA + PUB)
    DAC_write_register(AD_REG_POWER | 0x0005);

    // 2. Set both outputs to 0–5 V range
    DAC_write_register(AD_REG_RANGE | AD_ADDR_BOTH | 0x0000);
}

/* ---------------- Send 24-bit Word to DAC (BIT-BANGED) ---------------- */
void DAC_write_register(uint32_t data24)
{
    uint8_t i;

    // Start frame: /SYNC low
    SPI_PORT_OUT &= ~PIN_SPI_CS;
    __delay_cycles(4);

    // 24 bits, MSB first
    for (i = 0; i < 24; i++)
    {
        if (data24 & 0x800000UL)
            SPI_PORT_OUT |= PIN_SPI_MOSI;
        else
            SPI_PORT_OUT &= ~PIN_SPI_MOSI;

        // SCLK low
        SPI_PORT_OUT &= ~PIN_SPI_CLK;
        __delay_cycles(4);

        // SCLK high – DAC latches on this edge
        SPI_PORT_OUT |= PIN_SPI_CLK;
        __delay_cycles(4);

        data24 <<= 1;
    }

    // End frame: /SYNC high
    SPI_PORT_OUT |= PIN_SPI_CS;

    // Optionally clear MOSI
    SPI_PORT_OUT &= ~PIN_SPI_MOSI;
}

/* ---------------- ADC / Timer Setup ---------------- */
void initADC(void)
{
    ADC12CTL0 = ADC12SHT0_2 | ADC12ON;
    ADC12CTL1 = ADC12SHP | ADC12CONSEQ_1;     // Sequence of channels
    ADC12CTL2 = ADC12RES_2;                   // 12-bit

    // Sequence: MEM0 = A2, MEM1 = A12 (EOS)
    ADC12MCTL0 = ADC_A_CHANNEL;
    ADC12MCTL1 = ADC_B_CHANNEL | ADC12EOS;

    ADC12IER0  = ADC12IE1;                    // Interrupt at end-of-sequence
    ADC12CTL0 |= ADC12ENC;
}

void initTimer(void)
{
    TA0CCR0  = 800 - 1;                       // 8 MHz / 800 = 10 kHz
    TA0CCTL0 = CCIE;
    TA0CTL   = TASSEL_2 | MC_1 | TACLR;
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    ADC12CTL0 |= ADC12SC;                     // Start conversion
}

/* ---------------- ADC ISR: Filter, Detect & Update AD5722 ---------------- */
#pragma vector = ADC12_B_VECTOR
__interrupt void ADC12_B_ISR(void)
{
    switch (__even_in_range(ADC12IV, ADC12IV_ADC12RDYIFG))
    {
    case ADC12IV_ADC12IFG1: // End of sequence (MEM0 & MEM1 valid)
    {
        uint16_t raw_A = ADC12MEM0;   // A2 → Channel A
        uint16_t raw_B = ADC12MEM1;   // A12 → Channel B

        // ================= CHANNEL A =================
        // Fast EMA (strongly smoothed)
        g_fast_acc_A = g_fast_acc_A - (g_fast_acc_A >> FAST_SHIFT_A) + raw_A;
        uint16_t fast_A = (uint16_t)(g_fast_acc_A >> FAST_SHIFT_A);

        // Baseline EMA (very slow)
        g_base_acc_A = g_base_acc_A - (g_base_acc_A >> BASE_SHIFT_A) + fast_A;
        uint16_t base_A = (uint16_t)(g_base_acc_A >> BASE_SHIFT_A);

        // Dip depth
        uint16_t delta_A = 0;
        if (base_A > fast_A)
            delta_A = (uint16_t)(base_A - fast_A);

        // ---------- non-linear gain for A ----------
        uint32_t amp_delta_A = 0;
        if (delta_A > DELTA_NOISE_THRESH)
        {
            if (delta_A <= SMALL_DELTA_THRESH)
            {
                // small / medium dips
                amp_delta_A = (uint32_t)delta_A * SMALL_GAIN;
            }
            else
            {
                // bigger dips
                amp_delta_A = (uint32_t)delta_A * DROP_GAIN;
            }
        }

        if (amp_delta_A > base_A) amp_delta_A = base_A;

        uint16_t out_A_code = (uint16_t)(base_A - amp_delta_A);

        // *** RED LED BEHAVIOUR (UNCHANGED) ***
        // LED on A: low when dipped
        if (out_A_code < 2048U) P1OUT |= BIT0;
        else                    P1OUT &= ~BIT0;

        // ======== DROP-MIN DETECTION – CHANNEL A ========
        uint16_t thr_A = (base_A > DETECT_DELTA_CODES) ?
                         (uint16_t)(base_A - DETECT_DELTA_CODES) : 0;

        if (!g_inAnomaly_A)
        {
            // Not currently in a drop: check if we just went below threshold
            if (out_A_code < thr_A)
            {
                g_inAnomaly_A   = 1;
                g_current_min_A = out_A_code;
            }
        }
        else
        {
            // Already in a drop: update minimum
            if (out_A_code < g_current_min_A)
                g_current_min_A = out_A_code;

            // Drop ends when we come back above threshold
            if (out_A_code >= thr_A)
            {
                if (g_drop_count_A < MAX_DROPS)
                    g_drop_mins_A[g_drop_count_A++] = g_current_min_A;

                g_inAnomaly_A   = 0;
                g_current_min_A = 0xFFFF;
            }
        }

        // ================= CHANNEL B =================
        // Stage 1 EMA
        g_fast1_acc_B = g_fast1_acc_B - (g_fast1_acc_B >> FAST1_SHIFT_B) + raw_B;
        uint16_t fast1_B = (uint16_t)(g_fast1_acc_B >> FAST1_SHIFT_B);

        // Stage 2 EMA (extra smoothing)
        g_fast2_acc_B = g_fast2_acc_B - (g_fast2_acc_B >> FAST2_SHIFT_B) + fast1_B;
        uint16_t fast_B = (uint16_t)(g_fast2_acc_B >> FAST2_SHIFT_B);

        // Baseline EMA for B
        g_base_acc_B = g_base_acc_B - (g_base_acc_B >> BASE_SHIFT_B) + fast_B;
        uint16_t base_B = (uint16_t)(g_base_acc_B >> BASE_SHIFT_B);

        // Dip depth
        uint16_t delta_B = 0;
        if (base_B > fast_B)
            delta_B = (uint16_t)(base_B - fast_B);

        // ---------- non-linear gain for B ----------
        uint32_t amp_delta_B = 0;
        if (delta_B > DELTA_NOISE_THRESH)
        {
            if (delta_B <= SMALL_DELTA_THRESH)
            {
                amp_delta_B = (uint32_t)delta_B * SMALL_GAIN;
            }
            else
            {
                amp_delta_B = (uint32_t)delta_B * DROP_GAIN;
            }
        }

        if (amp_delta_B > base_B) amp_delta_B = base_B;

        uint16_t out_B_code = (uint16_t)(base_B - amp_delta_B);

        // ======== DROP-MIN DETECTION – CHANNEL B ========
        uint16_t thr_B = (base_B > DETECT_DELTA_CODES) ?
                         (uint16_t)(base_B - DETECT_DELTA_CODES) : 0;

        if (!g_inAnomaly_B)
        {
            if (out_B_code < thr_B)
            {
                g_inAnomaly_B   = 1;
                g_current_min_B = out_B_code;
            }
        }
        else
        {
            if (out_B_code < g_current_min_B)
                g_current_min_B = out_B_code;

            if (out_B_code >= thr_B)
            {
                if (g_drop_count_B < MAX_DROPS)
                    g_drop_mins_B[g_drop_count_B++] = g_current_min_B;

                // Make this drop available to TinyML logic in main()
                g_last_min_B   = g_current_min_B;
                g_last_base_B  = base_B;
                g_drop_ready_B = 1;

                g_inAnomaly_B   = 0;
                g_current_min_B = 0xFFFF;
            }
        }

        // ================= SCALE TO DAC CODES =================
        uint32_t scaled_A =
            ((uint32_t)out_A_code * ADC_TO_DAC_NUM + (ADC_TO_DAC_DEN / 2U))
            / ADC_TO_DAC_DEN;
        if (scaled_A > DAC_MAX_CODE) scaled_A = DAC_MAX_CODE;

        uint32_t scaled_B =
            ((uint32_t)out_B_code * ADC_TO_DAC_NUM + (ADC_TO_DAC_DEN / 2U))
            / ADC_TO_DAC_DEN;
        if (scaled_B > DAC_MAX_CODE) scaled_B = DAC_MAX_CODE;

        // --- Output to DAC A (VOUTA, pin 3) ---
        uint32_t cmd_A = AD_REG_DAC | AD_ADDR_A | (scaled_A << 4);
        DAC_write_register(cmd_A);

        __delay_cycles(100);  // Small gap between frames

        // --- Output to DAC B (VOUTB, pin 23) ---
        uint32_t cmd_B = AD_REG_DAC | AD_ADDR_B | (scaled_B << 4);
        DAC_write_register(cmd_B);
    }
        break;

    default:
        break;
    }
}

// ============================================================================
//  TinyML helper: estimate size from ADC codes (Channel B)
// ============================================================================
float tinyml_estimate_size_from_codes(uint16_t base_code, uint16_t min_code)
{
    if (base_code <= min_code)
        return 0.0f;

    uint16_t delta_code = base_code - min_code;
    float deltaV = (float)delta_code * ADC_CODE_TO_VOLTS;

    if (deltaV < MIN_DV_FOR_EST)
        return 0.0f;

    // Clamp to calibrated delta-V range
    if (deltaV < DROP_DV_MIN) deltaV = DROP_DV_MIN;
    if (deltaV > DROP_DV_MAX) deltaV = DROP_DV_MAX;

    float size_mm = DROP_A * deltaV * deltaV
                  + DROP_B * deltaV
                  + DROP_C;

    if (size_mm < 0.0f)
        size_mm = 0.0f;

    return size_mm;
}

// ============================================================================
//  Schedule N green LED flashes (TinyML result)
// ============================================================================
void schedule_green_flashes(uint8_t flashes)
{
    // If a previous sequence is running, we just overwrite with the new one.
    g_green_flashes_remaining = flashes;
    g_green_state             = 0;
    g_green_tick              = 0;
}

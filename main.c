#include <msp430.h>
#include <stdint.h>
#include <stdio.h>

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

// ---------- Delta / threshold settings ----------

// How far below baseline we consider "inside a drop" (in ADC codes)
#define DETECT_DELTA_CODES  20U     // tweak if needed

// Minimum delta to bother estimating size (in volts, later)
#define MIN_DV_FOR_EST      0.05f

// Scale 0–3.3 V ADC → 0–3.3 V DAC (AD5722 is 0–5 V)
#define ADC_TO_DAC_NUM      33U     // 3.3
#define ADC_TO_DAC_DEN      50U     // 5.0   => 33/50 ≈ 3.3/5
#define DAC_MAX_CODE        4095U

#define ADC_CODE_TO_VOLTS   (3.3f / 4095.0f)

// ============================================================================
//  DROP-MIN DETECTION / COMMUNICATION
// ============================================================================

#define MAX_DROPS           256

// Filter accumulators
volatile uint32_t g_fast_acc_A  = 0;
volatile uint32_t g_base_acc_A  = 0;

volatile uint32_t g_fast1_acc_B = 0;
volatile uint32_t g_fast2_acc_B = 0;
volatile uint32_t g_base_acc_B  = 0;

// Drop-min detection state (A)
volatile uint8_t  g_inAnomaly_A    = 0;
volatile uint16_t g_current_min_A  = 0xFFFF;
volatile uint16_t g_drop_mins_A[MAX_DROPS];
volatile uint16_t g_drop_count_A   = 0;

// Drop-min detection state (B)
volatile uint8_t  g_inAnomaly_B    = 0;
volatile uint16_t g_current_min_B  = 0xFFFF;
volatile uint16_t g_drop_mins_B[MAX_DROPS];
volatile uint16_t g_drop_count_B   = 0;

// For TinyML estimator (B channel)
volatile uint8_t  g_drop_ready_B   = 0;
volatile uint16_t g_last_min_B     = 0;
volatile uint16_t g_last_base_B    = 0;

// Prototypes
void initClock(void);
void initGPIO(void);
void initSPI(void);
void initDAC(void);
void initADC(void);
void initTimer(void);
void DAC_write_register(uint32_t data24);

float estimate_drop_size_mm(float deltaV);
void  handle_drop_codes(uint16_t base_code, uint16_t min_code);

// ============================================================================
//  MAIN
// ============================================================================
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog

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
        // Process completed drops from channel B OUTSIDE the ISR
        if (g_drop_ready_B)
        {
            uint16_t base_code, min_code;

            __disable_interrupt();
            base_code        = g_last_base_B;
            min_code         = g_last_min_B;
            g_drop_ready_B   = 0;
            __enable_interrupt();

            handle_drop_codes(base_code, min_code);

            // Heartbeat: flash LED each processed drop
            P1OUT ^= BIT0;
        }

        __no_operation();
    }
}

/* ---------------- CLOCK: SMCLK = 8 MHz ---------------- */
void initClock(void)
{
    FRCTL0   = FRCTLPW | NWAITS_1;
    CSCTL0_H = CSKEY >> 8;
    CSCTL1   = DCOFSEL_3;           // 8 MHz
    CSCTL2   = SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3   = DIVS__1 | DIVM__1;
    CSCTL0_H = 0;
}

/* ---------------- GPIO Setup ---------------- */
void initGPIO(void)
{
    PM5CTL0 &= ~LOCKLPM5;           // Unlock GPIO (FRAM devices)

    // LED on P1.0
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;

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
    // 8 MHz / 1600 = 5 kHz sampling (calmer than 10 kHz)
    TA0CCR0  = 1600 - 1;
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

        // Dip depth (kept for info, not used for DAC here)
        uint16_t delta_A = 0;
        if (base_A > fast_A)
            delta_A = (uint16_t)(base_A - fast_A);

        // For DAC we just use the smoothed signal
        uint16_t out_A_code = fast_A;

        // Optional LED indication: low when dipped
        if (delta_A > DETECT_DELTA_CODES)
            P1OUT |= BIT0;
        else
            P1OUT &= ~BIT0;

        // ======== DROP-MIN DETECTION – CHANNEL A (optional logging) ========
        uint16_t thr_A = (base_A > DETECT_DELTA_CODES) ?
                         (uint16_t)(base_A - DETECT_DELTA_CODES) : 0;

        if (!g_inAnomaly_A)
        {
            if (fast_A < thr_A)
            {
                g_inAnomaly_A   = 1;
                g_current_min_A = fast_A;
            }
        }
        else
        {
            if (fast_A < g_current_min_A)
                g_current_min_A = fast_A;

            if (fast_A >= thr_A)
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

        // For DAC we just use smoothed signal
        uint16_t out_B_code = fast_B;

        // ======== DROP-MIN DETECTION – CHANNEL B ========
        uint16_t thr_B = (base_B > DETECT_DELTA_CODES) ?
                         (uint16_t)(base_B - DETECT_DELTA_CODES) : 0;

        if (!g_inAnomaly_B)
        {
            if (fast_B < thr_B)
            {
                g_inAnomaly_B   = 1;
                g_current_min_B = fast_B;
            }
        }
        else
        {
            if (fast_B < g_current_min_B)
                g_current_min_B = fast_B;

            if (fast_B >= thr_B)
            {
                // Store into ring/log (optional)
                if (g_drop_count_B < MAX_DROPS)
                    g_drop_mins_B[g_drop_count_B++] = g_current_min_B;

                // Make values available for TinyML in main loop
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
//  TinyML Quadratic Model (frozen from Python)
// ============================================================================

// Replace these with your final Python coefficients
const float DROP_A = -2.6851f;   // a
const float DROP_B = 10.5871f;   // b
const float DROP_C = -0.0380f;   // c

const float DROP_DV_MIN = 0.55f;
const float DROP_DV_MAX = 1.80f;

float estimate_drop_size_mm(float deltaV)
{
    // Clamp deltaV to calibrated region
    if (deltaV < DROP_DV_MIN) deltaV = DROP_DV_MIN;
    if (deltaV > DROP_DV_MAX) deltaV = DROP_DV_MAX;

    float size_mm = DROP_A * deltaV * deltaV
                  + DROP_B * deltaV
                  + DROP_C;

    // Clamp to trained size range (5–10 mm)
    if (size_mm < 5.0f)  size_mm = 5.0f;
    if (size_mm > 10.0f) size_mm = 10.0f;

    return size_mm;
}

// Convert ADC codes → volts and run the TinyML estimator
void handle_drop_codes(uint16_t base_code, uint16_t min_code)
{
    if (base_code <= min_code)
        return;

    uint16_t delta_code = base_code - min_code;
    float deltaV = (float)delta_code * ADC_CODE_TO_VOLTS;

    if (deltaV < MIN_DV_FOR_EST)
        return;

    float size_mm = estimate_drop_size_mm(deltaV);

    // Convert to integers for minimal printf
    uint16_t delta_mV     = (uint16_t)(deltaV * 1000.0f + 0.5f);   // e.g. 0.95 V -> 950 mV
    uint16_t size_mm_x10  = (uint16_t)(size_mm * 10.0f + 0.5f);    // e.g. 7.2 mm -> 72

    // minimal printf: use %d only (no %u, no %f, no width)
    // Example output: DROP,dV=950 mV,size=7.2 mm
    printf("DROP,dV=%d mV,size=%d.%d mm\r\n",
           (int)delta_mV,
           (int)(size_mm_x10 / 10),
           (int)(size_mm_x10 % 10));
}
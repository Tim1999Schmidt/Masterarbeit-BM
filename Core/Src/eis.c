/**
 ******************************************************************************
 * @file    eis.c
 * @brief   See eis.h.
 ******************************************************************************
 */
#include "eis.h"
#include "tle9015.h"
#include "bm_config.h"

#define R_SHUNT_OHM           0.1f      /* RS_CELL / RS_STACK, 100 mOhm */
#define UOHM_PER_OHM          1.0e6f
#define POWERUP_DELAY_MS      500u      /* TLE9010 start-up (stack chip is supplied only once CONNECT_Z is on) */
#define AVERAGE_PAUSE_MS      200u      /* pause between two averages (Zellhalter firmware) */
#define STOP_TO_READ_DELAY_US 100u      /* after the stop command, before reading the result */
#define HALF_PI_F             1.57079632679490f

/* ---- ZMEAS_CFG register (0x0C): field positions --------------------------
 *   bit 15..14 mux_sel   bit 13 waveform   bit 12..10 measurement_time
 *   bit  9..6  freq div (octave)  bit 5..4 freq fund (fine step)  bit 3..0 settling_time */
#define CFG_MUX_SEL_POS        14u
#define CFG_WAVEFORM_POS       13u
#define CFG_MEAS_TIME_POS      10u
#define CFG_FREQ_DIV_POS        6u
#define CFG_FREQ_FUND_POS       4u
#define CFG_SETTLING_POS        0u
/* fixed field values (Zellhalter firmware) */
#define CFG_MUX_SEL             0u      /* input multiplexer: default */
#define CFG_MEAS_TIME_25MS      0x7u    /* 0b111 = 25 ms measuring time */
#define CFG_SETTLING_NONE       0u      /* no extra settling time */

/* ---- ZMEAS_MAG register (0x0E): |Z| = mantissa * 2^-(MAG_EXP_OFFSET + exponent) * R_shunt */
#define MAG_MANTISSA_SHIFT      4u      /* bits 15..4 = 12-bit mantissa */
#define MAG_EXPONENT_MASK    0x0Fu      /* bits  3..0 = 4-bit exponent */
#define MAG_EXP_OFFSET         10u

/* ---- ZMEAS_PHASE register (0x0F): 16 bit, 65536 steps = one full turn (2 pi) */
#define PHASE_HALF_TURN     32768u      /* = pi */
#define PHASE_QUADRANT_SHIFT   14u      /* bits 15..14 of the angle = quadrant 0..3 */
#define PHASE_QUARTER_MASK 0x3FFFu      /* bits 13..0 = angle inside the quadrant */
#define PHASE_QUARTER_STEPS 16384.0f    /* steps per quadrant (= pi/2) */

/* ZMEAS_AMPLITUDE register code per amplitude index (values from the Zellhalter
 * firmware / Infineon; the code is proportional to the current, ~29.85 per mA) */
static const struct
{
    uint16_t mA;
    uint16_t regCode;
} amplitudeTable[EIS_AMPLITUDE_COUNT] =
{
    {  50u, 0x05D4u },
    { 100u, 0x0BA9u },
    { 200u, 0x1751u },
    { 300u, 0x22FAu },
    { 400u, 0x2EA2u },
    { 500u, 0x3A4Bu },
};

/* Waiting time after the start of a measurement, per frequency range
 * (values from Infineon, Zellhalter firmware): index <= lastIndex -> waitMs */
static const struct
{
    uint8_t  lastIndex;
    uint16_t waitMs;
} waitTable[] =
{
    { 23u,   50u },     /* 7812.5 Hz .. 19.5 Hz */
    { 27u,  100u },
    { 31u,  200u },
    { 35u,  400u },
    { 39u,  700u },
    { 43u, 1200u },
    { 47u, 2300u },
    { EIS_FREQ_INDEX_MAX, 4500u },   /* .. 1.09 Hz */
};

typedef enum { ST_IDLE, ST_POWERUP, ST_FREQ_SETUP, ST_MEAS_START, ST_MEAS_WAIT, ST_PAUSE } EisState_t;

static EisState_t   state = ST_IDLE;
static Eis_Config_t cfg;
static uint8_t      numNodes;
static uint8_t      node;
static uint8_t      freq;            /* current frequency index */
static uint8_t      avg;             /* averages done for the current frequency */
static uint32_t     tick, waitMs;
static float        sumRe, sumIm;
static volatile uint8_t abortRequest;
static Eis_Result_t lastResult = EIS_OK;

/* results of the last completed sweep */
static int32_t      resRe[EIS_NUM_FREQ], resIm[EIS_NUM_FREQ];
static volatile uint8_t resValid;    /* 1 = resFirst..resLast hold a completed sweep */
static uint8_t      resFirst, resLast;

/* ---- helpers --------------------------------------------------------------- */
uint32_t EIS_FreqCentiHz(uint8_t index)
{
    const uint32_t fund = index % EIS_FREQ_STEPS_PER_OCTAVE;
    const uint32_t div  = index / EIS_FREQ_STEPS_PER_OCTAVE;
    /* f_max / (1 + fund/4) / 2^div = 4 * f_max / (4 + fund) / 2^div [0.01 Hz], rounded:
     * computed with one extra bit (2x) and halved at the end */
    return (((2u * EIS_FREQ_STEPS_PER_OCTAVE * EIS_F_MAX_CENTI_HZ) / (EIS_FREQ_STEPS_PER_OCTAVE + fund) >> div) + 1u) / 2u;
}

static uint32_t WaitTimeMs(uint8_t index)
{
    uint8_t i = 0u;
    while (index > waitTable[i].lastIndex)
        i++;
    return waitTable[i].waitMs;
}

static uint16_t CfgRegister(uint8_t index)
{
    return (uint16_t)((CFG_MUX_SEL                                         << CFG_MUX_SEL_POS)   |
                      ((uint32_t)(cfg.waveform & 1u)                      << CFG_WAVEFORM_POS)  |
                      (CFG_MEAS_TIME_25MS                                 << CFG_MEAS_TIME_POS) |
                      ((uint32_t)(index / EIS_FREQ_STEPS_PER_OCTAVE)      << CFG_FREQ_DIV_POS)  |
                      ((uint32_t)(index % EIS_FREQ_STEPS_PER_OCTAVE)      << CFG_FREQ_FUND_POS) |
                      (CFG_SETTLING_NONE                                  << CFG_SETTLING_POS));
}

/* sin and cos of x in [0, pi/2): Taylor series up to x^11 / x^12 (truncation
 * error < 6e-8, below float resolution). Own implementation instead of
 * sinf()/cosf() from the C library: those need ~2.5 KB flash for the general
 * argument reduction, which is not needed here - the quadrant comes exactly
 * from the integer phase register. */
static void SinCosFirstQuadrant(float x, float *s, float *c)
{
    const float x2 = x * x;
    *s = x * (1.0f + x2 * (-1.0f / 6.0f + x2 * (1.0f / 120.0f + x2 * (-1.0f / 5040.0f +
             x2 * (1.0f / 362880.0f + x2 * (-1.0f / 39916800.0f))))));
    *c = 1.0f + x2 * (-0.5f + x2 * (1.0f / 24.0f + x2 * (-1.0f / 720.0f + x2 * (1.0f / 40320.0f +
             x2 * (-1.0f / 3628800.0f + x2 * (1.0f / 479001600.0f))))));
}

void EIS_RawToReIm(uint16_t magRaw, uint16_t phaseRaw, float *reUOhm, float *imUOhm)
{
    const uint32_t mant = magRaw >> MAG_MANTISSA_SHIFT;
    const uint32_t ex   = magRaw & MAG_EXPONENT_MASK;
    /* |Z| = mant * 2^-(10+exp) * R_shunt, here directly in uOhm */
    const float mag = (float)mant * (R_SHUNT_OHM * UOHM_PER_OHM) / (float)(1ul << (MAG_EXP_OFFSET + ex));

    /* phi = phase / 32768 * pi + pi. In units of 2*pi/65536 the "+ pi" is + 32768,
     * so the angle stays an exact 16-bit integer: the top 2 bits are the quadrant,
     * the lower 14 bits the angle inside it. */
    const uint16_t a = (uint16_t)(phaseRaw + PHASE_HALF_TURN);
    const float    x = (float)(a & PHASE_QUARTER_MASK) * (HALF_PI_F / PHASE_QUARTER_STEPS);
    float s, c, cosPhi, sinPhi;

    SinCosFirstQuadrant(x, &s, &c);
    switch (a >> PHASE_QUADRANT_SHIFT)
    {
    case 0:  cosPhi =  c; sinPhi =  s; break;
    case 1:  cosPhi = -s; sinPhi =  c; break;
    case 2:  cosPhi = -c; sinPhi = -s; break;
    default: cosPhi =  s; sinPhi = -c; break;
    }
    *reUOhm = mag * cosPhi;
    *imUOhm = mag * sinPhi;
}

static int32_t RoundToInt(float v)
{
    return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/* stop the running measurement and put the chain back to sleep (farthest node first) */
static void Finish(Eis_Result_t result)
{
    uint8_t n;
    Tle_Write(node, TLE_REG_ZMEAS_CTRL, TLE_ZMEAS_CTRL_STOP);
    for (n = numNodes; n >= 1u; n--)
        Tle_Write(n, TLE_REG_GOTO_SLEEP, TLE_SLEEP_TIMER_SLEEP_NOW);

    if (result == EIS_OK)
        resValid = 1u;
    lastResult = result;
    state = ST_IDLE;
}

/* ---- public API ---------------------------------------------------------------- */
void EIS_Init(void)
{
    state = ST_IDLE;
    resValid = 0u;
}

Eis_Result_t EIS_Start(const Eis_Config_t *c, uint8_t numEis)
{
    if (state != ST_IDLE)
        return EIS_ERR_BUSY;
    if (c->chip > EIS_CHIP_STACK || (c->chip == EIS_CHIP_STACK && numEis < BM_NUM_EIS_WITH_STACK) ||
        c->fStart >= EIS_NUM_FREQ || c->fStop >= EIS_NUM_FREQ || c->fStop < c->fStart ||
        c->amplitude >= EIS_AMPLITUDE_COUNT || c->waveform > EIS_WAVE_RECTANGLE ||
        c->averages < EIS_AVERAGES_MIN || c->averages > EIS_AVERAGES_MAX)
        return EIS_ERR_CONFIG;

    cfg      = *c;
    numNodes = (numEis >= BM_NUM_EIS_WITH_STACK) ? BM_NUM_EIS_WITH_STACK : BM_NUM_EIS_CELL_ONLY;
    node     = (cfg.chip == EIS_CHIP_STACK) ? TLE_NODE_STACK : TLE_NODE_CELL;
    freq     = cfg.fStart;
    abortRequest = 0u;

    resValid = 0u;                       /* old results become invalid */
    resFirst = cfg.fStart;
    resLast  = cfg.fStop;

    tick  = HAL_GetTick();
    state = ST_POWERUP;
    lastResult = EIS_OK;
    return EIS_OK;
}

void EIS_Abort(void)
{
    if (state != ST_IDLE)
        abortRequest = 1u;
}

void EIS_Process(void)
{
    uint16_t mag, phase;
    float re, im;
    uint8_t n;

    if (state == ST_IDLE)
        return;

    if (abortRequest)
    {
        abortRequest = 0u;
        Finish(EIS_ERR_ABORTED);
        return;
    }

    switch (state)
    {
    case ST_POWERUP:
        if ((HAL_GetTick() - tick) < POWERUP_DELAY_MS)
            break;
        Tle_Wakeup();
        Tle_Enumerate(numNodes);
        for (n = 1u; n <= numNodes; n++)
            Tle_Write(n, TLE_REG_GOTO_SLEEP, TLE_SLEEP_TIMER_STAY_AWAKE);
        state = ST_FREQ_SETUP;
        break;

    case ST_FREQ_SETUP:
        Tle_Write(node, TLE_REG_ZMEAS_CFG, CfgRegister(freq));
        Tle_Write(node, TLE_REG_ZMEAS_AMP, amplitudeTable[cfg.amplitude].regCode);
        avg = 0u;
        sumRe = sumIm = 0.0f;
        waitMs = WaitTimeMs(freq);
        state = ST_MEAS_START;
        break;

    case ST_MEAS_START:
        Tle_Write(node, TLE_REG_ZMEAS_CTRL, TLE_ZMEAS_CTRL_START);
        tick = HAL_GetTick();
        state = ST_MEAS_WAIT;
        break;

    case ST_MEAS_WAIT:
        if ((HAL_GetTick() - tick) < waitMs)
            break;
        Tle_Write(node, TLE_REG_ZMEAS_CTRL, TLE_ZMEAS_CTRL_STOP);
        Tle_DelayUs(STOP_TO_READ_DELAY_US);
        if (!Tle_Read(node, TLE_REG_ZMEAS_MAG, &mag) || !Tle_Read(node, TLE_REG_ZMEAS_PHASE, &phase))
        {
            Finish(EIS_ERR_COMM);
            break;
        }
        EIS_RawToReIm(mag, phase, &re, &im);
        sumRe += re;
        sumIm += im;
        avg++;
        tick = HAL_GetTick();
        state = ST_PAUSE;
        break;

    case ST_PAUSE:
        if ((HAL_GetTick() - tick) < AVERAGE_PAUSE_MS)
            break;
        if (avg < cfg.averages)
        {
            state = ST_MEAS_START;
            break;
        }
        resRe[freq] = RoundToInt(sumRe / (float)cfg.averages);
        resIm[freq] = RoundToInt(sumIm / (float)cfg.averages);
        if (freq >= cfg.fStop)
        {
            Finish(EIS_OK);
            break;
        }
        freq++;
        state = ST_FREQ_SETUP;
        break;

    default:
        state = ST_IDLE;
        break;
    }
}

uint8_t EIS_IsBusy(void)                   { return (state != ST_IDLE) ? 1u : 0u; }
uint8_t EIS_HasResults(void)               { return resValid; }
uint8_t EIS_GetProgress(void)              { return (state != ST_IDLE) ? freq : EIS_PROGRESS_IDLE; }
Eis_Result_t EIS_GetLastResult(void)       { return lastResult; }
const Eis_Config_t *EIS_GetLastConfig(void){ return &cfg; }

uint8_t EIS_GetResult(uint8_t index, int32_t *reUOhm, int32_t *imUOhm)
{
    if (!resValid || index < resFirst || index > resLast)
        return 0u;
    *reUOhm = resRe[index];
    *imUOhm = resIm[index];
    return 1u;
}

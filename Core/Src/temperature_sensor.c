/**
 ******************************************************************************
 * @file    temperature_sensor.c
 * @brief   See temperature_sensor.h.
 *
 * Note on the measuring range: the front-end maps ~ -5 degC -> 0 V and ~54 degC
 * -> 3.3 V. Outside that the output saturates (and a broken PT1000 lead makes
 * the constant-current source drive the output to the top rail, i.e. it looks
 * like "54 degC"). Such readings are flagged by a
 * plausibility check on the raw counts (TEMP_FAULT_LOW/HIGH_COUNTS), reported
 * through TempSensor_GetFault() (STATUS / error code towards the MM).
 ******************************************************************************
 */
#include "temperature_sensor.h"
#include "adc_reader.h"
#include <stddef.h>

/* ---- Front-end description (BM = PM board) ----------------------------- */
#define ADC_MAX_COUNTS            4095.0f
#define R_CAL_LOW_OHM             1000.0f   /* CH3 / SEL3: bottom-of-range reference resistor */
#define R_CAL_HIGH_OHM            1200.0f   /* CH2 / SEL2: top-of-range reference resistor    */
#define R_PT1000_AT_0C            1000.0f

/* Nominal analog front-end (see temperature_sensor.h). Only used for the
 * counts -> resistance mapping until the first calibration has succeeded;
 * afterwards the 2-point calibration replaces all of these values. */
#define FE_I_SOURCE_A             100.0e-6f   /* constant-current source (tolerance ~ +-1 %) */
#define FE_VREF_V                   0.098f    /* reference potential of the bridge (tolerance ~ +-0.3 mV) */
#define FE_GAIN                   142.2f      /* op-amp gain (nominal) */
#define FE_VDDA_V                   3.3f      /* ADC reference (full scale) */
#define FE_NOMINAL_COUNTS(r)      ((((r) * FE_I_SOURCE_A) - FE_VREF_V) * FE_GAIN / FE_VDDA_V * ADC_MAX_COUNTS)

/* ---- Tunables ----------------------------------------------------------- */
#define TEMP_SAMPLE_INTERVAL_MS   100u     /* fixed measurement interval */
#define TEMP_OVERSAMPLE            16u     /* ADC conversions averaged per reading (one short burst) */
#define CAL_OVERSAMPLE             64u     /* ADC conversions averaged per calibration resistor      */
#define CAL_MIN_SPAN_COUNTS      1000.0f   /* plausibility: counts(1.2k) - counts(1k) must be at least this (nominal ~3530) */

/* PT1000 plausibility: a reading this close to either ADC rail means the
 * front-end is saturated (temperature outside ~-5..54 degC) or the sensor lead is
 * broken / shorted. Must be seen on TEMP_FAULT_DEBOUNCE consecutive readings
 * (= 300 ms) before it counts as a fault, so a single noisy reading is not reported. */
#define TEMP_FAULT_LOW_COUNTS       5.0f
#define TEMP_FAULT_HIGH_COUNTS   4090.0f
#define TEMP_FAULT_DEBOUNCE         3u

/* ReadAveragedCounts() drops the highest and the lowest sample, so it needs at least 3.
 * (Preprocessor check instead of C11 _Static_assert: the CubeIDE editor's parser flags
 * _Static_assert as a syntax error, while gcc accepts both.) */
#if (TEMP_OVERSAMPLE < 3u) || (CAL_OVERSAMPLE < 3u)
#error "TEMP_OVERSAMPLE and CAL_OVERSAMPLE must be at least 3 (ReadAveragedCounts drops min and max)"
#endif

/* Piecewise-linear resistance/temperature table, carried over verbatim from
 * temperature_driver_sw (values calculated from the datasheet formula for
 * NB-PTCO-164 PTFC102A1A0; these are samples of the standard PT1000 curve,
 * IEC 60751). Against that formula the table stays within ~0.007 K over
 * the measuring range (checked) - far below one ADC step (~0.015 K). */
#define PWL_SIZE 10u
static const float pwlResistanceOhm[PWL_SIZE] = { 900.0f, 950.0f, 1000.0f, 1050.0f, 1100.0f, 1150.0f, 1200.0f, 1250.0f, 1300.0f, 1350.0f };
static const float pwlTemperatureC[PWL_SIZE]  = { -25.488f, -12.7685f, 0.0f, 12.818f, 25.6845f, 38.6005f, 51.5665f, 64.583f, 77.651f, 90.7705f };
static float pwlSlope[PWL_SIZE - 1u];   /* degC per Ohm of each table segment */
static const float linpolSlopeFallback = 0.2593035671f; /* used only outside the table (-25..91 degC), where the front-end cannot measure anyway */

/* ---- SEL multiplexer ---------------------------------------------------- */
typedef enum
{
    MUX_PT1000 = 0,   /* CH1 / SEL1 */
    MUX_CAL_HIGH,     /* CH2 / SEL2, 1.20 kOhm */
    MUX_CAL_LOW       /* CH3 / SEL3, 1.00 kOhm */
} MuxChannel_t;

static void SelectChannel(MuxChannel_t channel)
{
    /* Break-before-make: switch everything off first, so two resistors are
     * never connected to the current source at the same time. */
    HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEL2_GPIO_Port, SEL2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEL3_GPIO_Port, SEL3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEL4_GPIO_Port, SEL4_Pin, GPIO_PIN_RESET); /* unused on this board revision */

    switch (channel)
    {
    case MUX_PT1000:   HAL_GPIO_WritePin(SEL1_GPIO_Port, SEL1_Pin, GPIO_PIN_SET); break;
    case MUX_CAL_HIGH: HAL_GPIO_WritePin(SEL2_GPIO_Port, SEL2_Pin, GPIO_PIN_SET); break;
    case MUX_CAL_LOW:  HAL_GPIO_WritePin(SEL3_GPIO_Port, SEL3_Pin, GPIO_PIN_SET); break;
    default: break;
    }
}

/* Mean of n consecutive ADC conversions of the TEMP channel. The single
 * highest and lowest value are dropped as outliers (same idea as in
 * current_sensor.c). */
static float ReadAveragedCounts(uint32_t n)
{
    uint32_t sum  = 0u;
    uint32_t minV = UINT32_MAX;
    uint32_t maxV = 0u;
    uint32_t i;

    for (i = 0u; i < n; i++)
    {
        const uint32_t v = AdcReader_ReadBlocking(ADC_READER_CH_TEMP);
        sum += v;
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
    }

    return (float)(sum - minV - maxV) / (float)(n - 2u);
}

/* ---- State ---------------------------------------------------------------- */
typedef enum
{
    TS_CAL_MEASURE_LOW = 0, /* 1.00 kOhm reference is connected: measure it, connect the 1.20 kOhm one */
    TS_CAL_MEASURE_HIGH,    /* 1.20 kOhm reference is connected: measure it, calibrate, connect the PT1000 */
    TS_RUN                  /* normal operation: PT1000 only */
} TempState_t;

static TempState_t state    = TS_CAL_MEASURE_LOW;
static uint32_t    lastTick = 0u;

static uint8_t recalRequested   = 0u;
static uint8_t calibrationOk    = 0u;   /* result of the last COMPLETED calibration (not cleared while a new one runs) */
static uint8_t measurementValid = 0u;   /* a PT1000 value exists (held during a recalibration) */
static float   lastPtCounts     = 0.0f; /* raw counts of the last PT1000 reading */

static uint8_t pt1000FaultCount = 0u;
static uint8_t pt1000Fault      = 0u;

/* Raw counts measured on the two calibration resistors (diagnostics). */
static float calCountsLow  = 0.0f;
static float calCountsHigh = 0.0f;

/* Active counts -> resistance mapping: two known points on a straight line. */
static float mapCountsLow   = 0.0f;   /* set to the nominal front-end in TempSensor_Init() */
static float mapResLow      = R_CAL_LOW_OHM;
static float mapOhmPerCount = 0.0f;

static float resistanceOhm    = R_PT1000_AT_0C;
static float temperatureRawC  = 0.0f;
static float temperatureC     = 0.0f;
static float trimOffsetC      = 0.0f;
static float trimSlope        = 0.0f;


static void SetMapping(float countsLow, float countsHigh, float resLowOhm, float resHighOhm)
{
    mapCountsLow   = countsLow;
    mapResLow      = resLowOhm;
    mapOhmPerCount = (resHighOhm - resLowOhm) / (countsHigh - countsLow);
}

static float ResistanceToTemperature(float resistanceOhmValue)
{
    uint8_t i = 0;

    while (i < PWL_SIZE && resistanceOhmValue > pwlResistanceOhm[i])
        i++;

    if (i > 0 && i < PWL_SIZE)
    {
        i--;
        /* PWL interpolation: T(R) = Tlower + (R - Rlower) * slope, see AN0970 (Analog Devices) */
        return pwlTemperatureC[i] + (resistanceOhmValue - pwlResistanceOhm[i]) * pwlSlope[i];
    }

    /* Outside the characterised range: fall back to a simple linear approximation around 0 degC */
    return (resistanceOhmValue - R_PT1000_AT_0C) * linpolSlopeFallback;
}

/* counts -> resistance -> temperature (+ trim) with the active mapping */
static float CountsToResistance(float counts)
{
    return mapResLow + (counts - mapCountsLow) * mapOhmPerCount;
}

static float ResistanceToTrimmedTemperature(float resistanceOhmValue, float *rawOut)
{
    const float raw = ResistanceToTemperature(resistanceOhmValue);
    if (rawOut) *rawOut = raw;
    return raw - trimOffsetC - (raw * trimSlope);
}

/* Connects the 1.00 kOhm reference; the state machine takes it from there.
 * The last PT1000 value (and the result of the previous calibration) stay
 * valid meanwhile, so a recalibration only pauses the measurement for ~300 ms. */
static void BeginCalibration(void)
{
    pt1000FaultCount = 0u;
    pt1000Fault      = 0u;
    SelectChannel(MUX_CAL_LOW);
    state = TS_CAL_MEASURE_LOW;
}

static void FinishCalibration(void)
{
    /* Both references must lie inside the ADC range (a reading at a rail means the
     * front-end saturates there - the calibration would then be silently wrong)
     * and be far enough apart. */
    if (((calCountsHigh - calCountsLow) >= CAL_MIN_SPAN_COUNTS) &&
        (calCountsLow  > TEMP_FAULT_LOW_COUNTS)  && (calCountsLow  < TEMP_FAULT_HIGH_COUNTS) &&
        (calCountsHigh > TEMP_FAULT_LOW_COUNTS)  && (calCountsHigh < TEMP_FAULT_HIGH_COUNTS))
    {
        SetMapping(calCountsLow, calCountsHigh, R_CAL_LOW_OHM, R_CAL_HIGH_OHM);
        calibrationOk = 1u;
    }
    else
    {
        /* Reference readings implausible (resistors not connected / SEL lines not
         * wired / front-end dead or saturated): keep the previous mapping, but do
         * not report ready - reported as TEMP_FAULT_CALIBRATION. */
        calibrationOk = 0u;
    }
}

/* One PT1000 reading: counts -> resistance -> temperature (+ trim). */
static void MeasurePt1000(void)
{
    const float counts = ReadAveragedCounts(TEMP_OVERSAMPLE);

    lastPtCounts    = counts;
    resistanceOhm   = CountsToResistance(counts);
    temperatureC    = ResistanceToTrimmedTemperature(resistanceOhm, &temperatureRawC);
    measurementValid = 1u;

    /* Plausibility check on the raw counts (see file header) */
    if ((counts < TEMP_FAULT_LOW_COUNTS) || (counts > TEMP_FAULT_HIGH_COUNTS))
    {
        if (pt1000FaultCount < TEMP_FAULT_DEBOUNCE)
            pt1000FaultCount++;
        if (pt1000FaultCount >= TEMP_FAULT_DEBOUNCE)
            pt1000Fault = 1u;
    }
    else
    {
        pt1000FaultCount = 0u;
        pt1000Fault      = 0u;
    }
}

/* ---- Public API ---------------------------------------------------------- */
void TempSensor_Init(void)
{
    uint8_t i;

    /* Precalculate the PWL segment slopes */
    for (i = 0; i < PWL_SIZE - 1u; i++)
        pwlSlope[i] = (pwlTemperatureC[i + 1] - pwlTemperatureC[i]) / (pwlResistanceOhm[i + 1] - pwlResistanceOhm[i]);

    SetMapping(FE_NOMINAL_COUNTS(R_CAL_LOW_OHM), FE_NOMINAL_COUNTS(R_CAL_HIGH_OHM), R_CAL_LOW_OHM, R_CAL_HIGH_OHM);
    recalRequested = 0u;

    BeginCalibration();          /* calibrate first, then measure */
    lastTick = HAL_GetTick();
}

void TempSensor_Process(void)
{
    const uint32_t now = HAL_GetTick();

    if ((now - lastTick) < TEMP_SAMPLE_INTERVAL_MS)
        return;
    lastTick = now;

    switch (state)
    {
    case TS_CAL_MEASURE_LOW:
        calCountsLow = ReadAveragedCounts(CAL_OVERSAMPLE);
        SelectChannel(MUX_CAL_HIGH);
        state = TS_CAL_MEASURE_HIGH;
        break;

    case TS_CAL_MEASURE_HIGH:
        calCountsHigh = ReadAveragedCounts(CAL_OVERSAMPLE);
        FinishCalibration();
        SelectChannel(MUX_PT1000);
        state = TS_RUN;
        break;

    case TS_RUN:
        if (recalRequested)
        {
            recalRequested = 0u;
            BeginCalibration();
        }
        else
        {
            MeasurePt1000();
        }
        break;

    default:
        BeginCalibration();
        break;
    }
}

void TempSensor_Recalibrate(void)
{
    recalRequested = 1u;
}

uint8_t TempSensor_IsReady(void)
{
    return (calibrationOk && measurementValid && !pt1000Fault) ? 1u : 0u;
}

TempSensor_Fault_t TempSensor_GetFault(void)
{
    if (pt1000Fault)
        return TEMP_FAULT_PT1000_RANGE;

    /* last calibration failed (and no new one is pending / running) */
    if ((state == TS_RUN) && !recalRequested && !calibrationOk)
        return TEMP_FAULT_CALIBRATION;

    return TEMP_FAULT_NONE;
}

float TempSensor_GetTemperature(void)
{
    return temperatureC;
}

float TempSensor_GetResistance(void)
{
    return resistanceOhm;
}

void TempSensor_SetTrim(float offsetDegC, float slope)
{
    trimOffsetC = offsetDegC;
    trimSlope   = slope;
}

uint16_t TempSensor_GetCalCountsLow(void)
{
    return (uint16_t)(calCountsLow + 0.5f);
}

uint16_t TempSensor_GetCalCountsHigh(void)
{
    return (uint16_t)(calCountsHigh + 0.5f);
}

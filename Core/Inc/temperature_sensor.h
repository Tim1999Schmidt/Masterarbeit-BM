/**
 ******************************************************************************
 * @file    temperature_sensor.h
 * @brief   PT1000 tab temperature measurement of the BM board (same front-end
 *          and code as on the PM board).
 *
 * Hardware (BM board, identical to the PM board):
 *   A constant-current source drives 100 uA through ONE of three resistors,
 *   selected by the SEL lines. An op-amp amplifies the difference between that
 *   voltage drop and a reference potential of ~98 mV by ~142.2:
 *
 *      Vadc = 142.2 * (R * 100 uA - 98 mV)
 *
 *   read by ADC1 channel 1 (TEMP, PA1). Nominal values (VDDA = 3.3 V):
 *
 *      R = ~980 Ohm  (~ -5 degC) ->  0 V      (lower end of the range)
 *      R = 1000 Ohm  (   0 degC) ->  ~0.28 V  (~353 counts)
 *      R = 1194 Ohm  (  50 degC) ->  ~3.04 V  (~3776 counts)
 *      R = 1200 Ohm  (~51 degC)  ->  ~3.13 V  (~3882 counts)
 *      R = ~1212 Ohm (~55 degC)  ->  3.3 V    (upper end of the range)
 *
 *   Tolerances (current source +-1 %, reference +-0.3 mV, gain) shift and
 *   scale this line; the 2-point calibration removes both.
 *
 *      CH1 / SEL1 : PT1000 sensor      - the only channel used in normal operation
 *      CH2 / SEL2 : 1.20 kOhm resistor - calibration only (top of the range)
 *      CH3 / SEL3 : 1.00 kOhm resistor - calibration only (bottom of the range)
 *      SEL4       : unused on this board revision (kept low)
 *
 * Measurement: one reading every TEMP_SAMPLE_INTERVAL_MS (100 ms, fixed).
 * Unlike the original RC-charging method there is nothing to wait for - no
 * settling time, no filter time, no temporal filter. A reading is a short burst
 * of ADC conversions on the already-selected CH1, averaged against ADC noise.
 *
 * Calibration: the two precision resistors sit near the two ends of the
 * measuring range (both inside it, not at the ADC rails), so measuring them
 * gives a 2-point calibration of the whole analog chain (current source,
 * reference potential, amplifier gain, ADC offset/gain):
 *
 *      R = 1000 Ohm + (counts - counts@1k) * (1200 - 1000) Ohm / (counts@1.2k - counts@1k)
 *
 * The PT1000 resistance is then converted to degC with a piecewise-linear
 * table in 10 K steps (values from the PT1000 sensor data sheet, see
 * temperature_sensor.c). The calibration runs on the
 * same fixed 100 ms clock, right after TempSensor_Init() and on request
 * (TempSensor_Recalibrate(), e.g. by the MM via I2C before a measurement
 * series, to remove drift of the front-end):
 *
 *      tick 0: connect the 1.00 kOhm reference (done in Init / on request)
 *      tick 1: measure it,            connect the 1.20 kOhm reference
 *      tick 2: measure it, calibrate, connect the PT1000 again
 *      tick 3: first PT1000 reading -> TempSensor_IsReady() = 1
 *
 * So a calibration takes ~300 ms. During a REcalibration the last PT1000 value
 * stays valid (held). (The one interval between connecting a
 * reference and measuring it is just a consequence of the fixed clock, not a
 * separate settling timer.)
 *
 * Measuring range: ~ -5 ... ~55 degC. Beyond
 * that the front-end output saturates at 0 V / 3.3 V and the reading is no
 * longer valid (see the note in temperature_sensor.c).
 *
 * Cooperative: call TempSensor_Process() once per main loop iteration. A tick
 * blocks for the ADC burst only (~1-2 ms; ~5-8 ms for a calibration reading).
 ******************************************************************************
 */
#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Problems with the tab temperature measurement. */
typedef enum
{
    TEMP_FAULT_NONE = 0,
    TEMP_FAULT_PT1000_RANGE,   /* PT1000 reading at an ADC rail (outside ~-5..55 degC, broken/shorted lead) */
    TEMP_FAULT_CALIBRATION     /* the last 2-point calibration failed (reference readings implausible / saturated) */
} TempSensor_Fault_t;

/* One-time setup; starts the calibration sequence. Call from main() before the main loop. */
void TempSensor_Init(void);

/* Fixed-interval measurement / calibration state machine. Call every main loop iteration. */
void TempSensor_Process(void);

/* Requests a new 2-point calibration (starts at the next 100 ms tick, takes
 * ~300 ms, during which the last temperature value is held). */
void TempSensor_Recalibrate(void);

/* 1 once the temperature is valid: the last calibration succeeded, a first
 * PT1000 value exists and it is plausible. */
uint8_t TempSensor_IsReady(void);

/* Current fault of the measurement (see TempSensor_Fault_t). */
TempSensor_Fault_t TempSensor_GetFault(void);

/* PT1000 results. */
float TempSensor_GetTemperature(void);   /* degC, including the optional trim */
float TempSensor_GetResistance(void);    /* Ohm */

/* Optional final trim against a reference thermometer (e.g. to remove the
 * PT1000's own tolerance): T = Traw - offsetDegC - Traw * slope. Default 0 / 0.
 * (Not to be confused with the automatic resistor calibration above.) */
void TempSensor_SetTrim(float offsetDegC, float slope);

/* Raw ADC counts (0..4095) measured on the two calibration resistors during
 * the last calibration - for bring-up diagnostics. Expected with the nominal
 * front-end (see above): about 360 and about 3910. */
uint16_t TempSensor_GetCalCountsLow(void);   /* 1.00 kOhm reference (CH3) */
uint16_t TempSensor_GetCalCountsHigh(void);  /* 1.20 kOhm reference (CH2) */

#ifdef __cplusplus
}
#endif

#endif /* TEMPERATURE_SENSOR_H */

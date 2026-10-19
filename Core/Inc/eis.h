/**
 ******************************************************************************
 * @file    eis.h
 * @brief   EIS frequency sweep with a TLE9010EP (cell or stack) as a
 *          non-blocking state machine, results as real/imaginary part in uOhm.
 *
 * Measurement procedure, frequency table, waiting times and register contents
 * are those of the Zellhalter firmware (EIS_Messung / Sweep_Messung):
 *   wake-up, enumeration, sleep timer off -> per frequency: ZMEAS_CFG,
 *   ZMEAS_AMPLITUDE -> per average: start, wait (50 ms .. 4.5 s depending on the
 *   frequency), stop, read magnitude + phase, accumulate, 200 ms pause ->
 *   chain back to sleep.
 * The sweep no longer blocks: EIS_Process() advances it from the main loop, so
 * the I2C slave, the console and the other measurements keep running.
 *
 * Frequency index 0..51: f = 7812.5 Hz / (1 + fund/4) / 2^div with
 * fund = index % 4, div = index / 4  (7812.5 Hz ... 1.09 Hz).
 *
 * Result per frequency (TLE9010 magnitude register = 12-bit mantissa + 4-bit
 * exponent, phase register = 16 bit):
 *   |Z| = mant * 2^-(10 + exp) * R_shunt        (R_shunt = 100 mOhm)
 *   phi = phase / 32768 * pi + pi                (as in the Zellhalter firmware)
 *   Re = |Z| cos(phi), Im = |Z| sin(phi), averaged, rounded to 1 uOhm.
 * Calculated in float, sin/cos with an own polynomial (the quadrant comes
 * exactly from the integer phase register; saves ~2.5 KB flash compared with
 * sinf/cosf). Checked against double for all 65536 phase values x all 16
 * exponents at full mantissa and 6.3 million mixed combinations: max. 0.1 uOhm
 * (at |Z| = 0.4 Ohm, the largest value the register can hold) and max.
 * 0.00001 deg; plus 0.5 uOhm from rounding to whole uOhm. Required were
 * <= 10 uOhm and <= 0.05 deg.
 ******************************************************************************
 */
#ifndef EIS_H
#define EIS_H

#include "main.h"
#include <stdint.h>

#define EIS_NUM_FREQ      52u      /* frequency index 0..51 */
#define EIS_FREQ_INDEX_MAX        (EIS_NUM_FREQ - 1u)
#define EIS_FREQ_STEPS_PER_OCTAVE 4u  /* index % 4 = fund (fine step), index / 4 = div (octave) */
#define EIS_F_MAX_CENTI_HZ        781250u   /* index 0 = 7812.50 Hz */

#define EIS_AMPLITUDE_COUNT   6u   /* amplitude index 0..5 = 50, 100, 200, 300, 400, 500 mA */
#define EIS_AVERAGES_MIN      1u
#define EIS_AVERAGES_MAX      9u
#define EIS_PROGRESS_IDLE     0xFFu  /* EIS_GetProgress() while no sweep runs */

typedef enum
{
    EIS_CHIP_CELL  = 0,
    EIS_CHIP_STACK = 1
} Eis_Chip_t;

typedef enum
{
    EIS_WAVE_SINE      = 0,
    EIS_WAVE_RECTANGLE = 1
} Eis_Waveform_t;

typedef struct
{
    uint8_t chip;        /* Eis_Chip_t */
    uint8_t fStart;      /* frequency index 0..51 (0 = 7812.5 Hz) */
    uint8_t fStop;       /* frequency index fStart..51 */
    uint8_t amplitude;   /* 0..EIS_AMPLITUDE_COUNT-1 = 50, 100, 200, 300, 400, 500 mA */
    uint8_t waveform;    /* Eis_Waveform_t */
    uint8_t averages;    /* EIS_AVERAGES_MIN..EIS_AVERAGES_MAX */
} Eis_Config_t;

typedef enum
{
    EIS_OK          = 0,
    EIS_ERR_CONFIG  = 1, /* parameter out of range / stack chip not present */
    EIS_ERR_BUSY    = 2, /* a sweep is already running */
    EIS_ERR_COMM    = 3, /* TLE9010 did not answer a read (after 3 attempts) */
    EIS_ERR_ABORTED = 4  /* aborted by command */
} Eis_Result_t;

void EIS_Init(void);
void EIS_Process(void);

/* Starts a sweep (checks the parameters; numEis = number of TLE9010 on this board). */
Eis_Result_t EIS_Start(const Eis_Config_t *cfg, uint8_t numEis);
void EIS_Abort(void);

uint8_t      EIS_IsBusy(void);
uint8_t      EIS_HasResults(void);        /* 1 = the last sweep completed, results available */
uint8_t      EIS_GetProgress(void);       /* frequency index being measured, EIS_PROGRESS_IDLE if idle */
Eis_Result_t EIS_GetLastResult(void);     /* outcome of the last sweep */
const Eis_Config_t *EIS_GetLastConfig(void);

/* Frequency of an index in 0.01 Hz (781250 = 7812.50 Hz). */
uint32_t EIS_FreqCentiHz(uint8_t index);

/* Result of a frequency of the last completed sweep. Returns 0 if that index
 * was not part of it (or no sweep completed yet). Interrupt-safe (I2C ISR). */
uint8_t EIS_GetResult(uint8_t index, int32_t *reUOhm, int32_t *imUOhm);

/* Converts the two raw TLE9010 registers into Re/Im in uOhm (float). Exposed
 * for tests; used internally for every single measurement. */
void EIS_RawToReIm(uint16_t magRaw, uint16_t phaseRaw, float *reUOhm, float *imUOhm);

#endif /* EIS_H */

/**
 ******************************************************************************
 * @file    bm_config.h
 * @brief   Per-board settings of the BM, kept in the last flash page, plus the
 *          build-time switches.
 *
 * Every BM runs the same firmware. What differs between the n boards is
 * stored in flash and set once over the debug console (uart_comm.h):
 *
 *   i2cAddr   7-bit I2C slave address towards the MM (unique per board)
 *   numEis    1 = only the cell TLE9010, 2 = cell + stack TLE9010 (first BM)
 *   iZeroMv   ADC voltage of the current path at 0 A (current zero calibration)
 *
 * The page used is the last 2 KB page of the 32 KB flash (page 15,
 * 0x08007800..0x08007FFF). It is excluded from the program in the linker
 * script (STM32C051K6TX_FLASH.ld: FLASH LENGTH = 30K), so the code can never
 * grow into it.
 ******************************************************************************
 */
#ifndef BM_CONFIG_H
#define BM_CONFIG_H

#include "main.h"
#include <stdint.h>

/* ---- build-time switches (can also be set as -D symbols in the IDE) ---- */
#ifndef BM_ENABLE_UART
#define BM_ENABLE_UART  1      /* debug console on USART1 / FT231X (USB) */
#endif

/* ---- allowed values ---- */
#define BM_I2C_ADDR_MIN       0x08u    /* 7-bit addresses 0x00..0x07 and 0x78..0x7F are reserved by the I2C standard */
#define BM_I2C_ADDR_MAX       0x77u
#define BM_NUM_EIS_CELL_ONLY  1u       /* only the cell TLE9010 */
#define BM_NUM_EIS_WITH_STACK 2u       /* cell + stack TLE9010 (first BM) */

/* ---- defaults for an unprogrammed board ---- */
#define BM_DEFAULT_I2C_ADDR   0x30u
#define BM_DEFAULT_NUM_EIS    BM_NUM_EIS_CELL_ONLY
#define BM_DEFAULT_I_ZERO_MV  1650     /* nominal current zero: REF1 = 1.65 V = VDD/2 */
#define BM_I_ZERO_MIN_MV      1000     /* plausible range of a stored zero; outside -> default */
#define BM_I_ZERO_MAX_MV      2300
typedef struct
{
    uint8_t i2cAddr;      /* BM_I2C_ADDR_MIN..BM_I2C_ADDR_MAX */
    uint8_t numEis;       /* BM_NUM_EIS_CELL_ONLY or BM_NUM_EIS_WITH_STACK */
    int16_t iZeroMv;      /* mV at the ADC for 0 A (default 1650 until "izero" is stored) */
} BmConfig_t;

/* Loads the settings from flash (defaults if the page is empty / invalid). */
void BmConfig_Load(void);

/* Current settings (RAM copy). Change fields, then call BmConfig_Save(). */
BmConfig_t *BmConfig_Get(void);

/* Writes the RAM copy to flash (erase + program, ~25 ms, CPU stalls meanwhile).
 * Does nothing if the flash already holds exactly these values, so repeated
 * calls do not wear the flash (STM32C0: >= 10000 erase cycles per page).
 * Only the console commands addr / neis / izero call it - never the normal
 * operation. Returns 1 on success. */
uint8_t BmConfig_Save(void);

#endif /* BM_CONFIG_H */

/**
 ******************************************************************************
 * @file    bm_io.h
 * @brief   Simple digital I/O of the BM: CONNECT_Z, the control/status pins of
 *          the TLE9015 transceiver and the two status LEDs.
 *
 *   CONNECT_Z   (PA5, out)  1 = MOSFET T1 on: cell connected to the parallel rail
 *                           (stack measurement), 0 = single cell
 *   NSLEEP      (PB0, out)  TLE9015 sleep, active low -> kept high (awake)
 *   ERRQ_RES    (PB5, out)  reset of the latched ERRQ, active low -> high, pulsed low
 *   ERRQ_EXT    (PB6, out)  external error input of the TLE9015, active low -> high
 *   ERRQ        (PB7, in)   latched error of the TLE9015, open drain, low = error
 *                           (internal pull-up switched on here)
 *   ERR_EXT_OUT (PA11, in)  high: error was triggered via ERRQ_EXT
 *   ERR_LOC_OUT (PA12, in)  high (for tERR_loc_out): emergency signal received via isoUART
 *   STATUS_LED1 (PA7)       heartbeat: 1 Hz idle, 5 Hz while an EIS measurement runs
 *   STATUS_LED2 (PA8)       on while an error is present
 ******************************************************************************
 */
#ifndef BM_IO_H
#define BM_IO_H

#include "main.h"
#include <stdint.h>

/* Puts all outputs into their safe/idle state (CONNECT_Z off, TLE9015 awake,
 * no external error) and clears a latched ERRQ. Call right after MX_GPIO_Init(). */
void BmIo_Init(void);

/* LED handling. Call every main loop iteration. */
void BmIo_Process(uint8_t eisBusy, uint8_t errorPresent);

void    BmIo_SetConnectZ(uint8_t on);
uint8_t BmIo_GetConnectZ(void);

uint8_t BmIo_IsErrqActive(void);     /* 1 = TLE9015 ERRQ latched (pin low) */
uint8_t BmIo_IsErrLocOut(void);
uint8_t BmIo_IsErrExtOut(void);
void    BmIo_ResetErrq(void);        /* ~2 ms low pulse on ERRQ_RES (blocking) */

#endif /* BM_IO_H */

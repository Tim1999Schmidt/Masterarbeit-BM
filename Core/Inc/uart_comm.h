/**
 ******************************************************************************
 * @file    uart_comm.h
 * @brief   Debug console of the BM on USART1 (PA9/PA10), reachable over USB via
 *          the onboard FT231X. For bring-up and tests without the MM, and to
 *          set the per-board settings (I2C address, number of TLE9010).
 *
 * Terminal: 115200 baud, 8N1, no flow control. Lines end with CR and/or LF
 * (max. 40 characters), no echo (switch on "local echo"), not case sensitive.
 * Numbers decimal or 0x.. hex. Replies start with "OK" or "ERR <code>" (code =
 * ERROR_CODE of the I2C interface, see i2c_comm.h).
 *
 *   s                       status: st (STATUS bits as I2C), err, conz, errq,
 *                           loc/ext (ERR_LOC_OUT / ERR_EXT_OUT), prog, addr,
 *                           neis, izero + measurements
 *   m                       measurements: u (mV), i (mA), t (degC), vdda (mV),
 *                           cal (raw counts on the temperature references)
 *   conz [0|1]              show / set CONNECT_Z (not during a sweep)
 *   eis c f1 f2 a w n       start sweep: chip (0 cell, 1 stack), start and stop
 *                           frequency index (0 = 7812.5 Hz .. 51 = 1.09 Hz),
 *                           amplitude 0..5 (50..500 mA), waveform (0 sine,
 *                           1 rectangle), averages 1..9
 *   abort                   stop the sweep
 *   res                     results of the last sweep:
 *                           RES;idx;f_Hz;re_uOhm;im_uOhm
 *   addr [0x08..0x77]       show / set the I2C address (stored in flash, active at once)
 *   neis [1|2]              show / set the number of TLE9010 (2 only on the first BM)
 *   izero                   current zero calibration (no current may flow!), stored
 *   recal                   recalibrate the temperature front-end
 *   clear                   clear the error code, reset the TLE9015 ERRQ
 *   rd node reg             bring-up: raw read of a TLE9010 register, shows the
 *                           received bytes (expected: id reg data_hi data_lo crc)
 *   log [ms]                periodic line every ms (>= 100), 0 = off:
 *                           LOG;ms;u_mV;i_mA;t_degC;status;error;progress
 *
 * Can be left out with BM_ENABLE_UART = 0 (bm_config.h) if flash gets short.
 ******************************************************************************
 */
#ifndef UART_COMM_H
#define UART_COMM_H

#include "main.h"

void UartComm_Init(void);     /* after MX_USART1_UART_Init() */
void UartComm_Update(void);   /* every main loop iteration */

#endif /* UART_COMM_H */

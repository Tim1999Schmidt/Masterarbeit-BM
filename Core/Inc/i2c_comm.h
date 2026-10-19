/**
 ******************************************************************************
 * @file    i2c_comm.h
 * @brief   I2C1 slave register map of the BM towards the master board (MM),
 *          via the PCA9615 differential I2C buffer. Same protocol as the PM.
 *
 * Address: 7-bit, per board from flash (bm_config.h, default 0x30), set with
 * the "addr" command of the debug console.
 *
 *   Master write:  [register address][data bytes...]  (P)
 *     The bytes of ONE write take effect together at its end. Writes to
 *     read-only registers / beyond the map are ignored.
 *   Master read:   [register address](Sr)[data bytes...] (P)
 *     Consistent snapshot taken when the read starts.
 * Multi-byte fields little-endian, signed unless noted.
 *
 *   Addr  Name        R/W  Type    Meaning
 *   0x00  STATUS       RO  uint8   bit0 EIS busy, bit1 results of the last sweep available,
 *                                  bit2 error (see ERROR_CODE), bit3 tab temperature valid,
 *                                  bit4 CONNECT_Z on, bit5 TLE9015 ERRQ latched,
 *                                  bit6 stack TLE9010 present (numEis = 2),
 *                                  bit7 current outside the measuring range
 *   0x01  CONTROL      W   uint8   bit0 start EIS sweep (with 0x04..0x09), bit1 abort sweep,
 *                                  bit2 clear error (+ reset ERRQ), bit3 recalibrate the
 *                                  temperature front-end, bit4 current zero calibration
 *                                  (no current may flow; RAM only, as often as needed -
 *                                  lost at reset, then the value stored with the console
 *                                  command "izero" applies)                  (reads back 0)
 *   0x02  ERROR_CODE   RO  uint8   0 none, 1 EIS config, 2 busy, 3 EIS communication,
 *                                  4 CONNECT_Z does not fit (cell: 0, stack: 1),
 *                                  5 temperature sensor, 6 TLE9015 ERRQ, 7 flash
 *   0x03  CONNECT_Z    RW  uint8   0 = single cell, 1 = cell on the parallel rail (stack
 *                                  measurement). Only changed while no sweep runs.
 *                                  The MM must check the cell voltages first!
 *   0x04  EIS_CHIP     RW  uint8   0 = cell TLE9010, 1 = stack TLE9010 (first BM only)
 *   0x05  F_START      RW  uint8   frequency index 0..51 (0 = 7812.5 Hz, 51 = 1.09 Hz)
 *   0x06  F_STOP       RW  uint8   frequency index F_START..51
 *   0x07  AMPLITUDE    RW  uint8   0..5 = 50, 100, 200, 300, 400, 500 mA
 *   0x08  WAVEFORM     RW  uint8   0 = sine, 1 = rectangle
 *   0x09  AVERAGES     RW  uint8   1..9
 *   0x0A  PROGRESS     RO  uint8   frequency index being measured, 0xFF = idle
 *   0x0B  NUM_EIS      RO  uint8   number of TLE9010 on this board (1 / 2)
 *   0x0C  U_CELL       RO  uint16  mV
 *   0x0E  I_CELL       RO  int16   mA
 *   0x10  T_TAB        RO  int16   degC x100 (tab PT1000)
 *   0x12  CAL_LOW      RO  uint16  raw counts on the 1.00 kOhm reference (diagnostics)
 *   0x14  CAL_HIGH     RO  uint16  raw counts on the 1.20 kOhm reference (diagnostics)
 *   0x16  RES_INDEX    RW  uint8   frequency index whose result appears at 0x18..0x23
 *   0x17  RES_VALID    RO  uint8   1 = RES_INDEX was part of the last completed sweep
 *   0x18  RES_FREQ     RO  uint32  frequency of RES_INDEX in 0.01 Hz
 *   0x1C  RES_RE       RO  int32   real part in uOhm
 *   0x20  RES_IM       RO  int32   imaginary part in uOhm
 *
 * Register map: 0x24 (36) bytes. Reading one result: write [0x16][index] (P),
 * then [0x17](Sr) and read 13 bytes (RES_VALID .. RES_IM). The result fields
 * are filled at the start of each read, so they always belong to the RES_INDEX
 * written before.
 *
 * Typical sequence of the MM:
 *   single cell: CONNECT_Z = 0, write 0x04..0x09, CONTROL = 0x01, poll STATUS
 *                until bit0 = 0, check bit1/ERROR_CODE, read the results.
 *   stack:       read U_CELL of all BMs, compare; CONNECT_Z = 1 on all BMs;
 *                on the first BM EIS_CHIP = 1, ..., CONTROL = 0x01; afterwards
 *                CONNECT_Z = 0 on all BMs.
 ******************************************************************************
 */
#ifndef I2C_COMM_H
#define I2C_COMM_H

#include "main.h"

/* Sets the slave address (from bm_config), enables the I2C1 interrupt and
 * starts listening. Call once after MX_I2C1_Init(). */
void I2cComm_Init(void);

/* Refreshes the read-only registers and executes what the master wrote.
 * Call once per main loop iteration. */
void I2cComm_Update(void);

/* Changes the slave address at runtime (debug console "addr"). */
void I2cComm_SetAddress(uint8_t addr7);

#endif /* I2C_COMM_H */

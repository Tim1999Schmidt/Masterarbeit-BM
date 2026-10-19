/**
 ******************************************************************************
 * @file    tle9015.h
 * @brief   Frame level access to the TLE9010EP EIS chips through the TLE9015
 *          isoUART transceiver on USART2 (PA3 RX / PA4 TX).
 *
 * Frame format, register numbers and CRC parameters are taken over unchanged
 * from the working Zellhalter firmware (tle9015.c, Stieberger 2025):
 *   UART: 1 MBaud, 8N1, MSB first (configured here, independent of BM.ioc)
 *   write: [0x02][0x80 | node][reg][data_hi][data_lo][CRC8(5 bytes) ^ 0x10]
 *   read : [0x02][node][reg][CRC8(3 bytes) ^ 0xF1]
 *          reply: [id][reg][data_hi][data_lo][crc]
 *   CRC8: polynomial 0x1D, init 0x00
 *
 * Daisy chain on a BM (Master on Top): MCU -> TLE9015 -> cell TLE9010 (node 1)
 * -> stack TLE9010 (node 2, only on the first BM). Node numbers are assigned by
 * Tle_Enumerate(); the last node is marked as final node.
 *
 * All functions are blocking for some 10..1000 us (1 MBaud) and are called
 * from the main loop only.
 ******************************************************************************
 */
#ifndef TLE9015_H
#define TLE9015_H

#include "main.h"
#include <stdint.h>

#define TLE_NODE_CELL    1u
#define TLE_NODE_STACK   2u

/* TLE9010 registers used (see the Zellhalter firmware) */
#define TLE_REG_GOTO_SLEEP    0x02u   /* sleep timer */
#define TLE_REG_ZMEAS_AMP     0x0Au   /* excitation amplitude */
#define TLE_REG_ZMEAS_CFG     0x0Cu   /* waveform, measuring time, frequency */
#define TLE_REG_ZMEAS_CTRL    0x0Du   /* start / stop */
#define TLE_REG_ZMEAS_MAG     0x0Eu   /* result: magnitude (12-bit mantissa + 4-bit exponent) */
#define TLE_REG_ZMEAS_PHASE   0x0Fu   /* result: phase (16 bit) */
#define TLE_REG_NODE_ID       0x25u   /* enumeration */

/* register values */
#define TLE_SLEEP_TIMER_STAY_AWAKE   0xFFFFu  /* GOTO_SLEEP: longest timer (~15 h) = awake for the sweep */
#define TLE_SLEEP_TIMER_SLEEP_NOW    0x0000u  /* GOTO_SLEEP: go to sleep immediately */
#define TLE_ZMEAS_CTRL_START         0x0001u
#define TLE_ZMEAS_CTRL_STOP          0x0000u  /* stop / reset the measurement */
#define TLE_NODE_ID_CLEAR            0x0000u  /* NODE_ID: remove the node number */
#define TLE_NODE_ID_FINAL_NODE       0x0800u  /* NODE_ID bit 11: last node of the chain */

/* Reconfigures USART2 for the isoUART link (1 MBaud, MSB first). Call once
 * after MX_USART2_UART_Init(). */
void Tle_Init(void);

/* Busy wait in microseconds (SysTick based, no timer needed). */
void Tle_DelayUs(uint32_t us);

/* Wake-up pulses for the chain (6 x 0x0F) + 500 us. */
void Tle_Wakeup(void);

/* Assigns node numbers 1..numNodes (old numbers are cleared first); the last
 * one becomes the final node of the chain. */
void Tle_Enumerate(uint8_t numNodes);

/* Write one register of one node. */
void Tle_Write(uint8_t node, uint8_t reg, uint16_t value);

/* Read one register of one node (up to 3 attempts). Returns 1 on success. */
uint8_t Tle_Read(uint8_t node, uint8_t reg, uint16_t *value);

/* Bring-up help: sends a read request and returns the raw received bytes
 * (up to maxLen) - used by the "rd" command of the debug console. */
#define TLE_RAW_REPLY_MAX_LEN   8u   /* buffer size for Tle_ReadRaw() in the console */
uint8_t Tle_ReadRaw(uint8_t node, uint8_t reg, uint8_t *buf, uint8_t maxLen);

#endif /* TLE9015_H */

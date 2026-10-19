/**
 ******************************************************************************
 * @file    bm_app.h
 * @brief   Application layer of the BM: the commands both interfaces (I2C from
 *          the MM, debug console) use, with the same checks, and the common
 *          status / error code.
 *
 * Rules enforced here (so they hold for every interface):
 *  - CONNECT_Z can only be changed while no EIS sweep is running.
 *  - Cell measurement (chip 0) only with CONNECT_Z = 0 (otherwise the cell
 *    TLE9010 would measure the parallel connection, not its own cell);
 *    stack measurement (chip 1) only with CONNECT_Z = 1 and only on a board
 *    with the stack TLE9010 (numEis = 2).
 *  - NOT checked here (each BM only knows its own cell): before the MM sets
 *    CONNECT_Z = 1 on the boards, it must compare the cell voltages (U_CELL) of
 *    all boards - cells with a large voltage difference must not be connected
 *    in parallel (Zellhalter firmware: max. 50 mV).
 ******************************************************************************
 */
#ifndef BM_APP_H
#define BM_APP_H

#include "main.h"
#include "eis.h"
#include <stdint.h>

typedef enum
{
    APP_ERR_NONE          = 0,
    APP_ERR_CONFIG        = 1,  /* EIS parameter out of range / stack chip not present */
    APP_ERR_BUSY          = 2,  /* command not allowed while a sweep is running */
    APP_ERR_EIS_COMM      = 3,  /* TLE9010 did not answer (sweep stopped) */
    APP_ERR_CONNECT_STATE = 4,  /* CONNECT_Z does not fit the requested measurement */
    APP_ERR_TEMP_SENSOR   = 5,  /* tab temperature: PT1000 implausible / calibration failed */
    APP_ERR_ERRQ          = 6,  /* TLE9015 ERRQ latched (emergency signal / external error) */
    APP_ERR_FLASH         = 7   /* saving the board settings failed */
} App_Error_t;

/* STATUS bits (same for I2C and the console) */
#define APP_ST_EIS_BUSY       0x01u
#define APP_ST_RESULTS        0x02u
#define APP_ST_ERROR          0x04u
#define APP_ST_TEMP_VALID     0x08u
#define APP_ST_CONNECT_Z      0x10u
#define APP_ST_ERRQ           0x20u
#define APP_ST_STACK_CHIP     0x40u
#define APP_ST_I_SATURATED    0x80u

void App_Process(void);                         /* call every main loop iteration */

App_Error_t App_StartEis(const Eis_Config_t *cfg);
void        App_AbortEis(void);
App_Error_t App_SetConnectZ(uint8_t on);
void        App_ClearError(void);               /* clears the error code and resets ERRQ */
void        App_SetError(App_Error_t e);        /* e.g. APP_ERR_FLASH from the console */

uint8_t     App_GetStatus(void);
App_Error_t App_GetErrorCode(void);

#endif /* BM_APP_H */

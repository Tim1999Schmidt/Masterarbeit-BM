/**
 ******************************************************************************
 * @file    bm_app.c
 * @brief   See bm_app.h.
 ******************************************************************************
 */
#include "bm_app.h"
#include "bm_config.h"
#include "bm_io.h"
#include "cell_meas.h"
#include "temperature_sensor.h"

static App_Error_t latchedError = APP_ERR_NONE;
static uint8_t     wasBusy;

void App_Process(void)
{
    const uint8_t busy = EIS_IsBusy();

    /* a sweep that ended with a communication error latches the error */
    if (wasBusy && !busy && EIS_GetLastResult() == EIS_ERR_COMM)
        latchedError = APP_ERR_EIS_COMM;
    wasBusy = busy;
}

App_Error_t App_StartEis(const Eis_Config_t *cfg)
{
    Eis_Result_t r;

    if ((cfg->chip == EIS_CHIP_CELL && BmIo_GetConnectZ()) ||
        (cfg->chip == EIS_CHIP_STACK && !BmIo_GetConnectZ()))
    {
        latchedError = APP_ERR_CONNECT_STATE;
        return latchedError;
    }

    r = EIS_Start(cfg, BmConfig_Get()->numEis);
    if (r == EIS_ERR_BUSY)
        latchedError = APP_ERR_BUSY;
    else if (r != EIS_OK)
        latchedError = APP_ERR_CONFIG;
    else
        return APP_ERR_NONE;
    return latchedError;
}

void App_AbortEis(void)
{
    EIS_Abort();
}

App_Error_t App_SetConnectZ(uint8_t on)
{
    on = on ? 1u : 0u;
    if (on == BmIo_GetConnectZ())
        return APP_ERR_NONE;
    if (EIS_IsBusy())
    {
        latchedError = APP_ERR_BUSY;
        return APP_ERR_BUSY;
    }
    BmIo_SetConnectZ(on);
    return APP_ERR_NONE;
}

void App_ClearError(void)
{
    latchedError = APP_ERR_NONE;
    BmIo_ResetErrq();
}

void App_SetError(App_Error_t e)
{
    latchedError = e;
}

App_Error_t App_GetErrorCode(void)
{
    if (latchedError != APP_ERR_NONE)
        return latchedError;
    if (BmIo_IsErrqActive())
        return APP_ERR_ERRQ;
    if (TempSensor_GetFault() != TEMP_FAULT_NONE)
        return APP_ERR_TEMP_SENSOR;
    return APP_ERR_NONE;
}

uint8_t App_GetStatus(void)
{
    return (uint8_t)((EIS_IsBusy()                      ? APP_ST_EIS_BUSY    : 0u) |
                     (EIS_HasResults()                  ? APP_ST_RESULTS     : 0u) |
                     (App_GetErrorCode() != APP_ERR_NONE ? APP_ST_ERROR      : 0u) |
                     (TempSensor_IsReady()              ? APP_ST_TEMP_VALID  : 0u) |
                     (BmIo_GetConnectZ()                ? APP_ST_CONNECT_Z   : 0u) |
                     (BmIo_IsErrqActive()               ? APP_ST_ERRQ        : 0u) |
                     (BmConfig_Get()->numEis >= BM_NUM_EIS_WITH_STACK ? APP_ST_STACK_CHIP : 0u) |
                     (CellMeas_IsCurrentSaturated()     ? APP_ST_I_SATURATED : 0u));
}

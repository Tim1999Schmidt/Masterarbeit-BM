/**
 ******************************************************************************
 * @file    bm_io.c
 * @brief   See bm_io.h.
 ******************************************************************************
 */
#include "bm_io.h"

#define ERRQ_RES_PULSE_MS        2u     /* longer than the TLE9015 deglitch time tERRQ_res_deglitch */
#define ERRQ_PULLUP_SETTLE_MS    1u     /* let the ERRQ line rise after enabling the pull-up */
#define LED_HALF_PERIOD_IDLE_MS  500u   /* LED1 1 Hz while idle */
#define LED_HALF_PERIOD_EIS_MS   100u   /* LED1 5 Hz during an EIS sweep */

static uint8_t connectZ;

void BmIo_Init(void)
{
    GPIO_InitTypeDef gi = {0};

    HAL_GPIO_WritePin(CONNECT_Z_GPIO_Port, CONNECT_Z_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(NSLEEP_GPIO_Port,    NSLEEP_Pin,    GPIO_PIN_SET);
    HAL_GPIO_WritePin(ERRQ_EXT_GPIO_Port,  ERRQ_EXT_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(ERRQ_RES_GPIO_Port,  ERRQ_RES_Pin,  GPIO_PIN_SET);
    connectZ = 0u;

    /* ERRQ is open drain on the TLE9015 side: needs a pull-up */
    gi.Pin  = ERRQ_Pin;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ERRQ_GPIO_Port, &gi);

    HAL_Delay(ERRQ_PULLUP_SETTLE_MS);
    BmIo_ResetErrq();
}

void BmIo_Process(uint8_t eisBusy, uint8_t errorPresent)
{
    const uint32_t half = eisBusy ? LED_HALF_PERIOD_EIS_MS : LED_HALF_PERIOD_IDLE_MS;
    HAL_GPIO_WritePin(STATUS_LED1_GPIO_Port, STATUS_LED1_Pin,
                      (((HAL_GetTick() / half) & 1u) != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STATUS_LED2_GPIO_Port, STATUS_LED2_Pin, errorPresent ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BmIo_SetConnectZ(uint8_t on)
{
    connectZ = on ? 1u : 0u;
    HAL_GPIO_WritePin(CONNECT_Z_GPIO_Port, CONNECT_Z_Pin, connectZ ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t BmIo_GetConnectZ(void)   { return connectZ; }
uint8_t BmIo_IsErrqActive(void)  { return (HAL_GPIO_ReadPin(ERRQ_GPIO_Port, ERRQ_Pin) == GPIO_PIN_RESET) ? 1u : 0u; }
uint8_t BmIo_IsErrLocOut(void)   { return (HAL_GPIO_ReadPin(ERR_LOC_OUT_GPIO_Port, ERR_LOC_OUT_Pin) == GPIO_PIN_SET) ? 1u : 0u; }
uint8_t BmIo_IsErrExtOut(void)   { return (HAL_GPIO_ReadPin(ERR_EXT_OUT_GPIO_Port, ERR_EXT_OUT_Pin) == GPIO_PIN_SET) ? 1u : 0u; }

void BmIo_ResetErrq(void)
{
    HAL_GPIO_WritePin(ERRQ_RES_GPIO_Port, ERRQ_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(ERRQ_RES_PULSE_MS);
    HAL_GPIO_WritePin(ERRQ_RES_GPIO_Port, ERRQ_RES_Pin, GPIO_PIN_SET);
}

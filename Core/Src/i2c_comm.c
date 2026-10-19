/**
 ******************************************************************************
 * @file    i2c_comm.c
 * @brief   See i2c_comm.h.
 *
 * The slave is driven directly through the I2C1 registers in one small ISR
 * instead of the HAL interrupt driver (HAL_I2C_EnableListen_IT / Seq_*_IT /
 * EV/ER_IRQHandler pull in ~3 KB of code incl. DMA abort paths - too much for
 * the 32 KB flash). Behaviour towards the master is the same as on the PM:
 *   - address match (ADDR): a pending write is committed (repeated START);
 *     for a read, a snapshot of the map is taken and sent from the pointer on
 *   - every received byte (RXNE): 1st byte = register pointer, then data
 *   - STOP: commits the write; an unsent preloaded TX byte is flushed
 * Clock stretching is enabled (CubeMX default), so the ISR latency only slows
 * the bus down, it never loses data.
 ******************************************************************************
 */
#include "i2c_comm.h"
#include "bm_app.h"
#include "bm_config.h"
#include "bm_io.h"
#include "cell_meas.h"
#include "eis.h"
#include "temperature_sensor.h"
#include <string.h>
#include <stddef.h>

extern I2C_HandleTypeDef hi2c1; /* initialised by MX_I2C1_Init() in main.c */

#define I2C_IRQ_PRIORITY  1u      /* above USART1 (2) */
#define I2C_OAR1_ADDR7_SHIFT 1u    /* OAR1: 7-bit address in bits 7..1 */
#define I2C_READ_PAST_END 0xFFu    /* sent if the master reads beyond the register map */
#define T_CENTI_PER_DEGC  100.0f   /* T_TAB in degC x100 */

typedef struct __attribute__((packed))
{
    uint8_t  status;      /* 0x00 RO */
    uint8_t  control;     /* 0x01 W  */
    uint8_t  errorCode;   /* 0x02 RO */
    uint8_t  connectZ;    /* 0x03 RW */
    uint8_t  eisChip;     /* 0x04 RW */
    uint8_t  fStart;      /* 0x05 RW */
    uint8_t  fStop;       /* 0x06 RW */
    uint8_t  amplitude;   /* 0x07 RW */
    uint8_t  waveform;    /* 0x08 RW */
    uint8_t  averages;    /* 0x09 RW */
    uint8_t  progress;    /* 0x0A RO */
    uint8_t  numEis;      /* 0x0B RO */
    uint16_t uCell;       /* 0x0C RO mV */
    int16_t  iCell;       /* 0x0E RO mA */
    int16_t  tTab;        /* 0x10 RO degC x100 */
    uint16_t calLow;      /* 0x12 RO */
    uint16_t calHigh;     /* 0x14 RO */
    uint8_t  resIndex;    /* 0x16 RW */
    uint8_t  resValid;    /* 0x17 RO */
    uint32_t resFreq;     /* 0x18 RO 0.01 Hz */
    int32_t  resRe;       /* 0x1C RO uOhm */
    int32_t  resIm;       /* 0x20 RO uOhm */
} RegisterMap_t;

#define REG_MAP_SIZE      ((uint8_t)sizeof(RegisterMap_t))
#define OFF(field)        ((uint8_t)offsetof(RegisterMap_t, field))
#define BIT(off)          (1ul << (off))

/* writable registers: CONTROL, CONNECT_Z, EIS_CHIP..AVERAGES, RES_INDEX.
 * All writable offsets are < 32, so a 32-bit mask is enough. */
#define WRITABLE_MASK     (BIT(OFF(control))  | BIT(OFF(connectZ)) | BIT(OFF(eisChip)) | \
                           BIT(OFF(fStart))   | BIT(OFF(fStop))    | BIT(OFF(amplitude)) | \
                           BIT(OFF(waveform)) | BIT(OFF(averages)) | BIT(OFF(resIndex)))
#define MASK_BITS         32u

#define LAYOUT_CHECK(name, cond)  typedef char layout_check_##name[(cond) ? 1 : -1]
LAYOUT_CHECK(map_size_is_36_bytes, sizeof(RegisterMap_t)             == 0x24u);
LAYOUT_CHECK(u_cell_at_0x0C,       offsetof(RegisterMap_t, uCell)    == 0x0Cu);
LAYOUT_CHECK(res_index_at_0x16,    offsetof(RegisterMap_t, resIndex) == 0x16u);
LAYOUT_CHECK(res_freq_at_0x18,     offsetof(RegisterMap_t, resFreq)  == 0x18u);
LAYOUT_CHECK(res_im_at_0x20,       offsetof(RegisterMap_t, resIm)    == 0x20u);
LAYOUT_CHECK(writable_below_32,    offsetof(RegisterMap_t, resIndex) < MASK_BITS);

#define CTRL_START        0x01u
#define CTRL_ABORT        0x02u
#define CTRL_CLEAR_ERROR  0x04u
#define CTRL_RECAL_TEMP   0x08u
#define CTRL_ZERO_CURRENT 0x10u

static RegisterMap_t regs;

/* ISR-side transaction state */
static uint8_t  regPointer, rxCount, wrPointer, txIndex;
static uint8_t  wrData[REG_MAP_SIZE];
static uint32_t wrMask;
static uint8_t  txSnapshot[REG_MAP_SIZE];

static volatile uint8_t pendingControlWrite, pendingConnectZWrite;

static int16_t ToInt16(float v)
{
    if (v >= (float)INT16_MAX) return INT16_MAX;
    if (v <= (float)INT16_MIN) return INT16_MIN;
    return (int16_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

static int16_t ClampI16(int32_t v)
{
    return (int16_t)((v > INT16_MAX) ? INT16_MAX : ((v < INT16_MIN) ? INT16_MIN : v));
}

/* result window, filled for RES_INDEX (main loop and at the start of every read) */
static void FillResult(void)
{
    int32_t re = 0, im = 0;
    regs.resValid = EIS_GetResult(regs.resIndex, &re, &im);
    regs.resFreq  = (regs.resIndex < EIS_NUM_FREQ) ? EIS_FreqCentiHz(regs.resIndex) : 0u;
    regs.resRe    = re;
    regs.resIm    = im;
}

static void FinishWriteTransaction(void)
{
    const uint32_t mask = wrMask;
    uint8_t off;

    if (mask != 0u)
    {
        uint8_t *dst = (uint8_t *)&regs;
        for (off = 0u; off < MASK_BITS && off < REG_MAP_SIZE; off++)
        {
            if (mask & BIT(off))
                dst[off] = wrData[off];
        }
        if (mask & BIT(OFF(control)))
            pendingControlWrite = 1u;
        if (mask & BIT(OFF(connectZ)))
            pendingConnectZWrite = 1u;
    }
    wrMask = 0u;
    rxCount = 0u;
}

/* ---- main-loop side ------------------------------------------------------------- */
void I2cComm_SetAddress(uint8_t addr7)
{
    hi2c1.Init.OwnAddress1 = (uint32_t)addr7 << I2C_OAR1_ADDR7_SHIFT;
    hi2c1.Instance->OAR1 &= ~I2C_OAR1_OA1EN;
    hi2c1.Instance->OAR1 = I2C_OAR1_OA1EN | hi2c1.Init.OwnAddress1;
}

void I2cComm_Init(void)
{
    memset(&regs, 0, sizeof(regs));
    regs.fStop     = EIS_FREQ_INDEX_MAX;
    regs.averages  = EIS_AVERAGES_MIN;
    regs.progress  = EIS_PROGRESS_IDLE;

    I2cComm_SetAddress(BmConfig_Get()->i2cAddr);
    I2C1->ICR = I2C_ICR_ADDRCF | I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
    I2C1->CR1 |= I2C_CR1_ADDRIE | I2C_CR1_RXIE | I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_ERRIE;
    HAL_NVIC_SetPriority(I2C1_IRQn, I2C_IRQ_PRIORITY, 0u);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);
}

void I2cComm_Update(void)
{
    RegisterMap_t snap;
    uint8_t doControl, doConnectZ;

    /* telemetry, built outside the critical section */
    const uint8_t  status    = App_GetStatus();
    const uint8_t  errorCode = (uint8_t)App_GetErrorCode();
    const uint8_t  connectZ  = BmIo_GetConnectZ();
    const uint8_t  progress  = EIS_GetProgress();
    const uint8_t  numEis    = BmConfig_Get()->numEis;
    const int32_t  u         = CellMeas_GetVoltageMv();
    const uint16_t uCell     = (uint16_t)((u < 0) ? 0 : ((u > (int32_t)UINT16_MAX) ? (int32_t)UINT16_MAX : u));
    const int16_t  iCell     = ClampI16(CellMeas_GetCurrentMa());
    const int16_t  tTab      = ToInt16(TempSensor_GetTemperature() * T_CENTI_PER_DEGC);
    const uint16_t calLow    = TempSensor_GetCalCountsLow();
    const uint16_t calHigh   = TempSensor_GetCalCountsHigh();

    __disable_irq();
    regs.status    = status;
    regs.errorCode = errorCode;
    regs.progress  = progress;
    regs.numEis    = numEis;
    regs.uCell     = uCell;
    regs.iCell     = iCell;
    regs.tTab      = tTab;
    regs.calLow    = calLow;
    regs.calHigh   = calHigh;
    FillResult();
    doControl  = pendingControlWrite;
    doConnectZ = pendingConnectZWrite;
    pendingControlWrite  = 0u;
    pendingConnectZWrite = 0u;
    snap = regs;
    if (doControl)
        regs.control = 0u;
    if (!doConnectZ)
        regs.connectZ = connectZ;          /* mirror the state in effect (also changes via the console) */
    __enable_irq();

    /* CONNECT_Z first, then commands (a write of both in one transaction
     * starts the sweep with the new state). The EIS parameters 0x04..0x09 are
     * simply taken from the register map when a sweep is started. */
    if (doConnectZ)
    {
        if (App_SetConnectZ(snap.connectZ) != APP_ERR_NONE)
        {
            __disable_irq();
            if (regs.connectZ == snap.connectZ)
                regs.connectZ = BmIo_GetConnectZ();   /* rejected: show the real state */
            __enable_irq();
        }
    }

    if (doControl)
    {
        if (snap.control & CTRL_CLEAR_ERROR)
            App_ClearError();
        if (snap.control & CTRL_ABORT)
            App_AbortEis();
        if (snap.control & CTRL_START)
        {
            Eis_Config_t c;
            c.chip      = snap.eisChip;
            c.fStart    = snap.fStart;
            c.fStop     = snap.fStop;
            c.amplitude = snap.amplitude;
            c.waveform  = snap.waveform;
            c.averages  = snap.averages;
            (void)App_StartEis(&c);
        }
        if (snap.control & CTRL_RECAL_TEMP)
            TempSensor_Recalibrate();
        if (snap.control & CTRL_ZERO_CURRENT)
            (void)CellMeas_ZeroCurrent();     /* RAM only - stored in flash only via the console "izero" */
    }
}

/* ---- ISR side ---------------------------------------------------------------------- */
static void RxByte(uint8_t b)
{
    if (rxCount == 0u)
    {
        wrPointer  = b;                                   /* first byte = register pointer */
        regPointer = (b < REG_MAP_SIZE) ? b : 0u;
    }
    else if (wrPointer < MASK_BITS)
    {
        if (WRITABLE_MASK & BIT(wrPointer))
        {
            wrData[wrPointer] = b;
            wrMask |= BIT(wrPointer);
        }
        wrPointer++;
    }
    if (rxCount < UINT8_MAX)
        rxCount++;
}

/* NOTE: do not enable "I2C1 global interrupt" in CubeMX - it would generate a
 * second I2C1_IRQHandler (HAL based) in stm32c0xx_it.c. */
void I2C1_IRQHandler(void)
{
    const uint32_t isr = I2C1->ISR;

    if (isr & (I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR))
    {
        I2C1->ICR = I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
        wrMask  = 0u;                                     /* bus error: drop the partial write */
        rxCount = 0u;
    }

    if (isr & I2C_ISR_RXNE)                               /* before ADDR: last byte of a write */
        RxByte((uint8_t)I2C1->RXDR);

    if (isr & I2C_ISR_ADDR)
    {
        FinishWriteTransaction();                         /* repeated START ends the write */
        if (isr & I2C_ISR_DIR)                            /* master reads */
        {
            FillResult();                                 /* result fields match RES_INDEX */
            memcpy(txSnapshot, &regs, REG_MAP_SIZE);
            txIndex = regPointer;
            I2C1->ISR |= I2C_ISR_TXE;                     /* flush a stale TXDR byte */
        }
        I2C1->ICR = I2C_ICR_ADDRCF;                       /* releases SCL */
    }

    if (isr & I2C_ISR_TXIS)
        I2C1->TXDR = (txIndex < REG_MAP_SIZE) ? txSnapshot[txIndex++] : I2C_READ_PAST_END;

    if (isr & I2C_ISR_NACKF)                              /* master ends a read */
        I2C1->ICR = I2C_ICR_NACKCF;

    if (isr & I2C_ISR_STOPF)
    {
        I2C1->ICR = I2C_ICR_STOPCF;
        FinishWriteTransaction();
        I2C1->ISR |= I2C_ISR_TXE;                         /* drop a preloaded, unsent byte */
    }
}

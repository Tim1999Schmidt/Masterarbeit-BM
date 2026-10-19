/**
 ******************************************************************************
 * @file    uart_comm.c
 * @brief   See uart_comm.h.
 *
 * Kept small (32 KB flash): no printf/scanf, USART1 registers used directly
 * (no HAL UART interrupt driver), integer output only.
 ******************************************************************************
 */
#include "uart_comm.h"
#include "bm_config.h"

#if BM_ENABLE_UART

#include "bm_app.h"
#include "bm_io.h"
#include "cell_meas.h"
#include "eis.h"
#include "i2c_comm.h"
#include "temperature_sensor.h"
#include "tle9015.h"
#include <string.h>

#define UART_IRQ_PRIORITY     2u      /* below I2C1 (1) */
#define RX_BUF_SIZE          32u      /* power of two */
#define TX_BUF_SIZE         256u      /* power of two */
#define LINE_MAX             40u
#define MAX_ARGS              6u
#define LOG_MIN_PERIOD_MS   100u
#define RES_LINE_SPACE       48u      /* free TX space needed for one result line */
#define RES_PRINT_NONE     0xFFu      /* resPrint: no result table being printed */
#define UINT32_DEC_DIGITS    10u      /* "4294967295" */
#define CENTI_PER_UNIT      100.0f    /* values printed with 2 decimals */
#define ASCII_DEL          0x7F       /* backspace key of many terminals */
#define EIS_CMD_ARGS          6u      /* eis chip fstart fstop amp wave avg */
#define RD_CMD_ARGS           2u      /* rd node reg */

static volatile uint8_t  rxBuf[RX_BUF_SIZE];
static volatile uint8_t  rxHead, rxTail;
static volatile uint8_t  txBuf[TX_BUF_SIZE];
static volatile uint16_t txHead, txTail;

static char     line[LINE_MAX + 1u];
static uint8_t  lineLen;
static uint32_t logPeriodMs, logLastTick;
static uint8_t  resPrint = RES_PRINT_NONE;   /* next result index to print */

/* ---- output ------------------------------------------------------------------- */
static uint16_t TxFree(void)
{
    return (uint16_t)((txTail - txHead - 1u) & (TX_BUF_SIZE - 1u));
}

static void PutChar(char c)
{
    const uint16_t next = (uint16_t)((txHead + 1u) & (TX_BUF_SIZE - 1u));
    if (next == txTail)
        return;                                  /* full: drop (debug output only) */
    txBuf[txHead] = (uint8_t)c;
    txHead = next;
    USART1->CR1 |= USART_CR1_TXEIE_TXFNFIE;
}

static void Put(const char *s)
{
    while (*s)
        PutChar(*s++);
}

static void PutUInt(uint32_t v)
{
    char buf[UINT32_DEC_DIGITS];
    uint8_t n = 0u;
    do { buf[n++] = (char)('0' + (v % 10u)); v /= 10u; } while (v != 0u);
    while (n > 0u)
        PutChar(buf[--n]);
}

static void PutInt(int32_t v)
{
    if (v < 0)
    {
        PutChar('-');
        PutUInt((uint32_t)(-(v + 1)) + 1u);
    }
    else
        PutUInt((uint32_t)v);
}

/* value given in 1/100: prints "-12.34" */
static void PutCenti(int32_t v)
{
    uint32_t a;
    if (v < 0) { PutChar('-'); a = (uint32_t)(-(v + 1)) + 1u; } else a = (uint32_t)v;
    PutUInt(a / 100u);
    PutChar('.');
    PutChar((char)('0' + (a / 10u) % 10u));
    PutChar((char)('0' + a % 10u));
}

static void PutHex2(uint8_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    PutChar(hex[v >> 4]);
    PutChar(hex[v & 0x0Fu]);
}

static void PutKV(const char *key, int32_t v)
{
    Put(key);
    PutInt(v);
}

static int32_t TempCenti(void)
{
    const float t = TempSensor_GetTemperature() * CENTI_PER_UNIT;
    return (int32_t)((t >= 0.0f) ? (t + 0.5f) : (t - 0.5f));
}

static void Ok(void)
{
    Put("OK\r\n");
}

static void ReplyError(App_Error_t e)
{
    if (e == APP_ERR_NONE) { Ok(); return; }
    PutKV("ERR ", (int32_t)e);
    Put("\r\n");
}

static void PrintMeasurements(void)
{
    PutKV("u=", CellMeas_GetVoltageMv());
    PutKV(" i=", CellMeas_GetCurrentMa());
    Put(" t=");
    PutCenti(TempCenti());
    PutKV(" vdda=", (int32_t)CellMeas_GetVddaMv());
    PutKV(" cal=", TempSensor_GetCalCountsLow());
    PutKV("/", TempSensor_GetCalCountsHigh());
    Put("\r\n");
}

static void PrintResultLine(uint8_t idx)
{
    int32_t re, im;
    if (!EIS_GetResult(idx, &re, &im))
        return;
    PutKV("RES;", idx);
    Put(";");
    PutCenti((int32_t)EIS_FreqCentiHz(idx));
    PutKV(";", re);
    PutKV(";", im);
    Put("\r\n");
}

/* ---- input ---------------------------------------------------------------------- */
/* Decimal or 0x-hex, optional '-'. Returns 1 on success. */
static uint8_t ParseInt(const char *s, int32_t *out)
{
    int32_t v = 0;
    uint8_t neg = 0u, digits = 0u, base = 10u;

    if (*s == '-') { neg = 1u; s++; }
    if (s[0] == '0' && s[1] == 'x') { base = 16u; s += 2; }
    for (; *s; s++)
    {
        uint8_t d;
        if (*s >= '0' && *s <= '9')                      d = (uint8_t)(*s - '0');
        else if (base == 16u && *s >= 'a' && *s <= 'f')  d = (uint8_t)(*s - 'a' + 10);
        else return 0u;
        v = v * (int32_t)base + d;
        digits++;
    }
    if (digits == 0u)
        return 0u;
    *out = neg ? -v : v;
    return 1u;
}

/* Splits "cmd a b c" in place; returns the number of numeric arguments or -1 on a bad number. */
static int8_t SplitArgs(char *cmd, int32_t *args)
{
    int8_t n = 0;
    char *p = strchr(cmd, ' ');

    while (p != NULL)
    {
        char *next;
        *p++ = '\0';
        while (*p == ' ')
            p++;
        if (*p == '\0')
            break;
        next = strchr(p, ' ');
        if (next != NULL)
            *next = '\0';
        if (n >= (int8_t)MAX_ARGS || !ParseInt(p, &args[n]))
            return -1;
        n++;
        p = next;
    }
    return n;
}

static void ProcessLine(char *cmd)
{
    int32_t a[MAX_ARGS];
    const int8_t n = SplitArgs(cmd, a);
    BmConfig_t *c = BmConfig_Get();

    if (n < 0)
    {
        Put("ERR number\r\n");
        return;
    }

    if (strcmp(cmd, "s") == 0)
    {
        PutKV("OK st=", App_GetStatus());
        PutKV(" err=", (int32_t)App_GetErrorCode());
        PutKV(" conz=", BmIo_GetConnectZ());
        PutKV(" errq=", BmIo_IsErrqActive());
        PutKV(" loc=", BmIo_IsErrLocOut());
        PutKV(" ext=", BmIo_IsErrExtOut());
        PutKV(" prog=", EIS_GetProgress());
        Put(" addr=0x");
        PutHex2(c->i2cAddr);
        PutKV(" neis=", c->numEis);
        PutKV(" izero=", c->iZeroMv);
        Put("\r\n   ");
        PrintMeasurements();
    }
    else if (strcmp(cmd, "m") == 0)
    {
        Put("OK ");
        PrintMeasurements();
    }
    else if (strcmp(cmd, "conz") == 0)
    {
        if (n == 1)
            ReplyError(App_SetConnectZ((uint8_t)(a[0] != 0)));
        else
        {
            PutKV("OK conz=", BmIo_GetConnectZ());
            Put("\r\n");
        }
    }
    else if (strcmp(cmd, "eis") == 0)
    {
        Eis_Config_t e;
        if (n != (int8_t)EIS_CMD_ARGS)
        {
            Put("ERR eis chip fstart fstop amp wave avg\r\n");
            return;
        }
        e.chip = (uint8_t)a[0]; e.fStart = (uint8_t)a[1]; e.fStop = (uint8_t)a[2];
        e.amplitude = (uint8_t)a[3]; e.waveform = (uint8_t)a[4]; e.averages = (uint8_t)a[5];
        {
            uint8_t k;
            for (k = 0u; k < EIS_CMD_ARGS; k++)
                if (a[k] < 0 || a[k] > (int32_t)UINT8_MAX)
                    e.averages = 0u;               /* -> rejected as config error */
        }
        ReplyError(App_StartEis(&e));
    }
    else if (strcmp(cmd, "abort") == 0)
    {
        App_AbortEis();
        Ok();
    }
    else if (strcmp(cmd, "res") == 0)
    {
        if (!EIS_HasResults())
        {
            Put("ERR no results\r\n");
            return;
        }
        Put("OK RES;idx;f_Hz;re_uOhm;im_uOhm\r\n");
        resPrint = EIS_GetLastConfig()->fStart;   /* printed piece by piece in UartComm_Update() */
    }
    else if (strcmp(cmd, "addr") == 0)
    {
        if (n == 1)
        {
            if (a[0] < (int32_t)BM_I2C_ADDR_MIN || a[0] > (int32_t)BM_I2C_ADDR_MAX)
            {
                Put("ERR 0x08..0x77\r\n");
                return;
            }
            c->i2cAddr = (uint8_t)a[0];
            I2cComm_SetAddress(c->i2cAddr);
            if (!BmConfig_Save()) { App_SetError(APP_ERR_FLASH); ReplyError(APP_ERR_FLASH); return; }
        }
        Put("OK addr=0x");
        PutHex2(c->i2cAddr);
        Put("\r\n");
    }
    else if (strcmp(cmd, "neis") == 0)
    {
        if (n == 1)
        {
            if (a[0] != (int32_t)BM_NUM_EIS_CELL_ONLY && a[0] != (int32_t)BM_NUM_EIS_WITH_STACK)
            {
                Put("ERR 1 or 2\r\n");
                return;
            }
            if (EIS_IsBusy()) { ReplyError(APP_ERR_BUSY); return; }
            c->numEis = (uint8_t)a[0];
            if (!BmConfig_Save()) { App_SetError(APP_ERR_FLASH); ReplyError(APP_ERR_FLASH); return; }
        }
        PutKV("OK neis=", c->numEis);
        Put("\r\n");
    }
    else if (strcmp(cmd, "izero") == 0)
    {
        const int32_t z = CellMeas_ZeroCurrent();
        if (z != c->iZeroMv)
        {
            PutKV("ERR izero=", z);
            Put(" mV implausible (current flowing?), kept\r\n");
            return;
        }
        PutKV("OK izero=", z);
        Put(" mV\r\n");
        if (!BmConfig_Save()) { App_SetError(APP_ERR_FLASH); ReplyError(APP_ERR_FLASH); }
    }
    else if (strcmp(cmd, "recal") == 0)
    {
        TempSensor_Recalibrate();
        Ok();
    }
    else if (strcmp(cmd, "clear") == 0)
    {
        App_ClearError();
        Ok();
    }
    else if (strcmp(cmd, "rd") == 0)
    {
        uint8_t buf[TLE_RAW_REPLY_MAX_LEN], k, len;
        if (n != (int8_t)RD_CMD_ARGS || EIS_IsBusy())
        {
            Put("ERR rd node reg (not during a sweep)\r\n");
            return;
        }
        len = Tle_ReadRaw((uint8_t)a[0], (uint8_t)a[1], buf, sizeof(buf));
        PutKV("OK n=", len);
        for (k = 0u; k < len; k++)
        {
            PutChar(' ');
            PutHex2(buf[k]);
        }
        Put("\r\n");
    }
    else if (strcmp(cmd, "log") == 0)
    {
        if (n == 1)
        {
            logPeriodMs = (a[0] <= 0) ? 0u : ((a[0] < (int32_t)LOG_MIN_PERIOD_MS) ? LOG_MIN_PERIOD_MS : (uint32_t)a[0]);
            logLastTick = HAL_GetTick();
        }
        PutKV("OK log=", (int32_t)logPeriodMs);
        Put("\r\nLOG;ms;u_mV;i_mA;t_degC;status;error;progress\r\n");
    }
    else
    {
        Put("ERR ? s m conz eis abort res addr neis izero recal clear rd log\r\n");
    }
}

/* ---- public API -------------------------------------------------------------------- */
void UartComm_Init(void)
{
    USART1->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    USART1->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
    HAL_NVIC_SetPriority(USART1_IRQn, UART_IRQ_PRIORITY, 0u);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    Put("\r\nBM debug console\r\n");
}

void UartComm_Update(void)
{
    while (rxTail != rxHead)
    {
        char ch = (char)rxBuf[rxTail];
        rxTail = (uint8_t)((rxTail + 1u) & (RX_BUF_SIZE - 1u));

        if (ch == '\r' || ch == '\n')
        {
            line[lineLen] = '\0';
            if (lineLen > 0u)
                ProcessLine(line);
            lineLen = 0u;
        }
        else if (ch == '\b' || ch == ASCII_DEL)
        {
            if (lineLen > 0u)
                lineLen--;
        }
        else if (lineLen < LINE_MAX && !(lineLen == 0u && ch == ' '))
        {
            if (ch >= 'A' && ch <= 'Z')
                ch = (char)(ch - 'A' + 'a');
            line[lineLen++] = ch;
        }
    }

    /* result table, a line at a time as the TX buffer empties */
    while (resPrint != RES_PRINT_NONE && TxFree() >= RES_LINE_SPACE)
    {
        PrintResultLine(resPrint);
        if (resPrint >= EIS_GetLastConfig()->fStop || resPrint >= EIS_FREQ_INDEX_MAX)
        {
            Put("OK end\r\n");
            resPrint = RES_PRINT_NONE;
        }
        else
            resPrint++;
    }

    if (logPeriodMs != 0u && (HAL_GetTick() - logLastTick) >= logPeriodMs && TxFree() >= RES_LINE_SPACE)
    {
        logLastTick = HAL_GetTick();
        PutKV("LOG;", (int32_t)HAL_GetTick());
        PutKV(";", CellMeas_GetVoltageMv());
        PutKV(";", CellMeas_GetCurrentMa());
        Put(";");
        PutCenti(TempCenti());
        PutKV(";", App_GetStatus());
        PutKV(";", (int32_t)App_GetErrorCode());
        PutKV(";", EIS_GetProgress());
        Put("\r\n");
    }
}

/* NOTE: if "USART1 global interrupt" is enabled in CubeMX later, delete the
 * handler CubeMX generates in stm32c0xx_it.c (this module does not use the HAL
 * UART interrupt driver). */
void USART1_IRQHandler(void)
{
    const uint32_t isr = USART1->ISR;

    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE))
        USART1->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;

    if (isr & USART_ISR_RXNE_RXFNE)
    {
        const uint8_t b = (uint8_t)USART1->RDR;
        const uint8_t next = (uint8_t)((rxHead + 1u) & (RX_BUF_SIZE - 1u));
        if (next != rxTail)
        {
            rxBuf[rxHead] = b;
            rxHead = next;
        }
    }

    if ((USART1->CR1 & USART_CR1_TXEIE_TXFNFIE) && (isr & USART_ISR_TXE_TXFNF))
    {
        if (txTail != txHead)
        {
            USART1->TDR = txBuf[txTail];
            txTail = (uint16_t)((txTail + 1u) & (TX_BUF_SIZE - 1u));
        }
        else
            USART1->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
    }
}

#else  /* !BM_ENABLE_UART */

void UartComm_Init(void)   { }
void UartComm_Update(void) { }

#endif /* BM_ENABLE_UART */

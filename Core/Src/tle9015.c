/**
 ******************************************************************************
 * @file    tle9015.c
 * @brief   See tle9015.h.
 *
 * Differences to the Zellhalter firmware (same protocol, more robust I/O):
 *  - USART2 registers are used directly (no HAL UART driver): exact timing at
 *    1 MBaud, small code, every wait has a timeout (no hang on a dead link).
 *  - A read collects the reply in one go (the old code received byte by byte
 *    with an abort in between, which can lose bytes at 1 MBaud) and checks that
 *    the reply belongs to the requested register; up to 3 attempts.
 *  - Microsecond delays use SysTick (the Cortex-M0+ has no DWT cycle counter).
 ******************************************************************************
 */
#include "tle9015.h"
#include <string.h>

#define TLE_BAUD               1000000u
#define US_PER_S               1000000u

/* ---- frame format (protocol values as in the Zellhalter firmware) ---- */
#define TLE_SYNC_BYTE             0x02u   /* first byte of every request frame */
#define TLE_WRITE_BIT             0x80u   /* ID byte bit 7: 1 = write, 0 = read */
#define TLE_NODE_MASK             0x7Fu   /* ID byte bits 6..0: node number */
#define TLE_BROADCAST_NODE        0x00u   /* node 0: taken by the next unnumbered node (enumeration) */
#define TLE_WRITE_FRAME_LEN          6u   /* sync, id, reg, data_hi, data_lo, crc */
#define TLE_READ_FRAME_LEN           4u   /* sync, id, reg, crc */
#define TLE_REPLY_LEN                5u   /* id, reg, data_hi, data_lo, crc */
#define TLE_REPLY_MIN_LEN            4u   /* id .. data_lo (the reply CRC is not checked) */
#define TLE_REPLY_IDX_REG            1u
#define TLE_REPLY_IDX_DATA_HI        2u
#define TLE_REPLY_IDX_DATA_LO        3u

/* CRC8 over the frame (without the CRC byte itself) */
#define CRC_POLY                  0x1Du
#define CRC_INIT                  0x00u
#define CRC_MSB                   0x80u
#define CRC_FINAL_XOR_WRITE       0x10u   /* XOR applied to the CRC of a write frame */
#define CRC_FINAL_XOR_READ        0xF1u   /* XOR applied to the CRC of a read frame  */

/* wake-up: a burst of 0x0F bytes on the isoUART */
#define TLE_WAKEUP_BYTE           0x0Fu
#define TLE_WAKEUP_BYTE_COUNT        6u
#define TLE_WAKEUP_SETTLE_US       500u   /* chain start-up after the wake-up burst */

/* ---- timing ---- */
#define TX_TIMEOUT_US            200u     /* per byte */
#define RX_FIRST_TIMEOUT_US     1000u     /* request -> first reply byte */
#define RX_GAP_TIMEOUT_US        100u     /* between reply bytes (1 byte = 10 us) */
#define READ_ATTEMPTS              3u
#define FRAME_GAP_US              50u     /* pause between frames (as in the old firmware) */
#define READ_RETRY_PAUSE_US      200u     /* pause before repeating a failed read */

/* USART BRR with 8x oversampling (OVER8 = 1): BRR[15:4] = USARTDIV[15:4],
 * BRR[3] = 0, BRR[2:0] = USARTDIV[3:0] >> 1 (RM0490, USART baud rate generation) */
#define BRR_OVER8_HIGH_MASK    0xFFF0u
#define BRR_OVER8_LOW_MASK     0x000Fu

/* ---- timing --------------------------------------------------------------- */
/* SysTick counts down from LOAD at the core clock; HAL keeps it running (1 ms). */
static uint32_t ElapsedTicks(uint32_t *prev)
{
    const uint32_t now  = SysTick->VAL;
    const uint32_t load = SysTick->LOAD + 1u;
    const uint32_t d    = (*prev >= now) ? (*prev - now) : (*prev + load - now);
    *prev = now;
    return d;
}

void Tle_DelayUs(uint32_t us)
{
    const uint32_t target = us * (SystemCoreClock / US_PER_S);
    uint32_t prev = SysTick->VAL, elapsed = 0u;
    while (elapsed < target)
        elapsed += ElapsedTicks(&prev);
}

/* Waits until (USART2->ISR & flag) or the timeout expires. Returns 1 if set. */
static uint8_t WaitFlag(uint32_t flag, uint32_t timeoutUs)
{
    const uint32_t target = timeoutUs * (SystemCoreClock / US_PER_S);
    uint32_t prev = SysTick->VAL, elapsed = 0u;
    while ((USART2->ISR & flag) == 0u)
    {
        elapsed += ElapsedTicks(&prev);
        if (elapsed >= target)
            return 0u;
    }
    return 1u;
}

/* ---- byte level ------------------------------------------------------------- */
static uint8_t Crc8(const uint8_t *d, uint8_t n, uint8_t xorOut)
{
    uint8_t crc = CRC_INIT, i, b;
    for (i = 0u; i < n; i++)
    {
        crc ^= d[i];
        for (b = 0u; b < 8u; b++)
            crc = (crc & CRC_MSB) ? (uint8_t)((crc << 1) ^ CRC_POLY) : (uint8_t)(crc << 1);
    }
    return (uint8_t)(crc ^ xorOut);
}

static void TxBytes(const uint8_t *d, uint8_t n)
{
    uint8_t i;
    for (i = 0u; i < n; i++)
    {
        if (!WaitFlag(USART_ISR_TXE_TXFNF, TX_TIMEOUT_US))
            return;
        USART2->TDR = d[i];
    }
    (void)WaitFlag(USART_ISR_TC, TX_TIMEOUT_US);
}

/* Discards everything received so far (e.g. an echo of the request). */
static void FlushRx(void)
{
    USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    while (USART2->ISR & USART_ISR_RXNE_RXFNE)
        (void)USART2->RDR;
}

static uint8_t RxBytes(uint8_t *buf, uint8_t maxLen)
{
    uint8_t n = 0u;
    while (n < maxLen)
    {
        if (!WaitFlag(USART_ISR_RXNE_RXFNE, (n == 0u) ? RX_FIRST_TIMEOUT_US : RX_GAP_TIMEOUT_US))
            break;
        buf[n++] = (uint8_t)USART2->RDR;
    }
    USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    return n;
}

static void SendWriteFrame(uint8_t id, uint8_t reg, uint16_t value)
{
    uint8_t f[TLE_WRITE_FRAME_LEN];
    f[0] = TLE_SYNC_BYTE;
    f[1] = id;
    f[2] = reg;
    f[3] = (uint8_t)(value >> 8);
    f[4] = (uint8_t)value;
    f[5] = Crc8(f, TLE_WRITE_FRAME_LEN - 1u, CRC_FINAL_XOR_WRITE);
    TxBytes(f, TLE_WRITE_FRAME_LEN);
    Tle_DelayUs(FRAME_GAP_US);
}

/* ---- public API ------------------------------------------------------------------ */
void Tle_Init(void)
{
    const uint32_t pclk = HAL_RCC_GetPCLK1Freq();

    USART2->CR1 &= ~USART_CR1_UE;               /* MSBFIRST / BRR only writable while disabled */
    USART2->CR2 |= USART_CR2_MSBFIRST;
    if (pclk >= 16u * TLE_BAUD)
    {
        USART2->CR1 &= ~USART_CR1_OVER8;
        USART2->BRR = (pclk + TLE_BAUD / 2u) / TLE_BAUD;
    }
    else
    {
        /* below 16 MHz PCLK: 8x oversampling */
        const uint32_t div = (2u * pclk + TLE_BAUD / 2u) / TLE_BAUD;
        USART2->CR1 |= USART_CR1_OVER8;
        USART2->BRR = (div & BRR_OVER8_HIGH_MASK) | ((div & BRR_OVER8_LOW_MASK) >> 1);
    }
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void Tle_Wakeup(void)
{
    uint8_t wake[TLE_WAKEUP_BYTE_COUNT];
    memset(wake, TLE_WAKEUP_BYTE, sizeof(wake));
    TxBytes(wake, TLE_WAKEUP_BYTE_COUNT);
    Tle_DelayUs(TLE_WAKEUP_SETTLE_US);
}

void Tle_Enumerate(uint8_t numNodes)
{
    uint8_t n;

    /* clear old node numbers */
    for (n = 1u; n <= numNodes; n++)
        SendWriteFrame((uint8_t)(TLE_WRITE_BIT | n), TLE_REG_NODE_ID, TLE_NODE_ID_CLEAR);

    /* each frame to node 0 is taken by the next not yet numbered node in the
     * chain; the last one is flagged as final node */
    for (n = 1u; n <= numNodes; n++)
        SendWriteFrame(TLE_WRITE_BIT | TLE_BROADCAST_NODE, TLE_REG_NODE_ID,
                       (uint16_t)(n | ((n == numNodes) ? TLE_NODE_ID_FINAL_NODE : 0u)));
}

void Tle_Write(uint8_t node, uint8_t reg, uint16_t value)
{
    SendWriteFrame((uint8_t)(TLE_WRITE_BIT | node), reg, value);
}

uint8_t Tle_ReadRaw(uint8_t node, uint8_t reg, uint8_t *buf, uint8_t maxLen)
{
    uint8_t f[TLE_READ_FRAME_LEN];
    f[0] = TLE_SYNC_BYTE;
    f[1] = (uint8_t)(node & TLE_NODE_MASK);
    f[2] = reg;
    f[3] = Crc8(f, TLE_READ_FRAME_LEN - 1u, CRC_FINAL_XOR_READ);
    TxBytes(f, TLE_READ_FRAME_LEN);
    FlushRx();
    return RxBytes(buf, maxLen);
}

uint8_t Tle_Read(uint8_t node, uint8_t reg, uint16_t *value)
{
    uint8_t buf[TLE_REPLY_LEN], attempt;

    for (attempt = 0u; attempt < READ_ATTEMPTS; attempt++)
    {
        /* reply: [id][reg][data_hi][data_lo][crc] */
        if (Tle_ReadRaw(node, reg, buf, TLE_REPLY_LEN) >= TLE_REPLY_MIN_LEN && buf[TLE_REPLY_IDX_REG] == reg)
        {
            *value = (uint16_t)(((uint16_t)buf[TLE_REPLY_IDX_DATA_HI] << 8) | buf[TLE_REPLY_IDX_DATA_LO]);
            Tle_DelayUs(FRAME_GAP_US);
            return 1u;
        }
        Tle_DelayUs(READ_RETRY_PAUSE_US);
    }
    return 0u;
}

/**
 ******************************************************************************
 * @file    bm_config.c
 * @brief   See bm_config.h.
 ******************************************************************************
 */
#include "bm_config.h"
#include <string.h>

#define CFG_PAGE          15u
#define CFG_ADDR          (FLASH_BASE + CFG_PAGE * FLASH_PAGE_SIZE)   /* 0x08007800 */
/* Format version of the stored record. Load() only accepts a record with exactly
 * this version; an erased page reads 0xFFFF and is therefore never taken as valid.
 * Increase it whenever the layout of CfgRecord_t changes, so that a record in
 * the old layout is not misread (the board then starts with the defaults). */
#define CFG_RECORD_VERSION  1u

/* Stored record: exactly one 64-bit double word (the flash programming unit). */
typedef struct
{
    uint16_t version;    /* CFG_RECORD_VERSION */
    uint8_t  i2cAddr;
    uint8_t  numEis;
    int16_t  iZeroMv;
    uint16_t check;      /* ~(sum of the 3 halfwords above) */
} CfgRecord_t;

typedef char cfg_record_is_8_bytes[(sizeof(CfgRecord_t) == 8u) ? 1 : -1];

static BmConfig_t cfg;

static uint16_t Check(const CfgRecord_t *r)
{
    const uint16_t *h = (const uint16_t *)r;
    return (uint16_t)~(uint16_t)(h[0] + h[1] + h[2]);
}

void BmConfig_Load(void)
{
    CfgRecord_t r;
    memcpy(&r, (const void *)CFG_ADDR, sizeof(r));

    if (r.version == CFG_RECORD_VERSION && r.check == Check(&r) &&
        r.i2cAddr >= BM_I2C_ADDR_MIN && r.i2cAddr <= BM_I2C_ADDR_MAX &&
        (r.numEis == BM_NUM_EIS_CELL_ONLY || r.numEis == BM_NUM_EIS_WITH_STACK))
    {
        cfg.i2cAddr = r.i2cAddr;
        cfg.numEis  = r.numEis;
        cfg.iZeroMv = (r.iZeroMv >= BM_I_ZERO_MIN_MV && r.iZeroMv <= BM_I_ZERO_MAX_MV)
                      ? r.iZeroMv : BM_DEFAULT_I_ZERO_MV;
    }
    else
    {
        cfg.i2cAddr = BM_DEFAULT_I2C_ADDR;
        cfg.numEis  = BM_DEFAULT_NUM_EIS;
        cfg.iZeroMv = BM_DEFAULT_I_ZERO_MV;
    }
}

BmConfig_t *BmConfig_Get(void)
{
    return &cfg;
}

/* ---- flash access directly via the registers ---------------------------------
 * Same procedure as HAL_FLASHEx_Erase / HAL_FLASH_Program (standard 64-bit
 * programming, runs from flash, no RAM function needed). Using the HAL would
 * also link FLASH_Program_Fast, which ST places in RAM as executable code. */
#define FLASH_SR_ERRORS  (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | \
                          FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR | \
                          FLASH_SR_RDERR | FLASH_SR_OPTVERR)

/* timeout of WaitReady() in polling loops: a page erase takes ~22 ms, the loop
 * runs at least ~1 M times per 100 ms at 48 MHz -> generous margin */
#define FLASH_WAIT_LOOPS  2000000u

static uint8_t WaitReady(void)
{
    uint32_t guard = FLASH_WAIT_LOOPS;
    while ((FLASH->SR & (FLASH_SR_BSY1 | FLASH_SR_CFGBSY)) != 0u)
    {
        if (--guard == 0u)
            return 0u;
    }
    return ((FLASH->SR & FLASH_SR_ERRORS) == 0u) ? 1u : 0u;
}

static uint8_t ErasePageAndProgram(const uint32_t w[2])
{
    uint8_t ok;

    FLASH->KEYR = FLASH_KEY1;                    /* unlock sequence (RM0490, FLASH_KEYR) */
    FLASH->KEYR = FLASH_KEY2;
    FLASH->SR = FLASH_SR_ERRORS | FLASH_SR_EOP;   /* clear old flags */

    ok = WaitReady();
    if (ok)
    {
        FLASH->CR = FLASH_CR_PER | (CFG_PAGE << FLASH_CR_PNB_Pos);
        FLASH->CR |= FLASH_CR_STRT;
        ok = WaitReady();
        FLASH->CR &= ~FLASH_CR_PER;
    }
    if (ok)
    {
        FLASH->CR |= FLASH_CR_PG;
        *(volatile uint32_t *)CFG_ADDR        = w[0];   /* a double word: two word writes */
        *(volatile uint32_t *)(CFG_ADDR + 4u) = w[1];
        ok = WaitReady();
        FLASH->CR &= ~FLASH_CR_PG;
    }
    FLASH->SR = FLASH_SR_ERRORS | FLASH_SR_EOP;
    FLASH->CR |= FLASH_CR_LOCK;
    return ok;
}

uint8_t BmConfig_Save(void)
{
    CfgRecord_t r;
    uint32_t w[2];

    r.version = CFG_RECORD_VERSION;
    r.i2cAddr = cfg.i2cAddr;
    r.numEis  = cfg.numEis;
    r.iZeroMv = cfg.iZeroMv;
    r.check   = Check(&r);

    /* unchanged: do not erase/program the page again (flash endurance) */
    if (memcmp((const void *)CFG_ADDR, &r, sizeof(r)) == 0)
        return 1u;

    memcpy(w, &r, sizeof(w));
    if (!ErasePageAndProgram(w))
        return 0u;
    return (memcmp((const void *)CFG_ADDR, &r, sizeof(r)) == 0) ? 1u : 0u;
}

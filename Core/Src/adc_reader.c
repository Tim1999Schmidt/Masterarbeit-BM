/**
 ******************************************************************************
 * @file    adc_reader.c
 * @brief   See adc_reader.h.
 ******************************************************************************
 */
#include "adc_reader.h"

extern ADC_HandleTypeDef hadc1; /* initialised by MX_ADC1_Init() in main.c */

#define ADC_CHANNEL_NONE_SELECTED   0xFFFFFFFFu

static uint32_t selectedChannel = ADC_CHANNEL_NONE_SELECTED;

/* Leaves exactly `channel` in the ADC's (fixed) channel sequencer. Must be
 * called while the ADC is disabled - true here, because every read below
 * disables it again at the end (the internal-channel path bit for VREFINT in
 * ADC_CCR can only be written with ADEN = 0). */
static void SelectOnly(uint32_t channel)
{
    static const uint32_t usedChannels[] = { ADC_READER_CH_TEMP, ADC_READER_CH_CURRENT,
                                             ADC_READER_CH_VOLTAGE, ADC_READER_CH_VREFINT };
    ADC_ChannelConfTypeDef cfg = {0};
    uint32_t i;

    /* ADC_RANK_NONE removes a channel from the sequencer ... */
    cfg.Rank = ADC_RANK_NONE;
    for (i = 0; i < (sizeof(usedChannels) / sizeof(usedChannels[0])); i++)
    {
        if (usedChannels[i] != channel)
        {
            cfg.Channel = usedChannels[i];
            (void)HAL_ADC_ConfigChannel(&hadc1, &cfg);
        }
    }

    /* ... ADC_RANK_CHANNEL_NUMBER adds one. */
    cfg.Channel = channel;
    cfg.Rank = ADC_RANK_CHANNEL_NUMBER;
    (void)HAL_ADC_ConfigChannel(&hadc1, &cfg);

    selectedChannel = channel;
}

void AdcReader_Init(void)
{
    /* Built-in offset calibration of the ADC. Must run while the ADC is
     * disabled, i.e. after MX_ADC1_Init() and before the first conversion. */
    (void)HAL_ADCEx_Calibration_Start(&hadc1);
}

uint32_t AdcReader_ReadBlocking(uint32_t adcChannel)
{
    uint32_t value;

    if (adcChannel != selectedChannel)
        SelectOnly(adcChannel);

    /* Enable - convert - disable directly via the registers: same sequence as
     * HAL_ADC_Start/PollForConversion/Stop, without their flash cost. */
    ADC1->ISR = ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0u) { }
    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0u) { }
    value = ADC1->DR;
    ADC1->CR |= ADC_CR_ADDIS;
    while ((ADC1->CR & ADC_CR_ADEN) != 0u) { }

    return value;
}

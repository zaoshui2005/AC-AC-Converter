#include "pfc_adc_watchdog.h"
#include "hrtim.h"

/** PFC的HRTIM Timer E和Timer F四路功率输出位掩码。 */
#define PFC_PWM_OUTPUTS \
    (HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2 | \
     HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2)

volatile uint32_t pfc_adc_fault;

/**
* @brief          将浮点ADC计数限制到12位有效范围
* @param[in]      value 待限制的ADC计数值
* @retval         uint16_t 限制后的ADC计数值
*/
static uint16_t ClampAdcCount(float value)
{
    if (value <= 0.0f) {
        return 0U;
    }
    if (value >= PFC_ADC_FULL_SCALE) {
        return (uint16_t)PFC_ADC_FULL_SCALE;
    }
    return (uint16_t)(value + 0.5f);
}

/**
* @brief          在ADC中断中关闭PFC输出并锁存故障
* @param[in]      fault 要锁存的故障标志
* @retval         none
*/
static void TripFromISR(uint32_t fault)
{
    /* ADC硬件看门狗触发后，在ADC中断中立即关闭PFC四路输出。 */
    hhrtim1.Instance->sCommonRegs.ODISR = PFC_PWM_OUTPUTS;
    pfc_adc_fault |= fault;
}
ADC_AnalogWDGConfTypeDef watchdog = {0};

/**
* @brief          关闭并清除ADC1全部硬件看门狗中断状态
* @param[in]      none
* @retval         none
*/
void PFC_ADC_Watchdog_Disable(void)
{
    __HAL_ADC_DISABLE_IT(
        &hadc1,
        ADC_IT_AWD1 | ADC_IT_AWD2 | ADC_IT_AWD3);

    CLEAR_BIT(
        hadc1.State,
        HAL_ADC_STATE_AWD1 |
        HAL_ADC_STATE_AWD2 |
        HAL_ADC_STATE_AWD3);

    __HAL_ADC_CLEAR_FLAG(
        &hadc1,
        ADC_FLAG_AWD1 | ADC_FLAG_AWD2 | ADC_FLAG_AWD3);
}

/**
* @brief          配置PFC输入过流和母线过压ADC硬件看门狗
* @param[in]      none
* @retval         HAL_StatusTypeDef HAL执行状态
*/
HAL_StatusTypeDef PFC_ADC_Watchdog_Init(void)
{
    // ADC_AnalogWDGConfTypeDef watchdog = {0};
    HAL_StatusTypeDef status;
    HAL_StatusTypeDef restart_status;
    float current_limit_count;
    uint32_t current_low_count;
    uint32_t current_high_count;

    if (pfc_adc_state.calibration.ready == 0U) {
        return HAL_BUSY;
    }

    /*
     * 暂停ADC转换，确保看门狗配置过程没有正在执行的转换。
     */
    status = HAL_ADC_Stop_DMA(&hadc1);
    if (status != HAL_OK) {
        return status;
    }

    current_limit_count =
        PFC_ADC_CURRENT_LIMIT_A /
        PFC_ADC_IIN_A_PER_COUNT;

    current_low_count = ClampAdcCount(
        pfc_adc_state.calibration.current_offset_count -
        current_limit_count);

    current_high_count = ClampAdcCount(
        pfc_adc_state.calibration.current_offset_count +
        current_limit_count);

    /*
     * IOC已经配置：
     * AWD1、规则组、ADC_CHANNEL_4、AWD1中断。
     *
     * 这里仅更新上电校准后才能确定的过流窗口。
     */
    MODIFY_REG(
        hadc1.Instance->TR1,
        ADC_TR1_LT1 |
        ADC_TR1_HT1 |
        ADC_TR1_AWDFILT,
        (current_low_count << ADC_TR1_LT1_Pos) |
        (current_high_count << ADC_TR1_HT1_Pos) |
        ADC_AWD_FILTERING_3SAMPLES);//看门狗触发次数

    /* 清除配置前可能残留的AWD1标志。 */
    __HAL_ADC_CLEAR_FLAG(
        &hadc1,
        ADC_FLAG_AWD1);

    /* 确保AWD1硬件中断已打开。 */
    __HAL_ADC_ENABLE_IT(
        &hadc1,
        ADC_IT_AWD1);

    /*
     * AWD2仍由User代码配置，因为其母线过压阈值需要物理量换算。
     */
    watchdog.WatchdogNumber =
        ADC_ANALOGWATCHDOG_2;

    watchdog.WatchdogMode =
        ADC_ANALOGWATCHDOG_SINGLE_REG;

    watchdog.Channel =
        ADC_CHANNEL_2;

    watchdog.ITMode = ENABLE;//母线电压看门狗使能
    watchdog.LowThreshold = 0U;

    watchdog.HighThreshold = ClampAdcCount(
        PFC_ADC_VBUS_OFFSET_COUNT +
        PFC_ADC_BUS_OVERVOLTAGE_V /
        PFC_ADC_VBUS_V_PER_COUNT);

    watchdog.FilteringConfig =
        ADC_AWD_FILTERING_3SAMPLES;

    status = HAL_ADC_AnalogWDGConfig(
        &hadc1,
        &watchdog);

    restart_status = HAL_ADC_Start_DMA(
        &hadc1,
        (uint32_t *)pfc_adc_state.dma.sample,
        PFC_ADC_DMA_LENGTH);

    if (status == HAL_OK) {
        status = restart_status;
    }

    return status;
}

/**
* @brief          清除PFC ADC软件故障锁存标志
* @param[in]      none
* @retval         none
*/
void PFC_ADC_Watchdog_ClearFault(void)
{
    pfc_adc_fault = PFC_ADC_FAULT_NONE;
}

/**
* @brief          处理ADC1硬件看门狗1输入过流事件
* @param[in]      hadc ADC句柄地址
* @retval         none
*/
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        CLEAR_BIT(hadc->State, HAL_ADC_STATE_AWD1);
        TripFromISR(PFC_ADC_FAULT_OVERCURRENT);
    }
}

/**
* @brief          处理ADC1硬件看门狗2母线过压事件
* @param[in]      hadc ADC句柄地址
* @retval         none
*/
void HAL_ADCEx_LevelOutOfWindow2Callback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        CLEAR_BIT(hadc->State, HAL_ADC_STATE_AWD2);
        TripFromISR(PFC_ADC_FAULT_BUS_OVERVOLTAGE);
    }
}

/**
* @brief          处理ADC1采样或DMA错误事件
* @param[in]      hadc ADC句柄地址
* @retval         none
*/
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        TripFromISR(PFC_ADC_FAULT_ADC_ERROR);
    }
}
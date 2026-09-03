#include "inverter_adc_watchdog.h"
#include "hrtim.h"
/** 三相逆变HRTIM Timer A、B、C六路输出位掩码。 */
#define INVERTER_PWM_OUTPUTS \
    (HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | \
     HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | \
     HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2)

/** 三相逆变ADC看门狗对外运行状态。 */
volatile Inverter_ADC_WatchdogStateTypeDef inverter_adc_watchdog_state;

/**
 * @brief          将浮点ADC计数限制到12位有效范围
 * @param[in]      value 待限制的ADC计数值
 * @retval         uint16_t 限制并四舍五入后的ADC计数
 */
static uint16_t Inverter_ADC_ClampCount(float value)
{
    if (value <= 0.0f) {
        return 0U;
    }

    if (value >= INVERTER_ADC_FULL_SCALE) {
        return (uint16_t)INVERTER_ADC_FULL_SCALE;
    }

    return (uint16_t)(value + 0.5f);
}

/**
 * @brief          在ADC2 AWD1中断中关闭逆变输出并锁存故障
 * @param[in]      fault 要锁存的故障位
 * @retval         none
 */
static void Inverter_ADC_Watchdog_TripFromISR(
    uint32_t fault)
{
    /*
     * 仅关闭Timer A、B、C六路三相逆变输出。
     * 不操作PFC使用的Timer E、F输出。
     */
    hhrtim1.Instance->sCommonRegs.ODISR =
        INVERTER_PWM_OUTPUTS;

    inverter_adc_watchdog_state.fault |= fault;
    inverter_adc_watchdog_state.trip_count++;

    /* 持续越界时只触发一次，故障清除后再重新允许AWD1。 */
    __HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_AWD1);
}

/**
 * @brief          根据校准零点、增益和电流上限配置ADC2 AWD1实际阈值
 * @param[in]      none
 * @retval         HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef Inverter_ADC_Watchdog_Init(void)
{
    float limit_count_1;
    float limit_count_2;
    uint16_t low_1;
    uint16_t high_1;
    uint16_t low_2;
    uint16_t high_2;
    uint16_t common_low;
    uint16_t common_high;

    if (inverter_adc_state.zero_calibration.ready == 0U) {
        return HAL_ERROR;
    }

    /*
     * ADC2 AWD1在IOC中配置为ALL_REG，因此ADC2_IN3和ADC2_IN4
     * 共用TR1中的一组上下限。先分别计算两路安全窗口。
     */
    limit_count_1 =
        INVERTER_ADC_CURRENT_LIMIT_A /
        INVERTER_ADC_CURRENT_1_A_PER_COUNT;

    limit_count_2 =
        INVERTER_ADC_CURRENT_LIMIT_A /
        INVERTER_ADC_CURRENT_2_A_PER_COUNT;

    low_1 = Inverter_ADC_ClampCount(
        inverter_adc_state.zero_calibration.current_1_zero_count -
        limit_count_1);

    high_1 = Inverter_ADC_ClampCount(
        inverter_adc_state.zero_calibration.current_1_zero_count +
        limit_count_1);

    low_2 = Inverter_ADC_ClampCount(
        inverter_adc_state.zero_calibration.current_2_zero_count -
        limit_count_2);

    high_2 = Inverter_ADC_ClampCount(
        inverter_adc_state.zero_calibration.current_2_zero_count +
        limit_count_2);

    /*
     * 使用两个安全窗口的交集作为AWD1公共窗口。
     * 这样任一路达到自身限值都会触发；代价是两路偏置差异较大时
     * 公共窗口会变窄，因此必须保证两路采样板标定值接近。
     */
    common_low = (low_1 > low_2) ? low_1 : low_2;
    common_high = (high_1 < high_2) ? high_1 : high_2;

    if (common_low >= common_high) {
        return HAL_ERROR;
    }

    inverter_adc_watchdog_state =
        (Inverter_ADC_WatchdogStateTypeDef){0};

    inverter_adc_watchdog_state.current_1_low_count = low_1;
    inverter_adc_watchdog_state.current_1_high_count = high_1;
    inverter_adc_watchdog_state.current_2_low_count = low_2;
    inverter_adc_watchdog_state.current_2_high_count = high_2;
    inverter_adc_watchdog_state.common_low_count = common_low;
    inverter_adc_watchdog_state.common_high_count = common_high;

    /*
     * IOC已经完成AWD1、ALL_REG和中断使能配置。
     * 这里与PFC电流看门狗一样，只更新运行时实际阈值。
     */
    MODIFY_REG(
        hadc2.Instance->TR1,
        ADC_TR1_LT1 |
        ADC_TR1_HT1 |
        ADC_TR1_AWDFILT,
        ((uint32_t)common_low << ADC_TR1_LT1_Pos) |
        ((uint32_t)common_high << ADC_TR1_HT1_Pos) |
        ADC_AWD_FILTERING_3SAMPLES);

    CLEAR_BIT(hadc2.State, HAL_ADC_STATE_AWD1);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_AWD1);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_AWD1);

    return HAL_OK;
}

/**
 * @brief          处理ADC2 AWD1中断标志
 * @param[in]      none
 * @retval         none
 */
void Inverter_ADC_Watchdog_IRQHandler(void)
{
    /* 共享ADC1_2中断中只处理ADC2的AWD1事件。 */
    if ((__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_AWD1) == RESET) ||
        (__HAL_ADC_GET_IT_SOURCE(&hadc2, ADC_IT_AWD1) == RESET)) {
        return;
    }

    /*
     * 在HAL_ADC_IRQHandler(&hadc2)之前清除ADC2 AWD1标志，
     * 避免进入PFC文件中全局的HAL_ADC_LevelOutOfWindowCallback()。
     */
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_AWD1);
    CLEAR_BIT(hadc2.State, HAL_ADC_STATE_AWD1);

    Inverter_ADC_Watchdog_TripFromISR(
        INVERTER_ADC_FAULT_OVERCURRENT);
}

/**
 * @brief          清除逆变ADC看门狗故障并重新允许ADC2 AWD1中断
 * @param[in]      none
 * @retval         none
 */
void Inverter_ADC_Watchdog_ClearFault(void)
{
    inverter_adc_watchdog_state.fault =
        INVERTER_ADC_FAULT_NONE;

    CLEAR_BIT(hadc2.State, HAL_ADC_STATE_AWD1);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_AWD1);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_AWD1);
}

/**
 * @brief          判断逆变ADC看门狗是否锁存故障
 * @param[in]      none
 * @retval         uint8_t 故障状态
 */
uint8_t Inverter_ADC_Watchdog_IsFaulted(void)
{
    return (inverter_adc_watchdog_state.fault !=
            INVERTER_ADC_FAULT_NONE) ? 1U : 0U;
}

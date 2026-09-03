#include "inverter_svpwm.h"
/** SVPWM运行状态，可在调试器中观察。 */
volatile Inverter_SVPWM_StateTypeDef inverter_svpwm_state;

/**
 * @brief 将浮点数限制到指定范围
 */
static float Inverter_SVPWM_ClampFloat(float value,
                                       float minimum,
                                       float maximum)
{
    if (value > maximum) {
        return maximum;
    }

    if (value < minimum) {
        return minimum;
    }

    return value;
}

/**
 * @brief 根据三相指令的大小关系生成调试用扇区号
 */
static uint8_t Inverter_SVPWM_GetSector(float m_a,
                                        float m_b,
                                        float m_c)
{
    if (m_a >= m_b) {
        if (m_b >= m_c) {
            return 1U; /* A >= B >= C */
        }

        if (m_a >= m_c) {
            return 6U; /* A >= C > B */
        }

        return 5U;     /* C > A >= B */
    }

    if (m_a >= m_c) {
        return 2U;     /* B > A >= C */
    }

    if (m_b >= m_c) {
        return 3U;     /* B >= C > A */
    }

    return 4U;         /* C > B > A */
}

/**
 * @brief 将连续SVPWM上管占空比换算为中心对齐CMP1值
 *
 * @note 当前IOC生成的HRTIM配置满足duty=(PER-CMP1)/PER。
 *       SVPWM不使用0%/100%占空比，三相均保留比较值边界。
 */
static uint32_t Inverter_SVPWM_DutyToCompare(float duty)
{
    float compare_f;
    uint32_t compare;
    uint32_t period;

    period = inverter_svpwm_state.period_counts;
    duty = Inverter_SVPWM_ClampFloat(duty, 0.0f, 1.0f);

    compare_f = (1.0f - duty) * (float)period;
    compare = (uint32_t)(compare_f + 0.5f);

    if (compare < INVERTER_SVPWM_COMPARE_GUARD_COUNTS) {
        compare = INVERTER_SVPWM_COMPARE_GUARD_COUNTS;
    }

    if (compare >
        (period - INVERTER_SVPWM_COMPARE_GUARD_COUNTS)) {
        compare =
            period - INVERTER_SVPWM_COMPARE_GUARD_COUNTS;
    }

    return compare;
}

/**
 * @brief 写入Timer A/B/C的CMP1预装载寄存器
 */
static void Inverter_SVPWM_WritePreload(uint32_t compare_a,
                                         uint32_t compare_b,
                                         uint32_t compare_c)
{
    __HAL_HRTIM_SETCOMPARE(
        &hhrtim1,
        HRTIM_TIMERINDEX_TIMER_A,
        HRTIM_COMPAREUNIT_1,
        compare_a);

    __HAL_HRTIM_SETCOMPARE(
        &hhrtim1,
        HRTIM_TIMERINDEX_TIMER_B,
        HRTIM_COMPAREUNIT_1,
        compare_b);

    __HAL_HRTIM_SETCOMPARE(
        &hhrtim1,
        HRTIM_TIMERINDEX_TIMER_C,
        HRTIM_COMPAREUNIT_1,
        compare_c);
}

/**
 * @brief 初始化Timer A/B/C安全比较值和SVPWM状态
 */
HAL_StatusTypeDef Inverter_SVPWM_Init(void)
{
    HAL_StatusTypeDef status;
    uint32_t period_a;
    uint32_t period_b;
    uint32_t period_c;
    uint32_t compare_half;

    hhrtim1.Instance->sCommonRegs.ODISR =
        INVERTER_SVPWM_ALL_OUTPUTS;

    period_a =
        hhrtim1.Instance
            ->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A]
            .PERxR;
    period_b =
        hhrtim1.Instance
            ->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B]
            .PERxR;
    period_c =
        hhrtim1.Instance
            ->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C]
            .PERxR;

    if ((period_a <=
         (2U * INVERTER_SVPWM_COMPARE_GUARD_COUNTS)) ||
        (period_a != period_b) ||
        (period_a != period_c)) {
        return HAL_ERROR;
    }

    inverter_svpwm_state =
        (Inverter_SVPWM_StateTypeDef){0};
    inverter_svpwm_state.period_counts = period_a;
    inverter_svpwm_state.modulation_scale = 1.0f;
    inverter_svpwm_state.duty_a_next = 0.5f;
    inverter_svpwm_state.duty_b_next = 0.5f;
    inverter_svpwm_state.duty_c_next = 0.5f;

    compare_half = period_a / 2U;
    inverter_svpwm_state.compare_a_next = compare_half;
    inverter_svpwm_state.compare_b_next = compare_half;
    inverter_svpwm_state.compare_c_next = compare_half;

    Inverter_SVPWM_WritePreload(
        compare_half,
        compare_half,
        compare_half);

    status = HAL_HRTIM_SoftwareUpdate(
        &hhrtim1,
        HRTIM_TIMERUPDATE_A |
        HRTIM_TIMERUPDATE_B |
        HRTIM_TIMERUPDATE_C);
    if (status != HAL_OK) {
        return status;
    }

    inverter_svpwm_state.initialized = 1U;

    return HAL_OK;
}

/**
 * @brief 启动Timer A/B/C计数器
 */
HAL_StatusTypeDef Inverter_SVPWM_StartCounters(void)
{
    HAL_StatusTypeDef status;

    if (inverter_svpwm_state.initialized == 0U) {
        return HAL_ERROR;
    }

    status = HAL_HRTIM_WaveformCounterStart(
        &hhrtim1,
        INVERTER_SVPWM_COUNTERS);
    if (status == HAL_OK) {
        inverter_svpwm_state.counters_running = 1U;
    }

    return status;
}

/**
 * @brief 以三相50%同相安全状态使能六路互补输出
 */
HAL_StatusTypeDef Inverter_SVPWM_Enable(void)
{
    HAL_StatusTypeDef status;
    uint32_t compare_half;

    if ((inverter_svpwm_state.initialized == 0U) ||
        (inverter_svpwm_state.counters_running == 0U) ||
        (inverter_svpwm_state.outputs_enabled != 0U)) {
        return HAL_ERROR;
    }

    compare_half = inverter_svpwm_state.period_counts / 2U;
    Inverter_SVPWM_WritePreload(
        compare_half,
        compare_half,
        compare_half);

    inverter_svpwm_state.duty_a_next = 0.5f;
    inverter_svpwm_state.duty_b_next = 0.5f;
    inverter_svpwm_state.duty_c_next = 0.5f;
    inverter_svpwm_state.compare_a_next = compare_half;
    inverter_svpwm_state.compare_b_next = compare_half;
    inverter_svpwm_state.compare_c_next = compare_half;

    status = HAL_HRTIM_WaveformOutputStart(
        &hhrtim1,
        INVERTER_SVPWM_ALL_OUTPUTS);
    if (status == HAL_OK) {
        inverter_svpwm_state.outputs_enabled = 1U;
    }

    return status;
}

/**
 * @brief 清除SVPWM调制状态并保留HRTIM运行状态
 */
void Inverter_SVPWM_Reset(void)
{
    uint8_t initialized;
    uint8_t counters_running;
    uint8_t outputs_enabled;
    uint32_t period;

    initialized = inverter_svpwm_state.initialized;
    counters_running = inverter_svpwm_state.counters_running;
    outputs_enabled = inverter_svpwm_state.outputs_enabled;
    period = inverter_svpwm_state.period_counts;

    inverter_svpwm_state =
        (Inverter_SVPWM_StateTypeDef){0};
    inverter_svpwm_state.initialized = initialized;
    inverter_svpwm_state.counters_running = counters_running;
    inverter_svpwm_state.outputs_enabled = outputs_enabled;
    inverter_svpwm_state.period_counts = period;
    inverter_svpwm_state.modulation_scale = 1.0f;

    if (period != 0U) {
        inverter_svpwm_state.duty_a_next = 0.5f;
        inverter_svpwm_state.duty_b_next = 0.5f;
        inverter_svpwm_state.duty_c_next = 0.5f;
        inverter_svpwm_state.compare_a_next = period / 2U;
        inverter_svpwm_state.compare_b_next = period / 2U;
        inverter_svpwm_state.compare_c_next = period / 2U;
    }
}

/**
 * @brief 计算连续SVPWM并预装载下一周期三相CMP1
 */
void Inverter_SVPWM_Update(float m_a,
                           float m_b,
                           float m_c)
{
    float phase[3];
    float maximum;
    float minimum;
    float span;
    float span_limit;
    float scale;
    float zero_sequence;
    float duty[3];
    float duty_minimum;
    float duty_maximum;
    uint32_t compare_a;
    uint32_t compare_b;
    uint32_t compare_c;

    if (inverter_svpwm_state.initialized == 0U) {
        return;
    }

    inverter_svpwm_state.requested_m_a = m_a;
    inverter_svpwm_state.requested_m_b = m_b;
    inverter_svpwm_state.requested_m_c = m_c;

    phase[0] = m_a;
    phase[1] = m_b;
    phase[2] = m_c;

    maximum = phase[0];
    minimum = phase[0];

    if (phase[1] > maximum) {
        maximum = phase[1];
    }
    if (phase[2] > maximum) {
        maximum = phase[2];
    }
    if (phase[1] < minimum) {
        minimum = phase[1];
    }
    if (phase[2] < minimum) {
        minimum = phase[2];
    }

    duty_minimum =
        (float)INVERTER_SVPWM_COMPARE_GUARD_COUNTS /
        (float)inverter_svpwm_state.period_counts;
    duty_maximum = 1.0f - duty_minimum;

    /*
     * 连续SVPWM的零序注入会把三相跨度对称放到[-1, 1]内。
     * 同时为三路开关相保留CMP边界，因此可用跨度略小于2。
     */
    span_limit = 2.0f * (1.0f - 2.0f * duty_minimum);
    span = maximum - minimum;
    scale = 1.0f;

    if (span > span_limit) {
        scale = span_limit / span;
        phase[0] *= scale;
        phase[1] *= scale;
        phase[2] *= scale;
        maximum *= scale;
        minimum *= scale;
    }

    zero_sequence = -0.5f * (maximum + minimum);

    duty[0] =
        0.5f * (phase[0] + zero_sequence + 1.0f);
    duty[1] =
        0.5f * (phase[1] + zero_sequence + 1.0f);
    duty[2] =
        0.5f * (phase[2] + zero_sequence + 1.0f);

    duty[0] = Inverter_SVPWM_ClampFloat(
        duty[0], duty_minimum, duty_maximum);
    duty[1] = Inverter_SVPWM_ClampFloat(
        duty[1], duty_minimum, duty_maximum);
    duty[2] = Inverter_SVPWM_ClampFloat(
        duty[2], duty_minimum, duty_maximum);

    compare_a = Inverter_SVPWM_DutyToCompare(duty[0]);
    compare_b = Inverter_SVPWM_DutyToCompare(duty[1]);
    compare_c = Inverter_SVPWM_DutyToCompare(duty[2]);

    Inverter_SVPWM_WritePreload(
        compare_a,
        compare_b,
        compare_c);

    inverter_svpwm_state.modulation_scale = scale;
    inverter_svpwm_state.zero_sequence = zero_sequence;
    inverter_svpwm_state.applied_m_a =
        phase[0] + zero_sequence;
    inverter_svpwm_state.applied_m_b =
        phase[1] + zero_sequence;
    inverter_svpwm_state.applied_m_c =
        phase[2] + zero_sequence;
    inverter_svpwm_state.duty_a_next = duty[0];
    inverter_svpwm_state.duty_b_next = duty[1];
    inverter_svpwm_state.duty_c_next = duty[2];
    inverter_svpwm_state.compare_a_next = compare_a;
    inverter_svpwm_state.compare_b_next = compare_b;
    inverter_svpwm_state.compare_c_next = compare_c;
    inverter_svpwm_state.sector =
        Inverter_SVPWM_GetSector(
            phase[0],
            phase[1],
            phase[2]);
}

/**
 * @brief 通过ODISR立即关闭Timer A/B/C六路功率输出
 */
void Inverter_SVPWM_Disable(void)
{
    hhrtim1.Instance->sCommonRegs.ODISR =
        INVERTER_SVPWM_ALL_OUTPUTS;
    inverter_svpwm_state.outputs_enabled = 0U;
}

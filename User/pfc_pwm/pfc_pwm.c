#include "pfc_pwm.h"

volatile PFC_PWM_StateTypeDef pfc_pwm_state;

/**
 * @brief          将浮点数限制到指定范围
 * @param[in]      value 待限幅数值
 * @param[in]      minimum 最小允许值
 * @param[in]      maximum 最大允许值
 * @retval         float 限幅后的数值
 */
static float PFC_PWM_Clamp(float value,
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
 * @brief          将Timer E/F上管占空比换算为中心对齐CMP1值
 * @param[in]      duty 上管占空比，范围为0至PFC_PWM_MAX_ACTIVE_DUTY
 * @retval         uint32_t HRTIM CMP1比较值
 *
 * @note           当前HRTIM采用中心对齐计数，Output 1在向上计数CMP1事件置位，
 *                 在向下计数CMP1事件复位，因此：
 *
 *                 duty = (PER - CMP1) / PER
 *                 CMP1 = PER * (1 - duty)
 *
 *                 duty=0时CMP1=PER，对应上管关闭、互补下管导通。
 */
static uint32_t PFC_PWM_DutyToCompare(float duty)
{
    float compare_f;
    float minimum_duty;
    uint32_t compare;

    duty = PFC_PWM_Clamp(duty,
                         0.0f,
                         PFC_PWM_MAX_ACTIVE_DUTY);

    minimum_duty =
        (float)PFC_PWM_COMPARE_GUARD_COUNTS /
        (float)pfc_pwm_state.period_counts;

    /*
     * 过小的非零占空比会使比较事件过于靠近PER边界。
     * 将它归零；CMP1=PER表示上管零脉宽、互补下管保持导通。
     */
    if (duty < minimum_duty) {
        return pfc_pwm_state.period_counts;
    }

    compare_f =
        (1.0f - duty) *
        (float)pfc_pwm_state.period_counts;

    compare = (uint32_t)(compare_f + 0.5f);

    if (compare > pfc_pwm_state.period_counts) {
        compare = pfc_pwm_state.period_counts;
    }

    if (compare < PFC_PWM_COMPARE_GUARD_COUNTS) {
        compare = PFC_PWM_COMPARE_GUARD_COUNTS;
    }

    if (compare >
        (pfc_pwm_state.period_counts -
         PFC_PWM_COMPARE_GUARD_COUNTS)) {
        compare =
            pfc_pwm_state.period_counts -
            PFC_PWM_COMPARE_GUARD_COUNTS;
    }

    return compare;
}

/**
 * @brief          生成单极性倍频PWM的两个桥臂下一周期比较值
 * @param[in]      input_voltage_v 当前输入交流瞬时电压，单位为V
 * @param[in]      modulation 当前控制器有符号调制度
 * @param[out]     compare_e_next Timer E下一周期CMP1值
 * @param[out]     compare_f_next Timer F下一周期CMP1值
 * @param[out]     timer_e_duty_next 下一周期Timer E上管实际占空比
 * @param[out]     timer_f_duty_next 下一周期Timer F上管实际占空比
 * @param[out]     mode_next 下一周期开关模式
 * @retval         none
 */
static void PFC_PWM_BuildNextPeriod(
    float input_voltage_v,
    float modulation,
    uint32_t *compare_e_next,
    uint32_t *compare_f_next,
    float *timer_e_duty_next,
    float *timer_f_duty_next,
    PFC_PWM_ModeTypeDef *mode_next)
{
    float applied_modulation;
    float duty_e;
    float duty_f;

    applied_modulation = 0.0f;

    if (input_voltage_v > PFC_PWM_ZERO_CROSS_V) {
        /*
         * 交流输入正半周只允许正调制度，对应桥侧电压0或+Vbus。
         * 一个中心对齐周期从载波峰值开始的状态顺序为：
         *
         * S1S3 -> S1S4 -> S2S4 -> S1S4 -> S1S3。
         */
        applied_modulation = PFC_PWM_Clamp(
            modulation,
            0.0f,
            PFC_PWM_MAX_MODULATION);

        *mode_next = PFC_PWM_MODE_POSITIVE;
    } else if (input_voltage_v < -PFC_PWM_ZERO_CROSS_V) {
        /*
         * 交流输入负半周只允许负调制度，对应桥侧电压0或-Vbus。
         * 一个中心对齐周期从载波峰值开始的状态顺序为：
         *
         * S1S3 -> S2S3 -> S2S4 -> S2S3 -> S1S3。
         */
        applied_modulation = PFC_PWM_Clamp(
            modulation,
            -PFC_PWM_MAX_MODULATION,
            0.0f);

        *mode_next = PFC_PWM_MODE_NEGATIVE;
    } else {
        /*
         * 交流过零区：
         * 强制调制度为0，使S1和S3都为50%且同相；
         * 两个桥臂中点电压相等，桥侧差模电压为0。
         * 这不是故障停机；故障停机必须调用PFC_PWM_Disable()关闭四路输出。
         */
        *mode_next = PFC_PWM_MODE_ZERO;
    }

    /*
     * 双桥臂中心对齐单极性倍频调制：
     *
     * Timer E Output 1 = S1，Timer F Output 1 = S3；
     * S2和S4分别由HRTIM死区模块生成S1、S3的互补输出。
     *
     * 两桥臂占空比之差为有符号调制度：
     * duty_e - duty_f = applied_modulation。
     *
     * m=+0.20时：S1=60%，S3=40%，桥侧只出现0和+Vbus；
     * m=-0.20时：S1=40%，S3=60%，桥侧只出现0和-Vbus。
     */
    duty_e = 0.5f * (1.0f + applied_modulation);
    duty_f = 0.5f * (1.0f - applied_modulation);

    *compare_e_next = PFC_PWM_DutyToCompare(duty_e);
    *compare_f_next = PFC_PWM_DutyToCompare(duty_f);

    /*
     * 保存真正由比较值形成的占空比，而不是限幅前的请求值。
     * 这样调试观察值也包含最小脉宽和边界保护的影响。
     */
    *timer_e_duty_next =
        (float)(pfc_pwm_state.period_counts -
                *compare_e_next) /
        (float)pfc_pwm_state.period_counts;

    *timer_f_duty_next =
        (float)(pfc_pwm_state.period_counts -
                *compare_f_next) /
        (float)pfc_pwm_state.period_counts;
}

/**
 * @brief          生成Timer E/F固定占空比模式的下一周期比较值
 * @param[in]      timer_e_duty Timer E上管S1固定占空比
 * @param[in]      timer_f_duty Timer F上管S3固定占空比
 * @param[out]     compare_e_next Timer E下一周期CMP1值
 * @param[out]     compare_f_next Timer F下一周期CMP1值
 * @param[out]     timer_e_duty_next 下一周期Timer E上管实际占空比
 * @param[out]     timer_f_duty_next 下一周期Timer F上管实际占空比
 * @retval         none
 */
static void PFC_PWM_BuildFixedNextPeriod(
    float timer_e_duty,
    float timer_f_duty,
    uint32_t *compare_e_next,
    uint32_t *compare_f_next,
    float *timer_e_duty_next,
    float *timer_f_duty_next)
{
    *compare_e_next =
        PFC_PWM_DutyToCompare(timer_e_duty);
    *compare_f_next =
        PFC_PWM_DutyToCompare(timer_f_duty);

    /*
     * 使用最终CMP值反算实际占空比，使调试数据包含限幅、
     * 最小脉宽和比较边界保护的影响。
     */
    *timer_e_duty_next =
        (float)(pfc_pwm_state.period_counts -
                *compare_e_next) /
        (float)pfc_pwm_state.period_counts;

    *timer_f_duty_next =
        (float)(pfc_pwm_state.period_counts -
                *compare_f_next) /
        (float)pfc_pwm_state.period_counts;
}

/**
 * @brief          写入Timer E/F的CMP1预装载寄存器
 * @param[in]      compare_e_next Timer E下一周期CMP1值
 * @param[in]      compare_f_next Timer F下一周期CMP1值
 * @retval         none
 *
 * @note           不调用HAL_HRTIM_SoftwareUpdate()。Timer E/F会在下一个
 *                 Master更新事件把两个预装载值同步搬入活动寄存器。
 */
static void PFC_PWM_WritePreload(uint32_t compare_e_next,
                                 uint32_t compare_f_next)
{
    __HAL_HRTIM_SETCOMPARE(
        &hhrtim1,
        HRTIM_TIMERINDEX_TIMER_E,
        HRTIM_COMPAREUNIT_1,
        compare_e_next);

    __HAL_HRTIM_SETCOMPARE(
        &hhrtim1,
        HRTIM_TIMERINDEX_TIMER_F,
        HRTIM_COMPAREUNIT_1,
        compare_f_next);
}

/**
 * @brief          初始化PFC PWM运行状态和安全比较值
 * @param[in]      none
 * @retval         HAL_OK 初始化成功
 * @retval         HAL_ERROR Timer E/F周期无效或周期不一致
 */
HAL_StatusTypeDef PFC_PWM_Init(void)
{
    HAL_StatusTypeDef status;
    uint32_t period_e;
    uint32_t period_f;

    /* 初始化阶段先确保四路功率输出保持关闭。 */
    hhrtim1.Instance->sCommonRegs.ODISR =
        PFC_PWM_ALL_OUTPUTS;

    /* 读取Timer E的中心对齐计数峰值。 */
    period_e =
        hhrtim1.Instance
            ->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E]
            .PERxR;

    /* 读取Timer F的中心对齐计数峰值。 */
    period_f =
        hhrtim1.Instance
            ->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_F]
            .PERxR;

    /*
     * Timer E/F周期必须一致，并且必须为比较值边界保护
     * 留出足够计数空间。
     */
    if ((period_e <=
         (2U * PFC_PWM_COMPARE_GUARD_COUNTS)) ||
        (period_e != period_f)) {
        return HAL_ERROR;
    }

    /*
     * Timer E/F的上下计数、CMP1置位/复位、互补死区、
     * 预装载和Master同步更新均由threenibian.ioc配置。
     * 此处不再重复修改HRTIM静态配置寄存器。
     */

    /* 初始化PWM调试和运行状态。 */
    pfc_pwm_state.requested_modulation = 0.0f;
    pfc_pwm_state.timer_e_duty_next = 0.0f;
    pfc_pwm_state.timer_f_duty_next = 0.0f;
    pfc_pwm_state.period_counts = period_e;
    pfc_pwm_state.compare_e_next = period_e;
    pfc_pwm_state.compare_f_next = period_f;
    pfc_pwm_state.mode_next = PFC_PWM_MODE_DISABLED;
    pfc_pwm_state.initialized = 1U;
    pfc_pwm_state.outputs_enabled = 0U;

    /*
     * 初始化时四路功率输出仍处于关闭状态。
     * 先把Timer E Output 1内部状态强制为无效电平。
     */
    status = HAL_HRTIM_WaveformSetOutputLevel(
        &hhrtim1,
        HRTIM_TIMERINDEX_TIMER_E,
        HRTIM_OUTPUT_TE1,
        HRTIM_OUTPUTLEVEL_INACTIVE);

    if (status != HAL_OK) {
        return status;
    }

    /*
     * 把Timer F Output 1内部状态强制为无效电平。
     */
    status = HAL_HRTIM_WaveformSetOutputLevel(
        &hhrtim1,
        HRTIM_TIMERINDEX_TIMER_F,
        HRTIM_OUTPUT_TF1,
        HRTIM_OUTPUTLEVEL_INACTIVE);

    if (status != HAL_OK) {
        return status;
    }

    /*
     * 写入安全的初始比较值。
     * 此时输出仍然关闭，所以允许执行一次软件更新，
     * 将安全初值装入活动比较寄存器。
     */
    PFC_PWM_WritePreload(
        period_e,
        period_f);

    return HAL_HRTIM_SoftwareUpdate(
        &hhrtim1,
        HRTIM_TIMERUPDATE_E |
        HRTIM_TIMERUPDATE_F);
}
/**
 * @brief          同步启动HRTIM Master、Timer E和Timer F计数器
 * @param[in]      none
 * @retval         HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef PFC_PWM_StartCounters(void)
{
    if (pfc_pwm_state.initialized == 0U) {
        return HAL_ERROR;
    }

    return HAL_HRTIM_WaveformCounterStart(
        &hhrtim1,
        PFC_PWM_COUNTERS);
}

/**
 * @brief          用第一组调制度装载PWM并使能四路功率输出
 * @param[in]      input_voltage_v 当前输入交流瞬时电压，单位为V
 * @param[in]      modulation 当前控制器有符号调制度
 * @retval         HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef PFC_PWM_Enable(float input_voltage_v,
                                 float modulation)
{
    HAL_StatusTypeDef status;

    if ((pfc_pwm_state.initialized == 0U) ||
        (pfc_pwm_state.outputs_enabled != 0U)) {
        return HAL_ERROR;
    }

    /*
     * 这里只写第一组控制量的预装载值，不触发软件更新。
     * 输出使能后当前周期仍使用初始化阶段建立的安全零占空比；
     * 第一组控制量由下一个Master更新事件自动装入并开始生效。
     */
    PFC_PWM_UpdateNextPeriod(
        input_voltage_v,
        modulation);

    status = HAL_HRTIM_WaveformOutputStart(
        &hhrtim1,
        PFC_PWM_ALL_OUTPUTS);

    if (status == HAL_OK) {
        pfc_pwm_state.outputs_enabled = 1U;
    }

    return status;
}

/**
 * @brief          用第一组Timer E/F固定占空比使能四路功率输出
 * @param[in]      timer_e_duty Timer E上管S1固定占空比
 * @param[in]      timer_f_duty Timer F上管S3固定占空比
 * @retval         HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef PFC_PWM_EnableFixed(float timer_e_duty,
                                      float timer_f_duty)
{
    HAL_StatusTypeDef status;

    if ((pfc_pwm_state.initialized == 0U) ||
        (pfc_pwm_state.outputs_enabled != 0U)) {
        return HAL_ERROR;
    }

    /*
     * 只写第一组固定占空比预装载值，不触发软件更新。
     * 输出使能后，固定值在下一个Master更新边界同步生效。
     */
    PFC_PWM_UpdateFixedNextPeriod(
        timer_e_duty,
        timer_f_duty);

    status = HAL_HRTIM_WaveformOutputStart(
        &hhrtim1,
        PFC_PWM_ALL_OUTPUTS);

    if (status == HAL_OK) {
        pfc_pwm_state.outputs_enabled = 1U;
    }

    return status;
}

/**
 * @brief          计算并预装载下一个PWM周期的Timer E/F占空比
 * @param[in]      input_voltage_v 当前输入交流瞬时电压，单位为V
 * @param[in]      modulation 当前控制器有符号调制度
 * @retval         none
 */
void PFC_PWM_UpdateNextPeriod(float input_voltage_v,
                              float modulation)
{
    uint32_t compare_e_next;
    uint32_t compare_f_next;
    float timer_e_duty_next;
    float timer_f_duty_next;
    PFC_PWM_ModeTypeDef mode_next;

    if (pfc_pwm_state.initialized == 0U) {
        return;
    }

    PFC_PWM_BuildNextPeriod(
        input_voltage_v,
        modulation,
        &compare_e_next,
        &compare_f_next,
        &timer_e_duty_next,
        &timer_f_duty_next,
        &mode_next);

    /*
     * ADC采样由Timer F周期边界触发，本函数应在紧随其后的控制回调中执行。
     * 两次写入会在下一个Master更新事件一起装载，不改变当前PWM周期。
     */
    PFC_PWM_WritePreload(
        compare_e_next,
        compare_f_next);

    pfc_pwm_state.requested_modulation =
        modulation;
    pfc_pwm_state.timer_e_duty_next =
        timer_e_duty_next;
    pfc_pwm_state.timer_f_duty_next =
        timer_f_duty_next;
    pfc_pwm_state.compare_e_next =
        compare_e_next;
    pfc_pwm_state.compare_f_next =
        compare_f_next;
    pfc_pwm_state.mode_next =
        mode_next;
}

/**
 * @brief          预装载下一个PWM周期的Timer E/F固定占空比
 * @param[in]      timer_e_duty Timer E上管S1固定占空比
 * @param[in]      timer_f_duty Timer F上管S3固定占空比
 * @retval         none
 */
void PFC_PWM_UpdateFixedNextPeriod(float timer_e_duty,
                                   float timer_f_duty)
{
    uint32_t compare_e_next;
    uint32_t compare_f_next;
    float timer_e_duty_next;
    float timer_f_duty_next;

    if (pfc_pwm_state.initialized == 0U) {
        return;
    }

    PFC_PWM_BuildFixedNextPeriod(
        timer_e_duty,
        timer_f_duty,
        &compare_e_next,
        &compare_f_next,
        &timer_e_duty_next,
        &timer_f_duty_next);

    /*
     * 和闭环模式完全相同，只写Timer E/F预装载寄存器。
     * 当前PWM周期不变，两个固定值在下一个Master更新边界一起生效。
     */
    PFC_PWM_WritePreload(
        compare_e_next,
        compare_f_next);

    pfc_pwm_state.requested_modulation = 0.0f;
    pfc_pwm_state.timer_e_duty_next =
        timer_e_duty_next;
    pfc_pwm_state.timer_f_duty_next =
        timer_f_duty_next;
    pfc_pwm_state.compare_e_next =
        compare_e_next;
    pfc_pwm_state.compare_f_next =
        compare_f_next;
    pfc_pwm_state.mode_next =
        PFC_PWM_MODE_FIXED_EF;
}

/**
 * @brief          立即关闭PFC四路功率输出
 * @param[in]      none
 * @retval         none
 */
void PFC_PWM_Disable(void)
{
    /*
     * ODISR为异步安全关断路径，不等待下一PWM周期。
     * 正常占空比更新使用预装载；停机和故障必须优先保证关断速度。
     */
    hhrtim1.Instance->sCommonRegs.ODISR =
        PFC_PWM_ALL_OUTPUTS;

    pfc_pwm_state.outputs_enabled = 0U;
    pfc_pwm_state.timer_e_duty_next = 0.0f;
    pfc_pwm_state.timer_f_duty_next = 0.0f;
    pfc_pwm_state.mode_next = PFC_PWM_MODE_DISABLED;
}

#include "pfc_control.h"

/** PFC模式命令与运行数据，可直接在调试器中观察。 */
volatile PFC_ControlStateTypeDef pfc_control_state;

/** PFC电压PI外环和电流PR内环的内部历史状态。 */
static PFC_ControlRuntimeTypeDef pfc_control_runtime;

/**
 * @brief          将浮点数限制到指定范围
 * @param[in]      value 待限幅数值
 * @param[in]      minimum 最小允许值
 * @param[in]      maximum 最大允许值
 * @retval         float 限幅后的数值
 */
static float PFC_Control_Clamp(float value,
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
 * @brief 根据当前输入RMS更新输入电压功率前馈增益
 */
static void PFC_Control_UpdateLineFeedforward(void)
{
    float line_feedforward_gain;

    line_feedforward_gain =
        PFC_INPUT_RMS_NOMINAL_V /
        pfc_control_state.input_voltage_rms_v;

    pfc_control_state.line_feedforward_gain =
        PFC_Control_Clamp(
            line_feedforward_gain,
            PFC_LINE_FEEDFORWARD_MIN,
            PFC_LINE_FEEDFORWARD_MAX);
}

/**
 * @brief          清除电流PR内环的运行历史量
 * @param[in]      none
 * @retval         none
 *
 * @note           本函数保留初始化时计算得到的PR离散系数。
 */
static void PFC_Control_ClearCurrentPRHistory(void)
{
    pfc_control_runtime.current_pr.error_1 = 0.0f;
    pfc_control_runtime.current_pr.error_2 = 0.0f;
    pfc_control_runtime.current_pr.resonant_1 = 0.0f;
    pfc_control_runtime.current_pr.resonant_2 = 0.0f;
}

/**
 * @brief          根据控制频率和电网频率计算电流PR离散系数
 * @param[in]      none
 * @retval         none
 */
static void PFC_Control_ConfigureCurrentPR(void)
{
    PFC_ControlCurrentPRTypeDef *pr;
    float k;
    float omega;
    float denominator;

    pr = &pfc_control_runtime.current_pr;
    k = 2.0f * PFC_CONTROL_FREQ_HZ;
    omega = PFC_TWO_PI * PFC_GRID_FREQ_HZ;
    denominator =
        k * k +
        2.0f * PFC_CURRENT_PR_WC_RAD_S * k +
        omega * omega;

    /*
     * 有限带宽PR谐振支路：
     * Gres(s)=2*Kr*wc*s/(s²+2*wc*s+w0²)。
     * 使用双线性变换离散化，谐振中心为PFC_GRID_FREQ_HZ。
     */
    pr->b0 =
        2.0f * PFC_CURRENT_PR_KR *
        PFC_CURRENT_PR_WC_RAD_S * k /
        denominator;

    pr->a1 =
        (-2.0f * k * k + 2.0f * omega * omega) /
        denominator;

    pr->a2 =
        (k * k -
         2.0f * PFC_CURRENT_PR_WC_RAD_S * k +
         omega * omega) /
        denominator;
}

/**
 * @brief          清除闭环历史状态并重建软启动起点
 * @param[in]      initial_bus_voltage_v 当前实测母线电压，单位为V
 * @retval         none
 *
 * @note           当前模式和三个调试给定参数不会被修改。
 */
static void PFC_Control_ResetLoopState(
    float initial_bus_voltage_v)
{
    pfc_control_runtime =
        (PFC_ControlRuntimeTypeDef){0};

    pfc_control_state.bus_reference_v =
        PFC_Control_Clamp(initial_bus_voltage_v,
                          0.0f,
                          PFC_BUS_TARGET_V);
    pfc_control_state.bus_feedback_v =
        initial_bus_voltage_v;
    pfc_control_state.bus_error_v = 0.0f;
    pfc_control_state.current_command_nominal_rms_a = 0.0f;
    pfc_control_state.input_voltage_unit = 0.0f;
    pfc_control_state.current_reference_rms_a = 0.0f;
    pfc_control_state.current_reference_a = 0.0f;
    pfc_control_state.current_error_a = 0.0f;
    pfc_control_state.current_pr_output_v = 0.0f;
    pfc_control_state.bridge_voltage_command_v = 0.0f;
    pfc_control_state.modulation = 0.0f;
    pfc_control_state.modulation_saturated = 0U;



    PFC_Control_UpdateLineFeedforward();
    PFC_Control_ConfigureCurrentPR();
}

/**
 * @brief          执行母线电压PI外环
 * @param[in]      bus_voltage_v 母线反馈电压，单位为V
 * @retval         none
 */
static void PFC_Control_RunVoltageLoop(
    float bus_voltage_v)
{
    PFC_ControlVoltagePITypeDef *pi;
    float nominal_current_limit_rms_a;
    float nominal_current_command_rms_a;

    pi = &pfc_control_runtime.voltage_pi;

    pfc_control_state.bus_feedback_v =
        bus_voltage_v;

    /* 母线目标软启动。 */
    if (pfc_control_state.bus_reference_v <
        PFC_BUS_TARGET_V) {
        pfc_control_state.bus_reference_v +=
            PFC_BUS_REF_SLEW_V_PER_S /
            PFC_CONTROL_FREQ_HZ;

        if (pfc_control_state.bus_reference_v >
            PFC_BUS_TARGET_V) {
            pfc_control_state.bus_reference_v =
                PFC_BUS_TARGET_V;
        }
    }

    pfc_control_state.bus_error_v =
        pfc_control_state.bus_reference_v -
        pfc_control_state.bus_feedback_v;

    if (pi->divider_count == 0U) {
        /*
         * 最终电流命令不能超过3A，因此先根据线电压
         * 前馈增益反算PI额定电流命令的最大值。
         */
        nominal_current_limit_rms_a =
            PFC_CURRENT_REF_MAX_RMS_A /
            pfc_control_state.line_feedforward_gain;

        nominal_current_limit_rms_a =
            PFC_Control_Clamp(
                nominal_current_limit_rms_a,
                0.0f,
                PFC_CURRENT_REF_MAX_RMS_A);

        /* 电压PI积分。 */
        pi->integral =
            PFC_Control_Clamp(
                pi->integral +
                PFC_BUS_PI_KI *
                pfc_control_state.bus_error_v *
                ((float)PFC_VOLTAGE_LOOP_DIVIDER /
                 PFC_CONTROL_FREQ_HZ),
                0.0f,
                nominal_current_limit_rms_a);

        /* 额定35V输入下的电流命令。 */
        nominal_current_command_rms_a =
            PFC_Control_Clamp(
                PFC_BUS_PI_KP *
                pfc_control_state.bus_error_v +
                pi->integral,
                0.0f,
                nominal_current_limit_rms_a);

        pfc_control_state
            .current_command_nominal_rms_a =
            nominal_current_command_rms_a;

        /*
         * 输入31V时增大电流，输入41V时减小电流，
         * 使输入变化时功率大致保持不变。
         */
        pfc_control_state.current_reference_rms_a =
            PFC_Control_Clamp(
                nominal_current_command_rms_a *
                pfc_control_state.line_feedforward_gain,
                0.0f,
                PFC_CURRENT_REF_MAX_RMS_A);
    }

    pi->divider_count++;

    if (pi->divider_count >=
        PFC_VOLTAGE_LOOP_DIVIDER) {
        pi->divider_count = 0U;
    }
}

/**
 * @brief          执行一次输入电流PR内环和输入电压前馈
 * @param[in]      input_voltage_v 输入瞬时电压，单位为V
 * @param[in]      input_current_a 输入瞬时电流，单位为A
 * @param[in]      bus_voltage_v 母线电压，单位为V
 * @retval         float 有符号归一化调制度
 */
static float PFC_Control_RunCurrentLoop(
    float input_voltage_v,
    float input_current_a,
    float bus_voltage_v)
{
    PFC_ControlCurrentPRTypeDef *pr;
    float resonant_output_v;
    float raw_modulation;

    pr = &pfc_control_runtime.current_pr;

    /*
     * 使用实时输入RMS归一化输入电压。
     * 理想正弦波的Vin/Vin_rms自身有效值等于1。
     */
    pfc_control_state.input_voltage_unit =
        PFC_Control_Clamp(
            input_voltage_v /
            pfc_control_state.input_voltage_rms_v,
            -PFC_INPUT_UNIT_LIMIT,
            PFC_INPUT_UNIT_LIMIT);

    /* 生成与输入电压同相的瞬时电流参考。 */
    pfc_control_state.current_reference_a =
        pfc_control_state.current_reference_rms_a *
        pfc_control_state.input_voltage_unit;

    pfc_control_state.current_error_a =
        pfc_control_state.current_reference_a -
        input_current_a;

    /*
     * 母线电压过低时不运行PR历史递推，也不执行除法。
     * 这样母线恢复后不会带着低压期间积累的谐振状态突然输出。
     */
    if (bus_voltage_v < PFC_MIN_BUS_V) {
        PFC_Control_ClearCurrentPRHistory();
        pfc_control_state.current_pr_output_v = 0.0f;
        pfc_control_state.bridge_voltage_command_v = 0.0f;
        pfc_control_state.modulation = 0.0f;
        pfc_control_state.modulation_saturated = 0U;
        return 0.0f;
    }

    /* 计算有限带宽PR控制器的谐振支路输出。 */
    resonant_output_v =
        pr->b0 *
        pfc_control_state.current_error_a -
        pr->b0 *
        pr->error_2 -
        pr->a1 *
        pr->resonant_1 -
        pr->a2 *
        pr->resonant_2;

    resonant_output_v =
        PFC_Control_Clamp(
            resonant_output_v,
            -PFC_CURRENT_PR_OUTPUT_LIMIT_V,
            PFC_CURRENT_PR_OUTPUT_LIMIT_V);

    /* 比例支路与谐振支路相加，得到电感压差修正量。 */
    pfc_control_state.current_pr_output_v =
        PFC_Control_Clamp(
            PFC_CURRENT_PR_KP *
            pfc_control_state.current_error_a +
            resonant_output_v,
            -PFC_CURRENT_PR_OUTPUT_LIMIT_V,
            PFC_CURRENT_PR_OUTPUT_LIMIT_V);

    /* 保存本周期PR误差和谐振支路状态，供下一控制周期使用。 */
    pr->error_2 = pr->error_1;
    pr->error_1 =
        pfc_control_state.current_error_a;
    pr->resonant_2 = pr->resonant_1;
    pr->resonant_1 = resonant_output_v;

    /*
     * 输入电压前馈：
     * L*di/dt=Vin-Vbridge，因此桥侧电压指令为
     * Vin减去电流PR给出的电感压差修正量。
     */
    pfc_control_state.bridge_voltage_command_v =
        input_voltage_v -
        pfc_control_state.current_pr_output_v;

    raw_modulation =
        pfc_control_state.bridge_voltage_command_v /
        bus_voltage_v;

    pfc_control_state.modulation =
        PFC_Control_Clamp(
            raw_modulation,
            -PFC_MODULATION_LIMIT,
            PFC_MODULATION_LIMIT);

    pfc_control_state.modulation_saturated =
        (pfc_control_state.modulation !=
         raw_modulation) ? 1U : 0U;

    return pfc_control_state.modulation;
}

/**
 * @brief          初始化三模式PFC控制器
 * @param[in]      initial_bus_voltage_v 启动时实测母线电压，单位为V
 * @retval         none
 */
void PFC_Control_Init(float initial_bus_voltage_v)
{
    pfc_control_state =
        (PFC_ControlStateTypeDef){0};

    pfc_control_state.mode =
       PFC_CONTROL_RUN_MODE;
    pfc_control_state.fixed_timer_e_duty =
        PFC_Control_Clamp(
            PFC_EF_FIXED_TIMER_E_DUTY,
            0.0f,
            PFC_PWM_MAX_ACTIVE_DUTY);
    pfc_control_state.fixed_timer_f_duty =
        PFC_Control_Clamp(
            PFC_EF_FIXED_TIMER_F_DUTY,
            0.0f,
            PFC_PWM_MAX_ACTIVE_DUTY);
    pfc_control_state.current_loop_reference_rms_a =
        PFC_Control_Clamp(
            PFC_CURRENT_LOOP_REFERENCE_RMS_A,
            0.0f,
            PFC_CURRENT_REF_MAX_RMS_A);
    /*
 * 第一个交流周期RMS尚未计算完成时，
 * 暂时使用35V默认输入有效值。
 */
    pfc_control_state.input_voltage_rms_v =
        PFC_INPUT_RMS_NOMINAL_V;

    pfc_control_state.input_voltage_rms_valid = 0U;
    pfc_control_state.input_voltage_unit = 0.0f;

    PFC_Control_UpdateLineFeedforward();

    PFC_Control_ResetLoopState(
        initial_bus_voltage_v);
}

/**
 * @brief          清除PI/PR历史状态并从当前母线电压重新开始
 * @param[in]      initial_bus_voltage_v 复位时实测母线电压，单位为V
 * @retval         none
 */
void PFC_Control_Reset(float initial_bus_voltage_v)
{
    PFC_Control_ResetLoopState(
        initial_bus_voltage_v);
}

/**
 * @brief          设置EF固定模式的Timer E/F上管占空比
 * @param[in]      timer_e_duty Timer E上管S1占空比
 * @param[in]      timer_f_duty Timer F上管S3占空比
 * @retval         none
 */
void PFC_Control_SetFixedEFDuty(float timer_e_duty,
                                float timer_f_duty)
{
    pfc_control_state.fixed_timer_e_duty =
        PFC_Control_Clamp(
            timer_e_duty,
            0.0f,
            PFC_PWM_MAX_ACTIVE_DUTY);

    pfc_control_state.fixed_timer_f_duty =
        PFC_Control_Clamp(
            timer_f_duty,
            0.0f,
            PFC_PWM_MAX_ACTIVE_DUTY);
}

/**
 * @brief          设置单电流环模式的输入电流有效值给定
 * @param[in]      current_reference_rms_a 电流有效值给定，单位为A RMS
 * @retval         none
 */
void PFC_Control_SetCurrentReferenceRms(
    float current_reference_rms_a)
{
    pfc_control_state.current_loop_reference_rms_a =
        PFC_Control_Clamp(
            current_reference_rms_a,
            0.0f,
            PFC_CURRENT_REF_MAX_RMS_A);
}

/**
 * @brief          更新控制器使用的输入交流电压有效值
 * @param[in]      input_voltage_rms_v 输入电压有效值，单位为V RMS
 * @retval         none
 */
void PFC_Control_SetInputVoltageRms(
    float input_voltage_rms_v)
{
    /* 异常结果不允许覆盖上一次有效RMS。 */
    if ((input_voltage_rms_v <
         PFC_INPUT_RMS_MIN_VALID_V) ||
        (input_voltage_rms_v >
         PFC_INPUT_RMS_MAX_VALID_V)) {
        return;
         }

    pfc_control_state.input_voltage_rms_v =
        input_voltage_rms_v;

    pfc_control_state.input_voltage_rms_valid =
        1U;

    PFC_Control_UpdateLineFeedforward();
}

/**
 * @brief          根据当前模式执行一次控制算法
 * @param[in]      input_voltage_v 输入交流瞬时电压，单位为V
 * @param[in]      input_current_a 输入交流瞬时电流，单位为A
 * @param[in]      bus_voltage_v 经过数字滤波的母线电压，单位为V
 * @retval         float 电流环/PFC模式的有符号调制度
 */
float PFC_Control_Run(float input_voltage_v,
                      float input_current_a,
                      float bus_voltage_v)
{
    pfc_control_state.bus_feedback_v =
        bus_voltage_v;

    if (pfc_control_state.mode ==
        PFC_CONTROL_MODE_EF_FIXED) {
        /*
         * 固定模式完全旁路PI和PR。
         * 实际E/F占空比由PFC_Control_UpdatePWMNextPeriod()直接交给PWM层。
         */
        pfc_control_state.bus_reference_v =
            bus_voltage_v;
        pfc_control_state.bus_error_v = 0.0f;
        pfc_control_state.current_reference_rms_a = 0.0f;
        pfc_control_state.current_reference_a = 0.0f;
        pfc_control_state.current_error_a = 0.0f;
        pfc_control_state.current_pr_output_v = 0.0f;
        pfc_control_state.bridge_voltage_command_v = 0.0f;
        pfc_control_state.modulation = 0.0f;
        pfc_control_state.modulation_saturated = 0U;
        return 0.0f;
    }

    if (pfc_control_state.mode ==
        PFC_CONTROL_MODE_CURRENT_LOOP) {
        /*
         * 单电流环模式不更新母线PI。
         * 手动有效值给定仍会乘以输入电压波形，形成同相正弦电流给定。
         */
        pfc_control_state.bus_reference_v =
            bus_voltage_v;
        pfc_control_state.bus_error_v = 0.0f;
        pfc_control_state.current_reference_rms_a =
            pfc_control_state
                .current_loop_reference_rms_a;
    } else {
        PFC_Control_RunVoltageLoop(
            bus_voltage_v);
    }

    return PFC_Control_RunCurrentLoop(
        input_voltage_v,
        input_current_a,
        bus_voltage_v);
}

/**
 * @brief          计算第一组模式命令并使能PFC四路PWM输出
 * @param[in]      input_voltage_v 输入交流瞬时电压，单位为V
 * @param[in]      input_current_a 输入交流瞬时电流，单位为A
 * @param[in]      bus_voltage_v 经过数字滤波的母线电压，单位为V
 * @retval         HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef PFC_Control_EnablePWM(
    float input_voltage_v,
    float input_current_a,
    float bus_voltage_v)
{
    float modulation;

    modulation = PFC_Control_Run(
        input_voltage_v,
        input_current_a,
        bus_voltage_v);

    if (pfc_control_state.mode ==
        PFC_CONTROL_MODE_EF_FIXED) {
        return PFC_PWM_EnableFixed(
            pfc_control_state.fixed_timer_e_duty,
            pfc_control_state.fixed_timer_f_duty);
    }

    return PFC_PWM_Enable(
        input_voltage_v,
        modulation);
}

/**
 * @brief          计算并预装载下一个PWM周期的模式命令
 * @param[in]      input_voltage_v 输入交流瞬时电压，单位为V
 * @param[in]      input_current_a 输入交流瞬时电流，单位为A
 * @param[in]      bus_voltage_v 经过数字滤波的母线电压，单位为V
 * @retval         none
 */
void PFC_Control_UpdatePWMNextPeriod(
    float input_voltage_v,
    float input_current_a,
    float bus_voltage_v)
{
    float modulation;

    modulation = PFC_Control_Run(
        input_voltage_v,
        input_current_a,
        bus_voltage_v);

    if (pfc_control_state.mode ==
        PFC_CONTROL_MODE_EF_FIXED) {
        PFC_PWM_UpdateFixedNextPeriod(
            pfc_control_state.fixed_timer_e_duty,
            pfc_control_state.fixed_timer_f_duty);
        return;
    }

    PFC_PWM_UpdateNextPeriod(
        input_voltage_v,
        modulation);
}

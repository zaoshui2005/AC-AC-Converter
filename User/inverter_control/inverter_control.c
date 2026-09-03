#include "inverter_control.h"

#include <math.h>

#include "inverter_adc_watchdog.h"
#include "inverter_svpwm.h"
/** 三相逆变控制运行状态，可在调试器中观察。 */
volatile Inverter_Control_StateTypeDef
    inverter_control_state;

/** 写1请求启动三相逆变。 */
volatile uint8_t inverter_start_request;

/** 写1请求停止三相逆变。 */
volatile uint8_t inverter_stop_request;

/** 三相逆变控制最近一次HAL执行状态。 */
volatile HAL_StatusTypeDef inverter_control_last_status;

/** 写1请求在低频和高频之间切换。 */
volatile uint8_t inverter_frequency_toggle_request;

/** 主循环1kHz后台线电压RMS累加器。 */
typedef struct
{
    float sum_ab_v;
    float sum_bc_v;
    float sum_ca_v;
    float square_sum_ab_v2;
    float square_sum_bc_v2;
    float square_sum_ca_v2;
    uint32_t sample_count;
    uint32_t last_sample_tick_ms;
    uint32_t window_start_tick_ms;
    uint8_t tick_initialized;
} Inverter_LineVoltageRmsRuntimeTypeDef;

static Inverter_LineVoltageRmsRuntimeTypeDef
    inverter_line_voltage_rms_runtime;

/**
 * @brief 将浮点数限制到指定范围
 */
static float Inverter_Control_Clamp(float value,
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
 * @brief 根据30Hz/60Hz输出频率选择内部线电压RMS指令
 */
static float Inverter_Control_SelectLineVoltageTargetRms(
    float output_frequency_hz)
{
    if (output_frequency_hz <
        ((INVERTER_OUTPUT_FREQ_LOW_HZ +
          INVERTER_OUTPUT_FREQ_HIGH_HZ) * 0.5f)) {
        return INVERTER_LINE_VOLTAGE_TARGET_RMS_30HZ;
    }

    return INVERTER_LINE_VOLTAGE_TARGET_RMS_60HZ;
}

/**
 * @brief 清除线电压整周期RMS累加器和慢速PI状态
 */
static void Inverter_Control_ResetLineVoltageRmsPI(void)
{
    inverter_line_voltage_rms_runtime =
        (Inverter_LineVoltageRmsRuntimeTypeDef){0};

    inverter_control_state.line_voltage_ab_rms_v = 0.0f;
    inverter_control_state.line_voltage_bc_rms_v = 0.0f;
    inverter_control_state.line_voltage_ca_rms_v = 0.0f;
    inverter_control_state.line_voltage_feedback_rms_v = 0.0f;
    inverter_control_state.line_voltage_imbalance_rms_v = 0.0f;
    inverter_control_state.line_voltage_rms_error_v = 0.0f;
    inverter_control_state.line_voltage_rms_pi_integral_v = 0.0f;
    inverter_control_state.line_voltage_rms_pi_target_v = 0.0f;
    inverter_control_state.line_voltage_rms_pi_correction_v = 0.0f;
    inverter_control_state.line_voltage_rms_window_s = 0.0f;
    inverter_control_state.line_voltage_rms_window_samples = 0U;
    inverter_control_state.line_voltage_rms_ready = 0U;
    inverter_control_state.line_voltage_rms_pi_active = 0U;
    inverter_control_state.line_voltage_rms_pi_saturated = 0U;
}

/**
 * @brief 在主循环按1ms采样、100ms计算线电压RMS并运行慢速PI
 *
 * @note 快速中断只负责原有控制和更新Uab/Ubc状态。本函数读取32位float
 *       快照，不关中断、不使用double，也不向20kHz路径增加RMS乘加。
 *       100ms窗口包含30Hz三周期或60Hz六周期。
 */
static void Inverter_Control_ProcessLineVoltageRmsPI(void)
{
    Inverter_LineVoltageRmsRuntimeTypeDef *rms;
    float u_ab_v;
    float u_bc_v;
    float u_ca_v;
    float sample_count_f;
    float mean_ab_v;
    float mean_bc_v;
    float mean_ca_v;
    float mean_square_ab_v2;
    float mean_square_bc_v2;
    float mean_square_ca_v2;
    float rms_ab_v;
    float rms_bc_v;
    float rms_ca_v;
    float rms_max_v;
    float rms_min_v;
    float error_v;
    float integral_candidate_v;
    float proportional_v;
    float output_unclamped_v;
    float output_v;
    float correction_step_limit_v;
    float correction_delta_v;
    float window_s;
    uint32_t current_tick_ms;
    uint32_t window_elapsed_ms;
    uint32_t sample_count;

    if (inverter_control_state.enabled == 0U) {
        return;
    }

    rms = &inverter_line_voltage_rms_runtime;
    current_tick_ms = HAL_GetTick();

    if (rms->tick_initialized == 0U) {
        rms->last_sample_tick_ms = current_tick_ms;
        rms->window_start_tick_ms = current_tick_ms;
        rms->tick_initialized = 1U;
        return;
    }

    if ((current_tick_ms - rms->last_sample_tick_ms) <
        INVERTER_LINE_RMS_SAMPLE_PERIOD_MS) {
        return;
    }

    rms->last_sample_tick_ms = current_tick_ms;

    /* Cortex-M4上对齐的32位float读写是原子的，无需为快照关闭中断。 */
    u_ab_v = inverter_control_state.u_ab_v;
    u_bc_v = inverter_control_state.u_bc_v;
    u_ca_v = -(u_ab_v + u_bc_v);

    rms->sum_ab_v += u_ab_v;
    rms->sum_bc_v += u_bc_v;
    rms->sum_ca_v += u_ca_v;
    rms->square_sum_ab_v2 += u_ab_v * u_ab_v;
    rms->square_sum_bc_v2 += u_bc_v * u_bc_v;
    rms->square_sum_ca_v2 += u_ca_v * u_ca_v;
    rms->sample_count++;

    if (rms->sample_count < INVERTER_LINE_RMS_WINDOW_SAMPLES) {
        return;
    }

    sample_count = rms->sample_count;
    sample_count_f = (float)sample_count;
    mean_ab_v = rms->sum_ab_v / sample_count_f;
    mean_bc_v = rms->sum_bc_v / sample_count_f;
    mean_ca_v = rms->sum_ca_v / sample_count_f;

    mean_square_ab_v2 =
        rms->square_sum_ab_v2 / sample_count_f -
        mean_ab_v * mean_ab_v;
    mean_square_bc_v2 =
        rms->square_sum_bc_v2 / sample_count_f -
        mean_bc_v * mean_bc_v;
    mean_square_ca_v2 =
        rms->square_sum_ca_v2 / sample_count_f -
        mean_ca_v * mean_ca_v;

    mean_square_ab_v2 =
        Inverter_Control_Clamp(mean_square_ab_v2, 0.0f, 1.0e9f);
    mean_square_bc_v2 =
        Inverter_Control_Clamp(mean_square_bc_v2, 0.0f, 1.0e9f);
    mean_square_ca_v2 =
        Inverter_Control_Clamp(mean_square_ca_v2, 0.0f, 1.0e9f);

    rms_ab_v = sqrtf(mean_square_ab_v2);
    rms_bc_v = sqrtf(mean_square_bc_v2);
    rms_ca_v = sqrtf(mean_square_ca_v2);

    inverter_control_state.line_voltage_ab_rms_v = rms_ab_v;
    inverter_control_state.line_voltage_bc_rms_v = rms_bc_v;
    inverter_control_state.line_voltage_ca_rms_v = rms_ca_v;
    inverter_control_state.line_voltage_feedback_rms_v =
        sqrtf((mean_square_ab_v2 +
               mean_square_bc_v2 +
               mean_square_ca_v2) / 3.0f);

    rms_max_v = fmaxf(rms_ab_v, fmaxf(rms_bc_v, rms_ca_v));
    rms_min_v = fminf(rms_ab_v, fminf(rms_bc_v, rms_ca_v));
    inverter_control_state.line_voltage_imbalance_rms_v =
        rms_max_v - rms_min_v;

    window_elapsed_ms =
        current_tick_ms - rms->window_start_tick_ms;
    window_s = (float)window_elapsed_ms * 0.001f;
    if (window_s <= 0.0f) {
        window_s = 0.1f;
    }
    inverter_control_state.line_voltage_rms_window_s = window_s;
    inverter_control_state.line_voltage_rms_window_samples =
        sample_count;
    inverter_control_state.line_voltage_rms_ready = 1U;

    /* 保留1ms节拍状态，只清除已经消费的100ms统计窗口。 */
    rms->sum_ab_v = 0.0f;
    rms->sum_bc_v = 0.0f;
    rms->sum_ca_v = 0.0f;
    rms->square_sum_ab_v2 = 0.0f;
    rms->square_sum_bc_v2 = 0.0f;
    rms->square_sum_ca_v2 = 0.0f;
    rms->sample_count = 0U;
    rms->window_start_tick_ms = current_tick_ms;

    error_v =
        inverter_control_state.line_voltage_reference_rms_v -
        inverter_control_state.line_voltage_feedback_rms_v;
    inverter_control_state.line_voltage_rms_error_v = error_v;

    /* 软启动未完成或反馈异常时不改变幅值补偿。 */
    if ((inverter_control_state.line_voltage_reference_rms_v <
         inverter_control_state.line_voltage_target_rms_v) ||
        (inverter_control_state.line_voltage_feedback_rms_v <
         INVERTER_LINE_RMS_PI_MIN_VALID_V) ||
        (inverter_control_state.line_voltage_feedback_rms_v >
         INVERTER_LINE_RMS_PI_MAX_VALID_V)) {
        inverter_control_state.line_voltage_rms_pi_active = 0U;
        inverter_control_state.line_voltage_rms_pi_saturated = 0U;
        return;
    }

    inverter_control_state.line_voltage_rms_pi_active = 1U;
    proportional_v = INVERTER_LINE_RMS_PI_KP * error_v;
    integral_candidate_v =
        inverter_control_state.line_voltage_rms_pi_integral_v +
        INVERTER_LINE_RMS_PI_KI * error_v * window_s;
    integral_candidate_v =
        Inverter_Control_Clamp(
            integral_candidate_v,
            -INVERTER_LINE_RMS_PI_TRIM_LIMIT_V,
            INVERTER_LINE_RMS_PI_TRIM_LIMIT_V);

    /* SVPWM已过调制且电压仍偏低时，禁止积分项继续向上增长。 */
    if ((inverter_svpwm_state.modulation_scale <
         INVERTER_LINE_RMS_PI_MIN_MOD_SCALE) &&
        (error_v > 0.0f)) {
        integral_candidate_v =
            inverter_control_state.line_voltage_rms_pi_integral_v;
    }

    output_unclamped_v = proportional_v + integral_candidate_v;
    output_v =
        Inverter_Control_Clamp(
            output_unclamped_v,
            -INVERTER_LINE_RMS_PI_TRIM_LIMIT_V,
            INVERTER_LINE_RMS_PI_TRIM_LIMIT_V);

    /* 条件积分抗饱和：误差能把输出拉回限幅区时才继续积分。 */
    if ((output_v == output_unclamped_v) ||
        ((output_unclamped_v >
          INVERTER_LINE_RMS_PI_TRIM_LIMIT_V) &&
         (error_v < 0.0f)) ||
        ((output_unclamped_v <
          -INVERTER_LINE_RMS_PI_TRIM_LIMIT_V) &&
         (error_v > 0.0f))) {
        inverter_control_state.line_voltage_rms_pi_integral_v =
            integral_candidate_v;
    }

    output_unclamped_v =
        proportional_v +
        inverter_control_state.line_voltage_rms_pi_integral_v;
    output_v =
        Inverter_Control_Clamp(
            output_unclamped_v,
            -INVERTER_LINE_RMS_PI_TRIM_LIMIT_V,
            INVERTER_LINE_RMS_PI_TRIM_LIMIT_V);

    inverter_control_state.line_voltage_rms_pi_target_v =
        output_v;

    /* 在100ms后台周期内限速施加PI结果，不向20kHz中断添加斜坡运算。 */
    correction_step_limit_v =
        INVERTER_LINE_RMS_PI_SLEW_V_PER_S * window_s;
    correction_delta_v =
        inverter_control_state.line_voltage_rms_pi_target_v -
        inverter_control_state.line_voltage_rms_pi_correction_v;
    inverter_control_state.line_voltage_rms_pi_correction_v +=
        Inverter_Control_Clamp(
            correction_delta_v,
            -correction_step_limit_v,
            correction_step_limit_v);

    inverter_control_state.line_voltage_rms_pi_saturated =
        ((output_v != output_unclamped_v) ||
         ((inverter_svpwm_state.modulation_scale <
           INVERTER_LINE_RMS_PI_MIN_MOD_SCALE) &&
          (error_v > 0.0f))) ? 1U : 0U;
}

/**
 * @brief 配置双线性变换离散准PR系数
 */
static void Inverter_PR_Configure(
    volatile Inverter_PR_ControllerTypeDef *controller,
    float kp,
    float kr,
    float wc_rad_s,
    float resonant_frequency_hz,
    float output_limit)
{
    float transform_k;
    float w0_rad_s;
    float denominator;

    transform_k = 2.0f * INVERTER_CONTROL_FREQ_HZ;
    w0_rad_s =
        INVERTER_TWO_PI * resonant_frequency_hz;

    denominator =
        transform_k * transform_k +
        2.0f * wc_rad_s * transform_k +
        w0_rad_s * w0_rad_s;

    controller->kp = kp;
    controller->b0 =
        2.0f * kr * wc_rad_s * transform_k /
        denominator;
    controller->b2 = -controller->b0;
    controller->a1 =
        2.0f *
        (w0_rad_s * w0_rad_s -
         transform_k * transform_k) /
        denominator;
    controller->a2 =
        (transform_k * transform_k -
         2.0f * wc_rad_s * transform_k +
         w0_rad_s * w0_rad_s) /
        denominator;
    controller->output_limit = output_limit;
    controller->error_z1 = 0.0f;
    controller->error_z2 = 0.0f;
    controller->resonant_z1 = 0.0f;
    controller->resonant_z2 = 0.0f;
}

/**
 * @brief 清除单个PR历史量，不改变系数
 */
static void Inverter_PR_Reset(
    volatile Inverter_PR_ControllerTypeDef *controller)
{
    controller->error_z1 = 0.0f;
    controller->error_z2 = 0.0f;
    controller->resonant_z1 = 0.0f;
    controller->resonant_z2 = 0.0f;
}

/**
 * @brief 按当前输出频率配置两路电压PR和两路电流PR
 */
static void Inverter_Control_ConfigurePRControllers(
    float output_frequency_hz)
{
    Inverter_PR_Configure(
        &inverter_control_state.voltage_pr_a,
        INVERTER_VOLTAGE_PR_KP,
        INVERTER_VOLTAGE_PR_KR,
        INVERTER_VOLTAGE_PR_WC_RAD_S,
        output_frequency_hz,
        INVERTER_CURRENT_REFERENCE_LIMIT_A);
    Inverter_PR_Configure(
        &inverter_control_state.voltage_pr_c,
        INVERTER_VOLTAGE_PR_KP,
        INVERTER_VOLTAGE_PR_KR,
        INVERTER_VOLTAGE_PR_WC_RAD_S,
        output_frequency_hz,
        INVERTER_CURRENT_REFERENCE_LIMIT_A);
    Inverter_PR_Configure(
        &inverter_control_state.current_pr_a,
        INVERTER_CURRENT_PR_KP,
        INVERTER_CURRENT_PR_KR,
        INVERTER_CURRENT_PR_WC_RAD_S,
        output_frequency_hz,
        INVERTER_VOLTAGE_CORRECTION_LIMIT_V);
    Inverter_PR_Configure(
        &inverter_control_state.current_pr_c,
        INVERTER_CURRENT_PR_KP,
        INVERTER_CURRENT_PR_KR,
        INVERTER_CURRENT_PR_WC_RAD_S,
        output_frequency_hz,
        INVERTER_VOLTAGE_CORRECTION_LIMIT_V);
}

/**
 * @brief 执行一次离散准PR
 */
static float Inverter_PR_Run(
    volatile Inverter_PR_ControllerTypeDef *controller,
    float error)
{
    float resonant;
    float output;
    float limited_output;

    resonant =
        -controller->a1 * controller->resonant_z1 -
        controller->a2 * controller->resonant_z2 +
        controller->b0 * error +
        controller->b2 * controller->error_z2;

    output = controller->kp * error + resonant;
    limited_output =
        Inverter_Control_Clamp(
            output,
            -controller->output_limit,
            controller->output_limit);

    /*
     * 饱和时把本周期谐振状态回算到受限输出，抑制PR历史量继续增大。
     */
    if (limited_output != output) {
        resonant =
            limited_output - controller->kp * error;
        resonant =
            Inverter_Control_Clamp(
                resonant,
                -controller->output_limit,
                controller->output_limit);
    }

    controller->error_z2 = controller->error_z1;
    controller->error_z1 = error;
    controller->resonant_z2 = controller->resonant_z1;
    controller->resonant_z1 = resonant;

    return limited_output;
}

/**
 * @brief 初始化双PR、SVPWM和Timer A/B/C计数器
 */
HAL_StatusTypeDef Inverter_Control_Init(void)
{
    HAL_StatusTypeDef status;

    inverter_control_state =
        (Inverter_Control_StateTypeDef){0};
    inverter_start_request = 0U;
    inverter_stop_request = 0U;
    inverter_frequency_toggle_request = 0U;
    inverter_control_last_status = HAL_OK;
    inverter_control_state.output_frequency_hz =
        INVERTER_OUTPUT_FREQ_DEFAULT_HZ;
    inverter_control_state.line_voltage_target_rms_v =
        Inverter_Control_SelectLineVoltageTargetRms(
            inverter_control_state.output_frequency_hz);
    Inverter_Control_ConfigurePRControllers(
        inverter_control_state.output_frequency_hz);

    status = Inverter_SVPWM_Init();
    if (status != HAL_OK) {
        Inverter_SVPWM_Disable();
        inverter_control_last_status = status;
        return status;
    }

    status = Inverter_SVPWM_StartCounters();
    if (status != HAL_OK) {
        Inverter_SVPWM_Disable();
        inverter_control_last_status = status;
        return status;
    }

    inverter_control_state.initialized = 1U;

#if (INVERTER_CONTROL_AUTO_START != 0U)
    inverter_start_request = 1U;
#endif

    return HAL_OK;
}

/**
 * @brief 在停机状态选择30Hz或60Hz，并同步重算四个PR谐振系数
 */
HAL_StatusTypeDef Inverter_Control_SetOutputFrequency(
    float output_frequency_hz)
{
    float selected_frequency_hz;

    if (inverter_control_state.initialized == 0U) {
        inverter_control_last_status = HAL_ERROR;
        return HAL_ERROR;
    }

    if (inverter_control_state.enabled != 0U) {
        inverter_control_last_status = HAL_BUSY;
        return HAL_BUSY;
    }

    if (fabsf(output_frequency_hz -
              INVERTER_OUTPUT_FREQ_LOW_HZ) <= 0.1f) {
        selected_frequency_hz =
            INVERTER_OUTPUT_FREQ_LOW_HZ;
    } else if (fabsf(output_frequency_hz -
                     INVERTER_OUTPUT_FREQ_HIGH_HZ) <= 0.1f) {
        selected_frequency_hz =
            INVERTER_OUTPUT_FREQ_HIGH_HZ;
    } else {
        inverter_control_last_status = HAL_ERROR;
        return HAL_ERROR;
    }

    inverter_control_state.output_frequency_hz =
        selected_frequency_hz;
    inverter_control_state.line_voltage_target_rms_v =
        Inverter_Control_SelectLineVoltageTargetRms(
            selected_frequency_hz);
    Inverter_Control_ConfigurePRControllers(
        selected_frequency_hz);
    Inverter_Control_Reset();

    inverter_control_last_status = HAL_OK;
    return HAL_OK;
}

/**
 * @brief 清除PR历史量、相角和软启动参考
 */
void Inverter_Control_Reset(void)
{
    Inverter_PR_Reset(
        &inverter_control_state.voltage_pr_a);
    Inverter_PR_Reset(
        &inverter_control_state.voltage_pr_c);
    Inverter_PR_Reset(
        &inverter_control_state.current_pr_a);
    Inverter_PR_Reset(
        &inverter_control_state.current_pr_c);

    Inverter_Control_ResetLineVoltageRmsPI();

    inverter_control_state.phase_rad = 0.0f;
    inverter_control_state.line_voltage_reference_rms_v = 0.0f;
    inverter_control_state.i_a_reference_a = 0.0f;
    inverter_control_state.i_b_reference_a = 0.0f;
    inverter_control_state.i_c_reference_a = 0.0f;
    inverter_control_state.v_a_correction_v = 0.0f;
    inverter_control_state.v_c_correction_v = 0.0f;
    inverter_control_state.v_a_command_v = 0.0f;
    inverter_control_state.v_b_command_v = 0.0f;
    inverter_control_state.v_c_command_v = 0.0f;
    inverter_control_state.m_a = 0.0f;
    inverter_control_state.m_b = 0.0f;
    inverter_control_state.m_c = 0.0f;
}

/**
 * @brief 处理启动、停止和故障状态
 */
void Inverter_Control_Service(float dc_bus_v)
{
    HAL_StatusTypeDef status;

    if (inverter_control_state.initialized == 0U) {
        return;
    }

    inverter_control_state.dc_bus_v = dc_bus_v;

    if (Inverter_ADC_Watchdog_IsFaulted() != 0U) {
        Inverter_Control_Disable();
        inverter_start_request = 0U;
        inverter_stop_request = 0U;
        inverter_frequency_toggle_request = 0U;
        inverter_control_last_status = HAL_ERROR;
        return;
    }

    /*
 * 按键只提交切频请求，实际停机、改频和重启统一在这里完成。
 * 这样运行中不会直接调用SetOutputFrequency()而得到HAL_BUSY。
 */
    if (inverter_frequency_toggle_request != 0U) {
        uint8_t restart_after_switch;
        float target_frequency_hz;

        inverter_frequency_toggle_request = 0U;

        restart_after_switch =
            inverter_control_state.enabled;

        if (inverter_control_state.output_frequency_hz <
            ((INVERTER_OUTPUT_FREQ_LOW_HZ +
              INVERTER_OUTPUT_FREQ_HIGH_HZ) * 0.5f)) {

            target_frequency_hz =
                INVERTER_OUTPUT_FREQ_HIGH_HZ;
              } else {
                  target_frequency_hz =
                      INVERTER_OUTPUT_FREQ_LOW_HZ;
              }

        if (restart_after_switch != 0U) {
            Inverter_Control_Disable();
        }

        status = Inverter_Control_SetOutputFrequency(
            target_frequency_hz);

        if (status != HAL_OK) {
            inverter_control_last_status = status;
            return;
        }

        if (restart_after_switch != 0U) {
            inverter_start_request = 1U;
        }
    }

    /* sqrtf和慢速PI只在主循环运行，不占用20kHz ADC/PWM快速中断。 */
    Inverter_Control_ProcessLineVoltageRmsPI();

    if (inverter_stop_request != 0U) {
        inverter_stop_request = 0U;
        inverter_start_request = 0U;
        Inverter_Control_Disable();
        inverter_control_last_status = HAL_OK;
        return;
    }

    if (inverter_start_request == 0U) {
        return;
    }

    /*
     * 保留启动请求，直到PFC把直流母线建立到安全阈值。
     * 这样自动启动和调试器手动启动都不会在母线过低时误投入。
     */
    if (dc_bus_v <INVERTER_START_DC_BUS_V) {
        inverter_control_last_status = HAL_BUSY;
        return;
    }

    inverter_start_request = 0U;

    if (inverter_control_state.enabled != 0U) {
        inverter_control_last_status = HAL_BUSY;
        return;
    }

    Inverter_Control_Reset();
    status = Inverter_SVPWM_Enable();
    inverter_control_last_status = status;

    if (status == HAL_OK) {
        inverter_control_state.enabled = 1U;
    } else {
        Inverter_SVPWM_Disable();
    }
}

/**
 * @brief 在ADC1全传输回调中执行一次双PR闭环和SVPWM更新
 */
void Inverter_Control_Update(float u_ab_v,
                             float u_bc_v,
                             float i_a_a,
                             float i_c_a,
                             float dc_bus_v)
{
    float phase_peak_v;
    float line_voltage_command_rms_v;
    float u_ca_reference_v;
    float ac_balance_gain;
    float ac_balance_v;
    float voltage_error_a;
    float voltage_error_c;
    float current_error_a;
    float current_error_c;
    float current_reference_max;
    float current_reference_scale;
    float phase_step_rad;

    inverter_control_state.u_ab_v = u_ab_v;
    inverter_control_state.u_bc_v = u_bc_v;
    inverter_control_state.i_a_a = i_a_a;
    inverter_control_state.i_c_a = i_c_a;
    inverter_control_state.i_b_a = -i_a_a - i_c_a;
    inverter_control_state.dc_bus_v = dc_bus_v;

    /*
     * 由Uab、Ubc重构三相三线制等效相电压：
     * Va=(2Uab+Ubc)/3；
     * Vb=(-Uab+Ubc)/3；
     * Vc=-(Uab+2Ubc)/3。
     */
    inverter_control_state.v_a_v =
        (2.0f * u_ab_v + u_bc_v) / 3.0f;
    inverter_control_state.v_b_v =
        (-u_ab_v + u_bc_v) / 3.0f;
    inverter_control_state.v_c_v =
        -(u_ab_v + 2.0f * u_bc_v) / 3.0f;

    if ((inverter_control_state.initialized == 0U) ||
        (inverter_control_state.enabled == 0U)) {
        return;
    }

    if ((Inverter_ADC_Watchdog_IsFaulted() != 0U) ||
        (dc_bus_v < INVERTER_STOP_DC_BUS_V )) {
        Inverter_Control_Disable();
        inverter_control_last_status = HAL_ERROR;
        return;
    }

    inverter_control_state.line_voltage_reference_rms_v +=
        INVERTER_LINE_VOLTAGE_SLEW_V_PER_S /
        INVERTER_CONTROL_FREQ_HZ;
    if (inverter_control_state.line_voltage_reference_rms_v >
        inverter_control_state.line_voltage_target_rms_v) {
        inverter_control_state.line_voltage_reference_rms_v =
            inverter_control_state.line_voltage_target_rms_v;
    }

    /* RMS PI补偿已在后台限幅；快速路径只比原版增加一次浮点加法。 */
    line_voltage_command_rms_v =
        inverter_control_state.line_voltage_reference_rms_v +
        inverter_control_state.line_voltage_rms_pi_correction_v;
    phase_peak_v =
        line_voltage_command_rms_v *
        INVERTER_LINE_RMS_TO_PHASE_PEAK;

    /*
 * 首先生成未补偿的A、C相参考电压。
 */
    inverter_control_state.v_a_reference_v =
        phase_peak_v *
        sinf(inverter_control_state.phase_rad);

    /* ABC正序：B滞后A 120°，C超前A 120°。 */
    inverter_control_state.v_c_reference_v =
        phase_peak_v *
        sinf(inverter_control_state.phase_rad +
             INVERTER_TWO_PI_OVER_THREE);

    /*
     * 计算原始Uca参考：
     *
     * Uca = Vc - Va
     *
     * AC与CA方向相反，但两者RMS大小相同。
     */
    u_ca_reference_v =
        inverter_control_state.v_c_reference_v -
        inverter_control_state.v_a_reference_v;

    /* 根据当前30Hz/60Hz输出频率选择对应的AC/CA补偿系数。 */
    if (inverter_control_state.output_frequency_hz <
        ((INVERTER_OUTPUT_FREQ_LOW_HZ +
          INVERTER_OUTPUT_FREQ_HIGH_HZ) * 0.5f)) {
        ac_balance_gain =
            INVERTER_AC_BALANCE_GAIN_30HZ;
    } else {
        ac_balance_gain =
            INVERTER_AC_BALANCE_GAIN_60HZ;
    }

    /* 正补偿使Va和Vc互相靠近，从而降低AC/CA线电压。 */
    ac_balance_v =
        0.5f *
        ac_balance_gain *
        u_ca_reference_v;

    inverter_control_state.v_a_reference_v +=
        ac_balance_v;

    inverter_control_state.v_c_reference_v -=
        ac_balance_v;

    /*
     * A、C补偿完成后再重构B相，保证：
     *
     * Va + Vb + Vc = 0
     */
    inverter_control_state.v_b_reference_v =
        -inverter_control_state.v_a_reference_v -
        inverter_control_state.v_c_reference_v;

    // inverter_control_state.v_a_reference_v =
    //     phase_peak_v *
    //     sinf(inverter_control_state.phase_rad);
    // /* ABC正序：B滞后A 120°，C超前A 120°。 */
    // inverter_control_state.v_c_reference_v =
    //     phase_peak_v *
    //     sinf(inverter_control_state.phase_rad +
    //          INVERTER_TWO_PI_OVER_THREE);
    // inverter_control_state.v_b_reference_v =
    //     -inverter_control_state.v_a_reference_v -
    //     inverter_control_state.v_c_reference_v;

    voltage_error_a =
        inverter_control_state.v_a_reference_v -
        inverter_control_state.v_a_v;
    voltage_error_c =
        inverter_control_state.v_c_reference_v -
        inverter_control_state.v_c_v;

    inverter_control_state.i_a_reference_a =
        Inverter_PR_Run(
            &inverter_control_state.voltage_pr_a,
            voltage_error_a);
    inverter_control_state.i_c_reference_a =
        Inverter_PR_Run(
            &inverter_control_state.voltage_pr_c,
            voltage_error_c);
    inverter_control_state.i_b_reference_a =
        -inverter_control_state.i_a_reference_a -
        inverter_control_state.i_c_reference_a;

    /*
     * 两个外环独立限幅后，恢复出的B相瞬态值仍可能超过单相限值。
     * 对三相参考统一缩放，保持iA+iB+iC=0且不改变相间比例。
     */
    current_reference_max =
        fmaxf(
            fabsf(inverter_control_state.i_a_reference_a),
            fmaxf(
                fabsf(inverter_control_state.i_b_reference_a),
                fabsf(inverter_control_state.i_c_reference_a)));

    if (current_reference_max >
        INVERTER_CURRENT_REFERENCE_LIMIT_A) {
        current_reference_scale =
            INVERTER_CURRENT_REFERENCE_LIMIT_A /
            current_reference_max;
        inverter_control_state.i_a_reference_a *=
            current_reference_scale;
        inverter_control_state.i_b_reference_a *=
            current_reference_scale;
        inverter_control_state.i_c_reference_a *=
            current_reference_scale;
    }

    current_error_a =
        inverter_control_state.i_a_reference_a -
        inverter_control_state.i_a_a;
    current_error_c =
        inverter_control_state.i_c_reference_a -
        inverter_control_state.i_c_a;

    inverter_control_state.v_a_correction_v =
        Inverter_PR_Run(
            &inverter_control_state.current_pr_a,
            current_error_a);
    inverter_control_state.v_c_correction_v =
        Inverter_PR_Run(
            &inverter_control_state.current_pr_c,
            current_error_c);

    /*
     * 参考相电压前馈与电流PR修正量相加，再由A/C恢复B相。
     */
    inverter_control_state.v_a_command_v =
        inverter_control_state.v_a_reference_v +
        inverter_control_state.v_a_correction_v;
    inverter_control_state.v_c_command_v =
        inverter_control_state.v_c_reference_v +
        inverter_control_state.v_c_correction_v;
    inverter_control_state.v_b_command_v =
        -inverter_control_state.v_a_command_v -
        inverter_control_state.v_c_command_v;
    inverter_control_state.m_a =
        2.0f * inverter_control_state.v_a_command_v /
        dc_bus_v;
    inverter_control_state.m_b =
        2.0f * inverter_control_state.v_b_command_v /
        dc_bus_v;
    inverter_control_state.m_c =
        2.0f * inverter_control_state.v_c_command_v /
        dc_bus_v;

    Inverter_SVPWM_Update(
        inverter_control_state.m_a,
        inverter_control_state.m_b,
        inverter_control_state.m_c);

    phase_step_rad =
        INVERTER_TWO_PI *
        inverter_control_state.output_frequency_hz /
        INVERTER_CONTROL_FREQ_HZ;
    inverter_control_state.phase_rad += phase_step_rad;

    if (inverter_control_state.phase_rad >=
        INVERTER_TWO_PI) {
        inverter_control_state.phase_rad -=
            INVERTER_TWO_PI;
    }

    inverter_control_state.update_count++;
}

/**
 * @brief 请求在主循环中启动三相逆变
 */
void Inverter_Control_RequestStart(void)
{
    inverter_stop_request = 0U;
    inverter_start_request = 1U;
}

/**
 * @brief 请求在主循环中停止三相逆变
 */
void Inverter_Control_RequestStop(void)
{
    inverter_start_request = 0U;
    inverter_stop_request = 1U;
}

/**
 * @brief 请求在低频和高频之间切换
 */
void Inverter_Control_RequestFrequencyToggle(void)
{
    inverter_frequency_toggle_request = 1U;
}

/**
 * @brief 立即关闭三相逆变六路输出并复位控制器
 */
void Inverter_Control_Disable(void)
{
    Inverter_SVPWM_Disable();
    inverter_control_state.enabled = 0U;
    Inverter_Control_Reset();
    Inverter_SVPWM_Reset();
}

#ifndef PFC_CONTROL_H
#define PFC_CONTROL_H

#include <stdint.h>
#include "pfc_pwm.h"

/**
 * @brief PFC控制器工作模式
 */
typedef enum
{
    /**
     * Timer E/F固定占空比模式。
     * 电压环和电流环均不运行，用于检查驱动和功率级开关波形。
     */
    PFC_CONTROL_MODE_EF_FIXED = 0,

    /**
     * 单电流环模式。
     * 母线电压PI外环不运行，使用手动电流有效值给定运行电流PR内环。
     */
    PFC_CONTROL_MODE_CURRENT_LOOP,

    /**
     * 正常PFC闭环模式。
     * 运行母线电压PI外环、电流PR内环和输入电压前馈。
     */
    PFC_CONTROL_MODE_PFC
} PFC_ControlModeTypeDef;

/* ==================== 模式选择与调试给定 ==================== */

/**
 * PFC编译运行模式，只需要修改这一行。
 *
 * 可选值：
 * PFC_CONTROL_MODE_EF_FIXED；
 * PFC_CONTROL_MODE_CURRENT_LOOP；
 * PFC_CONTROL_MODE_PFC。
 */
#ifndef PFC_CONTROL_RUN_MODE
#define PFC_CONTROL_RUN_MODE \
PFC_CONTROL_MODE_PFC
#endif


/**
 * EF固定模式下Timer E上管S1的默认占空比。
 *
 * @note Timer E下管S2由HRTIM死区模块产生互补波。
 */
#define PFC_EF_FIXED_TIMER_E_DUTY 0.50f

/**
 * EF固定模式下Timer F上管S3的默认占空比。
 *
 * @note Timer F下管S4由HRTIM死区模块产生互补波。
 */
#define PFC_EF_FIXED_TIMER_F_DUTY 0.50f

/**
 * 单电流环模式的默认输入电流有效值给定，单位为A RMS。
 *
 * @note 首次带功率调试建议从较小电流开始。
 */
#define PFC_CURRENT_LOOP_REFERENCE_RMS_A 1.5f//只有单电流环才会生效

/* ==================== PFC基本控制参数 ==================== */

/** PFC电流环和PWM命令更新频率，单位为Hz。 */
#define PFC_CONTROL_FREQ_HZ 20000.0f

/** 输入交流电压频率，单位为Hz。 */
#define PFC_GRID_FREQ_HZ 50.0f

/**
 * 尚未完成实时RMS计算时使用的默认输入电压，单位为V RMS。
 */
#define PFC_INPUT_RMS_NOMINAL_V 35.0f

/** 允许接受的最小输入电压有效值。 */
#define PFC_INPUT_RMS_MIN_VALID_V 28.0f

/** 允许接受的最大输入电压有效值。 */
#define PFC_INPUT_RMS_MAX_VALID_V 45.0f

/**
 * Vin/Vin_rms的绝对值上限。
 * 理想正弦峰值为1.414，留出噪声裕量。
 */
#define PFC_INPUT_UNIT_LIMIT 1.55f

/** 输入电压功率前馈最小增益。 */
#define PFC_LINE_FEEDFORWARD_MIN 0.70f

/** 输入电压功率前馈最大增益。 */
#define PFC_LINE_FEEDFORWARD_MAX 1.30f

/** 正常PFC模式的直流母线目标电压，单位为V。 */
#define PFC_BUS_TARGET_V  60.0f

/** 正常PFC模式的母线参考电压软启动斜率，单位为V/s。 */
#define PFC_BUS_REF_SLEW_V_PER_S 2.0f


/** 允许计算调制度的最低母线电压，单位为V。 */
#define PFC_MIN_BUS_V 15.0f

/** 计算PR控制器系数使用的2π常数。 */
#define PFC_TWO_PI 6.28318530718f

/* ==================== 直流母线电压PI外环参数 ==================== */

/**
 * 电压环相对电流环的分频系数。
 *
 * @note 10kHz控制频率、分频10时，电压环执行频率为1kHz。
 */
#define PFC_VOLTAGE_LOOP_DIVIDER 1U

/** 直流母线电压PI控制器比例增益。 */
#define PFC_BUS_PI_KP 0.3f

/** 直流母线电压PI控制器积分增益。 */
#define PFC_BUS_PI_KI 15.0f

/** 电压环和单电流环允许的最大电流有效值给定，单位为A RMS。 */
#define PFC_CURRENT_REF_MAX_RMS_A 7.5f

/* ==================== 输入电流PR内环参数 ==================== */

/** 输入电流PR控制器比例增益。 */
#define PFC_CURRENT_PR_KP 5.0f//4.8f

/** 输入电流PR控制器谐振增益。 */
#define PFC_CURRENT_PR_KR 20.0f//18.0f

/** 输入电流PR控制器谐振带宽，单位为rad/s。 */
#define PFC_CURRENT_PR_WC_RAD_S 5.0f

/** 输入电流PR控制器输出电压绝对值上限，单位为V。 */
#define PFC_CURRENT_PR_OUTPUT_LIMIT_V 18.0f

/**
 * PFC归一化调制度绝对值上限。
 *
 * @note 与双桥臂单极性倍频PWM的单管最大占空比保持一致。
 *       PFC_PWM_MAX_ACTIVE_DUTY=0.98时，本值为0.96。
 */
#define PFC_MODULATION_LIMIT \
    PFC_PWM_MAX_MODULATION

#if (PFC_VOLTAGE_LOOP_DIVIDER == 0U)
#error "PFC_VOLTAGE_LOOP_DIVIDER must be greater than zero"
#endif

/**
 * @brief PFC直流母线电压PI外环运行状态
 */
typedef struct
{
    float integral;         /**< 电压PI积分项，输出单位为A RMS。 */
    uint16_t divider_count; /**< 电压外环相对电流内环的分频计数值。 */
} PFC_ControlVoltagePITypeDef;

/**
 * @brief PFC输入电流PR内环运行状态
 */
typedef struct
{
    float b0;         /**< PR谐振支路当前误差前向系数。 */
    float a1;         /**< PR谐振支路前一次输出反馈系数。 */
    float a2;         /**< PR谐振支路前两次输出反馈系数。 */
    float error_1;    /**< 前一次输入电流误差，单位为A。 */
    float error_2;    /**< 前两次输入电流误差，单位为A。 */
    float resonant_1; /**< 前一次PR谐振支路输出，单位为V。 */
    float resonant_2; /**< 前两次PR谐振支路输出，单位为V。 */
} PFC_ControlCurrentPRTypeDef;

/**
 * @brief PFC控制算法内部历史状态
 */
typedef struct
{
    PFC_ControlVoltagePITypeDef voltage_pi; /**< 母线电压PI外环历史状态。 */
    PFC_ControlCurrentPRTypeDef current_pr; /**< 输入电流PR内环历史状态。 */
} PFC_ControlRuntimeTypeDef;

/**
 * @brief PFC模式命令与对外调试状态
 */
typedef struct
{
    PFC_ControlModeTypeDef mode;     /**< PFC_CONTROL_RUN_MODE对应的运行模式。 */
    float fixed_timer_e_duty;        /**< EF固定模式的Timer E上管占空比。 */
    float fixed_timer_f_duty;        /**< EF固定模式的Timer F上管占空比。 */
    float current_loop_reference_rms_a; /**< 单电流环模式的手动电流给定。 */
    float input_voltage_rms_v;/*** 控制器当前使用的输入交流有效值，单位为V RMS。*/
    float input_voltage_unit;  /*** 输入瞬时电压归一化值Vin/Vin_rms。*/
    float line_feedforward_gain;/*** 输入电压功率前馈增益，等于35V/实测输入RMS。*/
    uint8_t input_voltage_rms_valid;/*** 输入电压RMS有效标志。*/
    float bus_reference_v;          /**< 软启动后的母线参考电压，单位为V。 */
    float bus_feedback_v;           /**< 母线反馈电压，单位为V。 */
    float bus_error_v;              /**< 母线参考值与反馈值之差，单位为V。 */
    float current_command_nominal_rms_a;    /*** 电压PI输出的额定输入电压下电流命令，单位为A RMS。* 该值还没有乘输入电压功率前馈。*/
    float current_reference_rms_a;  /**< 当前实际使用的电流有效值给定。 */
    float current_reference_a;      /**< 与输入电压同相的瞬时电流给定。 */
    float current_error_a;          /**< 瞬时电流给定与采样电流之差。 */
    float current_pr_output_v;      /**< 电流PR内环输出修正电压，单位为V。 */
    float bridge_voltage_command_v; /**< PFC全桥交流侧电压指令，单位为V。 */
    float modulation;               /**< 电流环/PFC模式的有符号调制度。 */
    uint8_t modulation_saturated;   /**< 调制度限幅标志，1表示达到限幅。 */
} PFC_ControlStateTypeDef;

/** PFC模式命令与运行数据，可直接在调试器中观察。 */
extern volatile PFC_ControlStateTypeDef pfc_control_state;

/**
 * @brief          初始化三模式PFC控制器
 * @param[in]      initial_bus_voltage_v 启动时实测母线电压，单位为V
 * @retval         none
 *
 * @note           初始化后采用PFC_CONTROL_RUN_MODE宏选择的模式。
 */
void PFC_Control_Init(float initial_bus_voltage_v);

/**
 * @brief          清除PI/PR历史状态并从当前母线电压重新开始
 * @param[in]      initial_bus_voltage_v 复位时实测母线电压，单位为V
 * @retval         none
 *
 * @note           本函数保留当前模式、固定占空比和单电流环给定。
 */
void PFC_Control_Reset(float initial_bus_voltage_v);

/**
 * @brief          设置EF固定模式的Timer E/F上管占空比
 * @param[in]      timer_e_duty Timer E上管S1占空比
 * @param[in]      timer_f_duty Timer F上管S3占空比
 * @retval         none
 *
 * @note           两个参数均会被限制到0至PFC_PWM_MAX_ACTIVE_DUTY。
 *                 S2和S4仍由HRTIM死区模块产生互补输出。
 *                 固定模式不根据交流极性换相，带功率前必须确认
 *                 E/F给定、输入极性和限流条件。
 */
void PFC_Control_SetFixedEFDuty(float timer_e_duty,
                                float timer_f_duty);

/**
 * @brief          设置单电流环模式的输入电流有效值给定
 * @param[in]      current_reference_rms_a 电流有效值给定，单位为A RMS
 * @retval         none
 */
void PFC_Control_SetCurrentReferenceRms(
    float current_reference_rms_a);

/**
 * @brief          更新控制器使用的输入交流电压有效值
 * @param[in]      input_voltage_rms_v 输入电压有效值，单位为V RMS
 * @retval         none
 */
void PFC_Control_SetInputVoltageRms(
    float input_voltage_rms_v);

/**
 * @brief          根据当前模式执行一次控制算法
 * @param[in]      input_voltage_v 输入交流瞬时电压，单位为V
 * @param[in]      input_current_a 输入交流瞬时电流，单位为A
 * @param[in]      bus_voltage_v 经过数字滤波的母线电压，单位为V
 * @retval         float 电流环/PFC模式的有符号调制度
 *
 * @note           EF固定模式返回0；该模式的E/F占空比保存在状态结构体中。
 */
float PFC_Control_Run(float input_voltage_v,
                      float input_current_a,
                      float bus_voltage_v);

/**
 * @brief          计算第一组模式命令并使能PFC四路PWM输出
 * @param[in]      input_voltage_v 输入交流瞬时电压，单位为V
 * @param[in]      input_current_a 输入交流瞬时电流，单位为A
 * @param[in]      bus_voltage_v 经过数字滤波的母线电压，单位为V
 * @retval         HAL_StatusTypeDef HAL执行状态
 *
 * @note           本函数不触发运行时软件更新。第一组命令在下一个
 *                 HRTIM Master更新边界生效。
 */
HAL_StatusTypeDef PFC_Control_EnablePWM(
    float input_voltage_v,
    float input_current_a,
    float bus_voltage_v);

/**
 * @brief          计算并预装载下一个PWM周期的模式命令
 * @param[in]      input_voltage_v 输入交流瞬时电压，单位为V
 * @param[in]      input_current_a 输入交流瞬时电流，单位为A
 * @param[in]      bus_voltage_v 经过数字滤波的母线电压，单位为V
 * @retval         none
 *
 * @note           本函数只写Timer E/F的CMP预装载值。当前PWM周期不变，
 *                 新命令在下一个HRTIM Master更新边界同步生效。
 */
void PFC_Control_UpdatePWMNextPeriod(
    float input_voltage_v,
    float input_current_a,
    float bus_voltage_v);

#endif /* PFC_CONTROL_H */

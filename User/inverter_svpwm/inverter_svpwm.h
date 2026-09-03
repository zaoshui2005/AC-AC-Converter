#ifndef INVERTER_SVPWM_H
#define INVERTER_SVPWM_H

#include <stdint.h>

#include "hrtim.h"
/* ==================== HRTIM与三相桥映射 ==================== */

/** Timer A互补输出驱动功率管1和功率管4。 */
#define INVERTER_SVPWM_PHASE_A_OUTPUTS \
    (HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2)

/** Timer B互补输出驱动功率管3和功率管6。 */
#define INVERTER_SVPWM_PHASE_B_OUTPUTS \
    (HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2)

/** Timer C互补输出驱动功率管5和功率管2。 */
#define INVERTER_SVPWM_PHASE_C_OUTPUTS \
    (HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2)

/** 三相桥六路HRTIM功率输出位掩码。 */
#define INVERTER_SVPWM_ALL_OUTPUTS \
    (INVERTER_SVPWM_PHASE_A_OUTPUTS | \
     INVERTER_SVPWM_PHASE_B_OUTPUTS | \
     INVERTER_SVPWM_PHASE_C_OUTPUTS)

/** 三相逆变使用的HRTIM计数器，Master由PFC应用先行启动。 */
#define INVERTER_SVPWM_COUNTERS \
    (HRTIM_TIMERID_TIMER_A | \
     HRTIM_TIMERID_TIMER_B | \
     HRTIM_TIMERID_TIMER_C)

/** CMP1与0、PER边界之间保留的最小计数。 */
#define INVERTER_SVPWM_COMPARE_GUARD_COUNTS 24U

/**
 * @brief 连续SVPWM输出和调试状态
 *
 * @note 本模块同时负责Timer A/B/C的初始化、计数器、输出使能、
 *       紧急关断以及连续SVPWM计算。
 */
typedef struct
{
    float requested_m_a;      /**< 控制器请求的A相归一化调制量。 */
    float requested_m_b;      /**< 控制器请求的B相归一化调制量。 */
    float requested_m_c;      /**< 控制器请求的C相归一化调制量。 */
    float modulation_scale;   /**< 过调制时对三相指令统一施加的缩放系数。 */
    float zero_sequence;      /**< -(max+min)/2连续零序分量。 */
    float applied_m_a;        /**< 注入零序后的A相调制量。 */
    float applied_m_b;        /**< 注入零序后的B相调制量。 */
    float applied_m_c;        /**< 注入零序后的C相调制量。 */
    float duty_a_next;        /**< 下一周期Timer A上管占空比。 */
    float duty_b_next;        /**< 下一周期Timer B上管占空比。 */
    float duty_c_next;        /**< 下一周期Timer C上管占空比。 */
    uint32_t period_counts;   /**< Timer A/B/C中心对齐计数峰值。 */
    uint32_t compare_a_next;  /**< 下一周期Timer A CMP1值。 */
    uint32_t compare_b_next;  /**< 下一周期Timer B CMP1值。 */
    uint32_t compare_c_next;  /**< 下一周期Timer C CMP1值。 */
    uint8_t sector;           /**< 按三相调制量大小关系得到的扇区1至6。 */
    uint8_t initialized;       /**< SVPWM和HRTIM输出后端已初始化。 */
    uint8_t counters_running;  /**< Timer A/B/C计数器已经启动。 */
    uint8_t outputs_enabled;   /**< 三相桥六路功率输出已经使能。 */
} Inverter_SVPWM_StateTypeDef;

/** SVPWM运行状态，可在调试器中观察。 */
extern volatile Inverter_SVPWM_StateTypeDef inverter_svpwm_state;

/**
 * @brief          初始化Timer A/B/C安全比较值和SVPWM状态
 * @retval         HAL_OK 初始化成功，六路功率输出保持关闭
 * @retval         HAL_ERROR Timer A/B/C周期无效或不一致
 * @retval         其他HAL状态 HRTIM操作失败
 */
HAL_StatusTypeDef Inverter_SVPWM_Init(void);

/**
 * @brief          启动Timer A/B/C计数器，但不使能六路功率输出
 * @retval         HAL_StatusTypeDef HAL执行状态
 *
 * @note           必须在HRTIM Master采样时基启动之后调用。
 */
HAL_StatusTypeDef Inverter_SVPWM_StartCounters(void);

/**
 * @brief          以三相50%同相安全状态使能六路互补输出
 * @retval         HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef Inverter_SVPWM_Enable(void);

/**
 * @brief          计算连续SVPWM零序并预装载下一周期三相CMP1
 * @param[in]      m_a A相归一化相电压指令，定义为2乘Va_cmd除以Vdc
 * @param[in]      m_b B相归一化相电压指令，定义为2乘Vb_cmd除以Vdc
 * @param[in]      m_c C相归一化相电压指令，定义为2乘Vc_cmd除以Vdc
 *
 * @note           使用等效的min-max零序注入：m0=-(max+min)/2。
 *                 过调制时三相统一缩放，不分别削顶。
 */
void Inverter_SVPWM_Update(float m_a,
                           float m_b,
                           float m_c);

/**
 * @brief          清除SVPWM调制状态
 *
 * @note           保留HRTIM周期、初始化、计数器运行和输出使能状态。
 */
void Inverter_SVPWM_Reset(void);

/**
 * @brief          通过ODISR立即关闭Timer A/B/C六路功率输出
 */
void Inverter_SVPWM_Disable(void);

#endif /* INVERTER_SVPWM_H */

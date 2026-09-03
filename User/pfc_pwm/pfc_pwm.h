#ifndef PFC_PWM_H
#define PFC_PWM_H

#include <stdint.h>
#include "hrtim.h"

/* ==================== HRTIM与功率开关映射 ==================== */

/**
 * @brief PFC全桥功率开关与HRTIM输出的对应关系
 *
 * @note  S1：左桥臂上管，PC8/HRTIM1_CHE1；
 *        S2：左桥臂下管，PC9/HRTIM1_CHE2；
 *        S3：右桥臂上管，PC6/HRTIM1_CHF1；
 *        S4：右桥臂下管，PC7/HRTIM1_CHF2。
 */
#define PFC_PWM_S1_OUTPUT HRTIM_OUTPUT_TE1 /**< S1对应Timer E Output 1。 */
#define PFC_PWM_S2_OUTPUT HRTIM_OUTPUT_TE2 /**< S2对应Timer E Output 2。 */
#define PFC_PWM_S3_OUTPUT HRTIM_OUTPUT_TF1 /**< S3对应Timer F Output 1。 */
#define PFC_PWM_S4_OUTPUT HRTIM_OUTPUT_TF2 /**< S4对应Timer F Output 2。 */

/** PFC四路HRTIM功率输出位掩码。 */
#define PFC_PWM_ALL_OUTPUTS \
    (PFC_PWM_S1_OUTPUT | PFC_PWM_S2_OUTPUT | \
     PFC_PWM_S3_OUTPUT | PFC_PWM_S4_OUTPUT)

/** PFC使用的HRTIM计数器：Master负责同步更新，Timer E/F产生两桥臂PWM。 */
#define PFC_PWM_COUNTERS \
    (HRTIM_TIMERID_MASTER | \
     HRTIM_TIMERID_TIMER_E | \
     HRTIM_TIMERID_TIMER_F)

/* ==================== 可调PWM参数 ==================== */

/**
 * Timer E/F单个上管允许的最大占空比。
 * 保留2%余量，避免比较值过于靠近计数器边界。
 */
#define PFC_PWM_MAX_ACTIVE_DUTY 0.98f

/**
 * 双桥臂单极性倍频PWM允许的有符号调制度绝对值上限。
 *
 * @note duty_e=0.5*(1+m)，duty_f=0.5*(1-m)。
 *       单管最大占空比为0.98时，|m|最大只能为0.96，
 *       才能同时保证另一桥臂占空比不低于0.02并保持两边对称。
 */
#define PFC_PWM_MAX_MODULATION \
    (2.0f * PFC_PWM_MAX_ACTIVE_DUTY - 1.0f)

/**
 * 输入电压过零判断阈值，单位为V。
 * 绝对值不超过该阈值时，强制m=0，两桥臂均输出50%同相PWM，
 * 因而桥侧差模电压为0。
 */
#define PFC_PWM_ZERO_CROSS_V 0.05f

/**
 * Timer E/F的CMP1与0、PER边界之间保留的最小计数。
 *
 * @note 当前工程Timer E/F使用HRTIM_PRESCALERRATIO_MUL4，
 *       对应STM32G474在Up/Down模式下要求的24个计数边界余量。
 *       如果修改HRTIM预分频，必须按参考手册同步修改本参数。
 */
#define PFC_PWM_COMPARE_GUARD_COUNTS 24U

/**
 * @brief 下一PWM周期将使用的开关工作模式
 */
typedef enum
{
    PFC_PWM_MODE_DISABLED = 0, /**< 四路功率输出均被关闭。 */
    PFC_PWM_MODE_ZERO,         /**< 过零区：S1/S3均为50%同相PWM。 */
    PFC_PWM_MODE_POSITIVE,     /**< 正半周：双桥臂产生0/+Vbus倍频PWM。 */
    PFC_PWM_MODE_NEGATIVE,     /**< 负半周：双桥臂产生0/-Vbus倍频PWM。 */
    PFC_PWM_MODE_FIXED_EF      /**< Timer E/F分别保持指定的固定占空比。 */
} PFC_PWM_ModeTypeDef;

/**
 * @brief PFC PWM运行和调试状态
 *
 * @note  compare_e_next和compare_f_next是本控制周期写入的预装载值，
 *        在下一个HRTIM Master更新事件才成为活动比较值。
 */
typedef struct
{
    float requested_modulation;   /**< 控制器给出的有符号调制度。 */
    float timer_e_duty_next;      /**< 下一周期Timer E上管S1的实际占空比。 */
    float timer_f_duty_next;      /**< 下一周期Timer F上管S3的实际占空比。 */
    uint32_t period_counts;       /**< Timer E/F中心对齐计数峰值。 */
    uint32_t compare_e_next;      /**< 下一周期Timer E CMP1预装载值。 */
    uint32_t compare_f_next;      /**< 下一周期Timer F CMP1预装载值。 */
    PFC_PWM_ModeTypeDef mode_next;/**< 下一周期的开关工作模式。 */
    uint8_t initialized;          /**< PWM模块已经初始化标志。 */
    uint8_t outputs_enabled;      /**< PFC四路HRTIM输出已经使能标志。 */
} PFC_PWM_StateTypeDef;

extern volatile PFC_PWM_StateTypeDef pfc_pwm_state; /**< PFC PWM运行状态。 */

/**
 * @brief          初始化PFC PWM运行状态和安全比较值
 * @param[in]      none
 * @retval         HAL_OK 初始化成功
 * @retval         HAL_ERROR Timer E/F周期无效或周期不一致
 *
 * @note           Timer E/F的静态波形配置由threenibian.ioc生成。
 *                 本函数必须在MX_HRTIM1_Init()之后、使能功率输出之前调用；
 *                 初始化阶段四路功率输出保持关闭。
 */
HAL_StatusTypeDef PFC_PWM_Init(void);

/**
 * @brief          同步启动HRTIM Master、Timer E和Timer F计数器
 * @param[in]      none
 * @retval         HAL_StatusTypeDef HAL执行状态
 *
 * @note           Master必须运行，Timer E/F的预装载值才能按照当前CubeMX
 *                 配置在下一Master更新事件同步生效。本函数只启动计数器，
 *                 不使能TE1/TE2/TF1/TF2功率输出。
 */
HAL_StatusTypeDef PFC_PWM_StartCounters(void);

/**
 * @brief          预装载第一组调制度并使能四路功率输出
 * @param[in]      input_voltage_v 当前输入交流瞬时电压，单位为V
 * @param[in]      modulation 当前控制器有符号调制度
 * @retval         HAL_StatusTypeDef HAL执行状态
 *
 * @note           本函数不触发软件更新。调用时当前活动值仍是初始化建立的
 *                 安全零占空比；第一组控制量在下一个Master更新边界生效。
 *                 闭环模式使用双桥臂单极性倍频PWM：
 *                 S1占空比=0.5*(1+m)，S3占空比=0.5*(1-m)；
 *                 S2/S4由HRTIM硬件生成带死区的互补输出。
 *                 本函数仅用于投入PWM，后续每周期必须调用
 *                 PFC_PWM_UpdateNextPeriod()。
 */
HAL_StatusTypeDef PFC_PWM_Enable(float input_voltage_v,
                                 float modulation);

/**
 * @brief          预装载第一组Timer E/F固定占空比并使能四路功率输出
 * @param[in]      timer_e_duty Timer E上管S1固定占空比
 * @param[in]      timer_f_duty Timer F上管S3固定占空比
 * @retval         HAL_StatusTypeDef HAL执行状态
 *
 * @note           S2和S4仍由HRTIM死区模块产生互补输出。本函数不触发
 *                 软件更新；第一组固定值在下一个Master更新边界生效。
 *                 固定模式不执行交流极性换相，只用于受控调试。
 */
HAL_StatusTypeDef PFC_PWM_EnableFixed(float timer_e_duty,
                                      float timer_f_duty);

/**
 * @brief          计算并预装载下一个PWM周期的Timer E/F占空比
 * @param[in]      input_voltage_v 当前输入交流瞬时电压，单位为V
 * @param[in]      modulation 当前控制器有符号调制度
 * @retval         none
 *
 * @note           本函数只写CMP1预装载寄存器，不触发软件更新：
 *                 当前PWM周期保持不变；
 *                 下一个Master更新事件同时装载Timer E/F新比较值；
 *                 因此新占空比从下一个完整PWM周期开始生效。
 *                 m>0时产生0/+Vbus，m<0时产生0/-Vbus。
 */
void PFC_PWM_UpdateNextPeriod(float input_voltage_v,
                              float modulation);

/**
 * @brief          预装载下一个PWM周期的Timer E/F固定占空比
 * @param[in]      timer_e_duty Timer E上管S1固定占空比
 * @param[in]      timer_f_duty Timer F上管S3固定占空比
 * @retval         none
 *
 * @note           两个占空比相互独立，均限制到0至
 *                 PFC_PWM_MAX_ACTIVE_DUTY。函数只写CMP1预装载寄存器，
 *                 新值在下一个Master更新边界同步生效。
 *                 固定模式不执行交流极性换相，只用于受控调试。
 */
void PFC_PWM_UpdateFixedNextPeriod(float timer_e_duty,
                                   float timer_f_duty);

/**
 * @brief          立即关闭PFC四路功率输出
 * @param[in]      none
 * @retval         none
 *
 * @note           停机和故障关断不等待下一周期，以保证保护速度。
 *                 关闭输出后如需重新投入，应重新初始化控制器并调用
 *                 PFC_PWM_Enable()。
 */
void PFC_PWM_Disable(void);

#endif

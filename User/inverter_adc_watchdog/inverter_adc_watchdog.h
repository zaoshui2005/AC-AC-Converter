#ifndef INVERTER_ADC_WATCHDOG_H
#define INVERTER_ADC_WATCHDOG_H

#include <stdint.h>
#include "adc.h"
#include "inverter_adc.h"
/** 两路逆变电流允许的绝对值上限，单位为A。 */
#define INVERTER_ADC_CURRENT_LIMIT_A          8.0f

/** 当前无三相逆变ADC看门狗故障。 */
#define INVERTER_ADC_FAULT_NONE               0U

/** ADC2 AWD1检测到任一路逆变电流超出公共安全窗口。 */
#define INVERTER_ADC_FAULT_OVERCURRENT        (1UL << 0)

/**
 * @brief 三相逆变ADC看门狗运行状态
 */
typedef struct
{
    uint16_t current_1_low_count;
    /**< 第一路电流按自身校准零点和增益计算的下限。 */

    uint16_t current_1_high_count;
    /**< 第一路电流按自身校准零点和增益计算的上限。 */

    uint16_t current_2_low_count;
    /**< 第二路电流按自身校准零点和增益计算的下限。 */

    uint16_t current_2_high_count;
    /**< 第二路电流按自身校准零点和增益计算的上限。 */

    uint16_t common_low_count;
    /**< ADC2 AWD1实际写入的公共窗口下限。 */

    uint16_t common_high_count;
    /**< ADC2 AWD1实际写入的公共窗口上限。 */

    uint32_t fault;
    /**< 软件锁存故障位。 */

    uint32_t trip_count;
    /**< ADC2 AWD1累计触发次数。 */
} Inverter_ADC_WatchdogStateTypeDef;

/** 三相逆变ADC看门狗对外运行状态。 */
extern volatile Inverter_ADC_WatchdogStateTypeDef
    inverter_adc_watchdog_state;

/**
 * @brief          根据校准零点、增益和电流上限配置ADC2 AWD1实际阈值
 * @param[in]      none
 * @retval         HAL_OK 配置成功
 * @retval         HAL_ERROR 两路安全窗口没有公共交集
 *
 * @note           IOC必须配置：ADC2 AWD1、ALL_REG、IT使能、0~4095初始窗口。
 * @note           本函数不启动ADC2，不执行ADC内部自动校准。
 * @note           必须在三相逆变四路上电零点校准完成后调用。
 * @note           调用前必须先停止ADC2 DMA，写入阈值后再重新启动ADC2。
 */
HAL_StatusTypeDef Inverter_ADC_Watchdog_Init(void);

/**
 * @brief          处理ADC2 AWD1中断标志
 * @param[in]      none
 * @retval         none
 *
 * @note           必须放在ADC1_2_IRQHandler()中，且位于
 *                 HAL_ADC_IRQHandler(&hadc2)之前调用。
 * @note           该入口只处理ADC2，不修改ADC1/PFC看门狗逻辑。
 */
void Inverter_ADC_Watchdog_IRQHandler(void);

/**
 * @brief          清除逆变ADC看门狗故障并重新允许ADC2 AWD1中断
 * @param[in]      none
 * @retval         none
 *
 * @note           本函数不会重新打开HRTIM三相逆变输出。
 */
void Inverter_ADC_Watchdog_ClearFault(void);

/**
 * @brief          判断逆变ADC看门狗是否锁存故障
 * @param[in]      none
 * @retval         0 未锁存故障
 * @retval         1 已锁存故障
 */
uint8_t Inverter_ADC_Watchdog_IsFaulted(void);

#endif /* INVERTER_ADC_WATCHDOG_H */

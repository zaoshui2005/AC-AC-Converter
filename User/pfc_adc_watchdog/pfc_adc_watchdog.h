#ifndef PFC_ADC_WATCHDOG_H
#define PFC_ADC_WATCHDOG_H

#include <stdint.h>
#include "pfc_adc.h"

#define PFC_ADC_CURRENT_LIMIT_A       13.0f /**< PFC输入电流硬件过流阈值，单位为A。 */
#define PFC_ADC_BUS_OVERVOLTAGE_V    85.0f /**< PFC直流母线硬件过压阈值，单位为V。 */

#define PFC_ADC_FAULT_NONE             0U        /**< 当前没有ADC故障。 */
#define PFC_ADC_FAULT_OVERCURRENT     (1UL << 0) /**< PFC输入过流故障。 */
#define PFC_ADC_FAULT_BUS_OVERVOLTAGE (1UL << 1) /**< PFC母线过压故障。 */
#define PFC_ADC_FAULT_ADC_ERROR       (1UL << 2) /**< ADC采样或DMA错误。 */

extern volatile uint32_t pfc_adc_fault; /**< PFC ADC故障锁存标志。 */

/**
* @brief          关闭并清除ADC1全部硬件看门狗中断状态
* @param[in]      none
* @retval         none
*
* @note           仅用于功率输出关闭的启动采样和零点校准阶段。
*/
void PFC_ADC_Watchdog_Disable(void);

/**
* @brief          配置PFC输入过流和母线过压ADC硬件看门狗
* @param[in]      none
* @retval         HAL_StatusTypeDef HAL执行状态
*
* @note           必须在pfc_adc_state.calibration.ready变为1后调用。
*/
HAL_StatusTypeDef PFC_ADC_Watchdog_Init(void);

/**
* @brief          清除PFC ADC软件故障锁存标志
* @param[in]      none
* @retval         none
*
* @note           本函数不会重新打开HRTIM输出。
*/
void PFC_ADC_Watchdog_ClearFault(void);

#endif /* PFC_ADC_WATCHDOG_H */
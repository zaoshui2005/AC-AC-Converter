#ifndef PFC_APP_H
#define PFC_APP_H

#include <stdint.h>
#include "main.h"

/**
 * 等待PFC输入电压和输入电流零点校准完成的最长时间，单位为ms。
 */
#define PFC_APP_CALIBRATION_TIMEOUT_MS 1000U

/**
 * ADC硬件看门狗配置并重新启动DMA后，等待测量值更新的时间，单位为ms。
 */
#define PFC_APP_MEASUREMENT_SETTLING_MS 2U

/**
 * 调试器或按键逻辑将本变量写为1，请求启动PFC四路PWM输出。
 *
 * @note PFC_App_Loop()处理请求后会自动清零。
 */
extern volatile uint8_t pfc_start_request;

/**
 * 调试器或按键逻辑将本变量写为1，请求立即关闭PFC四路PWM输出。
 *
 * @note PFC_App_Loop()处理请求后会自动清零。
 */
extern volatile uint8_t pfc_stop_request;

/**
 * PFC应用层最近一次HAL执行状态，可在调试器中观察。
 */
extern volatile HAL_StatusTypeDef pfc_app_last_status;

/**
 * @brief          为逆变上电零点校准启动安全采样
 * @param[in]      none
 * @retval         HAL_OK ADC DMA和HRTIM采样时基已经启动
 * @retval         HAL_BUSY 安全采样已经启动过
 * @retval         其他HAL状态 底层模块启动失败
 *
 * @note           本函数关闭ADC1看门狗，初始化安全比较值并启动
 *                 ADC1 DMA及HRTIM Master、Timer E/F计数器。
 * @note           不初始化PFC控制器、不提交PFC启动请求，也不使能
 *                 TE1、TE2、TF1、TF2功率输出。
 */
HAL_StatusTypeDef PFC_App_StartSamplingForCalibration(void);

/**
 * @brief          完成PFC看门狗、控制器和应用状态初始化
 * @param[in]      none
 * @retval         HAL_OK 初始化成功，系统已经就绪但PWM功率输出仍然关闭
 * @retval         其他HAL状态 底层模块初始化失败
 *
 * @note           必须在PFC_App_StartSamplingForCalibration()成功、
 *                 逆变四路零点校准完成且逆变电流看门狗恢复后调用。
 * @note           本函数才会配置PFC看门狗、初始化PFC控制器并提交
 *                 pfc_start_request；功率输出仍由主循环稍后使能。
 */
HAL_StatusTypeDef PFC_App_Init(void);

/**
 * @brief          处理PFC启动、停机和故障状态
 * @param[in]      none
 * @retval         none
 *
 * @note           在main()的while(1)中持续调用。20kHz控制算法不在本函数
 *                 中执行，而是在ADC半传输完成回调中执行。
 */
void PFC_App_Loop(void);

#endif /* PFC_APP_H */
#include "pfc_app.h"
#include "pfc_adc.h"
#include "pfc_adc_watchdog.h"
#include "pfc_control.h"
#include "pfc_pwm.h"

/** 写1请求启动PFC，PFC_App_Loop()处理后自动清零。 */
volatile uint8_t pfc_start_request;

/** 写1请求停止PFC，PFC_App_Loop()处理后自动清零。 */
volatile uint8_t pfc_stop_request;

/** PFC应用层最近一次HAL执行状态。 */
volatile HAL_StatusTypeDef pfc_app_last_status;

/** PFC_App_Init()已经成功完成标志。 */
static uint8_t pfc_app_initialized;

/** 校准用ADC DMA和HRTIM采样时基已经安全启动标志。 */
static uint8_t pfc_calibration_sampling_started;

/**
 * @brief          为逆变上电零点校准启动安全采样
 * @param[in]      none
 * @retval         HAL_OK 启动成功，全部功率输出保持关闭
 * @retval         HAL_BUSY 安全采样已经启动过
 * @retval         其他HAL状态 底层模块启动失败
 */
HAL_StatusTypeDef PFC_App_StartSamplingForCalibration(void)
{
    HAL_StatusTypeDef status;

    if (pfc_calibration_sampling_started != 0U) {
        return HAL_BUSY;
    }

    pfc_start_request = 0U;
    pfc_stop_request = 0U;
    pfc_app_last_status = HAL_OK;
    pfc_app_initialized = 0U;

    /* 校准阶段不允许ADC1看门狗产生PFC故障或打开功率输出。 */
    PFC_ADC_Watchdog_Disable();
    PFC_ADC_Watchdog_ClearFault();

    /* 先建立安全比较值，并通过ODISR保持PFC四路功率输出关闭。 */
    status = PFC_PWM_Init();
    if (status != HAL_OK) {
        pfc_app_last_status = status;
        return status;
    }

    /* ADC1 DMA先进入等待HRTIM_TRG1的状态。 */
    status = PFC_ADC_SamplingStart();
    if (status != HAL_OK) {
        pfc_app_last_status = status;
        return status;
    }

    /*
     * 校准阶段只保留ADC1 DMA全传输中断来处理v1、v2、a1、a2。
     * 暂停半传输中断，避免正式PFC初始化前运行PFC采样和控制回调。
     * PFC_ADC_Watchdog_Init()稍后重启ADC1 DMA时会恢复半传输中断。
     */
    __HAL_DMA_DISABLE_IT(
        hadc1.DMA_Handle,
        DMA_IT_HT);

    /*
     * 只启动Master和Timer E/F计数器来产生采样触发；
     * TE1、TE2、TF1、TF2功率输出仍由ODISR保持关闭。
     */
    status = PFC_PWM_StartCounters();
    if (status != HAL_OK) {
        (void)PFC_ADC_Stop();
        pfc_app_last_status = status;
        return status;
    }

    pfc_calibration_sampling_started = 1U;
    pfc_app_last_status = HAL_OK;

    return HAL_OK;
}

/**
 * @brief          完成PFC看门狗、控制器和应用状态初始化
 * @param[in]      none
 * @retval         HAL_OK 初始化成功，PWM功率输出保持关闭
 * @retval         其他HAL状态 底层模块初始化失败
 */
HAL_StatusTypeDef PFC_App_Init(void)
{
    HAL_StatusTypeDef status;

    pfc_start_request = 0U;
    pfc_stop_request = 0U;
    pfc_app_last_status = HAL_OK;
    pfc_app_initialized = 0U;

    if (pfc_calibration_sampling_started == 0U) {
        pfc_app_last_status = HAL_ERROR;
        return HAL_ERROR;
    }

    /* 校准采样阶段若出现ADC错误，不允许继续初始化或启动PFC。 */
    if (pfc_adc_fault != PFC_ADC_FAULT_NONE) {
        PFC_PWM_Disable();
        pfc_app_last_status = HAL_ERROR;
        return HAL_ERROR;
    }

    /*
     * 四路逆变零点已经完成，现在才配置PFC输入过流和母线过压
     * ADC硬件看门狗。配置过程会停止并重新启动ADC1 DMA。
     */
    status = PFC_ADC_Watchdog_Init();
    if (status != HAL_OK) {
        PFC_PWM_Disable();
        pfc_app_last_status = status;
        return status;
    }

    /* 等待DMA重启后产生有效的输入和母线测量值。 */
    HAL_Delay(PFC_APP_MEASUREMENT_SETTLING_MS);

    /*
     * 如果看门狗在测量稳定期间已经触发，保持功率输出关闭，
     * 不允许控制器进入待启动状态。
     */
    if (pfc_adc_fault != PFC_ADC_FAULT_NONE) {
        PFC_PWM_Disable();
        pfc_app_last_status = HAL_ERROR;
        return HAL_ERROR;
    }

    PFC_Control_Init(
        pfc_adc_state.measurement.bus_voltage_v);

    pfc_app_initialized = 1U;
    pfc_app_last_status = HAL_OK;
    /* 所有初始化、校准和保护检查完成后才提交启动请求。 */
    // HAL_Delay(2000);
    pfc_start_request = 1U;

    return HAL_OK;
}

/**
 * @brief          处理PFC启动、停机和故障状态
 * @param[in]      none
 * @retval         none
 */
void PFC_App_Loop(void)
{
    HAL_StatusTypeDef status;

    if (pfc_app_initialized == 0U) {
        return;
    }

    /*
     * ADC看门狗中断已经通过ODISR异步关闭硬件输出。
     * 此处调用PFC_PWM_Disable()用于同步PWM软件状态，并阻止重新启动。
     */
    if (pfc_adc_fault != PFC_ADC_FAULT_NONE) {
        PFC_PWM_Disable();
        pfc_start_request = 0U;
        pfc_stop_request = 0U;
        pfc_app_last_status = HAL_ERROR;
        return;
    }

    if (pfc_stop_request != 0U) {
        pfc_stop_request = 0U;
        pfc_start_request = 0U;
        PFC_PWM_Disable();
        pfc_app_last_status = HAL_OK;
        return;
    }

    if (pfc_start_request == 0U) {
        return;
    }

    pfc_start_request = 0U;

    if (pfc_pwm_state.outputs_enabled != 0U) {
        pfc_app_last_status = HAL_BUSY;
        return;
    }

    /* 每次重新启动前清除PI和PR历史量。 */
    PFC_Control_Reset(
        pfc_adc_state.measurement.bus_voltage_v);

    /*
     * 计算第一组控制命令并使能四路功率输出。
     * 第一组新比较值会在下一个Master更新边界生效。
     */
    status = PFC_Control_EnablePWM(
        pfc_adc_state.measurement.input_voltage_v,
        pfc_adc_state.measurement.input_current_a,
        pfc_adc_state.measurement.bus_voltage_v);

    pfc_app_last_status = status;

    if (status != HAL_OK) {
        PFC_PWM_Disable();
    }
}
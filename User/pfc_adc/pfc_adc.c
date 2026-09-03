#include "pfc_adc.h"
#include "pfc_adc_watchdog.h"
#include "pfc_control.h"
#include <math.h>

/** PFC ADC对外运行数据。 */
volatile PFC_ADC_StateTypeDef pfc_adc_state;

/** PFC ADC内部校准和数字滤波运行状态。 */
static PFC_ADC_RuntimeTypeDef pfc_adc_runtime;

/**
 * @brief          对直流母线电压进行100Hz二阶陷波
 * @param[in]      input_v 未滤波母线电压，单位为V
 * @retval         经过100Hz陷波后的母线电压，单位为V
 */
static float PFC_ADC_VBusNotch100Hz(float input_v)
{
    PFC_ADC_NotchStateTypeDef *notch;
    float output_v;

    notch = &pfc_adc_runtime.bus_voltage_notch;

    /*
     * 首次运行时让历史输入和输出等于当前输入，
     * 防止滤波器启动时产生较大的瞬态波动。
     */
    if (notch->initialized == 0U) {
        notch->x1 = input_v;
        notch->x2 = input_v;
        notch->y1 = input_v;
        notch->y2 = input_v;
        notch->initialized = 1U;

        return input_v;
    }

    output_v =
        PFC_ADC_VBUS_NOTCH_B0 * input_v +
        PFC_ADC_VBUS_NOTCH_B1 * notch->x1 +
        PFC_ADC_VBUS_NOTCH_B2 * notch->x2 -
        PFC_ADC_VBUS_NOTCH_A1 * notch->y1 -
        PFC_ADC_VBUS_NOTCH_A2 * notch->y2;

    notch->x2 = notch->x1;
    notch->x1 = input_v;
    notch->y2 = notch->y1;
    notch->y1 = output_v;

    return output_v;
}

/**
 * @brief          对直流母线电压进行500Hz一阶低通滤波
 * @param[in]      input_v 经过100Hz陷波后的母线电压，单位为V
 * @retval         经过500Hz低通后的母线电压，单位为V
 */
static float PFC_ADC_VBusLowPass500Hz(float input_v)
{
    PFC_ADC_LowPassStateTypeDef *lpf;

    lpf = &pfc_adc_runtime.bus_voltage_lpf;

    if (lpf->initialized == 0U) {
        lpf->output_v = input_v;
        lpf->initialized = 1U;
    } else {
        lpf->output_v +=
            PFC_ADC_VBUS_LPF_ALPHA *
            (input_v - lpf->output_v);
    }

    return lpf->output_v;
}

/**
 * @brief          更新输入交流电压有效值
 * @param[in]      input_voltage_v 输入交流瞬时电压，单位为V
 * @retval         none
 *
 * @note           20kHz采样、50Hz输入时，每400点计算一次RMS。
 */
static void PFC_ADC_UpdateInputVoltageRms(
    float input_voltage_v)
{
    PFC_ADC_InputVoltageRmsTypeDef *rms;
    float rms_unfiltered_v;

    rms = &pfc_adc_runtime.input_voltage_rms;

    /* 累加瞬时电压平方。 */
    rms->square_sum_v2 +=
        input_voltage_v * input_voltage_v;

    rms->sample_count++;

    /* 未收集满一个50Hz周期时不更新结果。 */
    if (rms->sample_count <
        PFC_ADC_VIN_RMS_WINDOW_SAMPLES) {
        return;
        }

    /* Vrms=sqrt(sum(v²)/N)。 */
    rms_unfiltered_v =
        sqrtf(
            rms->square_sum_v2 /
            (float)PFC_ADC_VIN_RMS_WINDOW_SAMPLES);

    pfc_adc_state.measurement
        .input_voltage_rms_unfiltered_v =
        rms_unfiltered_v;

    /* 第一个周期直接建立初值。 */
    if (rms->initialized == 0U) {
        rms->filtered_rms_v =
            rms_unfiltered_v;

        rms->initialized = 1U;
    } else {
        /* 对每周期计算出的RMS进行一阶平滑。 */
        rms->filtered_rms_v +=
            PFC_ADC_VIN_RMS_FILTER_ALPHA *
            (rms_unfiltered_v -
             rms->filtered_rms_v);
    }

    pfc_adc_state.measurement.input_voltage_rms_v =
        rms->filtered_rms_v;

    pfc_adc_state.measurement.input_voltage_rms_ready =
        1U;

    /* 开始下一个交流周期。 */
    rms->square_sum_v2 = 0.0f;
    rms->sample_count = 0U;
}

/**
* @brief          校准ADC1并启动六Rank循环DMA采样
* @param[in]      none
* @retval         HAL_StatusTypeDef HAL执行状态
*/
HAL_StatusTypeDef PFC_ADC_Start(void)
{
    return PFC_ADC_SamplingStart();
}

/**
* @brief          停止ADC1循环DMA采样
* @param[in]      none
* @retval         HAL_StatusTypeDef HAL执行状态
*/
HAL_StatusTypeDef PFC_ADC_Stop(void)
{
    return HAL_ADC_Stop_DMA(&hadc1);
}

/**
* @brief          处理ADC1 DMA前三个Rank半传输完成事件
* @param[in]      none
* @retval         none
*
* @note           DMA长度为6，半传输中断产生时Rank1至Rank3已经搬运完成。
*/
void PFC_ADC_ProcessHalfTransfer(void)
{
    // PFC_ADC_CalibrationWorkTypeDef *calibration;
    float bus_voltage_v;
    float bus_voltage_notched_v;

    // calibration = &pfc_adc_runtime.calibration;

    /*
     * 立即锁存前三个Rank。
     * 调试器停机后DMA数组可能继续变化，但CPU快照不会变化。
     */
    pfc_adc_state.raw.input_current =
        pfc_adc_state.dma.sample[PFC_ADC_IIN_INDEX];
    pfc_adc_state.raw.input_voltage =
        pfc_adc_state.dma.sample[PFC_ADC_VIN_INDEX];
    pfc_adc_state.raw.bus_voltage =
        pfc_adc_state.dma.sample[PFC_ADC_VBUS_INDEX];

    // /*
    //  * 按原工程流程进行上电零点校准。
    //  * 校准期间交流输入、电感电流和PWM功率输出必须为0。
    //  */
    // if (pfc_adc_state.calibration.ready == 0U) {
    //     if (calibration->discard_count <
    //         PFC_ADC_CALIB_DISCARD) {
    //         calibration->discard_count++;
    //         return;
    //     }
    //
    //     calibration->current_offset_sum +=
    //         pfc_adc_state.raw.input_current;
    //     calibration->voltage_offset_sum +=
    //         pfc_adc_state.raw.input_voltage;
    //     calibration->sample_count++;
    //
    //     if (calibration->sample_count >=
    //         PFC_ADC_CALIB_SAMPLES) {
    //         pfc_adc_state.calibration.current_offset_count =
    //             (float)calibration->current_offset_sum /
    //             (float)PFC_ADC_CALIB_SAMPLES;
    //         pfc_adc_state.calibration.voltage_offset_count =
    //             (float)calibration->voltage_offset_sum /
    //             (float)PFC_ADC_CALIB_SAMPLES;
    //         pfc_adc_state.calibration.ready = 1U;
    //     }
    //     return;
    // }

    pfc_adc_state.measurement.input_current_a =
        ((float)pfc_adc_state.raw.input_current -
         pfc_adc_state.calibration.current_offset_count) *
        PFC_ADC_IIN_A_PER_COUNT;

    pfc_adc_state.measurement.input_voltage_v =
        ((float)pfc_adc_state.raw.input_voltage -
         pfc_adc_state.calibration.voltage_offset_count) *
        PFC_ADC_VIN_V_PER_COUNT;//改符号

    PFC_ADC_UpdateInputVoltageRms(
    pfc_adc_state.measurement.input_voltage_v);

    bus_voltage_v =
        ((float)pfc_adc_state.raw.bus_voltage -
         PFC_ADC_VBUS_OFFSET_COUNT) *
        PFC_ADC_VBUS_V_PER_COUNT;

    /* 保存倍率换算后、尚未滤波的直流母线电压。 */
    pfc_adc_state.measurement.bus_voltage_unfiltered_v =
        bus_voltage_v;

    /* 消除单相整流产生的100Hz直流母线二倍频纹波。 */
    bus_voltage_notched_v =
        PFC_ADC_VBusNotch100Hz(bus_voltage_v);
    pfc_adc_state.measurement.bus_voltage_notched_v =
        bus_voltage_notched_v;

    /* 继续使用500Hz低通抑制ADC噪声和开关高频干扰。 */
    pfc_adc_state.measurement.bus_voltage_v =
        PFC_ADC_VBusLowPass500Hz(
            bus_voltage_notched_v);
}

/**
 * @brief          处理ADC1规则组DMA半传输完成事件
 * @param[in]      hadc ADC句柄地址
 * @retval         none
 */
void HAL_ADC_ConvHalfCpltCallback(
    ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) {
        return;
    }

    /*
     * 必须先处理本次ADC采样：
     * 锁存原始值、换算物理量、计算输入电压RMS、
     * 更新母线陷波和低通结果。
     */
    PFC_ADC_ProcessHalfTransfer();

    /* 把实时输入交流电压RMS送给控制器。 */
    if (pfc_adc_state.measurement
            .input_voltage_rms_ready != 0U) {
        PFC_Control_SetInputVoltageRms(
            pfc_adc_state.measurement
                .input_voltage_rms_v);
            }

    /*
     * 无故障且PWM已投入时，
     * 计算并预装载下一PWM周期的控制量。
     */
    if ((pfc_adc_state.calibration.ready != 0U) &&
        (pfc_adc_fault == PFC_ADC_FAULT_NONE) &&
        (pfc_pwm_state.outputs_enabled != 0U)) {
        PFC_Control_UpdatePWMNextPeriod(
            pfc_adc_state.measurement.input_voltage_v,
            pfc_adc_state.measurement.input_current_a,
            pfc_adc_state.measurement.bus_voltage_v);
        }
}

/**
* @brief          初始化并启动ADC1六Rank DMA循环采样
* @param[in]      none
* @retval         HAL_OK 启动成功
* @retval         其他HAL状态 ADC校准或DMA启动失败
*
* @note           ADC1规则组数量必须为6，前三个Rank通道顺序必须为：
*                 Rank1：PA3/ADC1_IN4，PFC输入电流；
*                 Rank2：PC2/ADC1_IN8，PFC输入电压；
*                 Rank3：PA1/ADC1_IN2，PFC母线电压；
*                 Rank4至Rank6由原工程其他ADC1采样通道使用。
*/
HAL_StatusTypeDef PFC_ADC_SamplingStart(void)
{
    HAL_StatusTypeDef status;

    /* 先停止ADC1 DMA，避免清空结构体时DMA仍在写入缓冲区。 */
    (void)HAL_ADC_Stop_DMA(&hadc1);

    /* 一次性清空全部对外采样数据和内部运行状态。 */
    pfc_adc_state =
        (PFC_ADC_StateTypeDef){0};
    pfc_adc_runtime =
        (PFC_ADC_RuntimeTypeDef){0};

    /* 设置自动校准完成前使用的输入电压和输入电流默认零点。 */
    pfc_adc_state.calibration.current_offset_count =
        PFC_ADC_IIN_OFFSET_COUNT;
    pfc_adc_state.calibration.voltage_offset_count =
        PFC_ADC_VIN_OFFSET_COUNT;
    /*
 * 使用固定零点，不进行启动自动校准。
 */
    pfc_adc_state.calibration.ready = 1U;

    /* 执行ADC1单端输入校准。 */
    status = HAL_ADCEx_Calibration_Start(
        &hadc1,
        ADC_SINGLE_ENDED);

    if (status != HAL_OK) {
        return status;
    }

    /* 启动ADC1六Rank循环DMA采样，等待HRTIM_TRG1触发。 */
    status = HAL_ADC_Start_DMA(
        &hadc1,
        (uint32_t *)pfc_adc_state.dma.sample,
        PFC_ADC_DMA_LENGTH);

    return status;
}
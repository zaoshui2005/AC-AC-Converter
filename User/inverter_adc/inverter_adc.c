#include "inverter_adc.h"
#include "inverter_control.h"
#include "pfc_adc.h"
/** 三相逆变ADC采样对外运行数据。 */
volatile Inverter_ADC_StateTypeDef inverter_adc_state;

/** ADC1六Rank循环DMA缓冲区首地址。 */
static const volatile uint16_t *inverter_adc1_dma_buffer;

/** 三相逆变四路一次性上电零点校准过程。 */
typedef struct
{
    uint32_t discard_count;
    uint32_t sample_count;
    uint32_t voltage_1_zero_sum;
    uint32_t voltage_2_zero_sum;
    uint32_t current_1_zero_sum;
    uint32_t current_2_zero_sum;
} Inverter_ADC_ZeroCalibrationWorkTypeDef;

/** 三相逆变四路一次性上电零点校准过程数据。 */
static Inverter_ADC_ZeroCalibrationWorkTypeDef
    inverter_adc_zero_calibration_work;

/**
 * @brief          累加一次四路同步采样并在样本足够时计算零点
 * @retval         none
 */
static void Inverter_ADC_ProcessZeroCalibration(
    uint16_t voltage_1_raw,
    uint16_t voltage_2_raw,
    uint16_t current_1_raw,
    uint16_t current_2_raw)
{
    Inverter_ADC_ZeroCalibrationWorkTypeDef *calibration;

    calibration = &inverter_adc_zero_calibration_work;

    if (calibration->discard_count <
        INVERTER_ADC_ZERO_CALIB_DISCARD_SAMPLES) {
        calibration->discard_count++;
        return;
    }

    calibration->voltage_1_zero_sum += voltage_1_raw;
    calibration->voltage_2_zero_sum += voltage_2_raw;
    calibration->current_1_zero_sum += current_1_raw;
    calibration->current_2_zero_sum += current_2_raw;
    calibration->sample_count++;

    if (calibration->sample_count >=
        INVERTER_ADC_ZERO_CALIB_SAMPLES) {
        inverter_adc_state.zero_calibration.voltage_1_zero_count =
            (float)calibration->voltage_1_zero_sum /
            (float)calibration->sample_count;
        inverter_adc_state.zero_calibration.voltage_2_zero_count =
            (float)calibration->voltage_2_zero_sum /
            (float)calibration->sample_count;
        inverter_adc_state.zero_calibration.current_1_zero_count =
            (float)calibration->current_1_zero_sum /
            (float)calibration->sample_count;
        inverter_adc_state.zero_calibration.current_2_zero_count =
            (float)calibration->current_2_zero_sum /
            (float)calibration->sample_count;

        /* 四个零点全部写完后再置位，主循环此后才能使用结果。 */
        inverter_adc_state.zero_calibration.ready = 1U;
    }
}

/**
 * @brief          初始化并启动三相逆变ADC采样
 * @param[in]      adc1_dma_buffer ADC1 DMA 缓冲区地址，
 *                                 与 PFC 采样共用同一个 ADC1 DMA 缓冲区
 * @retval         HAL_OK    初始化并启动成功
 * @retval         HAL_ERROR ADC或DMA启动失败
 * @note           必须在 MX_DMA_Init()、MX_ADC1_Init()、
 *                 MX_ADC2_Init() 和 MX_HRTIM1_Init() 执行完成后调用
 */
HAL_StatusTypeDef Inverter_App_Init(
    const volatile uint16_t *adc1_dma_buffer)
{
    /* 绑定 ADC1 DMA 缓冲区，并初始化三相逆变采样状态。 */
    if (Inverter_ADC_Init(adc1_dma_buffer) != HAL_OK) {
        return HAL_ERROR;
    }

    /*
     * 启动 ADC2 DMA。
     * ADC2 只负责同步采集两路线电流，不产生 DMA 半传输和全传输中断。
     */
    if (Inverter_ADC_Start() != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief          初始化三相逆变ADC采样模块
 * @param[in]      adc1_dma_buffer ADC1六Rank循环DMA缓冲区首地址
 * @retval         HAL_OK 初始化成功
 * @retval         HAL_ERROR adc1_dma_buffer为空
 */
HAL_StatusTypeDef Inverter_ADC_Init(
    const volatile uint16_t *adc1_dma_buffer)
{
    if (adc1_dma_buffer == NULL) {
        return HAL_ERROR;
    }

    inverter_adc_state =
        (Inverter_ADC_StateTypeDef){0};

    inverter_adc_zero_calibration_work =
        (Inverter_ADC_ZeroCalibrationWorkTypeDef){0};

    inverter_adc1_dma_buffer = adc1_dma_buffer;

    /* 校准完成并写入动态阈值前，禁止ADC2电流看门狗中断。 */
    __HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_AWD1);
    CLEAR_BIT(hadc2.State, HAL_ADC_STATE_AWD1);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_AWD1);

    return HAL_OK;
}

/**
 * @brief          启动ADC2两Rank循环DMA采样
 * @param[in]      none
 * @retval         HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef Inverter_ADC_Start(void)
{
    HAL_StatusTypeDef status;

    /*
     * 停止可能残留的ADC2 DMA。
     * 本模块按要求不调用HAL_ADCEx_Calibration_Start()。
     */
    (void)HAL_ADC_Stop_DMA(&hadc2);

    inverter_adc_state.dma =
        (Inverter_ADC_DmaBufferTypeDef){0};

    /*
     * 启动ADC2两Rank循环DMA：
     * Rank1=PA6/ADC2_IN3，Rank2=PA7/ADC2_IN4。
     * ADC2等待HRTIM_TRG1上升沿，不会自由运行。
     */
    status = HAL_ADC_Start_DMA(
        &hadc2,
        (uint32_t *)inverter_adc_state.dma.sample,
        INVERTER_ADC2_DMA_LENGTH);

    if (status != HAL_OK) {
        return status;
    }

    /*
     * ADC2只负责硬件触发和DMA搬运。
     * 当前IOC未使能DMA1 Channel4 NVIC，这里再同时关闭DMA通道的
     * HT、TC、TE中断源和NVIC入口，避免后续配置变化引入ADC2 DMA中断。
     * ADC2模拟看门狗走ADC1_2_IRQn，不受这里影响。
     */
    __HAL_DMA_DISABLE_IT(
        hadc2.DMA_Handle,
        DMA_IT_HT | DMA_IT_TC | DMA_IT_TE);

    HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);

    __HAL_DMA_CLEAR_FLAG(
        hadc2.DMA_Handle,
        __HAL_DMA_GET_GI_FLAG_INDEX(
            hadc2.DMA_Handle));

    HAL_NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn);

    return HAL_OK;
}

/**
 * @brief          等待v1、v2、a1、a2上电零点校准完成
 * @param[in]      timeout_ms 最长等待时间，单位为ms
 * @retval         HAL_OK 四路零点校准完成
 * @retval         HAL_TIMEOUT 等待超时
 */
HAL_StatusTypeDef Inverter_ADC_WaitForZeroCalibration(
    uint32_t timeout_ms)
{
    uint32_t start_tick;

    start_tick = HAL_GetTick();

    while (inverter_adc_state.zero_calibration.ready == 0U) {
        if ((HAL_GetTick() - start_tick) >= timeout_ms) {
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

/**
 * @brief          停止ADC2循环DMA采样
 * @param[in]      none
 * @retval         HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef Inverter_ADC_Stop(void)
{
    return HAL_ADC_Stop_DMA(&hadc2);
}

/**
 * @brief          处理ADC1六Rank DMA全传输完成事件
 * @param[in]      none
 * @retval         none
 */
void Inverter_ADC_ProcessFullTransfer(void)
{
    uint16_t voltage_1_raw;
    uint16_t voltage_2_raw;
    uint16_t current_1_raw;
    uint16_t current_2_raw;

    if (inverter_adc1_dma_buffer == NULL) {
        return;
    }

    /*
     * ADC1 Rank4和Rank5为两路逆变电压。
     * Rank6为占位采样，保证ADC1全传输事件发生在后三个Rank完成后。
     */
    voltage_1_raw =
        inverter_adc1_dma_buffer[
            INVERTER_ADC_VOLTAGE_1_INDEX];

    voltage_2_raw =
        inverter_adc1_dma_buffer[
            INVERTER_ADC_VOLTAGE_2_INDEX];

    /*
     * ADC1和ADC2由同一个HRTIM_TRG1同时触发。
     * ADC2只有两个Rank，转换早于ADC1六Rank完成，因此此处直接锁存。
     */
    current_1_raw =
        inverter_adc_state.dma.sample[
            INVERTER_ADC_CURRENT_1_INDEX];

    current_2_raw =
        inverter_adc_state.dma.sample[
            INVERTER_ADC_CURRENT_2_INDEX];

    inverter_adc_state.raw.voltage_1 = voltage_1_raw;
    inverter_adc_state.raw.voltage_2 = voltage_2_raw;
    inverter_adc_state.raw.current_1 = current_1_raw;
    inverter_adc_state.raw.current_2 = current_2_raw;

    /* 校准期间只累加四路零点，不换算物理量，也不运行逆变闭环。 */
    if (inverter_adc_state.zero_calibration.ready == 0U) {
        Inverter_ADC_ProcessZeroCalibration(
            voltage_1_raw,
            voltage_2_raw,
            current_1_raw,
            current_2_raw);
        return;
    }

    /* 根据上电校准零点和手动标定倍率换算两路逆变电压。 */
    inverter_adc_state.measurement.voltage_1_v =
        ((float)voltage_1_raw -
         inverter_adc_state.zero_calibration.voltage_1_zero_count) *
        INVERTER_ADC_VOLTAGE_1_V_PER_COUNT;

    inverter_adc_state.measurement.voltage_2_v =
        ((float)voltage_2_raw -
         inverter_adc_state.zero_calibration.voltage_2_zero_count) *
        INVERTER_ADC_VOLTAGE_2_V_PER_COUNT;

    /* 根据上电校准零点和手动标定增益换算两路逆变电流。 */
    inverter_adc_state.measurement.current_1_a =
        ((float)current_1_raw -
         inverter_adc_state.zero_calibration.current_1_zero_count) *
        INVERTER_ADC_CURRENT_1_A_PER_COUNT;

    inverter_adc_state.measurement.current_2_a =
        ((float)current_2_raw -
         inverter_adc_state.zero_calibration.current_2_zero_count) *
        INVERTER_ADC_CURRENT_2_A_PER_COUNT;

    inverter_adc_state.update_count++;
    inverter_adc_state.data_ready = 1U;

    /*
     * ADC1 Rank1~3半传输运行PFC；Rank4~6全传输到达后，
     * ADC2两路电流也已同步搬运完成，在此执行一次20kHz逆变闭环。
     */
    Inverter_Control_Update(
        inverter_adc_state.measurement.voltage_1_v,
        inverter_adc_state.measurement.voltage_2_v,
        inverter_adc_state.measurement.current_1_a,
        inverter_adc_state.measurement.current_2_a,
        pfc_adc_state.measurement.bus_voltage_v);
}

/**
 * @brief          清除三相逆变采样结果和更新标志
 * @param[in]      none
 * @retval         none
 */
void Inverter_ADC_ClearData(void)
{
    inverter_adc_state.raw =
        (Inverter_ADC_RawDataTypeDef){0};

    inverter_adc_state.measurement =
        (Inverter_ADC_MeasurementTypeDef){0};

    inverter_adc_state.update_count = 0U;
    inverter_adc_state.data_ready = 0U;
}

/**
 * @brief          ADC规则组DMA全传输完成普通HAL回调
 * @param[in]      hadc ADC句柄地址
 * @retval         none
 *
 * @note           工程中未启用HAL动态回调注册。
 * @note           ADC2 DMA的HT/TC中断已关闭，因此正常情况下只处理ADC1。
 */
void HAL_ADC_ConvCpltCallback(
    ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) {
        return;
    }

    Inverter_ADC_ProcessFullTransfer();
}

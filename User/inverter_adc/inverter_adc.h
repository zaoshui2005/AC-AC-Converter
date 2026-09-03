#ifndef INVERTER_ADC_H
#define INVERTER_ADC_H

#include <stdint.h>
#include "adc.h"

/** ADC参考电压，单位为V。 */
#define INVERTER_ADC_VREF                       3.3f

/** 12位ADC满量程计数值。 */
#define INVERTER_ADC_FULL_SCALE                 4095.0f

/** ADC2规则组Rank数量，同时也是ADC2循环DMA缓冲区长度。 */
#define INVERTER_ADC2_DMA_LENGTH                2U

/** ADC1六Rank DMA中第一路逆变电压索引：Rank4/ADC1_IN6/PC0。 */
#define INVERTER_ADC_VOLTAGE_1_INDEX            3U

/** ADC1六Rank DMA中第二路逆变电压索引：Rank5/ADC1_IN7/PC1。 */
#define INVERTER_ADC_VOLTAGE_2_INDEX            4U

/** ADC2两Rank DMA中第一路逆变电流索引：Rank1/ADC2_IN3/PA6。 */
#define INVERTER_ADC_CURRENT_1_INDEX            0U

/** ADC2两Rank DMA中第二路逆变电流索引：Rank2/ADC2_IN4/PA7。 */
#define INVERTER_ADC_CURRENT_2_INDEX            1U

/** 上电零点校准开始前丢弃的四路同步采样次数。 */
#define INVERTER_ADC_ZERO_CALIB_DISCARD_SAMPLES 2048U

/** 上电零点校准用于求平均值的四路同步采样次数。 */
#define INVERTER_ADC_ZERO_CALIB_SAMPLES         4096U

/** 等待三相逆变四路零点校准完成的最长时间，单位为ms。 */
#define INVERTER_ADC_ZERO_CALIB_TIMEOUT_MS      1000U

/** 第一路逆变电压采样还原倍率：实际电压/ADC引脚电压。 */
#define INVERTER_ADC_VOLTAGE_1_SCALE            60.40f//60.35f//62.47f//59.25f//59.68f//60.51f

/** 第二路逆变电压采样还原倍率：实际电压/ADC引脚电压。 */
#define INVERTER_ADC_VOLTAGE_2_SCALE            60.09f//59.95f//60.59f//59.35f//59.35f//59.84f

/** 第一路逆变电流采样增益，单位为V/A。 */
#define INVERTER_ADC_CURRENT_1_GAIN_V_PER_A     0.08334f//0.08334f

/** 第二路逆变电流采样增益，单位为V/A。 */
#define INVERTER_ADC_CURRENT_2_GAIN_V_PER_A     0.08334f//0.08334f

/** 第一路逆变电压每个ADC计数对应的实际电压，单位为V/count。 */
#define INVERTER_ADC_VOLTAGE_1_V_PER_COUNT \
    (INVERTER_ADC_VREF * INVERTER_ADC_VOLTAGE_1_SCALE / \
     INVERTER_ADC_FULL_SCALE)

/** 第二路逆变电压每个ADC计数对应的实际电压，单位为V/count。 */
#define INVERTER_ADC_VOLTAGE_2_V_PER_COUNT \
    (INVERTER_ADC_VREF * INVERTER_ADC_VOLTAGE_2_SCALE / \
     INVERTER_ADC_FULL_SCALE)

/** 第一路逆变电流每个ADC计数对应的实际电流，单位为A/count。 */
#define INVERTER_ADC_CURRENT_1_A_PER_COUNT \
    (INVERTER_ADC_VREF / INVERTER_ADC_FULL_SCALE / \
     INVERTER_ADC_CURRENT_1_GAIN_V_PER_A)

/** 第二路逆变电流每个ADC计数对应的实际电流，单位为A/count。 */
#define INVERTER_ADC_CURRENT_2_A_PER_COUNT \
    (INVERTER_ADC_VREF / INVERTER_ADC_FULL_SCALE / \
     INVERTER_ADC_CURRENT_2_GAIN_V_PER_A)

/**
 * @brief ADC2两Rank循环DMA缓冲区
 */
typedef struct
{
    uint16_t sample[INVERTER_ADC2_DMA_LENGTH];
    /**< sample[0]=ADC2_IN3，sample[1]=ADC2_IN4。 */
} Inverter_ADC_DmaBufferTypeDef;

/**
 * @brief 三相逆变四路采样原始ADC快照
 */
typedef struct
{
    uint16_t voltage_1; /**< ADC1 Rank4/ADC1_IN6/PC0原始值。 */
    uint16_t voltage_2; /**< ADC1 Rank5/ADC1_IN7/PC1原始值。 */
    uint16_t current_1; /**< ADC2 Rank1/ADC2_IN3/PA6原始值。 */
    uint16_t current_2; /**< ADC2 Rank2/ADC2_IN4/PA7原始值。 */
} Inverter_ADC_RawDataTypeDef;

/**
 * @brief 三相逆变采样物理量
 */
typedef struct
{
    float voltage_1_v; /**< 第一路逆变电压，单位为V。 */
    float voltage_2_v; /**< 第二路逆变电压，单位为V。 */
    float current_1_a; /**< 第一路逆变电流，单位为A。 */
    float current_2_a; /**< 第二路逆变电流，单位为A。 */
} Inverter_ADC_MeasurementTypeDef;

/**
 * @brief 三相逆变四路上电零点校准结果
 */
typedef struct
{
    float voltage_1_zero_count; /**< v1自动校准得到的零点ADC计数。 */
    float voltage_2_zero_count; /**< v2自动校准得到的零点ADC计数。 */
    float current_1_zero_count; /**< a1自动校准得到的零点ADC计数。 */
    float current_2_zero_count; /**< a2自动校准得到的零点ADC计数。 */
    uint8_t ready;              /**< 四路零点均已校准完成标志。 */
} Inverter_ADC_ZeroCalibrationTypeDef;

/**
 * @brief 三相逆变ADC采样运行状态
 */
typedef struct
{
    Inverter_ADC_DmaBufferTypeDef dma;
    /**< ADC2两路电流循环DMA缓冲区。 */

    Inverter_ADC_RawDataTypeDef raw;
    /**< 本控制周期锁存的四路原始值。 */

    Inverter_ADC_MeasurementTypeDef measurement;
    /**< 本控制周期换算后的四路物理量。 */

    Inverter_ADC_ZeroCalibrationTypeDef zero_calibration;
    /**< v1、v2、a1、a2的一次性上电零点校准结果。 */

    uint32_t update_count;
    /**< 校准完成后四路测量值正常更新次数。 */

    uint8_t data_ready;
    /**< 四路采样已更新标志，1表示数据有效。 */
} Inverter_ADC_StateTypeDef;

/** 三相逆变ADC采样对外运行数据。 */
extern volatile Inverter_ADC_StateTypeDef inverter_adc_state;

/**
 * @brief          初始化三相逆变ADC采样模块
 * @param[in]      adc1_dma_buffer ADC1六Rank循环DMA缓冲区首地址
 * @retval         HAL_OK 初始化成功
 * @retval         HAL_ERROR adc1_dma_buffer为空
 *
 * @note           本函数不启动ADC；采样启动后只执行一次四路零点校准。
 */
HAL_StatusTypeDef Inverter_ADC_Init(
    const volatile uint16_t *adc1_dma_buffer);

/**
 * @brief          启动ADC2两Rank循环DMA采样
 * @param[in]      none
 * @retval         HAL_OK 启动成功
 * @retval         其他HAL状态 ADC2 DMA启动失败
 *
 * @note           ADC2启动后等待HRTIM_TRG1上升沿。
 * @note           本函数不执行ADC内部自动校准；首次触发后开始零点采样。
 */
HAL_StatusTypeDef Inverter_ADC_Start(void);

/**
 * @brief          等待v1、v2、a1、a2上电零点校准完成
 * @param[in]      timeout_ms 最长等待时间，单位为ms
 * @retval         HAL_OK 四路零点校准完成
 * @retval         HAL_TIMEOUT 等待超时
 *
 * @note           仅等待初始化时已开始的校准，不会重新发起校准。
 * @note           调用前必须已经启动ADC1、ADC2和HRTIM采样触发。
 */
HAL_StatusTypeDef Inverter_ADC_WaitForZeroCalibration(
    uint32_t timeout_ms);

/**
 * @brief          停止ADC2循环DMA采样
 * @param[in]      none
 * @retval         HAL_StatusTypeDef HAL执行状态
 */
HAL_StatusTypeDef Inverter_ADC_Stop(void);

/**
 * @brief          处理ADC1六Rank DMA全传输完成事件
 * @param[in]      none
 * @retval         none
 *
 * @note           该函数由HAL_ADC_ConvCpltCallback()自动调用。
 */
void Inverter_ADC_ProcessFullTransfer(void);

/**
 * @brief          清除三相逆变采样结果和更新标志
 * @param[in]      none
 * @retval         none
 *
 * @note           不停止ADC2 DMA，也不解除ADC1 DMA缓冲区绑定。
 */
void Inverter_ADC_ClearData(void);

/**
 * @brief          初始化并启动三相逆变ADC采样
 * @param[in]      adc1_dma_buffer ADC1 DMA 缓冲区地址，
 *                                 与 PFC 采样共用同一个 ADC1 DMA 缓冲区
 * @retval         HAL_OK    初始化并启动成功
 * @retval         HAL_ERROR ADC或DMA启动失败
 * @note           必须在 MX_DMA_Init()、MX_ADC1_Init()、
 *                 MX_ADC2_Init() 和 MX_HRTIM1_Init() 执行完成后调用
 * @note           电流模拟看门狗须在四路零点校准完成后单独初始化。
 */
HAL_StatusTypeDef Inverter_App_Init(
    const volatile uint16_t *adc1_dma_buffer);

#endif /* INVERTER_ADC_H */

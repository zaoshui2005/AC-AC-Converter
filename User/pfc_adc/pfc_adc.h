#ifndef PFC_ADC_H
#define PFC_ADC_H

#include <stdint.h>
#include "adc.h"

/** ADC1规则组的Rank数量和循环DMA传输长度。 */
#define PFC_ADC_DMA_LENGTH       6U       /**< ADC1规则组的DMA采样通道数量。 */
#define PFC_ADC_IIN_INDEX        0U       /**< PFC输入电流所在Rank对应的DMA数组索引。 */
#define PFC_ADC_VIN_INDEX        1U       /**< PFC输入电压所在Rank对应的DMA数组索引。 */
#define PFC_ADC_VBUS_INDEX       2U       /**< PFC直流母线电压所在Rank对应的DMA数组索引。 */
#define PFC_ADC_VREF             3.3f     /**< ADC参考电压，单位为V。 */
#define PFC_ADC_FULL_SCALE       4095.0f  /**< 12位ADC满量程计数值。 */
#define PFC_ADC_VIN_SCALE        60.92f   /**< PFC输入电压采样模块的还原倍率。 */
#define PFC_ADC_IIN_GAIN_V_PER_A 0.08334f /**< PFC输入电流采样增益，单位为V/A。 */
#define PFC_ADC_VBUS_SCALE       29.7f   /**< 直流母线电压采样模块的校准还原倍率。 */
#define PFC_ADC_VIN_OFFSET_V     1.721f    /**< PFC输入电压采样通道的偏置电压，单位为V。 */
#define PFC_ADC_IIN_OFFSET_V     1.645f    /**< PFC输入电流采样通道的偏置电压，单位为V。 */
#define PFC_ADC_VBUS_OFFSET_V    0.0f     /**< PFC直流母线电压采样通道的偏置电压，单位为V。 */
#define PFC_ADC_SAMPLE_FREQ_HZ   20000.0f /**< HRTIM触发ADC1规则组的采样频率，单位为Hz。 */
#define PFC_ADC_VBUS_LPF_HZ      500.0f   /**< 直流母线一阶低通滤波截止频率，单位为Hz。 */
#define PFC_ADC_CALIB_DISCARD    2048U    /**< 零点校准前丢弃的ADC1半缓冲区样本数。 */
#define PFC_ADC_CALIB_SAMPLES    4096U    /**< 用于计算输入电压和输入电流零点的样本数。 */
#define PFC_ADC_TWO_PI           6.28318530718f /**< 计算数字滤波系数使用的2π常数。 */

/**
 * 输入50Hz、ADC采样20kHz时，一个交流周期对应400个采样点。
 */
#define PFC_ADC_VIN_RMS_WINDOW_SAMPLES 400U

/**
 * 输入电压RMS周期级低通滤波系数。
 * 越大响应越快，越小结果越平滑。
 */
#define PFC_ADC_VIN_RMS_FILTER_ALPHA 0.50f
/**
 * 100Hz二阶陷波器系数。
 * 适用于20kHz采样频率、100Hz中心频率和Q=5。
 */
#define PFC_ADC_VBUS_NOTCH_B0     0.99686876f  /**< 100Hz陷波器当前输入前向系数。 */
#define PFC_ADC_VBUS_NOTCH_B1    -1.99275373f  /**< 100Hz陷波器前一次输入前向系数。 */
#define PFC_ADC_VBUS_NOTCH_B2     0.99686876f  /**< 100Hz陷波器前两次输入前向系数。 */
#define PFC_ADC_VBUS_NOTCH_A1    -1.99275373f  /**< 100Hz陷波器前一次输出反馈系数。 */
#define PFC_ADC_VBUS_NOTCH_A2     0.99373752f  /**< 100Hz陷波器前两次输出反馈系数。 */

/** PFC输入电压采样通道的默认偏置ADC计数值。 */
#define PFC_ADC_VIN_OFFSET_COUNT \
    (PFC_ADC_VIN_OFFSET_V * PFC_ADC_FULL_SCALE / PFC_ADC_VREF)

/** PFC输入电流采样通道的默认偏置ADC计数值。 */
#define PFC_ADC_IIN_OFFSET_COUNT \
    (PFC_ADC_IIN_OFFSET_V * PFC_ADC_FULL_SCALE / PFC_ADC_VREF)

/** PFC直流母线电压采样通道的偏置ADC计数值。 */
#define PFC_ADC_VBUS_OFFSET_COUNT \
    (PFC_ADC_VBUS_OFFSET_V * PFC_ADC_FULL_SCALE / PFC_ADC_VREF)

/** PFC输入电压每个ADC计数对应的实际电压，单位为V/count。 */
#define PFC_ADC_VIN_V_PER_COUNT \
    (PFC_ADC_VREF * PFC_ADC_VIN_SCALE / PFC_ADC_FULL_SCALE)

/** PFC输入电流每个ADC计数对应的实际电流，单位为A/count。 */
#define PFC_ADC_IIN_A_PER_COUNT \
    (PFC_ADC_VREF / PFC_ADC_FULL_SCALE / PFC_ADC_IIN_GAIN_V_PER_A)

/** 直流母线电压每个ADC计数对应的实际电压，单位为V/count。 */
#define PFC_ADC_VBUS_V_PER_COUNT \
    (PFC_ADC_VREF * PFC_ADC_VBUS_SCALE / PFC_ADC_FULL_SCALE)

/** 直流母线一阶低通滤波系数。 */
#define PFC_ADC_VBUS_LPF_ALPHA \
    ((PFC_ADC_TWO_PI * PFC_ADC_VBUS_LPF_HZ) / \
     (PFC_ADC_SAMPLE_FREQ_HZ + PFC_ADC_TWO_PI * PFC_ADC_VBUS_LPF_HZ))

/**
 * @brief ADC1循环DMA缓冲区结构体
 */
typedef struct
{
    uint16_t sample[PFC_ADC_DMA_LENGTH]; /**< ADC1六个规则Rank的DMA原始采样值。 */
} PFC_ADC_DmaBufferTypeDef;

/**
 * @brief PFC三个采样通道的原始ADC快照结构体
 */
typedef struct
{
    uint16_t input_current; /**< 本次换算使用的PFC输入电流ADC快照。 */
    uint16_t input_voltage; /**< 本次换算使用的PFC输入电压ADC快照。 */
    uint16_t bus_voltage;   /**< 本次换算使用的PFC直流母线电压ADC快照。 */
} PFC_ADC_RawDataTypeDef;

/**
 * @brief PFC采样物理量和滤波结果
 */
typedef struct
{
    /** 输入交流瞬时电压，单位为V。 */
    float input_voltage_v;

    /** 输入交流瞬时电流，单位为A。 */
    float input_current_a;

    /** 最近一个完整周期直接计算出的输入电压有效值，单位为V RMS。 */
    float input_voltage_rms_unfiltered_v;

    /** 经过周期级平滑后的输入电压有效值，单位为V RMS。 */
    float input_voltage_rms_v;

    /** RMS有效标志：0表示尚未计算完成，1表示结果可用。 */
    uint8_t input_voltage_rms_ready;

    /** 未滤波母线电压，单位为V。 */
    float bus_voltage_unfiltered_v;

    /** 经过100Hz陷波后的母线电压，单位为V。 */
    float bus_voltage_notched_v;

    /** 最终用于控制的母线反馈电压，单位为V。 */
    float bus_voltage_v;
} PFC_ADC_MeasurementTypeDef;
/**
 * @brief PFC输入电压和输入电流零点校准结果结构体
 */
typedef struct
{
    float voltage_offset_count; /**< 输入电压通道自动校准得到的零点ADC计数值。 */
    float current_offset_count; /**< 输入电流通道自动校准得到的零点ADC计数值。 */
    uint8_t ready;              /**< ADC零点校准完成标志，1表示校准完成。 */
} PFC_ADC_CalibrationResultTypeDef;

/**
 * @brief PFC ADC对外运行数据结构体
 */
typedef struct
{
    PFC_ADC_DmaBufferTypeDef dma;              /**< ADC1六Rank循环DMA缓冲区。 */
    PFC_ADC_RawDataTypeDef raw;                /**< PFC三个采样通道的原始ADC快照。 */
    PFC_ADC_MeasurementTypeDef measurement;    /**< PFC采样通道的物理量换算结果。 */
    PFC_ADC_CalibrationResultTypeDef calibration; /**< 输入电压和输入电流零点校准结果。 */
} PFC_ADC_StateTypeDef;

/**
 * @brief 输入电压和输入电流自动零点校准过程结构体
 */
typedef struct
{
    uint32_t discard_count;     /**< 启动后已经丢弃的稳定等待样本数。 */
    uint32_t sample_count;      /**< 已经累加的有效零点校准样本数。 */
    uint32_t voltage_offset_sum; /**< 输入电压通道零点ADC计数累加值。 */
    uint32_t current_offset_sum; /**< 输入电流通道零点ADC计数累加值。 */
} PFC_ADC_CalibrationWorkTypeDef;

/**
 * @brief 输入交流电压周期RMS计算状态
 */
typedef struct
{
    /** 当前RMS窗口内输入电压平方和，单位为V²。 */
    float square_sum_v2;

    /** 当前窗口已经累计的采样点数。 */
    uint16_t sample_count;

    /** 平滑后的输入电压有效值，单位为V RMS。 */
    float filtered_rms_v;

    /** RMS滤波器初始化标志。 */
    uint8_t initialized;
} PFC_ADC_InputVoltageRmsTypeDef;

/**
 * @brief 二阶陷波器运行状态结构体
 */
typedef struct
{
    float x1;          /**< 前一次输入值。 */
    float x2;          /**< 前两次输入值。 */
    float y1;          /**< 前一次输出值。 */
    float y2;          /**< 前两次输出值。 */
    uint8_t initialized; /**< 陷波器初始化标志，1表示已经初始化。 */
} PFC_ADC_NotchStateTypeDef;

/**
 * @brief 一阶低通滤波器运行状态结构体
 */
typedef struct
{
    float output_v;      /**< 一阶低通滤波器当前输出值，单位为V。 */
    uint8_t initialized; /**< 低通滤波器初始化标志，1表示已经初始化。 */
} PFC_ADC_LowPassStateTypeDef;

/**
 * @brief PFC ADC模块内部运行状态
 */
typedef struct
{
    /** 输入电压和输入电流零点校准状态。 */
    PFC_ADC_CalibrationWorkTypeDef calibration;

    /** 输入交流电压RMS计算状态。 */
    PFC_ADC_InputVoltageRmsTypeDef input_voltage_rms;

    /** 母线100Hz陷波器状态。 */
    PFC_ADC_NotchStateTypeDef bus_voltage_notch;

    /** 母线低通滤波器状态。 */
    PFC_ADC_LowPassStateTypeDef bus_voltage_lpf;
} PFC_ADC_RuntimeTypeDef;

/** PFC ADC对外运行数据。 */
extern volatile PFC_ADC_StateTypeDef pfc_adc_state;

/**
* @brief          校准ADC1并启动六Rank循环DMA采样
* @param[in]      none
* @retval         HAL_StatusTypeDef HAL执行状态
*
* @note           CubeMX中ADC1规则组数量必须为6，前三个Rank必须按以下顺序配置：
*                 Rank1 PA3/ADC1_IN4=PFC输入电流；
*                 Rank2 PC2/ADC1_IN8=PFC输入电压；
*                 Rank3 PA1/ADC1_IN2=PFC母线电压；
*                 Rank4至Rank6由原工程其他ADC1采样通道使用。
*/
HAL_StatusTypeDef PFC_ADC_Start(void);

/**
* @brief          停止ADC1循环DMA采样
* @param[in]      none
* @retval         HAL_StatusTypeDef HAL执行状态
*/
HAL_StatusTypeDef PFC_ADC_Stop(void);

/**
* @brief          处理ADC1 DMA前三个Rank半传输完成事件
* @param[in]      none
* @retval         none
*/
void PFC_ADC_ProcessHalfTransfer(void);

/**
* @brief          初始化并启动ADC1六Rank DMA循环采样
* @param[in]      none
* @retval         HAL_OK 启动成功
* @retval         其他HAL状态 ADC校准或DMA启动失败
*/
HAL_StatusTypeDef PFC_ADC_SamplingStart(void);

#endif /* PFC_ADC_H */

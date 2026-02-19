#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include "driver/twai.h" // 确保包含 TWAI 定义


// ===========================================
// [System] 数据结构定义
// ===========================================
#define ENCODER_TOTAL_NUM 21
#define TACTILE_GROUP_NUM 5

// ------------------- [新增/修改部分 Start] -------------------
// [NEW] 用于存放 DIAAGC 寄存器的值 (包含 MAGH/MAGL 和 AGC 增益)
extern uint16_t g_enc_diag[ENCODER_TOTAL_NUM];
// ------------------- [新增/修改部分 End] -------------------



// ===========================================
// [AS5047P] 错误码定义 (新增)
// ===========================================
#define AS5047P_ERR_FRERR    0x0001  // 帧错误
#define AS5047P_ERR_INVCOMM  0x0002  // 无效命令
#define AS5047P_ERR_PARERR   0x0004  // 奇偶校验错误

#define ERR_CODE_NONE        0x0000  // 无错误
#define ERR_CODE_LINK_LOST   0xFFFF  // 物理断连
#define ERR_CODE_ERRFL_BASE  0x1000  // ERRFL错误基址
#define ERR_CODE_DIAG_BASE   0x2000  // 诊断错误基址

// ===========================================
// [System] 状态机定义 (新增)
// ===========================================
enum EncoderFSMState {
    FSM_READ_ANGLE = 0,
    FSM_READ_ERRFL = 1,
    FSM_READ_DIAAGC = 2
};
// ===========================================
// [System] 数据结构定义 (扩展)
// ===========================================
struct EncoderData {
    uint16_t rawAngles[ENCODER_TOTAL_NUM];
    uint16_t finalAngles[ENCODER_TOTAL_NUM];
    uint16_t errorFlags[ENCODER_TOTAL_NUM];      // 0=正常, 1=错误
    uint16_t latchedErrors[ENCODER_TOTAL_NUM];   // 详细错误码
};
struct CheckData {
    uint16_t rawData[ENCODER_TOTAL_NUM]; 
    uint16_t rawAngles[ENCODER_TOTAL_NUM];
    uint8_t  errorFlags[ENCODER_TOTAL_NUM];
    uint8_t  parityCheckFlags[ENCODER_TOTAL_NUM];
    uint16_t connectionStatus[ENCODER_TOTAL_NUM]; // 0: OK, 0xFFFF: Lost
};


// --- 触觉传感器数据结构 (根据你的描述修改) ---

// 1610 型号: 25点 * 3轴 + 3合力 = 75 + 3 = 78 Bytes
struct TacIndent1610 {
    uint8_t forces[25][3]; // 详细点阵力
    uint8_t global[3];     // 合力 Fx, Fy, Fz
};

// 2015 型号: 52点 * 3轴 + 3合力 = 156 + 3 = 159 Bytes
struct TacIndent2015 {
    uint8_t forces[52][3]; // 详细点阵力
    uint8_t global[3];     // 合力 Fx, Fy, Fz
};

// 单个模组包含的传感器
struct TacGroup {
    TacIndent2015 sensor_A; // 2015 #1
    TacIndent1610 sensor_B; // 1610 #1
    TacIndent1610 sensor_C; // 1610 #2
};

// 整个系统的触觉数据快照
struct TactileData {
    TacGroup groups[TACTILE_GROUP_NUM];
};

// --- 远程指令结构 ---
struct RemoteCommand {
    uint8_t cmd_type;
    float   value;
    bool    is_new;
};

// ===========================================
// [Hardware] 引脚定义 (ESP32-S3)
// ===========================================
// --- HSPI: 编码器 (AS5047P) ---
// 请根据您的电路图核对 GPIO
#define PIN_ENC_MISO    47
#define PIN_ENC_MOSI    38
#define PIN_ENC_SCLK    48
#define PIN_ENC_CS      7   // 公共片选

// MUX (74HC151) 用于切换 5 组传感器
#define PIN_MUX_A       2
#define PIN_MUX_B       4
#define PIN_MUX_C       5

// --- VSPI: 触觉传感器 ---
#define PIN_TAC_MISO    13
#define PIN_TAC_MOSI    11
#define PIN_TAC_SCLK    12
#define PIN_TAC_CS_A    7
#define PIN_TAC_CS_B    6
//#define PIN_TAC_CS_C    8  //引脚c被电阻一直上拉


// [Hardware] 引脚定义
// ===========================================
// TWAI (CAN)
#define PIN_TWAI_TX     9 
#define PIN_TWAI_RX     8
#define CAN_BAUD_RATE   TWAI_TIMING_CONFIG_1MBITS()

// MUX 控制引脚 (假设用于切换 5 组触觉)
#define PIN_MUX_A       2
#define PIN_MUX_B       4
#define PIN_MUX_C       5


// ===========================================
// [Parameter] 任务与通信参数
// ===========================================
#define SPI_FREQ_ENC    10000000 // 10MHz
#define SPI_FREQ_TAC    5000000  // 5MHz

// 任务优先级与核心
#define TASK_ENC_PRIO   10
#define TASK_ENC_CORE   0
#define TASK_TAC_PRIO   5
#define TASK_TAC_CORE   1

#endif // CONFIG_H
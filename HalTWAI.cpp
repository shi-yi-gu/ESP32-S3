
#include "hal/HalTWAI.h"
#include <cstring>

// 全局实例
HalTWAI twaiBus;

HalTWAI::HalTWAI() : _is_initialized(false) {}

bool HalTWAI::begin() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_TWAI_TX,
        (gpio_num_t)PIN_TWAI_RX,
        TWAI_MODE_NORMAL
    );
    g_config.tx_queue_len = 50;
    g_config.rx_queue_len = 10;

    twai_timing_config_t t_config = CAN_BAUD_RATE;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return false;
    }
    if (twai_start() != ESP_OK) {
        return false;
    }

    _is_initialized = true;
    return true;
}

void HalTWAI::sendFrame(uint32_t id, const uint8_t* data, uint8_t len) {
    if (!_is_initialized) return;

    twai_message_t message;
    message.identifier = id;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = len;
    memcpy(message.data, data, len);

    twai_transmit(&message, 0);
}

void HalTWAI::sendEncoderData(const EncoderData& encData) {
    uint8_t buffer[8];
    int idx = 0;

    // 1. 发送前 5 帧 (包含 20 个编码器数据)
    for (int frame = 0; frame < 5; frame++) {
        // 手动打包 4 个编码器数据 (确保大端序: High Byte First)
        for (int i = 0; i < 4; i++) {
            uint16_t val = encData.rawAngles[idx + i];
            
            // [修复] 手动拆分字节，不使用 memcpy
            buffer[i * 2]     = (uint8_t)(val >> 8);   // 高8位
            buffer[i * 2 + 1] = (uint8_t)(val & 0xFF); // 低8位
        }
        
        sendFrame(CAN_ID_ENC_BASE + frame, buffer, 8);
        idx += 4;
    }

    // 2. 发送最后一帧 (包含第 21 个编码器)
    memset(buffer, 0, 8);
    uint16_t lastVal = encData.rawAngles[idx];
    
    // [修复] 同样手动拆分
    buffer[0] = (uint8_t)(lastVal >> 8);
    buffer[1] = (uint8_t)(lastVal & 0xFF);
    
    sendFrame(CAN_ID_ENC_BASE + 5, buffer, 8);
}


void HalTWAI::sendTactileSummary(const TactileData& tacData) {
    uint8_t buffer[8];
    uint8_t bufPtr = 0;
    uint32_t frameId = 0x200;

    for (int g = 0; g < TACTILE_GROUP_NUM; g++) {
        const TacGroup& grp = tacData.groups[g];
        const uint8_t* ptrs[3] = {
            grp.sensor_A.global,
            grp.sensor_B.global,
            grp.sensor_C.global
        };

        for (int s = 0; s < 3; s++) {
            for (int k = 0; k < 3; k++) {
                buffer[bufPtr++] = ptrs[s][k];
                if (bufPtr == 8) {
                    sendFrame(frameId++, buffer, 8);
                    bufPtr = 0;
                    memset(buffer, 0, 8);
                }
            }
        }
    }
    if (bufPtr > 0) {
        sendFrame(frameId, buffer, bufPtr);
    }
}

void HalTWAI::sendTactileFullDump(const TactileData& tacData) {
    // 占位，按需实现
}

// 【新增 2023-10】发送校准确认帧
bool HalTWAI::sendCalibrationAck(bool success) {
    if (!_is_initialized) return false;

    twai_message_t message;
    message.identifier = 0x300; // 【定义】P4 监听的反馈 ID (可根据协议修改)
    message.extd = 0;           // 标准帧
    message.rtr = 0;
    message.data_length_code = 1; // 数据长度 1字节
    
    // Payload: 0x01 代表完成/成功，0x00 代表失败
    message.data[0] = success ? 0x01 : 0x00; 

    // 发送消息 (非阻塞模式，超时0ms)
    return (twai_transmit(&message, 0) == ESP_OK);
}

bool HalTWAI::receiveMonitor(RemoteCommand* outCmd) {
    if (!_is_initialized) return false;

    twai_message_t message;
    if (twai_receive(&message, 0) == ESP_OK) {
        if (outCmd) {
            outCmd->cmd_type = message.data[0];
            outCmd->is_new = true;
        }
        return true;
    }
    return false;
}

// 【新增】发送错误状态帧
// 格式: [ErrorMask(3B)] [FirstErrIdx] [ErrCode_H] [ErrCode_L] [Reserved(2B)]
void HalTWAI::sendErrorStatus(const EncoderData& data) {
    uint8_t buf[8] = {0};
    
    // Byte 0-2: 21位错误掩码 (每个bit代表一个编码器)
    uint32_t errMask = 0;
    int firstErrIdx = -1;
    uint16_t firstErrCode = 0;
    
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        if (data.errorFlags[i]) {
            errMask |= (1UL << i);
            if (firstErrIdx < 0) {
                firstErrIdx = i;
                firstErrCode = data.latchedErrors[i];
            }
        }
    }
    
    // 如果没有错误，不发送
    if (errMask == 0) return;
    
    buf[0] = errMask & 0xFF;
    buf[1] = (errMask >> 8) & 0xFF;
    buf[2] = (errMask >> 16) & 0x1F;
    buf[3] = (firstErrIdx >= 0) ? firstErrIdx : 0xFF;
    buf[4] = (firstErrCode >> 8) & 0xFF;
    buf[5] = firstErrCode & 0xFF;
    buf[6] = 0;  // Reserved
    buf[7] = 0;
    
    sendFrame(CAN_ID_ERROR_STATUS, buf, 8);
}

bool HalTWAI::maintain() {
    twai_status_info_t status_info;
    
    // 获取当前驱动状态
    if (twai_get_status_info(&status_info) != ESP_OK) {
        return false;
    }

    // 1. 处理 Bus-Off (总线挂死)
    if (status_info.state == TWAI_STATE_BUS_OFF) {
        // Serial.println("!!! [CAN ALERT] BUS-OFF Detected! Initiating Recovery... !!!");
        
        // 触发恢复流程 (这需要一定时间，取决于总线活动)
        // 注意：恢复期间不能发送/接收
        if (twai_initiate_recovery() != ESP_OK) {
            // Serial.println("!!! [CAN FAIL] Recovery Initiation Failed");
        }
        return false;
    }

    // 2. 处理 Stopped 状态 (恢复完成后，驱动处于停止状态，需要重新 Start)
    if (status_info.state == TWAI_STATE_STOPPED) {
        // Serial.println(">>> [CAN INFO] Bus Recovered. Restarting Driver...");
        
        if (twai_start() == ESP_OK) {
            // Serial.println(">>> [CAN INFO] Driver Restarted Successfully.");
            // 可以在这里重置某些错误计数
        } else {
            // Serial.println("!!! [CAN FAIL] Driver Start Failed");
        }
        return false; // 刚启动，本周期暂不处理数据
    }

    // 3. 检查接收队列溢出 (Rx Overrun)
    if (status_info.rx_overrun_count > 0) {
        // Serial.printf("[CAN WARN] Rx Buffer Overrun: %d\n", status_info.rx_overrun_count);
        // 如果溢出严重，可能需要清空接收队列
    }

    // 4. 检查错误计数 (可选报警)
    if (status_info.tx_error_counter > 100 || status_info.rx_error_counter > 100) {
        Serial.printf("[CAN WARN] High Error Count! Tx:%d Rx:%d\n", 
                      status_info.tx_error_counter, status_info.rx_error_counter);
    }

    return (status_info.state == TWAI_STATE_RUNNING);
}
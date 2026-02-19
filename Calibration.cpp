
#include "Calibration.h"

CalibrationManager calibManager;

CalibrationManager::CalibrationManager() {
    // 构造函数初始化，默认偏移量全为 0
    memset(offsets, 0, sizeof(offsets));
}

void CalibrationManager::begin() {
    _prefs.begin(_namespace, true); // true = 只读模式打开尝试读取
    size_t len = _prefs.getBytesLength(_key_offsets);
    
    if (len == sizeof(offsets)) {
        _prefs.getBytes(_key_offsets, offsets, sizeof(offsets));
        Serial.println("[Calib] Offsets loaded from NVS.");
    } else {
        Serial.println("[Calib] No valid offsets found. Using default 0.");
        memset(offsets, 0, sizeof(offsets));
    }
    _prefs.end();
}

void CalibrationManager::saveCurrentAsZero(const uint16_t* currentRawAngles) {
    if (currentRawAngles == nullptr) return;

    // 1. 更新内存副本
    memcpy(offsets, currentRawAngles, sizeof(offsets));

    // 2. 写入 Flash (切换为读写模式)
    _prefs.begin(_namespace, false); 
    size_t written = _prefs.putBytes(_key_offsets, offsets, sizeof(offsets));
    _prefs.end();

    if (written == sizeof(offsets)) {
        Serial.println("[Calib] New Zero points Saved successfully!");
    } else {
        Serial.println("[Calib] Error: Failed to save to Flash!");
    }
}

// =========================================================
// [修改重点] 批量转换函数
// =========================================================
void CalibrationManager::calibrateAll(const uint16_t* rawInput, uint16_t* calibratedOutput) {
    if (rawInput == nullptr || calibratedOutput == nullptr) return;

    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        // 1. 关键：过滤异常值，只保留有效14位数据
        uint16_t raw = rawInput[i] & ENCODER_MASK;
        uint16_t off = offsets[i] & ENCODER_MASK;

        // 2. 使用位运算实现防溢出环绕减法
        // 原理：利用无符号整数的二进制补码特性
        // diff 会自动处理 (raw < off) 的情况，比 if 语句更快
        uint16_t diff = raw - off;
        
        // 3. 再次掩码，确保结果合法
        calibratedOutput[i] = diff & ENCODER_MASK;
    }
}


void CalibrationManager::clearStorage() {
    _prefs.begin(_namespace, false);
    _prefs.clear();
    _prefs.end();
    memset(offsets, 0, sizeof(offsets));
    Serial.println("[Calib] Storage cleared!");
}
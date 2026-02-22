#ifndef HAL_ENCODERS_H
#define HAL_ENCODERS_H

#include <Arduino.h>
#include <SPI.h>
#include "driver/spi_master.h"
#include "../config/Config.h" 



class HalEncoders {
public:
    static HalEncoders& getInstance() {
        static HalEncoders instance;
        return instance;
    }

    void begin();
    void getData(EncoderData& outData);
    EncoderData getData();
    
    static const char* getErrorString(uint16_t errorCode);

private:
    HalEncoders();


    // ==========================================
    // 3. 寄存器地址
    // ==========================================
    static constexpr uint16_t AS5047P_REG_NOP         = 0x0000;    //空操作寄存器 (No Operation)。读取此寄存器相当于发送空指令，通常用于在不执行任何操作的情况下获取上一帧的数据。
    static constexpr uint16_t AS5047P_REG_ERRFL       = 0x0001;    //错误标志寄存器 (Error Flag)。存储通讯或硬件错误状态。读取后会自动清零。包含位：<br>- Bit 0: FRERR (帧错误)<br>- Bit 1: INVCOMM (无效命令)<br>- Bit 2: PARERR (奇偶校验错误
    static constexpr uint16_t AS5047P_REG_PROG        = 0x0003;    //编程寄存器 (Programming)。用于 OTP (一次性可编程存储器) 的编程操作。
    static constexpr uint16_t AS5047P_REG_DIAAGC      = 0x3FFC;    //诊断与自动增益控制 (Diagnostic and AGC)。<br>包含磁场强度状态标志（磁场过强/过弱）和当前 AGC 增益值，用于判断磁铁安装距离是否合适。
    static constexpr uint16_t AS5047P_REG_MAG         = 0x3FFD;    //CORDIC 幅值 (CORDIC Magnitude)。内部 CORDIC 算法计算出的磁场矢量幅值，反映当前检测到的磁场强度数据。
    static constexpr uint16_t AS5047P_REG_ANGLEUNC    = 0x3FFE;    //未补偿角度 (Angle Uncompensated)。<br>原始测量的 14-bit 角度值，不包含动态角度误差补偿 (DAEC™)。
    static constexpr uint16_t AS5047P_REG_ANGLECOM    = 0x3FFF;    //补偿后角度 (Angle Compensated)。<br>经过动态角度误差补偿 (DAEC™) 处理后的 14-bit 角度值。这是读取电机位置时最常用的寄存器。


    spi_device_handle_t _spi;
    const uint8_t group_sizes_[5] = {4, 4, 4, 4, 5}; 

    // 状态机变量 (新增)
    EncoderFSMState _fsmStates[ENCODER_TOTAL_NUM];
    uint16_t        _cmdPipeline[ENCODER_TOTAL_NUM];
    uint16_t        _latchedErrorCodes[ENCODER_TOTAL_NUM];
    uint8_t         _recoveryAttempts[ENCODER_TOTAL_NUM];
    
    static const uint8_t MAX_RECOVERY = 3;

    void setupSPI();
    void setMux(uint8_t channel);
    uint8_t get_parity(uint16_t n);
    uint16_t build_command_frame(uint16_t address, bool is_read);
};

extern HalEncoders& encoders;

#endif
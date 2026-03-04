#include "HalEncoders.h"
#include <cstring>

HalEncoders &encoders = HalEncoders::getInstance();

HalEncoders::HalEncoders()
{
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
    {
        _fsmStates[i] = FSM_READ_ANGLE;
        _cmdPipeline[i] = AS5047P_REG_ANGLECOM;
        _latchedErrorCodes[i] = 0;
        _recoveryAttempts[i] = 0;
    }
}

void HalEncoders::begin()
{
    pinMode(PIN_ENC_CS, OUTPUT);
    digitalWrite(PIN_ENC_CS, HIGH);
    pinMode(PIN_MUX_A, OUTPUT);
    pinMode(PIN_MUX_B, OUTPUT);
    pinMode(PIN_MUX_C, OUTPUT);
    setupSPI();
}

void HalEncoders::setupSPI()
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = 38,
        .miso_io_num = 47,
        .sclk_io_num = 48,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {
        .mode = 1,
        .clock_speed_hz = 2 * 1000 * 1000,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    spi_bus_add_device(SPI2_HOST, &devcfg, &_spi);
}

void HalEncoders::setMux(uint8_t ch)
{
    digitalWrite(PIN_MUX_A, (ch & 0x01));
    digitalWrite(PIN_MUX_B, (ch >> 1) & 0x01);
    digitalWrite(PIN_MUX_C, (ch >> 2) & 0x01);
    delayMicroseconds(5);
}

uint8_t HalEncoders::get_parity(uint16_t n)
{
    uint8_t p = 0;
    while (n)
    {
        p ^= 1;
        n &= (n - 1);
    }
    return p;
}

uint16_t HalEncoders::build_command_frame(uint16_t addr, bool is_read)
{
    if (is_read)
        addr |= 0x4000;
    if (get_parity(addr & 0x7FFF))
        addr |= 0x8000;
    return addr;
}

uint8_t HalEncoders::remapGroupIndex(uint8_t groupCount, uint8_t transferIndex) const
{
    // Base SPI order is reversed: transferIndex=0 maps to the last encoder in a group.
    uint8_t originalIndex = groupCount - 1 - transferIndex;

    if (groupCount == 4)
    {
        // Rule for 4 encoders: swap 3<->4, 
        // old->new (0-based): [0,1,3,2]
        static constexpr uint8_t kOldToNew[4] = {0, 1, 3, 2};
        return kOldToNew[originalIndex];
    }

    if (groupCount == 5)
    {
        // Rule for 5 encoders: swap 4<->5, 
        // old->new (0-based): [4,3,2,0,1]
        static constexpr uint8_t kOldToNew[5] = {0, 1, 2, 4, 3};
        return kOldToNew[originalIndex];
    }

    return originalIndex;
}

int HalEncoders::mapGlobalEncoderIndex(int groupStart, uint8_t groupCount, uint8_t transferIndex) const
{
    return groupStart + remapGroupIndex(groupCount, transferIndex);
}

const char *HalEncoders::getErrorString(uint16_t errorCode)
{
    if (errorCode == ERR_CODE_NONE)
        return "OK";
    if (errorCode == ERR_CODE_LINK_LOST)
        return "LINK_LOST";
    if (errorCode & ERR_CODE_DIAG_BASE)
    {
        if (errorCode & 0x0800)
            return "MAG_LOW";
        if (errorCode & 0x0400)
            return "MAG_HIGH";
        return "DIAG_ERR";
    }
    if (errorCode & ERR_CODE_ERRFL_BASE)
    {
        if (errorCode & 0x01)
            return "FRAME_ERR";
        if (errorCode & 0x02)
            return "INV_CMD";
        if (errorCode & 0x04)
            return "PAR_ERR";
    }
    return "UNKNOWN";
}

EncoderData HalEncoders::getData()
{
    EncoderData d;
    getData(d);
    return d;
}

void HalEncoders::getData(EncoderData &outData)
{
    spi_transaction_t t;
    uint16_t tx_buf[8], rx_buf[8], pending_cmds[8];
    int global_idx = 0;

    for (int grp = 0; grp < 5; grp++)
    {
        uint8_t count = group_sizes_[grp];

        // 1. 构建指令
        for (uint8_t k = 0; k < count; k++)
        {
            int id = mapGlobalEncoderIndex(global_idx, count, k);
            if (id >= ENCODER_TOTAL_NUM)
                id = ENCODER_TOTAL_NUM - 1;

            uint16_t reg;
            switch (_fsmStates[id])
            {
            case FSM_READ_ERRFL:
                reg = AS5047P_REG_ERRFL;
                break;
            case FSM_READ_DIAAGC:
                reg = AS5047P_REG_DIAAGC;
                break;
            default:
                reg = AS5047P_REG_ANGLECOM;
                break;
            }
            pending_cmds[k] = reg;
            uint16_t frame = build_command_frame(reg, true);
            tx_buf[k] = (frame >> 8) | (frame << 8);
        }

        // 2. SPI传输
        setMux(grp + 3);
        memset(&t, 0, sizeof(t));
        t.length = count * 16;
        t.tx_buffer = tx_buf;
        t.rx_buffer = rx_buf;

        digitalWrite(PIN_ENC_CS, LOW);
        delayMicroseconds(1);
        spi_device_polling_transmit(_spi, &t);
        digitalWrite(PIN_ENC_CS, HIGH);
        delayMicroseconds(2);

        // 3. 解析响应
        for (uint8_t k = 0; k < count; k++)
        {
            int id = mapGlobalEncoderIndex(global_idx, count, k);
            if (id >= ENCODER_TOTAL_NUM)
                continue;

            uint16_t raw_val = (rx_buf[k] << 8) | (rx_buf[k] >> 8);
            uint16_t prev_cmd = _cmdPipeline[id];
            _cmdPipeline[id] = pending_cmds[k];

            // 物理断连检测
            if (raw_val == 0x0000 || raw_val == 0xFFFF)
            {
                outData.errorFlags[id] = 1;
                _latchedErrorCodes[id] = ERR_CODE_LINK_LOST;
                _recoveryAttempts[id]++;
                if (_recoveryAttempts[id] >= MAX_RECOVERY)
                {
                    _fsmStates[id] = FSM_READ_ANGLE;
                }
                continue;
            }

            // 奇偶校验
            if (get_parity(raw_val & 0x7FFF) != ((raw_val >> 15) & 0x01))
            {
                continue;
            }

            bool error_bit = (raw_val & 0x4000);
            uint16_t payload = raw_val & 0x3FFF;

            // 状态机处理
            switch (prev_cmd)
            {
            case AS5047P_REG_ANGLECOM:
                if (error_bit)
                { 
                    outData.errorFlags[id] = 1;
                    _fsmStates[id] = FSM_READ_ERRFL;
                }
                else
                {
                    outData.rawAngles[id] = payload;
                    outData.errorFlags[id] = 0;
                    _recoveryAttempts[id] = 0;
                    if (_latchedErrorCodes[id] != ERR_CODE_LINK_LOST)
                    {
                        _latchedErrorCodes[id] = 0;
                    }
                    _fsmStates[id] = FSM_READ_ANGLE;
                }
                break;

            case AS5047P_REG_ERRFL:
                outData.errorFlags[id] = 1;
                if (payload & 0x07)
                {
                    _latchedErrorCodes[id] = ERR_CODE_ERRFL_BASE | (payload & 0x0F);
                }
                _fsmStates[id] = FSM_READ_DIAAGC;
                break;

            case AS5047P_REG_DIAAGC:
                outData.errorFlags[id] = 1;
                if (payload & 0x0F00)
                {
                    _latchedErrorCodes[id] = ERR_CODE_DIAG_BASE | (payload & 0x0FFF);
                }
                _recoveryAttempts[id]++;
                _fsmStates[id] = FSM_READ_ANGLE; // 自动恢复
                break;
            default:
                // 如果流水线里出现未知寄存器，强制回到读角度
                _fsmStates[id] = FSM_READ_ANGLE;
                break;
            }
        }
        global_idx += count;
    }

    memcpy(outData.latchedErrors, _latchedErrorCodes, sizeof(_latchedErrorCodes));
}

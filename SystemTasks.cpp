#include "SystemTasks.h"
#include "Config.h"
#include "HalEncoders.h"
#include "HalTactile.h"
#include "HalTWAI.h"
#include "esp_task_wdt.h" // 引入看门狗

// 任务句柄 (全局)
TaskHandle_t xEncTask = NULL;
TaskHandle_t xTacTask = NULL;
TaskHandle_t xCanTask = NULL;
TaskHandle_t xSysMgrTask = NULL;

// 队列
QueueHandle_t xQueueEncoderData = NULL;

// 测试计数器
static int g_testCounter = 0;

// 控制标志
volatile bool g_requestZeroCalibration = false;

/**
 * 编码器任务 (200Hz)
 * 负责读取编码器原始数据并发送到队列
 */
void Task_Encoders(void* pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // 本地数据缓冲
    EncoderData localData;

    Serial.println("[Task_Encoders] Started!");

    for (;;) {
        // 1. 读取编码器原始数据
        EncoderData rawData = encoders.getData();
        
        // 复制原始数据到本地结构
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
            localData.rawAngles[i] = rawData.rawAngles[i];
            localData.errorFlags[i] = rawData.errorFlags[i];
        }

        // 打印测试结果 (每100次循环打印一次)
        if ((g_testCounter % 100) == 1) {
            Serial.println(">>> Encoders (raw Angle: 0~16383)");
            for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
                // [ID:数据] 格式优化
                Serial.printf("[%02d:%05d] ", i, localData.rawAngles[i]);
                // 每 5 个换行
                if ((i + 1) % 5 == 0) {
                    Serial.println();
                }
            }
            if (ENCODER_TOTAL_NUM % 5 != 0) {
                Serial.println();
            }
        }

        // 2. 处理错误标记
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
            if (localData.errorFlags[i]) {
                localData.rawAngles[i] = 0xFFFF;  // 错误标记
            }
        }

        // 3. 发送到队列
        if (xQueueEncoderData != NULL) {
            xQueueOverwrite(xQueueEncoderData, &localData);
            
            // 调试打印 (每秒一次，避免刷屏)
            static uint32_t lastPrint = 0;
            if (millis() - lastPrint > 1000) {
                // Serial.println("[Task_Encoders] Queue filled OK.");
                lastPrint = millis();
            }
        } else {
            // 错误：队列未创建
            static uint32_t lastErr = 0;
            if (millis() - lastErr > 2000) {
                Serial.println("[FATAL] xQueueEncoderData is NULL!");
                lastErr = millis();
            }
        }
        
        g_testCounter++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * 触觉任务 (10Hz)
 * 负责处理触觉传感器数据
 */
void Task_Tactile(void* pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.println("[Task_Tactile] Started!");

    for (;;) {
        // tactile.update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * CAN 通信任务 (100Hz)
 * 负责发送编码器数据和接收远程命令
 */
void Task_CanBus(void *pvParameters) {
    (void)pvParameters;
    
    EncoderData txData;
    static uint8_t errorSendCounter = 0;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    Serial.println("[Task_CanBus] Started!");

    for (;;) {
        // 1. 维护总线状态 (自恢复、报警)
        bool isBusOk = twaiBus.maintain();

        if (isBusOk) {
            // 2. 从队列获取最新数据
            if (xQueueEncoderData && xQueueReceive(xQueueEncoderData, &txData, 0) == pdTRUE) {
                
                // 3. 发送角度数据
                twaiBus.sendEncoderData(txData);

                // 4. 每隔5帧检查并发送错误状态
                errorSendCounter++;
                if (errorSendCounter >= 5) {
                    errorSendCounter = 0;
                    
                    // 检查是否有错误
                    bool hasError = false;
                    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
                        if (txData.errorFlags[i]) {
                            hasError = true;
                            break;
                        }
                    }
                    
                    if (hasError) {
                        twaiBus.sendErrorStatus(txData);
                    }
                }
            }

            // 5. 接收处理
            twai_message_t rxMsg;
            if (twai_receive(&rxMsg, 0) == ESP_OK) {
                
                // 收到校准指令 (ID:0x200, Data:0xCA)
                if (rxMsg.identifier == 0x200 && rxMsg.data_length_code > 0 && rxMsg.data[0] == 0xCA) {
                    
                    // 发送通知给 SysMgr 任务
                    if (xSysMgrTask != NULL) {
                        xTaskNotifyGive(xSysMgrTask); 
                    }
                }
            }
        } else {
            // 总线故障中 - 降低频率，给驱动恢复时间
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * 系统管理任务
 * 负责处理低频、耗时、全局性的操作（如校准、保存配置等）
 */
void Task_SysMgr(void *pvParameters) {
    Serial.println("[SysMgr] Manager Task Started (Waiting for CMD)...");

    for (;;) {
        // 等待信号 (无限期阻塞，此时不消耗 CPU)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        Serial.println("[SysMgr] Calibration Sequence Started...");

        // 1. 先获取数据 (此时 xEncTask 还在运行，确保 SPI 锁能正常获取和释放)
        EncoderData currentData = encoders.getData(); 
        
        // 2. 数据拿到后，再挂起任务 (为了安全的操作)
        if (xEncTask) {
            vTaskSuspend(xEncTask);
        }
        if (xTacTask) {
            vTaskSuspend(xTacTask);
        }
        if (xCanTask) {
            vTaskSuspend(xCanTask);
        }
        
        // 3. 喂系统级看门狗
        esp_task_wdt_reset();

        // 4. 再次喂狗
        esp_task_wdt_reset();

        // 5. 恢复任务
        if (xCanTask) {
            vTaskResume(xCanTask);
        }
        if (xTacTask) {
            vTaskResume(xTacTask);
        }
        if (xEncTask) {
            vTaskResume(xEncTask);
        }
        
        Serial.println("[SysMgr] Calibration Done & Saved.");

        // 6. 发送 CAN 反馈
        vTaskDelay(pdMS_TO_TICKS(50)); // 给 CAN 任务一点恢复时间
        twaiBus.sendCalibrationAck(true);
    }
}

/**
 * 启动系统任务
 * 负责创建队列和所有任务
 */
void startSystemTasks() {
    // 1. 创建队列
    xQueueEncoderData = xQueueCreate(1, sizeof(EncoderData));
    if (xQueueEncoderData == NULL) {
        Serial.println("[FATAL] Queue creation failed!");
        while(1) {
            vTaskDelay(1000);
        }
    }
    
    Serial.println("[SystemTasks] Queue created successfully!");
    
    // 2. 创建编码器任务 - Core 1, 高优先级
    xTaskCreatePinnedToCore(
        Task_Encoders, "EncTask", 4096, NULL, 10, &xEncTask, 1
    );

    // 3. 创建触觉任务 - Core 0, 中优先级
    xTaskCreatePinnedToCore(
        Task_Tactile, "TacTask", 8192, NULL, 5, &xTacTask, 0
    );

    // 4. 创建 CAN 任务 - Core 0, 中优先级
    xTaskCreatePinnedToCore(
        Task_CanBus, "CanTask", 4096, NULL, 5, &xCanTask, 0
    );

    // 5. 创建系统管理任务 - Core 1, 低优先级
    xTaskCreatePinnedToCore(
        Task_SysMgr, "SysMgr", 6144, NULL, 1, &xSysMgrTask, 1 
    );
    
    Serial.println("[SystemTasks] All tasks created successfully!");
}
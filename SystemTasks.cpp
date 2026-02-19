#include "SystemTasks.h"
#include "Config.h"
#include "HalEncoders.h"
#include "HalTactile.h"
#include "HalTWAI.h"
#include "Calibration.h"
#include "esp_task_wdt.h" // 引入看门狗

// 任务句柄 (全局)
TaskHandle_t xEncTask = NULL;
TaskHandle_t xTacTask = NULL;
TaskHandle_t xCanTask = NULL;  // 【修复】添加 CAN 任务句柄
TaskHandle_t xSysMgrTask = NULL; // [新增] 管理任务句柄

// 全局实例
// HalEncoders& encoders = HalEncoders::getInstance();
CalibrationManager calibrationManager;

    static int times_test;

// 队列
QueueHandle_t xQueueEncoderData = NULL;

// 控制标志
volatile bool g_requestZeroCalibration = false;

// --- 编码器任务 (200Hz) ---
void Task_Encoders(void* pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // 本地数据缓冲 (使用 EncoderData，不是 EncoderDataPacket)
    EncoderData localData;
    // uint16_t calibrated[ENCODER_TOTAL_NUM];

    Serial.println("[Task_Encoders] Started!");

    for (;;) {
        // 1. 读取编码器原始数据
        EncoderData rawData = encoders.getData();
        // encoders.getData(localData);
        // 复制原始数据到本地结构
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
            localData.rawAngles[i] = rawData.rawAngles[i];
            localData.errorFlags[i] = rawData.errorFlags[i];
        }

        //打印测试结果
        if ((times_test%100)==1)
        {
    Serial.println(">>> Encoders (raw Angle: 0~16383)");
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        // [ID:数据] 格式优化
        Serial.printf("[%02d:%05d] ", i, localData.rawAngles[i]);
        // 每 5 个换行
        if ((i + 1) % 5 == 0) Serial.println();
    }
    if (ENCODER_TOTAL_NUM % 5 != 0) Serial.println();
        }

        // 4. 填充rawlAngles
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
            if (localData.errorFlags[i]) {
                localData.rawAngles[i] = 0xFFFF;  // 错误标记
            }
        }

        // 5. 发送到队列 【关键修复点】
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
        times_test++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


// --- 触觉任务 (50Hz) ---
void Task_Tactile(void* pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // tactile.update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// --- CAN 通信任务 (100Hz) ---
void Task_CanBus(void *pvParameters) {
    (void)pvParameters;
    
    // twaiBus.begin();
    
    EncoderData txData;
    RemoteCommand cmd{};
    static uint8_t errorSendCounter = 0;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    for (;;) {

        // 【新增】1. 维护总线状态 (自恢复、报警)
        bool isBusOk = twaiBus.maintain();

         if (isBusOk) {
        // Serial.println("\n======= [发送任务执行中] =======");
        // 1. 从队列获取最新数据
        if (xQueueEncoderData && xQueueReceive(xQueueEncoderData, &txData, 0) == pdTRUE) {
            
            // 2. 发送角度数据 (使用原有打包方式)
            twaiBus.sendEncoderData(txData);
            // Serial.println("\n======= [编码器数据发送完毕] =======");

            // 3. 【新增】每隔5帧检查并发送错误状态
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

        // 4. 接收处理
        twai_message_t rxMsg;
        if (twai_receive(&rxMsg, 0) == ESP_OK) {
            
            // 收到校准指令 (ID:0x200, Data:0xCA)
            if (rxMsg.identifier == 0x200 && rxMsg.data_length_code > 0 && rxMsg.data[0] == 0xCA) {
                
                // [关键修改] 不直接执行，而是发送通知给 SysMgr 任务
                if (xSysMgrTask != NULL) {
                    xTaskNotifyGive(xSysMgrTask); 
                }
            }
        }
        // while (twaiBus.receiveMonitor(&cmd)) {
        //     // 处理远程命令
        //     if (rxMsg.identifier == 0x200 && rxMsg.data_length_code >= 1) {
        //         if (rxMsg.data[0] == 0x01) {
        //             g_requestZeroCalibration = true;
        //         }
        //     }
        // }
        } else {
        // >>> 总线故障中 <<<
        // 可以选择在这里闪烁 LED 指示故障
            vTaskDelay(pdMS_TO_TICKS(50)); // 故障时降低频率，给驱动恢复时间
        }

        // Serial.println("\n======= [发送任务执行中] =======");
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
// --- [新增] 系统管理任务函数 ---
// 负责处理低频、耗时、全局性的操作（如校准、保存配置等）
// --- [修正后] 系统管理任务函数 ---
void Task_SysMgr(void *pvParameters) {
    // 【修改 1】删除/注释掉这行。
    // 不要监控一个会长期睡眠的任务，否则只要不发命令，看门狗就会重启系统。
    // esp_task_wdt_add(NULL); 

    Serial.println("[SysMgr] Manager Task Started (Waiting for CMD)...");

    for (;;) {
        // 等待信号 (无限期阻塞，此时不消耗 CPU)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        Serial.println("[SysMgr] Calibration Sequence Started...");

        // ---------------------------------------------------
        // [关键逻辑调整] 防止 SPI 死锁
        // ---------------------------------------------------
        
        // 1. 先获取数据 (此时 xEncTask 还在运行，确保 SPI 锁能正常获取和释放)
        // 假设 getData 内部有 SPI 操作，必须在任务运行态下调用
        EncoderData currentData = encoders.getData(); 
        
        // 2. 数据拿到后，再挂起任务 (为了安全的 Flash 写入)
        if (xEncTask) vTaskSuspend(xEncTask);
        if (xTacTask) vTaskSuspend(xTacTask);
        if (xCanTask) vTaskSuspend(xCanTask); 
        
        // 3. 喂系统级看门狗 (如果开启了 IDLE 监控，这一步其实主要是为了防止 Flash 操作过长)
        esp_task_wdt_reset();

        // 4. 执行 NVS 写入 (耗时操作)
        calibManager.saveCurrentAsZero(currentData.rawAngles);
        
        // 5. 再次喂狗
        esp_task_wdt_reset();

        // 6. 恢复任务
        if (xCanTask) vTaskResume(xCanTask);
        if (xTacTask) vTaskResume(xTacTask);
        if (xEncTask) vTaskResume(xEncTask);
        
        Serial.println("[SysMgr] Calibration Done & Saved.");

        // ---------------------------------------------------

        // 7. 发送 CAN 反馈
        vTaskDelay(pdMS_TO_TICKS(50)); // 给 CAN 任务一点恢复时间
        twaiBus.sendCalibrationAck(true);
    }
}

// 启动任务
void startSystemTasks() {

        // 【关键】先创建队列！
    xQueueEncoderData = xQueueCreate(1, sizeof(EncoderData));
    if (xQueueEncoderData == NULL) {
        Serial.println("[FATAL] Queue creation failed!");
        while(1) vTaskDelay(1000);
    }
    // 编码器任务 - Core 1, 高优先级
    xTaskCreatePinnedToCore(
        Task_Encoders, "EncTask", 4096, NULL, 10, &xEncTask, 1
    );

    // 触觉任务 - Core 0, 中优先级, 大栈
    xTaskCreatePinnedToCore(
        Task_Tactile, "TacTask", 8192, NULL, 5, &xTacTask, 0
    );

    // CAN 任务 - Core 1, 中优先级
    xTaskCreatePinnedToCore(
        Task_CanBus, "CanTask", 4096, NULL, 5, &xCanTask, 0
    );

    // [新增] 创建系统管理任务
    // 优先级设为 1 (低)，堆栈给多一点 (6144) 因为 NVS 操作耗栈
    xTaskCreatePinnedToCore(
        Task_SysMgr,   "SysMgr",   6144,  NULL,  1,  &xSysMgrTask, 1 
    );
}
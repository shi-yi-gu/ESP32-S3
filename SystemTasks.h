#ifndef SYSTEM_TASKS_H
#define SYSTEM_TASKS_H

#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 声明外部可访问的任务句柄
extern TaskHandle_t xEncTask;
extern TaskHandle_t xTacTask;
extern TaskHandle_t xCanTask;
extern TaskHandle_t xSysMgrTask;

// 任务函数声明
void Task_Encoders(void* pvParameters);
void Task_Tactile(void* pvParameters);
void Task_CanBus(void *pvParameters);
void Task_SysMgr(void *pvParameters);

// 启动系统任务
void startSystemTasks();

#endif
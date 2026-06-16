#include "ForceChassis.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "matrix.h"

float velocity[8] = {0};
TaskHandle_t task_handle;
WheelState state = WHEEL_IDEL;
void ChassisCalculateProcess(void *param)
{
    Chassis_t *chassis = (Chassis_t *)param;
    int wheel_num=chassis->wheel_num;

    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {
        // 安全检查部分:
        uint8_t safe_check=0;
        for(int i=0;i<wheel_num;i++)
        {
            state = chassis->wheel[i]->state_cb(chassis->wheel[i]);
            if(state==WHEEL_HEALTH)   //轮子正常
                safe_check=safe_check|(0x01<<i);
            else if(state==WHEEL_IDEL)  //如果是空闲模式，请求执行复位
                chassis->wheel[i]->reset_cb(chassis->wheel[i]);
            else if(state==WHEEL_ERROR) //轮子出现错误，执行底盘回调通知应用层
                chassis->wheel_err_cb(chassis,chassis->wheel[i]);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(chassis->update_dt_ms));
    }
}

#ifndef __FORCE_CHASSIS_H__
#define __FORCE_CHASSIS_H__

#include "arm_math.h"
#include "svd.h"
#include "Vector.h"

typedef enum{
    WHEEL_IDEL,         //轮子不会对指令做出响应
    WHEEL_HEALTH,       //轮子全部功能正常
    WHEEL_RESETING,     //轮子正在复位
    WHEEL_ERROR         //轮子异常，影响正常使用，一般尝试复位指令
}WheelState;

typedef struct Wheel_t Wheel_t;
typedef struct Chassis_t Chassis_t;

typedef void(*WheelReset_Cb_t)(Wheel_t *_this);                                      //调用该函数请求舵轮复位
typedef WheelState(*WheelState_Cb_t)(Wheel_t *_this);          //获取舵轮状态
typedef Vector2D(*GetWheelVelocity_Cb_t)(Wheel_t *_this); //获取轮子的速度方向
typedef void(*WheelError_Cb_t)(Chassis_t *_this,Wheel_t *wheel);           //轮子运行异常回调

struct Wheel_t{
    WheelReset_Cb_t reset_cb;           //请求轮子执行复位动作
    WheelState_Cb_t state_cb;           //获取轮子的状态
    GetWheelVelocity_Cb_t get_vel_cb;   //获取轮子速度回调

    //用户数据
    void* user_data;
};

struct Chassis_t{
    //硬件驱动部分
    Wheel_t *wheel[2];
    uint32_t wheel_num;
    WheelError_Cb_t wheel_err_cb;
    
    //控制周期
    uint32_t update_dt_ms;
};

void ChassisCalculateProcess(void *param);

#endif

#include "Task_Init.h"
#include "usart.h"
#include "semphr.h"
#include "usb_trans.h"
#include "motorEx.h"

SteeringWheel steeringWheelArray[2];
Wheel_t wheelArray[2];
Chassis_t chassis;

//句柄
TaskHandle_t Wheel_Handles[2];
TaskHandle_t Can_Send_Handle;
TaskHandle_t Uart_Handle;
extern TaskHandle_t task_handle;

//接收
uint8_t usart4_dma_buff[40];
Pack_TransRemote_t Pack_Trans;

//任务
void Can_Send(void *pvParameters);
void Wheel_Task(void *pvParameters);
void Uart_RXTask(void *pvParameters);

//信号量
extern SemaphoreHandle_t Chassis_semaphore;

Motor3508Ex_t Rm3508_Motor;

void Task_Init(void)
{
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));

    steeringWheelArray[0].Key_GPIO_Port = GPIOA;
    steeringWheelArray[0].Key_GPIO_Pin = GPIO_PIN_7;
    steeringWheelArray[1].Key_GPIO_Port = GPIOA;
    steeringWheelArray[1].Key_GPIO_Pin = GPIO_PIN_4;

    for(int i = 0; i < 2; i++)
    {
        wheelArray[i].user_data = &steeringWheelArray[i];
        wheelArray[i].reset_cb = WheelReset_Callback;
        wheelArray[i].state_cb = WheelState_Callback;
        wheelArray[i].get_vel_cb = GetWheelVelocity_Callback;
        chassis.wheel[i] = &wheelArray[i];
    }
		
    chassis.wheel_num = 2;
    chassis.update_dt_ms = 2;
    chassis.wheel_err_cb = WheelError_Callback;
		
		vTaskDelay(1000);
		xTaskCreate(ChassisCalculateProcess, "ChassisCalculateProcess", 300, &chassis, 5, &task_handle);
		xTaskCreate(Wheel_Task, "wheel_task1", 300, &wheelArray[0], 4, &Wheel_Handles[0]);
		xTaskCreate(Wheel_Task, "wheel_task2", 300, &wheelArray[1], 4, &Wheel_Handles[1]);
    
		xTaskCreate(Can_Send, "Can_Send", 300, NULL, 4, &Can_Send_Handle);

		xTaskCreate(Uart_RXTask, "Uart_RXTask", 300, NULL, 4, &Uart_Handle);
}

void Wheel_Task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    Wheel_t *wheel=(Wheel_t *)pvParameters;
    SteeringWheel *swheel = (SteeringWheel *)wheel->user_data;

    swheel->Steering_Vel_PID.Kp = 7.0f;
    swheel->Steering_Vel_PID.Ki = 0.007f;
    swheel->Steering_Vel_PID.Kd = 0.0f;
    swheel->Steering_Vel_PID.limit = 10000.0f;
    swheel->Steering_Vel_PID.output_limit = 9600.0f;

    swheel->Steering_Dir_PID.Kp = 130.0f;
    swheel->Steering_Dir_PID.Ki = 0.0f;
    swheel->Steering_Dir_PID.Kd = 3.0f;
    swheel->Steering_Dir_PID.limit = 10.0f;
    swheel->Steering_Dir_PID.output_limit = 10000.0f;

    swheel->Driver_Vel_PID.Kp = 0.17f;
    swheel->Driver_Vel_PID.Ki = 0.003f;
    swheel->Driver_Vel_PID.Kd = 1.7f;
    swheel->Driver_Vel_PID.limit = 50000.0f;
    swheel->Driver_Vel_PID.output_limit = 45.0f;

    swheel->offset = 0.0f;
    swheel->maxRotateAngle = 350.0f;
    swheel->floatRotateAngle = 340.0f;
    swheel->ready_edge_flag = 0;
	  swheel->expextForce = 0.0f;
		
    for(;;)
    {
			UpdateAngle(swheel);
			PID_Control2(swheel->currentDirection, swheel->putoutDirection, &swheel->Steering_Dir_PID);//角度环
			PID_Control2(swheel->SteeringMotor.Speed, swheel->Steering_Dir_PID.pid_out, &swheel->Steering_Vel_PID);//速度环
			
			PID_Control_d(swheel->DriveMotor.epm / 7.0f, (swheel->putoutVelocity / wheel_radius / (2.0f * PI) * 60.0f) * 3.3333334f, &swheel->Driver_Vel_PID);
			
			vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
    }
}

//can发送
int16_t motorCurrentBuf[4] = {0};
float driveCurrentBuf[2] = {0};
int16_t can2_3508_Tx_Data[4]={0};
float exp_scale_3508_t = 0.0f;
float exp_height_3508 = 0.0f;
void Can_Send(void *pvParameters)
{
		TickType_t last_wake_time = xTaskGetTickCount();
		
		Rm3508_Motor.hcan = &hcan2;
		Rm3508_Motor.ID = 0x202;
	
		steeringWheelArray[0].DriveMotor.hcan = &hcan2;
		steeringWheelArray[0].DriveMotor.motor_id = 0x02;
		steeringWheelArray[1].DriveMotor.hcan = &hcan2;
		steeringWheelArray[1].DriveMotor.motor_id = 0x03;
	
		Rm3508_Motor.pos_pid.Kp = 0.08f;
		Rm3508_Motor.pos_pid.Ki = 0.0f;
		Rm3508_Motor.pos_pid.Kd = 0.0f;
		Rm3508_Motor.pos_pid.limit = 10000.0f;
		Rm3508_Motor.pos_pid.output_limit = 9000.0f;
		
		Rm3508_Motor.vel_pid.Kp = 13.0f;
		Rm3508_Motor.vel_pid.Ki = 0.5f;
		Rm3508_Motor.vel_pid.Kd = 0.0f;
		Rm3508_Motor.vel_pid.limit = 10000.0f;
		Rm3508_Motor.vel_pid.output_limit = 16384.0f;
	
		for(;;)
		{
			steeringWheelArray[0].expectDirection = Pack_Trans.expectDirection[0];
			steeringWheelArray[1].expectDirection = Pack_Trans.expectDirection[1];
			steeringWheelArray[0].expextVelocity = - Pack_Trans.expextVelocity[0];
			steeringWheelArray[1].expextVelocity = - Pack_Trans.expextVelocity[1];
			
			motorCurrentBuf[1] = steeringWheelArray[0].Steering_Vel_PID.pid_out;
			motorCurrentBuf[2] = steeringWheelArray[1].Steering_Vel_PID.pid_out;

			MotorSend(&hcan1, 0x200, motorCurrentBuf);
			
			driveCurrentBuf[0] = steeringWheelArray[0].Driver_Vel_PID.pid_out;
			driveCurrentBuf[1] = steeringWheelArray[1].Driver_Vel_PID.pid_out;
      
			VESC_SetCurrent(&steeringWheelArray[0].DriveMotor, driveCurrentBuf[0]);
			VESC_SetCurrent(&steeringWheelArray[1].DriveMotor, driveCurrentBuf[1]);
			
			float exp_scale_3508 = exp_height_3508/(2*M_PI*Rm3508_r)*19*8192;
			
			PID_Control2(Rm3508_Motor.actual_pos, exp_scale_3508_t,&Rm3508_Motor.pos_pid);
			PID_Control2(Rm3508_Motor.motor.Speed, Rm3508_Motor.pos_pid.pid_out,&Rm3508_Motor.vel_pid);
			can2_3508_Tx_Data[1] = Rm3508_Motor.vel_pid.pid_out;
			
			MotorSend(&hcan2,0x200,can2_3508_Tx_Data);
			
			vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
		}
}

void Uart_RXTask(void *pvParameters)
{
    while (1)
    {
      if(xSemaphoreTake(Chassis_semaphore, pdMS_TO_TICKS(2000)) == pdTRUE)
      {
        memcpy(&Pack_Trans, usart4_dma_buff, sizeof(Pack_Trans));
      }
			else 
			{
				Pack_Trans.expectDirection[0] = 0.0f;
				Pack_Trans.expectDirection[1] = 0.0f;
				Pack_Trans.expextVelocity[0] = 0.0f;
				Pack_Trans.expextVelocity[1] = 0.0f;
			}
    }
}

//中断
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
  if (hcan->Instance == CAN1)
  {
    if (ID == 0x202)
    {
      M2006_Receive(&steeringWheelArray[0].SteeringMotor, Recv);
    }
    else if (ID == 0x203)
    {
      M2006_Receive(&steeringWheelArray[1].SteeringMotor, Recv);
    }
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
	VESC_ReceiveHandler(&steeringWheelArray[0].DriveMotor, hcan, ID, Recv);
	VESC_ReceiveHandler(&steeringWheelArray[1].DriveMotor, hcan, ID, Recv);
	Motor3508Recv(&Rm3508_Motor,hcan, ID, Recv);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART4)
	{
		HAL_UART_DMAStop(huart);
		// 重置HAL状态
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_READY;
		huart->gState = HAL_UART_STATE_READY;
		
		// 然后清除错误标志 - 按照STM32F4参考手册要求的顺序
		uint32_t isrflags = READ_REG(huart->Instance->SR);
		
		// 按顺序处理各种错误标志，必须先读SR再读DR来清除错误
		if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) 
		{
				// 对于ORE、NE、FE错误，需要先读SR再读DR
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
				volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); // 这个读取会清除ORE、NE、FE        
		}
		
		if (isrflags & USART_SR_PE)
		{
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
		}
		__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));
	}
}

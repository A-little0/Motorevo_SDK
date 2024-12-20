#ifndef __BSP_MOTOREVO_H
#define __BSP_MOTOREVO_H

#include "main.h"
#include "can.h"

#define MOTOREVO_POSITION_MAX       12.5f
#define MOTOREVO_POSITION_MIN      -12.5f
#define MOTOREVO_VELOCITY_MAX       10.0f
#define MOTOREVO_VELOCITY_MIN      -10.0f
#define MOTOREVO_TORQUE_MAX         50.0f
#define MOTOREVO_TORQUE_MIN        -50.0f
#define MOTOREVO_POSITION_KP_MAX    250.0f
#define MOTOREVO_POSITION_KP_MIN    0.0f
#define MOTOREVO_POSITION_KD_MAX    50.0f
#define MOTOREVO_POSITION_KD_MIN    0.0f
#define MOTOREVO_VELOCITY_KP_MAX    250.0f       
#define MOTOREVO_VELOCITY_KP_MIN    0.0f
#define MOTOREVO_VELOCITY_KI_MAX    0.05f
#define MOTOREVO_VELOCITY_KI_MIN    0.0f
#define MOTOREVO_VELOCITY_KD_MAX    50.0f
#define MOTOREVO_VELOCITY_KD_MIN    0.0f

#define MOTOREVO_CONFIG_CONTROL_MODE_INDEX 0x11
#define MOTOREVO_CONFIG_ID_CONTROLLER_KP_INDEX 0x12
#define MOTOREVO_CONFIG_ID_CONTROLLER_KI_INDEX 0x13
#define MOTOREVO_CONFIG_IQ_CONTROLLER_KP_INDEX 0x14
#define MOTOREVO_CONFIG_IQ_CONTROLLER_KI_INDEX 0x15
#define MOTOREVO_CONFIG_CURRENT_DEAD_ZONE_INDEX 0x16
#define MOTOREVO_CONFIG_VELOCITY_DEAD_ZONE_INDEX 0x17
#define MOTOREVO_CONFIG_POSITION_DEAD_ZONE_INDEX 0x18
#define MOTOREVO_CONFIG_CURRENT_INTEGRAL_LIMIT_INDEX 0x19
#define MOTOREVO_CONFIG_VELOCITY_INTEGRAL_LIMIT_INDEX 0x20
#define MOTOREVO_CONFIG_TORQUE_LIMIT_INDEX 0x21
#define MOTOREVO_CONFIG_ELECTRIC_ANGLE_OFFSET_INDEX 0x22
#define MOTOREVO_CONFIG_MACHINE_ANGLE_OFFSET_INDEX 0x23
#define MOTOREVO_CONFIG_CAN_COM_THETA_MIN_INDEX 0x24
#define MOTOREVO_CONFIG_CAN_COM_THETA_MAX_INDEX 0x25
#define MOTOREVO_CONFIG_CAN_COM_VELOCITY_MIN_INDEX 0x26
#define MOTOREVO_CONFIG_CAN_COM_VELOCITY_MAX_INDEX 0x27
#define MOTOREVO_CONFIG_CAN_COM_KP_MIN_INDEX 0x28
#define MOTOREVO_CONFIG_CAN_COM_KP_MAX_INDEX 0x29
#define MOTOREVO_CONFIG_CAN_COM_KD_MIN_INDEX 0x30
#define MOTOREVO_CONFIG_CAN_COM_KD_MAX_INDEX 0x31
#define MOTOREVO_CONFIG_CAN_COM_kI_MIN_INDEX 0x32
#define MOTOREVO_CONFIG_CAN_COM_KI_MAX_INDEX 0x33
#define MOTOREVO_CONFIG_CAN_COM_TORQUE_KP_MIN_INDEX 0x34
#define MOTOREVO_CONFIG_CAN_COM_TORQUE_KP_MAX_INDEX 0x35
#define MOTOREVO_CONFIG_MOTOR_ID_INDEX 0x36
#define MOTOREVO_CONFIG_CAN_COM_TIME_OUT_INDEX 0x37
#define MOTOREVO_CONFIG_DEFAULT_POSITION_KP_INDEX 0x38
#define MOTOREVO_CONFIG_DEFAULT_POSITION_KD_INDEX 0x39
#define MOTOREVO_CONFIG_DEFAULT_VELOCITY_KP_INDEX 0x40
#define MOTOREVO_CONFIG_DEFAULT_VELOCITY_KI_INDEX 0x41
#define MOTOREVO_CONFIG_FIRMWARE_VERSION_INDEX 0x10
#define MOTOREVO_CONFIG_ACCELERATION_INDEX 0x42
#define MOTOREVO_CONFIG_TORQUE_SLOPE_INDEX 0x43
#define MOTOREVO_CONFIG_NPP_INDEX 0x44
#define MOTOREVO_CONFIG_GERA_RATIO_INDEX 0x45
#define MOTOREVO_CONFIG_TORQUE_CONSTANT_INDEX 0x46
#define MOTOREVO_CONFIG_ROTATE_DIR_INDEX 0x47
#define MOTOREVO_CONFIG_BUS_OV_LOCK_INDEX 0x49
#define MOTOREVO_CONFIG_BUS_UV_LOCK_INDEX 0x50
#define MOTOREVO_CONFIG_PHASE_OC_LOCK_INDEX 0x51
#define MOTOREVO_CONFIG_PCB_OT_LOCK_INDEX 0x52
#define MOTOREVO_CONFIG_STUCK_CURRENT_LOCK_INDEX 0x51
#define MOTOREVO_CONFIG_STUCK_CURRENT_INDEX 0x52
#define MOTOREVO_CONFIG_STUCK_VELOCITY_INDEX 0x54
#define MOTOREVO_CONFIG_STUCK_TIME_INDEX 0x55
#define MOTOREVO_CONFIG_PROTECT_SITCHS_INDEX 0x56

typedef enum{
    Motorevo_Reset_Status,
    Motorevo_Motor_Status_Servo,
    Motorevo_Motor_Status_TPMix,
    Motorevo_Motor_Status_Velocity,
    Motorevo_Motor_Status_Torque,
    Motorevo_Motor_Status_torque4,
}Motorevo_Status;

typedef struct{
    uint8_t id;// 电机id

    struct{
        float position;// 位置, 单位rad
        float velocity;// 速度, 单位rad/s
        float torque;// 力矩，单位NM
    }baseData;

    struct{
        Motorevo_Status status;// 电机运行状态
        float p_kp;//位置kp
        float p_kd;//位置kd
        float v_kp;//速度kp
        float v_kd;//速度kd
        float v_ki;//速度ki
    }controlData;

    uint8_t temperature; // 温度，单位摄氏度
    uint8_t errorCode;   // 故障码
}Motorevo_DataHandleTypeDef;


/**
  * @name           Motorevo_motor_init
  * @brief          电机句柄初始化函数
  * @param[in]      device:电机句柄
  * @param[in]      id:初始化id
  * @param[in]      status:初始化状态
  * @retval         
  */
void Motorevo_motor_init(Motorevo_DataHandleTypeDef *device, uint8_t id, Motorevo_Status status);

/**
  * @name           Motorevo_motor_cantx_communication
  * @brief          can发送函数
  * @note           CAN 标准帧，帧ID为Motor ID, 波特率为1Mhz
  * @param[in]      id:设备id
  * @param[in]      data:can发送报文
  * @retval         
  */
void Motorevo_motor_cantx_communication(uint8_t id, uint8_t* data);

/**
  * @name           Motorevo_motor_canrx_communication
  * @brief          can发送函数
  * @note           CAN 标准帧，帧ID为Motor ID, 波特率为1Mhz
  * @param[in]      id:设备id
  * @param[in]      data:can发送报文
  * @retval         
  */
void Motorevo_motor_canrx_communication(Motorevo_DataHandleTypeDef *device, uint8_t* canrx_data);

/**
  * @name           Motorevo_enter_motor_state
  * @brief          设置电机为运行模式
  * @note           在torrque4 mode下帧id为100
  * @param[in]      id:设备id
  * @retval         none
  */
void Motorevo_enter_motor_state(uint8_t id);


/**
  * @name           Motorevo_enter_reset_state
  * @brief          设置电机为休眠模式
  * @note           在torrque4 mode下帧id为100
  * @param[in]      id:设备id
  * @retval         none
  */
void Motorevo_enter_reset_state(uint8_t id);



/**
  * @name           Motorevo_set_zero_position
  * @brief          设置电机零点
  * @note           在torrque4 mode下帧id为100
  * @param[in]      id:设备id
  * @retval         none
  */
void Motorevo_set_zero_position(uint8_t id);



/**
  * @name           Motorevo_motor_servo_control_mode
  * @brief          形参参数均需要通过CAN通信的方式设定，角度死区，力限保护参数需要在上位机设置
  * @param[in]      id:设备id
  * @param[in]      set_position:目标位置, rad, [-12.5 12.5]
  * @param[in]      velocity_limit:速度限幅, rad/s, [-10 10]
  * @param[in]      p_kp:位置环kp,8bits,[0.0 250.0]
  * @param[in]      p_kd:位置环kd,8bits,[0.0 50.0]
  * @param[in]      v_kp:速度环kp,8bits,[0.0 250.0]
  * @param[in]      v_ki:速度环ki,8bits,[0.0 250.0]
  * @param[in]      v_kd:速度环kd,8bits,[0.0 0.05]
  * @retval         none
  */
void Motorevo_motor_servo_control_mode(
  uint8_t id, 
  float set_position, 
  float velocity_limit, 
  float p_kp,
  float p_kd, 
  float v_kp, 
  float v_ki, 
  float v_kd
);


/**
  * @name           Motorevo_motor_torque_position_mix_control_mode
  * @brief          使用 CAN 控制电机在力位混合模式（P-T-M Mode）运行 
  * @note           形参参数均需要通过CAN通信的方式设定，力限保护参数需要在上位机设置
  * @param[in]      id:设备id
  * @param[in]      set_position:目标位置,12bits, rad, [-12.5 12.5]
  * @param[in]      set_velocity:目标速度,12bits, rad/s, [-10 10]
  * @param[in]      set_torque:目标力矩,12bits, rad/s, [-50 50]
  * @param[in]      p_kp:位置环kp,12bits,[0.0 250.0]
  * @param[in]      p_kd:位置环kd,8bits,[0.0 50.0]
  * @retval         none
  */
void Motorevo_motor_torque_position_mix_control_mode(
  uint8_t id, 
  float set_position, 
  float set_velocity, 
  float set_torque, 
  float p_kp,
  float p_kd
);



/**
  * @name           Motorevo_motor_torque_control_mode
  * @brief          使用CAN控制电机在力矩模式（Torque Mode）运行
  * @param[in]      id:设备id
  * @param[in]      set_torque:目标力矩,12bits, rad/s, [-50 50]
  * @retval         none
  */
void Motorevo_motor_torque_control_mode(uint8_t id, float set_torque);



/**
  * @name           Mototevo_motor_velocity_control_mode
  * @brief          使用 CAN 控制电机在速度模式（Velocity Mode）运行 
  * @param[in]      id:设备id
  * @param[in]      set_velocity:目标速度,12bits, rad/s, [-10 10]
  * @param[in]      v_kp:速度环kp,12bits,[0.0 250.0]
  * @param[in]      v_ki:速度环ki,12bits,[0.0 250.0]
  * @param[in]      v_kd:速度环kd,12bits,[0.0 0.05]
  * @retval         none
  */
void Mototevo_motor_velocity_control_mode(uint8_t id, float set_velocity, float v_kp, float v_ki, float v_kd);


/**
  * @name           Motorevo_motor_torque_control_mode4
  * @brief          使用CAN控制电机在力矩4模式（Torque4 Mode）运行  
  * @note           控制帧为CAN标准帧，帧ID为100，波特率为1Mhz
  *                 用一帧CAN指令对4个电机同时控制力矩
  * @param[in]      id:设备id
  * @param[in]      set_torque1:对于(Motor ID-1 % 4)=0的电机目标力矩,12bits, rad/s, [-50 50]
  * @param[in]      set_torque2:对于(Motor ID-1 % 4)=1的电机目标力矩,12bits, rad/s, [-50 50]
  * @param[in]      set_torque3:对于(Motor ID-1 % 4)=2的电机目标力矩,12bits, rad/s, [-50 50]
  * @param[in]      set_torque4:对于(Motor ID-1 % 4)=3的电机目标力矩,12bits, rad/s, [-50 50]
  * @retval         none
  */
void Motorevo_motor_torque_control_mode4(float set_torque1, float set_torque2, float set_torque3, float set_torque4);


void Motorevo_write_motor_config(uint8_t id, uint8_t index, uint8_t* data4);
void Motorevo_read_motor_config(uint8_t id, uint8_t index);
#endif
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
    uint8_t id;// ���id

    struct{
        float position;// λ��, ��λrad
        float velocity;// �ٶ�, ��λrad/s
        float torque;// ���أ���λNM
    }baseData;

    struct{
        Motorevo_Status status;// �������״̬
        float p_kp;//λ��kp
        float p_kd;//λ��kd
        float v_kp;//�ٶ�kp
        float v_kd;//�ٶ�kd
        float v_ki;//�ٶ�ki
    }controlData;

    uint8_t temperature; // �¶ȣ���λ���϶�
    uint8_t errorCode;   // ������
}Motorevo_DataHandleTypeDef;


/**
  * @name           Motorevo_motor_init
  * @brief          ��������ʼ������
  * @param[in]      device:������
  * @param[in]      id:��ʼ��id
  * @param[in]      status:��ʼ��״̬
  * @retval         
  */
void Motorevo_motor_init(Motorevo_DataHandleTypeDef *device, uint8_t id, Motorevo_Status status);

/**
  * @name           Motorevo_motor_cantx_communication
  * @brief          can���ͺ���
  * @note           CAN ��׼֡��֡IDΪMotor ID, ������Ϊ1Mhz
  * @param[in]      id:�豸id
  * @param[in]      data:can���ͱ���
  * @retval         
  */
void Motorevo_motor_cantx_communication(uint8_t id, uint8_t* data);

/**
  * @name           Motorevo_motor_canrx_communication
  * @brief          can���ͺ���
  * @note           CAN ��׼֡��֡IDΪMotor ID, ������Ϊ1Mhz
  * @param[in]      id:�豸id
  * @param[in]      data:can���ͱ���
  * @retval         
  */
void Motorevo_motor_canrx_communication(Motorevo_DataHandleTypeDef *device, uint8_t* canrx_data);

/**
  * @name           Motorevo_enter_motor_state
  * @brief          ���õ��Ϊ����ģʽ
  * @note           ��torrque4 mode��֡idΪ100
  * @param[in]      id:�豸id
  * @retval         none
  */
void Motorevo_enter_motor_state(uint8_t id);


/**
  * @name           Motorevo_enter_reset_state
  * @brief          ���õ��Ϊ����ģʽ
  * @note           ��torrque4 mode��֡idΪ100
  * @param[in]      id:�豸id
  * @retval         none
  */
void Motorevo_enter_reset_state(uint8_t id);



/**
  * @name           Motorevo_set_zero_position
  * @brief          ���õ�����
  * @note           ��torrque4 mode��֡idΪ100
  * @param[in]      id:�豸id
  * @retval         none
  */
void Motorevo_set_zero_position(uint8_t id);



/**
  * @name           Motorevo_motor_servo_control_mode
  * @brief          �ββ�������Ҫͨ��CANͨ�ŵķ�ʽ�趨���Ƕ����������ޱ���������Ҫ����λ������
  * @param[in]      id:�豸id
  * @param[in]      set_position:Ŀ��λ��, rad, [-12.5 12.5]
  * @param[in]      velocity_limit:�ٶ��޷�, rad/s, [-10 10]
  * @param[in]      p_kp:λ�û�kp,8bits,[0.0 250.0]
  * @param[in]      p_kd:λ�û�kd,8bits,[0.0 50.0]
  * @param[in]      v_kp:�ٶȻ�kp,8bits,[0.0 250.0]
  * @param[in]      v_ki:�ٶȻ�ki,8bits,[0.0 250.0]
  * @param[in]      v_kd:�ٶȻ�kd,8bits,[0.0 0.05]
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
  * @brief          ʹ�� CAN ���Ƶ������λ���ģʽ��P-T-M Mode������ 
  * @note           �ββ�������Ҫͨ��CANͨ�ŵķ�ʽ�趨�����ޱ���������Ҫ����λ������
  * @param[in]      id:�豸id
  * @param[in]      set_position:Ŀ��λ��,12bits, rad, [-12.5 12.5]
  * @param[in]      set_velocity:Ŀ���ٶ�,12bits, rad/s, [-10 10]
  * @param[in]      set_torque:Ŀ������,12bits, rad/s, [-50 50]
  * @param[in]      p_kp:λ�û�kp,12bits,[0.0 250.0]
  * @param[in]      p_kd:λ�û�kd,8bits,[0.0 50.0]
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
  * @brief          ʹ��CAN���Ƶ��������ģʽ��Torque Mode������
  * @param[in]      id:�豸id
  * @param[in]      set_torque:Ŀ������,12bits, rad/s, [-50 50]
  * @retval         none
  */
void Motorevo_motor_torque_control_mode(uint8_t id, float set_torque);



/**
  * @name           Mototevo_motor_velocity_control_mode
  * @brief          ʹ�� CAN ���Ƶ�����ٶ�ģʽ��Velocity Mode������ 
  * @param[in]      id:�豸id
  * @param[in]      set_velocity:Ŀ���ٶ�,12bits, rad/s, [-10 10]
  * @param[in]      v_kp:�ٶȻ�kp,12bits,[0.0 250.0]
  * @param[in]      v_ki:�ٶȻ�ki,12bits,[0.0 250.0]
  * @param[in]      v_kd:�ٶȻ�kd,12bits,[0.0 0.05]
  * @retval         none
  */
void Mototevo_motor_velocity_control_mode(uint8_t id, float set_velocity, float v_kp, float v_ki, float v_kd);


/**
  * @name           Motorevo_motor_torque_control_mode4
  * @brief          ʹ��CAN���Ƶ��������4ģʽ��Torque4 Mode������  
  * @note           ����֡ΪCAN��׼֡��֡IDΪ100��������Ϊ1Mhz
  *                 ��һ֡CANָ���4�����ͬʱ��������
  * @param[in]      id:�豸id
  * @param[in]      set_torque1:����(Motor ID-1 % 4)=0�ĵ��Ŀ������,12bits, rad/s, [-50 50]
  * @param[in]      set_torque2:����(Motor ID-1 % 4)=1�ĵ��Ŀ������,12bits, rad/s, [-50 50]
  * @param[in]      set_torque3:����(Motor ID-1 % 4)=2�ĵ��Ŀ������,12bits, rad/s, [-50 50]
  * @param[in]      set_torque4:����(Motor ID-1 % 4)=3�ĵ��Ŀ������,12bits, rad/s, [-50 50]
  * @retval         none
  */
void Motorevo_motor_torque_control_mode4(float set_torque1, float set_torque2, float set_torque3, float set_torque4);


void Motorevo_write_motor_config(uint8_t id, uint8_t index, uint8_t* data4);
void Motorevo_read_motor_config(uint8_t id, uint8_t index);
#endif
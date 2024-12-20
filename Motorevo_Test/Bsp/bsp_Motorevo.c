/**
  ****************************(C) COPYRIGHT 2024 ��;****************************
  * @file       bsp_Motorevo.c/h
  * @brief     
  *             ���������ֵ��can�����ļ�������canͨѶ�շ���λ��ģʽ
  *             �ٶ�ģʽ��������ģʽ����API���Լ����������ȡ��д��API��
  *             
  * @note      

  * 
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-6-2024      chenjiangnan    
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 ��;****************************
  */
#include "bsp_Motorevo.h"

/**
  * @name           float_to_uint
  * @brief          ����ת�޷���
  * @param[in]      x:�������
  * @param[in]      x_min:��Сֵ
  * @param[in]      x_max:���ֵ
  * @param[in]      bits:����
  * @retval         
  */
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  
   if(x > x_max) x = x_max;
   else if(x < x_min) x = x_min;
   
   return (int) ((x - offset)*((float)((1 << bits)-1))/span);
}


/**
  * @name           uint16_to_float
  * @brief          �޷���16λת����
  * @param[in]      x:�޷��ű���
  * @param[in]      x_min:��Сֵ
  * @param[in]      x_max:���ֵ
  * @param[in]      bits:����
  * @retval         
  */
static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;

    return offset * x / span + x_min;
}


/**
  * @name           Motorevo_motor_init
  * @brief          ��������ʼ������
  * @param[in]      device:������
  * @param[in]      id:��ʼ��id
  * @param[in]      status:��ʼ��״̬
  * @retval         
  */
void Motorevo_motor_init(Motorevo_DataHandleTypeDef *device, uint8_t id, Motorevo_Status status)
{
  device->id = id;
  device->controlData.status = status;
  device->controlData.p_kp = 0;
  device->controlData.p_kd = 0;
  device->controlData.v_kp = 0;
  device->controlData.v_ki = 0;
  device->controlData.v_kd = 0;

  device->baseData.position = 0;
  device->baseData.torque = 0;
  device->baseData.velocity = 0;

  device->errorCode = 0;
  device->temperature = 0;
}

/**
  * @name           Motorevo_motor_canrx_communication
  * @brief          can���ͺ���
  * @note           CAN ��׼֡��֡IDΪMotor ID, ������Ϊ1Mhz
  * @param[in]      id:�豸id
  * @param[in]      data:can���ͱ���
  * @retval         
  */
void Motorevo_motor_canrx_communication(Motorevo_DataHandleTypeDef *device, uint8_t* canrx_data)
{
  // switch(canrx_data[0])
  // {
  //   case Motorevo_Reset_Status:{
  //     device->controlData.status = Motorevo_Reset_Status;
  //   }break;
  //   case Motorevo_Motor_Status_Servo:{
  //     device->controlData.status = Motorevo_Motor_Status_Servo;
  //   }break;
  //   case Motorevo_Motor_Status_TPMix:{
  //     device->controlData.status = Motorevo_Motor_Status_TPMix;
  //   }break;
  //   case Motorevo_Motor_Status_Velocity:{
  //     device->controlData.status = Motorevo_Motor_Status_Velocity;
  //   }break;
  //   case Motorevo_Motor_Status_Torque:{
  //     device->controlData.status = Motorevo_Motor_Status_Torque;
  //   }break;
  //   case Motorevo_Motor_Status_torque4:{
  //     device->controlData.status = Motorevo_Motor_Status_torque4;
  //   }break;
  //   default:{

  //   }
  // }

  float t_position = (canrx_data[1] << 8) | canrx_data[2];
  float t_velocity = (canrx_data[3] << 4) | ((canrx_data[4] >> 4) & 0xF);
  float t_torque = ((canrx_data[4] & 0xF ) << 8) | canrx_data[5];
	
  device->baseData.position = uint16_to_float(t_position, MOTOREVO_POSITION_MIN, MOTOREVO_POSITION_MAX, 16);
  device->baseData.velocity = uint16_to_float(t_velocity, MOTOREVO_VELOCITY_MIN, MOTOREVO_VELOCITY_MAX, 12);
  device->baseData.torque = uint16_to_float(t_torque, MOTOREVO_TORQUE_MIN, MOTOREVO_TORQUE_MAX, 12);
	
  device->errorCode = canrx_data[6];
  device->temperature = canrx_data[7];
}


/**
  * @name           Motorevo_motor_cantx_communication
  * @brief          can���ͺ���
  * @note           CAN ��׼֡��֡IDΪMotor ID, ������Ϊ1Mhz
  * @param[in]      id:�豸id
  * @param[in]      data:can���ͱ���
  * @retval         
  */
void Motorevo_motor_cantx_communication(uint8_t id, uint8_t* data)
{
  CAN_TxHeaderTypeDef motorevo_tx_message;
  uint32_t send_mail_box;

  motorevo_tx_message.StdId = id;
  motorevo_tx_message.IDE = CAN_ID_STD;
  motorevo_tx_message.RTR = CAN_RTR_DATA;
  motorevo_tx_message.DLC = 0x08;

  HAL_CAN_AddTxMessage(&hcan1, &motorevo_tx_message, data, &send_mail_box);
}


/**
  * @name           Motorevo_enter_motor_state
  * @brief          ���õ��Ϊ����ģʽ
  * @note           ��torrque4 mode��֡idΪ100
  * @param[in]      id:�豸id
  * @retval         none
  */
void Motorevo_enter_motor_state(uint8_t id)
{
  uint8_t can_send_data[8];

  for(int i =0; i<7; i++)
  {
    can_send_data[i] = 0xFF;
  }
  can_send_data[7] = 0xFC;

  Motorevo_motor_cantx_communication(id, can_send_data);
}


/**
  * @name           Motorevo_enter_reset_state
  * @brief          ���õ��Ϊ����ģʽ
  * @note           ��torrque4 mode��֡idΪ100
  * @param[in]      id:�豸id
  * @retval         none
  */
void Motorevo_enter_reset_state(uint8_t id)
{
  uint8_t can_send_data[8];

  for(int i =0; i<7; i++)
  {
    can_send_data[i] = 0xFF;
  }
  can_send_data[7] = 0xFD;

  Motorevo_motor_cantx_communication(id, can_send_data);
}


/**
  * @name           Motorevo_set_zero_position
  * @brief          ���õ�����
  * @note           ��torrque4 mode��֡idΪ100
  * @param[in]      id:�豸id
  * @retval         none
  */
void Motorevo_set_zero_position(uint8_t id)
{
  uint8_t can_send_data[8];

  for(int i =0; i<7; i++)
  {
    can_send_data[i] = 0xFD;
  }
  can_send_data[7] = 0xFE;

  Motorevo_motor_cantx_communication(id, can_send_data);
}


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
)
{
  uint8_t can_send_data[8] = {0};
  
  uint16_t t_p = float_to_uint(set_position, MOTOREVO_POSITION_MIN, MOTOREVO_POSITION_MAX, 16);
  uint8_t t_v_limit = float_to_uint(velocity_limit, MOTOREVO_VELOCITY_MIN, MOTOREVO_VELOCITY_MAX, 8);
  uint8_t t_p_kp = float_to_uint(p_kp, MOTOREVO_POSITION_KP_MIN, MOTOREVO_POSITION_KP_MAX, 8);
  uint8_t t_p_kd = float_to_uint(p_kd, MOTOREVO_POSITION_KD_MIN, MOTOREVO_POSITION_KD_MAX, 8);
  uint8_t t_v_kp = float_to_uint(v_kp, MOTOREVO_VELOCITY_KP_MIN, MOTOREVO_VELOCITY_KP_MAX, 8);
  uint8_t t_v_ki = float_to_uint(v_ki, MOTOREVO_VELOCITY_KI_MIN, MOTOREVO_VELOCITY_KI_MAX, 8);
  uint8_t t_v_kd = float_to_uint(v_kd, MOTOREVO_VELOCITY_KD_MIN, MOTOREVO_VELOCITY_KD_MAX, 8);
 
  can_send_data[0] = t_p >> 8;
  can_send_data[1] = t_p & 0xFF;
  can_send_data[2] = t_v_limit;
  can_send_data[3] = t_p_kp;
  can_send_data[4] = t_p_kd;
  can_send_data[5] = t_v_kp;
  can_send_data[6] = t_v_kd;
  can_send_data[7] = t_v_ki;

  Motorevo_motor_cantx_communication(id, can_send_data);
}


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
)
{
  uint8_t can_send_data[8] = {0};
  
  uint16_t t_p = float_to_uint(set_position, MOTOREVO_POSITION_MIN, MOTOREVO_POSITION_MAX, 16);
  uint16_t t_v = float_to_uint(set_velocity, MOTOREVO_VELOCITY_MIN, MOTOREVO_VELOCITY_MAX, 12);
  uint16_t t_t = float_to_uint(set_torque, MOTOREVO_TORQUE_MIN, MOTOREVO_TORQUE_MAX, 12);
  uint16_t t_p_kp = float_to_uint(p_kp, MOTOREVO_POSITION_KP_MIN, MOTOREVO_POSITION_KP_MAX, 12);
  uint16_t t_p_kd = float_to_uint(p_kd, MOTOREVO_POSITION_KD_MIN, MOTOREVO_POSITION_KD_MAX, 12);
 
  can_send_data[0] = t_p >> 8;
  can_send_data[1] = t_p & 0xFF;
  can_send_data[2] = t_v >> 4;
  can_send_data[3] = ((t_v & 0xF) << 4) | ((t_p_kp >> 8) & 0xF);
  can_send_data[4] = t_p_kp & 0xFF; 
  can_send_data[5] = t_p_kd >> 4;
  can_send_data[6] = ((t_p_kd & 0xF) << 4) | ((t_t >> 8) & 0xF);
  can_send_data[7] = t_t & 0xFF;

  Motorevo_motor_cantx_communication(id, can_send_data);  
}


/**
  * @name           Motorevo_motor_torque_control_mode
  * @brief          ʹ��CAN���Ƶ��������ģʽ��Torque Mode������
  * @param[in]      id:�豸id
  * @param[in]      set_torque:Ŀ������,12bits, rad/s, [-50 50]
  * @retval         none
  */
void Motorevo_motor_torque_control_mode(uint8_t id, float set_torque)
{
  uint8_t can_send_data[8] = {0};

  uint16_t t_t = float_to_uint(set_torque, MOTOREVO_TORQUE_MIN, MOTOREVO_TORQUE_MAX, 16);

  can_send_data[0] = t_t >> 8;
  can_send_data[1] = t_t & 0xFF;
  for(int i = 2; i < 7; i++)
  {
    can_send_data[i] = 0;
  }
  can_send_data[7] = 0xAB;

  Motorevo_motor_cantx_communication(id, can_send_data);
}


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
void Mototevo_motor_velocity_control_mode(uint8_t id, float set_velocity, float v_kp, float v_ki, float v_kd)
{
  uint8_t can_send_data[8] = {0};

  uint16_t t_v = float_to_uint(set_velocity, MOTOREVO_VELOCITY_MIN, MOTOREVO_VELOCITY_MAX, 16);
  uint16_t t_v_kp = float_to_uint(v_kp, MOTOREVO_VELOCITY_KP_MIN, MOTOREVO_VELOCITY_KP_MAX, 12);
  uint16_t t_v_ki = float_to_uint(v_ki, MOTOREVO_VELOCITY_KI_MIN, MOTOREVO_VELOCITY_KI_MAX, 12); 
  uint16_t t_v_kd = float_to_uint(v_kd, MOTOREVO_VELOCITY_KD_MIN, MOTOREVO_VELOCITY_KD_MAX, 12); 

  can_send_data[0] = t_v >> 8;
  can_send_data[1] = t_v & 0xFF;
  can_send_data[2] = t_v_kp >> 4;
  can_send_data[3] = ((t_v_kp & 0xF) << 4) | ((t_v_kd >> 8) & 0xF);
  can_send_data[4] = t_v_kd & 0xFF;
  can_send_data[5] = t_v_ki >> 4;
  can_send_data[6] = (t_v_ki & 0xF) << 4;
  can_send_data[7] = 0xAC;

  Motorevo_motor_cantx_communication(id, can_send_data);
}


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
void Motorevo_motor_torque_control_mode4(float set_torque1, float set_torque2, float set_torque3, float set_torque4)
{
  uint8_t can_send_data[8] = {0};

  uint16_t t_t1 = float_to_uint(set_torque1, MOTOREVO_TORQUE_MIN, MOTOREVO_TORQUE_MAX, 16);
  uint16_t t_t2 = float_to_uint(set_torque2, MOTOREVO_TORQUE_MIN, MOTOREVO_TORQUE_MAX, 16);
  uint16_t t_t3 = float_to_uint(set_torque3, MOTOREVO_TORQUE_MIN, MOTOREVO_TORQUE_MAX, 16);
  uint16_t t_t4 = float_to_uint(set_torque4, MOTOREVO_TORQUE_MIN, MOTOREVO_TORQUE_MAX, 16);

  can_send_data[0] = t_t1 >> 8;
  can_send_data[1] = t_t1 & 0xFF;
  can_send_data[2] = t_t2 >> 8;
  can_send_data[3] = t_t2 & 0xFF;
  can_send_data[4] = t_t3 >> 8;
  can_send_data[5] = t_t3 & 0xFF;
  can_send_data[6] = t_t4 >> 8;
  can_send_data[7] = t_t4 & 0xFF;

  Motorevo_motor_cantx_communication(100, can_send_data);
}


/**
  * @name           Motorevo_write_motor_config
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
void Motorevo_write_motor_config(uint8_t id, uint8_t index, uint8_t* data4)
{
  uint8_t can_send_data[8] = {0};

  can_send_data[0] = 0x67;
  can_send_data[1] = index;
  can_send_data[2] = data4[0];
  can_send_data[3] = data4[1];
  can_send_data[4] = data4[2];
  can_send_data[5] = data4[3];
  can_send_data[6] = 0x15;
  can_send_data[7] = 0x76;

}
void Motorevo_read_motor_config(uint8_t id, uint8_t index)
{
  uint8_t can_send_data[8] = {0};

  can_send_data[0] = 0x67;
  can_send_data[1] = index;
  can_send_data[6] = 0x04;
  can_send_data[7] = 0x76;
}
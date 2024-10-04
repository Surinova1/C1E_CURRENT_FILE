/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "math.h"
#include <stdint.h>
#include <stdlib.h>
#include "EEPROM.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	#define					SET									1
	#define					NULL								0
	#define					BUZZER_ON						HAL_GPIO_WritePin(Buzzer_1_GPIO_Port,Buzzer_1_Pin, GPIO_PIN_RESET);			HAL_GPIO_WritePin(Buzzer_2_GPIO_Port,Buzzer_2_Pin, GPIO_PIN_RESET);	
	#define					BUZZER_OFF					HAL_GPIO_WritePin(Buzzer_1_GPIO_Port,Buzzer_1_Pin, GPIO_PIN_SET);			HAL_GPIO_WritePin(Buzzer_2_GPIO_Port,Buzzer_2_Pin, GPIO_PIN_SET);	
	#define					BUZZER_TOGGLE				HAL_GPIO_TogglePin(Buzzer_1_GPIO_Port,Buzzer_1_Pin );					HAL_GPIO_TogglePin(Buzzer_2_GPIO_Port,Buzzer_2_Pin);
	#define					STOP_WHEELS					for(uint8_t i = 1; i <= 4; i++){Set_Motor_Torque(i, 0);}
	
	#define					LSB												0
	#define					MSB	 											1
	#define					REMOTE										1
	#define					DATA											2
	#define					STEERING_BOUNDARY 				2
	#define					STEERING_HOMING_SPEED	 	15
	#define					STEERING_KP			 					5
	#define					STEERING_MAX_VEL					50
	
	#define					VELOCITY					0
	#define					TORQUE						1
	#define					VEL_LIMIT					2
	#define					MOT_ERROR					3
	#define					ENC_ERROR					4
	#define					IQ								5
	#define					ENC_EST						6
	#define					T_RAMP						7
	#define					REQ_STATE					8
	#define					SNL_ERROR					9
	#define					SENSL_EST					10
	#define					POSITION					11
	
	#define					IMU_L							0x08
	#define					IMU_R							0x09
	#define					LF_STEER					0x10
	#define					LR_STEER					0x14
	#define					RF_STEER					0x12
	#define					RR_STEER					0x13
	#define					L_ARM							0x11
	#define					R_ARM							0x15
	#define					P_ARM							0x16
	#define					LT_SENS						0x17
	#define					L_VERT						0x21
	#define					R_VERT						0x22
	#define					C_LMT							0x07
	#define					FL_FLAP						0x17
	#define					FR_FLAP						0x18
	#define					RL_FLAP						0x19
	#define					RR_FLAP						0x20
	#define					CMD_MASK					0x01F	
	
	#define					HEARTBEAT					0x01
	#define					POS_ID						0x0C
	#define					VEL_ID						0x0D
	#define					TRQ_ID						0x0E
	#define					VLMT_ID						0x0F
	#define					MERR_ID						0x03
	#define					ENERR_ID					0x04
	#define					SNERR_ID					0x05
	#define					IQM_ID						0x014
	#define					ENEST_ID					0x009
	#define					T_RAMP_ID					0x1C
	#define					REQ_STATE_ID			0x07
	#define					SENS_EST					0x015
	#define					VOLTAGE					  0x017
	
	#define					ALL_WHEEL					1
	#define					CRAB							3
	#define					ZERO_TURN					2
	#define					WIDTH_SHRINK			4
	#define					WIDTH_EXTEND			5
	#define					FRONT_WHEEL				6
	#define					WIDTH_SPEED				10
	#define					WIDE_ANGLE				5
	#define					SHRINK_ANGLE			5
	#define					Anti_Windup_Limit		2
	#define					V_LIMIT							70
	#define					C_LIMIT							70
	#define					ARM_HOMING_SPEED	 	15
	
	#define					BT_READ						HAL_GPIO_ReadPin(UART5_State_GPIO_Port,UART5_State_Pin);
	#define 				EEPROM_ADDRESS 	0xA0 // Adjust according to the EEPROM's address
	#define 				PAGE_SIZE 				64        // EEPROM page size

	#define ARRAY_SIZE 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

osThreadId DefaultTasksHandle;
osThreadId HP_TasksHandle;
osThreadId LP_TasksHandle;
/* USER CODE BEGIN PV */
/* 							BT_VARIABLES 						*/
uint8_t BT_Rx[8], BT_Count=0, RxBuff[8];
bool BT_State=0 ,  Prev_BT_State=0;
/* 							BT_VARIABLES 						*/


/* 							CAN_VARIABLES 						*/

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader2;
CAN_RxHeaderTypeDef RxHeader2;
uint8_t TxData[8];
uint8_t RxData[8];
uint8_t TxData2[8];
uint8_t RxData2[8];
uint8_t RxData2_Temp[8];
uint8_t RxData_Temp[8];
uint32_t TxMailbox, CAN_Count=0;
uint8_t Node_Id[30],PREV_Node_Id[30], Received_Node_Id=0, Received_Command_Id=0;
uint8_t Sensor_Id[10], Axis_State[30];
float Motor_Velocity[20], Rover_Voltage=0,Motor_Current[20], Rover_Voltage_Temp=0;uint8_t Motor_Error[20], Encoder_Error[20] , Volt_Tx=0, Volt_Tx_Temp=0;
uint8_t LFD=1,LRD=2,RFD=3,RRD=4,LVert=5, RVert=6, Contour=7, LFS=8, LRS=9, RFS=10, RRS=11, L_Arm=12, R_Arm=13, P_Arm=14 , Upper_Width =16 , Lower_Width = 15, Cutter=17, Side_Belt = 18, Selective = 19, Paddle =20;

/* 							CAN_VARIABLES 						*/

/* 							IMU_VARIABLES 						*/

float L_Roll=0, L_Pitch=0, R_Roll=0, R_Pitch=0;
float Left_Roll_Pos = 1.5, Right_Roll_Pos = 3.5, Right_Pitch_Pos = 8.1, Left_Pitch_Pos=15.5, Left_Column_Error =0 , Left_Col_Pos = 0;
bool Left_IMU_State=1, Initiate_Process=0;

/* 							IMU_VARIABLES 						*/
/* 							STEERING_VARIABLES 						*/
uint16_t Track_Width = 1730, Min_Track_Width = 1730, Zero_Turn_Angle = 33, Wheel_Base = 900;
uint16_t Steer_Angle[5];
float LF_Steering=0, LR_Steering=0, RF_Steering=0, RR_Steering=0;	
float LF_HomePos =9, LR_HomePos= 680 , RF_HomePos= 451 , RR_HomePos = 619;	// -->	HOME POSITIONS LF_HomePos = 190, LR_HomePos= 87 , RF_HomePos= 220 , RR_HomePos = 623;
float LF_Speed=0, LR_Speed=0, RF_Speed=0, RR_Speed=0 , LF_Speed_Temp =0, LR_Speed_Temp =0 , RF_Speed_Temp=0, RR_Speed_Temp=0, LF_Error=0, LR_Error=0, RF_Error=0, RR_Error=0;		
_Bool Steering_Reset_Flag = SET , LF_SET = NULL , LR_SET = NULL, RF_SET = NULL, RR_SET = NULL , BUZZ_SW = SET;
float Inner_Angle =0 , Outer_Angle=0, Prev_Inner_Angle =0 , AW_Angle=0 , Outer_Angle_2=0 , LS_Angle=0, RS_Angle=0;
bool Angle_Ready = 0;
/* 							STEERING_VARIABLES 						*/
/* 							SENSING_VARIABLES 						*/

float FL_Raw =0, FR_Raw = 0, RL_Raw = 0, RR_Raw = 0;
float FL_Angle=0, FR_Angle=0, RL_Angle =0, RR_Angle=0, FL_Angle_Temp=0;
uint16_t FL_Home_Pos = 563 , FR_Home_Pos = 0, RL_Home_Pos = 0, RR_Home_Pos = 0;
int16_t Left_Arm_Motor_Count=0, Right_Arm_Motor_Count=0, Right_Arm_Motor_Value=0, Left_Arm_Motor_Value=0, Pitch_Arm_Motor_Count=0, Pitch_Arm_Motor_Value=0;
float L_Arm_Speed=0, R_Arm_Speed=0, L_Arm_Speed_Temp=0, R_Arm_Speed_Temp=0, Pitch_Arm_Speed_Temp=0, Tri_Arm_Speed=0;double Pitch_Arm_Speed=0;
_Bool Front_Left_Bush = 0, Front_Right_Bush = 0, Front_Bushes_Sensed = 0, First_Sense=0 , Rear_Bush=0;
int Flaps_Target = 36, Flap_Error=0,Flap_Error_Right = 0,Flaps_Target_Right=36;
float Flap_Kp = 2, Pitch_Kp=2 ;
/* 							SENSING_VARIABLES 						*/


/* 							JOYSTICK_VARIABLES 						*/
uint8_t Mode=1,Mode_Temp, Speed=1, Joystick=0, Steering_Mode=1, Shearing=0, Skiffing=0, Side_Trimmer=0, Pot_Angle=90, Joystick_Temp=0, Shearing_Temp=0, Steering_Mode_Temp=1, BT_Steer_Temp=0;		 
/* 							JOYSTICK_VARIABLES 						*/


/* 							DRIVE_WHEELS_VARIABLES 						*/
bool DRIVES_ERROR_FLAG = NULL;
float L_R_Err=0, R_R_Err=0, C_Err=0, Contour_Avg=0, Drive_Torque=1, Wheel_Torque = 10;
float Vel_Limit=10, Vel_Limit_Temp=1, Torque=0, Torque_Temp=0 , Prev_Torque=0, Prev_Vel_Limit=30;
int Left_Wheels_Torque =0, Left_Wheels_Torque_Temp=0;

/* 							DRIVE_WHEELS_VARIABLES 						*/

float Absolute_Position[20];
int16_t Absolute_Position_Int[20];

/* 							FRAME_CONTROLS_VARIABLES 						*/
float L_Vert_Speed=0, R_Vert_Speed=0, L_Vert_Speed_Temp=0, R_Vert_Speed_Temp=0, Contour_Speed=0, Contour_Speed_Temp=0;
bool Left_Error_Flag=NULL , Right_Error_Flag=NULL , Contour_Error_Flag=NULL, FRAME_NO_ERROR_FLAG=SET,Contour_Limit=SET,Vertical_Limit=SET; 
float  R_Error_Change=0, R_Error_Slope=0, R_Error_Area=0, R_Prev_Error=0;
float R_Kp=7, R_Ki=0, R_Kd=5; 
long R_P=0, R_I=0, R_D=0;
float Error=0, L_Prev_Error=0, L_Error_Change=0, L_Error_Slope=0, L_Error_Area=0, Left_Out=0, Right_Out=0, Contour_Out=0;
float  C_Error_Change=0, C_Error_Slope=0, C_Error_Area=0, C_Prev_Error=0;
float C_Kp=9, C_Ki=2, C_Kd=5;         //20 5  5500    //20  0.05  17000
long C_P=0, C_I=0, C_D=0;
double dt=0.01 ;
int Left_Vertical_Error=0;
int Current_Vel_Limit = 0, Modified_Vel_Limit = 0, Modified_Vel_Limit_Temp =0 , Prev_Mod=0; 
float Width_Motor_Speed=0, Width_Motor_Temp=0, Lower_Width_Motor_Speed = 0, Upper_Width_Motor_Speed = 0, Lower_Width_Motor_Speed_Temp=0, Upper_Width_Motor_Speed_Temp=0;
int Lower_Width_Motor_Count=0, Upper_Width_Motor_Count=0, Lower_Width_Motor_Value=0, Upper_Width_Motor_Value=0; // Total Counts
int Right_Vertical_Motor_Count =0, Contour_Motor_Count =0, Right_Vertical_Motor_Value =0, Contour_Motor_Value =0;;
bool Right_Vertical_On_Limit = 0, Contour_On_Limit=0;
/* 							FRAME_CONTROLS_VARIABLES 						*/



/* 							EEPROM_VARIABLES 						*/

//int16_t Read_Value[28], Write_Value[28], Prev_Write_Value[28],Read_Value_1[28],Max_val=600;
int16_t Read_Value[36] = {0}, Write_Value[36], Prev_Write_Value[36],Read_Value_1[36],Start_Value,End_Value;
bool Store_Data = 0;
/* 							EEPROM_VARIABLES 						*/

bool Buzz_Switch = 0, Frame_Buzz_Switch=0;

/* 							MANUAL WHEEL CONTROLS 						*/
uint8_t Manual_Axis_1=1,Manual_Axis_2=2,Manual_Axis_3=3,Manual_Axis_4=4,Manual_Axis,All_Whel_SameTorque=1;
float Manual_Torque_1,Manual_Torque_2,Manual_Torque_3,Manual_Torque_4,Manual_Torque;
float Manual_Vel_Limit =20,Manual_Vel_Limit_1=20,Manual_Vel_Limit_2=20,Manual_Vel_Limit_3=20,Manual_Vel_Limit_4=20,Manual_Torque_Temp=0,Manual_Vel_Limit_Temp=20;
float Manual_Vel_Limit_Temp_1=20,Manual_Vel_Limit_Temp_2=20,Manual_Vel_Limit_Temp_3=20,Manual_Vel_Limit_Temp_4=20,Manual_Torque_Temp_1=0,Manual_Torque_Temp_2=0,Manual_Torque_Temp_3=0,Manual_Torque_Temp_4=0;;

uint8_t Left_Vel_Limit = 5, Right_Vel_Limit=5, Prev_Left_Vel_Limit = 15, Prev_Right_Vel_Limit = 15, Left_Transmit_Vel=0, Right_Transmit_Vel=0;
int  Frame_Vel_Limit = 0, Left_Steering_Vel_Limit = 0, Right_Steering_Vel_Limit=0;

int Steering_Angle =0;
double Rover_Centre_Dist=0, TimeTaken=0, Inner_Speed=0, Outer_Speed=0;
int WheelBase=900/2, TrackWidth=1300/2;
double kmph = 1;
int Left_Steering_Speed=0, Right_Steering_Speed=0, Left_Frame_Speed =0;

//int16_t Flap_Data[ARRAY_SIZE] = {0};
//int Flap_Angle = 0, Flap_Count=0;
//float Arm_Angle=0,Left_Arm_Pos = 0, Right_Arm_Pos = 0, Pitch_Arm_Pos = 0,Tri_Arm_Pos = 0, Left_Arm_Pos_Temp = 0, Right_Arm_Pos_Temp = 0, Pitch_Arm_Pos_Temp = 0;

uint64_t left_tick_count = 0;
uint64_t right_tick_count = 0;
bool FLAG = SET;
uint8_t Prev_Joystick = 0;
int count = 0;
float Rover_Velocity = 0.0;

bool JOYSTICK_STATE_FLAG = SET, AXIS_STATE_FLAG = SET, HEARTBEAT_FLAG = SET, FET_TEMP_FLAG = SET, OPERATION_MONITOR_FLAG = SET, MOTORS_STOP_FLAG = SET;
uint64_t Tick_Count1 = 0, Tick_Count2 = 0;
uint16_t Node_Id_Temp[30];
// FET_Temperature[21];
int Node = 0, fet = 0;


float  LF_Error_Change=0, LF_Error_Slope=0, LF_Error_Area=0, LF_Prev_Error=0;
float LF_Kp=3, LF_Ki=1, LF_Kd=0;  
long LF_P=0, LF_I=0, LF_D=0;
float Left_Frame_Out=0;

uint8_t Tx_Uart[2];

int16_t Flap_Data[ARRAY_SIZE] = {0};
float Flap_Angle = 0, Flap_Angle_Right = 0,Flap_Count=0;
int16_t Flap_Data_Right[ARRAY_SIZE] = {0};

int Flap_Max_Angle = 55;
float Arm_Angle=0,Left_Arm_Pos = 0, Right_Arm_Pos = 0, Pitch_Arm_Pos = 0,Tri_Arm_Pos = 0, Left_Arm_Pos_Temp = 0, Right_Arm_Pos_Temp = 0, Pitch_Arm_Pos_Temp = 0;
int Flap_Mod_Value = 0, Flap_Mod_Value_Right = 0;
bool Front_Bush = 1;
uint8_t Array_Element,Arm_Max_Speed=40;
int Left_Arm_Current_Pos,Right_Arm_Current_Pos,Pitch_Arm_Current_Pos;
float Shear_Height_Diff =0, L_Arm_Travel = 0, R_Arm_Travel = 0, Shear_Roll_Angle = 0, Pitch_Compensation_mm = 0, Pitch_Target = 0, Pitch_Error = 0;

float  P_Error_Change=0, P_Error_Slope=0, P_Error_Area=0, P_Prev_Error=0, Pitch_Out =0;


long L_P=0,L_I=0,L_D=0;
float L_Kp=0,L_Ki=0,L_Kd=0;

long RA_P=0,RA_I=0,RA_D=0;
float RA_Kp=0,RA_Ki=0,RA_Kd=0;

long P_P=0,P_I=0,P_D=0;
float P_Kp=0,P_Ki=0,P_Kd=0;

float RightArm_Out=0,RA_Error_Change=0,RA_Error_Slope=0,RA_Error_Area=0,RA_Prev_Error=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM14_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);
void Start_HP_Tasks(void const * argument);
void Start_LP_Tasks(void const * argument);

/* USER CODE BEGIN PFP */

float New_Sensor_Pos(double Sensor_Value, double Zero_Pos);
void Start_Calibration_For (int axis_id, int command_id, uint8_t loop_times);
void Drives_Error_Check(void);
void Reboot (int Axis);
void Heal_Error(uint8_t Axis_Id);
void Stop_Motors(void);
void Set_Motor_Torque ( uint8_t Axis , float Torque );
void Set_Motor_Velocity ( uint8_t Axis , float Velocity );
void CAN_Transmit ( uint8_t NODE, uint8_t Command, float Tx_Data,	uint8_t Data_Size, uint8_t Frame_Format);
float Differintial_Angle ( double Inner_Angle_Set );
void Manual_Controls (void);
void Joystick_Reception(void);
void Drive_Wheel_Controls(void);
void Steering_Controls (void);
void Battery_Status_Indication(void);
float Contour_PID ( float Contour_Val , unsigned long long 	C_Time_Stamp );
float Right_Verticality_PID ( float Right_Roll_Value , unsigned long long 	R_Time_Stamp );
void Frame_Controls(void);
void Dynamic_Width_Adjustment (void);
void EEPROM_Store_Data (void);
void Read_EEPROM_Data(void);
void Frame_Synchronization(void);
void Top_Flap_Sensing(void);
void Manual_Wheel_Control(void);
void Wheel_Speeds_Calc(int Inner_Angle);
void New_Drive_Controls(void);
void Transmit_Motor_Torque (void);
void Left_Frame_Controls (void);
 void Clear_Error (int Axis);
 void Position_Flap_Sensing(void);
 void Shearing_Motors(void);
 void Emergency_Stop(void);
 void Operations_Monitor(void);
 float Left_Frame_PID ( float Left_Error_Value , unsigned long long 	L_Time_Stamp );
 void Top_Sensing_Roll(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Update_Array(int16_t* Array, uint8_t Size, int16_t Latest_Value) 
{//uint8_t Sample_Size =5;
    for (uint8_t i = 0; i < Size - 1; i++) {
        Array[i] = Array[i + 1];
    }   
    Array[Size - 1] = Latest_Value;
		
//		for (uint8_t i = Size - 1 ; i >= Sample_Size ; i ++ )
//		{
//			Flap_Angle += Array[i];
//		}
//		
//		Flap_Angle = Flap_Angle / Sample_Size;
//	Flap_Angle = (Array[Size-1] + Array[Size-2] + Array[Size-3]+ Array[Size-4]+ Array[Size-5]) / 5; 
		
		Flap_Count++;
}


/* 							UART RECEPTION INTERRUPTS 						*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
			HAL_UART_Receive_IT(&huart5,BT_Rx ,sizeof(BT_Rx));
			BT_Count++;
}
/* 							UART RECEPTION INTERRUPTS 						*/
void Absolute_Position_Reception( uint8_t Node_Id )
{
  memcpy(&Absolute_Position[Node_Id],RxData2, sizeof(float)); 
	Absolute_Position_Int[Node_Id] = Absolute_Position[Node_Id]; 
}
float CAN_Reception(uint8_t byte_choice)
{
		float Can_Temp;
	
	if ( byte_choice == LSB )
	{
		for (int k=0; k<=3; k++)
		{
			RxBuff[k]= RxData2[k];
		}
		memcpy(&Can_Temp, RxBuff,4);
		
	}
	
	else if ( byte_choice == MSB )
	{
		for (int k=0; k<=3; k++)
		{
			RxBuff[k]= RxData2[k+4];
		}
		memcpy(&Can_Temp, RxBuff,4);
		
	}
	
	else { }

	return(Can_Temp);
}
float KMPHtoRPS(float kmph)
{
    float rps=0; float Circumference = 1335.177;
    rps = (((kmph/(Circumference/1000)*1000)/3600)*150);
    return rps;
}

uint16_t CAN_SPI_READ(uint8_t Data[8] )
{ uint16_t Enc_Angle=0;
	
				Enc_Angle = Data[0];
				Enc_Angle = Enc_Angle << 8 | Data[1];
	return Enc_Angle;
}	

/*                                CAN 1  Reception                                     */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData);

	switch(RxHeader.StdId)
	{
			
		case (IMU_L)	 :	L_Roll = ((int16_t)(RxData[1]<<8 | RxData[0]))/16.0;	L_Pitch = ((int16_t)(RxData[3]<<8 | RxData[2]))/16.0;				  Sensor_Id[1]++; Node_Id[21]++;	break; 
		
		case (IMU_R)	 :	R_Roll = ((int16_t)(RxData[1]<<8 | RxData[0]))/16.0;	R_Pitch = ((int16_t)(RxData[3]<<8 | RxData[2]))/16.0;				  Sensor_Id[2]++;Node_Id[22]++;		break;
			
		case (LF_STEER): 	Steer_Angle[1] = CAN_SPI_READ(RxData);  			LF_Steering = New_Sensor_Pos ( Steer_Angle[1] , LF_HomePos ) ;						Sensor_Id[3]++;Node_Id[23]++;	 break;
		
		case (LR_STEER): 	Steer_Angle[2] = CAN_SPI_READ(RxData);				LR_Steering = New_Sensor_Pos ( Steer_Angle[2] , LR_HomePos ) ;						Sensor_Id[4]++;Node_Id[24]++; break;
		
		case (RF_STEER): 	Steer_Angle[3] = CAN_SPI_READ(RxData);				RF_Steering = New_Sensor_Pos ( Steer_Angle[3] , RF_HomePos ) ;						Sensor_Id[5]++;Node_Id[25]++;	break;
		
		case (RR_STEER): 	Steer_Angle[4] = CAN_SPI_READ(RxData);	 			RR_Steering = New_Sensor_Pos ( Steer_Angle[4] , RR_HomePos ) ;						Sensor_Id[6]++;Node_Id[26]++; break;
		
/*		case (L_ARM): 		Left_Arm_Raw =CAN_SPI_READ(RxData); 					Left_Arm = New_Sensor_Pos (Left_Arm_Raw, LA_Home_Pos );										ARM_SPI[0]++;				  Sensor_Id[7]++; break;
		
		case (R_ARM): 		Right_Arm_Raw =CAN_SPI_READ(RxData2); 				Right_Arm = New_Sensor_Pos(Right_Arm_Raw, RA_Home_Pos );									ARM_SPI[1]++;				  Sensor_Id[8]++;break;
		
		case (P_Arm_Enc): 		Pitch_Arm_Raw =CAN_SPI_READ(RxData2); 				Pitch_Arm = New_Sensor_Pos(Pitch_Arm_Raw, PA_Home_Pos );     							ARM_SPI[2]++;			    Sensor_Id[9]++;break;
				
		case (L_VERT): 		Left_Vert_Raw =CAN_SPI_READ(RxData2); 				Left_Vertical = New_Sensor_Pos (Left_Vert_Raw, LV_Home_Pos );							FRAME_SPI[0]++;				Sensor_Id[10]++;break;
		
		case (R_VERT): 		Right_Vert_Raw =CAN_SPI_READ(RxData2); 				Right_Vertical= New_Sensor_Pos(Right_Vert_Raw, RV_Home_Pos );							FRAME_SPI[1]++;				Sensor_Id[11]++; break;
		
		case (C_LMT): 		Contour_Limit_Raw =CAN_SPI_READ(RxData2); 		Contour_Limit = New_Sensor_Pos(Contour_Limit_Raw, C_Home_Pos );						FRAME_SPI[2]++;			  Sensor_Id[12]++;break;
*/	
		case (FL_FLAP): 	FL_Raw =CAN_SPI_READ(RxData); 								
											FL_Angle = New_Sensor_Pos(FL_Raw, FL_Home_Pos ); 
											FL_Angle= abs(FL_Angle - 53);										
											Update_Array(Flap_Data ,ARRAY_SIZE , FL_Angle ) ;
											//if ( FL_Angle_Temp != FL_Angle ) {Update_Array(Flap_Data ,ARRAY_SIZE , FL_Angle ) ;  	FL_Angle_Temp = FL_Angle; }	
											Sensor_Id[7]++;
											break;
    case (FR_FLAP): FR_Raw =CAN_SPI_READ(RxData); 
		                Sensor_Id[8]++;
										break;
/*				
		case (FR_FLAP): 	FR_Raw =CAN_SPI_READ(RxData); 								FR_Angle = -(New_Sensor_Pos(FR_Raw, FR_Home_Pos ));													FRAME_SPI[0]++;				Sensor_Id[14]++;break;
		
		case (RL_FLAP): 	RL_Raw =CAN_SPI_READ(RxData); 								RL_Angle = New_Sensor_Pos(RL_Raw, RL_Home_Pos );							  					FRAME_SPI[1]++;				Sensor_Id[15]++; break;
		
		case (RR_FLAP): 	RR_Raw =CAN_SPI_READ(RxData); 								RR_Angle = -(New_Sensor_Pos(RR_Raw, RR_Home_Pos ));													FRAME_SPI[2]++;			  Sensor_Id[16]++;break;
*/		
    default: break;
	}

			RxHeader.StdId=0;
}

void Set_Motor_Position ( uint8_t Axis , float Position )
{
		CAN_Transmit(Axis,POSITION,Position,4,DATA);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan2)
{
	HAL_CAN_GetRxMessage(hcan2, CAN_RX_FIFO1, &RxHeader2, RxData2); // changed


	Received_Node_Id = RxHeader2.StdId >> 5;
	Received_Command_Id = RxHeader2.StdId & CMD_MASK;
		
	switch( Received_Command_Id )
	{
		case HEARTBEAT:  							Node_Id[Received_Node_Id]++;    Axis_State[Received_Node_Id] = RxData2[4]; break;
		
		case ENEST_ID:  							Motor_Velocity[Received_Node_Id]	= CAN_Reception(MSB);  Absolute_Position_Reception(Received_Node_Id); 	   break;		

		case SENS_EST:  							Motor_Velocity[Received_Node_Id]	= CAN_Reception(MSB); 				  			 	 break;
		
		case MERR_ID:  								Motor_Error[Received_Node_Id]			= CAN_Reception(MSB); 								 	 break;
		
		case ENERR_ID:  							Encoder_Error[Received_Node_Id]		= CAN_Reception(MSB); 					 			 	 break;
		
		case SNERR_ID:  							Encoder_Error[Received_Node_Id]		= CAN_Reception(LSB); 					  		 	 break;
		
		case IQM_ID:  								Motor_Current[Received_Node_Id]		= CAN_Reception(MSB); 					  		 	 break;
		
		case VOLTAGE: 								memcpy(&Rover_Voltage, RxData2, 4);	 																		 	 break;

		default: 																																													  	 	 break;

	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM14_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	
	BUZZER_ON;
	HAL_Delay(1000);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	HAL_CAN_Start(&hcan2);HAL_Delay(1000);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
	
	HAL_Delay(1500);
//	for ( uint8_t i = 6 ; i < 25 ; i++ ) {	Start_Calibration_For (i, 8, 10); }
//	for ( uint8_t i = 1 ; i < 5; i++ ) { Start_Calibration_For (6, 8, 5);Start_Calibration_For (13, 8, 5);Start_Calibration_For (12, 8, 5);Start_Calibration_For (14, 8, 5);}
	
	
	/* UART INITS */
	MX_UART4_Init();
	MX_UART5_Init();
	HAL_UART_Receive_IT(&huart5,BT_Rx ,sizeof(BT_Rx));
	/* UART INITS */
	
//	Left_IMU_State = ( Sensor_Id[1] == 0 || Sensor_Id[2]  == 0 ) ? NULL : SET ;
//	if ( !Left_IMU_State ) Error_Handler();
	
//	for(uint8_t i=1 ; i < 5 ; i++) 
//	{
//	CAN_Transmit(i,VEL_LIMIT,20,4,DATA);
//	}
//	
//					for(uint8_t i=1 ; i < 5 ; i++) 
//					{
//						CAN_Transmit(i,VEL_LIMIT,30,4,DATA);
//						HAL_Delay(20);
//						Prev_Vel_Limit = 30;
//					}
//	HAL_Delay(1500);
//	Reboot(4);HAL_Delay(1500);
//	while ( Axis_State[1] != 8 || Axis_State[2] != 8 || Axis_State[3] != 8 || Axis_State[4] != 8  )
//	{
		
//			if ( Axis_State[1] != 8 ) {Reboot(1); HAL_Delay(2500);}
//			if ( Axis_State[2] != 8 ) {Reboot(2); HAL_Delay(2500);}
//			if ( Axis_State[3] != 8 ) {Reboot(3); HAL_Delay(2500);}
//			if ( Axis_State[4] != 8 ) {Reboot(4); HAL_Delay(2500);}
//	}
//Lower_Width_Motor_Count = Upper_Width_Motor_Count = 500;
//memcpy(&Write_Value[0], &Lower_Width_Motor_Count, sizeof(Lower_Width_Motor_Count));
//	memcpy(&Write_Value[4], &Upper_Width_Motor_Count, sizeof(Upper_Width_Motor_Count));
//	
//	EEPROM_Write(3, 0, (uint8_t *)Write_Value, sizeof(Write_Value));
	
	Steering_Reset_Flag = SET;
	LF_Speed= LR_Speed=RF_Speed=RR_Speed=0;
//	for (int i = 1; i <= 256; i++)
//	{
//	EEPROM_PageErase(i);
//	}
	Read_EEPROM_Data();	
	
//	Left_Arm_Motor_Value  = 0;
//	Right_Arm_Motor_Value = 0;
//	Pitch_Arm_Motor_Value = 0;
//Lower_Width_Motor_Value = 0;	
//Upper_Width_Motor_Value = 0;
//Right_Vertical_Motor_Value=0;
//Contour_Motor_Value=0;

	BUZZER_OFF;
	//Initiate_Process = SET;
//	Error_Handler();
HAL_TIM_Base_Start_IT(&htim14);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of DefaultTasks */
//  osThreadDef(DefaultTasks, StartDefaultTask, osPriorityNormal, 0, 128);
//  DefaultTasksHandle = osThreadCreate(osThread(DefaultTasks), NULL);

//  /* definition and creation of HP_Tasks */
//  osThreadDef(HP_Tasks, Start_HP_Tasks, osPriorityAboveNormal, 0, 128);
//  HP_TasksHandle = osThreadCreate(osThread(HP_Tasks), NULL);

//  /* definition and creation of LP_Tasks */
//  osThreadDef(LP_Tasks, Start_LP_Tasks, osPriorityBelowNormal, 0, 128);
//  LP_TasksHandle = osThreadCreate(osThread(LP_Tasks), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		BT_State = BT_READ;
		Joystick_Reception();
	EEPROM_Store_Data();
		   Operations_Monitor();
		if(OPERATION_MONITOR_FLAG==SET){
		//Drives_Error_Check();
		Left_Frame_Controls();
		New_Drive_Controls();
    Steering_Controls();
	   Frame_Controls();
		Dynamic_Width_Adjustment();
	Shearing_Motors ();
			//Top_Sensing_Roll();
		Shearing_Motors();
		}
		else{Emergency_Stop();}
		
		//Position_Flap_Sensing();
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0;	// which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0x000<<5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0x000<<5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 14; //14 // how many filters to assign to the CAN1 (master can)

	HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 12;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 14; //14 // which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	canfilterconfig.FilterIdHigh = 0x000<<5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0x000<<5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 14;//14	// how many filters to assign to the CAN1 (master can)

	HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 9000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_1_Pin|LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Buzzer_1_Pin|Buzzer_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_1_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_2_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UART4_State_Pin */
  GPIO_InitStruct.Pin = UART4_State_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UART4_State_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_1_Pin Buzzer_2_Pin */
  GPIO_InitStruct.Pin = Buzzer_1_Pin|Buzzer_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : UART5_State_Pin */
  GPIO_InitStruct.Pin = UART5_State_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UART5_State_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CAN_Transmit ( uint8_t NODE, uint8_t Command, float Tx_Data,	uint8_t Data_Size, uint8_t Frame_Format)
{
//uint64_t BUFF;
	TxHeader.ExtId = NULL;
	
	TxHeader.TransmitGlobalTime = DISABLE;
	
	TxHeader.IDE = CAN_ID_STD;
	
	TxHeader.DLC	= Data_Size;
	
	TxHeader.RTR = (Frame_Format == REMOTE) ? (CAN_RTR_REMOTE) : (Frame_Format == DATA) ? (CAN_RTR_DATA):(CAN_RTR_REMOTE);
	
	switch (Command)
	{
		case VELOCITY:	
									memcpy (TxData, &Tx_Data, Data_Size);					
									TxHeader.StdId = (NODE << 5)| VEL_ID;
									break;
		
		case POSITION:	
									memcpy (TxData, &Tx_Data, Data_Size);					
									TxHeader.StdId = (NODE << 5)| POS_ID;
									break;
		
		case TORQUE:	
									memcpy (TxData, &Tx_Data, Data_Size);					
									TxHeader.StdId = (NODE << 5)| TRQ_ID;
							//	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
									break;
		
		case VEL_LIMIT:	

									//TxData[6] = 0x20; for 40 amps//8C - 70//0x70-60 amps
									TxData[6] = 0x8C;
									TxData[7] = 0x42;
									memcpy (TxData, &Tx_Data, 4);	
									TxHeader.DLC	= 8;
									TxHeader.StdId = (NODE << 5)| 0x00F;
									break;
		
		case MOT_ERROR: 	
									TxHeader.StdId = (NODE << 5)| MERR_ID;
									break;
		
		case ENC_ERROR:					
									TxHeader.StdId = (NODE << 5)| ENERR_ID;
									break;
		
		case SNL_ERROR:					
									TxHeader.StdId = (NODE << 5)| SNERR_ID;
									break;
		
		case IQ:						
									TxHeader.StdId = (NODE << 5)| IQM_ID;
									break;
		
		case ENC_EST:					
									TxHeader.StdId = (NODE << 5)| ENEST_ID;
									break;
									
		case T_RAMP:					
									memcpy (TxData, &Tx_Data, Data_Size);	
									TxHeader.StdId = (NODE << 5)| T_RAMP_ID;
									break;
		
		case SENSL_EST:					
									memcpy (TxData, &Tx_Data, Data_Size);	
									TxHeader.StdId = (NODE << 5)| 0x015;
									break;
		
		case REQ_STATE:	 
									break;
		
		case 0x017:
									TxHeader.StdId = (NODE << 5)| 0x017;
									break;
		
		default: break;
		

	}
//	
		for ( uint8_t i=0 ; i<5; i++ ) //1
	{

				HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox); 
				//if ( Initiate_Process) HAL_Delay(1);
				CAN_Count++;
	}
	
	for ( uint8_t i=0 ; i<8; i++ ) 
	{
		TxData[i] = 0;
	}
				

}
float New_Sensor_Pos(double Sensor_Value, double Zero_Pos)
{
double output;

output=((Sensor_Value-Zero_Pos)>360.0)? ((Sensor_Value-Zero_Pos)-720.0) : (Sensor_Value-Zero_Pos);
output = (output<-359.0)?(output+720.0):(output);
return (output/2);

	
/*output=((sensorvalue-zero_pos)>(360.0/2))? ((sensorvalue-zero_pos)-(720.0/2)) : (sensorvalue-zero_pos);
output = (output<-(359.0/2))?(output+(720.0/2)):(output);
return (output);*/
}

void Start_Calibration_For (int axis_id, int command_id, uint8_t loop_times)
{
				memcpy(TxData, &command_id, 4);		
				TxHeader.DLC = 4;	
				TxHeader.IDE = CAN_ID_STD;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.StdId = ( axis_id <<5) | 0x007 ;	
			HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);HAL_Delay(20); 		
}
void Drives_Error_Check(void)
{

	for(uint8_t i = 1; i < 5; i++)
	{
		if ( i != 5 ){ if ( Axis_State[i] != 8 ){ DRIVES_ERROR_FLAG = SET; Heal_Error(i); HAL_Delay(10);
			 } //HAL_Delay(3); 
		}
		
		//HAL_Delay(3);
	}
}
void Reboot (int Axis)
{	
	TxHeader.DLC = 4;	
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = ( Axis <<5) | 0x016 ;	
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
	//HAL_Delay(1); 		
}
void Clear_Error(int Axis)
{
	TxHeader.DLC = 4;	
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = ( Axis <<5) | 0x018 ;	
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
}
void Set_Motor_Torque ( uint8_t Axis , float Torque )
{
	Torque =  (Axis==2) ? -Torque : Torque ;	

	CAN_Transmit(Axis,TORQUE,Torque,4,DATA); HAL_Delay(3);//10
}
void Set_Motor_Velocity ( uint8_t Axis , float Velocity )
{
	Velocity = Axis == 9 ? -Velocity : Velocity;
	 CAN_Transmit(Axis,VELOCITY,Velocity,4,DATA); HAL_Delay(2);
}
void Stop_Motors(void)
{
			for(uint8_t i = 1; i <= 4; i++){Set_Motor_Torque(i, 0);}	for(uint8_t i = 8; i <= 11; i++){Set_Motor_Velocity(i, 0);}
LF_Speed_Temp = RF_Speed_Temp = LR_Speed_Temp = RR_Speed_Temp = 0;
}

void Read_EEPROM_Data(void)
{
	//Lower_Width_Motor_Count = -1000;	
//Upper_Width_Motor_Count = 188;	
//					 memcpy(&Write_Value[0], &Lower_Width_Motor_Count, sizeof(Lower_Width_Motor_Count));
//memcpy(&Write_Value[4], &Upper_Width_Motor_Count, sizeof(Upper_Width_Motor_Count));						
//	EEPROM_Write(3, 0, (uint8_t *)Write_Value, sizeof(Write_Value));
					
//	EEPROM_PageErase (3);				
	EEPROM_Read(60, 0, (uint8_t *)Read_Value, sizeof(Read_Value));
//	EEPROM_Read(6, 0, (uint8_t *)Read_Value_1, sizeof(Read_Value_1));
	memcpy(&Lower_Width_Motor_Value, &Read_Value[28],4 );	 				
	memcpy(&Upper_Width_Motor_Value, &Read_Value[4],4 );
	memcpy(&Left_Arm_Motor_Value, &Read_Value[8],4 );
	memcpy(&Right_Arm_Motor_Value, &Read_Value[12],4 );
	memcpy(&Pitch_Arm_Motor_Value, &Read_Value[16],4 );
	memcpy(&Right_Vertical_Motor_Value, &Read_Value[20],4 );
	memcpy(&Contour_Motor_Value, &Read_Value[24],4 );
	
	
//	if(Read_Value[0]>=600) 
//	{memcpy(&Lower_Width_Motor_Value, &Read_Value_1[28],4 );}	 
//  if(Read_Value[4]>=600)	
//	{memcpy(&Upper_Width_Motor_Value, &Read_Value_1[4],4 );}
//	if(Read_Value[8]>=600)
//	{memcpy(&Left_Arm_Motor_Value, &Read_Value_1[8],4 );}
//	if(Read_Value[12]>=600)
//	{memcpy(&Right_Arm_Motor_Value, &Read_Value_1[12],4 );}
//	if(Read_Value[16]>=600)
//	{memcpy(&Pitch_Arm_Motor_Value, &Read_Value_1[16],4 );}
//	if(Read_Value[20]>=600)
//	{memcpy(&Right_Vertical_Motor_Value, &Read_Value_1[20],4 );}
//	if(Read_Value[24]>=600)
//	{	memcpy(&Contour_Motor_Value, &Read_Value_1[24],4 );}
	
	
	Lower_Width_Motor_Value 		 = Lower_Width_Motor_Value 			== -1 ? 0 : Lower_Width_Motor_Value;
	Upper_Width_Motor_Value 		 = Upper_Width_Motor_Value 			== -1 ? 0 : Upper_Width_Motor_Value;
	Left_Arm_Motor_Value    		 = Left_Arm_Motor_Value    			== -1 ? 0 : Left_Arm_Motor_Value;
	Right_Arm_Motor_Value  		   = Right_Arm_Motor_Value   			== -1 ? 0 : Right_Arm_Motor_Value;
	Pitch_Arm_Motor_Value   		 = Pitch_Arm_Motor_Value   			== -1 ? 0 : Pitch_Arm_Motor_Value;
	Right_Vertical_Motor_Value   = Right_Vertical_Motor_Value   == -1 ? 0 : Right_Vertical_Motor_Value;
	Contour_Motor_Value   			 = Contour_Motor_Value          == -1 ? 0 : Contour_Motor_Value;

}
void Heal_Error(uint8_t Axis_Id)
{
	BUZZER_ON;
	Stop_Motors();
	
	while ( Axis_State[Axis_Id] != 8 )
	{
		Reboot(Axis_Id);	HAL_Delay(2000);
		//Start_Calibration_For ( Axis_Id,  8 , 2 ); HAL_Delay(1500);
	}
	
	DRIVES_ERROR_FLAG = NULL;
	BUZZER_OFF;
}

void Joystick_Reception(void)
{
	/*				JOYSTICK VALUES ASSIGNING								*/
	if (( BT_Rx[0] == 0xAA ) &&	( BT_Rx[7] == 0xFF ))
	{	
		Mode 						 = BT_Rx[1];
		Speed 					 = BT_Rx[2]  != 0 ? BT_Rx[2] : Speed ;
		Steering_Mode 	 = BT_Rx[3];
		Pot_Angle        = BT_Rx[4]; 
		Joystick         = BT_Rx[5];
		Shearing				 = BT_Rx[6];
//		Pot_Angle        = abs(Pot_Angle-180);
		if( Steering_Mode == 0 ) Steering_Mode=1;
	
		/*				Steering Reset on every Steering Mode Change								*/
		if ( BT_Steer_Temp != BT_Rx[3])
		{	
			if ( Steering_Mode_Temp != Steering_Mode )
			{
				Steering_Reset_Flag = SET;
				Steering_Mode_Temp=Steering_Mode;
			}
			BT_Steer_Temp = BT_Rx[3];
		}
	/*				Steering Reset on every Steering Mode Change			  				*/
	}
	else {}
}
void Manual_Wheel_Control(void)
{ 
	if(All_Whel_SameTorque==1){
		
//	for(uint8_t i=1;i<5;i++){
//		if(Manua+6l_Vel_Limit != Manual_Vel_Limit_Temp)	{
//			   CAN_Transmit(i,VEL_LIMIT,Manual_Vel_Limit,4,DATA);HAL_Delay(10);}
//	   	   Manual_Vel_Limit_Temp = Manual_Vel_Limit;
//		if(Manual_Torque != Manual_Torque_Temp ) 
//			{
//	       Set_Motor_Torque (i , Manual_Torque );HAL_Delay(10);}
//         Manual_Torque_Temp = Manual_Torque;	
//			
//	}
				if ( Manual_Torque_Temp != Manual_Torque )
		{
			for(uint8_t i = 1; i < 5 ; i++ ) {Set_Motor_Torque ( i , Manual_Torque); HAL_Delay(1);} 
			Manual_Torque_Temp = Manual_Torque;
		}
		if(Manual_Vel_Limit != Manual_Vel_Limit_Temp)
		{
			   for(int i=1;i<=4;i++){ CAN_Transmit(i,VEL_LIMIT,Manual_Vel_Limit,4,DATA);HAL_Delay(10); }
	   	   Manual_Vel_Limit_Temp = Manual_Vel_Limit;
		}
		
		
}
	else if(All_Whel_SameTorque==0){
		
  if(Manual_Torque_1 != Manual_Torque_Temp_1 ){
	      Set_Motor_Torque (Manual_Axis_1 , Manual_Torque_1 );HAL_Delay(1);
	      Manual_Torque_1 = Manual_Torque_Temp_1;}
	if(Manual_Vel_Limit_1 != Manual_Vel_Limit_Temp_1){
	      CAN_Transmit(Manual_Axis_1,VEL_LIMIT,Manual_Vel_Limit_1,4,DATA);HAL_Delay(1);
	      Manual_Vel_Limit_1 = Manual_Vel_Limit_Temp_1;
	}
	
	  if(Manual_Torque_2 != Manual_Torque_Temp_2 || Manual_Vel_Limit_2 != Manual_Vel_Limit_Temp_2 ){
	      Set_Motor_Torque (Manual_Axis_2 , Manual_Torque_2 );HAL_Delay(1);
			  Manual_Torque_2 = Manual_Torque_Temp_2;
	      CAN_Transmit(Manual_Axis_2,VEL_LIMIT,Manual_Vel_Limit_2,4,DATA);HAL_Delay(1);
	      Manual_Vel_Limit_2 = Manual_Vel_Limit_Temp_2;
	}
		
		 if(Manual_Torque_3 != Manual_Torque_Temp_3 || Manual_Vel_Limit_3 != Manual_Vel_Limit_Temp_3 ){
	         Set_Motor_Torque (Manual_Axis_3 , Manual_Torque_3 );HAL_Delay(1);
			     Manual_Torque_3 = Manual_Torque_Temp_3;
	         CAN_Transmit(Manual_Axis_3,VEL_LIMIT,Manual_Vel_Limit_3,4,DATA);HAL_Delay(1);	       
	         Manual_Vel_Limit_3 = Manual_Vel_Limit_Temp_3;
	 }
		
		  if(Manual_Torque_4 != Manual_Torque_Temp_4 || Manual_Vel_Limit_4 != Manual_Vel_Limit_Temp_4 ){
	          Set_Motor_Torque (Manual_Axis_4 , Manual_Torque_4 );HAL_Delay(1);
				    Manual_Torque_4 = Manual_Torque_Temp_4;
	          CAN_Transmit(Manual_Axis_4,VEL_LIMIT,Manual_Vel_Limit_4,4,DATA);HAL_Delay(1);
	          Manual_Vel_Limit_4 = Manual_Vel_Limit_Temp_4;
		}
	
	}
}
void Manual_Controls (void)
{
	/*------------------------------WHEEL_MOTORS----------------------------------- */
	if ( Left_Wheels_Torque != Left_Wheels_Torque_Temp )
	{
		CAN_Transmit(1,VEL_LIMIT,30,4,DATA);HAL_Delay(1);
		CAN_Transmit(2,VEL_LIMIT,30,4,DATA);HAL_Delay(1);
		Set_Motor_Torque ( 1 , Left_Wheels_Torque );HAL_Delay(1);
		Set_Motor_Torque ( 2 , Left_Wheels_Torque );
		
		Left_Wheels_Torque_Temp = Left_Wheels_Torque;
	}
	
	/*------------------------------WHEEL_MOTORS----------------------------------- */
	/*------------------------------FRAME_MOTORS----------------------------------- */
	
//		if( L_Vert_Speed_Temp != L_Vert_Speed ) 																															// checking if the new value is not equal to old value
//			{
//				Set_Motor_Velocity (LVert , L_Vert_Speed );
//				L_Vert_Speed_Temp = L_Vert_Speed ;																																// Overwriting old value with new value. 
//			}		

		if( R_Vert_Speed_Temp != R_Vert_Speed ) 																															// checking if the new value is not equal to old value
			{
				Set_Motor_Velocity (RVert , R_Vert_Speed );	
				R_Vert_Speed_Temp = R_Vert_Speed ;																																// Overwriting old value with new value.
			} 

		if( Contour_Speed_Temp != Contour_Speed ) 																														// checking if the new value is not equal to old value
			{
				Set_Motor_Velocity (Contour , Contour_Speed );
				Contour_Speed_Temp = Contour_Speed ;																															// Overwriting old value with new value.
			}	
	/*------------------------------FRAME_MOTORS----------------------------------- */
	/*------------------------------ARM_MOTORS------------------------------------- */
		if( L_Arm_Speed_Temp != L_Arm_Speed )
		{
			Set_Motor_Velocity (L_Arm , L_Arm_Speed );	
			L_Arm_Speed_Temp = L_Arm_Speed ;
		}
		
		if( R_Arm_Speed_Temp != R_Arm_Speed )
		{
			Set_Motor_Velocity (R_Arm , R_Arm_Speed );	
			R_Arm_Speed_Temp = R_Arm_Speed ;
		}
		if( Pitch_Arm_Speed_Temp != Pitch_Arm_Speed ) 
		{
			Set_Motor_Velocity (P_Arm , Pitch_Arm_Speed );	
			Pitch_Arm_Speed_Temp = Pitch_Arm_Speed ;
		}
	/*------------------------------ARM_MOTORS------------------------------------- */
	/*---------------------------STEERING_MOTORS----------------------------------- */
	if ( LF_Speed_Temp != LF_Speed )
	{
		LF_Speed= LF_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : LF_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : LF_Speed; 
		Set_Motor_Velocity( LFS , -LF_Speed );
		LF_Speed_Temp = LF_Speed ;
	}
	
	if ( LR_Speed_Temp != LR_Speed )
	{
		LR_Speed= LR_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : LR_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : LR_Speed;
		 Set_Motor_Velocity( LRS , -LR_Speed );
		LR_Speed_Temp = LR_Speed ;
	}
	if ( RF_Speed_Temp != RF_Speed )
	{	
		RF_Speed= RF_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : RF_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : RF_Speed;
		Set_Motor_Velocity( RFS , -RF_Speed );
		RF_Speed_Temp = RF_Speed ;
	}

	if ( RR_Speed_Temp != RR_Speed )
	{
		RR_Speed= RR_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : RR_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : RR_Speed;
		Set_Motor_Velocity( RRS , -RR_Speed );
		RR_Speed_Temp = RR_Speed ;
	}
	/*---------------------------STEERING_MOTORS----------------------------------- */
	
	/*------------------------------WIDTH_MOTORS----------------------------------- */
//	if ( Width_Motor_Speed != Width_Motor_Temp)
//	{
//		Set_Motor_Velocity ( Upper_Width, Width_Motor_Speed); 
//	//	Set_Motor_Velocity ( Lower_Width, Width_Motor_Speed);
//		Width_Motor_Temp = Width_Motor_Speed; 
//	}
	/*------------------------------WIDTH_MOTORS----------------------------------- */
	


}
void Drive_Wheel_Controls(void)
{
	/* Actuates the Drive Wheels only when the Steering Resets have been completed and the Bluetooth is in Connection. */

//	if ( (!Steering_Reset_Flag) && (Speed!= 0) && (BT_State)) // ----> ACUTAL CONDITION
	if ( (Speed!= 0) && Left_IMU_State  ) //&& (Steering_Mode!= 1) )//&& (BT_State))   // mode == 2 added
  {
		
//  if ( (R_R_Err > 6 || R_R_Err < -6) || (C_Err > 6 || C_Err < -6) ) {Joystick = 0; }// Safety STOP  }
		
//	Vel_Limit = Joystick == 0 ? 20 : Speed * 30;
//	Vel_Limit = Joystick == 0 ? 20 : Speed * 16; // actual
		//Vel_Limit = Speed == 1 ? 25 : Speed == 2 ? 35 : Speed == 3 ? 45: Speed;
		if ( Joystick == 1 )
		{
		if (Speed == 1 && Motor_Velocity[3] > 15 ) Vel_Limit = 30;
		if ( Speed == 2 && Motor_Velocity[3] > 15) Vel_Limit = 45;
		if ( Speed == 3 && Motor_Velocity[3] > 15) Vel_Limit = 65;
		}
		else if ( Joystick == 2 )
		{
		if (Speed == 1 && Motor_Velocity[3] < -15 ) Vel_Limit = 30;
		if ( Speed == 2 && Motor_Velocity[3] < -15) Vel_Limit = 45;
		if ( Speed == 3 && Motor_Velocity[3] < -15) Vel_Limit = 65;
		}
		
		
		Vel_Limit = Joystick == 0 ? 20 : Vel_Limit;
		
		if ( Motor_Velocity[3] < 15 && Motor_Velocity[3] > -15 )
		{
			if ( Joystick_Temp != Joystick )
			{
				switch (Joystick)
				{
					case 0 :   Torque =  NULL; 							break;								
					case 1 :   Torque =	 Wheel_Torque;			break; 
					case 2 :   Torque = -Wheel_Torque;			break; 
					default:																break;
				}
				
				if ( Steering_Mode == ZERO_TURN ) //Mode - 3 : zero turn
				{
					for ( uint8_t i = 1 ; i < 5 ; i++ )
					{
						Torque = (i==3 )  ? -Torque : Torque ;   
						Set_Motor_Torque ( i , Torque );
					}	
				}
				else
				{     
					for ( uint8_t i = 1 ; i < 5 ; i++ )
					{ 
							Set_Motor_Torque ( i , Torque );
					}
				}
				Joystick_Temp = Joystick;
			}
		}
			
		
			if ( (Vel_Limit_Temp != Vel_Limit) && Vel_Limit != 0  )
			{
			
				if( Prev_Vel_Limit > Vel_Limit ) 	
				{ for(uint8_t i = 8; i <= 11; i++){Set_Motor_Velocity(i, 0);}
				 for ( uint8_t  v = Prev_Vel_Limit-2; v >= Vel_Limit  ; v=v-2 )
				 {	
					 for(uint8_t i=1 ; i < 5 ; i++) 
					{
						if(v>10){CAN_Transmit(i,VEL_LIMIT,v,4,DATA);HAL_Delay(3);}
					}
					HAL_Delay(50);
					 
				 }		
				 Prev_Vel_Limit = Vel_Limit ;
				}
				else
				{for(uint8_t i = 8; i <= 11; i++){Set_Motor_Velocity(i, 0);}
				 for ( uint8_t v = Prev_Vel_Limit+2; v <= Vel_Limit  ; v=v+2 )
				 {	
					 for(uint8_t i=1 ; i < 5 ; i++) 
					{
						if(v>10){CAN_Transmit(i,VEL_LIMIT,v,4,DATA);HAL_Delay(3);}
					}
					HAL_Delay(100);
				 }
				 Prev_Vel_Limit = Vel_Limit ;
				}
			 Vel_Limit_Temp = Vel_Limit;
			}
			
		
		if ( Motor_Velocity[3] > 15 || Motor_Velocity[3] < -15 )
		{
			if ( Joystick_Temp != Joystick )
			{
				switch (Joystick)
				{
					case 0 :   Torque =  NULL; 							break;								
					case 1 :   Torque =	 Wheel_Torque;			break; 
					case 2 :   Torque = -Wheel_Torque;			break; 
					default:																break;
				}
				
				if ( Steering_Mode == ZERO_TURN ) //Mode - 3 : zero turn
				{
					for ( uint8_t i = 1 ; i < 5 ; i++ )
					{
						Torque = (i==3 )  ? -Torque : Torque ;   
						Set_Motor_Torque ( i , Torque );
					}	
				}
				else
				{     
					for ( uint8_t i = 1 ; i < 5 ; i++ )
					{ 
							Set_Motor_Torque ( i , Torque );
					}
				}
				Joystick_Temp = Joystick;
			}
		}
			
		if ( (!BT_State ) && Joystick != 0 )
		{
			for ( uint8_t i = 1 ; i < 5 ; i++ ){Set_Motor_Torque ( i , NULL );}
			Joystick = 0 ;
			BUZZER_ON;
		}
	
	}
}
void Transmit_Motor_Torque (void)
{
		if ( Joystick_Temp != Joystick )
		{
			switch (Joystick)
			{
				case 0 :   Torque = NULL; 							break;								
				case 1 :   Torque =	Wheel_Torque;				break; 
				case 2 :   Torque =-Wheel_Torque;				break; 
				default :																break;
			}
			Joystick_Temp = Joystick;
		}
		
		if ( Torque_Temp != Torque )
		{
			if ( Steering_Mode == ZERO_TURN ) //Mode - 3 : zero turn
			{
				for ( uint8_t i = 1 ; i < 5 ; i++ )
				{
				 Torque = (i==3 ) ? -Torque : Torque ;   
				 Set_Motor_Torque ( i , Torque );
				}	
			}
			else
			{
			 for ( uint8_t i = 1 ; i < 5 ; i++ )
			 { 
				Set_Motor_Torque ( i , Torque ); 
			 }
			}
			Torque_Temp = Torque;
		}
		Prev_Joystick = Joystick;
}

//void New_Drive_Controls(void)
//{
//	if ( (Speed!= 0) && Left_IMU_State  ) //&& (Steering_Mode!= 1) )//&& (BT_State))   // mode == 2 added
//	{
////		Vel_Limit = Speed*15;
////		Vel_Limit = Vel_Limit > 50 ? 50 : Vel_Limit < 10 ? 10 : Vel_Limit;

//		if ( (R_R_Err > 6 || R_R_Err < -6) || (C_Err > 6 || C_Err < -6) || (Left_Vertical_Error > 6 || Left_Vertical_Error < -6) ){ Joystick = 0;}// Stop_Motors(); }// Safety STOP  (L_R_Err > 5 || L_R_Err < -5)
//	
//		if ( Joystick == 1 )
//		{
//		if ( Motor_Velocity[3] > 10 ) Vel_Limit = Speed*15;
//		}
//		else if ( Joystick == 2 )
//		{
//		if ( Motor_Velocity[3] < -10 ) Vel_Limit = Speed*15;
//		}
//		
//		Vel_Limit = Joystick == 0 ? 10 : Vel_Limit;   //15
//		
//		if ( Steering_Mode != ALL_WHEEL ){ Left_Steering_Speed = Right_Steering_Speed = 0; }
//	//		Left_Frame_Speed=0;
//			Left_Vel_Limit = Vel_Limit  + Left_Steering_Speed + Left_Frame_Speed+4;
//			Right_Vel_Limit = Vel_Limit + Right_Steering_Speed+4;
//	/////////////////////////////////////////////////////////////////////////APPLYING TORQUE/////////////////////////////////////////////////////////////////////////////////////////////////////////////
////		if ( Motor_Velocity[3] < 10 && Motor_Velocity[3] > -10 )
////		{
////		Transmit_Motor_Torque();
////		}
//		
//			
//				if (Joystick == 1 || Joystick == 2)
//				{
//					if (Joystick != Joystick_Temp)
//					{
//						Transmit_Motor_Torque();
//						Joystick_Temp = Joystick;
//						//FLAG = SET;
//					}
//				}
//				
//				if (Joystick == 0)
//				{
//					//if ((Motor_Velocity[1] <= Left_Vel_Limit + 2 && Motor_Velocity[1] >= Left_Vel_Limit - 2) && (Motor_Velocity[3] <= Right_Vel_Limit + 2 && Motor_Velocity[3] >= Right_Vel_Limit - 2))
//					//{
//						Transmit_Motor_Torque();
//						//FLAG = NULL;
//					//}
//				}
//				
//				
//					
//				
//			
//			
//		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////		Transmit_Motor_Torque();
//		
///*	
////		if ( Joystick_Temp != Joystick )
////		{
 ////			switch (Joystick)
////			{
////				case 0 :   Torque = NULL; 							break;								
////				case 1 :   Torque =	Wheel_Torque;				break; 
////				case 2 :   Torque =-Wheel_Torque;				break; 
////				default :																break;
////			}
////			Joystick_Temp = Joystick;
////		}
////		
////		if ( Torque_Temp != Torque )
////		{
////			if ( Steering_Mode == ZERO_TURN ) //Mode - 3 : zero turn
////			{
////				for ( uint8_t i = 1 ; i < 5 ; i++ )
////				{
////				 Torque = (i==3 ) ? -Torque : Torque ;   
////				 Set_Motor_Torque ( i , Torque );
////				}	
////			}
////			else
////			{
////             for ( uint8_t i = 1 ; i < 5 ; i++ )
////             { 
////              Set_Motor_Torque ( i , Torque ); 
////             }
////			}
////			Torque_Temp = Torque;
////		}*/

//			
//		
//		//if ( Motor_Velocity[3] > 10 || Motor_Velocity[3] < -10 ){Transmit_Motor_Torque();}
//		
////			if ( Left_Vel_Limit > Prev_Left_Vel_Limit)
////			{
////				Left_Transmit_Vel = Prev_Left_Vel_Limit + 1;
////				for(uint8_t i=1 ; i <= 2 ; i++) { CAN_Transmit(i,VEL_LIMIT,Left_Transmit_Vel,4,DATA);HAL_Delay(1); }
////				Prev_Left_Vel_Limit = Left_Transmit_Vel; HAL_Delay(5);
////			}
////			else if ( Left_Vel_Limit < Prev_Left_Vel_Limit)
////			{
////				Left_Transmit_Vel = Prev_Left_Vel_Limit - 1;
////				for(uint8_t i=1 ; i <= 2 ; i++) { CAN_Transmit(i,VEL_LIMIT,Left_Transmit_Vel,4,DATA);HAL_Delay(1); }
////				Prev_Left_Vel_Limit = Left_Transmit_Vel; HAL_Delay(5);
////			}
//			//////////////////////////////////////////////////////////////////////////////////////////////////////////
//			if(Left_Vel_Limit != Left_Transmit_Vel)
//			{
//				if(HAL_GetTick() - left_tick_count >= 50)
//				{
//					if(Left_Vel_Limit > Left_Transmit_Vel)
//					{
//						Left_Transmit_Vel++;
//					}
//					
//					else if(Left_Vel_Limit < Left_Transmit_Vel)
//					{
//						Left_Transmit_Vel--;
//						

//					}
//					else{}
//					
//					for(uint8_t i=1 ; i <= 2 ; i++) { CAN_Transmit(i,VEL_LIMIT,Left_Transmit_Vel,4,DATA);HAL_Delay(1);}
//					left_tick_count = HAL_GetTick();
//				
//			}
//		}
//			
//			if(Right_Vel_Limit != Right_Transmit_Vel)
//			{
//				if(HAL_GetTick() - right_tick_count >= 50)
//				{
//					if(Right_Vel_Limit > Right_Transmit_Vel)
//					{
//						Right_Transmit_Vel++;
//					}
//					
//					else if(Right_Vel_Limit < Right_Transmit_Vel)
//					{
//						Right_Transmit_Vel--;
//					}
//					
//					else{}
//						
//					for(uint8_t i=3 ; i <= 4 ; i++) { CAN_Transmit(i,VEL_LIMIT,Right_Transmit_Vel,4,DATA);HAL_Delay(1);}
//					right_tick_count = HAL_GetTick();
//				}
//			}
//			////////////////////////////////////////////////////////////////////////////////////////////////////////////
//			
//			 
//			
////			if ( Right_Vel_Limit > Prev_Right_Vel_Limit)
////			{
////				Right_Transmit_Vel = Prev_Right_Vel_Limit + 1;
////				for(uint8_t i=3 ; i <= 4 ; i++) { CAN_Transmit(i,VEL_LIMIT,Right_Transmit_Vel,4,DATA);HAL_Delay(1); }
////				Prev_Right_Vel_Limit = Right_Transmit_Vel; HAL_Delay(5);
////			}
////			else if ( Right_Vel_Limit < Prev_Right_Vel_Limit)
////			{
////				Right_Transmit_Vel = Prev_Right_Vel_Limit - 1;
////				for(uint8_t i=3 ; i <= 4 ; i++) { CAN_Transmit(i,VEL_LIMIT,Right_Transmit_Vel,4,DATA);HAL_Delay(1); }
////				Prev_Right_Vel_Limit = Right_Transmit_Vel; HAL_Delay(5);
////			}
//			
//		//if ( Motor_Velocity[3] > 10 || Motor_Velocity[3] < -10 ){Transmit_Motor_Torque();}

//	}	
//	else 
//	{
//		for ( uint8_t i = 1 ; i < 5 ; i++ )
//		{ 
//				Set_Motor_Torque ( i , 0 ); HAL_Delay(1);
//		}
//	}
//	
////	if ( (!BT_State ) && Joystick != 0 )
////	{
////		for ( uint8_t i = 1 ; i < 5 ; i++ ){Set_Motor_Torque ( i , NULL ); HAL_Delay(1);}
////		Joystick = 0 ;
////	}
//}

 void New_Drive_Controls(void)
{
	if ( (Speed!= 0) && Left_IMU_State  ) //&& (Steering_Mode!= 1) )//&& (BT_State))   // mode == 2 added
	{
		Rover_Velocity = (fabs(Motor_Velocity[1]) + fabs(Motor_Velocity[2]) + fabs(Motor_Velocity[3]) + fabs(Motor_Velocity[4])) / 4;
//		Vel_Limit = Speed*15;
//		Vel_Limit = Vel_Limit > 50 ? 50 : Vel_Limit < 10 ? 10 : Vel_Limit;

	//	if ( (R_R_Err > 6 || R_R_Err < -6) || (C_Err > 6 || C_Err < -6) || (Left_Vertical_Error > 6 || Left_Vertical_Error < -6) ){ Joystick = 0;}// Stop_Motors(); }// Safety STOP  (L_R_Err > 5 || L_R_Err < -5)
	
//		if ( Joystick == 1 )
//		{
//		if ( Motor_Velocity[3] > 10 ) Vel_Limit = Speed*15;
//		}
//		else if ( Joystick == 2 )
//		{
//		if ( Motor_Velocity[3] < -10 ) Vel_Limit = Speed*15;
//		}
		
		Vel_Limit = Joystick == 0 ? 10 : Speed * 15;  		//15
		if(Joystick != 0 && Steering_Mode != 1)
		{
			Vel_Limit = 15;
			Left_Frame_Speed = 0;
		}
		
		else if (Joystick == 0 && Steering_Mode != 1)
		{
			Vel_Limit = 0;
			Left_Frame_Speed = 0;
		}
 if ((Joystick!=0)&&(Steering_Mode==ALL_WHEEL)&&(LF_Steering >=25 || RF_Steering<=-25)){	Vel_Limit = 15;}
		
		
		if ( Steering_Mode != ALL_WHEEL ){ Left_Steering_Speed = Right_Steering_Speed = 0; }
	//		Left_Frame_Speed=0;
			Left_Vel_Limit = Vel_Limit  + Left_Steering_Speed + Left_Frame_Speed +4;
			Right_Vel_Limit = Vel_Limit + Right_Steering_Speed+4;
	/////////////////////////////////////////////////////////////////////////APPLYING TORQUE/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		if ( Motor_Velocity[3] < 10 && Motor_Velocity[3] > -10 )
//		{
//		Transmit_Motor_Torque();
//		}
		
			
				// if (Joystick == 1 || Joystick == 2)
				// {
				// 	if (Joystick != Joystick_Temp)
				// 	{
				// 		Transmit_Motor_Torque();
				// 		Joystick_Temp = Joystick;
				// 		//FLAG = SET;
				// 	}
				// }
				
				// if (Joystick == 0)
				// {
				// 	//if ((Motor_Velocity[1] <= Left_Vel_Limit + 2 && Motor_Velocity[1] >= Left_Vel_Limit - 2) && (Motor_Velocity[3] <= Right_Vel_Limit + 2 && Motor_Velocity[3] >= Right_Vel_Limit - 2))
				// 	//{
				// 		Transmit_Motor_Torque();
				// 		//FLAG = NULL;
				// 	//}
				
				if (( Joystick != 0 && Prev_Joystick == 0) || ( Joystick == 0 && Prev_Joystick != 0))
				{
					if( Rover_Velocity < 12 && Rover_Velocity > -12) Transmit_Motor_Torque();
					else {}
				}
				else
				{
//					 Transmit_Motor_Torque();
//						count++;
				}
					

				// if ( Joystick == 0 && Prev_Joystick != 0)
				// {
				// 	if( Motor_Velocity[3] < 12 ) Transmit_Motor_Torque();
				// 	else {}
				// 	Prev_Joystick = Joystick;
				// }
				

			if(Left_Vel_Limit != Left_Transmit_Vel)
			{
				if(HAL_GetTick() - left_tick_count >= 50)
				{
					if(Left_Vel_Limit > Left_Transmit_Vel)
					{
						Left_Transmit_Vel++;
					}
					
					else if(Left_Vel_Limit < Left_Transmit_Vel)
					{
						//Left_Transmit_Vel--;
						
						if ( fabs(Motor_Velocity[3]) <= 20 ) Left_Transmit_Vel--;
						else if ( fabs(Motor_Velocity[3]) > 20 && fabs(Motor_Velocity[3]) <= 30 ) Left_Transmit_Vel = Left_Transmit_Vel - 3 ;
						else Left_Transmit_Vel = Left_Transmit_Vel - 5;
					}
					else{}
					
					for(uint8_t i=1 ; i <= 2 ; i++) { CAN_Transmit(i,VEL_LIMIT,Left_Transmit_Vel,4,DATA);HAL_Delay(1);}
					left_tick_count = HAL_GetTick();
				
			}
		}
			
			if(Right_Vel_Limit != Right_Transmit_Vel)
			{
				if(HAL_GetTick() - right_tick_count >= 50)
				{
					if(Right_Vel_Limit > Right_Transmit_Vel)
					{
						Right_Transmit_Vel++;
					}
					
					else if(Right_Vel_Limit < Right_Transmit_Vel)
					{
						//Right_Transmit_Vel--;
						
						if ( fabs(Motor_Velocity[3]) <= 20 ) Right_Transmit_Vel--;
						else if ( fabs(Motor_Velocity[3]) > 20 && fabs(Motor_Velocity[3]) <= 30 ) Right_Transmit_Vel = Right_Transmit_Vel - 3 ;
						else Right_Transmit_Vel = Right_Transmit_Vel - 5;
					}
					
					else{}
						
					for(uint8_t i=3 ; i <= 4 ; i++) { CAN_Transmit(i,VEL_LIMIT,Right_Transmit_Vel,4,DATA);HAL_Delay(1);}
					right_tick_count = HAL_GetTick();
				}
			}

	}	
	else 
	{
		for ( uint8_t i = 1 ; i < 5 ; i++ )
		{ 
				Set_Motor_Torque ( i , 0 ); HAL_Delay(1);
		}
	}
	
//	if ( (!BT_State ) && Joystick != 0 )
//	{
//		for ( uint8_t i = 1 ; i < 5 ; i++ ){Set_Motor_Torque ( i , NULL ); HAL_Delay(1);}
//		Joystick = 0 ;
//	}
}

void Wheel_Speeds_Calc(int Inner_Angle)
{

	
		kmph = Vel_Limit / 30;
    Steering_Angle = fabs((float)Inner_Angle); // new float
    Rover_Centre_Dist = ((WheelBase/sin(Steering_Angle*(3.14/180)))+TrackWidth)/1000;
    TimeTaken =  Rover_Centre_Dist/(kmph*0.277);
    Inner_Speed = ((Rover_Centre_Dist- 0.65)/TimeTaken)*3.6;
    Inner_Speed = KMPHtoRPS(Inner_Speed) -  Vel_Limit;
    
    Outer_Speed = ((Rover_Centre_Dist+ 0.65)/TimeTaken)*3.6;
    Outer_Speed = KMPHtoRPS(Outer_Speed) - Vel_Limit;
   
    if ( Inner_Angle <= -3 )
    {
        Left_Steering_Speed = Inner_Speed;
        Right_Steering_Speed = Outer_Speed;
    }
    else if ( Inner_Angle > 2 ) // Right Turn of the Rover
		{
				Left_Steering_Speed = Outer_Speed;
				Right_Steering_Speed = Inner_Speed;
		}
		else
		{
				Left_Steering_Speed = Right_Steering_Speed = NULL;		
		}

}

float Differintial_Angle ( double Inner_Angle_Set )
{
	double TAN=0;float theta=0;
		
		TAN = tan( (Inner_Angle_Set) / 180 * 3.14 );
		
		theta = (atan ( 900 / ((900 / TAN ) + (Track_Width*2 ))) * (180/3.14)) ; //1557
	
		return ( theta );

}

void Steering_Controls (void)
{
/*	If the Steering Reset Flag is SET, all the Steering wheels will return to their Home Position.	
		Flag Sets on Power Up and at every Steering Mode Change.																*/	
	
	if ( Steering_Reset_Flag )
	{
		/*///////////////////////////////////////////////////////////	STEERING RESET CONTROLLER /////////////////////////////////////////////////////////	*/
		if ( (LF_Steering <= STEERING_BOUNDARY ) && ( LF_Steering >= -STEERING_BOUNDARY ) ) { LF_Speed = 0;	LF_SET = SET;} 
		else {LF_Speed = ( LF_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LF_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}		
		
		if ( (LR_Steering <= STEERING_BOUNDARY ) && ( LR_Steering >= -STEERING_BOUNDARY ) ) { LR_Speed = 0;	LR_SET = SET;} 
		else {LR_Speed = ( LR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
		
		if ( (RF_Steering <= STEERING_BOUNDARY ) && ( RF_Steering >= -STEERING_BOUNDARY ) ) { RF_Speed = 0;	RF_SET = SET;} 
		else {RF_Speed = ( RF_Steering > STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : ( RF_Steering < STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : 0;}	
		
		if ( (RR_Steering <= STEERING_BOUNDARY ) && ( RR_Steering >= -STEERING_BOUNDARY ) ) { RR_Speed = 0;	RR_SET = SET;} 
		else {RR_Speed = ( RR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( RR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
		/*///////////////////////////////////////////////////////////	STEERING RESET CONTROLLER /////////////////////////////////////////////////////////	*/
		
		LF_Error = LR_Error = RF_Error = RR_Error = 1;
		if ((LF_SET) && (LR_SET) && (RF_SET) && (RR_SET) )	
		{
			LF_SET = LR_SET = RF_SET = RR_SET = NULL ;
			Steering_Reset_Flag=NULL;
		}
		else{}
	}
		
	else if( !Steering_Reset_Flag  )
	{
		switch ( Steering_Mode )
		{
			case 0:
						if ( (LF_Steering <= STEERING_BOUNDARY ) && ( LF_Steering >= -STEERING_BOUNDARY ) ) { LF_Speed = 0;	LF_SET = SET;} 
						else {LF_Speed = ( LF_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LF_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}		
						
						if ( (LR_Steering <= STEERING_BOUNDARY ) && ( LR_Steering >= -STEERING_BOUNDARY ) ) { LR_Speed = 0;	LR_SET = SET;} 
						else {LR_Speed = ( LR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
						
						if ( (RF_Steering <= STEERING_BOUNDARY ) && ( RF_Steering >= -STEERING_BOUNDARY ) ) { RF_Speed = 0;	RF_SET = SET;} 
						else {RF_Speed = ( RF_Steering > STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : ( RF_Steering < STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : 0;}	
						
						if ( (RR_Steering <= STEERING_BOUNDARY ) && ( RR_Steering >= -STEERING_BOUNDARY ) ) { RR_Speed = 0;	RR_SET = SET;} 
						else {RR_Speed = ( RR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( RR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
						break;
			case ALL_WHEEL :	 //	 --> ALL WHEEL STEERING  
							/*///////////////////////////////////////////////////////////////////////////////////	ALL WHEEL STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////	*/
							
							Inner_Angle =	(( Pot_Angle / 2 ) - 45);
							
							if ( Inner_Angle <= -1 )  // Left Turn of the Rover
							{
								LF_Error = (-Inner_Angle - (LF_Steering)) ;						LF_Speed = LF_Error * STEERING_KP;
								LR_Error = (Inner_Angle - (LR_Steering)) ; 						LR_Speed = LR_Error * STEERING_KP;							
												
								Outer_Angle = Differintial_Angle((-Inner_Angle ))	;		RF_Error = (-Outer_Angle - (-RF_Steering)) ;RF_Speed = RF_Error * STEERING_KP;									
								Outer_Angle_2	= Differintial_Angle((-Inner_Angle ))	;	RR_Error = (-Outer_Angle - (RR_Steering)) ; RR_Speed = RR_Error * STEERING_KP;
							
								Prev_Inner_Angle = Inner_Angle;
							}

							else if ( Inner_Angle >= 1 ) // Right Turn of the Rover   //0
							{
								RF_Error = (Inner_Angle - (-RF_Steering)) ;						RF_Speed = RF_Error * STEERING_KP;	
								RR_Error = (Inner_Angle - (RR_Steering)) ; 						RR_Speed = RR_Error * STEERING_KP;	
								
								Outer_Angle = Differintial_Angle((Inner_Angle ))	;		LF_Error = (-Outer_Angle- (LF_Steering)) ;LF_Speed = LF_Error * STEERING_KP;	
								Outer_Angle_2	= Differintial_Angle((Inner_Angle ));		LR_Error = (Outer_Angle_2 - (LR_Steering)) ; LR_Speed = LR_Error * STEERING_KP;
									
								Prev_Inner_Angle = Inner_Angle;
							}	
							
							else // Home Pos of the Rover
							{								
								Steering_Reset_Flag = SET;
								LF_Error = LR_Error = RF_Error = RR_Error = 1;
							}
					
							Wheel_Speeds_Calc(Inner_Angle);
							break;
			/*///////////////////////////////////////////////////////////////////////////////////	ALL WHEEL STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////	*/				
			case CRAB :							//	--> CRAB STEERING			
			/*///////////////////////////////////////////////////////////////////////////////////	CRAB STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////////	*/
			
							AW_Angle = ( Pot_Angle - 90) ; 
			
							LF_Speed = (LF_Steering > AW_Angle -STEERING_BOUNDARY && LF_Steering < AW_Angle +STEERING_BOUNDARY ) ? 0 : ( LF_Steering < AW_Angle ) ? STEERING_HOMING_SPEED: ( LF_Steering > AW_Angle ) ? -STEERING_HOMING_SPEED : 0;		
							LR_Speed = (LR_Steering > AW_Angle -STEERING_BOUNDARY && LR_Steering < AW_Angle +STEERING_BOUNDARY ) ? 0 : ( LR_Steering < AW_Angle ) ? STEERING_HOMING_SPEED: ( LR_Steering > AW_Angle ) ? -STEERING_HOMING_SPEED : 0;					
							RF_Speed = (RF_Steering > AW_Angle -STEERING_BOUNDARY && RF_Steering < AW_Angle +STEERING_BOUNDARY ) ? 0 : ( RF_Steering < AW_Angle-STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED: ( RF_Steering > AW_Angle+STEERING_BOUNDARY ) ?  STEERING_HOMING_SPEED : 0;
							RR_Speed = (RR_Steering > AW_Angle -STEERING_BOUNDARY && RR_Steering < AW_Angle +STEERING_BOUNDARY ) ? 0 : ( RR_Steering < AW_Angle ) ? STEERING_HOMING_SPEED: ( RR_Steering > AW_Angle ) ? -STEERING_HOMING_SPEED : 0;
							break;
				/*///////////////////////////////////////////////////////////////////////////////////	CRAB STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////////	*/	

				case ZERO_TURN :
				/*///////////////////////////////////////////////////////////////////////////////////	ZERO TURN - STEERING FUNCTION ////////////////////////////////////////////////////////////////////////////////////////	*/
							AW_Angle = Zero_Turn_Angle ; 
			
							LF_Speed = (LF_Steering > -AW_Angle -STEERING_BOUNDARY && LF_Steering < -AW_Angle +STEERING_BOUNDARY ) ? 0  : ( LF_Steering < -AW_Angle ) ? STEERING_HOMING_SPEED: ( LF_Steering > -AW_Angle ) ? -STEERING_HOMING_SPEED :  0;		
							LR_Speed = (LR_Steering > AW_Angle -STEERING_BOUNDARY  && LR_Steering < AW_Angle +STEERING_BOUNDARY )  ? 0  : ( LR_Steering < AW_Angle )  ? STEERING_HOMING_SPEED: ( LR_Steering > AW_Angle )  ? -STEERING_HOMING_SPEED :  0;
							RF_Speed = (RF_Steering > AW_Angle -STEERING_BOUNDARY  && RF_Steering < AW_Angle +STEERING_BOUNDARY )  ? 0  : ( RF_Steering < AW_Angle )  ? -STEERING_HOMING_SPEED: ( RF_Steering > AW_Angle )  ? STEERING_HOMING_SPEED :  0;		
							RR_Speed = (RR_Steering > -AW_Angle -STEERING_BOUNDARY && RR_Steering < -AW_Angle +STEERING_BOUNDARY ) ? 0  : ( RR_Steering < -AW_Angle ) ? STEERING_HOMING_SPEED: ( RR_Steering > -AW_Angle ) ? -STEERING_HOMING_SPEED	:  0;

							break;
				/*///////////////////////////////////////////////////////////////////////////////////	ZERO TURN - STEERING FUNCTION ////////////////////////////////////////////////////////////////////////////////////////	*/
	/*			case FRONT_WHEEL :	 //	 --> FRONT WHEEL STEERING  */
							/*///////////////////////////////////////////////////////////////////////////////////	ALL WHEEL STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////	*/
			/*				Inner_Angle =	( Pot_Angle / 2 ) - 45;
								
							if ( Inner_Angle <= -1 )  
							{
							
							LF_Error = (-Inner_Angle - (LF_Steering)) ;LF_Speed = LF_Error * STEERING_KP;
							Outer_Angle = FWD_Differintial_Angle((-Inner_Angle ))	;	Flash_Factor = Inner_Angle / Outer_Angle; Outer_Steering_Speed = Inner_Steering_Speed / Flash_Factor;
							RF_Error = (-Outer_Angle - (-RF_Steering)) ;RF_Speed = RF_Error * STEERING_KP;									
							Outer_Angle_2	= FWD_Differintial_Angle((-Inner_Angle ))	;					
							Prev_Inner_Angle = Inner_Angle;
							}
							
							
							else if ( Inner_Angle >= 0 ) 
							{
							RF_Error = (Inner_Angle - (-RF_Steering)) ;RF_Speed = RF_Error * STEERING_KP;	
							Outer_Angle = FWD_Differintial_Angle((Inner_Angle ))	;	Flash_Factor = Inner_Angle / Outer_Angle; Outer_Steering_Speed = Inner_Steering_Speed / Flash_Factor;
							Outer_Angle_2	= FWD_Differintial_Angle((Inner_Angle ))	;		
							LF_Error = (-Outer_Angle- (LF_Steering)) ;LF_Speed = LF_Error * STEERING_KP;
							Prev_Inner_Angle = Inner_Angle;
							
							}	
							
							else 
							{								
								Steering_Reset_Flag = SET;
								LF_Error = LR_Error = RF_Error = RR_Error = 1;
							}
							
							if ( (LR_Steering <= STEERING_BOUNDARY ) && ( LR_Steering >= -STEERING_BOUNDARY ) ) { LR_Speed = 0;	LR_SET = SET;} 
							else {LR_Speed = ( LR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
							
							if ( (RR_Steering <= STEERING_BOUNDARY ) && ( RR_Steering >= -STEERING_BOUNDARY ) ) { RR_Speed = 0;	RR_SET = SET;} 
							else {RR_Speed = ( RR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( RR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	

							break;*/
							
				case WIDTH_EXTEND: 
					
							LS_Angle = WIDE_ANGLE; 
							RS_Angle =-WIDE_ANGLE; 
							LF_Speed = (LF_Steering > LS_Angle -STEERING_BOUNDARY && LF_Steering < LS_Angle +STEERING_BOUNDARY ) ? 0 : ( LF_Steering < LS_Angle ) ? STEERING_HOMING_SPEED: ( LF_Steering > LS_Angle ) ? -STEERING_HOMING_SPEED : 0;		
							LR_Speed = (LR_Steering > LS_Angle -STEERING_BOUNDARY && LR_Steering < LS_Angle +STEERING_BOUNDARY ) ? 0 : ( LR_Steering < LS_Angle ) ? STEERING_HOMING_SPEED: ( LR_Steering > LS_Angle ) ? -STEERING_HOMING_SPEED : 0;
							RF_Speed= RR_Speed=0;			
							Angle_Ready = ((LF_Steering > LS_Angle -STEERING_BOUNDARY && LF_Steering < LS_Angle +STEERING_BOUNDARY )&& (RF_Steering > RS_Angle -STEERING_BOUNDARY && RF_Steering < RS_Angle +STEERING_BOUNDARY ))? SET: NULL;
							Angle_Ready = ((LF_Steering > LS_Angle -STEERING_BOUNDARY && LF_Steering < LS_Angle +STEERING_BOUNDARY ))? SET: NULL;
											
							break;
				
				case WIDTH_SHRINK: 
								
							LS_Angle = - SHRINK_ANGLE; 
							RS_Angle = SHRINK_ANGLE; 
							LF_Speed = (LF_Steering > LS_Angle -STEERING_BOUNDARY && LF_Steering < LS_Angle +STEERING_BOUNDARY ) ? 0 : ( LF_Steering < LS_Angle ) ? STEERING_HOMING_SPEED: ( LF_Steering > LS_Angle ) ? -STEERING_HOMING_SPEED : 0;
							LR_Speed = (LR_Steering > LS_Angle -STEERING_BOUNDARY && LR_Steering < LS_Angle +STEERING_BOUNDARY ) ? 0 : ( LR_Steering < LS_Angle ) ? STEERING_HOMING_SPEED: ( LR_Steering > LS_Angle ) ? -STEERING_HOMING_SPEED : 0;
							RF_Speed= RR_Speed=0;			
							Angle_Ready = ((LF_Steering > LS_Angle -STEERING_BOUNDARY && LF_Steering < LS_Angle +STEERING_BOUNDARY ))? SET: NULL;			
							break;
										
			default :  break;
		}
	}
					
			/*----------------------------- STEERING VELOCITY CONTROLLER --------------------------------*/
	LF_Speed = LF_Speed > 0 && LF_Steering >= 90 ?  0 : LF_Speed < 0 && LF_Steering <= -90 ? 0 : LF_Speed;
	LR_Speed = LR_Speed > 0 && LR_Steering >= 90 ?  0 : LR_Speed < 0 && LR_Steering <= -90 ? 0 : LR_Speed;
	RF_Speed = RF_Speed < 0 && RF_Steering >= 90 ?  0 : RF_Speed > 0 && RF_Steering <= -90 ? 0 : RF_Speed;
	RR_Speed = RR_Speed > 0 && RR_Steering >= 90 ?  0 : RR_Speed < 0 && RR_Steering <= -90 ? 0 : RR_Speed;
	
			if ( LF_Speed_Temp != LF_Speed )
			{
				LF_Speed= LF_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : LF_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : LF_Speed; 
				Set_Motor_Velocity( LFS , -LF_Speed );
				LF_Speed_Temp = LF_Speed ;
			}
			
			if ( LR_Speed_Temp != LR_Speed )
			{
				LR_Speed= LR_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : LR_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : LR_Speed;
				 Set_Motor_Velocity( LRS , -LR_Speed );
				LR_Speed_Temp = LR_Speed ;
			}
						
			if ( RF_Speed_Temp != RF_Speed )
			{	
				RF_Speed= RF_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : RF_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : RF_Speed;
				Set_Motor_Velocity( RFS , -RF_Speed );
				RF_Speed_Temp = RF_Speed ;
			}
										
			if ( RR_Speed_Temp != RR_Speed )
			{
				RR_Speed= RR_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : RR_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : RR_Speed;
				Set_Motor_Velocity( RRS , -RR_Speed );
				RR_Speed_Temp = RR_Speed ;
			}	
			/*----------------------------- STEERING VELOCITY CONTROLLER --------------------------------*/
}

void Battery_Status_Indication(void)
{BT_State = BT_READ ;

		if ( Rover_Voltage >= 46 ) Volt_Tx = 1;
		else Volt_Tx = 0;
		
		if (( Volt_Tx_Temp != Volt_Tx) || ( Prev_BT_State != BT_State))
		{
		HAL_UART_Transmit_IT(&huart5,&Volt_Tx,sizeof(Volt_Tx));
		Volt_Tx_Temp = Volt_Tx;
		Prev_BT_State = BT_State;
		}
		

}
void Operations_Monitor(void)
{
    MOTORS_STOP_FLAG = SET;
	if (HAL_GetTick() - Tick_Count1 >= 1500)
	{
		//if (BT_State == NULL) { JOYSTICK_STATE_FLAG = NULL;}
		
		for (uint8_t i = 1; i < 27; i++)
		{
			if(i!=5 && i!=17&& i!=18&& i!=19&&i!=20 && i!=6)
			{
			//if (Axis_State[i] != 8) { AXIS_STATE_FLAG = NULL; }
			
			if (Node_Id[i] == Node_Id_Temp[i]) { HEARTBEAT_FLAG = NULL; }
			Node_Id_Temp[i] = Node_Id[i];}
			
		//	if (FET_Temperature[i] > 95) { FET_TEMP_FLAG = NULL; }
		}
		
		for (uint8_t i = 1; i < 17; i++)
		{
			if(i!=5 && i!=17&& i!=18&& i!=19&&i!=20 && i!=6){if (Axis_State[i] != 8) { AXIS_STATE_FLAG = NULL; }
		}
	}
		if((Right_Vertical_Motor_Count>=200 && Right_Vertical_Motor_Count<=210)||(Right_Vertical_Motor_Count>=210)||(Right_Vertical_Motor_Count<=-200 && Right_Vertical_Motor_Count>=-210) || (Right_Vertical_Motor_Count<=-210) ) Vertical_Limit=NULL;
	if((Contour_Motor_Count>=150 && Contour_Motor_Count<=160)||(Contour_Motor_Count>=160) || (Contour_Motor_Count<=-150 && Contour_Motor_Count>=-160)||(Contour_Motor_Count<=-150)) Contour_Limit=NULL;
		OPERATION_MONITOR_FLAG = AXIS_STATE_FLAG == SET && HEARTBEAT_FLAG == SET && Contour_Limit==SET && Vertical_Limit==SET ? SET : NULL;
		
		Tick_Count1 = HAL_GetTick();
	}
}

void Emergency_Stop(void)
{
	BUZZER_ON;
	
	if (MOTORS_STOP_FLAG)
	{
		for (uint8_t i = 1; i < 5; i++) {Set_Motor_Torque(i , 0);}
		for (uint8_t i = 6; i < 17; i++){Set_Motor_Velocity(i, 0);}
		MOTORS_STOP_FLAG = NULL;      
	}
	
	if (HAL_GetTick() - Tick_Count2 >= 1500)
	{
//		if (JOYSTICK_STATE_FLAG == NULL)
//		{
//			if (BT_State == 1){JOYSTICK_STATE_FLAG = SET;}
//		}
		
		if (AXIS_STATE_FLAG == NULL)
		{
			for (uint8_t i = 1; i < 17; i++)
				{
					
					if(i!=5 && i!=17&& i!=18&& i!=19 &&i!=20 && i!= 6){
						while (Axis_State[i] != 8)
					{
						Reboot(i);
						HAL_Delay(2000);
					}
					}
				
		}
				AXIS_STATE_FLAG = SET;
	}
		
		if (HEARTBEAT_FLAG == NULL)
		{
			for (uint8_t i = 1; i < 27; i++)
			{if(i!=5 && i!=17 && i!=18&& i!=19&&i!=20 && i!=6){
					while (Node_Id[i] == Node_Id_Temp[i]){ //Node++;
					}
						Node++;
					}
			//Node_Id_Temp[i] = Node_Id[i];
			}
			HEARTBEAT_FLAG = SET;
			
		}
		Node = 0;
//		if (FET_TEMP_FLAG == NULL)
//		{
//			for (uint8_t i = 1; i < 21; i++)
//			{
//				if (FET_Temperature[i] > 95){ fet++;}
//			}
//			if (fet == 0){FET_TEMP_FLAG = SET;}
//			fet = 0;
//		}
		
		OPERATION_MONITOR_FLAG =  AXIS_STATE_FLAG == SET && HEARTBEAT_FLAG == SET && Vertical_Limit==SET && Contour_Limit==SET? SET : NULL;
		if (OPERATION_MONITOR_FLAG) {BUZZER_OFF;}
		
		Tick_Count2 = HAL_GetTick();
	}
	
	
}


void Frame_Controls(void)
{	
/*-----------------------------PID CONTROL-------------------------------------*/
  if (!Left_IMU_State )
	{
	L_Vert_Speed = R_Vert_Speed = Contour_Speed = 0;
	//BUZZER_ON;
	}
	else if ( Left_IMU_State ) 


	{	// ADD ZERO ERROR CLR FLAG AND HEARTBEAT OK FLAG	
	/*	L_R_Err =  Left_Roll_Pos - L_Roll ;                                               										// L roll error = Target value(0.68) - current value. 
		
		L_Vert_Speed = Left_Verticality_PID ( L_R_Err , NULL );         																			// L Vertical speed from left verticality pid function. 

		L_Vert_Speed = (( L_Vert_Speed <= 3 ) && ( L_Vert_Speed >= -3 ) ) ? 0 : L_Vert_Speed;                 // Assigning 0 to L Vertical Speed if it is between - 2 to 2 (to avoid oscillations)

		Left_Error_Flag =( L_Vert_Speed == 0 ) ? NULL : SET;																									// (CHECK) for basic testing. to set once the vertical speed is zero(correction completed) ISSUE
	
*/
			
	
		R_R_Err =  Right_Pitch_Pos - R_Pitch 	;																																	// R roll error = Target value(-3.0625) - current value.
	
		R_Vert_Speed = Right_Verticality_PID ( R_R_Err , NULL );																							// R Vertical speed from right verticality pid function. 
	
	  R_Vert_Speed = (( R_Vert_Speed <= 2 ) && ( R_Vert_Speed >= -2 ) ) ? 0 : R_Vert_Speed;									// Assigning 0 to R Vertical Speed if it is between - 2 to 2 (to avoid oscillations)

		Right_Error_Flag =( R_Vert_Speed == 0 ) ? NULL : SET;																									// (CHECK) for basic testing. to set once the vertical speed is zero(correction completed) ISSUE


		//Contour_Avg =	R_Pitch ;																																						
		
		C_Err =   Right_Roll_Pos - R_Roll ;																															// (CHECK) Contour error = Target value(2.5) - current value.
	
	//	if ( C_Err > 1 || C_Err < -1)  // Error Boundary
	//	{
		Contour_Speed = Contour_PID( C_Err , NULL );																													// Contour speed from contour pid function.
		
		Contour_Speed = (( Contour_Speed <= 2 ) && ( Contour_Speed >= -2) ) ? 0 : Contour_Speed;							// Assigning 0 to Contour Speed if it is between - 1 to 1 (to avoid oscillations)

		Contour_Error_Flag = ( Contour_Speed == 0 ) ? NULL : SET;																							// (CHECK) for basic testing. to set once the vertical speed is zero(correction completed) ISSUE
	//	}
	//	else Contour_Speed = 0;
	
			
		FRAME_NO_ERROR_FLAG = ( !Left_Error_Flag && !Right_Error_Flag && !Contour_Error_Flag ) ? NULL : SET;		// (CHECK) for basic testing. 

/*-----------------------------PID CONTROL-------------------------------------*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*-----------------------------CHECK FOR LIMITS AND SET VELOCITIES TO MOTOR-------------------------------------*/
		
//		if ( L_Vert_Speed > 0 && Left_Vertical < -FRAME_LIMIT) L_Vert_Speed = 0;
//		if ( L_Vert_Speed < 0 && Left_Vertical > 	FRAME_LIMIT) L_Vert_Speed = 0;
		
		if( L_Vert_Speed_Temp != L_Vert_Speed ) 																															// checking if the new value is not equal to old value
		{
			//Set_Motor_Velocity (LVert , L_Vert_Speed );
			L_Vert_Speed_Temp = L_Vert_Speed ;																																// Overwriting old value with new value. 
		}		
			
		//R_Vert_Speed = R_Vert_Speed > 0 && Right_Vertical_Motor_Count >= 550 ? 0 : R_Vert_Speed < 0 && Right_Vertical_Motor_Count <= -550 ? 0 : R_Vert_Speed ;
	  // 	Right_Vertical_On_Limit = Right_Vertical_Motor_Count >= 550 ||  Right_Vertical_Motor_Count <= -550 ? SET : NULL;
		if( R_Vert_Speed_Temp != R_Vert_Speed ) 																															// checking if the new value is not equal to old value
		{
			Set_Motor_Velocity (RVert , -R_Vert_Speed );	
			R_Vert_Speed_Temp = R_Vert_Speed ;																																// Overwriting old value with new value.
		} 
	//	Contour_Speed = Contour_Speed > 0 && Contour_Motor_Count >= 550 ? 0 : Contour_Speed < 0 && Contour_Motor_Count <= -550 ? 0 : Contour_Speed ;
	//	Contour_On_Limit = Contour_Motor_Count >= 550 ||  Contour_Motor_Count <= -550 ? SET : NULL;		
		if( Contour_Speed_Temp != Contour_Speed ) 																														// checking if the new value is not equal to old value
		{
			Set_Motor_Velocity (Contour , Contour_Speed );
			Contour_Speed_Temp = Contour_Speed ;																															// Overwriting old value with new value.
		}		
		
		Frame_Buzz_Switch = ( Right_Vertical_On_Limit == 1 ) || (Contour_On_Limit == 1) ?  1 : 0;

			/*-----------------------------CHECK FOR LIMITS AND SET VELOCITIES TO MOTOR-------------------------------------*/				
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////						
}
}	


float Right_Verticality_PID ( float Right_Roll_Value , unsigned long long 	R_Time_Stamp )
{
		//dt = Time_Stamp - time;

					R_Error_Change = Right_Roll_Value - R_Prev_Error;
					R_Error_Slope = R_Error_Change / dt;
					R_Error_Area = R_Error_Area + ( R_Error_Change * dt ) ;
			
				
				
			R_P = R_Kp * Right_Roll_Value;
			 
			R_I	= R_Ki * R_Error_Area;						 R_I = R_I > Anti_Windup_Limit ? Anti_Windup_Limit : R_I < -Anti_Windup_Limit ? -Anti_Windup_Limit : R_I ;	

			R_D = R_Kd * R_Error_Slope; 
				
			
			
				Right_Out = R_P + R_I + R_D ;
		
			
			//	Right_Out = (Right_Roll_Value < V_BOUNDARY && Right_Roll_Value	> -V_BOUNDARY) ?	0: Right_Out;

				Right_Out = Right_Out > V_LIMIT ? V_LIMIT : Right_Out < -V_LIMIT ? -V_LIMIT : Right_Out;

		
			R_Prev_Error = Right_Roll_Value;
		//	time = Time_Stamp;
			
			return Right_Out;

}

float Contour_PID ( float Contour_Val , unsigned long long 	C_Time_Stamp )
{
		//dt = Time_Stamp - time;

					C_Error_Change = Contour_Val - C_Prev_Error;
					C_Error_Slope = C_Error_Change / dt;
					C_Error_Area = C_Error_Area + ( C_Error_Change * dt ) ;
			
				
			C_P = C_Kp * Contour_Val;
			 
			C_I	= C_Ki * C_Error_Area;						 C_I = C_I > Anti_Windup_Limit ? Anti_Windup_Limit : C_I < -Anti_Windup_Limit ? -Anti_Windup_Limit : C_I ;	

			C_D = C_Kd * C_Error_Slope; 
				
			
			
				Contour_Out = C_P + C_I + C_D ;
			

				Contour_Out = Contour_Out > C_LIMIT ? C_LIMIT : Contour_Out < -C_LIMIT ? -C_LIMIT : Contour_Out;

		
			C_Prev_Error = Contour_Val;
		//	time = Time_Stamp;
			
			return Contour_Out;

}

float Left_Frame_PID ( float Left_Error_Value , unsigned long long 	L_Time_Stamp )
{	
	

					LF_Error_Change = Left_Error_Value - LF_Prev_Error;
					LF_Error_Slope = LF_Error_Change / dt;
					LF_Error_Area = LF_Error_Area + ( LF_Error_Change * dt ) ;
			
				
				
			LF_P = LF_Kp * Left_Error_Value;
			 
			LF_I	= LF_Ki * LF_Error_Area;						 LF_I = LF_I > Anti_Windup_Limit ? Anti_Windup_Limit : LF_I < -Anti_Windup_Limit ? -Anti_Windup_Limit : LF_I ;	

			LF_D = LF_Kd * LF_Error_Slope; 
				
			
			
				Left_Frame_Out = LF_P + LF_I + LF_D ;
		
			
			//	Right_Out = (Right_Roll_Value < V_BOUNDARY && Right_Roll_Value	> -V_BOUNDARY) ?	0: Right_Out;

				Left_Frame_Out = Left_Frame_Out > 10 ? 10 : Left_Frame_Out < -10 ? -10 : Left_Frame_Out;

		
			LF_Prev_Error = Left_Error_Value;
		//	time = Time_Stamp;
			
			return Left_Frame_Out;

}

float Left_Arm_PID ( float Left_Flap_Error , unsigned long long 	L_Time_Stamp )
{
		//dt = Time_Stamp - time;
	


			L_Error_Change = Left_Flap_Error - L_Prev_Error;
			L_Error_Slope  = L_Error_Change / dt;
			L_Error_Area   = L_Error_Area + ( L_Error_Change * dt ) ;
			
				
				
			L_P = L_Kp * Left_Flap_Error;
			 
			L_I	= L_Ki * L_Error_Area;						 L_I = L_I > Anti_Windup_Limit ? Anti_Windup_Limit : L_I < -Anti_Windup_Limit ? -Anti_Windup_Limit : L_I ;	

			L_D = L_Kd * L_Error_Slope; 
				
			
			
			Left_Out = L_P + L_I + L_D ;

			Left_Out = Left_Out > Arm_Max_Speed ? Arm_Max_Speed : Left_Out < -Arm_Max_Speed ? -Arm_Max_Speed : Left_Out;

			L_Prev_Error = Left_Flap_Error;
			
			return Left_Out;

}
float Right_Arm_PID ( float Right_Flap_Error , unsigned long long 	RA_Time_Stamp )
{
		//dt = Time_Stamp - time;
	


			RA_Error_Change = Right_Flap_Error - RA_Prev_Error;
			RA_Error_Slope  = RA_Error_Change / dt;
			RA_Error_Area   = RA_Error_Area + ( RA_Error_Change * dt ) ;
			
				
				
			RA_P = RA_Kp * Right_Flap_Error;
			 
			RA_I	= RA_Ki * RA_Error_Area;						 RA_I = RA_I > Anti_Windup_Limit ? Anti_Windup_Limit : RA_I < -Anti_Windup_Limit ? -Anti_Windup_Limit : RA_I ;	

			RA_D = RA_Kd * RA_Error_Slope; 
				
			
			
			RightArm_Out = RA_P + RA_I + RA_D ;

			RightArm_Out = RightArm_Out > Arm_Max_Speed ? Arm_Max_Speed : RightArm_Out < -Arm_Max_Speed ? -Arm_Max_Speed : RightArm_Out;

			RA_Prev_Error = Right_Flap_Error;
			
			return RightArm_Out;

}
float Pitch_Arm_PID ( float Pitch_Error , unsigned long long 	R_Time_Stamp )
{
		//dt = Time_Stamp - time;
	


			P_Error_Change = Pitch_Error - P_Prev_Error;
			P_Error_Slope  = P_Error_Change / dt;
			P_Error_Area   = P_Error_Area + ( P_Error_Change * dt ) ;
			
				
				
			P_P = P_Kp * Pitch_Error;
			 
			P_I	= P_Ki * P_Error_Area;						 P_I = P_I > Anti_Windup_Limit ? Anti_Windup_Limit : P_I < -Anti_Windup_Limit ? -Anti_Windup_Limit : P_I ;	

			P_D = P_Kd * P_Error_Slope; 
				
			
			
			Pitch_Out = P_P + P_I + P_D ;

			Pitch_Out = Pitch_Out > Arm_Max_Speed ? Arm_Max_Speed : Pitch_Out < -Arm_Max_Speed ? -Arm_Max_Speed : Pitch_Out;

			P_Prev_Error = Pitch_Error;
			
			return Pitch_Out;

}
void EEPROM_Store_Data (void)
{
	Lower_Width_Motor_Count = Lower_Width_Motor_Value + Absolute_Position_Int[15];
	Upper_Width_Motor_Count = Upper_Width_Motor_Value + Absolute_Position_Int[16];
	memcpy(&Write_Value[0], &Start_Value, sizeof(Start_Value));
	memcpy(&Write_Value[32], &Start_Value, sizeof(End_Value));
	memcpy(&Write_Value[28], &Lower_Width_Motor_Count, sizeof(Lower_Width_Motor_Count));
	memcpy(&Write_Value[4], &Upper_Width_Motor_Count, sizeof(Upper_Width_Motor_Count));
	
	Left_Arm_Motor_Count  = Left_Arm_Motor_Value + Absolute_Position_Int[12];
	Right_Arm_Motor_Count = Right_Arm_Motor_Value + Absolute_Position_Int[13];
	Pitch_Arm_Motor_Count = Pitch_Arm_Motor_Value + Absolute_Position_Int[14];	
	memcpy(&Write_Value[8], &Left_Arm_Motor_Count, sizeof(Left_Arm_Motor_Count));
	memcpy(&Write_Value[12], &Right_Arm_Motor_Count, sizeof(Right_Arm_Motor_Count));
	memcpy(&Write_Value[16], &Pitch_Arm_Motor_Count, sizeof(Pitch_Arm_Motor_Count));
	
	Right_Vertical_Motor_Count = Right_Vertical_Motor_Value + Absolute_Position_Int[6];
	Contour_Motor_Count = Contour_Motor_Value + Absolute_Position_Int[7];	
	memcpy(&Write_Value[20], &Right_Vertical_Motor_Count, sizeof(Right_Vertical_Motor_Count));
	memcpy(&Write_Value[24], &Contour_Motor_Count, sizeof(Contour_Motor_Count));
	
	
		for(uint8_t i =0; i <= 35; i++)
		{
			if(Prev_Write_Value[i] != Write_Value[i])
			{
				Store_Data= 1;
				Prev_Write_Value[i] = Write_Value[i];
			}
			else Store_Data = 0;
		}
		
		
		if ( Store_Data)
		{
		EEPROM_Write(60, 0, (uint8_t *)Write_Value, sizeof(Write_Value)); //HAL_Delay(10);
//		EEPROM_Write(6, 0, (uint8_t *)Write_Value, sizeof(Write_Value)); 
		}
		else {}
Left_Arm_Current_Pos = Left_Arm_Motor_Count;
	Right_Arm_Current_Pos = Right_Arm_Motor_Count;
	Pitch_Arm_Current_Pos = Pitch_Arm_Motor_Count;

	L_Arm_Travel = Left_Arm_Motor_Count * 3.32;
	R_Arm_Travel = Right_Arm_Motor_Count * 3.32;
}

void Frame_Synchronization(void)
{  
      Left_Vertical_Error =  Left_Pitch_Pos - L_Pitch ; //L_R_Err = x;
      
      Current_Vel_Limit = Vel_Limit ;
      
      if ( Left_Vertical_Error > 3 )  Modified_Vel_Limit = Joystick == 2 ? Current_Vel_Limit - 10 : Current_Vel_Limit + 10  ;//(Left_Vertical_Error * Width_Kp) ;
    
      else if ( Left_Vertical_Error < -3 )  Modified_Vel_Limit = Joystick == 2 ? Current_Vel_Limit + 10 : Current_Vel_Limit - 10;//(Left_Vertical_Error * Width_Kp) ;
    
      else Modified_Vel_Limit = Current_Vel_Limit ;
  
      if ( Modified_Vel_Limit != Modified_Vel_Limit_Temp )
      {
        if( Modified_Vel_Limit > Current_Vel_Limit )   
        {
				 for ( uint8_t v = Current_Vel_Limit+3; v <= Modified_Vel_Limit  ; v=v+3 )
				 {  
					 for(uint8_t i=1 ; i < 3 ; i++) 
					 {
						 CAN_Transmit(i,VEL_LIMIT,v,4,DATA);
					 }
						HAL_Delay(5);
					}    
				 Prev_Mod = Modified_Vel_Limit;  
        }
      else if( Modified_Vel_Limit < Current_Vel_Limit )
      {
			 for ( uint8_t v = Current_Vel_Limit-3; v >= Modified_Vel_Limit  ; v=v-3 )
			 {  
				for(uint8_t i=1 ; i < 3 ; i++) 
				{
				  CAN_Transmit(i,VEL_LIMIT,v,4,DATA);
				}
				HAL_Delay(5);
			 }
			 Prev_Mod = Modified_Vel_Limit;
      }
      else 
      {
        if ( Prev_Mod > Current_Vel_Limit )
        {          
					for ( uint8_t v = Prev_Mod-1; v >= Modified_Vel_Limit  ; v=v-1 )
					{  
					 for(uint8_t i=1 ; i < 3 ; i++) 
					 {
						CAN_Transmit(i,VEL_LIMIT,Current_Vel_Limit,4,DATA);
					 }
					 HAL_Delay(5); 
					}
        }
        else if ( Prev_Mod < Current_Vel_Limit )
        {          
					for ( uint8_t v = Prev_Mod+1; v <= Modified_Vel_Limit  ; v=v+1 )
					 {  
						 for(uint8_t i=1 ; i < 3 ; i++) 
						 {
							CAN_Transmit(i,VEL_LIMIT,Current_Vel_Limit,4,DATA);
						 }
					  HAL_Delay(5);
					 }
        }
        
        else {}
          Prev_Mod = Current_Vel_Limit;   
      }
      Modified_Vel_Limit_Temp = Modified_Vel_Limit ; 
      }

  else { }
}

void Left_Frame_Controls (void)
 {
		 
	 if(Joystick == 2)
	 {
		 Left_Vertical_Error =  Left_Pitch_Pos - L_Pitch ;
			Left_Frame_Speed=Left_Frame_PID(Left_Vertical_Error,0);
		 Left_Frame_Speed = Left_Frame_Speed < 3 && Left_Frame_Speed > -3 ? 0 : Left_Frame_Speed;
	 }
	 else
	 {
		 Left_Frame_Speed = 0;
	 }
		//Left_Frame_Speed = Left_Frame_Speed < 2 && Left_Frame_Speed > -2 ? 0 : Left_Frame_Speed;
	//Left_Frame_Speed = abs(Left_Frame_Speed);
	//Left_Frame_Speed = Joystick == 1 ? -Left_Frame_Speed: Joystick == 2 ? Left_Frame_Speed : 0;
//		if ( Left_Vertical_Error >= 2 )  Left_Frame_Speed = Joystick == 2 ? 15: -15  ;//(Left_Vertical_Error * Width_Kp) ;
//	
//		else if ( Left_Vertical_Error <= -2 )  Left_Frame_Speed = Joystick == 2 ? -15 : 15;//(Left_Vertical_Error * Width_Kp) ;
//	
//		else Left_Frame_Speed = 0 ;


}
void Top_Flap_Sensing(void)
{
  Flap_Angle = Flap_Data[7];
	Front_Left_Bush  =  Flap_Angle > 20 ? 1 : 0;
	
	if ( Mode == 1)
	{
		L_Arm_Speed 		 = Left_Arm_Motor_Count <= 1 && Left_Arm_Motor_Count  >=-1 ? 0 : Left_Arm_Motor_Count < -1 ? 10 : -10;
		R_Arm_Speed 		 = Right_Arm_Motor_Count<= 1 && Right_Arm_Motor_Count >= -1 ? 0 : Right_Arm_Motor_Count < -1 ? 10 : -10;
		Pitch_Arm_Speed  = Pitch_Arm_Motor_Count <= 1 && Pitch_Arm_Motor_Count >= -1 ? 0 : Pitch_Arm_Motor_Count < -1 ? 10 : -10;
		//Tri_Arm_Speed =  -ARM_HOMING_SPEED ; 
		
		First_Sense = 0;
	}
	else if (Mode ==2)
	{
		if ( Front_Left_Bush ) First_Sense = SET;
		
		if (!First_Sense)
		{
			Tri_Arm_Speed =  ARM_HOMING_SPEED ; 
		}
		else
		{
			if ( Front_Left_Bush )
			{
				Flap_Error = Flaps_Target - Flap_Angle ;
				//Tri_Arm_Speed = Flap_Error < 2 && Flap_Error > -2 ? 0 : Flap_Error * Flap_Kp;
				Tri_Arm_Speed = Flap_Error * Flap_Kp;
			}
			else Tri_Arm_Speed = 0;
		}
		
		Tri_Arm_Speed = Tri_Arm_Speed > 30 ? 30 : Tri_Arm_Speed < -30 ? -30 : Tri_Arm_Speed;
		
		

		//L_Arm_Speed = R_Arm_Speed = Pitch_Arm_Speed = 0;
//		L_Arm_Speed 		 = Left_Arm_Motor_Count > 58 && Left_Arm_Motor_Count  <62 ? 0 : Left_Arm_Motor_Count < 58 ? 10 : -10;
//		R_Arm_Speed 		 = Right_Arm_Motor_Count > 58 && Right_Arm_Motor_Count  <62 ? 0 : Right_Arm_Motor_Count < 58 ? 10 : -10;
//		Pitch_Arm_Speed = Pitch_Arm_Motor_Count > 58 && Pitch_Arm_Motor_Count  <62 ? 0 : Pitch_Arm_Motor_Count < 58 ? 10 : -10;
		L_Arm_Speed = R_Arm_Speed = Pitch_Arm_Speed = Tri_Arm_Speed;
	}
	else {L_Arm_Speed = R_Arm_Speed = Pitch_Arm_Speed = 0;}
	
	
	 
	
	//L_Arm_Speed = L_Arm_Speed > 0 && Left_Arm_Motor_Count >= 45 ? 10 : L_Arm_Speed < 0 && Left_Arm_Motor_Count <= 10 ? -10 : L_Arm_Speed ;
	L_Arm_Speed = L_Arm_Speed > 0 && Left_Arm_Motor_Count >= 50 ? 0 : L_Arm_Speed < 0 && Left_Arm_Motor_Count <= 5 ? 0 : L_Arm_Speed ;
	if( L_Arm_Speed_Temp != L_Arm_Speed )
	{
		Set_Motor_Velocity (L_Arm , L_Arm_Speed );	
		L_Arm_Speed_Temp = L_Arm_Speed ;
	}
	//R_Arm_Speed = R_Arm_Speed > 0 && Right_Arm_Motor_Count >= 45 ? 10 : R_Arm_Speed < 0 && Right_Arm_Motor_Count <= 10 ? -10 : R_Arm_Speed ;
	R_Arm_Speed = R_Arm_Speed > 0 && Right_Arm_Motor_Count >= 50 ? 0 : R_Arm_Speed < 0 && Right_Arm_Motor_Count <= 5 ? 0 : R_Arm_Speed ;
	if( R_Arm_Speed_Temp != R_Arm_Speed )
	{
		Set_Motor_Velocity (R_Arm , R_Arm_Speed );	
		R_Arm_Speed_Temp = R_Arm_Speed ;
	}
	Pitch_Arm_Speed = Pitch_Arm_Speed/2.39;
	Pitch_Arm_Speed = Pitch_Arm_Speed > 0 && Pitch_Arm_Motor_Count >= 13  ? 5 : Pitch_Arm_Speed < 0 && Pitch_Arm_Motor_Count <= 10 ? -5 : Pitch_Arm_Speed ;
	Pitch_Arm_Speed = Pitch_Arm_Speed > 0 && Pitch_Arm_Motor_Count >= 23  ? 0 : Pitch_Arm_Speed < 0 && Pitch_Arm_Motor_Count <= 0 ? 0 : Pitch_Arm_Speed ;
	if( Pitch_Arm_Speed_Temp != Pitch_Arm_Speed ) 
	{
		Set_Motor_Velocity (P_Arm , Pitch_Arm_Speed );	
		Pitch_Arm_Speed_Temp = Pitch_Arm_Speed ;
	}

}
void Top_Sensing_Roll(void)
{
	Array_Element    =  Speed + 4 ;

	Flap_Mod_Value = Flap_Data[Array_Element];
	Flap_Mod_Value_Right = Flap_Data_Right[Array_Element];

	Front_Left_Bush  =  Flap_Mod_Value < 40 ? 0 : 1;
	Front_Right_Bush = 	Flap_Mod_Value_Right < 40 ? 0: 1;
	
	Front_Bush = (Front_Left_Bush == 1 || Front_Right_Bush == 1) ? 1 : 0;

	if (Mode != 3 ) // Drive / Height Adjustment Modes
	{
		L_Arm_Speed 		= Left_Arm_Motor_Count  > 1 ? -ARM_HOMING_SPEED : Left_Arm_Motor_Count  < -1 ? ARM_HOMING_SPEED : 0;   //0.5
		R_Arm_Speed 		= Right_Arm_Motor_Count > 1 ? -ARM_HOMING_SPEED : Right_Arm_Motor_Count < -1 ? ARM_HOMING_SPEED : 0;    //0.5
		Pitch_Arm_Speed 	= Pitch_Arm_Motor_Count > 1 ? -ARM_HOMING_SPEED : Pitch_Arm_Motor_Count < -1 ? ARM_HOMING_SPEED : 0;   //0.5
		First_Sense = NULL;
	}
	
	else if(Mode == 3)  // Sensing_Mode
	{
		if ( Front_Bush ) First_Sense = SET; // Flap Not Yet Sensed - Initially
		
		if (!First_Sense) // Move Down to Sense the Bush
		{
			L_Arm_Speed = R_Arm_Speed  =  ARM_HOMING_SPEED ; 
            Pitch_Arm_Speed = ARM_HOMING_SPEED * 0.81; 
		}
	
		else  // Bush - Sensed Succesfully
		{
			if ( Front_Bush ) // Gap - Not Detected
			{
				Flap_Error = Front_Left_Bush == 1 ? Flaps_Target - Flap_Mod_Value : Right_Arm_Current_Pos - Left_Arm_Current_Pos; 
				L_Arm_Speed  = Left_Arm_PID ( Flap_Error , NULL );

				Flap_Error_Right = Front_Right_Bush == 1 ? Flaps_Target_Right - Flap_Mod_Value_Right : Left_Arm_Current_Pos - Right_Arm_Current_Pos; 
				R_Arm_Speed  = Right_Arm_PID ( Flap_Error_Right , NULL );

				Shear_Height_Diff = L_Arm_Travel - R_Arm_Travel;
				Shear_Roll_Angle = (atan ( Shear_Height_Diff / 2245 )) * (180/3.14);

				Pitch_Compensation_mm = fabs(Shear_Height_Diff * 0.435) ;
			// Pitch_Compensation_Rot = Pitch_Compensation_mm / 3.32 ;
				Pitch_Compensation_mm = Shear_Roll_Angle > 0 ? -Pitch_Compensation_mm : Pitch_Compensation_mm;

			// Pitch_Target = (Left_Arm_Current_Pos * 0.81 ) + Pitch_Compensation_Rot;
				Pitch_Target = ((L_Arm_Travel + Pitch_Compensation_mm) / 3.32) * 0.81 ;
				Pitch_Error = Pitch_Target - Pitch_Arm_Current_Pos;
				Pitch_Arm_Speed  = Pitch_Arm_PID ( Pitch_Error , NULL );
					
			}
			
			else // Gap - Detected
			{
				//L_Arm_Speed 		= Left_Arm_Motor_Count  > 1 ? -ARM_HOMING_SPEED : Left_Arm_Motor_Count  < -1 ? ARM_HOMING_SPEED : 0;    //0.5
				//R_Arm_Speed 		= Right_Arm_Motor_Count > 1 ? -ARM_HOMING_SPEED : Right_Arm_Motor_Count < -1 ? ARM_HOMING_SPEED : 0;   //0.5
				//Pitch_Arm_Speed 	= Pitch_Arm_Motor_Count > 1 ? -ARM_HOMING_SPEED : Pitch_Arm_Motor_Count < -1 ? ARM_HOMING_SPEED : 0;   //0.5
				L_Arm_Speed = R_Arm_Speed = Pitch_Arm_Speed = 0;
			}
			
		}
		
		//L_Arm_Speed = R_Arm_Speed = Tri_Arm_Speed ; 
		//Pitch_Arm_Speed = Tri_Arm_Speed * 0.81 ; 

	}
	
	else {}
		
	L_Arm_Speed = L_Arm_Speed > 0 && Left_Arm_Motor_Count > 20 ? 10 : L_Arm_Speed < 0 && Left_Arm_Motor_Count < 10 ? -10 : L_Arm_Speed;
	L_Arm_Speed = L_Arm_Speed > 0 && Left_Arm_Motor_Count > 30 ? 0 : L_Arm_Speed < 0 && Left_Arm_Motor_Count < 0 ? 0 : L_Arm_Speed; // Arm - Boundaries
	
	if( L_Arm_Speed_Temp != L_Arm_Speed )
	{
		for(uint8_t i = 0; i <= 2; i++)
		{
			Set_Motor_Velocity (12 , L_Arm_Speed );	
		}
		L_Arm_Speed_Temp = L_Arm_Speed ;
	}
	R_Arm_Speed = R_Arm_Speed > 0 && Right_Arm_Motor_Count > 20 ? 10 : R_Arm_Speed < 0 && Right_Arm_Motor_Count < 10 ? -10 : R_Arm_Speed;
	R_Arm_Speed = R_Arm_Speed > 0 && Right_Arm_Motor_Count > 30 ? 0 : R_Arm_Speed < 0 && Right_Arm_Motor_Count < 0 ? 0 : R_Arm_Speed;
	if( R_Arm_Speed_Temp != R_Arm_Speed )
	{
		for(uint8_t i = 0; i <= 2; i++)
		{
			Set_Motor_Velocity (13 , R_Arm_Speed );	
		}
		R_Arm_Speed_Temp = R_Arm_Speed ;
	}
	Pitch_Arm_Speed = Pitch_Arm_Speed > 0 && Pitch_Arm_Motor_Count > 15 ? 10 : Pitch_Arm_Speed < 0 && Pitch_Arm_Motor_Count < 5 ? -10 : Pitch_Arm_Speed;
	Pitch_Arm_Speed = Pitch_Arm_Speed > 0 && Pitch_Arm_Motor_Count > 20 ? 0 : Pitch_Arm_Speed < 0 && Pitch_Arm_Motor_Count < 0 ? 0 : Pitch_Arm_Speed;
	if( Pitch_Arm_Speed_Temp != Pitch_Arm_Speed )
	{
		for(uint8_t i = 0; i <= 2; i++)
		{
			Set_Motor_Velocity (14 , Pitch_Arm_Speed );	
		}
		Pitch_Arm_Speed_Temp = Pitch_Arm_Speed ;
	}

}
void Dynamic_Width_Adjustment (void)
{
	float Width_Speed=46.5/2 - 5;

	if ( !Steering_Reset_Flag  && Steering_Mode >= 4 )
	{
		if ( Steering_Mode == WIDTH_SHRINK && Angle_Ready ) 
		{
			if ( Joystick == 1 )Width_Motor_Speed =  -Width_Speed;
			else if ( Joystick == 2 ) Width_Motor_Speed  = Width_Speed;
			else Width_Motor_Speed = 0;
		}
		
		if ( Steering_Mode == WIDTH_EXTEND  && Angle_Ready ) 
		{
			if ( Joystick ==1 ) Width_Motor_Speed = Width_Speed;
			else if ( Joystick == 2 ) Width_Motor_Speed = -Width_Speed;
			else Width_Motor_Speed = 0;
		}
		else if ( Steering_Mode < 4 )
		{
			Width_Motor_Speed = 0;
			//Dynamic_Width_Corrections();
		}

	}
	else 
	{
		Width_Motor_Speed = 0 ;
	}
	
	if( Steering_Mode < 4 ) Width_Motor_Speed = 0;
	Lower_Width_Motor_Speed = Upper_Width_Motor_Speed = Width_Motor_Speed ;
	
	Lower_Width_Motor_Speed=(( Lower_Width_Motor_Speed < 0) && (Lower_Width_Motor_Count<=-550  )) ? -10:(( Lower_Width_Motor_Speed > 0) && ( Lower_Width_Motor_Count >= -50 ))?10:Lower_Width_Motor_Speed;
	if (( Lower_Width_Motor_Speed < 0) && (( Lower_Width_Motor_Count >= -605 && Lower_Width_Motor_Count<=-595  )|| Lower_Width_Motor_Count <= -605)) Lower_Width_Motor_Speed = 0;
	else if (( Lower_Width_Motor_Speed > 0) && (( Lower_Width_Motor_Count >=-5 && Lower_Width_Motor_Count<=5 )||Lower_Width_Motor_Count>=5 )) Lower_Width_Motor_Speed = 0;
	else{}
	
	Upper_Width_Motor_Speed=(( Upper_Width_Motor_Speed < 0) && (Upper_Width_Motor_Count<=-550  )) ? -10:(( Upper_Width_Motor_Speed > 0) && ( Upper_Width_Motor_Count >= -50 ))?10:Upper_Width_Motor_Speed;
	if (( Upper_Width_Motor_Speed < 0) &&(( Upper_Width_Motor_Count >= -605 && Upper_Width_Motor_Count<=-595  )|| Upper_Width_Motor_Count <= -605))  Upper_Width_Motor_Speed = 0;
	else if (( Upper_Width_Motor_Speed > 0) &&(( Upper_Width_Motor_Count >=-5 && Upper_Width_Motor_Count<=5 )||Upper_Width_Motor_Count>=5 )) Upper_Width_Motor_Speed = 0;
	else{}
	
	if ( Width_Motor_Speed != Width_Motor_Temp || Lower_Width_Motor_Speed != Lower_Width_Motor_Speed_Temp || Upper_Width_Motor_Speed != Upper_Width_Motor_Speed_Temp )
	{
		for ( uint8_t i = 0 ; i < 4 ; i++)
		{
		Set_Motor_Velocity ( 15, Lower_Width_Motor_Speed); 
		Set_Motor_Velocity ( 16, Upper_Width_Motor_Speed);
		}
		Width_Motor_Temp = Width_Motor_Speed; 
		Lower_Width_Motor_Speed_Temp=Lower_Width_Motor_Speed;
		Upper_Width_Motor_Speed_Temp=Upper_Width_Motor_Speed;
		
	}

}
void Shearing_Motors (void)
{
		if ( Shearing_Temp != Shearing )
	{
		if ( Shearing == 2 )
		{
//			for(uint8_t i = 17; i <= 20; i++)
//	   {
//		    Clear_Error(i);	HAL_Delay(1500);
//		    Start_Calibration_For ( i,  8 , 2 );
//				HAL_Delay(20);
//	    }

			for ( int i=0; i < 5; i++)
			{			
				//Set_Motor_Velocity( 17 , 20 ); HAL_Delay(10); // SELECTIVE
				Set_Motor_Velocity( 18 , 20 ); HAL_Delay(10); // MAIN PADDLE
				Set_Motor_Velocity( 19 , 20 ); HAL_Delay(10);	// SIDE PADDLE
				Set_Motor_Velocity( 20 , 20 ); 							// CUTTER
			}
		}
		else 
		{	
			for ( int i=0; i < 4 ; i++)
			{
				Set_Motor_Velocity( 17 , 0 ); 
				Set_Motor_Velocity( 18 , 0 ); 
				Set_Motor_Velocity( 19 , 0 ); 
				Set_Motor_Velocity( 20 , 0 );				
			}
		}
		
		 Shearing_Temp = Shearing ;
	}
}

void Position_Flap_Sensing(void)
{
		FL_Angle = Flap_Data[5] ;
		Front_Left_Bush  =  FL_Angle > 45 ? 0 : 1;
	
	
	if (Mode == 0 )
	{
			Tri_Arm_Pos = 0;
		Left_Arm_Pos = Right_Arm_Pos = Pitch_Arm_Pos = Tri_Arm_Pos;
	}
	
	else if ( Mode == 1)
	{
		if ( Front_Left_Bush )
		{
				
			Arm_Angle = (FL_Angle /3.2) -1 ;//3.2
			Tri_Arm_Pos = (Arm_Angle/360)*600;
			Left_Arm_Pos = Right_Arm_Pos = Tri_Arm_Pos;
			Pitch_Arm_Pos = Tri_Arm_Pos * 0.6082 ;
		}
		else 
		{
		}
	}
	
	
	if( Left_Arm_Pos_Temp != Left_Arm_Pos ) 
	{
		Set_Motor_Position (L_Arm , Left_Arm_Pos );	
		Left_Arm_Pos_Temp = Left_Arm_Pos ;
	}
	
	if( Right_Arm_Pos_Temp != Right_Arm_Pos ) 
	{
		Set_Motor_Position (R_Arm , Right_Arm_Pos );	
		Right_Arm_Pos_Temp = Right_Arm_Pos ;
	}
	
	if( Pitch_Arm_Pos_Temp != Pitch_Arm_Pos ) 
	{
		Set_Motor_Position (P_Arm , Pitch_Arm_Pos );	
		Pitch_Arm_Pos_Temp = Pitch_Arm_Pos ;
	}

}


//if ( Motor_Velocity[8] != LF_Speed ) 

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the DefaultTasks thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  while(1)
  {
	//    Manual_Controls();
		// Manual_Wheel_Control();
		//if(Joystick == 0 )
		Joystick_Reception();
		EEPROM_Store_Data();
//		if ( Manual_Torque_Temp != Manual_Torque )
//		{
//			for(uint8_t i = 1; i < 5 ; i++ ) {Set_Motor_Torque ( i , Manual_Torque); HAL_Delay(10);} 
//			Manual_Torque_Temp = Manual_Torque;
//		}
		

  		Shearing_Motors();
//		Buzz_Switch = (Frame_Buzz_Switch == 1) ? 1 : 0;
//		if ( Buzz_Switch == 1 ) {HAL_GPIO_WritePin(Buzzer_1_GPIO_Port,Buzzer_1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(Buzzer_2_GPIO_Port,Buzzer_2_Pin, GPIO_PIN_SET);	}
//		else HAL_GPIO_WritePin(Buzzer_1_GPIO_Port,Buzzer_1_Pin, GPIO_PIN_SET);HAL_GPIO_WritePin(Buzzer_2_GPIO_Port,Buzzer_2_Pin, GPIO_PIN_SET);	;
//						Frame_Controls();

    HAL_Delay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_HP_Tasks */
/**
* @brief Function implementing the HP_Tasks thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_HP_Tasks */
void Start_HP_Tasks(void const * argument)
{
  /* USER CODE BEGIN Start_HP_Tasks */
  /* Infinite loop */
  while(1)
  {
		Position_Flap_Sensing();
//  	Top_Flap_Sensing();
//		Drive_Wheel_Controls();
		Steering_Controls();
//		Dynamic_Width_Adjustment();
	//	Frame_Controls();
	New_Drive_Controls();
		//Left_Frame_Controls();
    HAL_Delay(10); 
  }
  /* USER CODE END Start_HP_Tasks */
}

/* USER CODE BEGIN Header_Start_LP_Tasks */
/**
* @brief Function implementing the LP_Tasks thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_LP_Tasks */
void Start_LP_Tasks(void const * argument)
{
  /* USER CODE BEGIN Start_LP_Tasks */
  /* Infinite loop */
  while(1)
  {
		HAL_Delay(100);
		Battery_Status_Indication();
		Drives_Error_Check();
  }
  /* USER CODE END Start_LP_Tasks */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/*******************************************************************************************************/
/*
 * stepper.h
 * Created on: 19-Apr-2024
 * Author: Le Huu An
 * Mail: an.lehuu2001.hust@gmail.com
 */
/*******************************************************************************************************/

#ifndef STEPPER_H_
#define STEPPER_H_

#include "main.h"

/*Include---------------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "stdbool.h"

/*Define----------------------------------------------------------------------------------*/
	#define START				 			0
	#define RETURN_ZERO 			1
	#define READY 						2
	#define RUNNING						3
	#define CONFIRM_SETTING		4
	#define RUN_FREE     			5
	#define AUTO_RUN					6
	
	#define ZERO 		0
	#define I 			1
	#define II 			2
	#define III			3
	#define IV      4
	#define V      	5
	
	#define CCW     0
	#define CW      1
		
/*Declare Structs--------------------------------------------------------------------------*/
	typedef struct{
		float degree_I, degree_II, degree_III, degree_IV, degree_V;
		uint32_t num_Pul_I, num_Pul_II, num_Pul_III, num_Pul_IV, num_Pul_V;
		float speed_needle, speed_comeZero;					// speed of needle when rotate normal and return Zero 
		int32_t num_Pul[20];
		
		uint16_t ppr;					// pulse per round 	
		uint8_t gear_ratio;
		uint16_t period_needle, period_comeZero;	//(us)
		uint8_t Pos_in_Flash;			// position that saving in flash
		
		volatile uint8_t vStateMotor;
		volatile uint8_t Direction;
		volatile GPIO_PinState isSwitchLimit;
		volatile uint32_t cnt_SwitchLimit;
		volatile uint8_t getCTHT;
		
		volatile uint32_t time_start_Pul, interval_Pul;		// time to set pulse
		volatile uint32_t time_start_delay;		// time to set pulse
		
		volatile int32_t PulseSetPoint, PulseCurrent;
		volatile uint8_t currentPos, targetPos;
		
		volatile bool toRunning, toZero, toConfirmSetting, toSetting, toRunFree, toAutoRun, toChangeLevel;
		volatile bool flag_startPul, flag_isDelaying;
		volatile bool flag_setI, flag_setII, flag_setIII, flag_setIV, flag_setV;
		volatile uint8_t cnt_AutoRun;
		
		float temp_degree_I, temp_degree_II, temp_degree_III, temp_degree_IV, temp_degree_V;
		uint8_t temp_needle_pos;
	}Motor_t;
	
/*Extern variable------------------------------------------------------------------------*/
	extern Motor_t vMotor;
	
/*Declare Functions------------------------------------------------------------------------*/

//	int32_t Cal_Pul(uint8_t currentPos, uint8_t targetPos);
//	void Motor_Init(Motor_t *pM, uint16_t ppr, float speed);
//	void Process_Motor(Motor_t *pM);
//	
//	void SetTargetPos(Motor_t *pM, uint8_t targetPos);
//	void Reset_Pulse(Motor_t *pM);
//	void SetMotorDir(uint8_t dir);
//	void Enable_Motor(void);
//	void Disable_Motor(void);

	
	int32_t Cal_Pul(int32_t *pNum_Pul, uint8_t currentPos, uint8_t targetPos);
	void Motor_Init(Motor_t *pM, uint16_t _ppr, float _speed_needle, uint8_t _gear_ratio, uint8_t _previous_Pos);
	void Process_Motor(Motor_t *pM);
	
	void SetTargetPos(Motor_t *pM, uint8_t targetPos);
	void Reset_Pulse(Motor_t *pM);
	void SetMotorDir(uint8_t dir);
	void Enable_Motor(void);
	void Disable_Motor(void);
	void Brake_OPEN(void);
	void Brake_CLOSE(void);
	
	GPIO_PinState getSwitchLimit(void);
	
	void Gen_a_Pul(Motor_t *pM);
	
	void Degree_Setup(Motor_t *pM, float _I, float _II, float _III, float _IV, float _V);
	void Cal_Num_All_Pul(Motor_t *pM);
	void Update_Degree(Motor_t *pM);
	
	void Command_toSetDegree(Motor_t *pM, uint8_t _Pos, float _degree);
	void Command_toZERO(Motor_t *pM);
	void Command_toRUNFREE(Motor_t *pM, int32_t _targetPul);
	void Command_toConfirm(Motor_t *pM);
	void Command_toChangeLevel(Motor_t *pM, uint8_t _PosTarget);
	void Command_toAutoRun(Motor_t *pM);
	
	void Move_Needle_toPos(Motor_t *pM, uint8_t _PosTarget);
	bool delay_motor(uint32_t us);	
	void Auto_func(Motor_t *pM);
	


/*******************************************************************************************************/
#endif /* STEPPER_H_*/



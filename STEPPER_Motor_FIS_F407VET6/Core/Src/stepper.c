/*******************************************************************************************************/
/*
 * stepper.c
 * Created on: 15-June-2024
 * Author: Le Huu An
 * Mail: an.lehuu2001.hust@gmail.com
 * Zalo: 0966784072
 */
/*
NOTE:

*/
/*******************************************************************************************************/

/*Include---------------------------------------------------------------------------------*/
#include <stepper.h>


/*Extern----------------------------------------------------------------------------------*/
extern uint8_t result;


/*Declare Variable-------------------------------------------------------------------------*/
	Motor_t vMotor;
	

/*Function---------------------------------------------------------------------------------*/

/*______________________________________________________________________________*/
	uint32_t degree_to_pulse(uint8_t _gear_ratio, uint16_t _ppr, float _degree){
		return (int)((_gear_ratio*_ppr*_degree)/360);
	}
/*______________________________________________________________________________*/
	void Motor_Init(Motor_t *pM, uint16_t _ppr, float _speed_needle, uint8_t _gear_ratio, uint8_t _Pos_in_Flash){
		pM->ppr = _ppr;
		pM->gear_ratio = _gear_ratio;
		pM->speed_needle = _speed_needle;
		pM->period_needle = (int)((60*1e6)/(pM->gear_ratio*pM->speed_needle*pM->ppr));

		pM->vStateMotor = READY;
		pM->Direction = CCW;
		pM->speed_comeZero = 1;
		pM->period_comeZero = (int)((60*1e6)/(pM->gear_ratio*pM->speed_comeZero*pM->ppr));	 

		/*Gen pulse variables*/
		pM->flag_startPul = true;	 
		pM->time_start_Pul = 0;
		pM->interval_Pul = 0;
		pM->PulseCurrent = 0;
		pM->PulseSetPoint = 0;

		/*Needle Position variables*/
		pM->isSwitchLimit = GPIO_PIN_SET; pM->cnt_SwitchLimit = 0;
		pM->Pos_in_Flash = _Pos_in_Flash;
		pM->targetPos = ZERO;
		pM->currentPos = pM->Pos_in_Flash;

		pM->toZero = true;
		pM->toRunning = false;
		pM->toConfirmSetting = false;
		pM->toSetting = false;
		pM->toRunFree = false;
		pM->toChangeLevel = false;
		pM->toAutoRun = true;

		pM->flag_isDelaying = false;
		pM->time_start_delay = 0;

		pM->temp_degree_I = 0; 
		pM->temp_degree_II = 0; 
		pM->temp_degree_III = 0; 
		pM->temp_degree_IV = 0; 
		pM->temp_degree_V = 0;
		pM->targetPos = ZERO;
	}

/*______________________________________________________________________________*/
	void Degree_Setup(Motor_t *pM, float _I, float _II, float _III, float _IV, float _V){
		pM->degree_I = _I;
		pM->degree_II = _II;
		pM->degree_III = _III;
		pM->degree_IV = _IV;
		pM->degree_V = _V;	
		
		Cal_Num_All_Pul(pM);
	}
/*______________________________________________________________________________*/
	void Cal_Num_All_Pul(Motor_t *pM){
		pM->num_Pul_I = degree_to_pulse(pM->gear_ratio, pM->ppr, pM->degree_I);
		pM->num_Pul_II = degree_to_pulse(pM->gear_ratio, pM->ppr, pM->degree_II);
		pM->num_Pul_III = degree_to_pulse(pM->gear_ratio, pM->ppr, pM->degree_III);
		pM->num_Pul_IV = degree_to_pulse(pM->gear_ratio, pM->ppr, pM->degree_IV);
		pM->num_Pul_V = degree_to_pulse(pM->gear_ratio, pM->ppr, pM->degree_V);
		
		pM->num_Pul[0] = pM->num_Pul_II - pM->num_Pul_I;					// I to II
		pM->num_Pul[1] = pM->num_Pul_III - pM->num_Pul_I; 				// I to III
		pM->num_Pul[2] = pM->num_Pul_IV - pM->num_Pul_I; 					// I to IV
		pM->num_Pul[3] = pM->num_Pul_V - pM->num_Pul_I;						// I to V
		pM->num_Pul[4] = pM->num_Pul_I - pM->num_Pul_II; 					// II to I
		pM->num_Pul[5] = pM->num_Pul_III - pM->num_Pul_II; 					// II to III
		pM->num_Pul[6] = pM->num_Pul_IV - pM->num_Pul_II; 					// II to IV
		pM->num_Pul[7] = pM->num_Pul_V - pM->num_Pul_II; 					// II to V
		pM->num_Pul[8] = pM->num_Pul_I - pM->num_Pul_III; 					// III to I
		pM->num_Pul[9] = pM->num_Pul_II - pM->num_Pul_III; 					// III to II
		pM->num_Pul[10] = pM->num_Pul_IV - pM->num_Pul_III; 					// III to IV
		pM->num_Pul[11] = pM->num_Pul_V - pM->num_Pul_III; 					// III to V
		pM->num_Pul[12] = pM->num_Pul_I - pM->num_Pul_IV; 				// IV to I
		pM->num_Pul[13] = pM->num_Pul_II - pM->num_Pul_IV; 				// IV to II
		pM->num_Pul[14] = pM->num_Pul_III - pM->num_Pul_IV; 				// IV to III
		pM->num_Pul[15] = pM->num_Pul_V - pM->num_Pul_IV;					// IV to V
		pM->num_Pul[16] = pM->num_Pul_I - pM->num_Pul_V; 				// V to I
		pM->num_Pul[17] = pM->num_Pul_II - pM->num_Pul_V; 				// V to II
		pM->num_Pul[18] = pM->num_Pul_III - pM->num_Pul_V; 				// V to III
		pM->num_Pul[19] = pM->num_Pul_IV - pM->num_Pul_V;					// V to IV
	}

/*______________________________________________________________________________*/
	int32_t Cal_Pul(int32_t *pNum_Pul, uint8_t currentPos, uint8_t targetPos){
		if(currentPos < targetPos)
			return pNum_Pul[4*currentPos+targetPos-6];
		else
			return pNum_Pul[4*currentPos+targetPos-5];
	}
/*______________________________________________________________________________*/	
	void Reset_Pulse(Motor_t *pM){
		pM->flag_startPul = true;
		pM->time_start_Pul = 0;
		pM->interval_Pul = 0;
	}
/*______________________________________________________________________________*/	
	void SetMotorDir(uint8_t dir){
		if(dir){
			HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);		// CW Dir
		}
		else{
			HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);		// CCW Dir
		}
	}
/*______________________________________________________________________________*/	
	void Enable_Motor(void){
		HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET);
	}
/*______________________________________________________________________________*/	
	void Disable_Motor(void){
		HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
	}
/*______________________________________________________________________________*/	
	void Brake_OPEN(void){
		HAL_GPIO_WritePin(CTRL_BRAKE_GPIO_Port, CTRL_BRAKE_Pin, GPIO_PIN_SET);
	}
/*______________________________________________________________________________*/	
	void Brake_CLOSE(void){
		HAL_GPIO_WritePin(CTRL_BRAKE_GPIO_Port, CTRL_BRAKE_Pin, GPIO_PIN_RESET);
	}
/*______________________________________________________________________________*/
	GPIO_PinState getSwitchLimit(void){
		//return HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
		return HAL_GPIO_ReadPin(LIMIT_SWITCH_GPIO_Port, LIMIT_SWITCH_Pin);
	}
/*______________________________________________________________________________*/
	void Gen_a_Pul(Motor_t *pM){
		if(pM->flag_startPul){
					pM->time_start_Pul = getMicroSecond();
					pM->flag_startPul = false;
			}
		else{
			pM->interval_Pul = getMicroSecond() - pM->time_start_Pul;
			if(pM->interval_Pul <= pM->period_needle/3){
				HAL_GPIO_WritePin(PUL_GPIO_Port, PUL_Pin, GPIO_PIN_SET);
			}
			else{
				if(pM->interval_Pul <= pM->period_needle){
					HAL_GPIO_WritePin(PUL_GPIO_Port, PUL_Pin, GPIO_PIN_RESET);
				}
				else{
					/*Hoan thanh 1 xung*/
					Reset_Pulse(pM);					
					if(pM->vStateMotor != RETURN_ZERO)
						pM->PulseCurrent++;					
				}
			}
		}
	}
	
/*______________________________________________________________________________*/
	void Command_toSetDegree(Motor_t *pM, uint8_t _Pos, float _degree){
		if(_Pos == I){
			pM->flag_setI = true;
			pM->temp_degree_I = _degree;
		}
		else if(_Pos == II){
			pM->flag_setII = true;
			pM->temp_degree_II = _degree;
		}
		else if(_Pos == III){
			pM->flag_setIII = true;
			pM->temp_degree_III = _degree;
		}
		else if(_Pos == IV){
			pM->flag_setIV = true;
			pM->temp_degree_IV = _degree;
		}
		else if(_Pos == V){
			pM->flag_setV = true;
			pM->temp_degree_V = _degree;
		}
		else;
		
		pM->temp_needle_pos = _Pos;
		pM->PulseSetPoint = degree_to_pulse(pM->gear_ratio, pM->ppr, _degree);
		pM->vStateMotor = READY;
		pM->toZero = true;
		pM->toSetting = true;
	}
/*______________________________________________________________________________*/
	void Command_toZERO(Motor_t *pM){
		pM->toZero = true;
	}
/*______________________________________________________________________________*/
	void Command_toRUNFREE(Motor_t *pM, int32_t _targetPul){
		pM->toRunFree = true;
		if(_targetPul > 0){
			pM->PulseSetPoint = _targetPul;
			pM->Direction = CW;
		}
		else{
			pM->PulseSetPoint = -_targetPul;
			pM->Direction = CCW;
		}
	}
/*______________________________________________________________________________*/
	void Command_toConfirm(Motor_t *pM){
		pM->toConfirmSetting = true;
	}
/*______________________________________________________________________________*/
	void Command_toAutoRun(Motor_t *pM){
		pM->toAutoRun = true;
	}
/*______________________________________________________________________________*/
	void Command_toChangeLevel(Motor_t *pM, uint8_t _PosTarget){
		Move_Needle_toPos(pM, _PosTarget);
	}
/*______________________________________________________________________________*/
	void Move_Needle_toPos(Motor_t *pM, uint8_t _PosTarget){
		if(_PosTarget == I){
			pM->targetPos = I;
			pM->PulseSetPoint = pM->num_Pul_I;
		}
		else if(_PosTarget == II){
			pM->targetPos = II;
			pM->PulseSetPoint = pM->num_Pul_II;
		}
		else if(_PosTarget == III){
			pM->targetPos = III;
			pM->PulseSetPoint = pM->num_Pul_III;
		}
		else if(_PosTarget == IV){
			pM->targetPos = IV;
			pM->PulseSetPoint = pM->num_Pul_IV;
		}
		else if(_PosTarget == V){
			pM->targetPos = V;
			pM->PulseSetPoint = pM->num_Pul_V;
		}
		else;
		pM->toZero = true;
		pM->toRunning = true;
		pM->vStateMotor = READY;
	}
	
/*______________________________________________________________________________*/
	void SetTargetPos(Motor_t *pM, uint8_t targetPos){
		if(pM->vStateMotor == READY){
			if(targetPos != pM->currentPos){
				pM->targetPos = targetPos;					
				int32_t targetP = Cal_Pul(pM->num_Pul, pM->currentPos, pM->targetPos);
				if(targetP > 0){
					pM->PulseSetPoint = targetP;
					pM->Direction = CW;
				}
				else{
					pM->PulseSetPoint = -targetP;
					pM->Direction = CCW;
				}
				pM->toRunning = true;
			}
		}
	}
/*______________________________________________________________________________*/
	static bool flag_delay_motor = true;
	static uint32_t time_start_delay = 0;
	extern uint32_t __s, __e, __i;
	
	bool delay_motor(uint32_t us){
		if(flag_delay_motor){
			time_start_delay = getMicroSecond();
			__s = time_start_delay;
			flag_delay_motor = false;
			return false;
		}
		else{
			__e = getMicroSecond();
			if((getMicroSecond() - time_start_delay) >= us){
				__i = __e - __s;
				flag_delay_motor = true;
				return true;
			}
			else{
				__i = 0;
				return false;
			}
		}
	}
/*______________________________________________________________________________*/
	static uint8_t flag_autoset = 0;
	static uint8_t cnt_auto = 0;
	
	void Auto_func(Motor_t *pM){
		if(flag_autoset == 1){
			if(cnt_auto == 1){
				pM->PulseSetPoint = pM->num_Pul_I;
			}
			else if(cnt_auto == 2){
				pM->PulseSetPoint = pM->num_Pul_II;
			}
			else if(cnt_auto == 3){
				pM->PulseSetPoint = pM->num_Pul_III;
			}
			else if(cnt_auto == 4){
				pM->PulseSetPoint = pM->num_Pul_IV;
			}
			else if(cnt_auto == 5){
				pM->PulseSetPoint = pM->num_Pul_V;
				
			}
			//Brake_OPEN();
			Enable_Motor();
			
			if(delay_motor(3e6)){
					flag_autoset = 0;
					pM->toRunning = true;
					pM->vStateMotor = READY;
					pM->Direction = CW;
			}
		}
		else{
			if(delay_motor(60e6)){
				flag_autoset = 1;
				pM->toZero = true;
				if(++cnt_auto > 5)	cnt_auto = 1;
			}
		
		}
	}	
	
/*______________________________________________________________________________*/
	void Update_Degree(Motor_t *pM){
		if(pM->flag_setI){
					pM->degree_I = pM->temp_degree_I;
					/***Write value to flash*/
					pM->flag_setI = false;
		}
		if(pM->flag_setII){
			pM->degree_II = pM->temp_degree_II;
			/***Write value to flash*/
			pM->flag_setII = false;
		}
		if(pM->flag_setIII){
			pM->degree_III = pM->temp_degree_III;
			/***Write value to flash*/
			pM->flag_setIII = false;
		}
		if(pM->flag_setIV){
			pM->degree_IV = pM->temp_degree_IV;
			/***Write value to flash*/
			pM->flag_setIV = false;
		}
		if(pM->flag_setV){
			pM->degree_V = pM->temp_degree_V;
			/***Write value to flash*/
			pM->flag_setV = false;
		}
	
	}
	
/*______________________________________________________________________________*/
	void Process_Motor(Motor_t *pM){
		switch(pM->vStateMotor)
		{
		//-------------------------------------------
			case READY:
			{
				if(pM->toZero){
					pM->getCTHT = getSwitchLimit();
					if(!pM->getCTHT){
						pM->cnt_SwitchLimit++;
					}
					else{
						pM->cnt_SwitchLimit = 0;
					}
					if(!pM->getCTHT && pM->cnt_SwitchLimit >= 100){
						pM->toZero = false;	
						pM->cnt_SwitchLimit = 0;
					}
					else{				// not touch the switch limit
						//Brake_OPEN();		/*Open Brake*/		
						Enable_Motor();		/*Enable Motor*/
						SetMotorDir(CCW);	/*Set direction motor*/
						
						//pM->vStateMotor = RETURN_ZERO;
						if(delay_motor(3e6)){		/*Wait Openning brake finish*/
							pM->vStateMotor = RETURN_ZERO;
							
						}
					}					
				}
				else if(pM->toRunning){		
					//Brake_OPEN();		/*Open Brake*/		
					Enable_Motor();		/*Enable Motor*/	
					SetMotorDir(pM->Direction);	/*Set direction motor*/
					
					//pM->vStateMotor = RUNNING;
					if(delay_motor(3e6)){		/*Wait Openning brake finish*/
						pM->vStateMotor = RUNNING;
					}		
				}
				else if(pM->toRunFree){					
					//Brake_OPEN();		/*Open Brake*/
					Enable_Motor();		/*Enable Motor*/					
					SetMotorDir(pM->Direction);		/*Set direction motor*/
					pM->vStateMotor = RUN_FREE;
				}
				else if(pM->toSetting){					
					//Brake_OPEN();			/*Open Brake*/					
					Enable_Motor();			/*Enable Motor*/	
					SetMotorDir(CW);		/*Set direction motor*/
					
					//pM->vStateMotor = RUNNING;
					if(delay_motor(3e6)){		/*Wait Openning brake finish*/
						pM->vStateMotor = RUNNING;
					}
				}
				else if(pM->toConfirmSetting){				
					pM->vStateMotor = CONFIRM_SETTING;
				}
				else if(pM->toAutoRun){
					pM->vStateMotor = AUTO_RUN;
				}
				
				else;	
				break;
			}
		//-------------------------------------------
			case RUNNING:
			{
				if(pM->PulseCurrent == pM->PulseSetPoint){				// Finish Rotation, stop motor										
					Disable_Motor();		/*Disable Motor*/				
					//Brake_CLOSE();		/*Close Brake*/
				
//					pM->vStateMotor = READY;
//					pM->toRunning = false;
//					pM->toSetting = false;
//					pM->PulseCurrent = 0;
//					pM->PulseSetPoint = 0;
//					pM->currentPos = pM->targetPos;
//					Reset_Pulse(pM);
					if(delay_motor(3e6)){		/*Wait Openning brake finish*/
						pM->vStateMotor = READY;
						pM->toRunning = false;
						pM->toSetting = false;
						pM->PulseCurrent = 0;
						pM->PulseSetPoint = 0;
						pM->currentPos = pM->targetPos;
							Reset_Pulse(pM);
					}					
				}
				else{			
					Gen_a_Pul(pM);	/*Gen Pulse*/
				}
				
				break;
			}
		//-------------------------------------------
			case RETURN_ZERO:
			{
				pM->getCTHT = getSwitchLimit();
				if(!pM->getCTHT){
					pM->cnt_SwitchLimit++;
				}
				else{
					pM->cnt_SwitchLimit = 0;
				}
				if(!pM->getCTHT && pM->cnt_SwitchLimit >= 100){		// needle at zero, return zero finish		
									
					Disable_Motor();			/*Disable Motor*/					
					//Brake_CLOSE();			/*Close Brake*/
					Reset_Pulse(pM);
					pM->Direction = CW;
					
					if(delay_motor(3e6)){		// delay 2s
						pM->PulseCurrent = 0;
						pM->vStateMotor = READY;
						pM->toZero = false;
						result = 1;
						
						pM->cnt_SwitchLimit = 0;
					}
				}
				else{						// return zero									
					Gen_a_Pul(pM);			/*Gen Pulse*/

				}
				break;
			}
		//-------------------------------------------
			case RUN_FREE:	
			{				
				Gen_a_Pul(pM);		/*Gen Pulse*/		

				if(pM->PulseCurrent == pM->PulseSetPoint){				// Finish Rotation
					//stop motor					
					Disable_Motor();		/*Disable Motor*/					
					//Brake_CLOSE();		 /*Close Brake*/
					pM->vStateMotor = READY;
					pM->toRunFree = false;
					pM->PulseCurrent = 0;
					pM->PulseSetPoint = 0;
					Reset_Pulse(pM);	
					result = 2;					
				}	
				break;
			}
		//-------------------------------------------
			case CONFIRM_SETTING:		// confirm the setting
			{
				/*Update the degree in flash*/
				Update_Degree(pM);
					
				Cal_Num_All_Pul(pM);
				
				/*Setpoint: move the needle to Position saved in flash*/
				Move_Needle_toPos(pM, pM->Pos_in_Flash);
				
				pM->toConfirmSetting = false;
				break;
			}
		//-------------------------------------------
			case AUTO_RUN:
			{		
				Auto_func(pM);	
				break;
			}
		}
		//-------------------------------------------
	}
/*______________________________________________________________________________*/	

/*******************************************************************************************************/
//if(BLT_Pos_set != 0){
//	if(BLT_Pos_set == 1){
//		Degree_Setting(&vMotor, I, BLT_degree);
//	}
//	else if(BLT_Pos_set == 2){
//		Degree_Setting(&vMotor, II, BLT_degree);
//	}
//	else if(BLT_Pos_set == 3){
//		Degree_Setting(&vMotor, III, BLT_degree);
//	}
//	else if(BLT_Pos_set == 4){
//		Degree_Setting(&vMotor, IV, BLT_degree);
//	}
//	else if(BLT_Pos_set == 5){
//		Degree_Setting(&vMotor, V, BLT_degree);
//	}
//	else;
//	BLT_Pos_set = 0;
//}

#include <stdlib.h>
#include "motor.h"
#include "mxconstants.h"
#include "stm32f3xx_hal.h"


#define STEPS_PER_FULL		64
#define INTERNAL_GEAR_RATIO	64
#define OUTPUT_GEAR_RATIO	2
#define STEP_ANGLE			(double)(360.0 / (double)(STEPS_PER_FULL*INTERNAL_GEAR_RATIO*OUTPUT_GEAR_RATIO))

#define STEPS_PER_ROTATION	(STEPS_PER_FULL * INTERNAL_GEAR_RATIO * OUTPUT_GEAR_RATIO )

int16_t current_angle_deg = 0;
int16_t target_angle_deg = 0;
uint32_t target_step_count = 0;
uint32_t step_count = 0;
bool direction = false;

bool motor_running = false;

uint32_t step_interval = 0;
uint32_t target_interval = 0;

#define MIN_SPEED 0x8000
#define MAX_SPEED 0x4000

#define MOTOR_SEQ_COUNT		8
uint8_t motor_movement_cw [MOTOR_SEQ_COUNT] =
						   {
								   0b1000,
								   0b1100,
								   0b0100,
								   0b0110,
								   0b0010,
								   0b0011,
								   0b0001,
								   0b1001
						   };



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	HAL_GPIO_TogglePin(testpin_GPIO_Port, testpin_Pin);
	if(target_step_count == step_count)
	{
		motors_off();
		motor_running = false;
		HAL_TIM_Base_Stop_IT(&htim6);
	}
	else{

	if(direction)
		step_cw();
	else
		step_acw();
	}
}


void set_target(int16_t target_deg)
{
	target_angle_deg = target_deg;
	target_step_count = (uint32_t)((double)target_angle_deg / STEP_ANGLE);
	current_angle_deg = step_count * STEP_ANGLE;
	bool dir = (angle_difference(current_angle_deg, target_angle_deg) > 0);
	printf("t=%d:%d, c=%d:%d\n\r", target_angle_deg, target_step_count, current_angle_deg, step_count);
	if(dir)
	printf("cw\n\r");
	else
		printf("acw\n\r");
	set_speed(100, dir);
}

bool motors_running()
{
	return motor_running;
}
//@param speed 0-255 (0 being off) linear scale between off and full speed
//@param clockwise - direction to rotate
void set_speed(uint8_t speed, bool cw_nacw)
{
	direction = cw_nacw;
	if(0 == speed)
	{
		motors_off();
		motor_running = false;
		HAL_TIM_Base_Stop_IT(&htim6);
	}
	else
	{
		htim6.Instance->ARR = MIN_SPEED;
//		htim6.Instance->ARR = (MIN_SPEED - ((MIN_SPEED - MAX_SPEED) / (256 - (uint32_t)speed)));
		if(!motor_running)
		{
			HAL_TIM_Base_Start_IT(&htim6);
			motor_running = true;
		}
	}
}

bool get_zero_state()
{
	return (HAL_GPIO_ReadPin(ZERO_GPIO_Port, ZERO_Pin) == GPIO_PIN_RESET);
}

void motors_off()
{
	HAL_GPIO_WritePin(MOTOR_1_GPIO_Port,MOTOR_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_2_GPIO_Port,MOTOR_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_3_GPIO_Port,MOTOR_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_4_GPIO_Port,MOTOR_4_Pin, GPIO_PIN_RESET);
}

void write_motors(uint8_t state)
{
	motors_off();
	if(state & 0x01)
	{
		HAL_GPIO_WritePin(MOTOR_1_GPIO_Port,MOTOR_1_Pin, GPIO_PIN_SET);
	}
//	else
//	{
//		HAL_GPIO_WritePin(MOTOR_1_GPIO_Port,MOTOR_1_Pin, GPIO_PIN_RESET);
//	}

	if(state & 0x02)
	{
		HAL_GPIO_WritePin(MOTOR_2_GPIO_Port,MOTOR_2_Pin, GPIO_PIN_SET);
	}
//	else
//	{
//		HAL_GPIO_WritePin(MOTOR_2_GPIO_Port,MOTOR_2_Pin, GPIO_PIN_RESET);
//	}

	if(state & 0x04)
	{
		HAL_GPIO_WritePin(MOTOR_3_GPIO_Port,MOTOR_3_Pin, GPIO_PIN_SET);
	}
//	else
//	{
//		HAL_GPIO_WritePin(MOTOR_3_GPIO_Port,MOTOR_3_Pin, GPIO_PIN_RESET);
//	}

	if(state & 0x08)
	{
		HAL_GPIO_WritePin(MOTOR_4_GPIO_Port,MOTOR_4_Pin, GPIO_PIN_SET);
	}
//	else
//	{
//		HAL_GPIO_WritePin(MOTOR_4_GPIO_Port,MOTOR_4_Pin, GPIO_PIN_RESET);
//	}
}

void step_acw()
{
	uint8_t i = --step_count%8; //0-7 val
	write_motors(motor_movement_cw[i]);
	step_count %= STEPS_PER_ROTATION;
}

void step_cw()
{
	write_motors(motor_movement_cw[step_count++%8]);
	step_count %= STEPS_PER_ROTATION;
}


bool zero_motor()
{
	//get us to zero location
	//if we are already at 0, move 10degrees acw first
	while(get_zero_state())
	{
		step_acw();
		HAL_Delay(1);
	}

	//rotate clockwise until the zero location is triggered
	while(!get_zero_state())
	{
		step_cw();
		HAL_Delay(1);
	}
	 motors_off();
	 current_angle_deg = 0;
	 step_count = 0;

	return true;
}


int16_t angle_difference(int16_t b1, int16_t b2) {
    int16_t r = (b2 - b1) % 360;
    if (r < -180) {
      r += 360;
    }
    if (r >= 180) {
      r -= 360;
    }
    return r;
}

void move_motor(int16_t angle)
{
	//move motor to a specified angle

	int16_t angle_to_move = angle_difference(current_angle_deg, angle);
	bool cw_nacw= angle_to_move > 0; //this is clockwise or not.
	angle_to_move = abs(angle_to_move);
	printf("From %d to %d = %d\n\r", current_angle_deg, angle, angle_to_move);
	uint16_t steps = (int16_t)((double)(angle_to_move) / STEP_ANGLE);
	printf("Moving %d steps\n\r", steps);
	if(!cw_nacw)
	{
		printf("anti");
	}
	printf("clockwise\n\r");
	for(uint32_t i = 0; i < steps; i++)
	{
		step_motor(cw_nacw);
		HAL_Delay(1);
	}
	current_angle_deg = angle;
}



void test_motors()
{
	for(uint32_t i = 0; i < INTERNAL_GEAR_RATIO * STEPS_PER_FULL; i++)
		{
			write_motors(motor_movement_cw[i%8]);
			HAL_Delay(1);
		}
	motors_off();
	HAL_Delay(1000);
	for(uint32_t i = 0; i < INTERNAL_GEAR_RATIO * STEPS_PER_FULL; i++)
	{
		write_motors(motor_movement_cw[7-i%8]);
		HAL_Delay(1);
	}
	motors_off();
	move_motor(45);
	move_motor(90);
	move_motor(0);
	move_motor(240);
	move_motor(45);
	move_motor(0);
	HAL_Delay(1000);
}

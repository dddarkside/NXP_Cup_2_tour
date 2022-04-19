extern "C"
{
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"

#include "Modules/mSpi.h"
#include "Modules/mDac.h"
#include "Modules/mAccelMagneto.h"
#include "Modules/mGyro.h"
#include "Modules/mTimer.h"
#include "Modules/mCpu.h"
#include "Modules/mSwitch.h"
#include "Modules/mLeds.h"
#include "Modules/mAd.h"
#include "Modules/mDelay.h"
#include "Modules/mRS232.h"
#include "Modules/mVL6180x.h"
}

#include "Pixy/Pixy2SPI_SS.h"

#define PIXY_WIDTH     	315
#define PIXY_HEIGHT    	207
#define PIXY_ROW_NB    	100
#define PIXY_COL_START 	3
#define PIXY_COL_STEP  	5

#define PIXY_MID_X		31
#define PIXY_MID_ERR 	3

/* Коэфициенты цветов */
#define RED_W   0.3f
#define GREEN_W 0.59f
#define BLUE_W  0.11f

#define BLACK_BRIGHTNESS_ERR	6

#define BRIGHTNESS_BUFF_SIZE	62

#define MAX_LINE_WIDTH			10

#define THRESHOLD_LEFT_HIGH_TWO_LINE	16
#define THRESHOLD_LEFT_LOW_TWO_LINE		21
#define THRESHOLD_RIGHT_LOW_TWO_LINE	42
#define THRESHOLD_RIGHT_HIGH_TWO_LINE	45

#define THRESHOLD_ERR_LOW_ONE_LINE		3
#define THRESHOLD_ERR_HIGH_ONE_LINE		6

#define EDGE_LEFT_ZONE					16
#define EDGE_RIGHT_ZONE					45

#define MOTOR_DUTY_FORWARD  0.6f
#define MOTOR_DUTY_STOP     0.0f
#define MOTOR_DUTY_SMOOTHLY 0.5f
#define SERVO_DUTY_LEFT_ONE 0.5f
#define MOTOR_DUTY_SHARPLY  0.5f

#define SERVO_DUTY_FORWARD  		0.0f
#define SERVO_DUTY_STOP     		0.0f
#define SERVO_DUTY_LEFT_SMOOTHLY 	0.5f
#define SERVO_DUTY_LEFT_ONE         0.7f
#define SERVO_DUTY_LEFT_SHARPLY  	1.0f
#define SERVO_DUTY_RIGHT_SMOOTHLY 	-0.5f
#define SERVO_DUTY_RIGHT_ONE        -0.7f
#define SERVO_DUTY_RIGHT_SHARPLY  	-1.0f

#define SERVO_PORT_1	0
#define SERVO_PORT_2	1

enum state
{
	LEFT_SMOOTHLY,
	LEFT_ONE,
	LEFT_SHARPLY,

	RIGHT_SMOOTHLY,
	RIGHT_SHARPLY,

	FORWARD,
	STOP
};


void setup_mk()
{
	//--------------------------------------------------------------------
		// Device and card setup
		//--------------------------------------------------------------------
		// PLL Config --> CPU 100MHz, bus and periph 50MH z
		mCpu_Setup();

		// Config and start switches and pushers
		mSwitch_Setup();
		mSwitch_Open();

		// Config and start of LEDs
		mLeds_Setup();
		mLeds_Open();

		// Config and start of ADC
		mAd_Setup();
		mAd_Open();

		// Config and start of SPI
		mSpi_Setup();
		mSpi_Open();

		// Config and start non-blocking delay by PIT
		mDelay_Setup();
		mDelay_Open();

		// Timer Config for Speed Measurement and PWM Outputs for Servos
		mTimer_Setup();
		mTimer_Open();

		// Setup FXOS8700CQ
		mAccelMagneto_Setup();
		mAccelMagneto_Open();

		// Setup FXAS21002C
		mGyro_Setup();
		mGyro_Open();

		// Config and start of the DAC0 used to drive the driver LED lighting
		mDac_Setup();
		mDac_Open();

		// Setup and start of motor and servo PWM controls and speed measurement
		mTimer_Setup();
		mTimer_Open();

		// Enable IRQ at the CPU -> Primask
		__enable_irq();

		// UART 4 monitoring image
		mRs232_Setup();
		mRs232_Open();
}

void fill_brightness_buff(uint8_t brightness_buff[], Pixy2SPI_SS &pixy)//Яркость
{
	uint8_t r, g, b;

	for (int i = 0; i < BRIGHTNESS_BUFF_SIZE; i++)
	{
		pixy.video.getRGB(PIXY_COL_START + PIXY_COL_STEP * i, PIXY_ROW_NB, &r, &g, &b, false);
		brightness_buff[i] = (r * RED_W + g * GREEN_W + b * BLUE_W) / 3;
	}
}

void fill_road_edges(uint8_t road_edges[], uint8_t brightness_buff[])//Найти края
{
	uint8_t i = 0;
	uint8_t j = BRIGHTNESS_BUFF_SIZE - 1;
	bool was_the_left_line_found = false;
	bool was_the_right_line_found = false;

	while (i < BRIGHTNESS_BUFF_SIZE)
	{
		if (brightness_buff[i] > BLACK_BRIGHTNESS_ERR)//найдена ли линия
		{
			was_the_left_line_found = true;
			road_edges[0] = i;
			break;
		}
		i++;
	}

	while (j > 0)
	{
		if (brightness_buff[j] > BLACK_BRIGHTNESS_ERR)//найдена ли линия
		{
			was_the_right_line_found = true;
			road_edges[1] = j;
			break;
		}
		j--;
	}

	// Если линия не была найдена, пишем что она в центре
	// Тогда мидпоинт будет ближе к найденой линии, а если обеих нет, то поедем по центру
	// Возможные проблемы: Если препятствие оставляет очень маленькую область свободной, но такого вроде не будет
	if (was_the_right_line_found==false)
	{
		road_edges[1] = 63;
	}
	if (was_the_left_line_found==false)
	{
		road_edges[0] = -1;
	}
}

enum state choosing_state(uint8_t road_edges[])
{
	if (road_edges[0] < 13 && road_edges[1]>49)return FORWARD;
	else if (road_edges[0] == -1 && road_edges[1]>49)return RIGHT_SMOOTHLY;
	else if (road_edges[0] == -1 && road_edges[1]<=49)return RIGHT_SHARPLY;
	else if (road_edges[0] < 13 && road_edges[1] == 63)return LEFT_SMOOTHLY;
	else if (road_edges[0] >= 13 && road_edges[1] == 63)return LEFT_SHARPLY;
	else return FORWARD;
}

void handle_state(enum state state)//////////////////////////////////////////////////////
{
	switch (state)
	{
	case FORWARD:
		mTimer_SetServoDuty(SERVO_PORT_1, SERVO_DUTY_FORWARD);
		mTimer_SetMotorDuty(-MOTOR_DUTY_FORWARD, MOTOR_DUTY_FORWARD);
		break;
	case STOP:
		mTimer_SetServoDuty(SERVO_PORT_1, SERVO_DUTY_STOP);
		mTimer_SetMotorDuty(-MOTOR_DUTY_STOP, MOTOR_DUTY_STOP);
		break;
	case LEFT_SMOOTHLY:
		mTimer_SetServoDuty(SERVO_PORT_1, SERVO_DUTY_LEFT_SMOOTHLY);
		mTimer_SetMotorDuty(-MOTOR_DUTY_SMOOTHLY, MOTOR_DUTY_SMOOTHLY);
		break;
	case LEFT_SHARPLY:
		mTimer_SetServoDuty(SERVO_PORT_1, SERVO_DUTY_LEFT_SHARPLY);
		mTimer_SetMotorDuty(-MOTOR_DUTY_SHARPLY, MOTOR_DUTY_SHARPLY);
		break;
	case RIGHT_SMOOTHLY:
		mTimer_SetServoDuty(SERVO_PORT_1, SERVO_DUTY_RIGHT_SMOOTHLY);
		mTimer_SetMotorDuty(-MOTOR_DUTY_SMOOTHLY, MOTOR_DUTY_SMOOTHLY);
		break;
	case RIGHT_SHARPLY:
		mTimer_SetServoDuty(SERVO_PORT_1, SERVO_DUTY_RIGHT_SHARPLY);
		mTimer_SetMotorDuty(-MOTOR_DUTY_SHARPLY, MOTOR_DUTY_SHARPLY);
		break;
	}
}



int main(void)
{
	bool is_runing = false;
	enum state state = FORWARD;

	uint8_t	brightness_buff[BRIGHTNESS_BUFF_SIZE];
	uint8_t	road_edges[2];

	setup_mk();

	Pixy2SPI_SS pixy;
	pixy.init();
	//pixy.setLamp(1, 1);

	// Горящая дампочка говорит, что ты не сконфигурировал камеру по краям дороги
	mLeds_Write(kMaskLed2, kLedOn);

	while (true)
	{
		if (!mSwitch_ReadSwitch(kSw1) && mSwitch_ReadSwitch(kSw4) && mSwitch_ReadPushBut(kPushButSW1))
			is_runing = true;

		// Режим работы
		else if (mSwitch_ReadSwitch(kSw4) && !mSwitch_ReadSwitch(kSw1) && is_runing)
		{
			fill_brightness_buff(brightness_buff, pixy);
			fill_road_edges(road_edges, brightness_buff);

			//print_brightness_buff(brightness_buff);
			//print_road_edges(road_edges);
			//print_mid_point(mid_point, line_count);

			state = choosing_state(road_edges);
		}

		//print_state(state, line_count, mid_point);

		handle_state(state);
	}

	return (0);
}

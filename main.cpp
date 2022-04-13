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
#define PIXY_ROW_NB    	50
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

#define MAX_LINE_WIDTH			12

#define THRESHOLD_LEFT_HIGH_TWO_LINE	15
#define THRESHOLD_LEFT_LOW_TWO_LINE		28
#define THRESHOLD_RIGHT_LOW_TWO_LINE	34
#define THRESHOLD_RIGHT_HIGH_TWO_LINE	47

#define THRESHOLD_ERR_LOW_ONE_LINE		7
#define THRESHOLD_ERR_HIGH_ONE_LINE		15

#define EDGE_LEFT_ZONE					16
#define EDGE_RIGHT_ZONE					45

#define MOTOR_DUTY_FORWARD 0.5f
#DEFINE START_SPEED 0.75f
#define MOTOR_DUTY_STOP     0.0f
#define MOTOR_DUTY_SMOOTHLY 0.5f
#define MOTOR_DUTY_SHARPLY  0.5f

#define SERVO_DUTY_FORWARD  		0.0f
#define SERVO_DUTY_STOP     		0.0f
#define SERVO_DUTY_LEFT_SMOOTHLY 	0.3f
#define SERVO_DUTY_LEFT_HALF_SHARPLY 0.7f
#define SERVO_DUTY_LEFT_SHARPLY  	1.0f
#define SERVO_DUTY_RIGHT_SMOOTHLY 	-0.3f
#define SERVO_DUTY_RIGHT_HALF_SHARPLY -0.7f
#define SERVO_DUTY_RIGHT_SHARPLY  	-1.0f

#define SERVO_PORT_1	0
#define SERVO_PORT_2	1

#define STOP_TIME 20

uint8_t timer = 0;

enum state
{
	LEFT_SMOOTHLY,
	LEFT_HALF_SHARPLY,
	LEFT_SHARPLY,

	RIGHT_SMOOTHLY,
	RIGHT_HALF_SHARPLY,
	RIGHT_SHARPLY,

	FORWARD,
	STOP
};

enum last_line
{
	RIGHT,
	RIGHT_EDGE,
	LEFT,
	LEFT_EDGE,
	BOTH
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

void fill_brightness_buff(uint8_t brightness_buff[], Pixy2SPI_SS &pixy)
{
	uint8_t r, g, b;

	for (int i = 0; i < BRIGHTNESS_BUFF_SIZE; i++)
	{
		pixy.video.getRGB(PIXY_COL_START + PIXY_COL_STEP * i, PIXY_ROW_NB, &r, &g, &b, false);
		brightness_buff[i] = (r * RED_W + g * GREEN_W + b * BLUE_W) / 3;
	}
}

void fill_road_edges(uint8_t road_edges[], uint8_t brightness_buff[])
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
			break
		}
		i++;
	}

	while (j > 0)
	{
		if (brightness_buff[j] > BLACK_BRIGHTNESS_ERR)//найдена ли линия
		{
			was_the_right_line_found = true;
			road_edges[1] = j;
			break
		}
		j--;
	}

	// Если линия не была найдена, пишем что она в центре
	// Тогда мидпоинт будет ближе к найденой линии, а если обеих нет, то поедем по центру
	// Возможные проблемы: Если препятствие оставляет очень маленькую область свободной, но такого вроде не будет
	if (was_the_right_line_found==false)
	{
		road_edges[1] = PIXY_MID_X;
	}
	if (was_the_left_line_found==false)
	{
		road_edges[0] = PIXY_MID_X;
	}
}

int get_lines_count(uint8_t road_edges[], uint8_t brightness_buff[])
{
	uint8_t left_x = road_edges[0];
	uint8_t right_x = road_edges[1];

	if (left_x > right_x)
		return (0);
	else
	{
		while (left_x < right_x)
		{
			// Проверка на то, что камера действительно зафиксировала две линии, а не края одной
			if (brightness_buff[left_x] > BLACK_BRIGHTNESS_ERR)
				return (2);
			left_x++;
		}
		return (1);
	}
}

enum last_line get_last_line(uint8_t mid_point, uint8_t line_count, enum last_line last_line)
{
	if (line_count == 2)
		return (BOTH);
	else if (line_count == 1 && (last_line == BOTH || last_line == LEFT_EDGE || last_line == RIGHT_EDGE))
	{
		if (mid_point <= EDGE_LEFT_ZONE)
			return (LEFT_EDGE);
		else if (mid_point < PIXY_MID_X)
			return (LEFT);
		else if (mid_point >= EDGE_RIGHT_ZONE)
			return (RIGHT_EDGE);
		else
			return (RIGHT);
	}
	else if (line_count == 1 && last_line == LEFT && mid_point <= EDGE_LEFT_ZONE)
		return (LEFT_EDGE);
	else if (line_count == 1 && last_line == RIGHT && mid_point >= EDGE_RIGHT_ZONE)
			return (RIGHT_EDGE);
	else
		return (last_line);
}

enum state get_state_by_two_lines(uint8_t mid_point)
{
	if (mid_point < THRESHOLD_LEFT_HIGH_TWO_LINE)
		return (LEFT_SHARPLY);
	else if (mid_point >= THRESHOLD_LEFT_HIGH_TWO_LINE && mid_point <= THRESHOLD_LEFT_LOW_TWO_LINE)
		return (LEFT_SMOOTHLY);
	else if (mid_point > THRESHOLD_RIGHT_HIGH_TWO_LINE)
		return (RIGHT_SHARPLY);
	else if (mid_point >= THRESHOLD_RIGHT_LOW_TWO_LINE && mid_point <= THRESHOLD_RIGHT_HIGH_TWO_LINE)
		return (RIGHT_SMOOTHLY);
	else
		return (FORWARD);
}

enum state get_state_by_one_line(uint8_t mid_point, uint8_t standart_line_x)
{
	if (mid_point < standart_line_x - THRESHOLD_ERR_HIGH_ONE_LINE)
		return (LEFT_SHARPLY);
	else if (mid_point >= standart_line_x - THRESHOLD_ERR_HIGH_ONE_LINE
			&& mid_point <= standart_line_x - THRESHOLD_ERR_LOW_ONE_LINE)
		return (LEFT_SMOOTHLY);
	else if (mid_point > standart_line_x + THRESHOLD_ERR_HIGH_ONE_LINE)
		return (RIGHT_SHARPLY);
	else if (mid_point >= standart_line_x + THRESHOLD_ERR_LOW_ONE_LINE
			&& mid_point <= standart_line_x + THRESHOLD_ERR_HIGH_ONE_LINE)
		return (RIGHT_SMOOTHLY);
	else
		return (FORWARD);
}

// Возвращает true, если была зафиксирована одна, но широкая черная линия,
// или если между двумя черными линиями (краями дороги) была найдена еще одна
bool is_stop_line
(
	uint8_t brightness_buff[],
	uint8_t left_line_x,
	uint8_t right_line_x,
	uint8_t line_count
)
{
	uint8_t i = left_line_x;

	if (line_count == 1)
	{
		if (right_line_x - left_line_x > MAX_LINE_WIDTH)
			return (true);
		else
			return (false);
	}

	// Пропускаем черные пиксели левой линии дороги
	while (brightness_buff[i] <= BLACK_BRIGHTNESS_ERR && i < right_line_x)
		i++;

	if (i - left_line_x > MAX_LINE_WIDTH)
		return (true);

	// Пропускаем белые пиксели, пока не наткнемся на линию
	while (brightness_buff[i] > BLACK_BRIGHTNESS_ERR && i < right_line_x)
		i++;

	if (right_line_x - i > MAX_LINE_WIDTH)
		return (true);

	return (false);
}

enum state get_state_by_lines
(
		uint8_t mid_point,
		enum last_line last_line,
		const uint8_t &left_line_x,
		const uint8_t &right_line_x
)
{
	if (last_line == LEFT || last_line == LEFT_EDGE)
		return (get_state_by_one_line(mid_point, left_line_x));
	else if (last_line == RIGHT || last_line == RIGHT_EDGE)
		return (get_state_by_one_line(mid_point, right_line_x));
	else
		return (get_state_by_two_lines(mid_point));
}

void handle_state(enum state state)
{
	switch (state)
	{
	case FORWARD:
		mTimer_SetServoDuty(SERVO_PORT_1, SERVO_DUTY_FORWARD);
		if (timer>20) mTimer_SetMotorDuty(-START_SPEED, START_SPEED);
		else mTimer_SetMotorDuty(-MOTOR_DUTY_FORWARD, MOTOR_DUTY_FORWARD);
		break;
	case STOP:
		mTimer_SetServoDuty(SERVO_PORT_1, SERVO_DUTY_STOP);
		mTimer_SetMotorDuty(-MOTOR_DUTY_STOP, MOTOR_DUTY_STOP);
		break;
	case LEFT_SMOOTHLY:
		mTimer_SetServoDuty(SERVO_PORT_1, SERVO_DUTY_LEFT_SMOOTHLY);
		mTimer_SetMotorDuty(-MOTOR_DUTY_SMOOTHLY, MOTOR_DUTY_SMOOTHLY);
		break;
	case RIGHT_HALF_SHARPLY:
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
	case LEFT_HALF_SHARPLY:
		mTimer_SetServoDuty(SERVO_PORT_1, SERVO_DUTY_LEFT_SMOOTHLY);
		mTimer_SetMotorDuty(-MOTOR_DUTY_SMOOTHLY, MOTOR_DUTY_SMOOTHLY);
		break;
	case RIGHT_SHARPLY:
		mTimer_SetServoDuty(SERVO_PORT_1, SERVO_DUTY_RIGHT_SHARPLY);
		mTimer_SetMotorDuty(-MOTOR_DUTY_SHARPLY, MOTOR_DUTY_SHARPLY);
		break;
	}
}

void print_brightness_buff(uint8_t brightness_buff[])
{
	for (int i = 0; i < BRIGHTNESS_BUFF_SIZE; i++)
	{
		printf("%d ", brightness_buff[i]);
	}
	printf("\n----------------------------------------------------\n");
}

void print_road_edges(uint8_t road_edges[])
{
	printf("%d %d\n", road_edges[0], road_edges[1]);
}

void print_mid_point(uint8_t mid_point, uint8_t line_count)
{
	printf("x: %d\tline_cnt: %d\n", mid_point, line_count);
}

void print_state(enum state state, uint8_t line_count, uint8_t mid_point)
{
	char *state_str;

	if (state == LEFT_SMOOTHLY)
		state_str = "LEFT_SMOOTHLY";
	else if (state == LEFT_SHARPLY)
		state_str = "LEFT_SHARPLY";
	else if (state == RIGHT_SMOOTHLY)
		state_str = "RIGHT_SMOOTHLY";
	else if (state == RIGHT_SHARPLY)
		state_str = "RIGHT_SHARPLY";
	else if (state == FORWARD)
		state_str = "FORWARD";
	else
		state_str = "STOP";
	printf("State: %s,\tline count: %d,\t%d\n", state_str, line_count, mid_point);
}

int main(void)
{
	bool			is_runing = false;

	uint8_t			brightness_buff[BRIGHTNESS_BUFF_SIZE];
	uint8_t			road_edges[2];
	uint8_t			mid_point;
	uint8_t			line_count;

	enum state 		state = STOP;
	enum last_line	last_line = BOTH;

	const uint8_t left_line_standart_x = 0;
	const uint8_t right_line_standart_x = 0;

	setup_mk();

	Pixy2SPI_SS pixy;
	pixy.init();
	//pixy.setLamp(1, 1);

	// Горящая дампочка говорит, что ты не сконфигурировал камеру по краям дороги
	mLeds_Write(kMaskLed2, kLedOn);

	while (true)
	{
		if (!mSwitch_ReadSwitch(kSw1) && mSwitch_ReadSwitch(kSw4) && mSwitch_ReadPushBut(kPushButSW2))
			is_runing = true;

		// Режим конфигурации камеры по краям дороги
		if (mSwitch_ReadSwitch(kSw1) && !mSwitch_ReadSwitch(kSw4))
		{
			while (mSwitch_ReadSwitch(kSw1))
			{
				fill_brightness_buff(brightness_buff, pixy);
				fill_road_edges(road_edges, brightness_buff);
				line_count = get_lines_count(road_edges, brightness_buff);
				mid_point = (road_edges[0] + road_edges[1]) / 2;

				if (line_count == 2 && mid_point >= PIXY_MID_X - PIXY_MID_ERR && mid_point <= PIXY_MID_X + PIXY_MID_ERR)
					mLeds_Write(kMaskLed2, kLedOff);		// Потухшая лампочка говорит, что конфигурация выполнена и можно переключаться в режим работы
				else
					mLeds_Write(kMaskLed2, kLedOn);
			}

			const_cast<uint8_t&>(left_line_standart_x) = road_edges[0];
			const_cast<uint8_t&>(right_line_standart_x) = road_edges[1];
		}
		// Режим работы
		else if (mSwitch_ReadSwitch(kSw4) && !mSwitch_ReadSwitch(kSw1) && is_runing)
		{
			fill_brightness_buff(brightness_buff, pixy);
			fill_road_edges(road_edges, brightness_buff);
			line_count = get_lines_count(road_edges, brightness_buff);
			mid_point = (road_edges[0] + road_edges[1]) / 2;
			last_line = get_last_line(mid_point, line_count, last_line);

			//print_brightness_buff(brightness_buff);
			//print_road_edges(road_edges);
			//print_mid_point(mid_point, line_count);

			if (is_stop_line(brightness_buff, road_edges[0], road_edges[1], line_count))
			{
				mTimer_SetMotorDuty(MOTOR_DUTY_SHARPLY, -MOTOR_DUTY_SHARPLY);
				usleep(50000);
				is_runing = false;
				state = STOP;
				last_line = BOTH;
			}
			else
				state = get_state_by_lines(mid_point, last_line, left_line_standart_x, right_line_standart_x);
		}
		else
			state = STOP;

		//print_state(state, line_count, mid_point);

		handle_state(state);
	}

	return (0);
}

#include "def.h"

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

#define MOTOR_DUTY_FORWARD  0.6f
#define MOTOR_DUTY_SMOOTHLY 0.5f
#define SERVO_DUTY_LEFT_ONE 0.5f
#define MOTOR_DUTY_SHARPLY  0.5f

#define SERVO_DUTY_FORWARD  		0.0f
#define SERVO_DUTY_LEFT_SMOOTHLY 	0.5f
#define SERVO_DUTY_LEFT_SHARPLY  	1.0f
#define SERVO_DUTY_RIGHT_SMOOTHLY 	-0.5f
#define SERVO_DUTY_RIGHT_SHARPLY  	-1.0f

#define SERVO_PORT_1	0
#define SERVO_PORT_2	1

#define LEFT_WHEEL 13
#define RIGHT_WHEEL 49

enum state
{
	LEFT_SMOOTHLY,
	LEFT_SHARPLY,

	RIGHT_SMOOTHLY,
	RIGHT_SHARPLY,

	FORWARD,
	STOP
};

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

enum state choosing_state(uint8_t road_edges[],enum state last_one)
{
	if (road_edges[0] >-1 && road_edges[1] < 63 )//Две линии
	{
		uint8_t mid = (road_edges[0]+road_edges[1]/2);

		if (mid == BRIGHTNESS_BUFF_SIZE/2 || mid == (BRIGHTNESS_BUFF_SIZE/2)+1)return FORWARD;
		else if (mid < BRIGHTNESS_BUFF_SIZE/2)return LEFT_SMOOTHLY;
		else return RIGHT_SMOOTHLY;

	}
	else if (road_edges[0] == -1 && (road_edges[1] > RIGHT_WHEEL && road_edges[1] < RIGHT_WHEEL+3))return FORWARD;//только правая линия
	else if (road_edges[0] == -1 && road_edges[1] < RIGHT_WHEEL)return LEFT_SHARPLY ;
	else if (road_edges[0] == -1 && road_edges[1] > RIGHT_WHEEL+3)return LEFT_SMOOTHLY ;

	else if ((road_edges[0] < LEFT_WHEEL && road_edges[0] > LEFT_WHEEL-3) && road_edges[1] == 63)return FORWARD;//только левая линия
	else if (road_edges[0] < LEFT_WHEEL-3 && road_edges[1] == 63)return RIGHT_SMOOTHLY;
	else return RIGHT_SHARPLY;// if (road_edges[0] > LEFT_WHEEL && road_edges[1] == 63)
}

void handle_state(enum state state)//////////////////////////////////////////////////////
{
	switch (state)
	{
	case FORWARD:
		mTimer_SetServoDuty(SERVO_PORT_1, SERVO_DUTY_FORWARD);
		mTimer_SetMotorDuty(-MOTOR_DUTY_FORWARD, MOTOR_DUTY_FORWARD);
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
	enum state last_one = state;

	uint8_t	brightness_buff[BRIGHTNESS_BUFF_SIZE];
	uint8_t	road_edges[2];

	setup_mk();

	Pixy2SPI_SS pixy;
	pixy.init();
	//pixy.setLamp(1, 1);

	while (true)
	{
		if (!mSwitch_ReadSwitch(kSw1) && mSwitch_ReadSwitch(kSw4) && mSwitch_ReadPushBut(kPushButSW1))
			is_runing = true;

		// Режим работы
		else if (mSwitch_ReadSwitch(kSw4) && !mSwitch_ReadSwitch(kSw1) && is_runing)
		{
			fill_brightness_buff(brightness_buff, pixy);//собираем значения яркости
			fill_road_edges(road_edges, brightness_buff);//находим линии

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

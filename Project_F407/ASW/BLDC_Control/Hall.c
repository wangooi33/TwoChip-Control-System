/* includes ------------------------------------------------------------------*/
#include "hall.h"
#include "tim.h"
#include "foc.h"

/* global variable -----------------------------------------------------------*/
Hall_Info_t Hall_Info;
const uint8_t hall_sequence[6] = {0x06,0x04,0x05,0x01,0x03,0x02};
const float hall_angle_table[8] = 
{
	0.0f,
	3.0f * HALL_STEP_ANGLE,
	5.0f * HALL_STEP_ANGLE,
	4.0f * HALL_STEP_ANGLE,
	1.0f * HALL_STEP_ANGLE,
	2.0f * HALL_STEP_ANGLE,
	0.0f,
	0.0f
};

/* macro ---------------------------------------------------------------------*/
#define HALL_SPEED_FILTER_ALPHA	(0.15f)
/* 低速换向时可直接接受与命令方向一致的霍尔沿。 */
#define HALL_DIR_CHANGE_SPEED_LIMIT	(30.0f)

/* public functions ----------------------------------------------------------*/
float Angle_Normalize(float angle)
{
	while (angle >= TWO_PI)
	{
		angle -= TWO_PI;
	}
	while (angle < 0.0f)
	{
		angle += TWO_PI;
	}
	return angle;
}
static float Hall_GetStateAngle(uint8_t hall_state)
{
	return Angle_Normalize(hall_angle_table[hall_state] + PI / 6.0f);
}
int Hall_GetIndex(uint8_t state)
{
	for (int i = 0; i < 6; i++)
	{
		if (hall_sequence[i] == state)
		{
			return i;
		}
	}
	return -1;
}
int8_t Hall_GetDirection(uint8_t old_state, uint8_t new_state)
{
	int old_index = Hall_GetIndex(old_state);
	int new_index = Hall_GetIndex(new_state);

	if (old_index < 0 || new_index < 0)
	{
		return 0;
	}
	/* 正转 */
	if (((old_index + 1) % 6) == new_index)
	{
		return 1;
	}
	/* 反转 */
	if (((old_index + 5) % 6) == new_index)
	{
		return -1;
	}

	return 0;
}


void Hall_Enable(void)
{
	//__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_TRIGGER);
	HAL_TIMEx_HallSensor_Start_IT(&htim3);
}
uint8_t Hall_ReadState(void)
{
	uint8_t hall_state = 0;

	hall_state  = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	hall_state |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) << 1;
	hall_state |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) << 2;

	return hall_state;
}

void Hall_UpdateEdge(uint8_t hall_state, uint32_t hall_period)
{
	int8_t dir;
	int8_t commandedDirection;
	int8_t previousDirection;
	uint8_t confirmedReversal = 0;
	uint8_t commandDirectionAccepted = 0;
	uint8_t speedFilterReset = 0;
	int32_t hallStepDelta;
	float angleStep;

	/* 全0或全1的霍尔编码不是有效扇区，直接忽略 */
	if (hall_state == 0x0 || hall_state == 0x7)
	{
		return;
	}

	if (!Hall_Info.initialized)
	{
		/* 首个有效状态只用于建立初始角度，不计算速度和方向 */
		Hall_Info.state = hall_state;
		Hall_Info.last_state = hall_state;
		Hall_Info.hall_angle = Hall_GetStateAngle(hall_state);
		Hall_Info.angle = Hall_Info.hall_angle;
		Hall_Info.initialized = 1;
		Hall_Info.last_event_ms = SystemRunTime_1ms;
		return;
	}
	if (hall_state == Hall_Info.state)
	{
		/* 重复边沿视为毛刺，并丢弃尚未确认的反转记录 */
		Hall_Info.pending_valid = 0;
		Hall_Info.pending_direction = 0;
		Hall_Info.pending_period = 0;
		return;
	}
	dir = Hall_GetDirection(Hall_Info.state,hall_state);
	if (dir == 0)
	{
		/* 非法状态跳变只清除待确认反转，不更新当前状态 */
		Hall_Info.pending_valid = 0;
		Hall_Info.pending_direction = 0;
		Hall_Info.pending_period = 0;
		return;
	}
	if (hall_period < HALL_MIN_PERIOD_COUNT || hall_period > 65535U)
	{
		/* 周期不在合理范围内时判定为干扰，忽略本次边沿 */
		Hall_Info.pending_valid = 0;
		Hall_Info.pending_direction = 0;
		Hall_Info.pending_period = 0;
		return;
	}

	commandedDirection = Hall_Info.commanded_direction;
	/* 与命令同向且在低速或方向未建立时，本次边沿可直接被接受 */
	if (dir == commandedDirection &&
	    (Hall_Info.direction == 0 ||
	     (Hall_Info.speed_filter > -HALL_DIR_CHANGE_SPEED_LIMIT && Hall_Info.speed_filter < HALL_DIR_CHANGE_SPEED_LIMIT)))
	{
		commandDirectionAccepted = 1;
	}
	/* 与已确认方向相反且未被命令同向条件接受时，进入反转确认流程 */
	if (Hall_Info.direction != 0 && dir != Hall_Info.direction && commandDirectionAccepted == 0)
	{
		if (Hall_Info.pending_valid == 0)
		{
			/* 记录第一个反向边沿，暂不改变当前方向 */
			Hall_Info.pending_valid = 1;
			Hall_Info.pending_direction = dir;
			Hall_Info.pending_period = hall_period;
			return;
		}

		if (Hall_Info.pending_direction != dir)
		{
			/* 方向相反的反向边沿之间出现新的状态，重新记录 */
			Hall_Info.pending_direction = dir;
			Hall_Info.pending_period = hall_period;
			return;
		}

		/* 连续两个同向反向边沿才确认真反转。 */
		confirmedReversal = 1;
		/* 两次反向边沿对应两个扇区步进，周期相加后再计算速度 */
		hall_period += Hall_Info.pending_period;
	}
	previousDirection = Hall_Info.direction;
	Hall_Info.pending_valid = 0;
	Hall_Info.pending_direction = 0;
	Hall_Info.pending_period = 0;

	/* 提交新的霍尔状态、方向以及周期计数值 */
	Hall_Info.last_state = Hall_Info.state;
	Hall_Info.state = hall_state;
	Hall_Info.direction = dir;
	Hall_Info.hall_period = hall_period;
	if (previousDirection != 0 && previousDirection != dir)
	{
		/* 已确认方向发生反转时，丢弃旧的速度滤波历史 */
		speedFilterReset = 1;
	}

	/* 确认反转时,已经累计了两个扇区步进 */
	hallStepDelta = (int32_t)dir * (confirmedReversal ? 2 : 1);
	Hall_Info.hall_step_count += hallStepDelta;
	Hall_Info.unwrapped_electrical_deg = (float)Hall_Info.hall_step_count * 60.0f;
	Hall_Info.position_turns = Hall_Info.unwrapped_electrical_deg / ((float)BLDC_POLE_PAIRS * 360.0f);

	/* 计时器频率 = 1MHz */
	angleStep = confirmedReversal ? (2.0f * HALL_STEP_ANGLE) : HALL_STEP_ANGLE;
	/* 根据当前方向、扇区跨度和周期计数值计算瞬时电角速度 */
	Hall_Info.speed = (float)dir * angleStep * (float)HALL_TIMER_HZ / (float)hall_period;
	if (speedFilterReset || Hall_Info.speed_filter_init == 0)
	{
		/* 首次测量或方向反转后直接装载当前速度 */
		Hall_Info.speed_filter = Hall_Info.speed;
		Hall_Info.speed_filter_init = 1;
	}
	else
	{
		/* 正常情况下对瞬时速度做一阶低通滤波 */
		Hall_Info.speed_filter += HALL_SPEED_FILTER_ALPHA * (Hall_Info.speed - Hall_Info.speed_filter);
	}
	Hall_Info.hall_angle = Hall_GetStateAngle(hall_state);
	/* 每个扇区重新锚定插值角度,防止加速过程中误差持续累积。 */
	Hall_Info.angle = Hall_Info.hall_angle;
	/* 更新时间戳，供插值函数判断霍尔信号是否长时间无更新 */
	Hall_Info.last_event_ms = SystemRunTime_1ms;

	/* 通知启动状态机已有新的有效霍尔边沿 */
	Hall_Info.new_event = 1;
}
void Hall_Interpolate(float Ts)
{
	/* 在两个霍尔边沿之间,对FOC电角度做连续插值的函数 */

	if (!Hall_Info.initialized)
	{
		/* 尚未完成霍尔初始化，没有可用于插值的基准角度 */
		return;
	}

	if ((SystemRunTime_1ms - Hall_Info.last_event_ms) > HALL_SPEED_TIMEOUT_MS)
	{
		/* 长时间没有霍尔边沿说明电机已停转，只清零速度，不继续积分角度 */
		Hall_Info.speed = 0.0f;
		Hall_Info.speed_filter = 0.0f;
		return;
	}

	/* θ(k+1) = θ(k) + ωe × Ts */
	/*计算FOC所使用的电角度,使用speed避免因为低通滤波造成相位滞后 */
	Hall_Info.angle += Hall_Info.speed * Ts;
	/* Park变换使用的角度保持在一个电周期内 */
	Hall_Info.angle = Angle_Normalize(Hall_Info.angle);
	/* 计算相对机械圈数,使用滤波后的速度避免位置抖动 */
	Hall_Info.unwrapped_electrical_deg += Hall_Info.speed_filter * (180.0f / PI) * Ts;
	Hall_Info.position_turns = Hall_Info.unwrapped_electrical_deg / ((float)BLDC_POLE_PAIRS * 360.0f);
}


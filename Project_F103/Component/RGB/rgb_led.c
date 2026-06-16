#include <math.h>
#include "rgb_led.h"
#include "tim.h"

/* global variable ----------------------------------------------------------*/
RGB_Breath_t Breath1;

/* const --------------------------------------------------------------------*/

//Gamma 2.2 校正查找表
const uint8_t gamma22[256] =
{
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,
	1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,
	2,3,3,3,3,3,3,3,4,4,4,4,4,5,5,5,
	5,6,6,6,6,7,7,7,7,8,8,8,9,9,9,10,
	10,10,11,11,11,12,12,13,13,13,14,14,15,15,16,16,
	17,17,18,18,19,19,20,20,21,21,22,22,23,24,24,25,
	25,26,27,27,28,29,29,30,31,32,32,33,34,35,35,36,
	37,38,39,39,40,41,42,43,44,45,46,47,48,49,50,50,
	51,52,54,55,56,57,58,59,60,61,62,63,64,66,67,68,
	69,70,72,73,74,75,77,78,79,81,82,83,85,86,87,89,
	90,92,93,95,96,98,99,101,102,104,105,107,109,110,112,114,
	115,117,119,120,122,124,126,128,129,131,133,135,137,138,140,142,
	144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
	177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
	215,218,220,223,225,228,231,233,236,239,241,244,247,250,252,255
};

/* function implementation --------------------------------------------------*/
void RGB_SetColor(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t ccr_r;
    uint16_t ccr_g;
    uint16_t ccr_b;

    r = gamma22[r];
    g = gamma22[g];
    b = gamma22[b];

    ccr_r = ((uint32_t)r * 999) / 255;
    ccr_g = ((uint32_t)g * 999) / 255;
    ccr_b = ((uint32_t)b * 999) / 255;

    TIM3->CCR1 = ccr_r;
    TIM3->CCR2 = ccr_g;
    TIM3->CCR4 = ccr_b;
}
RGB_t HSV2RGB(float h, float s, float v)
{
    RGB_t rgb;

    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h/60.0f,2)-1));
    float m = v - c;

    float r,g,b;

    if (h < 60)
    {
        r=c; g=x; b=0;
    }
    else if (h < 120)
    {
        r=x; g=c; b=0;
    }
    else if (h < 180)
    {
        r=0; g=c; b=x;
    }
    else if (h < 240)
    {
        r=0; g=x; b=c;
    }
    else if(h < 300)
    {
        r=x; g=0; b=c;
    }
    else
    {
        r=c; g=0; b=x;
    }

    rgb.r = (uint8_t)((r+m)*255);
    rgb.g = (uint8_t)((g+m)*255);
    rgb.b = (uint8_t)((b+m)*255);

    return rgb;
}
void RGB_SetHSV(float h,float s,float v)
{
    RGB_t rgb;

    rgb = HSV2RGB(h,s,v);

    RGB_SetColor(rgb.r, rgb.g, rgb.b);
}
void RGB_RainbowTask(void)
{
    static uint16_t hue = 0;

    RGB_SetHSV(hue,1.0f,1.0f);
	hue = (hue + 1) % 360;
    if (hue >= 360)
    {
		hue = 0;
	}
}
void RGB_BreathInit(RGB_Breath_t *breath)
{
    breath->idx = 0;
    breath->dir = 1;
}
void RGB_Init(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	RGB_BreathInit(&Breath1);
}
void RGB_BreathTask(RGB_Breath_t *breath, RGB_t color)
{
    if(breath->dir > 0)
    {
        if (breath->idx < 255)
        {
			breath->idx++;
		}
        else
        {
			breath->dir = -1;
		}
    }
    else
    {
        if (breath->idx > 0)
        {
			breath->idx--;
		}
        else
        {
			breath->dir = 1;
		}
    }

    RGB_SetColor( ((uint16_t)color.r * breath->idx) / 255,
			       ((uint16_t)color.g * breath->idx) / 255,
			       ((uint16_t)color.b * breath->idx) / 255);
}
void Task1_RainbowRGB(void *pvParameters)
{
	for (;;)
    {
    	RGB_RainbowTask();
		vTaskDelay(10);
	}
}


#ifndef __SIXSTEP_H
#define __SIXSTEP_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "BLDC_Control.h"       /* BLDC_Info, Phase_t, MotorDir_t */
#include "Hall.h"               /* Hall_Start / Hall_UpdateSpeed / Hall_GetState */

/* =============================================================================
 *  六步换相模块
 *
 *  职责:
 *    1. 维护正转/反转换向表
 *    2. 根据 Hall 状态切换 MOSFET (TIM8 + 低边 GPIO)
 *    3. 周期入口 SixStep_HallCyclic: 消费 Hall 沿 -> 更新转速 -> 换相
 *
 *  FOC 模式不会调用本模块, 六步换相完全不参与。
 * ==========================================================================*/

/* 换向表条目: PWM 高边相 + 低边常通相 */
typedef struct
{
    Phase_t PwmPhase;
    Phase_t LowPhase;
} BLDCMosCom_t;

/* global variable -----------------------------------------------------------*/
extern BLDCMosCom_t *pHallTable;

/* functions prototypes ------------------------------------------------------*/
void    SixStep_Start( void );                 /* 选表 + 启动 Hall + 初始换相 */
void    SixStep_Stop( void );                  /* 停止 Hall 并清当前相 */
void    SixStep_HallCyclic( void );            /* 消费 Hall 沿: 测速 + 换相 */
void    SixStep_ChangeMOSstate( Phase_t PwmPhase, Phase_t LowPhase, uint16_t Duty );
void    SixStep_HallTableSelect( MotorDir_t Dir );
void    SixStep_UpdateActiveDuty( uint16_t duty );
void    SixStep_DisableAllMos( void );

#ifdef __cplusplus
}
#endif

#endif /* __SIXSTEP_H */

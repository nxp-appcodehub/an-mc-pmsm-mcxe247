/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MC_PERIPH_INIT_H_
#define _MC_PERIPH_INIT_H_

#include "fsl_device_registers.h"
#include "mcdrv_pwm3ph_ftm.h"
#include "mcdrv_adc_mcxe.h"
#include "mcdrv_qdc_ftm.h"
#include "fsl_trgmux.h"
#include "fsl_adc12.h"
#include "fsl_pdb.h"

#include "m2_pmsm_appconfig.h"

  /* Structure used during clocks and modulo calculations */
typedef struct _clock_setup
{
    uint32_t ui32FastPeripheralClock;
    uint32_t ui32CpuFrequency;
    uint32_t ui32BusClock;
    uint32_t ui32SysPllClock;
    uint16_t ui16M1SpeedLoopFreq;
    uint16_t ui16M1SpeedLoopModulo;
    uint16_t ui16M1PwmFreq;
    uint16_t ui16M1PwmModulo;
    uint16_t ui16M1PwmDeadTime;
} clock_setup_t;

/******************************************************************************
 * Timing
 ******************************************************************************/
/* Fast loop interrupt generation timer*/
#define M1_PWM_TIMER            (FTM0)
/* PWM generation timer */
#define M1_PWM_TIMER_FTM0
/* PWM frequency in Hz*/
#define M1_PWM_FREQ             (10000U)
/* PWM modulo = FTM_input_clock / M1_PWM_FREQ */
#define M1_PWM_MODULO           ( CLOCK_GetSysClkFreq(kSCG_SysClkCore) / M1_PWM_FREQ)
/* PWM vs. Fast control loop ratio */
#define M1_FOC_FREQ_VS_PWM_FREQ (1U)
/* Fast loop frequency in Hz */
#define M1_FAST_LOOP_FREQ       (M1_PWM_FREQ / M1_FOC_FREQ_VS_PWM_FREQ)
/* Slow loop interrupt generation timer*/
#define M1_SLOW_LOOP_TIMER      (FTM2)
/* Slow loop interrupt generation timer */
#define M1_SLOW_LOOP_TIMER_FTM2
/* Slow control loop frequency */
#define M1_SLOW_LOOP_FREQ       (1000U)
/* Slow Loop modulo = FTM_input_clock / M1_SLOW_LOOP_FREQ */
#define M1_SLOW_LOOP_MODULO     (CLOCK_GetSysClkFreq(kSCG_SysClkCore) / M1_SLOW_LOOP_FREQ)
/* Fast loop period */
#define M1_FAST_LOOP_TS         ((float_t)1.0 / (float_t)(M1_FAST_LOOP_FREQ))
/* Slow loop period */
#define M1_SLOW_LOOP_TS         ((float_t)1.0 / (float_t)(M1_SLOW_LOOP_FREQ))
/* Speed loop frequency */
#define M1_SPEED_LOOP_FREQ 		(1000)

#define M1_QDC_TIMER      		(FTM1)

 /******************************************************************************
  * Output control
  ******************************************************************************/
/* Assignment of FTM channels to motor phases */
#define M1_PWM_PAIR_PHA         (0U)
#define M1_PWM_PAIR_PHB         (4U)
#define M1_PWM_PAIR_PHC         (6U)

/* Output PWM deadtime enable */
#define M1_PWM_DEADTIME_ENABLE          (1U)
/* Output PWM deadtime value in nanoseconds */
#define M1_PWM_DEADTIME (500)
/* Output PWM deadtime prescaler */
#define M1_PWM_DEADTIME_LENGTH_DTPS     (1U)
/* Output PWM deadtime register value */
#define M1_PWM_DEADTIME_LENGTH_DTVAL    (20U)

/* Over-current Fault enable */
#define M1_FAULT_ENABLE         (1U)
/* Over-current Fault detection number */
#define M1_FAULT_NUM            (0U)
/* Over-current Fault detection polarity */
#define M1_FAULT_POL            (0U)
/* Over-current Fault detection use CMP */
#define M1_FAULT_CMP_ENABLE     (1U)
/* Over-current Fault detection CMP instance */
#define M1_FAULT_CMP_INSTANCE   (CMP0)
/* Over-current Fault detection threshold */
#define M1_FAULT_CMP_THRESHOLD  (230U)

/* DC bus braking resistor control */
#define M1_BRAKE_SET()          
#define M1_BRAKE_CLEAR()        

/* Top and Bottom transistors PWM polarity */
#define M1_PWM_POL_TOP         (1U)
#define M1_PWM_POL_BOTTOM      (1U)

/******************************************************************************
 * ADC measurement definition
 ******************************************************************************/
/* Configuration table of ADC channels according to the input pin signals:
 *
 * Proper ADC channel assignment needs to follow these rules:
 *   - one phase current must be assigned to both ADC modules
 *   - two other phase current channels must be assigned to different ADC modules
 *   - Udcb and auxiliary channels must be assigned to differrent ADC modules

 *   Quantity | ADC Module 0 | ADC Module 1
 *   --------------------------------------------------------------------------
 *   I_A      |   ADC_CH8   |   ADC_CH8
 *   I_B      |   -------   |   ADC_CH0
 *   I_C      |   ADC_CH0   |   -------
 *   U_DCB    |   ADC_CH13   |   -------
 *   AUX      |   -------   |   ADC_CH26
 */
/* ADC channels assingnment */
/* Phase current assignment in sectors 1,6 */
#define M1_SEC16_PH_C_BASE      (ADC0)
#define M1_SEC16_PH_C_CHANNEL   (0U)
#define M1_SEC16_PH_B_BASE      (ADC1)
#define M1_SEC16_PH_B_CHANNEL   (0U)
/* Phase current assignment in sectors 2,3 */
#define M1_SEC23_PH_C_BASE      (ADC0)
#define M1_SEC23_PH_C_CHANNEL   (0U)
#define M1_SEC23_PH_A_BASE      (ADC1)
#define M1_SEC23_PH_A_CHANNEL   (8U)
/* Phase current assignment in sectors 4,5 */
#define M1_SEC45_PH_A_BASE      (ADC0)
#define M1_SEC45_PH_A_CHANNEL   (8U)
#define M1_SEC45_PH_B_BASE      (ADC1)
#define M1_SEC45_PH_B_CHANNEL   (0U)
/* DC bus voltage channels assingnment */
#define M1_UDCB_BASE            (ADC0)
#define M1_UDCB_CHANNEL         (13U)
/* Auxiliary channel assingnment */
#define M1_AUX_BASE             (ADC1)
#define M1_AUX_CHANNEL          (26U)

/* ADC0 channel muxing (ADC_MUXSEL) */
#define ADC0_MUXSEL             (0U)
/* ADC1 channel muxing (ADC_MUXSEL) */
#define ADC1_MUXSEL             (0U)

/* Phase current offset measurement filter window */
#define ADC_OFFSET_WINDOW       (3U)

/* PDB pre-trigger delay (dead-time/2) */
#define PDB_PRETRIG_DELAY       (10U)

/******************************************************************************
 * Inrush relay
 ******************************************************************************/
/* Inrush relay enable */
#define M1_INRUSH_ENABLE        (0U)
/* Inrush relay delay */
#define M1_INRUSH_DELAY         (0U)
/* Inrush relay pin */
#define M1_INRUSH_SET()         
#define M1_INRUSH_CLEAR()       

/* DC bus braking threshold hysteresis */
#define M1_U_DCB_HYSTERESIS (0.05F)
/******************************************************************************
 * MC driver macro definition and check - do not change this part
 ******************************************************************************/
/******************************************************************************
 * Define motor ADC control functions
 ******************************************************************************/
#define M1_MCDRV_CURR_3PH_VOLT_DCB_GET(par) \
    MCDRV_Curr3Ph2ShGet(par); \
    MCDRV_VoltDcBusGet(par);  \
    MCDRV_AuxValGet(par);
#define M1_MCDRV_CURR_3PH_CHAN_ASSIGN(par) (MCDRV_Curr3Ph2ShChanAssign(par))
#define M1_MCDRV_CURR_3PH_CALIB_INIT(par) (MCDRV_Curr3Ph2ShCalibInit(par))
#define M1_MCDRV_CURR_3PH_CALIB(par) (MCDRV_Curr3Ph2ShCalib(par))
#define M1_MCDRV_CURR_3PH_CALIB_SET(par) (MCDRV_Curr3Ph2ShCalibSet(par))

/******************************************************************************
 * Define motor 3-ph PWM control functions
 ******************************************************************************/
#define M1_MCDRV_PWM3PH_SET(par) (MCDRV_FtmPwm3PhSet(par))
#define M1_MCDRV_PWM3PH_EN(par) (MCDRV_FtmPwm3PhOutEn(par))
#define M1_MCDRV_PWM3PH_DIS(par) (MCDRV_FtmPwm3PhOutDis(par))
#define M1_MCDRV_PWM3PH_FLT_GET(par) (MCDRV_FtmPwm3PhFltGet(par))

#define M1_MCDRV_QD_GET(par) (MCDRV_QdEncGet(par))
#define M1_MCDRV_QD_SET_DIRECTION(par) (MCDRV_QdEncSetDirection(par))
#define M1_MCDRV_QD_SET_PULSES(par) (MCDRV_QdEncSetPulses(par))
#define M1_MCDRV_QD_CLEAR(par) (MCDRV_QdEncClear(par))

/******************************************************************************
 * global variable definitions
 ******************************************************************************/
extern mcdrv_pwm3ph_ftm_t g_sM1Pwm3ph;
extern mcdrv_adc_mcxe_t g_sM1Curr3phDcBus;
extern mcdrv_eqd_enc_t g_sM1Enc;
extern clock_setup_t g_sClockSetup;
/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
void MCDRV_Init_M1(void);

#ifdef __cplusplus
}
#endif
#endif /* _MC_PERIPH_INIT_H_  */

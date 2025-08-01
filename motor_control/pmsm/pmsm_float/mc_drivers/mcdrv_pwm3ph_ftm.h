/*
* Copyright 2016, Freescale Semiconductor, Inc.
* Copyright 2016-2021, 2025 NXP
*
* NXP Proprietary. This software is owned or controlled by NXP and may
* only be used strictly in accordance with the applicable license terms. 
* By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that
* you have read, and that you agree to comply with and are bound by,
* such license terms.  If you do not agree to be bound by the applicable
* license terms, then you may not retain, install, activate or otherwise
* use the software.
 */
#ifndef _MCDRV_FTM_PWM3PH_H_
#define _MCDRV_FTM_PWM3PH_H_

#include "fsl_device_registers.h"
#include "mlib.h"
#include "gmclib.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define MCDRV_FTM0 (0)

/* init sensors/actuators pointers */
#define M1_SET_PTR_DUTY(par1) (g_sM1Pwm3ph.psUABC = &(par1))

typedef struct _mcdrv_pwm3ph_ftm
{
    GMCLIB_3COOR_T_F16 *psUABC; /* pointer to the 3-phase pwm duty cycles */
    FTM_Type *pui32PwmBase;     /* pointer to phase A top value */
    uint16_t ui16ChanPhA;       /* number of channel for phase A */
    uint16_t ui16ChanPhB;       /* number of channel for phase A top */
    uint16_t ui16ChanPhC;       /* number of channel for phase B bottom */
    uint16_t ui16FaultNum;      /* FTM Fault number for over current fault detection */
    uint16_t ui16FaultFixNum;   /* FTM fault number for fixed over-current fault detection */
    uint16_t ui16FaultAdjNum;   /* FTM fault number for adjustable over-current fault detection */
} mcdrv_pwm3ph_ftm_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief FTM value register updates
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_FtmPwm3PhSet(mcdrv_pwm3ph_ftm_t *this);

/*!
 * @brief Function enables PWM outputs
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_FtmPwm3PhOutEn(mcdrv_pwm3ph_ftm_t *this);
/*!
 * @brief Function disables PWM outputs
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_FtmPwm3PhOutDis(mcdrv_pwm3ph_ftm_t *this);

/*!
 * @brief Function return actual value of over current flag
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_FtmPwm3PhFltGet(mcdrv_pwm3ph_ftm_t *this);

#ifdef __cplusplus
}
#endif

#endif /* _MCDRV_FTM_PWM3PH_H_ */

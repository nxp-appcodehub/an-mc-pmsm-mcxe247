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

#include "mcdrv_pwm3ph_ftm.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static bool_t s_statusPass;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief FTM value register updates
 *
 * @param this Pointer to the current object
 *
 * @return none
 */
void MCDRV_FtmPwm3PhSet(mcdrv_pwm3ph_ftm_t *this)
{
    frac16_t f16DutyCycle;
    GMCLIB_3COOR_T_F16 sUABCtemp;

    /* pointer to duty cycle structure */
    sUABCtemp = *this->psUABC;

    /* phase A */
    f16DutyCycle                                            = MLIB_Mul_F16(this->pui32PwmBase->MOD, sUABCtemp.f16A);
    this->pui32PwmBase->CONTROLS[this->ui16ChanPhA].CnV     = FTM_CnV_VAL(MLIB_Neg_F16(f16DutyCycle));
    this->pui32PwmBase->CONTROLS[this->ui16ChanPhA + 1].CnV = FTM_CnV_VAL(f16DutyCycle);

    /* phase B */
    f16DutyCycle                                            = MLIB_Mul_F16(this->pui32PwmBase->MOD, sUABCtemp.f16B);
    this->pui32PwmBase->CONTROLS[this->ui16ChanPhB].CnV     = FTM_CnV_VAL(MLIB_Neg_F16(f16DutyCycle));
    this->pui32PwmBase->CONTROLS[this->ui16ChanPhB + 1].CnV = FTM_CnV_VAL(f16DutyCycle);

    /* phase C */
    f16DutyCycle                                            = MLIB_Mul_F16(this->pui32PwmBase->MOD, sUABCtemp.f16C);
    this->pui32PwmBase->CONTROLS[this->ui16ChanPhC].CnV     = FTM_CnV_VAL(MLIB_Neg_F16(f16DutyCycle));
    this->pui32PwmBase->CONTROLS[this->ui16ChanPhC + 1].CnV = FTM_CnV_VAL(f16DutyCycle);

    /* Set LDOK bit in FTm PWMLOAD register */
    this->pui32PwmBase->PWMLOAD = 0x0200;

}

/*!
 * @brief Function enables PWM outputs
 *
 * @param this Pointer to the current object
 *
 * @return none
 */
void MCDRV_FtmPwm3PhOutEn(mcdrv_pwm3ph_ftm_t *this)
{

    uint8_t ui8MaskTemp = 0U;

    ui8MaskTemp = ~((3U << (this->ui16ChanPhA)) | (3U << (this->ui16ChanPhB)) | (3U << (this->ui16ChanPhC)));

    /* FlexTIMER
     * OUTMASK register = mcPWM_BASE + $0x60
     *
     * Any write to the OUTMASK register, stores the value in its write buffer. The register is
     * updated with the value of its write buffer according to PWM synchronization.
     *
     * CHnOM = 0 - Channel output is not masked. It continues to operate normally.
     * CHnOM = 1 - Channel output is masked. It is forced to its inactive state.
     * |----------------------------------------------------------------------------|
     * |bits:   |   31..8   |  7   |   6  |   5  |  4   |   3  |  2   |  1   |  0   |
     * |Meaning:| RESERVED  |CH7OM |CH6OM |CH5OM |CH4OM |CH3OM |CH2OM |CH1OM |CH0OM |
     * |----------------------------------------------------------------------------|
     * |Value:  |     0     |      |      |      |      |      |      |      |      |
     * |----------------------------------------------------------------------------|
     */

    /* PWM outputs enabled required FTM channels */
    this->pui32PwmBase->OUTMASK &= ui8MaskTemp;

}

/*!
 * @brief Function disables PWM outputs
 *
 * @param this Pointer to the current object
 *
 * @return none
 */
void MCDRV_FtmPwm3PhOutDis(mcdrv_pwm3ph_ftm_t *this)
{

    uint8_t ui8MaskTemp = 0U;

    ui8MaskTemp = (3U << (this->ui16ChanPhA)) | (3U << (this->ui16ChanPhB)) | (3U << (this->ui16ChanPhC));

    /* FlexTIMER
     * OUTMASK register = mcPWM_BASE + $0x60
     *
     * Any write to the OUTMASK register, stores the value in its write buffer. The register is
     * updated with the value of its write buffer according to PWM synchronization.
     *
     * CHnOM = 0 - Channel output is not masked. It continues to operate normally.
     * CHnOM = 1 - Channel output is masked. It is forced to its inactive state.
     * |----------------------------------------------------------------------------|
     * |bits:   |   31..8   |  7   |   6  |   5  |  4   |   3  |  2   |  1   |  0   |
     * |Meaning:| RESERVED  |CH7OM |CH6OM |CH5OM |CH4OM |CH3OM |CH2OM |CH1OM |CH0OM |
     * |----------------------------------------------------------------------------|
     * |Value:  |     0     |      |      |      |      |      |      |      |      |
     * |----------------------------------------------------------------------------|
     */

    /* PWM outputs disable required FTM channels */
    this->pui32PwmBase->OUTMASK |= ui8MaskTemp;

}

/*!
 * @brief Function return actual value of over current flag
 *
 * @param this Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_FtmPwm3PhFltGet(mcdrv_pwm3ph_ftm_t *this)
{
    /* Read fixed-value over-current flag */
    s_statusPass = this->pui32PwmBase->FMS & (1 << this->ui16FaultFixNum | 1 << this->ui16FaultAdjNum);

    /* Clear fault flags */
    this->pui32PwmBase->FMS &= ~FTM_FMS_FAULTF_MASK;

    return ((s_statusPass > 0));
}

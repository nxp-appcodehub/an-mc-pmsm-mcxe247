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

#include "state_machine.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

RAM_FUNC_LIB
static void SM_StateFaultFast(sm_app_ctrl_t *psAppCtrl);
RAM_FUNC_LIB
static void SM_StateInitFast(sm_app_ctrl_t *psAppCtrl);
RAM_FUNC_LIB
static void SM_StateStopFast(sm_app_ctrl_t *psAppCtrl);
RAM_FUNC_LIB
static void SM_StateRunFast(sm_app_ctrl_t *psAppCtrl);

RAM_FUNC_LIB
static void SM_StateFaultSlow(sm_app_ctrl_t *psAppCtrl);
RAM_FUNC_LIB
static void SM_StateInitSlow(sm_app_ctrl_t *psAppCtrl);
RAM_FUNC_LIB
static void SM_StateStopSlow(sm_app_ctrl_t *psAppCtrl);
RAM_FUNC_LIB
static void SM_StateRunSlow(sm_app_ctrl_t *psAppCtrl);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief State machine function field - fast */
const pfcn_void_psm g_SM_STATE_TABLE_FAST[4] = {SM_StateFaultFast, SM_StateInitFast, SM_StateStopFast, SM_StateRunFast};

/*! @brief State machine function field - slow */
const pfcn_void_psm g_SM_STATE_TABLE_SLOW[4] = {SM_StateFaultSlow, SM_StateInitSlow, SM_StateStopSlow, SM_StateRunSlow};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief State machine Fault state in fast
 *
 * @param psAppCtrl     The pointer of the State machine application
 *
 * @return None
 */
RAM_FUNC_LIB
static void SM_StateFaultFast(sm_app_ctrl_t *psAppCtrl)
{
    /* User Fault function */
    psAppCtrl->psStateFast->Fault();

    /* if clear fault command flag */
    if ((psAppCtrl->uiCtrl & SM_CTRL_FAULT_CLEAR) > 0U)
    {
        /* Clear INIT_DONE, FAULT, FAULT_CLEAR flags */
        psAppCtrl->uiCtrl &= (uint16_t)(~(SM_CTRL_INIT_DONE | SM_CTRL_FAULT | SM_CTRL_FAULT_CLEAR));

        /* User Fault to Init transition function */
        psAppCtrl->psTrans->FaultStop();

        /* Init state */
        psAppCtrl->eState = kSM_AppStop;
    }
}

/*!
 * @brief State machine Init state in fast
 *
 * @param psAppCtrl     The pointer of the State machine application
 *
 * @return None
 */
RAM_FUNC_LIB
static void SM_StateInitFast(sm_app_ctrl_t *psAppCtrl)
{
    /* User Init function */
    psAppCtrl->psStateFast->Init();

    /* if fault flag */
    if ((psAppCtrl->uiCtrl & SM_CTRL_FAULT) > 0U)
    {
        /* User Init to Fault transition function */
        psAppCtrl->psTrans->InitFault();

        /* Fault state */
        psAppCtrl->eState = kSM_AppFault;
    }
    /* if INIT_DONE flag */
    else if ((psAppCtrl->uiCtrl & SM_CTRL_INIT_DONE) > 0U)
    {
        /* Clear INIT_DONE, START_STOP, OM_CHANGE, STOP_ACK, RUN_ACK flags */
        psAppCtrl->uiCtrl &= (uint16_t)(~(SM_CTRL_INIT_DONE | SM_CTRL_STOP | SM_CTRL_START | SM_CTRL_STOP_ACK | SM_CTRL_RUN_ACK));

        /* User Init to Stop transition function */
        psAppCtrl->psTrans->InitStop();

        /* Stop state */
        psAppCtrl->eState = kSM_AppStop;
    }
    else
	{
    	/* Else */
		;
	}
}

/*!
 * @brief State machine Stop state in fast
 *
 * @param psAppCtrl     The pointer of the State machine application
 *
 * @return None
 */
RAM_FUNC_LIB
static void SM_StateStopFast(sm_app_ctrl_t *psAppCtrl)
{
    /* User Stop function */
    psAppCtrl->psStateFast->Stop();

    /* if fault */
    if ((psAppCtrl->uiCtrl & SM_CTRL_FAULT) > 0U)
    {
        /* User Stop to Fault transition function */
        psAppCtrl->psTrans->StopFault();

        /* Fault state */
        psAppCtrl->eState = kSM_AppFault;
    }
    else if ((psAppCtrl->uiCtrl & SM_CTRL_START) > 0U)
    {
        /* User Stop to Run transition function, user must set up the SM_CTRL_RUN_ACK
        flag to allow the RUN state */
        psAppCtrl->psTrans->StopRun();

        /* Clears the START command */
        if ((psAppCtrl->uiCtrl & SM_CTRL_RUN_ACK) > 0U)
        {
            /* Clears the RUN_ACK flag */
            psAppCtrl->uiCtrl &= (uint16_t)(~(SM_CTRL_RUN_ACK | SM_CTRL_START));

            /* Run state */
            psAppCtrl->eState = kSM_AppRun;
        }
    }
    else
	{
		;
	}
}

/*!
 * @brief State machine Run state in fast
 *
 * @param psAppCtrl     The pointer of the State machine application
 *
 * @return None
 */
RAM_FUNC_LIB
static void SM_StateRunFast(sm_app_ctrl_t *psAppCtrl)
{
    /* User Run function */
    psAppCtrl->psStateFast->Run();

    if ((psAppCtrl->uiCtrl & SM_CTRL_FAULT) > 0U)
    {
        /* User Run to Fault transition function */
        psAppCtrl->psTrans->RunFault();

        /* Fault state */
        psAppCtrl->eState = kSM_AppFault;
    }
    else if ((psAppCtrl->uiCtrl & SM_CTRL_STOP) > 0U)
    {
        /* User Run to Stop transition function, user must set up the SM_CTRL_STOP_ACK
        flag to allow the STOP state */
        psAppCtrl->psTrans->RunStop();

        if ((psAppCtrl->uiCtrl & SM_CTRL_STOP_ACK) > 0U)
        {
            /* Clears the STOP_ACK flag */
            psAppCtrl->uiCtrl &= (uint16_t)(~(SM_CTRL_STOP_ACK | SM_CTRL_STOP));

            /* Run state */
            psAppCtrl->eState = kSM_AppStop;
        }
    }
    else
    {
    	;
    }
}

/*!
 * @brief State machine Fault state in slow
 *
 * @param psAppCtrl     The pointer of the State machine application
 *
 * @return None
 */
RAM_FUNC_LIB
static void SM_StateFaultSlow(sm_app_ctrl_t *psAppCtrl)
{
    psAppCtrl->psStateSlow->Fault();
}

/*!
 * @brief State machine Init state in slow
 *
 * @param psAppCtrl     The pointer of the State machine application
 *
 * @return None
 */
RAM_FUNC_LIB
static void SM_StateInitSlow(sm_app_ctrl_t *psAppCtrl)
{
    psAppCtrl->psStateSlow->Init();
}

/*!
 * @brief State machine Stop state in slow
 *
 * @param psAppCtrl     The pointer of the State machine application
 *
 * @return None
 */
RAM_FUNC_LIB
static void SM_StateStopSlow(sm_app_ctrl_t *psAppCtrl)
{
    psAppCtrl->psStateSlow->Stop();
}

/*!
 * @brief State machine Run state in slow
 *
 * @param psAppCtrl     The pointer of the State machine application
 *
 * @return None
 */
RAM_FUNC_LIB
static void SM_StateRunSlow(sm_app_ctrl_t *psAppCtrl)
{
    psAppCtrl->psStateSlow->Run();
}

/*******************************************************************************
*
* Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
* Copyright 2016-2021, 2024 NXP
*
* NXP Proprietary. This software is owned or controlled by NXP and may
* only be used strictly in accordance with the applicable license terms. 
* By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that
* you have read, and that you agree to comply with and are bound by,
* such license terms.  If you do not agree to be bound by the applicable
* license terms, then you may not retain, install, activate or otherwise
* use the software.
* 
*
****************************************************************************//*!
*
* @brief Main GFLIB header file for devices without FPU. 
* 
*******************************************************************************/
#ifndef _GFLIB_H_
#define _GFLIB_H_

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
* Includes
*******************************************************************************/ 
#include "mlib.h"
#include "GFLIB_Acos_F16.h" 
#include "GFLIB_Asin_F16.h" 
#include "GFLIB_AtanYX_F16.h"
#include "GFLIB_Atan_F32.h"
#include "GFLIB_CtrlBetaIPDp_A32.h"     
#include "GFLIB_CtrlBetaIPp_A32.h"  
#include "GFLIB_CtrlPIDp_A32.h"   
#include "GFLIB_CtrlPIp_A32.h"
#include "GFLIB_DFlexRamp_A32.h"    
#include "GFLIB_DRamp_F16.h" 
#include "GFLIB_DRamp_F32.h"     
#include "GFLIB_FlexRamp_A32.h"      
#include "GFLIB_FlexSRamp_A32.h"     
#include "GFLIB_Hyst_F16_Asmi.h"
#include "GFLIB_Integrator_A32.h" 
#include "GFLIB_Limit_F16.h" 
#include "GFLIB_Limit_F32.h" 
#include "GFLIB_LowerLimit_F16.h" 
#include "GFLIB_LowerLimit_F32.h"
#include "GFLIB_LutPer_F16_Asmi.h"   
#include "GFLIB_LutPer_F32.h"    
#include "GFLIB_Lut_F16_Asmi.h"
#include "GFLIB_Lut_F32.h"
#include "GFLIB_Ramp_F16.h"
#include "GFLIB_Ramp_F32.h"
#include "GFLIB_SinCos_F32.h"
#include "GFLIB_Sqrt_F16.h"
#include "GFLIB_Sqrt_F32.h"  
#include "GFLIB_Tan_F32.h" 
#include "GFLIB_UpperLimit_F16.h" 
#include "GFLIB_UpperLimit_F32.h" 
#include "GFLIB_VectorLimit_F16.h"     

/*******************************************************************************
* Macros
*******************************************************************************/
#define GFLIB_Acos_F16(f16Val)                                                GFLIB_Acos_F16_C(f16Val)
#define GFLIB_Asin_F16(f16Val)                                                GFLIB_Asin_F16_C(f16Val)
#define GFLIB_AtanYX_F16(f16Y, f16X, pbErrFlag)                               GFLIB_AtanYX_F16_Asm(f16Y, f16X, pbErrFlag)  
#define GFLIB_Atan_F16(f16Val)                                                GFLIB_Atan_F16_Asm(f16Val)  
#define GFLIB_Cos_F16(f16Angle)                                               GFLIB_Cos_F16_Asmi(f16Angle)  
#define GFLIB_CtrlBetaIPpAWInit_F16(f16InitVal, psParam)                      GFLIB_CtrlBetaIPpAWInit_F16_Ci(f16InitVal, psParam)
#define GFLIB_CtrlBetaIPpAW_F16(f16InReq, f16In, pbStopIntegFlag, psParam)    GFLIB_CtrlBetaIPpAW_F16_C(f16InReq, f16In, pbStopIntegFlag, psParam)    
#define GFLIB_CtrlBetaIPDpAWInit_F16(f16InitVal, psParam)                     GFLIB_CtrlBetaIPDpAWInit_F16_Ci(f16InitVal, psParam)
#define GFLIB_CtrlBetaIPDpAW_F16(f16InReq, f16In, f16InErrD, pbStopIntegFlag, psParam) GFLIB_CtrlBetaIPDpAW_F16_C(f16InReq, f16In, f16InErrD, pbStopIntegFlag, psParam)    
#define GFLIB_CtrlPIDpAWInit_F16(f16InitVal, psParam)                         GFLIB_CtrlPIDpAWInit_F16_Ci(f16InitVal, psParam)   
#define GFLIB_CtrlPIDpAW_F16(f16InErr, f16InErrD, pbStopIntegFlag, psParam)   GFLIB_CtrlPIDpAW_F16_Asm(f16InErr, f16InErrD, pbStopIntegFlag, psParam)
#define GFLIB_CtrlPIpAWInit_F16(f16InitVal, psParam)                          GFLIB_CtrlPIpAWInit_F16_Ci(f16InitVal, psParam)
#define GFLIB_CtrlPIpAW_F16(f16InErr, pbStopIntegFlag, psParam)               GFLIB_CtrlPIpAW_F16_Asm(f16InErr, pbStopIntegFlag, psParam)
#define GFLIB_DFlexRampInit_F16(f16InitVal, psParam)                          GFLIB_DFlexRampInit_F16_C(f16InitVal, psParam)
#define GFLIB_DFlexRampCalcIncr_F16(f16Target, a32Duration, f32IncrSatMot, f32IncrSatGen, psParam) GFLIB_DFlexRampCalcIncr_F16_C(f16Target, a32Duration, f32IncrSatMot, f32IncrSatGen, psParam)
#define GFLIB_DFlexRamp_F16(f16Instant, pbStopFlagMot, pbStopFlagGen,psParam) GFLIB_DFlexRamp_F16_C(f16Instant, pbStopFlagMot, pbStopFlagGen, psParam)
#define GFLIB_DRampInit_F16(f16InitVal, psParam)                              GFLIB_DRampInit_F16_Ci(f16InitVal, psParam)
#define GFLIB_DRampInit_F32(f32InitVal, psParam)                              GFLIB_DRampInit_F32_Ci(f32InitVal, psParam)
#define GFLIB_DRamp_F16(f16Target, f16Instant, pbStopFlag, psParam)           GFLIB_DRamp_F16_Asm(f16Target, f16Instant, pbStopFlag, psParam)
#define GFLIB_DRamp_F32(f32Target, f32Instant, pbStopFlag, psParam)           GFLIB_DRamp_F32_Asm(f32Target, f32Instant, pbStopFlag, psParam)
#define GFLIB_FlexRampInit_F16(f16InitVal, psParam)                           GFLIB_FlexRampInit_F16_C(f16InitVal, psParam)
#define GFLIB_FlexRampCalcIncr_F16(f16Target, a32Duration, psParam)           GFLIB_FlexRampCalcIncr_F16_C(f16Target, a32Duration, psParam)
#define GFLIB_FlexRamp_F16(psParam)                                           GFLIB_FlexRamp_F16_C(psParam)
#define GFLIB_FlexSRampInit_F16(f16InitVal, psParam)                          GFLIB_FlexSRampInit_F16_C(f16InitVal, psParam)
#define GFLIB_FlexSRampCalcIncr_F16(f16Target, a32Duration, psParam)          GFLIB_FlexSRampCalcIncr_F16_C(f16Target, a32Duration, psParam)
#define GFLIB_FlexSRamp_F16(psParam)                                          GFLIB_FlexSRamp_F16_C(psParam)
#define GFLIB_Hyst_F16(f16Val, psParam)                                       GFLIB_Hyst_F16_Asmi(f16Val, psParam)
#define GFLIB_IntegratorInit_F16(f16InitVal, psParam)                         GFLIB_IntegratorInit_F16_Ci(f16InitVal, psParam)
#define GFLIB_Integrator_F16(f16InVal, psParam)                               GFLIB_Integrator_F16_Ci(f16InVal, psParam)
#define GFLIB_Limit_F16(f16Val, f16LLim, f16ULim)                             GFLIB_Limit_F16_Ci(f16Val, f16LLim, f16ULim)  
#define GFLIB_Limit_F32(f32Val, f32LLim, f32ULim)                             GFLIB_Limit_F32_Ci(f32Val, f32LLim, f32ULim)
#define GFLIB_LowerLimit_F16(f16Val, f16LLim)                                 GFLIB_LowerLimit_F16_Ci(f16Val, f16LLim)  
#define GFLIB_LowerLimit_F32(f32Val, f32LLim)                                 GFLIB_LowerLimit_F32_Ci(f32Val, f32LLim)
#define GFLIB_Lut1D_F16(f16X, pf16Table, u16TableSize)                        GFLIB_Lut1D_F16_Asmi(f16X, pf16Table, u16TableSize)
#define GFLIB_LutPer1D_F16(f16X, pf16Table, u16TableSize)                     GFLIB_LutPer1D_F16_Asmi( f16X, pf16Table, u16TableSize)
#define GFLIB_Lut1D_F32(f32X, pf32Table, u16TableSize)                        GFLIB_Lut1D_F32_C(f32X, pf32Table, u16TableSize)
#define GFLIB_LutPer1D_F32(f32X, pf32Table, u16TableSize)                     GFLIB_LutPer1D_F32_C(f32X, pf32Table, u16TableSize)
#define GFLIB_RampInit_F16(f16InitVal, psParam)                               GFLIB_RampInit_F16_Ci(f16InitVal, psParam)
#define GFLIB_RampInit_F32(f32InitVal, psParam)                               GFLIB_RampInit_F32_Ci(f32InitVal, psParam)
#define GFLIB_Ramp_F16(f16Target, psParam)                                    GFLIB_Ramp_F16_Ci(f16Target, psParam)
#define GFLIB_Ramp_F32(f32Target, psParam)                                    GFLIB_Ramp_F32_Ci(f32Target, psParam)
#define GFLIB_Sin_F16(f16Angle)                                               GFLIB_Sin_F16_Asm(f16Angle)
#define GFLIB_SinCos_F16(f16Angle, f16SinCos)                                 GFLIB_SinCos_F16_Ci(f16Angle, f16SinCos)

#define GFLIB_Sqrt_F16(f16Val)                                                GFLIB_Sqrt_F16_C(f16Val)
#define GFLIB_Sqrt_F16l(f32Val)                                               GFLIB_Sqrt_F16l_C(f32Val)
#define GFLIB_Tan_F16(f16Angle)                                               GFLIB_Tan_F16_C(f16Angle)
#define GFLIB_UpperLimit_F16(f16Val, f16ULim)                                 GFLIB_UpperLimit_F16_Ci(f16Val, f16ULim)
#define GFLIB_UpperLimit_F32(f32Val, f32ULim)                                 GFLIB_UpperLimit_F32_Ci(f32Val, f32ULim)  
#define GFLIB_VectorLimit_F16(psVectorIn, f16Lim, psVectorOut)                GFLIB_VectorLimit_F16_C(psVectorIn, f16Lim, psVectorOut)
#define GFLIB_VectorLimit1_F16(psVectorIn, f16Lim, psVectorOut)               GFLIB_VectorLimit1_F16_C(psVectorIn, f16Lim, psVectorOut) 
  
#if defined(__cplusplus)
}
#endif

#endif /* _GFLIB_H_ */  

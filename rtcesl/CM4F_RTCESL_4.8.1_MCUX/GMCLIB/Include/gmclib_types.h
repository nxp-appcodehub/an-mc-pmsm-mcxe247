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
* @brief  GMCLIB Types  
* 
*******************************************************************************/
#ifndef _GMCLIB_TYPES_H_
#define _GMCLIB_TYPES_H_

#if defined(__cplusplus) 
extern "C" {
#endif

/*******************************************************************************
* Includes
*******************************************************************************/ 
#include "mlib_types.h"

/******************************************************************************
* Macros 
******************************************************************************/  
#define GMCLIB_CONST    
  
/* GMCLIB_ONE_DIV_SQRT3_F16 define, the value is: 32768 / sqrt(3) = 18919 = 0x49E7 */
#define GMCLIB_ONE_DIV_SQRT3_F16  (frac16_t)18919
/* GMCLIB_SQRT3_DIV_2_F16 define, the value is: 32768 * sqrt(3) / 2 = 28378 = 0x6EDA */  
#define GMCLIB_SQRT3_DIV_2_F16    (frac16_t)28378

/* GMCLIB_ONE_DIV_SQRT3_FLT define, the value is: 0.5773502691896258 */
#define GMCLIB_ONE_DIV_SQRT3_FLT  (float_t) 0.5773502691896258F
/* GMCLIB_SQRT3_FLT define, the value is: 0.8660254037844390 */  
#define GMCLIB_SQRT3_DIV_2_FLT    (float_t) 0.8660254037844390F    
/* GMCLIB_SQRT3_DIVBY_2_FLT define, the value is: 1.7320508075688800 */  
#define GMCLIB_SQRT3_FLT          (float_t) 1.7320508075688800F
    
/*******************************************************************************
* Types 
********************************************************************************/ 
/* Integer data types */

/* The GMCLIB_3COOR_T_F16 structure type corresponds to the three-phase stationary
coordinate system based on the A, B, C components. */
typedef struct
{
    frac16_t f16A;
    frac16_t f16B;
    frac16_t f16C;
} GMCLIB_3COOR_T_F16;

/* The GMCLIB_2COOR_ALBE_T_F16 structure type corresponds to the two-phase
stationary coordinate system based on the Alpha and Beta orthogonal components. */
typedef struct
{
    frac16_t f16Alpha;
    frac16_t f16Beta;
} GMCLIB_2COOR_ALBE_T_F16;

/* The GMCLIB_2COOR_DQ_T_F16 structure type corresponds to the two-phase rotating
coordinate system based on the D and Q orthogonal components. */
typedef struct
{
    frac16_t f16D;
    frac16_t f16Q;
} GMCLIB_2COOR_DQ_T_F16;

/* The GMCLIB_2COOR_DQ_T_F32 structure type corresponds to the two-phase rotating
coordinate system based on the D and Q orthogonal components. */
typedef struct
{
    frac32_t f32D;
    frac32_t f32Q;
} GMCLIB_2COOR_DQ_T_F32;

/* The GMCLIB_2COOR_ALBE_T_F16 structure type corresponds to the two-phase
stationary coordinate system based on the Alpha and Beta orthogonal components. */
typedef struct
{
    frac32_t f32Alpha;
    frac32_t f32Beta;
} GMCLIB_2COOR_ALBE_T_F32;

/* The GMCLIB_2COOR_AB_T_F16 structure type corresponds to the two-phase stationary
coordinate system based on the A and B components. */
    
typedef struct
{
    frac16_t f16A;   /* First argument */
    frac16_t f16B;   /* Second argument */
} GMCLIB_2COOR_AB_T_F16;   
  
/* The GMCLIB_2COOR_AB_T_F32 structure type corresponds to the two-phase stationary
coordinate system based on the A and B components. */

typedef struct
{
    frac32_t f32A;   /* First argument */
    frac32_t f32B;   /* Second argument */
} GMCLIB_2COOR_AB_T_F32; 


/* Floating point data types */

/* The GMCLIB_3COOR_T_FLT structure type corresponds to the three-phase stationary
coordinate system based on the A, B, C components. */
typedef struct
{
    float_t fltA;
    float_t fltB;
    float_t fltC;
} GMCLIB_3COOR_T_FLT;

/* The GMCLIB_2COOR_ALBE_T_FLT structure type corresponds to the two-phase
stationary coordinate system based on the Alpha and Beta orthogonal components. */
typedef struct
{
    float_t fltAlpha;
    float_t fltBeta;
} GMCLIB_2COOR_ALBE_T_FLT;

/* The GMCLIB_2COOR_DQ_T_FLT structure type corresponds to the two-phase rotating
coordinate system based on the D and Q orthogonal components. */
typedef struct
{
    float_t fltD;
    float_t fltQ;
} GMCLIB_2COOR_DQ_T_FLT;


/* The GMCLIB_2COOR_AB_T_F32 structure type corresponds to the two-phase stationary
coordinate system based on the A and B components. */

typedef struct
{
    float_t fltA;   /* First argument */
    float_t fltB;   /* Second argument */
} GMCLIB_2COOR_AB_T_FLT; 

#if defined(__cplusplus) 
}
#endif

#endif /* _GMCLIB_TYPES_H_ */

/*******************************************************************************
*
* Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
* Copyright 2016-2021 2024 NXP
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
* @brief  asm_mac.h
*
*******************************************************************************/

#ifndef ASM_MAC_H_
#define ASM_MAC_H_

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
* Macros
*******************************************************************************/

#if defined(__IAR_SYSTEMS_ASM__)           /* IAR compiler   */
    #define ASM_PREFIX(x)                   x
    #define ASM_LABEL(label)                label
    #define ASM_EXTERN(label)               EXTERN  label
    #define ASM_ALIGN(value)                ALIGNROM  value
    #define ASM_PUBLIC(label)               PUBLIC label
    #define ASM_CONST16(value)              DC16 value
    #define ASM_CONST32(value)              DC32 value
    #define ASM_LABEL_CONST32(label,value)  ASM_LABEL(label) ASM_CONST32(value)
    #define ASM_DATA_SECTION(label)         SECTION label : DATA (4)
    #define ASM_CODE_SECTION(label)         SECTION label : CODE (4)
    #define ASM_END                         END
    #define ASM_EQUATE(label, value)        label   EQU  value
    #define ASM_COMP_SPECIFIC_DIRECTIVES
    #define ASM_CODE                        CODE
    #define ASM_DATA                        DATA
#elif defined(__GNUC__)                     /* GCC compiler */
    #define ASM_PREFIX(x)                   x
    #define ASM_LABEL(label)                label:
    #define ASM_EXTERN(label)               .extern ASM_PREFIX(label)
    #define ASM_ALIGN(value)                .balign value
    #define ASM_PUBLIC(label)               .type ASM_PREFIX(label), %function; \
                                            .global ASM_PREFIX(label);		
    #define ASM_CONST16(value)              .short value
    #define ASM_CONST32(value)              .long value
    #define ASM_LABEL_CONST32(label,value)  ASM_LABEL(label) ASM_CONST32(value)
    #define ASM_DATA_SECTION(name)          .section name
    #define ASM_CODE_SECTION(name)          .section name
    #define ASM_END                         .end
    #define ASM_EQUATE(label,value)         .equ label, value
    #define ASM_COMP_SPECIFIC_DIRECTIVES    .syntax unified
    #define ASM_CODE                        .thumb
    #define ASM_DATA
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION) /* ARM compiler */
    #define ASM_PREFIX(x)                   x
    #define ASM_LABEL(label)                label
    #define ASM_EXTERN(label)               EXTERN label
    #define ASM_ALIGN(value)                ALIGN value
    #define ASM_PUBLIC(label)               EXPORT label
    #define ASM_CONST16(value)              DCWU value
    #define ASM_CONST32(value)              DCDU value
    #define ASM_LABEL_CONST32(label,value)  ASM_LABEL(label) ASM_CONST32(value)
    #define ASM_DATA_SECTION(label)         AREA |label|, DATA
    #define ASM_CODE_SECTION(label)         AREA |label|, CODE
    #define ASM_END                         END
    #define ASM_EQUATE(label, value)        label   EQU  value
    #define ASM_COMP_SPECIFIC_DIRECTIVES
    #define ASM_CODE                        THUMB
    #define ASM_DATA
#endif

#if defined(__cplusplus)
}
#endif

#endif /* ASM_MAC_H_ */

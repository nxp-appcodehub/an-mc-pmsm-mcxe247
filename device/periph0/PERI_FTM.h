/*
** ###################################################################
**     Processors:          MCXE247VLL
**                          MCXE247VLQ
**
**     Version:             rev. 1.0, 2025-02-21
**     Build:               b250417
**
**     Abstract:
**         CMSIS Peripheral Access Layer for FTM
**
**     Copyright 1997-2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2025 NXP
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 1.0 (2025-02-21)
**         Generated based on Rev1 Draft RM..
**
** ###################################################################
*/

/*!
 * @file PERI_FTM.h
 * @version 1.0
 * @date 2025-02-21
 * @brief CMSIS Peripheral Access Layer for FTM
 *
 * CMSIS Peripheral Access Layer for FTM
 */

#if !defined(PERI_FTM_H_)
#define PERI_FTM_H_                              /**< Symbol preventing repeated inclusion */

#if (defined(CPU_MCXE247VLL) || defined(CPU_MCXE247VLQ))
#include "MCXE247_COMMON.h"
#else
  #error "No valid CPU defined!"
#endif

/* ----------------------------------------------------------------------------
   -- Device Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Peripheral_access_layer Device Peripheral Access Layer
 * @{
 */


/*
** Start of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #if (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
  #else
    #pragma push
    #pragma anon_unions
  #endif
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=extended
#else
  #error Not supported compiler type
#endif

/* ----------------------------------------------------------------------------
   -- FTM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Peripheral_Access_Layer FTM Peripheral Access Layer
 * @{
 */

/** FTM - Size of Registers Arrays */
#define FTM_CONTROLS_COUNT                        8u
#define FTM_CV_MIRROR_COUNT                       8u

/** FTM - Register Layout Typedef */
typedef struct {
  __IO uint32_t SC;                                /**< Status And Control, offset: 0x0 */
  __IO uint32_t CNT;                               /**< Counter, offset: 0x4 */
  __IO uint32_t MOD;                               /**< Modulo, offset: 0x8 */
  struct {                                         /* offset: 0xC, array step: 0x8 */
    __IO uint32_t CnSC;                              /**< Channel (n) Status And Control, array offset: 0xC, array step: 0x8 */
    __IO uint32_t CnV;                               /**< Channel (n) Value, array offset: 0x10, array step: 0x8 */
  } CONTROLS[FTM_CONTROLS_COUNT];
  __IO uint32_t CNTIN;                             /**< Counter Initial Value, offset: 0x4C */
  __IO uint32_t STATUS;                            /**< Capture And Compare Status, offset: 0x50 */
  __IO uint32_t MODE;                              /**< Features Mode Selection, offset: 0x54 */
  __IO uint32_t SYNC;                              /**< Synchronization, offset: 0x58 */
  __IO uint32_t OUTINIT;                           /**< Initial State For Channels Output, offset: 0x5C */
  __IO uint32_t OUTMASK;                           /**< Output Mask, offset: 0x60 */
  __IO uint32_t COMBINE;                           /**< Function For Linked Channels, offset: 0x64 */
  __IO uint32_t DEADTIME;                          /**< Deadtime Configuration, offset: 0x68 */
  __IO uint32_t EXTTRIG;                           /**< FTM External Trigger, offset: 0x6C */
  __IO uint32_t POL;                               /**< Channels Polarity, offset: 0x70 */
  __IO uint32_t FMS;                               /**< Fault Mode Status, offset: 0x74 */
  __IO uint32_t FILTER;                            /**< Input Capture Filter Control, offset: 0x78 */
  __IO uint32_t FLTCTRL;                           /**< Fault Control, offset: 0x7C */
  __IO uint32_t QDCTRL;                            /**< Quadrature Decoder Control And Status, offset: 0x80, available only on: FTM1, FTM2, FTM4 (missing on FTM0, FTM3, FTM5, FTM6, FTM7) */
  __IO uint32_t CONF;                              /**< Configuration, offset: 0x84 */
  __IO uint32_t FLTPOL;                            /**< FTM Fault Input Polarity, offset: 0x88 */
  __IO uint32_t SYNCONF;                           /**< Synchronization Configuration, offset: 0x8C */
  __IO uint32_t INVCTRL;                           /**< FTM Inverting Control, offset: 0x90 */
  __IO uint32_t SWOCTRL;                           /**< FTM Software Output Control, offset: 0x94 */
  __IO uint32_t PWMLOAD;                           /**< FTM PWM Load, offset: 0x98 */
  __IO uint32_t HCR;                               /**< Half Cycle Register, offset: 0x9C */
  __IO uint32_t PAIR0DEADTIME;                     /**< Pair 0 Deadtime Configuration, offset: 0xA0 */
       uint8_t RESERVED_0[4];
  __IO uint32_t PAIR1DEADTIME;                     /**< Pair 1 Deadtime Configuration, offset: 0xA8 */
       uint8_t RESERVED_1[4];
  __IO uint32_t PAIR2DEADTIME;                     /**< Pair 2 Deadtime Configuration, offset: 0xB0 */
       uint8_t RESERVED_2[4];
  __IO uint32_t PAIR3DEADTIME;                     /**< Pair 3 Deadtime Configuration, offset: 0xB8 */
       uint8_t RESERVED_3[324];
  __IO uint32_t MOD_MIRROR;                        /**< Mirror of Modulo Value, offset: 0x200, available only on: FTM1, FTM2 (missing on FTM0, FTM3, FTM4, FTM5, FTM6, FTM7) */
  __IO uint32_t CV_MIRROR[FTM_CV_MIRROR_COUNT];    /**< Mirror of Channel (n) Match Value, array offset: 0x204, array step: 0x4, available only on: FTM1, FTM2 (missing on FTM0, FTM3, FTM4, FTM5, FTM6, FTM7) */
} FTM_Type;

/* ----------------------------------------------------------------------------
   -- FTM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Register_Masks FTM Register Masks
 * @{
 */

/*! @name SC - Status And Control */
/*! @{ */

#define FTM_SC_PS_MASK                           (0x7U)
#define FTM_SC_PS_SHIFT                          (0U)
/*! PS - Prescale Factor Selection
 *  0b000..Divide by 1
 *  0b001..Divide by 2
 *  0b010..Divide by 4
 *  0b011..Divide by 8
 *  0b100..Divide by 16
 *  0b101..Divide by 32
 *  0b110..Divide by 64
 *  0b111..Divide by 128
 */
#define FTM_SC_PS(x)                             (((uint32_t)(((uint32_t)(x)) << FTM_SC_PS_SHIFT)) & FTM_SC_PS_MASK)

#define FTM_SC_CLKS_MASK                         (0x18U)
#define FTM_SC_CLKS_SHIFT                        (3U)
/*! CLKS - Clock Source Selection
 *  0b00..No clock selected. This in effect disables the FTM counter.
 *  0b01..FTM input clock
 *  0b10..Fixed frequency clock
 *  0b11..External clock
 */
#define FTM_SC_CLKS(x)                           (((uint32_t)(((uint32_t)(x)) << FTM_SC_CLKS_SHIFT)) & FTM_SC_CLKS_MASK)

#define FTM_SC_CPWMS_MASK                        (0x20U)
#define FTM_SC_CPWMS_SHIFT                       (5U)
/*! CPWMS - Center-Aligned PWM Select
 *  0b0..FTM counter operates in Up Counting mode.
 *  0b1..FTM counter operates in Up-Down Counting mode.
 */
#define FTM_SC_CPWMS(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_SC_CPWMS_SHIFT)) & FTM_SC_CPWMS_MASK)

#define FTM_SC_RIE_MASK                          (0x40U)
#define FTM_SC_RIE_SHIFT                         (6U)
/*! RIE - Reload Point Interrupt Enable
 *  0b0..Reload point interrupt is disabled.
 *  0b1..Reload point interrupt is enabled.
 */
#define FTM_SC_RIE(x)                            (((uint32_t)(((uint32_t)(x)) << FTM_SC_RIE_SHIFT)) & FTM_SC_RIE_MASK)

#define FTM_SC_RF_MASK                           (0x80U)
#define FTM_SC_RF_SHIFT                          (7U)
/*! RF - Reload Flag
 *  0b0..A selected reload point did not happen.
 *  0b1..A selected reload point happened.
 */
#define FTM_SC_RF(x)                             (((uint32_t)(((uint32_t)(x)) << FTM_SC_RF_SHIFT)) & FTM_SC_RF_MASK)

#define FTM_SC_TOIE_MASK                         (0x100U)
#define FTM_SC_TOIE_SHIFT                        (8U)
/*! TOIE - Timer Overflow Interrupt Enable
 *  0b0..Disable TOF interrupts. Use software polling.
 *  0b1..Enable TOF interrupts. An interrupt is generated when TOF equals one.
 */
#define FTM_SC_TOIE(x)                           (((uint32_t)(((uint32_t)(x)) << FTM_SC_TOIE_SHIFT)) & FTM_SC_TOIE_MASK)

#define FTM_SC_TOF_MASK                          (0x200U)
#define FTM_SC_TOF_SHIFT                         (9U)
/*! TOF - Timer Overflow Flag
 *  0b0..FTM counter has not overflowed.
 *  0b1..FTM counter has overflowed.
 */
#define FTM_SC_TOF(x)                            (((uint32_t)(((uint32_t)(x)) << FTM_SC_TOF_SHIFT)) & FTM_SC_TOF_MASK)

#define FTM_SC_PWMEN0_MASK                       (0x10000U)
#define FTM_SC_PWMEN0_SHIFT                      (16U)
/*! PWMEN0 - Channel 0 PWM enable bit
 *  0b0..Channel output port is disabled.
 *  0b1..Channel output port is enabled.
 */
#define FTM_SC_PWMEN0(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN0_SHIFT)) & FTM_SC_PWMEN0_MASK)

#define FTM_SC_PWMEN1_MASK                       (0x20000U)
#define FTM_SC_PWMEN1_SHIFT                      (17U)
/*! PWMEN1 - Channel 1 PWM enable bit
 *  0b0..Channel output port is disabled.
 *  0b1..Channel output port is enabled.
 */
#define FTM_SC_PWMEN1(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN1_SHIFT)) & FTM_SC_PWMEN1_MASK)

#define FTM_SC_PWMEN2_MASK                       (0x40000U)
#define FTM_SC_PWMEN2_SHIFT                      (18U)
/*! PWMEN2 - Channel 2 PWM enable bit
 *  0b0..Channel output port is disabled.
 *  0b1..Channel output port is enabled.
 */
#define FTM_SC_PWMEN2(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN2_SHIFT)) & FTM_SC_PWMEN2_MASK)

#define FTM_SC_PWMEN3_MASK                       (0x80000U)
#define FTM_SC_PWMEN3_SHIFT                      (19U)
/*! PWMEN3 - Channel 3 PWM enable bit
 *  0b0..Channel output port is disabled.
 *  0b1..Channel output port is enabled.
 */
#define FTM_SC_PWMEN3(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN3_SHIFT)) & FTM_SC_PWMEN3_MASK)

#define FTM_SC_PWMEN4_MASK                       (0x100000U)
#define FTM_SC_PWMEN4_SHIFT                      (20U)
/*! PWMEN4 - Channel 4 PWM enable bit
 *  0b0..Channel output port is disabled.
 *  0b1..Channel output port is enabled.
 */
#define FTM_SC_PWMEN4(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN4_SHIFT)) & FTM_SC_PWMEN4_MASK)

#define FTM_SC_PWMEN5_MASK                       (0x200000U)
#define FTM_SC_PWMEN5_SHIFT                      (21U)
/*! PWMEN5 - Channel 5 PWM enable bit
 *  0b0..Channel output port is disabled.
 *  0b1..Channel output port is enabled.
 */
#define FTM_SC_PWMEN5(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN5_SHIFT)) & FTM_SC_PWMEN5_MASK)

#define FTM_SC_PWMEN6_MASK                       (0x400000U)
#define FTM_SC_PWMEN6_SHIFT                      (22U)
/*! PWMEN6 - Channel 6 PWM enable bit
 *  0b0..Channel output port is disabled.
 *  0b1..Channel output port is enabled.
 */
#define FTM_SC_PWMEN6(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN6_SHIFT)) & FTM_SC_PWMEN6_MASK)

#define FTM_SC_PWMEN7_MASK                       (0x800000U)
#define FTM_SC_PWMEN7_SHIFT                      (23U)
/*! PWMEN7 - Channel 7 PWM enable bit
 *  0b0..Channel output port is disabled.
 *  0b1..Channel output port is enabled.
 */
#define FTM_SC_PWMEN7(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_SC_PWMEN7_SHIFT)) & FTM_SC_PWMEN7_MASK)

#define FTM_SC_FLTPS_MASK                        (0xF000000U)
#define FTM_SC_FLTPS_SHIFT                       (24U)
/*! FLTPS - Filter Prescaler
 *  0b0000..Divide by 1
 *  0b0001..Divide by 2
 *  0b0010..Divide by 3
 *  0b0011..Divide by 4
 *  0b0100..Divide by 5
 *  0b0101..Divide by 6
 *  0b0110..Divide by 7
 *  0b0111..Divide by 8
 *  0b1000..Divide by 9
 *  0b1001..Divide by 10
 *  0b1010..Divide by 11
 *  0b1011..Divide by 12
 *  0b1100..Divide by 13
 *  0b1101..Divide by 14
 *  0b1110..Divide by 15
 *  0b1111..Divide by 16
 */
#define FTM_SC_FLTPS(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_SC_FLTPS_SHIFT)) & FTM_SC_FLTPS_MASK)
/*! @} */

/*! @name CNT - Counter */
/*! @{ */

#define FTM_CNT_COUNT_MASK                       (0xFFFFU)
#define FTM_CNT_COUNT_SHIFT                      (0U)
/*! COUNT - Counter Value */
#define FTM_CNT_COUNT(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CNT_COUNT_SHIFT)) & FTM_CNT_COUNT_MASK)
/*! @} */

/*! @name MOD - Modulo */
/*! @{ */

#define FTM_MOD_MOD_MASK                         (0xFFFFU)
#define FTM_MOD_MOD_SHIFT                        (0U)
/*! MOD - MOD */
#define FTM_MOD_MOD(x)                           (((uint32_t)(((uint32_t)(x)) << FTM_MOD_MOD_SHIFT)) & FTM_MOD_MOD_MASK)
/*! @} */

/*! @name CnSC - Channel (n) Status And Control */
/*! @{ */

#define FTM_CnSC_DMA_MASK                        (0x1U)
#define FTM_CnSC_DMA_SHIFT                       (0U)
/*! DMA - DMA Enable
 *  0b0..Disable DMA transfers.
 *  0b1..Enable DMA transfers.
 */
#define FTM_CnSC_DMA(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_DMA_SHIFT)) & FTM_CnSC_DMA_MASK)

#define FTM_CnSC_ICRST_MASK                      (0x2U)
#define FTM_CnSC_ICRST_SHIFT                     (1U)
/*! ICRST - FTM counter reset by the selected input capture event.
 *  0b0..FTM counter is not reset when the selected channel (n) input event is detected.
 *  0b1..FTM counter is reset when the selected channel (n) input event is detected.
 */
#define FTM_CnSC_ICRST(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_ICRST_SHIFT)) & FTM_CnSC_ICRST_MASK)

#define FTM_CnSC_ELSA_MASK                       (0x4U)
#define FTM_CnSC_ELSA_SHIFT                      (2U)
/*! ELSA - Channel (n) Edge or Level Select */
#define FTM_CnSC_ELSA(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_ELSA_SHIFT)) & FTM_CnSC_ELSA_MASK)

#define FTM_CnSC_ELSB_MASK                       (0x8U)
#define FTM_CnSC_ELSB_SHIFT                      (3U)
/*! ELSB - Channel (n) Edge or Level Select */
#define FTM_CnSC_ELSB(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_ELSB_SHIFT)) & FTM_CnSC_ELSB_MASK)

#define FTM_CnSC_MSA_MASK                        (0x10U)
#define FTM_CnSC_MSA_SHIFT                       (4U)
/*! MSA - Channel (n) Mode Select */
#define FTM_CnSC_MSA(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_MSA_SHIFT)) & FTM_CnSC_MSA_MASK)

#define FTM_CnSC_MSB_MASK                        (0x20U)
#define FTM_CnSC_MSB_SHIFT                       (5U)
/*! MSB - Channel (n) Mode Select */
#define FTM_CnSC_MSB(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_MSB_SHIFT)) & FTM_CnSC_MSB_MASK)

#define FTM_CnSC_CHIE_MASK                       (0x40U)
#define FTM_CnSC_CHIE_SHIFT                      (6U)
/*! CHIE - Channel (n) Interrupt Enable
 *  0b0..Disable channel (n) interrupt. Use software polling.
 *  0b1..Enable channel (n) interrupt.
 */
#define FTM_CnSC_CHIE(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_CHIE_SHIFT)) & FTM_CnSC_CHIE_MASK)

#define FTM_CnSC_CHF_MASK                        (0x80U)
#define FTM_CnSC_CHF_SHIFT                       (7U)
/*! CHF - Channel (n) Flag
 *  0b0..No channel (n) event has occurred.
 *  0b1..A channel (n) event has occurred.
 */
#define FTM_CnSC_CHF(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_CHF_SHIFT)) & FTM_CnSC_CHF_MASK)

#define FTM_CnSC_TRIGMODE_MASK                   (0x100U)
#define FTM_CnSC_TRIGMODE_SHIFT                  (8U)
/*! TRIGMODE - Trigger mode control
 *  0b0..Channel outputs will generate the normal PWM outputs without generating a pulse.
 *  0b1..If a match in the channel occurs, a trigger generation on channel output will happen. The trigger pulse width has one FTM clock cycle.
 */
#define FTM_CnSC_TRIGMODE(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_TRIGMODE_SHIFT)) & FTM_CnSC_TRIGMODE_MASK)

#define FTM_CnSC_CHIS_MASK                       (0x200U)
#define FTM_CnSC_CHIS_SHIFT                      (9U)
/*! CHIS - Channel (n) Input State
 *  0b0..The channel (n) input is zero.
 *  0b1..The channel (n) input is one.
 */
#define FTM_CnSC_CHIS(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_CHIS_SHIFT)) & FTM_CnSC_CHIS_MASK)

#define FTM_CnSC_CHOV_MASK                       (0x400U)
#define FTM_CnSC_CHOV_SHIFT                      (10U)
/*! CHOV - Channel (n) Output Value
 *  0b0..The channel (n) output is zero.
 *  0b1..The channel (n) output is one.
 */
#define FTM_CnSC_CHOV(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CnSC_CHOV_SHIFT)) & FTM_CnSC_CHOV_MASK)
/*! @} */

/* The count of FTM_CnSC */
#define FTM_CnSC_COUNT                           (8U)

/*! @name CnV - Channel (n) Value */
/*! @{ */

#define FTM_CnV_VAL_MASK                         (0xFFFFU)
#define FTM_CnV_VAL_SHIFT                        (0U)
/*! VAL - Channel Value */
#define FTM_CnV_VAL(x)                           (((uint32_t)(((uint32_t)(x)) << FTM_CnV_VAL_SHIFT)) & FTM_CnV_VAL_MASK)
/*! @} */

/* The count of FTM_CnV */
#define FTM_CnV_COUNT                            (8U)

/*! @name CNTIN - Counter Initial Value */
/*! @{ */

#define FTM_CNTIN_INIT_MASK                      (0xFFFFU)
#define FTM_CNTIN_INIT_SHIFT                     (0U)
/*! INIT - INIT */
#define FTM_CNTIN_INIT(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_CNTIN_INIT_SHIFT)) & FTM_CNTIN_INIT_MASK)
/*! @} */

/*! @name STATUS - Capture And Compare Status */
/*! @{ */

#define FTM_STATUS_CH0F_MASK                     (0x1U)
#define FTM_STATUS_CH0F_SHIFT                    (0U)
/*! CH0F - Channel 0 Flag
 *  0b0..No channel event has occurred.
 *  0b1..A channel event has occurred.
 */
#define FTM_STATUS_CH0F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH0F_SHIFT)) & FTM_STATUS_CH0F_MASK)

#define FTM_STATUS_CH1F_MASK                     (0x2U)
#define FTM_STATUS_CH1F_SHIFT                    (1U)
/*! CH1F - Channel 1 Flag
 *  0b0..No channel event has occurred.
 *  0b1..A channel event has occurred.
 */
#define FTM_STATUS_CH1F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH1F_SHIFT)) & FTM_STATUS_CH1F_MASK)

#define FTM_STATUS_CH2F_MASK                     (0x4U)
#define FTM_STATUS_CH2F_SHIFT                    (2U)
/*! CH2F - Channel 2 Flag
 *  0b0..No channel event has occurred.
 *  0b1..A channel event has occurred.
 */
#define FTM_STATUS_CH2F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH2F_SHIFT)) & FTM_STATUS_CH2F_MASK)

#define FTM_STATUS_CH3F_MASK                     (0x8U)
#define FTM_STATUS_CH3F_SHIFT                    (3U)
/*! CH3F - Channel 3 Flag
 *  0b0..No channel event has occurred.
 *  0b1..A channel event has occurred.
 */
#define FTM_STATUS_CH3F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH3F_SHIFT)) & FTM_STATUS_CH3F_MASK)

#define FTM_STATUS_CH4F_MASK                     (0x10U)
#define FTM_STATUS_CH4F_SHIFT                    (4U)
/*! CH4F - Channel 4 Flag
 *  0b0..No channel event has occurred.
 *  0b1..A channel event has occurred.
 */
#define FTM_STATUS_CH4F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH4F_SHIFT)) & FTM_STATUS_CH4F_MASK)

#define FTM_STATUS_CH5F_MASK                     (0x20U)
#define FTM_STATUS_CH5F_SHIFT                    (5U)
/*! CH5F - Channel 5 Flag
 *  0b0..No channel event has occurred.
 *  0b1..A channel event has occurred.
 */
#define FTM_STATUS_CH5F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH5F_SHIFT)) & FTM_STATUS_CH5F_MASK)

#define FTM_STATUS_CH6F_MASK                     (0x40U)
#define FTM_STATUS_CH6F_SHIFT                    (6U)
/*! CH6F - Channel 6 Flag
 *  0b0..No channel event has occurred.
 *  0b1..A channel event has occurred.
 */
#define FTM_STATUS_CH6F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH6F_SHIFT)) & FTM_STATUS_CH6F_MASK)

#define FTM_STATUS_CH7F_MASK                     (0x80U)
#define FTM_STATUS_CH7F_SHIFT                    (7U)
/*! CH7F - Channel 7 Flag
 *  0b0..No channel event has occurred.
 *  0b1..A channel event has occurred.
 */
#define FTM_STATUS_CH7F(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_STATUS_CH7F_SHIFT)) & FTM_STATUS_CH7F_MASK)
/*! @} */

/*! @name MODE - Features Mode Selection */
/*! @{ */

#define FTM_MODE_FTMEN_MASK                      (0x1U)
#define FTM_MODE_FTMEN_SHIFT                     (0U)
/*! FTMEN - FTM Enable
 *  0b0..TPM compatibility. Free running counter and synchronization compatible with TPM.
 *  0b1..Free running counter and synchronization are different from TPM behavior.
 */
#define FTM_MODE_FTMEN(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_MODE_FTMEN_SHIFT)) & FTM_MODE_FTMEN_MASK)

#define FTM_MODE_INIT_MASK                       (0x2U)
#define FTM_MODE_INIT_SHIFT                      (1U)
/*! INIT - Initialize The Channels Output */
#define FTM_MODE_INIT(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_MODE_INIT_SHIFT)) & FTM_MODE_INIT_MASK)

#define FTM_MODE_WPDIS_MASK                      (0x4U)
#define FTM_MODE_WPDIS_SHIFT                     (2U)
/*! WPDIS - Write Protection Disable
 *  0b0..Write protection is enabled.
 *  0b1..Write protection is disabled.
 */
#define FTM_MODE_WPDIS(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_MODE_WPDIS_SHIFT)) & FTM_MODE_WPDIS_MASK)

#define FTM_MODE_PWMSYNC_MASK                    (0x8U)
#define FTM_MODE_PWMSYNC_SHIFT                   (3U)
/*! PWMSYNC - PWM Synchronization Mode
 *  0b0..No restrictions. Software and hardware triggers can be used by MOD, CnV, OUTMASK, and FTM counter synchronization.
 *  0b1..Software trigger can only be used by MOD and CnV synchronization, and hardware triggers can only be used
 *       by OUTMASK and FTM counter synchronization.
 */
#define FTM_MODE_PWMSYNC(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_MODE_PWMSYNC_SHIFT)) & FTM_MODE_PWMSYNC_MASK)

#define FTM_MODE_CAPTEST_MASK                    (0x10U)
#define FTM_MODE_CAPTEST_SHIFT                   (4U)
/*! CAPTEST - Capture Test Mode Enable
 *  0b0..Capture test mode is disabled.
 *  0b1..Capture test mode is enabled.
 */
#define FTM_MODE_CAPTEST(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_MODE_CAPTEST_SHIFT)) & FTM_MODE_CAPTEST_MASK)

#define FTM_MODE_FAULTM_MASK                     (0x60U)
#define FTM_MODE_FAULTM_SHIFT                    (5U)
/*! FAULTM - Fault Control Mode
 *  0b00..Fault control is disabled for all channels.
 *  0b01..Fault control is enabled for even channels only (channels 0, 2, 4, and 6), and the selected mode is the manual fault clearing.
 *  0b10..Fault control is enabled for all channels, and the selected mode is the manual fault clearing.
 *  0b11..Fault control is enabled for all channels, and the selected mode is the automatic fault clearing.
 */
#define FTM_MODE_FAULTM(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_MODE_FAULTM_SHIFT)) & FTM_MODE_FAULTM_MASK)

#define FTM_MODE_FAULTIE_MASK                    (0x80U)
#define FTM_MODE_FAULTIE_SHIFT                   (7U)
/*! FAULTIE - Fault Interrupt Enable
 *  0b0..Fault control interrupt is disabled.
 *  0b1..Fault control interrupt is enabled.
 */
#define FTM_MODE_FAULTIE(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_MODE_FAULTIE_SHIFT)) & FTM_MODE_FAULTIE_MASK)
/*! @} */

/*! @name SYNC - Synchronization */
/*! @{ */

#define FTM_SYNC_CNTMIN_MASK                     (0x1U)
#define FTM_SYNC_CNTMIN_SHIFT                    (0U)
/*! CNTMIN - Minimum Loading Point Enable
 *  0b0..The minimum loading point is disabled.
 *  0b1..The minimum loading point is enabled.
 */
#define FTM_SYNC_CNTMIN(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_CNTMIN_SHIFT)) & FTM_SYNC_CNTMIN_MASK)

#define FTM_SYNC_CNTMAX_MASK                     (0x2U)
#define FTM_SYNC_CNTMAX_SHIFT                    (1U)
/*! CNTMAX - Maximum Loading Point Enable
 *  0b0..The maximum loading point is disabled.
 *  0b1..The maximum loading point is enabled.
 */
#define FTM_SYNC_CNTMAX(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_CNTMAX_SHIFT)) & FTM_SYNC_CNTMAX_MASK)

#define FTM_SYNC_REINIT_MASK                     (0x4U)
#define FTM_SYNC_REINIT_SHIFT                    (2U)
/*! REINIT - FTM Counter Reinitialization by Synchronization
 *  0b0..FTM counter continues to count normally.
 *  0b1..FTM counter is updated with its initial value when the selected trigger is detected.
 */
#define FTM_SYNC_REINIT(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_REINIT_SHIFT)) & FTM_SYNC_REINIT_MASK)

#define FTM_SYNC_SYNCHOM_MASK                    (0x8U)
#define FTM_SYNC_SYNCHOM_SHIFT                   (3U)
/*! SYNCHOM - Output Mask Synchronization
 *  0b0..OUTMASK register is updated with the value of its buffer in all rising edges of the FTM input clock.
 *  0b1..OUTMASK register is updated with the value of its buffer only by the PWM synchronization.
 */
#define FTM_SYNC_SYNCHOM(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_SYNCHOM_SHIFT)) & FTM_SYNC_SYNCHOM_MASK)

#define FTM_SYNC_TRIG0_MASK                      (0x10U)
#define FTM_SYNC_TRIG0_SHIFT                     (4U)
/*! TRIG0 - PWM Synchronization Hardware Trigger 0
 *  0b0..Trigger is disabled.
 *  0b1..Trigger is enabled.
 */
#define FTM_SYNC_TRIG0(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_TRIG0_SHIFT)) & FTM_SYNC_TRIG0_MASK)

#define FTM_SYNC_TRIG1_MASK                      (0x20U)
#define FTM_SYNC_TRIG1_SHIFT                     (5U)
/*! TRIG1 - PWM Synchronization Hardware Trigger 1
 *  0b0..Trigger is disabled.
 *  0b1..Trigger is enabled.
 */
#define FTM_SYNC_TRIG1(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_TRIG1_SHIFT)) & FTM_SYNC_TRIG1_MASK)

#define FTM_SYNC_TRIG2_MASK                      (0x40U)
#define FTM_SYNC_TRIG2_SHIFT                     (6U)
/*! TRIG2 - PWM Synchronization Hardware Trigger 2
 *  0b0..Trigger is disabled.
 *  0b1..Trigger is enabled.
 */
#define FTM_SYNC_TRIG2(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_TRIG2_SHIFT)) & FTM_SYNC_TRIG2_MASK)

#define FTM_SYNC_SWSYNC_MASK                     (0x80U)
#define FTM_SYNC_SWSYNC_SHIFT                    (7U)
/*! SWSYNC - PWM Synchronization Software Trigger
 *  0b0..Software trigger is not selected.
 *  0b1..Software trigger is selected.
 */
#define FTM_SYNC_SWSYNC(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_SYNC_SWSYNC_SHIFT)) & FTM_SYNC_SWSYNC_MASK)
/*! @} */

/*! @name OUTINIT - Initial State For Channels Output */
/*! @{ */

#define FTM_OUTINIT_CH0OI_MASK                   (0x1U)
#define FTM_OUTINIT_CH0OI_SHIFT                  (0U)
/*! CH0OI - Channel 0 Output Initialization Value
 *  0b0..The initialization value is 0.
 *  0b1..The initialization value is 1.
 */
#define FTM_OUTINIT_CH0OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH0OI_SHIFT)) & FTM_OUTINIT_CH0OI_MASK)

#define FTM_OUTINIT_CH1OI_MASK                   (0x2U)
#define FTM_OUTINIT_CH1OI_SHIFT                  (1U)
/*! CH1OI - Channel 1 Output Initialization Value
 *  0b0..The initialization value is 0.
 *  0b1..The initialization value is 1.
 */
#define FTM_OUTINIT_CH1OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH1OI_SHIFT)) & FTM_OUTINIT_CH1OI_MASK)

#define FTM_OUTINIT_CH2OI_MASK                   (0x4U)
#define FTM_OUTINIT_CH2OI_SHIFT                  (2U)
/*! CH2OI - Channel 2 Output Initialization Value
 *  0b0..The initialization value is 0.
 *  0b1..The initialization value is 1.
 */
#define FTM_OUTINIT_CH2OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH2OI_SHIFT)) & FTM_OUTINIT_CH2OI_MASK)

#define FTM_OUTINIT_CH3OI_MASK                   (0x8U)
#define FTM_OUTINIT_CH3OI_SHIFT                  (3U)
/*! CH3OI - Channel 3 Output Initialization Value
 *  0b0..The initialization value is 0.
 *  0b1..The initialization value is 1.
 */
#define FTM_OUTINIT_CH3OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH3OI_SHIFT)) & FTM_OUTINIT_CH3OI_MASK)

#define FTM_OUTINIT_CH4OI_MASK                   (0x10U)
#define FTM_OUTINIT_CH4OI_SHIFT                  (4U)
/*! CH4OI - Channel 4 Output Initialization Value
 *  0b0..The initialization value is 0.
 *  0b1..The initialization value is 1.
 */
#define FTM_OUTINIT_CH4OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH4OI_SHIFT)) & FTM_OUTINIT_CH4OI_MASK)

#define FTM_OUTINIT_CH5OI_MASK                   (0x20U)
#define FTM_OUTINIT_CH5OI_SHIFT                  (5U)
/*! CH5OI - Channel 5 Output Initialization Value
 *  0b0..The initialization value is 0.
 *  0b1..The initialization value is 1.
 */
#define FTM_OUTINIT_CH5OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH5OI_SHIFT)) & FTM_OUTINIT_CH5OI_MASK)

#define FTM_OUTINIT_CH6OI_MASK                   (0x40U)
#define FTM_OUTINIT_CH6OI_SHIFT                  (6U)
/*! CH6OI - Channel 6 Output Initialization Value
 *  0b0..The initialization value is 0.
 *  0b1..The initialization value is 1.
 */
#define FTM_OUTINIT_CH6OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH6OI_SHIFT)) & FTM_OUTINIT_CH6OI_MASK)

#define FTM_OUTINIT_CH7OI_MASK                   (0x80U)
#define FTM_OUTINIT_CH7OI_SHIFT                  (7U)
/*! CH7OI - Channel 7 Output Initialization Value
 *  0b0..The initialization value is 0.
 *  0b1..The initialization value is 1.
 */
#define FTM_OUTINIT_CH7OI(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTINIT_CH7OI_SHIFT)) & FTM_OUTINIT_CH7OI_MASK)
/*! @} */

/*! @name OUTMASK - Output Mask */
/*! @{ */

#define FTM_OUTMASK_CH0OM_MASK                   (0x1U)
#define FTM_OUTMASK_CH0OM_SHIFT                  (0U)
/*! CH0OM - Channel 0 Output Mask
 *  0b0..Channel output is not masked. It continues to operate normally.
 *  0b1..Channel output is masked. It is forced to its inactive state.
 */
#define FTM_OUTMASK_CH0OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH0OM_SHIFT)) & FTM_OUTMASK_CH0OM_MASK)

#define FTM_OUTMASK_CH1OM_MASK                   (0x2U)
#define FTM_OUTMASK_CH1OM_SHIFT                  (1U)
/*! CH1OM - Channel 1 Output Mask
 *  0b0..Channel output is not masked. It continues to operate normally.
 *  0b1..Channel output is masked. It is forced to its inactive state.
 */
#define FTM_OUTMASK_CH1OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH1OM_SHIFT)) & FTM_OUTMASK_CH1OM_MASK)

#define FTM_OUTMASK_CH2OM_MASK                   (0x4U)
#define FTM_OUTMASK_CH2OM_SHIFT                  (2U)
/*! CH2OM - Channel 2 Output Mask
 *  0b0..Channel output is not masked. It continues to operate normally.
 *  0b1..Channel output is masked. It is forced to its inactive state.
 */
#define FTM_OUTMASK_CH2OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH2OM_SHIFT)) & FTM_OUTMASK_CH2OM_MASK)

#define FTM_OUTMASK_CH3OM_MASK                   (0x8U)
#define FTM_OUTMASK_CH3OM_SHIFT                  (3U)
/*! CH3OM - Channel 3 Output Mask
 *  0b0..Channel output is not masked. It continues to operate normally.
 *  0b1..Channel output is masked. It is forced to its inactive state.
 */
#define FTM_OUTMASK_CH3OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH3OM_SHIFT)) & FTM_OUTMASK_CH3OM_MASK)

#define FTM_OUTMASK_CH4OM_MASK                   (0x10U)
#define FTM_OUTMASK_CH4OM_SHIFT                  (4U)
/*! CH4OM - Channel 4 Output Mask
 *  0b0..Channel output is not masked. It continues to operate normally.
 *  0b1..Channel output is masked. It is forced to its inactive state.
 */
#define FTM_OUTMASK_CH4OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH4OM_SHIFT)) & FTM_OUTMASK_CH4OM_MASK)

#define FTM_OUTMASK_CH5OM_MASK                   (0x20U)
#define FTM_OUTMASK_CH5OM_SHIFT                  (5U)
/*! CH5OM - Channel 5 Output Mask
 *  0b0..Channel output is not masked. It continues to operate normally.
 *  0b1..Channel output is masked. It is forced to its inactive state.
 */
#define FTM_OUTMASK_CH5OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH5OM_SHIFT)) & FTM_OUTMASK_CH5OM_MASK)

#define FTM_OUTMASK_CH6OM_MASK                   (0x40U)
#define FTM_OUTMASK_CH6OM_SHIFT                  (6U)
/*! CH6OM - Channel 6 Output Mask
 *  0b0..Channel output is not masked. It continues to operate normally.
 *  0b1..Channel output is masked. It is forced to its inactive state.
 */
#define FTM_OUTMASK_CH6OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH6OM_SHIFT)) & FTM_OUTMASK_CH6OM_MASK)

#define FTM_OUTMASK_CH7OM_MASK                   (0x80U)
#define FTM_OUTMASK_CH7OM_SHIFT                  (7U)
/*! CH7OM - Channel 7 Output Mask
 *  0b0..Channel output is not masked. It continues to operate normally.
 *  0b1..Channel output is masked. It is forced to its inactive state.
 */
#define FTM_OUTMASK_CH7OM(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_OUTMASK_CH7OM_SHIFT)) & FTM_OUTMASK_CH7OM_MASK)
/*! @} */

/*! @name COMBINE - Function For Linked Channels */
/*! @{ */

#define FTM_COMBINE_COMBINE0_MASK                (0x1U)
#define FTM_COMBINE_COMBINE0_SHIFT               (0U)
/*! COMBINE0 - Combine Channels For n = 0 */
#define FTM_COMBINE_COMBINE0(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMBINE0_SHIFT)) & FTM_COMBINE_COMBINE0_MASK)

#define FTM_COMBINE_COMP0_MASK                   (0x2U)
#define FTM_COMBINE_COMP0_SHIFT                  (1U)
/*! COMP0 - Complement Of Channel (n) For n = 0
 *  0b0..If the channels (n) and (n+1) are in Combine Mode or Modified Combine PWM Mode, the channel (n+1) output
 *       is the same as the channel (n) output. If the channel (n+1) is in Output Compare Mode, EPWM or CPWM, the
 *       channel (n+1) output is independent from channel (n) output.
 *  0b1..The channel (n+1) output is the complement of the channel (n) output.
 */
#define FTM_COMBINE_COMP0(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMP0_SHIFT)) & FTM_COMBINE_COMP0_MASK)

#define FTM_COMBINE_DECAPEN0_MASK                (0x4U)
#define FTM_COMBINE_DECAPEN0_SHIFT               (2U)
/*! DECAPEN0 - Dual Edge Capture Mode Enable For n = 0 */
#define FTM_COMBINE_DECAPEN0(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAPEN0_SHIFT)) & FTM_COMBINE_DECAPEN0_MASK)

#define FTM_COMBINE_DECAP0_MASK                  (0x8U)
#define FTM_COMBINE_DECAP0_SHIFT                 (3U)
/*! DECAP0 - Dual Edge Capture Mode Captures For n = 0
 *  0b0..The dual edge captures are inactive.
 *  0b1..The dual edge captures are active.
 */
#define FTM_COMBINE_DECAP0(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAP0_SHIFT)) & FTM_COMBINE_DECAP0_MASK)

#define FTM_COMBINE_DTEN0_MASK                   (0x10U)
#define FTM_COMBINE_DTEN0_SHIFT                  (4U)
/*! DTEN0 - Deadtime Enable For n = 0
 *  0b0..The deadtime insertion in this pair of channels is disabled.
 *  0b1..The deadtime insertion in this pair of channels is enabled.
 */
#define FTM_COMBINE_DTEN0(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DTEN0_SHIFT)) & FTM_COMBINE_DTEN0_MASK)

#define FTM_COMBINE_SYNCEN0_MASK                 (0x20U)
#define FTM_COMBINE_SYNCEN0_SHIFT                (5U)
/*! SYNCEN0 - Synchronization Enable For n = 0
 *  0b0..The PWM synchronization in this pair of channels is disabled.
 *  0b1..The PWM synchronization in this pair of channels is enabled.
 */
#define FTM_COMBINE_SYNCEN0(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_SYNCEN0_SHIFT)) & FTM_COMBINE_SYNCEN0_MASK)

#define FTM_COMBINE_FAULTEN0_MASK                (0x40U)
#define FTM_COMBINE_FAULTEN0_SHIFT               (6U)
/*! FAULTEN0 - Fault Control Enable For n = 0
 *  0b0..The fault control in this pair of channels is disabled.
 *  0b1..The fault control in this pair of channels is enabled.
 */
#define FTM_COMBINE_FAULTEN0(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_FAULTEN0_SHIFT)) & FTM_COMBINE_FAULTEN0_MASK)

#define FTM_COMBINE_MCOMBINE0_MASK               (0x80U)
#define FTM_COMBINE_MCOMBINE0_SHIFT              (7U)
/*! MCOMBINE0 - Modified Combine Mode For n = 0 */
#define FTM_COMBINE_MCOMBINE0(x)                 (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_MCOMBINE0_SHIFT)) & FTM_COMBINE_MCOMBINE0_MASK)

#define FTM_COMBINE_COMBINE1_MASK                (0x100U)
#define FTM_COMBINE_COMBINE1_SHIFT               (8U)
/*! COMBINE1 - Combine Channels For n = 2 */
#define FTM_COMBINE_COMBINE1(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMBINE1_SHIFT)) & FTM_COMBINE_COMBINE1_MASK)

#define FTM_COMBINE_COMP1_MASK                   (0x200U)
#define FTM_COMBINE_COMP1_SHIFT                  (9U)
/*! COMP1 - Complement Of Channel (n) For n = 2
 *  0b0..If the channels (n) and (n+1) are in Combine Mode or Modified Combine PWM Mode, the channel (n+1) output
 *       is the same as the channel (n) output. If the channel (n+1) is in Output Compare Mode, EPWM or CPWM, the
 *       channel (n+1) output is independent from channel (n) output.
 *  0b1..The channel (n+1) output is the complement of the channel (n) output.
 */
#define FTM_COMBINE_COMP1(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMP1_SHIFT)) & FTM_COMBINE_COMP1_MASK)

#define FTM_COMBINE_DECAPEN1_MASK                (0x400U)
#define FTM_COMBINE_DECAPEN1_SHIFT               (10U)
/*! DECAPEN1 - Dual Edge Capture Mode Enable For n = 2 */
#define FTM_COMBINE_DECAPEN1(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAPEN1_SHIFT)) & FTM_COMBINE_DECAPEN1_MASK)

#define FTM_COMBINE_DECAP1_MASK                  (0x800U)
#define FTM_COMBINE_DECAP1_SHIFT                 (11U)
/*! DECAP1 - Dual Edge Capture Mode Captures For n = 2
 *  0b0..The dual edge captures are inactive.
 *  0b1..The dual edge captures are active.
 */
#define FTM_COMBINE_DECAP1(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAP1_SHIFT)) & FTM_COMBINE_DECAP1_MASK)

#define FTM_COMBINE_DTEN1_MASK                   (0x1000U)
#define FTM_COMBINE_DTEN1_SHIFT                  (12U)
/*! DTEN1 - Deadtime Enable For n = 2
 *  0b0..The deadtime insertion in this pair of channels is disabled.
 *  0b1..The deadtime insertion in this pair of channels is enabled.
 */
#define FTM_COMBINE_DTEN1(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DTEN1_SHIFT)) & FTM_COMBINE_DTEN1_MASK)

#define FTM_COMBINE_SYNCEN1_MASK                 (0x2000U)
#define FTM_COMBINE_SYNCEN1_SHIFT                (13U)
/*! SYNCEN1 - Synchronization Enable For n = 2
 *  0b0..The PWM synchronization in this pair of channels is disabled.
 *  0b1..The PWM synchronization in this pair of channels is enabled.
 */
#define FTM_COMBINE_SYNCEN1(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_SYNCEN1_SHIFT)) & FTM_COMBINE_SYNCEN1_MASK)

#define FTM_COMBINE_FAULTEN1_MASK                (0x4000U)
#define FTM_COMBINE_FAULTEN1_SHIFT               (14U)
/*! FAULTEN1 - Fault Control Enable For n = 2
 *  0b0..The fault control in this pair of channels is disabled.
 *  0b1..The fault control in this pair of channels is enabled.
 */
#define FTM_COMBINE_FAULTEN1(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_FAULTEN1_SHIFT)) & FTM_COMBINE_FAULTEN1_MASK)

#define FTM_COMBINE_MCOMBINE1_MASK               (0x8000U)
#define FTM_COMBINE_MCOMBINE1_SHIFT              (15U)
/*! MCOMBINE1 - Modified Combine Mode For n = 2 */
#define FTM_COMBINE_MCOMBINE1(x)                 (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_MCOMBINE1_SHIFT)) & FTM_COMBINE_MCOMBINE1_MASK)

#define FTM_COMBINE_COMBINE2_MASK                (0x10000U)
#define FTM_COMBINE_COMBINE2_SHIFT               (16U)
/*! COMBINE2 - Combine Channels For n = 4 */
#define FTM_COMBINE_COMBINE2(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMBINE2_SHIFT)) & FTM_COMBINE_COMBINE2_MASK)

#define FTM_COMBINE_COMP2_MASK                   (0x20000U)
#define FTM_COMBINE_COMP2_SHIFT                  (17U)
/*! COMP2 - Complement Of Channel (n) For n = 4
 *  0b0..If the channels (n) and (n+1) are in Combine Mode or Modified Combine PWM Mode, the channel (n+1) output
 *       is the same as the channel (n) output. If the channel (n+1) is in Output Compare Mode, EPWM or CPWM, the
 *       channel (n+1) output is independent from channel (n) output.
 *  0b1..The channel (n+1) output is the complement of the channel (n) output.
 */
#define FTM_COMBINE_COMP2(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMP2_SHIFT)) & FTM_COMBINE_COMP2_MASK)

#define FTM_COMBINE_DECAPEN2_MASK                (0x40000U)
#define FTM_COMBINE_DECAPEN2_SHIFT               (18U)
/*! DECAPEN2 - Dual Edge Capture Mode Enable For n = 4 */
#define FTM_COMBINE_DECAPEN2(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAPEN2_SHIFT)) & FTM_COMBINE_DECAPEN2_MASK)

#define FTM_COMBINE_DECAP2_MASK                  (0x80000U)
#define FTM_COMBINE_DECAP2_SHIFT                 (19U)
/*! DECAP2 - Dual Edge Capture Mode Captures For n = 4
 *  0b0..The dual edge captures are inactive.
 *  0b1..The dual edge captures are active.
 */
#define FTM_COMBINE_DECAP2(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAP2_SHIFT)) & FTM_COMBINE_DECAP2_MASK)

#define FTM_COMBINE_DTEN2_MASK                   (0x100000U)
#define FTM_COMBINE_DTEN2_SHIFT                  (20U)
/*! DTEN2 - Deadtime Enable For n = 4
 *  0b0..The deadtime insertion in this pair of channels is disabled.
 *  0b1..The deadtime insertion in this pair of channels is enabled.
 */
#define FTM_COMBINE_DTEN2(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DTEN2_SHIFT)) & FTM_COMBINE_DTEN2_MASK)

#define FTM_COMBINE_SYNCEN2_MASK                 (0x200000U)
#define FTM_COMBINE_SYNCEN2_SHIFT                (21U)
/*! SYNCEN2 - Synchronization Enable For n = 4
 *  0b0..The PWM synchronization in this pair of channels is disabled.
 *  0b1..The PWM synchronization in this pair of channels is enabled.
 */
#define FTM_COMBINE_SYNCEN2(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_SYNCEN2_SHIFT)) & FTM_COMBINE_SYNCEN2_MASK)

#define FTM_COMBINE_FAULTEN2_MASK                (0x400000U)
#define FTM_COMBINE_FAULTEN2_SHIFT               (22U)
/*! FAULTEN2 - Fault Control Enable For n = 4
 *  0b0..The fault control in this pair of channels is disabled.
 *  0b1..The fault control in this pair of channels is enabled.
 */
#define FTM_COMBINE_FAULTEN2(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_FAULTEN2_SHIFT)) & FTM_COMBINE_FAULTEN2_MASK)

#define FTM_COMBINE_MCOMBINE2_MASK               (0x800000U)
#define FTM_COMBINE_MCOMBINE2_SHIFT              (23U)
/*! MCOMBINE2 - Modified Combine Mode For n = 4 */
#define FTM_COMBINE_MCOMBINE2(x)                 (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_MCOMBINE2_SHIFT)) & FTM_COMBINE_MCOMBINE2_MASK)

#define FTM_COMBINE_COMBINE3_MASK                (0x1000000U)
#define FTM_COMBINE_COMBINE3_SHIFT               (24U)
/*! COMBINE3 - Combine Channels For n = 6 */
#define FTM_COMBINE_COMBINE3(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMBINE3_SHIFT)) & FTM_COMBINE_COMBINE3_MASK)

#define FTM_COMBINE_COMP3_MASK                   (0x2000000U)
#define FTM_COMBINE_COMP3_SHIFT                  (25U)
/*! COMP3 - Complement Of Channel (n) for n = 6
 *  0b0..If the channels (n) and (n+1) are in Combine Mode or Modified Combine PWM Mode, the channel (n+1) output
 *       is the same as the channel (n) output. If the channel (n+1) is in Output Compare Mode, EPWM or CPWM, the
 *       channel (n+1) output is independent from channel (n) output.
 *  0b1..The channel (n+1) output is the complement of the channel (n) output.
 */
#define FTM_COMBINE_COMP3(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_COMP3_SHIFT)) & FTM_COMBINE_COMP3_MASK)

#define FTM_COMBINE_DECAPEN3_MASK                (0x4000000U)
#define FTM_COMBINE_DECAPEN3_SHIFT               (26U)
/*! DECAPEN3 - Dual Edge Capture Mode Enable For n = 6 */
#define FTM_COMBINE_DECAPEN3(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAPEN3_SHIFT)) & FTM_COMBINE_DECAPEN3_MASK)

#define FTM_COMBINE_DECAP3_MASK                  (0x8000000U)
#define FTM_COMBINE_DECAP3_SHIFT                 (27U)
/*! DECAP3 - Dual Edge Capture Mode Captures For n = 6
 *  0b0..The dual edge captures are inactive.
 *  0b1..The dual edge captures are active.
 */
#define FTM_COMBINE_DECAP3(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DECAP3_SHIFT)) & FTM_COMBINE_DECAP3_MASK)

#define FTM_COMBINE_DTEN3_MASK                   (0x10000000U)
#define FTM_COMBINE_DTEN3_SHIFT                  (28U)
/*! DTEN3 - Deadtime Enable For n = 6
 *  0b0..The deadtime insertion in this pair of channels is disabled.
 *  0b1..The deadtime insertion in this pair of channels is enabled.
 */
#define FTM_COMBINE_DTEN3(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_DTEN3_SHIFT)) & FTM_COMBINE_DTEN3_MASK)

#define FTM_COMBINE_SYNCEN3_MASK                 (0x20000000U)
#define FTM_COMBINE_SYNCEN3_SHIFT                (29U)
/*! SYNCEN3 - Synchronization Enable For n = 6
 *  0b0..The PWM synchronization in this pair of channels is disabled.
 *  0b1..The PWM synchronization in this pair of channels is enabled.
 */
#define FTM_COMBINE_SYNCEN3(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_SYNCEN3_SHIFT)) & FTM_COMBINE_SYNCEN3_MASK)

#define FTM_COMBINE_FAULTEN3_MASK                (0x40000000U)
#define FTM_COMBINE_FAULTEN3_SHIFT               (30U)
/*! FAULTEN3 - Fault Control Enable For n = 6
 *  0b0..The fault control in this pair of channels is disabled.
 *  0b1..The fault control in this pair of channels is enabled.
 */
#define FTM_COMBINE_FAULTEN3(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_FAULTEN3_SHIFT)) & FTM_COMBINE_FAULTEN3_MASK)

#define FTM_COMBINE_MCOMBINE3_MASK               (0x80000000U)
#define FTM_COMBINE_MCOMBINE3_SHIFT              (31U)
/*! MCOMBINE3 - Modified Combine Mode For n = 6 */
#define FTM_COMBINE_MCOMBINE3(x)                 (((uint32_t)(((uint32_t)(x)) << FTM_COMBINE_MCOMBINE3_SHIFT)) & FTM_COMBINE_MCOMBINE3_MASK)
/*! @} */

/*! @name DEADTIME - Deadtime Configuration */
/*! @{ */

#define FTM_DEADTIME_DTVAL_MASK                  (0x3FU)
#define FTM_DEADTIME_DTVAL_SHIFT                 (0U)
/*! DTVAL - Deadtime Value */
#define FTM_DEADTIME_DTVAL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_DEADTIME_DTVAL_SHIFT)) & FTM_DEADTIME_DTVAL_MASK)

#define FTM_DEADTIME_DTPS_MASK                   (0xC0U)
#define FTM_DEADTIME_DTPS_SHIFT                  (6U)
/*! DTPS - Deadtime Prescaler Value
 *  0b0x..Divide the FTM input clock by 1.
 *  0b10..Divide the FTM input clock by 4.
 *  0b11..Divide the FTM input clock by 16.
 */
#define FTM_DEADTIME_DTPS(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_DEADTIME_DTPS_SHIFT)) & FTM_DEADTIME_DTPS_MASK)

#define FTM_DEADTIME_DTVALEX_MASK                (0xF0000U)
#define FTM_DEADTIME_DTVALEX_SHIFT               (16U)
/*! DTVALEX - Extended Deadtime Value */
#define FTM_DEADTIME_DTVALEX(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_DEADTIME_DTVALEX_SHIFT)) & FTM_DEADTIME_DTVALEX_MASK)
/*! @} */

/*! @name EXTTRIG - FTM External Trigger */
/*! @{ */

#define FTM_EXTTRIG_CH2TRIG_MASK                 (0x1U)
#define FTM_EXTTRIG_CH2TRIG_SHIFT                (0U)
/*! CH2TRIG - Channel 2 External Trigger Enable
 *  0b0..The generation of this external trigger is disabled.
 *  0b1..The generation of this external trigger is enabled.
 */
#define FTM_EXTTRIG_CH2TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH2TRIG_SHIFT)) & FTM_EXTTRIG_CH2TRIG_MASK)

#define FTM_EXTTRIG_CH3TRIG_MASK                 (0x2U)
#define FTM_EXTTRIG_CH3TRIG_SHIFT                (1U)
/*! CH3TRIG - Channel 3 External Trigger Enable
 *  0b0..The generation of this external trigger is disabled.
 *  0b1..The generation of this external trigger is enabled.
 */
#define FTM_EXTTRIG_CH3TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH3TRIG_SHIFT)) & FTM_EXTTRIG_CH3TRIG_MASK)

#define FTM_EXTTRIG_CH4TRIG_MASK                 (0x4U)
#define FTM_EXTTRIG_CH4TRIG_SHIFT                (2U)
/*! CH4TRIG - Channel 4 External Trigger Enable
 *  0b0..The generation of this external trigger is disabled.
 *  0b1..The generation of this external trigger is enabled.
 */
#define FTM_EXTTRIG_CH4TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH4TRIG_SHIFT)) & FTM_EXTTRIG_CH4TRIG_MASK)

#define FTM_EXTTRIG_CH5TRIG_MASK                 (0x8U)
#define FTM_EXTTRIG_CH5TRIG_SHIFT                (3U)
/*! CH5TRIG - Channel 5 External Trigger Enable
 *  0b0..The generation of this external trigger is disabled.
 *  0b1..The generation of this external trigger is enabled.
 */
#define FTM_EXTTRIG_CH5TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH5TRIG_SHIFT)) & FTM_EXTTRIG_CH5TRIG_MASK)

#define FTM_EXTTRIG_CH0TRIG_MASK                 (0x10U)
#define FTM_EXTTRIG_CH0TRIG_SHIFT                (4U)
/*! CH0TRIG - Channel 0 External Trigger Enable
 *  0b0..The generation of this external trigger is disabled.
 *  0b1..The generation of this external trigger is enabled.
 */
#define FTM_EXTTRIG_CH0TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH0TRIG_SHIFT)) & FTM_EXTTRIG_CH0TRIG_MASK)

#define FTM_EXTTRIG_CH1TRIG_MASK                 (0x20U)
#define FTM_EXTTRIG_CH1TRIG_SHIFT                (5U)
/*! CH1TRIG - Channel 1 External Trigger Enable
 *  0b0..The generation of this external trigger is disabled.
 *  0b1..The generation of this external trigger is enabled.
 */
#define FTM_EXTTRIG_CH1TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH1TRIG_SHIFT)) & FTM_EXTTRIG_CH1TRIG_MASK)

#define FTM_EXTTRIG_INITTRIGEN_MASK              (0x40U)
#define FTM_EXTTRIG_INITTRIGEN_SHIFT             (6U)
/*! INITTRIGEN - Initialization Trigger Enable
 *  0b0..The generation of initialization trigger is disabled.
 *  0b1..The generation of initialization trigger is enabled.
 */
#define FTM_EXTTRIG_INITTRIGEN(x)                (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_INITTRIGEN_SHIFT)) & FTM_EXTTRIG_INITTRIGEN_MASK)

#define FTM_EXTTRIG_TRIGF_MASK                   (0x80U)
#define FTM_EXTTRIG_TRIGF_SHIFT                  (7U)
/*! TRIGF - Channel Trigger Flag
 *  0b0..No channel trigger was generated.
 *  0b1..A channel trigger was generated.
 */
#define FTM_EXTTRIG_TRIGF(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_TRIGF_SHIFT)) & FTM_EXTTRIG_TRIGF_MASK)

#define FTM_EXTTRIG_CH6TRIG_MASK                 (0x100U)
#define FTM_EXTTRIG_CH6TRIG_SHIFT                (8U)
/*! CH6TRIG - Channel 6 External Trigger Enable
 *  0b0..The generation of this external trigger is disabled.
 *  0b1..The generation of this external trigger is enabled.
 */
#define FTM_EXTTRIG_CH6TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH6TRIG_SHIFT)) & FTM_EXTTRIG_CH6TRIG_MASK)

#define FTM_EXTTRIG_CH7TRIG_MASK                 (0x200U)
#define FTM_EXTTRIG_CH7TRIG_SHIFT                (9U)
/*! CH7TRIG - Channel 7 External Trigger Enable
 *  0b0..The generation of this external trigger is disabled.
 *  0b1..The generation of this external trigger is enabled.
 */
#define FTM_EXTTRIG_CH7TRIG(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_EXTTRIG_CH7TRIG_SHIFT)) & FTM_EXTTRIG_CH7TRIG_MASK)
/*! @} */

/*! @name POL - Channels Polarity */
/*! @{ */

#define FTM_POL_POL0_MASK                        (0x1U)
#define FTM_POL_POL0_SHIFT                       (0U)
/*! POL0 - Channel 0 Polarity
 *  0b0..The channel polarity is active high.
 *  0b1..The channel polarity is active low.
 */
#define FTM_POL_POL0(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL0_SHIFT)) & FTM_POL_POL0_MASK)

#define FTM_POL_POL1_MASK                        (0x2U)
#define FTM_POL_POL1_SHIFT                       (1U)
/*! POL1 - Channel 1 Polarity
 *  0b0..The channel polarity is active high.
 *  0b1..The channel polarity is active low.
 */
#define FTM_POL_POL1(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL1_SHIFT)) & FTM_POL_POL1_MASK)

#define FTM_POL_POL2_MASK                        (0x4U)
#define FTM_POL_POL2_SHIFT                       (2U)
/*! POL2 - Channel 2 Polarity
 *  0b0..The channel polarity is active high.
 *  0b1..The channel polarity is active low.
 */
#define FTM_POL_POL2(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL2_SHIFT)) & FTM_POL_POL2_MASK)

#define FTM_POL_POL3_MASK                        (0x8U)
#define FTM_POL_POL3_SHIFT                       (3U)
/*! POL3 - Channel 3 Polarity
 *  0b0..The channel polarity is active high.
 *  0b1..The channel polarity is active low.
 */
#define FTM_POL_POL3(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL3_SHIFT)) & FTM_POL_POL3_MASK)

#define FTM_POL_POL4_MASK                        (0x10U)
#define FTM_POL_POL4_SHIFT                       (4U)
/*! POL4 - Channel 4 Polarity
 *  0b0..The channel polarity is active high.
 *  0b1..The channel polarity is active low.
 */
#define FTM_POL_POL4(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL4_SHIFT)) & FTM_POL_POL4_MASK)

#define FTM_POL_POL5_MASK                        (0x20U)
#define FTM_POL_POL5_SHIFT                       (5U)
/*! POL5 - Channel 5 Polarity
 *  0b0..The channel polarity is active high.
 *  0b1..The channel polarity is active low.
 */
#define FTM_POL_POL5(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL5_SHIFT)) & FTM_POL_POL5_MASK)

#define FTM_POL_POL6_MASK                        (0x40U)
#define FTM_POL_POL6_SHIFT                       (6U)
/*! POL6 - Channel 6 Polarity
 *  0b0..The channel polarity is active high.
 *  0b1..The channel polarity is active low.
 */
#define FTM_POL_POL6(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL6_SHIFT)) & FTM_POL_POL6_MASK)

#define FTM_POL_POL7_MASK                        (0x80U)
#define FTM_POL_POL7_SHIFT                       (7U)
/*! POL7 - Channel 7 Polarity
 *  0b0..The channel polarity is active high.
 *  0b1..The channel polarity is active low.
 */
#define FTM_POL_POL7(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_POL_POL7_SHIFT)) & FTM_POL_POL7_MASK)
/*! @} */

/*! @name FMS - Fault Mode Status */
/*! @{ */

#define FTM_FMS_FAULTF0_MASK                     (0x1U)
#define FTM_FMS_FAULTF0_SHIFT                    (0U)
/*! FAULTF0 - Fault Detection Flag 0
 *  0b0..No fault condition was detected at the fault input.
 *  0b1..A fault condition was detected at the fault input.
 */
#define FTM_FMS_FAULTF0(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTF0_SHIFT)) & FTM_FMS_FAULTF0_MASK)

#define FTM_FMS_FAULTF1_MASK                     (0x2U)
#define FTM_FMS_FAULTF1_SHIFT                    (1U)
/*! FAULTF1 - Fault Detection Flag 1
 *  0b0..No fault condition was detected at the fault input.
 *  0b1..A fault condition was detected at the fault input.
 */
#define FTM_FMS_FAULTF1(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTF1_SHIFT)) & FTM_FMS_FAULTF1_MASK)

#define FTM_FMS_FAULTF2_MASK                     (0x4U)
#define FTM_FMS_FAULTF2_SHIFT                    (2U)
/*! FAULTF2 - Fault Detection Flag 2
 *  0b0..No fault condition was detected at the fault input.
 *  0b1..A fault condition was detected at the fault input.
 */
#define FTM_FMS_FAULTF2(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTF2_SHIFT)) & FTM_FMS_FAULTF2_MASK)

#define FTM_FMS_FAULTF3_MASK                     (0x8U)
#define FTM_FMS_FAULTF3_SHIFT                    (3U)
/*! FAULTF3 - Fault Detection Flag 3
 *  0b0..No fault condition was detected at the fault input.
 *  0b1..A fault condition was detected at the fault input.
 */
#define FTM_FMS_FAULTF3(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTF3_SHIFT)) & FTM_FMS_FAULTF3_MASK)

#define FTM_FMS_FAULTIN_MASK                     (0x20U)
#define FTM_FMS_FAULTIN_SHIFT                    (5U)
/*! FAULTIN - Fault Inputs
 *  0b0..The logic OR of the enabled fault inputs is 0.
 *  0b1..The logic OR of the enabled fault inputs is 1.
 */
#define FTM_FMS_FAULTIN(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTIN_SHIFT)) & FTM_FMS_FAULTIN_MASK)

#define FTM_FMS_WPEN_MASK                        (0x40U)
#define FTM_FMS_WPEN_SHIFT                       (6U)
/*! WPEN - Write Protection Enable
 *  0b0..Write protection is disabled. Write protected bits can be written.
 *  0b1..Write protection is enabled. Write protected bits cannot be written.
 */
#define FTM_FMS_WPEN(x)                          (((uint32_t)(((uint32_t)(x)) << FTM_FMS_WPEN_SHIFT)) & FTM_FMS_WPEN_MASK)

#define FTM_FMS_FAULTF_MASK                      (0x80U)
#define FTM_FMS_FAULTF_SHIFT                     (7U)
/*! FAULTF - Fault Detection Flag
 *  0b0..No fault condition was detected.
 *  0b1..A fault condition was detected.
 */
#define FTM_FMS_FAULTF(x)                        (((uint32_t)(((uint32_t)(x)) << FTM_FMS_FAULTF_SHIFT)) & FTM_FMS_FAULTF_MASK)
/*! @} */

/*! @name FILTER - Input Capture Filter Control */
/*! @{ */

#define FTM_FILTER_CH0FVAL_MASK                  (0xFU)
#define FTM_FILTER_CH0FVAL_SHIFT                 (0U)
/*! CH0FVAL - Channel 0 Input Filter */
#define FTM_FILTER_CH0FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FILTER_CH0FVAL_SHIFT)) & FTM_FILTER_CH0FVAL_MASK)

#define FTM_FILTER_CH1FVAL_MASK                  (0xF0U)
#define FTM_FILTER_CH1FVAL_SHIFT                 (4U)
/*! CH1FVAL - Channel 1 Input Filter */
#define FTM_FILTER_CH1FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FILTER_CH1FVAL_SHIFT)) & FTM_FILTER_CH1FVAL_MASK)

#define FTM_FILTER_CH2FVAL_MASK                  (0xF00U)
#define FTM_FILTER_CH2FVAL_SHIFT                 (8U)
/*! CH2FVAL - Channel 2 Input Filter */
#define FTM_FILTER_CH2FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FILTER_CH2FVAL_SHIFT)) & FTM_FILTER_CH2FVAL_MASK)

#define FTM_FILTER_CH3FVAL_MASK                  (0xF000U)
#define FTM_FILTER_CH3FVAL_SHIFT                 (12U)
/*! CH3FVAL - Channel 3 Input Filter */
#define FTM_FILTER_CH3FVAL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FILTER_CH3FVAL_SHIFT)) & FTM_FILTER_CH3FVAL_MASK)
/*! @} */

/*! @name FLTCTRL - Fault Control */
/*! @{ */

#define FTM_FLTCTRL_FAULT0EN_MASK                (0x1U)
#define FTM_FLTCTRL_FAULT0EN_SHIFT               (0U)
/*! FAULT0EN - Fault Input 0 Enable
 *  0b0..Fault input is disabled.
 *  0b1..Fault input is enabled.
 */
#define FTM_FLTCTRL_FAULT0EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FAULT0EN_SHIFT)) & FTM_FLTCTRL_FAULT0EN_MASK)

#define FTM_FLTCTRL_FAULT1EN_MASK                (0x2U)
#define FTM_FLTCTRL_FAULT1EN_SHIFT               (1U)
/*! FAULT1EN - Fault Input 1 Enable
 *  0b0..Fault input is disabled.
 *  0b1..Fault input is enabled.
 */
#define FTM_FLTCTRL_FAULT1EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FAULT1EN_SHIFT)) & FTM_FLTCTRL_FAULT1EN_MASK)

#define FTM_FLTCTRL_FAULT2EN_MASK                (0x4U)
#define FTM_FLTCTRL_FAULT2EN_SHIFT               (2U)
/*! FAULT2EN - Fault Input 2 Enable
 *  0b0..Fault input is disabled.
 *  0b1..Fault input is enabled.
 */
#define FTM_FLTCTRL_FAULT2EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FAULT2EN_SHIFT)) & FTM_FLTCTRL_FAULT2EN_MASK)

#define FTM_FLTCTRL_FAULT3EN_MASK                (0x8U)
#define FTM_FLTCTRL_FAULT3EN_SHIFT               (3U)
/*! FAULT3EN - Fault Input 3 Enable
 *  0b0..Fault input is disabled.
 *  0b1..Fault input is enabled.
 */
#define FTM_FLTCTRL_FAULT3EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FAULT3EN_SHIFT)) & FTM_FLTCTRL_FAULT3EN_MASK)

#define FTM_FLTCTRL_FFLTR0EN_MASK                (0x10U)
#define FTM_FLTCTRL_FFLTR0EN_SHIFT               (4U)
/*! FFLTR0EN - Fault Input 0 Filter Enable
 *  0b0..Fault input filter is disabled.
 *  0b1..Fault input filter is enabled.
 */
#define FTM_FLTCTRL_FFLTR0EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FFLTR0EN_SHIFT)) & FTM_FLTCTRL_FFLTR0EN_MASK)

#define FTM_FLTCTRL_FFLTR1EN_MASK                (0x20U)
#define FTM_FLTCTRL_FFLTR1EN_SHIFT               (5U)
/*! FFLTR1EN - Fault Input 1 Filter Enable
 *  0b0..Fault input filter is disabled.
 *  0b1..Fault input filter is enabled.
 */
#define FTM_FLTCTRL_FFLTR1EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FFLTR1EN_SHIFT)) & FTM_FLTCTRL_FFLTR1EN_MASK)

#define FTM_FLTCTRL_FFLTR2EN_MASK                (0x40U)
#define FTM_FLTCTRL_FFLTR2EN_SHIFT               (6U)
/*! FFLTR2EN - Fault Input 2 Filter Enable
 *  0b0..Fault input filter is disabled.
 *  0b1..Fault input filter is enabled.
 */
#define FTM_FLTCTRL_FFLTR2EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FFLTR2EN_SHIFT)) & FTM_FLTCTRL_FFLTR2EN_MASK)

#define FTM_FLTCTRL_FFLTR3EN_MASK                (0x80U)
#define FTM_FLTCTRL_FFLTR3EN_SHIFT               (7U)
/*! FFLTR3EN - Fault Input 3 Filter Enable
 *  0b0..Fault input filter is disabled.
 *  0b1..Fault input filter is enabled.
 */
#define FTM_FLTCTRL_FFLTR3EN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FFLTR3EN_SHIFT)) & FTM_FLTCTRL_FFLTR3EN_MASK)

#define FTM_FLTCTRL_FFVAL_MASK                   (0xF00U)
#define FTM_FLTCTRL_FFVAL_SHIFT                  (8U)
/*! FFVAL - Fault Input Filter */
#define FTM_FLTCTRL_FFVAL(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FFVAL_SHIFT)) & FTM_FLTCTRL_FFVAL_MASK)

#define FTM_FLTCTRL_FSTATE_MASK                  (0x8000U)
#define FTM_FLTCTRL_FSTATE_SHIFT                 (15U)
/*! FSTATE - Fault output state
 *  0b0..FTM outputs will be placed into safe values when fault events in ongoing (defined by POL bits).
 *  0b1..FTM outputs will be tri-stated when fault event is ongoing
 */
#define FTM_FLTCTRL_FSTATE(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FLTCTRL_FSTATE_SHIFT)) & FTM_FLTCTRL_FSTATE_MASK)
/*! @} */

/*! @name QDCTRL - Quadrature Decoder Control And Status */
/*! @{ */

#define FTM_QDCTRL_QUADEN_MASK                   (0x1U)
#define FTM_QDCTRL_QUADEN_SHIFT                  (0U)
/*! QUADEN - Quadrature Decoder Mode Enable
 *  0b0..Quadrature Decoder mode is disabled.
 *  0b1..Quadrature Decoder mode is enabled.
 */
#define FTM_QDCTRL_QUADEN(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_QUADEN_SHIFT)) & FTM_QDCTRL_QUADEN_MASK)

#define FTM_QDCTRL_TOFDIR_MASK                   (0x2U)
#define FTM_QDCTRL_TOFDIR_SHIFT                  (1U)
/*! TOFDIR - Timer Overflow Direction In Quadrature Decoder Mode
 *  0b0..TOF bit was set on the bottom of counting. There was an FTM counter decrement and FTM counter changes
 *       from its minimum value (CNTIN register) to its maximum value (MOD register).
 *  0b1..TOF bit was set on the top of counting. There was an FTM counter increment and FTM counter changes from
 *       its maximum value (MOD register) to its minimum value (CNTIN register).
 */
#define FTM_QDCTRL_TOFDIR(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_TOFDIR_SHIFT)) & FTM_QDCTRL_TOFDIR_MASK)

#define FTM_QDCTRL_QUADIR_MASK                   (0x4U)
#define FTM_QDCTRL_QUADIR_SHIFT                  (2U)
/*! QUADIR - FTM Counter Direction In Quadrature Decoder Mode
 *  0b0..Counting direction is decreasing (FTM counter decrement).
 *  0b1..Counting direction is increasing (FTM counter increment).
 */
#define FTM_QDCTRL_QUADIR(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_QUADIR_SHIFT)) & FTM_QDCTRL_QUADIR_MASK)

#define FTM_QDCTRL_QUADMODE_MASK                 (0x8U)
#define FTM_QDCTRL_QUADMODE_SHIFT                (3U)
/*! QUADMODE - Quadrature Decoder Mode
 *  0b0..Phase A and phase B encoding mode.
 *  0b1..Count and direction encoding mode.
 */
#define FTM_QDCTRL_QUADMODE(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_QUADMODE_SHIFT)) & FTM_QDCTRL_QUADMODE_MASK)

#define FTM_QDCTRL_PHBPOL_MASK                   (0x10U)
#define FTM_QDCTRL_PHBPOL_SHIFT                  (4U)
/*! PHBPOL - Phase B Input Polarity
 *  0b0..Normal polarity. Phase B input signal is not inverted before identifying the rising and falling edges of this signal.
 *  0b1..Inverted polarity. Phase B input signal is inverted before identifying the rising and falling edges of this signal.
 */
#define FTM_QDCTRL_PHBPOL(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_PHBPOL_SHIFT)) & FTM_QDCTRL_PHBPOL_MASK)

#define FTM_QDCTRL_PHAPOL_MASK                   (0x20U)
#define FTM_QDCTRL_PHAPOL_SHIFT                  (5U)
/*! PHAPOL - Phase A Input Polarity
 *  0b0..Normal polarity. Phase A input signal is not inverted before identifying the rising and falling edges of this signal.
 *  0b1..Inverted polarity. Phase A input signal is inverted before identifying the rising and falling edges of this signal.
 */
#define FTM_QDCTRL_PHAPOL(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_PHAPOL_SHIFT)) & FTM_QDCTRL_PHAPOL_MASK)

#define FTM_QDCTRL_PHBFLTREN_MASK                (0x40U)
#define FTM_QDCTRL_PHBFLTREN_SHIFT               (6U)
/*! PHBFLTREN - Phase B Input Filter Enable
 *  0b0..Phase B input filter is disabled.
 *  0b1..Phase B input filter is enabled.
 */
#define FTM_QDCTRL_PHBFLTREN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_PHBFLTREN_SHIFT)) & FTM_QDCTRL_PHBFLTREN_MASK)

#define FTM_QDCTRL_PHAFLTREN_MASK                (0x80U)
#define FTM_QDCTRL_PHAFLTREN_SHIFT               (7U)
/*! PHAFLTREN - Phase A Input Filter Enable
 *  0b0..Phase A input filter is disabled.
 *  0b1..Phase A input filter is enabled.
 */
#define FTM_QDCTRL_PHAFLTREN(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_QDCTRL_PHAFLTREN_SHIFT)) & FTM_QDCTRL_PHAFLTREN_MASK)
/*! @} */

/*! @name CONF - Configuration */
/*! @{ */

#define FTM_CONF_LDFQ_MASK                       (0x1FU)
#define FTM_CONF_LDFQ_SHIFT                      (0U)
/*! LDFQ - Frequency of the Reload Opportunities */
#define FTM_CONF_LDFQ(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_CONF_LDFQ_SHIFT)) & FTM_CONF_LDFQ_MASK)

#define FTM_CONF_BDMMODE_MASK                    (0xC0U)
#define FTM_CONF_BDMMODE_SHIFT                   (6U)
/*! BDMMODE - Debug Mode */
#define FTM_CONF_BDMMODE(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_CONF_BDMMODE_SHIFT)) & FTM_CONF_BDMMODE_MASK)

#define FTM_CONF_GTBEEN_MASK                     (0x200U)
#define FTM_CONF_GTBEEN_SHIFT                    (9U)
/*! GTBEEN - Global Time Base Enable
 *  0b0..Use of an external global time base is disabled.
 *  0b1..Use of an external global time base is enabled.
 */
#define FTM_CONF_GTBEEN(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_CONF_GTBEEN_SHIFT)) & FTM_CONF_GTBEEN_MASK)

#define FTM_CONF_GTBEOUT_MASK                    (0x400U)
#define FTM_CONF_GTBEOUT_SHIFT                   (10U)
/*! GTBEOUT - Global Time Base Output
 *  0b0..A global time base signal generation is disabled.
 *  0b1..A global time base signal generation is enabled.
 */
#define FTM_CONF_GTBEOUT(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_CONF_GTBEOUT_SHIFT)) & FTM_CONF_GTBEOUT_MASK)

#define FTM_CONF_ITRIGR_MASK                     (0x800U)
#define FTM_CONF_ITRIGR_SHIFT                    (11U)
/*! ITRIGR - Initialization trigger on Reload Point
 *  0b0..Initialization trigger is generated on counter wrap events.
 *  0b1..Initialization trigger is generated when a reload point is reached.
 */
#define FTM_CONF_ITRIGR(x)                       (((uint32_t)(((uint32_t)(x)) << FTM_CONF_ITRIGR_SHIFT)) & FTM_CONF_ITRIGR_MASK)
/*! @} */

/*! @name FLTPOL - FTM Fault Input Polarity */
/*! @{ */

#define FTM_FLTPOL_FLT0POL_MASK                  (0x1U)
#define FTM_FLTPOL_FLT0POL_SHIFT                 (0U)
/*! FLT0POL - Fault Input 0 Polarity
 *  0b0..The fault input polarity is active high. A 1 at the fault input indicates a fault.
 *  0b1..The fault input polarity is active low. A 0 at the fault input indicates a fault.
 */
#define FTM_FLTPOL_FLT0POL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FLTPOL_FLT0POL_SHIFT)) & FTM_FLTPOL_FLT0POL_MASK)

#define FTM_FLTPOL_FLT1POL_MASK                  (0x2U)
#define FTM_FLTPOL_FLT1POL_SHIFT                 (1U)
/*! FLT1POL - Fault Input 1 Polarity
 *  0b0..The fault input polarity is active high. A 1 at the fault input indicates a fault.
 *  0b1..The fault input polarity is active low. A 0 at the fault input indicates a fault.
 */
#define FTM_FLTPOL_FLT1POL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FLTPOL_FLT1POL_SHIFT)) & FTM_FLTPOL_FLT1POL_MASK)

#define FTM_FLTPOL_FLT2POL_MASK                  (0x4U)
#define FTM_FLTPOL_FLT2POL_SHIFT                 (2U)
/*! FLT2POL - Fault Input 2 Polarity
 *  0b0..The fault input polarity is active high. A 1 at the fault input indicates a fault.
 *  0b1..The fault input polarity is active low. A 0 at the fault input indicates a fault.
 */
#define FTM_FLTPOL_FLT2POL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FLTPOL_FLT2POL_SHIFT)) & FTM_FLTPOL_FLT2POL_MASK)

#define FTM_FLTPOL_FLT3POL_MASK                  (0x8U)
#define FTM_FLTPOL_FLT3POL_SHIFT                 (3U)
/*! FLT3POL - Fault Input 3 Polarity
 *  0b0..The fault input polarity is active high. A 1 at the fault input indicates a fault.
 *  0b1..The fault input polarity is active low. A 0 at the fault input indicates a fault.
 */
#define FTM_FLTPOL_FLT3POL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_FLTPOL_FLT3POL_SHIFT)) & FTM_FLTPOL_FLT3POL_MASK)
/*! @} */

/*! @name SYNCONF - Synchronization Configuration */
/*! @{ */

#define FTM_SYNCONF_HWTRIGMODE_MASK              (0x1U)
#define FTM_SYNCONF_HWTRIGMODE_SHIFT             (0U)
/*! HWTRIGMODE - Hardware Trigger Mode
 *  0b0..FTM clears the TRIGj bit when the hardware trigger j is detected, where j = 0, 1,2.
 *  0b1..FTM does not clear the TRIGj bit when the hardware trigger j is detected, where j = 0, 1,2.
 */
#define FTM_SYNCONF_HWTRIGMODE(x)                (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWTRIGMODE_SHIFT)) & FTM_SYNCONF_HWTRIGMODE_MASK)

#define FTM_SYNCONF_CNTINC_MASK                  (0x4U)
#define FTM_SYNCONF_CNTINC_SHIFT                 (2U)
/*! CNTINC - CNTIN Register Synchronization
 *  0b0..CNTIN register is updated with its buffer value at all rising edges of FTM input clock.
 *  0b1..CNTIN register is updated with its buffer value by the PWM synchronization.
 */
#define FTM_SYNCONF_CNTINC(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_CNTINC_SHIFT)) & FTM_SYNCONF_CNTINC_MASK)

#define FTM_SYNCONF_INVC_MASK                    (0x10U)
#define FTM_SYNCONF_INVC_SHIFT                   (4U)
/*! INVC - INVCTRL Register Synchronization
 *  0b0..INVCTRL register is updated with its buffer value at all rising edges of FTM input clock.
 *  0b1..INVCTRL register is updated with its buffer value by the PWM synchronization.
 */
#define FTM_SYNCONF_INVC(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_INVC_SHIFT)) & FTM_SYNCONF_INVC_MASK)

#define FTM_SYNCONF_SWOC_MASK                    (0x20U)
#define FTM_SYNCONF_SWOC_SHIFT                   (5U)
/*! SWOC - SWOCTRL Register Synchronization
 *  0b0..SWOCTRL register is updated with its buffer value at all rising edges of FTM input clock.
 *  0b1..SWOCTRL register is updated with its buffer value by the PWM synchronization.
 */
#define FTM_SYNCONF_SWOC(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWOC_SHIFT)) & FTM_SYNCONF_SWOC_MASK)

#define FTM_SYNCONF_SYNCMODE_MASK                (0x80U)
#define FTM_SYNCONF_SYNCMODE_SHIFT               (7U)
/*! SYNCMODE - Synchronization Mode
 *  0b0..Legacy PWM synchronization is selected.
 *  0b1..Enhanced PWM synchronization is selected.
 */
#define FTM_SYNCONF_SYNCMODE(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SYNCMODE_SHIFT)) & FTM_SYNCONF_SYNCMODE_MASK)

#define FTM_SYNCONF_SWRSTCNT_MASK                (0x100U)
#define FTM_SYNCONF_SWRSTCNT_SHIFT               (8U)
/*! SWRSTCNT - FTM counter synchronization is activated by the software trigger
 *  0b0..The software trigger does not activate the FTM counter synchronization.
 *  0b1..The software trigger activates the FTM counter synchronization.
 */
#define FTM_SYNCONF_SWRSTCNT(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWRSTCNT_SHIFT)) & FTM_SYNCONF_SWRSTCNT_MASK)

#define FTM_SYNCONF_SWWRBUF_MASK                 (0x200U)
#define FTM_SYNCONF_SWWRBUF_SHIFT                (9U)
/*! SWWRBUF - MOD, HCR, CNTIN, and CV registers synchronization is activated by the software trigger
 *  0b0..The software trigger does not activate MOD, HCR, CNTIN, and CV registers synchronization.
 *  0b1..The software trigger activates MOD, HCR, CNTIN, and CV registers synchronization.
 */
#define FTM_SYNCONF_SWWRBUF(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWWRBUF_SHIFT)) & FTM_SYNCONF_SWWRBUF_MASK)

#define FTM_SYNCONF_SWOM_MASK                    (0x400U)
#define FTM_SYNCONF_SWOM_SHIFT                   (10U)
/*! SWOM - Output mask synchronization is activated by the software trigger
 *  0b0..The software trigger does not activate the OUTMASK register synchronization.
 *  0b1..The software trigger activates the OUTMASK register synchronization.
 */
#define FTM_SYNCONF_SWOM(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWOM_SHIFT)) & FTM_SYNCONF_SWOM_MASK)

#define FTM_SYNCONF_SWINVC_MASK                  (0x800U)
#define FTM_SYNCONF_SWINVC_SHIFT                 (11U)
/*! SWINVC - Inverting control synchronization is activated by the software trigger
 *  0b0..The software trigger does not activate the INVCTRL register synchronization.
 *  0b1..The software trigger activates the INVCTRL register synchronization.
 */
#define FTM_SYNCONF_SWINVC(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWINVC_SHIFT)) & FTM_SYNCONF_SWINVC_MASK)

#define FTM_SYNCONF_SWSOC_MASK                   (0x1000U)
#define FTM_SYNCONF_SWSOC_SHIFT                  (12U)
/*! SWSOC - Software output control synchronization is activated by the software trigger
 *  0b0..The software trigger does not activate the SWOCTRL register synchronization.
 *  0b1..The software trigger activates the SWOCTRL register synchronization.
 */
#define FTM_SYNCONF_SWSOC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_SWSOC_SHIFT)) & FTM_SYNCONF_SWSOC_MASK)

#define FTM_SYNCONF_HWRSTCNT_MASK                (0x10000U)
#define FTM_SYNCONF_HWRSTCNT_SHIFT               (16U)
/*! HWRSTCNT - FTM counter synchronization is activated by a hardware trigger
 *  0b0..A hardware trigger does not activate the FTM counter synchronization.
 *  0b1..A hardware trigger activates the FTM counter synchronization.
 */
#define FTM_SYNCONF_HWRSTCNT(x)                  (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWRSTCNT_SHIFT)) & FTM_SYNCONF_HWRSTCNT_MASK)

#define FTM_SYNCONF_HWWRBUF_MASK                 (0x20000U)
#define FTM_SYNCONF_HWWRBUF_SHIFT                (17U)
/*! HWWRBUF - MOD, HCR, CNTIN, and CV registers synchronization is activated by a hardware trigger
 *  0b0..A hardware trigger does not activate MOD, HCR, CNTIN, and CV registers synchronization.
 *  0b1..A hardware trigger activates MOD, HCR, CNTIN, and CV registers synchronization.
 */
#define FTM_SYNCONF_HWWRBUF(x)                   (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWWRBUF_SHIFT)) & FTM_SYNCONF_HWWRBUF_MASK)

#define FTM_SYNCONF_HWOM_MASK                    (0x40000U)
#define FTM_SYNCONF_HWOM_SHIFT                   (18U)
/*! HWOM - Output mask synchronization is activated by a hardware trigger
 *  0b0..A hardware trigger does not activate the OUTMASK register synchronization.
 *  0b1..A hardware trigger activates the OUTMASK register synchronization.
 */
#define FTM_SYNCONF_HWOM(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWOM_SHIFT)) & FTM_SYNCONF_HWOM_MASK)

#define FTM_SYNCONF_HWINVC_MASK                  (0x80000U)
#define FTM_SYNCONF_HWINVC_SHIFT                 (19U)
/*! HWINVC - Inverting control synchronization is activated by a hardware trigger
 *  0b0..A hardware trigger does not activate the INVCTRL register synchronization.
 *  0b1..A hardware trigger activates the INVCTRL register synchronization.
 */
#define FTM_SYNCONF_HWINVC(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWINVC_SHIFT)) & FTM_SYNCONF_HWINVC_MASK)

#define FTM_SYNCONF_HWSOC_MASK                   (0x100000U)
#define FTM_SYNCONF_HWSOC_SHIFT                  (20U)
/*! HWSOC - Software output control synchronization is activated by a hardware trigger
 *  0b0..A hardware trigger does not activate the SWOCTRL register synchronization.
 *  0b1..A hardware trigger activates the SWOCTRL register synchronization.
 */
#define FTM_SYNCONF_HWSOC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SYNCONF_HWSOC_SHIFT)) & FTM_SYNCONF_HWSOC_MASK)
/*! @} */

/*! @name INVCTRL - FTM Inverting Control */
/*! @{ */

#define FTM_INVCTRL_INV0EN_MASK                  (0x1U)
#define FTM_INVCTRL_INV0EN_SHIFT                 (0U)
/*! INV0EN - Pair Channels 0 Inverting Enable
 *  0b0..Inverting is disabled.
 *  0b1..Inverting is enabled.
 */
#define FTM_INVCTRL_INV0EN(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_INVCTRL_INV0EN_SHIFT)) & FTM_INVCTRL_INV0EN_MASK)

#define FTM_INVCTRL_INV1EN_MASK                  (0x2U)
#define FTM_INVCTRL_INV1EN_SHIFT                 (1U)
/*! INV1EN - Pair Channels 1 Inverting Enable
 *  0b0..Inverting is disabled.
 *  0b1..Inverting is enabled.
 */
#define FTM_INVCTRL_INV1EN(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_INVCTRL_INV1EN_SHIFT)) & FTM_INVCTRL_INV1EN_MASK)

#define FTM_INVCTRL_INV2EN_MASK                  (0x4U)
#define FTM_INVCTRL_INV2EN_SHIFT                 (2U)
/*! INV2EN - Pair Channels 2 Inverting Enable
 *  0b0..Inverting is disabled.
 *  0b1..Inverting is enabled.
 */
#define FTM_INVCTRL_INV2EN(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_INVCTRL_INV2EN_SHIFT)) & FTM_INVCTRL_INV2EN_MASK)

#define FTM_INVCTRL_INV3EN_MASK                  (0x8U)
#define FTM_INVCTRL_INV3EN_SHIFT                 (3U)
/*! INV3EN - Pair Channels 3 Inverting Enable
 *  0b0..Inverting is disabled.
 *  0b1..Inverting is enabled.
 */
#define FTM_INVCTRL_INV3EN(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_INVCTRL_INV3EN_SHIFT)) & FTM_INVCTRL_INV3EN_MASK)
/*! @} */

/*! @name SWOCTRL - FTM Software Output Control */
/*! @{ */

#define FTM_SWOCTRL_CH0OC_MASK                   (0x1U)
#define FTM_SWOCTRL_CH0OC_SHIFT                  (0U)
/*! CH0OC - Channel 0 Software Output Control Enable
 *  0b0..The channel output is not affected by software output control.
 *  0b1..The channel output is affected by software output control.
 */
#define FTM_SWOCTRL_CH0OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH0OC_SHIFT)) & FTM_SWOCTRL_CH0OC_MASK)

#define FTM_SWOCTRL_CH1OC_MASK                   (0x2U)
#define FTM_SWOCTRL_CH1OC_SHIFT                  (1U)
/*! CH1OC - Channel 1 Software Output Control Enable
 *  0b0..The channel output is not affected by software output control.
 *  0b1..The channel output is affected by software output control.
 */
#define FTM_SWOCTRL_CH1OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH1OC_SHIFT)) & FTM_SWOCTRL_CH1OC_MASK)

#define FTM_SWOCTRL_CH2OC_MASK                   (0x4U)
#define FTM_SWOCTRL_CH2OC_SHIFT                  (2U)
/*! CH2OC - Channel 2 Software Output Control Enable
 *  0b0..The channel output is not affected by software output control.
 *  0b1..The channel output is affected by software output control.
 */
#define FTM_SWOCTRL_CH2OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH2OC_SHIFT)) & FTM_SWOCTRL_CH2OC_MASK)

#define FTM_SWOCTRL_CH3OC_MASK                   (0x8U)
#define FTM_SWOCTRL_CH3OC_SHIFT                  (3U)
/*! CH3OC - Channel 3 Software Output Control Enable
 *  0b0..The channel output is not affected by software output control.
 *  0b1..The channel output is affected by software output control.
 */
#define FTM_SWOCTRL_CH3OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH3OC_SHIFT)) & FTM_SWOCTRL_CH3OC_MASK)

#define FTM_SWOCTRL_CH4OC_MASK                   (0x10U)
#define FTM_SWOCTRL_CH4OC_SHIFT                  (4U)
/*! CH4OC - Channel 4 Software Output Control Enable
 *  0b0..The channel output is not affected by software output control.
 *  0b1..The channel output is affected by software output control.
 */
#define FTM_SWOCTRL_CH4OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH4OC_SHIFT)) & FTM_SWOCTRL_CH4OC_MASK)

#define FTM_SWOCTRL_CH5OC_MASK                   (0x20U)
#define FTM_SWOCTRL_CH5OC_SHIFT                  (5U)
/*! CH5OC - Channel 5 Software Output Control Enable
 *  0b0..The channel output is not affected by software output control.
 *  0b1..The channel output is affected by software output control.
 */
#define FTM_SWOCTRL_CH5OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH5OC_SHIFT)) & FTM_SWOCTRL_CH5OC_MASK)

#define FTM_SWOCTRL_CH6OC_MASK                   (0x40U)
#define FTM_SWOCTRL_CH6OC_SHIFT                  (6U)
/*! CH6OC - Channel 6 Software Output Control Enable
 *  0b0..The channel output is not affected by software output control.
 *  0b1..The channel output is affected by software output control.
 */
#define FTM_SWOCTRL_CH6OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH6OC_SHIFT)) & FTM_SWOCTRL_CH6OC_MASK)

#define FTM_SWOCTRL_CH7OC_MASK                   (0x80U)
#define FTM_SWOCTRL_CH7OC_SHIFT                  (7U)
/*! CH7OC - Channel 7 Software Output Control Enable
 *  0b0..The channel output is not affected by software output control.
 *  0b1..The channel output is affected by software output control.
 */
#define FTM_SWOCTRL_CH7OC(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH7OC_SHIFT)) & FTM_SWOCTRL_CH7OC_MASK)

#define FTM_SWOCTRL_CH0OCV_MASK                  (0x100U)
#define FTM_SWOCTRL_CH0OCV_SHIFT                 (8U)
/*! CH0OCV - Channel 0 Software Output Control Value
 *  0b0..The software output control forces 0 to the channel output.
 *  0b1..The software output control forces 1 to the channel output.
 */
#define FTM_SWOCTRL_CH0OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH0OCV_SHIFT)) & FTM_SWOCTRL_CH0OCV_MASK)

#define FTM_SWOCTRL_CH1OCV_MASK                  (0x200U)
#define FTM_SWOCTRL_CH1OCV_SHIFT                 (9U)
/*! CH1OCV - Channel 1 Software Output Control Value
 *  0b0..The software output control forces 0 to the channel output.
 *  0b1..The software output control forces 1 to the channel output.
 */
#define FTM_SWOCTRL_CH1OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH1OCV_SHIFT)) & FTM_SWOCTRL_CH1OCV_MASK)

#define FTM_SWOCTRL_CH2OCV_MASK                  (0x400U)
#define FTM_SWOCTRL_CH2OCV_SHIFT                 (10U)
/*! CH2OCV - Channel 2 Software Output Control Value
 *  0b0..The software output control forces 0 to the channel output.
 *  0b1..The software output control forces 1 to the channel output.
 */
#define FTM_SWOCTRL_CH2OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH2OCV_SHIFT)) & FTM_SWOCTRL_CH2OCV_MASK)

#define FTM_SWOCTRL_CH3OCV_MASK                  (0x800U)
#define FTM_SWOCTRL_CH3OCV_SHIFT                 (11U)
/*! CH3OCV - Channel 3 Software Output Control Value
 *  0b0..The software output control forces 0 to the channel output.
 *  0b1..The software output control forces 1 to the channel output.
 */
#define FTM_SWOCTRL_CH3OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH3OCV_SHIFT)) & FTM_SWOCTRL_CH3OCV_MASK)

#define FTM_SWOCTRL_CH4OCV_MASK                  (0x1000U)
#define FTM_SWOCTRL_CH4OCV_SHIFT                 (12U)
/*! CH4OCV - Channel 4 Software Output Control Value
 *  0b0..The software output control forces 0 to the channel output.
 *  0b1..The software output control forces 1 to the channel output.
 */
#define FTM_SWOCTRL_CH4OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH4OCV_SHIFT)) & FTM_SWOCTRL_CH4OCV_MASK)

#define FTM_SWOCTRL_CH5OCV_MASK                  (0x2000U)
#define FTM_SWOCTRL_CH5OCV_SHIFT                 (13U)
/*! CH5OCV - Channel 5 Software Output Control Value
 *  0b0..The software output control forces 0 to the channel output.
 *  0b1..The software output control forces 1 to the channel output.
 */
#define FTM_SWOCTRL_CH5OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH5OCV_SHIFT)) & FTM_SWOCTRL_CH5OCV_MASK)

#define FTM_SWOCTRL_CH6OCV_MASK                  (0x4000U)
#define FTM_SWOCTRL_CH6OCV_SHIFT                 (14U)
/*! CH6OCV - Channel 6 Software Output Control Value
 *  0b0..The software output control forces 0 to the channel output.
 *  0b1..The software output control forces 1 to the channel output.
 */
#define FTM_SWOCTRL_CH6OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH6OCV_SHIFT)) & FTM_SWOCTRL_CH6OCV_MASK)

#define FTM_SWOCTRL_CH7OCV_MASK                  (0x8000U)
#define FTM_SWOCTRL_CH7OCV_SHIFT                 (15U)
/*! CH7OCV - Channel 7 Software Output Control Value
 *  0b0..The software output control forces 0 to the channel output.
 *  0b1..The software output control forces 1 to the channel output.
 */
#define FTM_SWOCTRL_CH7OCV(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_SWOCTRL_CH7OCV_SHIFT)) & FTM_SWOCTRL_CH7OCV_MASK)
/*! @} */

/*! @name PWMLOAD - FTM PWM Load */
/*! @{ */

#define FTM_PWMLOAD_CH0SEL_MASK                  (0x1U)
#define FTM_PWMLOAD_CH0SEL_SHIFT                 (0U)
/*! CH0SEL - Channel 0 Select
 *  0b0..Channel match is not included as a reload opportunity.
 *  0b1..Channel match is included as a reload opportunity.
 */
#define FTM_PWMLOAD_CH0SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH0SEL_SHIFT)) & FTM_PWMLOAD_CH0SEL_MASK)

#define FTM_PWMLOAD_CH1SEL_MASK                  (0x2U)
#define FTM_PWMLOAD_CH1SEL_SHIFT                 (1U)
/*! CH1SEL - Channel 1 Select
 *  0b0..Channel match is not included as a reload opportunity.
 *  0b1..Channel match is included as a reload opportunity.
 */
#define FTM_PWMLOAD_CH1SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH1SEL_SHIFT)) & FTM_PWMLOAD_CH1SEL_MASK)

#define FTM_PWMLOAD_CH2SEL_MASK                  (0x4U)
#define FTM_PWMLOAD_CH2SEL_SHIFT                 (2U)
/*! CH2SEL - Channel 2 Select
 *  0b0..Channel match is not included as a reload opportunity.
 *  0b1..Channel match is included as a reload opportunity.
 */
#define FTM_PWMLOAD_CH2SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH2SEL_SHIFT)) & FTM_PWMLOAD_CH2SEL_MASK)

#define FTM_PWMLOAD_CH3SEL_MASK                  (0x8U)
#define FTM_PWMLOAD_CH3SEL_SHIFT                 (3U)
/*! CH3SEL - Channel 3 Select
 *  0b0..Channel match is not included as a reload opportunity.
 *  0b1..Channel match is included as a reload opportunity.
 */
#define FTM_PWMLOAD_CH3SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH3SEL_SHIFT)) & FTM_PWMLOAD_CH3SEL_MASK)

#define FTM_PWMLOAD_CH4SEL_MASK                  (0x10U)
#define FTM_PWMLOAD_CH4SEL_SHIFT                 (4U)
/*! CH4SEL - Channel 4 Select
 *  0b0..Channel match is not included as a reload opportunity.
 *  0b1..Channel match is included as a reload opportunity.
 */
#define FTM_PWMLOAD_CH4SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH4SEL_SHIFT)) & FTM_PWMLOAD_CH4SEL_MASK)

#define FTM_PWMLOAD_CH5SEL_MASK                  (0x20U)
#define FTM_PWMLOAD_CH5SEL_SHIFT                 (5U)
/*! CH5SEL - Channel 5 Select
 *  0b0..Channel match is not included as a reload opportunity.
 *  0b1..Channel match is included as a reload opportunity.
 */
#define FTM_PWMLOAD_CH5SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH5SEL_SHIFT)) & FTM_PWMLOAD_CH5SEL_MASK)

#define FTM_PWMLOAD_CH6SEL_MASK                  (0x40U)
#define FTM_PWMLOAD_CH6SEL_SHIFT                 (6U)
/*! CH6SEL - Channel 6 Select
 *  0b0..Channel match is not included as a reload opportunity.
 *  0b1..Channel match is included as a reload opportunity.
 */
#define FTM_PWMLOAD_CH6SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH6SEL_SHIFT)) & FTM_PWMLOAD_CH6SEL_MASK)

#define FTM_PWMLOAD_CH7SEL_MASK                  (0x80U)
#define FTM_PWMLOAD_CH7SEL_SHIFT                 (7U)
/*! CH7SEL - Channel 7 Select
 *  0b0..Channel match is not included as a reload opportunity.
 *  0b1..Channel match is included as a reload opportunity.
 */
#define FTM_PWMLOAD_CH7SEL(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_CH7SEL_SHIFT)) & FTM_PWMLOAD_CH7SEL_MASK)

#define FTM_PWMLOAD_HCSEL_MASK                   (0x100U)
#define FTM_PWMLOAD_HCSEL_SHIFT                  (8U)
/*! HCSEL - Half Cycle Select
 *  0b0..Half cycle reload is disabled and it is not considered as a reload opportunity.
 *  0b1..Half cycle reload is enabled and it is considered as a reload opportunity.
 */
#define FTM_PWMLOAD_HCSEL(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_HCSEL_SHIFT)) & FTM_PWMLOAD_HCSEL_MASK)

#define FTM_PWMLOAD_LDOK_MASK                    (0x200U)
#define FTM_PWMLOAD_LDOK_SHIFT                   (9U)
/*! LDOK - Load Enable
 *  0b0..Loading updated values is disabled.
 *  0b1..Loading updated values is enabled.
 */
#define FTM_PWMLOAD_LDOK(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_LDOK_SHIFT)) & FTM_PWMLOAD_LDOK_MASK)

#define FTM_PWMLOAD_GLEN_MASK                    (0x400U)
#define FTM_PWMLOAD_GLEN_SHIFT                   (10U)
/*! GLEN - Global Load Enable
 *  0b0..Global Load Ok disabled.
 *  0b1..Global Load OK enabled. A pulse event on the module global load input sets the LDOK bit.
 */
#define FTM_PWMLOAD_GLEN(x)                      (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_GLEN_SHIFT)) & FTM_PWMLOAD_GLEN_MASK)

#define FTM_PWMLOAD_GLDOK_MASK                   (0x800U)
#define FTM_PWMLOAD_GLDOK_SHIFT                  (11U)
/*! GLDOK - Global Load OK
 *  0b0..No action.
 *  0b1..LDOK bit is set.
 */
#define FTM_PWMLOAD_GLDOK(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_PWMLOAD_GLDOK_SHIFT)) & FTM_PWMLOAD_GLDOK_MASK)
/*! @} */

/*! @name HCR - Half Cycle Register */
/*! @{ */

#define FTM_HCR_HCVAL_MASK                       (0xFFFFU)
#define FTM_HCR_HCVAL_SHIFT                      (0U)
/*! HCVAL - Half Cycle Value */
#define FTM_HCR_HCVAL(x)                         (((uint32_t)(((uint32_t)(x)) << FTM_HCR_HCVAL_SHIFT)) & FTM_HCR_HCVAL_MASK)
/*! @} */

/*! @name PAIR0DEADTIME - Pair 0 Deadtime Configuration */
/*! @{ */

#define FTM_PAIR0DEADTIME_DTVAL_MASK             (0x3FU)
#define FTM_PAIR0DEADTIME_DTVAL_SHIFT            (0U)
/*! DTVAL - Deadtime Value */
#define FTM_PAIR0DEADTIME_DTVAL(x)               (((uint32_t)(((uint32_t)(x)) << FTM_PAIR0DEADTIME_DTVAL_SHIFT)) & FTM_PAIR0DEADTIME_DTVAL_MASK)

#define FTM_PAIR0DEADTIME_DTPS_MASK              (0xC0U)
#define FTM_PAIR0DEADTIME_DTPS_SHIFT             (6U)
/*! DTPS - Deadtime Prescaler Value
 *  0b0x..Divide the FTM input clock by 1.
 *  0b10..Divide the FTM input clock by 4.
 *  0b11..Divide the FTM input clock by 16.
 */
#define FTM_PAIR0DEADTIME_DTPS(x)                (((uint32_t)(((uint32_t)(x)) << FTM_PAIR0DEADTIME_DTPS_SHIFT)) & FTM_PAIR0DEADTIME_DTPS_MASK)

#define FTM_PAIR0DEADTIME_DTVALEX_MASK           (0xF0000U)
#define FTM_PAIR0DEADTIME_DTVALEX_SHIFT          (16U)
/*! DTVALEX - Extended Deadtime Value */
#define FTM_PAIR0DEADTIME_DTVALEX(x)             (((uint32_t)(((uint32_t)(x)) << FTM_PAIR0DEADTIME_DTVALEX_SHIFT)) & FTM_PAIR0DEADTIME_DTVALEX_MASK)
/*! @} */

/*! @name PAIR1DEADTIME - Pair 1 Deadtime Configuration */
/*! @{ */

#define FTM_PAIR1DEADTIME_DTVAL_MASK             (0x3FU)
#define FTM_PAIR1DEADTIME_DTVAL_SHIFT            (0U)
/*! DTVAL - Deadtime Value */
#define FTM_PAIR1DEADTIME_DTVAL(x)               (((uint32_t)(((uint32_t)(x)) << FTM_PAIR1DEADTIME_DTVAL_SHIFT)) & FTM_PAIR1DEADTIME_DTVAL_MASK)

#define FTM_PAIR1DEADTIME_DTPS_MASK              (0xC0U)
#define FTM_PAIR1DEADTIME_DTPS_SHIFT             (6U)
/*! DTPS - Deadtime Prescaler Value
 *  0b0x..Divide the FTM input clock by 1.
 *  0b10..Divide the FTM input clock by 4.
 *  0b11..Divide the FTM input clock by 16.
 */
#define FTM_PAIR1DEADTIME_DTPS(x)                (((uint32_t)(((uint32_t)(x)) << FTM_PAIR1DEADTIME_DTPS_SHIFT)) & FTM_PAIR1DEADTIME_DTPS_MASK)

#define FTM_PAIR1DEADTIME_DTVALEX_MASK           (0xF0000U)
#define FTM_PAIR1DEADTIME_DTVALEX_SHIFT          (16U)
/*! DTVALEX - Extended Deadtime Value */
#define FTM_PAIR1DEADTIME_DTVALEX(x)             (((uint32_t)(((uint32_t)(x)) << FTM_PAIR1DEADTIME_DTVALEX_SHIFT)) & FTM_PAIR1DEADTIME_DTVALEX_MASK)
/*! @} */

/*! @name PAIR2DEADTIME - Pair 2 Deadtime Configuration */
/*! @{ */

#define FTM_PAIR2DEADTIME_DTVAL_MASK             (0x3FU)
#define FTM_PAIR2DEADTIME_DTVAL_SHIFT            (0U)
/*! DTVAL - Deadtime Value */
#define FTM_PAIR2DEADTIME_DTVAL(x)               (((uint32_t)(((uint32_t)(x)) << FTM_PAIR2DEADTIME_DTVAL_SHIFT)) & FTM_PAIR2DEADTIME_DTVAL_MASK)

#define FTM_PAIR2DEADTIME_DTPS_MASK              (0xC0U)
#define FTM_PAIR2DEADTIME_DTPS_SHIFT             (6U)
/*! DTPS - Deadtime Prescaler Value
 *  0b0x..Divide the FTM input clock by 1.
 *  0b10..Divide the FTM input clock by 4.
 *  0b11..Divide the FTM input clock by 16.
 */
#define FTM_PAIR2DEADTIME_DTPS(x)                (((uint32_t)(((uint32_t)(x)) << FTM_PAIR2DEADTIME_DTPS_SHIFT)) & FTM_PAIR2DEADTIME_DTPS_MASK)

#define FTM_PAIR2DEADTIME_DTVALEX_MASK           (0xF0000U)
#define FTM_PAIR2DEADTIME_DTVALEX_SHIFT          (16U)
/*! DTVALEX - Extended Deadtime Value */
#define FTM_PAIR2DEADTIME_DTVALEX(x)             (((uint32_t)(((uint32_t)(x)) << FTM_PAIR2DEADTIME_DTVALEX_SHIFT)) & FTM_PAIR2DEADTIME_DTVALEX_MASK)
/*! @} */

/*! @name PAIR3DEADTIME - Pair 3 Deadtime Configuration */
/*! @{ */

#define FTM_PAIR3DEADTIME_DTVAL_MASK             (0x3FU)
#define FTM_PAIR3DEADTIME_DTVAL_SHIFT            (0U)
/*! DTVAL - Deadtime Value */
#define FTM_PAIR3DEADTIME_DTVAL(x)               (((uint32_t)(((uint32_t)(x)) << FTM_PAIR3DEADTIME_DTVAL_SHIFT)) & FTM_PAIR3DEADTIME_DTVAL_MASK)

#define FTM_PAIR3DEADTIME_DTPS_MASK              (0xC0U)
#define FTM_PAIR3DEADTIME_DTPS_SHIFT             (6U)
/*! DTPS - Deadtime Prescaler Value
 *  0b0x..Divide the FTM input clock by 1.
 *  0b10..Divide the FTM input clock by 4.
 *  0b11..Divide the FTM input clock by 16.
 */
#define FTM_PAIR3DEADTIME_DTPS(x)                (((uint32_t)(((uint32_t)(x)) << FTM_PAIR3DEADTIME_DTPS_SHIFT)) & FTM_PAIR3DEADTIME_DTPS_MASK)

#define FTM_PAIR3DEADTIME_DTVALEX_MASK           (0xF0000U)
#define FTM_PAIR3DEADTIME_DTVALEX_SHIFT          (16U)
/*! DTVALEX - Extended Deadtime Value */
#define FTM_PAIR3DEADTIME_DTVALEX(x)             (((uint32_t)(((uint32_t)(x)) << FTM_PAIR3DEADTIME_DTVALEX_SHIFT)) & FTM_PAIR3DEADTIME_DTVALEX_MASK)
/*! @} */

/*! @name MOD_MIRROR - Mirror of Modulo Value */
/*! @{ */

#define FTM_MOD_MIRROR_FRACMOD_MASK              (0xF800U)
#define FTM_MOD_MIRROR_FRACMOD_SHIFT             (11U)
/*! FRACMOD - Modulo Fractional Value */
#define FTM_MOD_MIRROR_FRACMOD(x)                (((uint32_t)(((uint32_t)(x)) << FTM_MOD_MIRROR_FRACMOD_SHIFT)) & FTM_MOD_MIRROR_FRACMOD_MASK)

#define FTM_MOD_MIRROR_MOD_MASK                  (0xFFFF0000U)
#define FTM_MOD_MIRROR_MOD_SHIFT                 (16U)
/*! MOD - Mirror of the Modulo Integer Value */
#define FTM_MOD_MIRROR_MOD(x)                    (((uint32_t)(((uint32_t)(x)) << FTM_MOD_MIRROR_MOD_SHIFT)) & FTM_MOD_MIRROR_MOD_MASK)
/*! @} */

/*! @name CV_MIRROR - Mirror of Channel (n) Match Value */
/*! @{ */

#define FTM_CV_MIRROR_FRACVAL_MASK               (0xF800U)
#define FTM_CV_MIRROR_FRACVAL_SHIFT              (11U)
/*! FRACVAL - Channel (n) Match Fractional Value */
#define FTM_CV_MIRROR_FRACVAL(x)                 (((uint32_t)(((uint32_t)(x)) << FTM_CV_MIRROR_FRACVAL_SHIFT)) & FTM_CV_MIRROR_FRACVAL_MASK)

#define FTM_CV_MIRROR_VAL_MASK                   (0xFFFF0000U)
#define FTM_CV_MIRROR_VAL_SHIFT                  (16U)
/*! VAL - Mirror of the Channel (n) Match Integer Value */
#define FTM_CV_MIRROR_VAL(x)                     (((uint32_t)(((uint32_t)(x)) << FTM_CV_MIRROR_VAL_SHIFT)) & FTM_CV_MIRROR_VAL_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group FTM_Register_Masks */


/*!
 * @}
 */ /* end of group FTM_Peripheral_Access_Layer */


/*
** End of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #if (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
  #else
    #pragma pop
  #endif
#elif defined(__GNUC__)
  /* leave anonymous unions enabled */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=default
#else
  #error Not supported compiler type
#endif

/*!
 * @}
 */ /* end of group Peripheral_access_layer */


#endif  /* PERI_FTM_H_ */


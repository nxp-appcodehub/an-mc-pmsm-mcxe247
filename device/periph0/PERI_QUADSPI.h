/*
** ###################################################################
**     Processors:          MCXE247VLL
**                          MCXE247VLQ
**
**     Version:             rev. 1.0, 2025-02-21
**     Build:               b250417
**
**     Abstract:
**         CMSIS Peripheral Access Layer for QuadSPI
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
 * @file PERI_QuadSPI.h
 * @version 1.0
 * @date 2025-02-21
 * @brief CMSIS Peripheral Access Layer for QuadSPI
 *
 * CMSIS Peripheral Access Layer for QuadSPI
 */

#if !defined(PERI_QUADSPI_H_)
#define PERI_QUADSPI_H_                          /**< Symbol preventing repeated inclusion */

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
   -- QuadSPI Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QuadSPI_Peripheral_Access_Layer QuadSPI Peripheral Access Layer
 * @{
 */

/** QuadSPI - Size of Registers Arrays */
#define QuadSPI_RBDR_COUNT                        32u
#define QuadSPI_LUT_COUNT                         64u

/** QuadSPI - Register Layout Typedef */
typedef struct {
  __IO uint32_t MCR;                               /**< Module Configuration Register, offset: 0x0 */
       uint8_t RESERVED_0[4];
  __IO uint32_t IPCR;                              /**< IP Configuration Register, offset: 0x8 */
  __IO uint32_t FLSHCR;                            /**< Flash Configuration Register, offset: 0xC */
  __IO uint32_t BUF0CR;                            /**< Buffer0 Configuration Register, offset: 0x10 */
  __IO uint32_t BUF1CR;                            /**< Buffer1 Configuration Register, offset: 0x14 */
  __IO uint32_t BUF2CR;                            /**< Buffer2 Configuration Register, offset: 0x18 */
  __IO uint32_t BUF3CR;                            /**< Buffer3 Configuration Register, offset: 0x1C */
  __IO uint32_t BFGENCR;                           /**< Buffer Generic Configuration Register, offset: 0x20 */
  __IO uint32_t SOCCR;                             /**< SOC Configuration Register, offset: 0x24 */
       uint8_t RESERVED_1[8];
  __IO uint32_t BUF0IND;                           /**< Buffer0 Top Index Register, offset: 0x30 */
  __IO uint32_t BUF1IND;                           /**< Buffer1 Top Index Register, offset: 0x34 */
  __IO uint32_t BUF2IND;                           /**< Buffer2 Top Index Register, offset: 0x38 */
       uint8_t RESERVED_2[196];
  __IO uint32_t SFAR;                              /**< Serial Flash Address Register, offset: 0x100 */
  __IO uint32_t SFACR;                             /**< Serial Flash Address Configuration Register, offset: 0x104 */
  __IO uint32_t SMPR;                              /**< Sampling Register, offset: 0x108 */
  __I  uint32_t RBSR;                              /**< RX Buffer Status Register, offset: 0x10C */
  __IO uint32_t RBCT;                              /**< RX Buffer Control Register, offset: 0x110 */
       uint8_t RESERVED_3[60];
  __I  uint32_t TBSR;                              /**< TX Buffer Status Register, offset: 0x150 */
  __IO uint32_t TBDR;                              /**< TX Buffer Data Register, offset: 0x154 */
  __IO uint32_t TBCT;                              /**< Tx Buffer Control Register, offset: 0x158 */
  __I  uint32_t SR;                                /**< Status Register, offset: 0x15C */
  __IO uint32_t FR;                                /**< Flag Register, offset: 0x160 */
  __IO uint32_t RSER;                              /**< Interrupt and DMA Request Select and Enable Register, offset: 0x164 */
  __I  uint32_t SPNDST;                            /**< Sequence Suspend Status Register, offset: 0x168 */
  __IO uint32_t SPTRCLR;                           /**< Sequence Pointer Clear Register, offset: 0x16C */
       uint8_t RESERVED_4[16];
  __IO uint32_t SFA1AD;                            /**< Serial Flash A1 Top Address, offset: 0x180 */
  __IO uint32_t SFA2AD;                            /**< Serial Flash A2 Top Address, offset: 0x184 */
  __IO uint32_t SFB1AD;                            /**< Serial Flash B1 Top Address, offset: 0x188 */
  __IO uint32_t SFB2AD;                            /**< Serial Flash B2 Top Address, offset: 0x18C */
       uint8_t RESERVED_5[112];
  __I  uint32_t RBDR[QuadSPI_RBDR_COUNT];          /**< RX Buffer Data Register, array offset: 0x200, array step: 0x4 */
       uint8_t RESERVED_6[128];
  __IO uint32_t LUTKEY;                            /**< LUT Key Register, offset: 0x300 */
  __IO uint32_t LCKCR;                             /**< LUT Lock Configuration Register, offset: 0x304 */
       uint8_t RESERVED_7[8];
  __IO uint32_t LUT[QuadSPI_LUT_COUNT];            /**< Look-up Table register, array offset: 0x310, array step: 0x4 */
} QuadSPI_Type;

/* ----------------------------------------------------------------------------
   -- QuadSPI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup QuadSPI_Register_Masks QuadSPI Register Masks
 * @{
 */

/*! @name MCR - Module Configuration Register */
/*! @{ */

#define QuadSPI_MCR_SWRSTSD_MASK                 (0x1U)
#define QuadSPI_MCR_SWRSTSD_SHIFT                (0U)
/*! SWRSTSD
 *  0b0..No action
 *  0b1..Serial Flash domain flops are reset. Does not reset configuration registers. It is advisable to reset
 *       both the serial flash domain and AHB domain at the same time. Resetting only one domain might lead to side
 *       effects. The software resets need the clock to be running to propagate to the design. The MCR[MDIS] should
 *       therefore be set to 0 when the software reset bits are asserted. Also, before they can be deasserted again
 *       (by setting MCR[SWRSTSD] to 0), it is recommended to set the MCR[MDIS] bit to 1. Once the software resets
 *       have been deasserted, the normal operation can be started by setting the MCR[MDIS] bit to 0.
 */
#define QuadSPI_MCR_SWRSTSD(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_SWRSTSD_SHIFT)) & QuadSPI_MCR_SWRSTSD_MASK)

#define QuadSPI_MCR_SWRSTHD_MASK                 (0x2U)
#define QuadSPI_MCR_SWRSTHD_SHIFT                (1U)
/*! SWRSTHD
 *  0b0..No action
 *  0b1..AHB domain flops are reset. Does not reset configuration registers. It is advisable to reset both the
 *       serial flash domain and AHB domain at the same time. Resetting only one domain might lead to side effects.
 *       The software resets need the clock to be running to propagate to the design. The MCR[MDIS] should therefore
 *       be set to 0 when the software reset bits are asserted. Also, before they can be deasserted again (by
 *       setting MCR[SWRSTHD] to 0), it is recommended to set the MCR[MDIS] bit to 1. Once the software resets have been
 *       deasserted, the normal operation can be started by setting the MCR[MDIS] bit to 0.
 */
#define QuadSPI_MCR_SWRSTHD(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_SWRSTHD_SHIFT)) & QuadSPI_MCR_SWRSTHD_MASK)

#define QuadSPI_MCR_END_CFG_MASK                 (0xCU)
#define QuadSPI_MCR_END_CFG_SHIFT                (2U)
#define QuadSPI_MCR_END_CFG(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_END_CFG_SHIFT)) & QuadSPI_MCR_END_CFG_MASK)

#define QuadSPI_MCR_DQS_OUT_EN_MASK              (0x10U)
#define QuadSPI_MCR_DQS_OUT_EN_SHIFT             (4U)
/*! DQS_OUT_EN
 *  0b0..DQS as an output from controller is disabled
 *  0b1..DQS as an output from controller is enabled
 */
#define QuadSPI_MCR_DQS_OUT_EN(x)                (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_DQS_OUT_EN_SHIFT)) & QuadSPI_MCR_DQS_OUT_EN_MASK)

#define QuadSPI_MCR_DQS_LAT_EN_MASK              (0x20U)
#define QuadSPI_MCR_DQS_LAT_EN_SHIFT             (5U)
/*! DQS_LAT_EN
 *  0b0..DQS Latency disabled
 *  0b1..DQS feature with latency included enabled
 */
#define QuadSPI_MCR_DQS_LAT_EN(x)                (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_DQS_LAT_EN_SHIFT)) & QuadSPI_MCR_DQS_LAT_EN_MASK)

#define QuadSPI_MCR_DQS_EN_MASK                  (0x40U)
#define QuadSPI_MCR_DQS_EN_SHIFT                 (6U)
/*! DQS_EN
 *  0b0..DQS disabled.
 *  0b1..DQS enabled. When enabled, the incoming data is sampled on both the edges of DQS input when
 *       QSPI_MCR[DDR_EN] is set, else, on only one edge when QSPI_MCR[DDR_EN] is 0. The QSPI_SMPR[DDR_SMP] values are ignored.
 */
#define QuadSPI_MCR_DQS_EN(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_DQS_EN_SHIFT)) & QuadSPI_MCR_DQS_EN_MASK)

#define QuadSPI_MCR_DDR_EN_MASK                  (0x80U)
#define QuadSPI_MCR_DDR_EN_SHIFT                 (7U)
/*! DDR_EN
 *  0b0..2x clock are disabled for SDR instructions only
 *  0b1..2x clock are enabled supports both SDR and DDR instruction.
 */
#define QuadSPI_MCR_DDR_EN(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_DDR_EN_SHIFT)) & QuadSPI_MCR_DDR_EN_MASK)

#define QuadSPI_MCR_VAR_LAT_EN_MASK              (0x100U)
#define QuadSPI_MCR_VAR_LAT_EN_SHIFT             (8U)
/*! VAR_LAT_EN
 *  0b0..Fixed latency: Twice + 1 latency enable
 *  0b1..Variable latency: 'once' or 'twice + 1' the initial latency based on Data strobe during CA phase. If
 *       enabled also need to ensure QSPI_FLSHCR[TCSS] must be >= 2.
 */
#define QuadSPI_MCR_VAR_LAT_EN(x)                (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_VAR_LAT_EN_SHIFT)) & QuadSPI_MCR_VAR_LAT_EN_MASK)

#define QuadSPI_MCR_CLR_RXF_MASK                 (0x400U)
#define QuadSPI_MCR_CLR_RXF_SHIFT                (10U)
/*! CLR_RXF
 *  0b0..No action.
 *  0b1..Read and write pointers of the RX Buffer are reset to 0. QSPI_RBSR[RDBFL] is reset to 0.
 */
#define QuadSPI_MCR_CLR_RXF(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_CLR_RXF_SHIFT)) & QuadSPI_MCR_CLR_RXF_MASK)

#define QuadSPI_MCR_CLR_TXF_MASK                 (0x800U)
#define QuadSPI_MCR_CLR_TXF_SHIFT                (11U)
/*! CLR_TXF
 *  0b0..No action.
 *  0b1..Read and write pointers of the TX Buffer are reset to 0. QSPI_TBSR[TRCTR] is reset to 0.
 */
#define QuadSPI_MCR_CLR_TXF(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_CLR_TXF_SHIFT)) & QuadSPI_MCR_CLR_TXF_MASK)

#define QuadSPI_MCR_MDIS_MASK                    (0x4000U)
#define QuadSPI_MCR_MDIS_SHIFT                   (14U)
/*! MDIS
 *  0b0..Enable QuadSPI clocks.
 *  0b1..Allow external logic to disable QuadSPI clocks.
 */
#define QuadSPI_MCR_MDIS(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_MDIS_SHIFT)) & QuadSPI_MCR_MDIS_MASK)

#define QuadSPI_MCR_DOZE_MASK                    (0x8000U)
#define QuadSPI_MCR_DOZE_SHIFT                   (15U)
/*! DOZE
 *  0b0..A doze request will be ignored by the QuadSPI module
 *  0b1..A doze request will be processed by the QuadSPI module
 */
#define QuadSPI_MCR_DOZE(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_DOZE_SHIFT)) & QuadSPI_MCR_DOZE_MASK)

#define QuadSPI_MCR_ISD2FA_MASK                  (0x10000U)
#define QuadSPI_MCR_ISD2FA_SHIFT                 (16U)
/*! ISD2FA
 *  0b0..IOFA[2] is driven to logic L
 *  0b1..IOFA[2] is driven to logic H
 */
#define QuadSPI_MCR_ISD2FA(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_ISD2FA_SHIFT)) & QuadSPI_MCR_ISD2FA_MASK)

#define QuadSPI_MCR_ISD3FA_MASK                  (0x20000U)
#define QuadSPI_MCR_ISD3FA_SHIFT                 (17U)
/*! ISD3FA
 *  0b0..IOFA[3] is driven to logic L
 *  0b1..IOFA[3] is driven to logic H
 */
#define QuadSPI_MCR_ISD3FA(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_ISD3FA_SHIFT)) & QuadSPI_MCR_ISD3FA_MASK)

#define QuadSPI_MCR_ISD2FB_MASK                  (0x40000U)
#define QuadSPI_MCR_ISD2FB_SHIFT                 (18U)
/*! ISD2FB
 *  0b0..IOFB[2] is driven to logic L
 *  0b1..IOFB[2] is driven to logic H
 */
#define QuadSPI_MCR_ISD2FB(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_ISD2FB_SHIFT)) & QuadSPI_MCR_ISD2FB_MASK)

#define QuadSPI_MCR_ISD3FB_MASK                  (0x80000U)
#define QuadSPI_MCR_ISD3FB_SHIFT                 (19U)
/*! ISD3FB
 *  0b0..IOFB[3] is driven to logic L
 *  0b1..IOFB[3] is driven to logic H
 */
#define QuadSPI_MCR_ISD3FB(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_ISD3FB_SHIFT)) & QuadSPI_MCR_ISD3FB_MASK)

#define QuadSPI_MCR_SCLKCFG_MASK                 (0xFF000000U)
#define QuadSPI_MCR_SCLKCFG_SHIFT                (24U)
#define QuadSPI_MCR_SCLKCFG(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_MCR_SCLKCFG_SHIFT)) & QuadSPI_MCR_SCLKCFG_MASK)
/*! @} */

/*! @name IPCR - IP Configuration Register */
/*! @{ */

#define QuadSPI_IPCR_IDATSZ_MASK                 (0xFFFFU)
#define QuadSPI_IPCR_IDATSZ_SHIFT                (0U)
#define QuadSPI_IPCR_IDATSZ(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_IPCR_IDATSZ_SHIFT)) & QuadSPI_IPCR_IDATSZ_MASK)

#define QuadSPI_IPCR_SEQID_MASK                  (0xF000000U)
#define QuadSPI_IPCR_SEQID_SHIFT                 (24U)
#define QuadSPI_IPCR_SEQID(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_IPCR_SEQID_SHIFT)) & QuadSPI_IPCR_SEQID_MASK)
/*! @} */

/*! @name FLSHCR - Flash Configuration Register */
/*! @{ */

#define QuadSPI_FLSHCR_TCSS_MASK                 (0xFU)
#define QuadSPI_FLSHCR_TCSS_SHIFT                (0U)
#define QuadSPI_FLSHCR_TCSS(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_FLSHCR_TCSS_SHIFT)) & QuadSPI_FLSHCR_TCSS_MASK)

#define QuadSPI_FLSHCR_TCSH_MASK                 (0xF00U)
#define QuadSPI_FLSHCR_TCSH_SHIFT                (8U)
#define QuadSPI_FLSHCR_TCSH(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_FLSHCR_TCSH_SHIFT)) & QuadSPI_FLSHCR_TCSH_MASK)

#define QuadSPI_FLSHCR_TDH_MASK                  (0x30000U)
#define QuadSPI_FLSHCR_TDH_SHIFT                 (16U)
/*! TDH
 *  0b00..Data aligned with the posedge of Internal reference clock of QuadSPI
 *  0b01..Data aligned with 2x serial flash half clock
 */
#define QuadSPI_FLSHCR_TDH(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_FLSHCR_TDH_SHIFT)) & QuadSPI_FLSHCR_TDH_MASK)
/*! @} */

/*! @name BUF0CR - Buffer0 Configuration Register */
/*! @{ */

#define QuadSPI_BUF0CR_MSTRID_MASK               (0xFU)
#define QuadSPI_BUF0CR_MSTRID_SHIFT              (0U)
#define QuadSPI_BUF0CR_MSTRID(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF0CR_MSTRID_SHIFT)) & QuadSPI_BUF0CR_MSTRID_MASK)

#define QuadSPI_BUF0CR_ADATSZ_MASK               (0xFF00U)
#define QuadSPI_BUF0CR_ADATSZ_SHIFT              (8U)
/*! ADATSZ - AHB data transfer size */
#define QuadSPI_BUF0CR_ADATSZ(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF0CR_ADATSZ_SHIFT)) & QuadSPI_BUF0CR_ADATSZ_MASK)

#define QuadSPI_BUF0CR_HP_EN_MASK                (0x80000000U)
#define QuadSPI_BUF0CR_HP_EN_SHIFT               (31U)
#define QuadSPI_BUF0CR_HP_EN(x)                  (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF0CR_HP_EN_SHIFT)) & QuadSPI_BUF0CR_HP_EN_MASK)
/*! @} */

/*! @name BUF1CR - Buffer1 Configuration Register */
/*! @{ */

#define QuadSPI_BUF1CR_MSTRID_MASK               (0xFU)
#define QuadSPI_BUF1CR_MSTRID_SHIFT              (0U)
#define QuadSPI_BUF1CR_MSTRID(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF1CR_MSTRID_SHIFT)) & QuadSPI_BUF1CR_MSTRID_MASK)

#define QuadSPI_BUF1CR_ADATSZ_MASK               (0xFF00U)
#define QuadSPI_BUF1CR_ADATSZ_SHIFT              (8U)
/*! ADATSZ - AHB data transfer size */
#define QuadSPI_BUF1CR_ADATSZ(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF1CR_ADATSZ_SHIFT)) & QuadSPI_BUF1CR_ADATSZ_MASK)
/*! @} */

/*! @name BUF2CR - Buffer2 Configuration Register */
/*! @{ */

#define QuadSPI_BUF2CR_MSTRID_MASK               (0xFU)
#define QuadSPI_BUF2CR_MSTRID_SHIFT              (0U)
#define QuadSPI_BUF2CR_MSTRID(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF2CR_MSTRID_SHIFT)) & QuadSPI_BUF2CR_MSTRID_MASK)

#define QuadSPI_BUF2CR_ADATSZ_MASK               (0xFF00U)
#define QuadSPI_BUF2CR_ADATSZ_SHIFT              (8U)
/*! ADATSZ - AHB data transfer size */
#define QuadSPI_BUF2CR_ADATSZ(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF2CR_ADATSZ_SHIFT)) & QuadSPI_BUF2CR_ADATSZ_MASK)
/*! @} */

/*! @name BUF3CR - Buffer3 Configuration Register */
/*! @{ */

#define QuadSPI_BUF3CR_MSTRID_MASK               (0xFU)
#define QuadSPI_BUF3CR_MSTRID_SHIFT              (0U)
#define QuadSPI_BUF3CR_MSTRID(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF3CR_MSTRID_SHIFT)) & QuadSPI_BUF3CR_MSTRID_MASK)

#define QuadSPI_BUF3CR_ADATSZ_MASK               (0xFF00U)
#define QuadSPI_BUF3CR_ADATSZ_SHIFT              (8U)
/*! ADATSZ - AHB data transfer size */
#define QuadSPI_BUF3CR_ADATSZ(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF3CR_ADATSZ_SHIFT)) & QuadSPI_BUF3CR_ADATSZ_MASK)

#define QuadSPI_BUF3CR_ALLMST_MASK               (0x80000000U)
#define QuadSPI_BUF3CR_ALLMST_SHIFT              (31U)
#define QuadSPI_BUF3CR_ALLMST(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF3CR_ALLMST_SHIFT)) & QuadSPI_BUF3CR_ALLMST_MASK)
/*! @} */

/*! @name BFGENCR - Buffer Generic Configuration Register */
/*! @{ */

#define QuadSPI_BFGENCR_SEQID_MASK               (0xF000U)
#define QuadSPI_BFGENCR_SEQID_SHIFT              (12U)
#define QuadSPI_BFGENCR_SEQID(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_BFGENCR_SEQID_SHIFT)) & QuadSPI_BFGENCR_SEQID_MASK)
/*! @} */

/*! @name SOCCR - SOC Configuration Register */
/*! @{ */

#define QuadSPI_SOCCR_QSPISRC_MASK               (0x7U)
#define QuadSPI_SOCCR_QSPISRC_SHIFT              (0U)
/*! QSPISRC - QSPI clock source select
 *  0b000..Core/system clock
 *  0b001..MCGFLL clock
 *  0b010..MCGPLL clock
 *  0b011..MCGPLL 2x clock (DDR mode specific)
 *  0b100..IRC48M clock
 *  0b101..OSCERCLK clock
 *  0b110..MCGIRCLK clock
 *  0b111..Reserved
 */
#define QuadSPI_SOCCR_QSPISRC(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_QSPISRC_SHIFT)) & QuadSPI_SOCCR_QSPISRC_MASK)

#define QuadSPI_SOCCR_DQSLPEN_MASK               (0x100U)
#define QuadSPI_SOCCR_DQSLPEN_SHIFT              (8U)
/*! DQSLPEN - When this bit is set the internal generated DQS is selected and looped back to
 *    QuadSPI, without going to DQS pad. DQSPADLPEN should be cleared when this bit is set.
 *  0b0..DQS loop back is disabled
 *  0b1..DQS loop back is enabled
 */
#define QuadSPI_SOCCR_DQSLPEN(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_DQSLPEN_SHIFT)) & QuadSPI_SOCCR_DQSLPEN_MASK)

#define QuadSPI_SOCCR_DQSPADLPEN_MASK            (0x200U)
#define QuadSPI_SOCCR_DQSPADLPEN_SHIFT           (9U)
/*! DQSPADLPEN - When this bit is set the internal generated DQS will be sent to the DQS pad first
 *    and then looped back to QuadSPI. DQSLPEN should be cleared when this bit is set.
 *  0b0..DQS loop back from DQS pad is disabled
 *  0b1..DQS loop back from DQS pad is enabled
 */
#define QuadSPI_SOCCR_DQSPADLPEN(x)              (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_DQSPADLPEN_SHIFT)) & QuadSPI_SOCCR_DQSPADLPEN_MASK)

#define QuadSPI_SOCCR_DQSPHASEL_MASK             (0xC00U)
#define QuadSPI_SOCCR_DQSPHASEL_SHIFT            (10U)
/*! DQSPHASEL - Select phase shift for internal DQS generation. These bits are always zero in SDR mode.
 *  0b00..No phase shift
 *  0b01..Select 45 degree phase shift
 *  0b10..Select 90 degree phase shift
 *  0b11..Select 135 degree phase shift
 */
#define QuadSPI_SOCCR_DQSPHASEL(x)               (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_DQSPHASEL_SHIFT)) & QuadSPI_SOCCR_DQSPHASEL_MASK)

#define QuadSPI_SOCCR_DQSINVSEL_MASK             (0x1000U)
#define QuadSPI_SOCCR_DQSINVSEL_SHIFT            (12U)
/*! DQSINVSEL - Select clock source for internal DQS generation
 *  0b0..Use 1x internal reference clock for the DQS generation
 *  0b1..Use inverse 1x internal reference clock for the DQS generation
 */
#define QuadSPI_SOCCR_DQSINVSEL(x)               (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_DQSINVSEL_SHIFT)) & QuadSPI_SOCCR_DQSINVSEL_MASK)

#define QuadSPI_SOCCR_CK2EN_MASK                 (0x2000U)
#define QuadSPI_SOCCR_CK2EN_SHIFT                (13U)
/*! CK2EN - Flash CK2 clock pin enable
 *  0b0..CK2 flash clock is disabled
 *  0b1..CK2 flash clock is enabled
 */
#define QuadSPI_SOCCR_CK2EN(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_CK2EN_SHIFT)) & QuadSPI_SOCCR_CK2EN_MASK)

#define QuadSPI_SOCCR_DIFFCKEN_MASK              (0x4000U)
#define QuadSPI_SOCCR_DIFFCKEN_SHIFT             (14U)
/*! DIFFCKEN - Differential flash clock pins enable
 *  0b0..Differential flash clock is disabled
 *  0b1..Differential flash clock is enabled
 */
#define QuadSPI_SOCCR_DIFFCKEN(x)                (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_DIFFCKEN_SHIFT)) & QuadSPI_SOCCR_DIFFCKEN_MASK)

#define QuadSPI_SOCCR_OCTEN_MASK                 (0x8000U)
#define QuadSPI_SOCCR_OCTEN_SHIFT                (15U)
/*! OCTEN - Octal data pins enable
 *  0b0..QSPI0B_DATAx pins are assigned to QSPI Port B
 *  0b1..QSPI0B_DATAx pins are assigned to QSPI Port A
 */
#define QuadSPI_SOCCR_OCTEN(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_OCTEN_SHIFT)) & QuadSPI_SOCCR_OCTEN_MASK)

#define QuadSPI_SOCCR_DLYTAPSELA_MASK            (0x3F0000U)
#define QuadSPI_SOCCR_DLYTAPSELA_SHIFT           (16U)
/*! DLYTAPSELA - Delay chain tap number selection for QSPI Port A DQS
 *  0b000000..Select 1 delay chain tap
 *  0b000001..Select 2 delay chain tap
 *  0b000010..Select 3 delay chain tap
 *  0b000011..Select 4 delay chain tap
 *  0b000100..Select 5 delay chain tap
 *  0b000101..Select 6 delay chain tap
 *  0b000110..Select 7 delay chain tap
 *  0b000111..Select 8 delay chain tap
 *  0b001000..Select 9 delay chain tap
 *  0b001001..Select 10 delay chain tap
 *  0b001010..Select 11 delay chain tap
 *  0b001011..Select 12 delay chain tap
 *  0b001100..Select 13 delay chain tap
 *  0b001101..Select 14 delay chain tap
 *  0b001110..Select 15 delay chain tap
 *  0b001111..Select 16 delay chain tap
 *  0b010000..Select 17 delay chain tap
 *  0b010001..Select 18 delay chain tap
 *  0b010010..Select 19 delay chain tap
 *  0b010011..Select 20 delay chain tap
 *  0b010100..Select 21 delay chain tap
 *  0b010101..Select 22 delay chain tap
 *  0b010110..Select 23 delay chain tap
 *  0b010111..Select 24 delay chain tap
 *  0b011000..Select 25 delay chain tap
 *  0b011001..Select 26 delay chain tap
 *  0b011010..Select 27 delay chain tap
 *  0b011011..Select 28 delay chain tap
 *  0b011100..Select 29 delay chain tap
 *  0b011101..Select 30 delay chain tap
 *  0b011110..Select 31 delay chain tap
 *  0b011111..Select 32 delay chain tap
 *  0b100000..Select 33 delay chain tap
 *  0b100001..Select 34 delay chain tap
 *  0b100010..Select 35 delay chain tap
 *  0b100011..Select 36 delay chain tap
 *  0b100100..Select 37 delay chain tap
 *  0b100101..Select 38 delay chain tap
 *  0b100110..Select 39 delay chain tap
 *  0b100111..Select 40 delay chain tap
 *  0b101000..Select 41 delay chain tap
 *  0b101001..Select 42 delay chain tap
 *  0b101010..Select 43 delay chain tap
 *  0b101011..Select 44 delay chain tap
 *  0b101100..Select 45 delay chain tap
 *  0b101101..Select 46 delay chain tap
 *  0b101110..Select 47 delay chain tap
 *  0b101111..Select 48 delay chain tap
 *  0b110000..Select 49 delay chain tap
 *  0b110001..Select 50 delay chain tap
 *  0b110010..Select 51 delay chain tap
 *  0b110011..Select 52 delay chain tap
 *  0b110100..Select 53 delay chain tap
 *  0b110101..Select 54 delay chain tap
 *  0b110110..Select 55 delay chain tap
 *  0b110111..Select 56 delay chain tap
 *  0b111000..Select 57 delay chain tap
 *  0b111001..Select 58 delay chain tap
 *  0b111010..Select 59 delay chain tap
 *  0b111011..Select 60 delay chain tap
 *  0b111100..Select 61 delay chain tap
 *  0b111101..Select 62 delay chain tap
 *  0b111110..Select 63 delay chain tap
 *  0b111111..Select 64 delay chain tap
 */
#define QuadSPI_SOCCR_DLYTAPSELA(x)              (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_DLYTAPSELA_SHIFT)) & QuadSPI_SOCCR_DLYTAPSELA_MASK)

#define QuadSPI_SOCCR_DLYTAPSELB_MASK            (0x3F000000U)
#define QuadSPI_SOCCR_DLYTAPSELB_SHIFT           (24U)
/*! DLYTAPSELB - Delay chain tap number selection for QSPI Port B DQS
 *  0b000000..Select 1 delay chain tap
 *  0b000001..Select 2 delay chain tap
 *  0b000010..Select 3 delay chain tap
 *  0b000011..Select 4 delay chain tap
 *  0b000100..Select 5 delay chain tap
 *  0b000101..Select 6 delay chain tap
 *  0b000110..Select 7 delay chain tap
 *  0b000111..Select 8 delay chain tap
 *  0b001000..Select 9 delay chain tap
 *  0b001001..Select 10 delay chain tap
 *  0b001010..Select 11 delay chain tap
 *  0b001011..Select 12 delay chain tap
 *  0b001100..Select 13 delay chain tap
 *  0b001101..Select 14 delay chain tap
 *  0b001110..Select 15 delay chain tap
 *  0b001111..Select 16 delay chain tap
 *  0b010000..Select 17 delay chain tap
 *  0b010001..Select 18 delay chain tap
 *  0b010010..Select 19 delay chain tap
 *  0b010011..Select 20 delay chain tap
 *  0b010100..Select 21 delay chain tap
 *  0b010101..Select 22 delay chain tap
 *  0b010110..Select 23 delay chain tap
 *  0b010111..Select 24 delay chain tap
 *  0b011000..Select 25 delay chain tap
 *  0b011001..Select 26 delay chain tap
 *  0b011010..Select 27 delay chain tap
 *  0b011011..Select 28 delay chain tap
 *  0b011100..Select 29 delay chain tap
 *  0b011101..Select 30 delay chain tap
 *  0b011110..Select 31 delay chain tap
 *  0b011111..Select 32 delay chain tap
 *  0b100000..Select 33 delay chain tap
 *  0b100001..Select 34 delay chain tap
 *  0b100010..Select 35 delay chain tap
 *  0b100011..Select 36 delay chain tap
 *  0b100100..Select 37 delay chain tap
 *  0b100101..Select 38 delay chain tap
 *  0b100110..Select 39 delay chain tap
 *  0b100111..Select 40 delay chain tap
 *  0b101000..Select 41 delay chain tap
 *  0b101001..Select 42 delay chain tap
 *  0b101010..Select 43 delay chain tap
 *  0b101011..Select 44 delay chain tap
 *  0b101100..Select 45 delay chain tap
 *  0b101101..Select 46 delay chain tap
 *  0b101110..Select 47 delay chain tap
 *  0b101111..Select 48 delay chain tap
 *  0b110000..Select 49 delay chain tap
 *  0b110001..Select 50 delay chain tap
 *  0b110010..Select 51 delay chain tap
 *  0b110011..Select 52 delay chain tap
 *  0b110100..Select 53 delay chain tap
 *  0b110101..Select 54 delay chain tap
 *  0b110110..Select 55 delay chain tap
 *  0b110111..Select 56 delay chain tap
 *  0b111000..Select 57 delay chain tap
 *  0b111001..Select 58 delay chain tap
 *  0b111010..Select 59 delay chain tap
 *  0b111011..Select 60 delay chain tap
 *  0b111100..Select 61 delay chain tap
 *  0b111101..Select 62 delay chain tap
 *  0b111110..Select 63 delay chain tap
 *  0b111111..Select 64 delay chain tap
 */
#define QuadSPI_SOCCR_DLYTAPSELB(x)              (((uint32_t)(((uint32_t)(x)) << QuadSPI_SOCCR_DLYTAPSELB_SHIFT)) & QuadSPI_SOCCR_DLYTAPSELB_MASK)
/*! @} */

/*! @name BUF0IND - Buffer0 Top Index Register */
/*! @{ */

#define QuadSPI_BUF0IND_TPINDX0_MASK             (0xFFFFFFF8U)
#define QuadSPI_BUF0IND_TPINDX0_SHIFT            (3U)
#define QuadSPI_BUF0IND_TPINDX0(x)               (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF0IND_TPINDX0_SHIFT)) & QuadSPI_BUF0IND_TPINDX0_MASK)
/*! @} */

/*! @name BUF1IND - Buffer1 Top Index Register */
/*! @{ */

#define QuadSPI_BUF1IND_TPINDX1_MASK             (0xFFFFFFF8U)
#define QuadSPI_BUF1IND_TPINDX1_SHIFT            (3U)
#define QuadSPI_BUF1IND_TPINDX1(x)               (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF1IND_TPINDX1_SHIFT)) & QuadSPI_BUF1IND_TPINDX1_MASK)
/*! @} */

/*! @name BUF2IND - Buffer2 Top Index Register */
/*! @{ */

#define QuadSPI_BUF2IND_TPINDX2_MASK             (0xFFFFFFF8U)
#define QuadSPI_BUF2IND_TPINDX2_SHIFT            (3U)
#define QuadSPI_BUF2IND_TPINDX2(x)               (((uint32_t)(((uint32_t)(x)) << QuadSPI_BUF2IND_TPINDX2_SHIFT)) & QuadSPI_BUF2IND_TPINDX2_MASK)
/*! @} */

/*! @name SFAR - Serial Flash Address Register */
/*! @{ */

#define QuadSPI_SFAR_SFADR_MASK                  (0xFFFFFFFFU)
#define QuadSPI_SFAR_SFADR_SHIFT                 (0U)
#define QuadSPI_SFAR_SFADR(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SFAR_SFADR_SHIFT)) & QuadSPI_SFAR_SFADR_MASK)
/*! @} */

/*! @name SFACR - Serial Flash Address Configuration Register */
/*! @{ */

#define QuadSPI_SFACR_CAS_MASK                   (0xFU)
#define QuadSPI_SFACR_CAS_SHIFT                  (0U)
/*! CAS - Column Address Space */
#define QuadSPI_SFACR_CAS(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SFACR_CAS_SHIFT)) & QuadSPI_SFACR_CAS_MASK)

#define QuadSPI_SFACR_WA_MASK                    (0x10000U)
#define QuadSPI_SFACR_WA_SHIFT                   (16U)
/*! WA - Word Addressable
 *  0b0..Byte addressable serial flash mode.
 *  0b1..Word (2 byte) addressable serial flash mode.
 */
#define QuadSPI_SFACR_WA(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_SFACR_WA_SHIFT)) & QuadSPI_SFACR_WA_MASK)
/*! @} */

/*! @name SMPR - Sampling Register */
/*! @{ */

#define QuadSPI_SMPR_FSPHS_MASK                  (0x20U)
#define QuadSPI_SMPR_FSPHS_SHIFT                 (5U)
/*! FSPHS - Full Speed Phase selection for SDR instructions.
 *  0b0..Select sampling at non-inverted clock
 *  0b1..Select sampling at inverted clock.
 */
#define QuadSPI_SMPR_FSPHS(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SMPR_FSPHS_SHIFT)) & QuadSPI_SMPR_FSPHS_MASK)

#define QuadSPI_SMPR_FSDLY_MASK                  (0x40U)
#define QuadSPI_SMPR_FSDLY_SHIFT                 (6U)
/*! FSDLY - Full Speed Delay selection for SDR instructions. Select the delay with respect to the
 *    reference edge for the sample point valid for full speed commands.
 *  0b0..One clock cycle delay
 *  0b1..Two clock cycles delay.
 */
#define QuadSPI_SMPR_FSDLY(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SMPR_FSDLY_SHIFT)) & QuadSPI_SMPR_FSDLY_MASK)
/*! @} */

/*! @name RBSR - RX Buffer Status Register */
/*! @{ */

#define QuadSPI_RBSR_RDBFL_MASK                  (0x3F00U)
#define QuadSPI_RBSR_RDBFL_SHIFT                 (8U)
#define QuadSPI_RBSR_RDBFL(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RBSR_RDBFL_SHIFT)) & QuadSPI_RBSR_RDBFL_MASK)

#define QuadSPI_RBSR_RDCTR_MASK                  (0xFFFF0000U)
#define QuadSPI_RBSR_RDCTR_SHIFT                 (16U)
#define QuadSPI_RBSR_RDCTR(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RBSR_RDCTR_SHIFT)) & QuadSPI_RBSR_RDCTR_MASK)
/*! @} */

/*! @name RBCT - RX Buffer Control Register */
/*! @{ */

#define QuadSPI_RBCT_WMRK_MASK                   (0x1FU)
#define QuadSPI_RBCT_WMRK_SHIFT                  (0U)
#define QuadSPI_RBCT_WMRK(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_RBCT_WMRK_SHIFT)) & QuadSPI_RBCT_WMRK_MASK)

#define QuadSPI_RBCT_RXBRD_MASK                  (0x100U)
#define QuadSPI_RBCT_RXBRD_SHIFT                 (8U)
/*! RXBRD
 *  0b0..RX Buffer content is read using the AHB Bus registers QSPI_ARDB0 to QSPI_ARDB31.
 *  0b1..RX Buffer content is read using the IP Bus registers QSPI_RBDR0 to QSPI_RBDR31.
 */
#define QuadSPI_RBCT_RXBRD(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RBCT_RXBRD_SHIFT)) & QuadSPI_RBCT_RXBRD_MASK)
/*! @} */

/*! @name TBSR - TX Buffer Status Register */
/*! @{ */

#define QuadSPI_TBSR_TRBFL_MASK                  (0x3F00U)
#define QuadSPI_TBSR_TRBFL_SHIFT                 (8U)
#define QuadSPI_TBSR_TRBFL(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_TBSR_TRBFL_SHIFT)) & QuadSPI_TBSR_TRBFL_MASK)

#define QuadSPI_TBSR_TRCTR_MASK                  (0xFFFF0000U)
#define QuadSPI_TBSR_TRCTR_SHIFT                 (16U)
#define QuadSPI_TBSR_TRCTR(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_TBSR_TRCTR_SHIFT)) & QuadSPI_TBSR_TRCTR_MASK)
/*! @} */

/*! @name TBDR - TX Buffer Data Register */
/*! @{ */

#define QuadSPI_TBDR_TXDATA_MASK                 (0xFFFFFFFFU)
#define QuadSPI_TBDR_TXDATA_SHIFT                (0U)
#define QuadSPI_TBDR_TXDATA(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_TBDR_TXDATA_SHIFT)) & QuadSPI_TBDR_TXDATA_MASK)
/*! @} */

/*! @name TBCT - Tx Buffer Control Register */
/*! @{ */

#define QuadSPI_TBCT_WMRK_MASK                   (0x1FU)
#define QuadSPI_TBCT_WMRK_SHIFT                  (0U)
#define QuadSPI_TBCT_WMRK(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_TBCT_WMRK_SHIFT)) & QuadSPI_TBCT_WMRK_MASK)
/*! @} */

/*! @name SR - Status Register */
/*! @{ */

#define QuadSPI_SR_BUSY_MASK                     (0x1U)
#define QuadSPI_SR_BUSY_SHIFT                    (0U)
#define QuadSPI_SR_BUSY(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_BUSY_SHIFT)) & QuadSPI_SR_BUSY_MASK)

#define QuadSPI_SR_IP_ACC_MASK                   (0x2U)
#define QuadSPI_SR_IP_ACC_SHIFT                  (1U)
#define QuadSPI_SR_IP_ACC(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_IP_ACC_SHIFT)) & QuadSPI_SR_IP_ACC_MASK)

#define QuadSPI_SR_AHB_ACC_MASK                  (0x4U)
#define QuadSPI_SR_AHB_ACC_SHIFT                 (2U)
#define QuadSPI_SR_AHB_ACC(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB_ACC_SHIFT)) & QuadSPI_SR_AHB_ACC_MASK)

#define QuadSPI_SR_AHBTRN_MASK                   (0x40U)
#define QuadSPI_SR_AHBTRN_SHIFT                  (6U)
#define QuadSPI_SR_AHBTRN(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHBTRN_SHIFT)) & QuadSPI_SR_AHBTRN_MASK)

#define QuadSPI_SR_AHB0NE_MASK                   (0x80U)
#define QuadSPI_SR_AHB0NE_SHIFT                  (7U)
#define QuadSPI_SR_AHB0NE(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB0NE_SHIFT)) & QuadSPI_SR_AHB0NE_MASK)

#define QuadSPI_SR_AHB1NE_MASK                   (0x100U)
#define QuadSPI_SR_AHB1NE_SHIFT                  (8U)
#define QuadSPI_SR_AHB1NE(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB1NE_SHIFT)) & QuadSPI_SR_AHB1NE_MASK)

#define QuadSPI_SR_AHB2NE_MASK                   (0x200U)
#define QuadSPI_SR_AHB2NE_SHIFT                  (9U)
#define QuadSPI_SR_AHB2NE(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB2NE_SHIFT)) & QuadSPI_SR_AHB2NE_MASK)

#define QuadSPI_SR_AHB3NE_MASK                   (0x400U)
#define QuadSPI_SR_AHB3NE_SHIFT                  (10U)
#define QuadSPI_SR_AHB3NE(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB3NE_SHIFT)) & QuadSPI_SR_AHB3NE_MASK)

#define QuadSPI_SR_AHB0FUL_MASK                  (0x800U)
#define QuadSPI_SR_AHB0FUL_SHIFT                 (11U)
#define QuadSPI_SR_AHB0FUL(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB0FUL_SHIFT)) & QuadSPI_SR_AHB0FUL_MASK)

#define QuadSPI_SR_AHB1FUL_MASK                  (0x1000U)
#define QuadSPI_SR_AHB1FUL_SHIFT                 (12U)
#define QuadSPI_SR_AHB1FUL(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB1FUL_SHIFT)) & QuadSPI_SR_AHB1FUL_MASK)

#define QuadSPI_SR_AHB2FUL_MASK                  (0x2000U)
#define QuadSPI_SR_AHB2FUL_SHIFT                 (13U)
#define QuadSPI_SR_AHB2FUL(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB2FUL_SHIFT)) & QuadSPI_SR_AHB2FUL_MASK)

#define QuadSPI_SR_AHB3FUL_MASK                  (0x4000U)
#define QuadSPI_SR_AHB3FUL_SHIFT                 (14U)
#define QuadSPI_SR_AHB3FUL(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_AHB3FUL_SHIFT)) & QuadSPI_SR_AHB3FUL_MASK)

#define QuadSPI_SR_RXWE_MASK                     (0x10000U)
#define QuadSPI_SR_RXWE_SHIFT                    (16U)
#define QuadSPI_SR_RXWE(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_RXWE_SHIFT)) & QuadSPI_SR_RXWE_MASK)

#define QuadSPI_SR_RXFULL_MASK                   (0x80000U)
#define QuadSPI_SR_RXFULL_SHIFT                  (19U)
#define QuadSPI_SR_RXFULL(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_RXFULL_SHIFT)) & QuadSPI_SR_RXFULL_MASK)

#define QuadSPI_SR_RXDMA_MASK                    (0x800000U)
#define QuadSPI_SR_RXDMA_SHIFT                   (23U)
#define QuadSPI_SR_RXDMA(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_RXDMA_SHIFT)) & QuadSPI_SR_RXDMA_MASK)

#define QuadSPI_SR_TXEDA_MASK                    (0x1000000U)
#define QuadSPI_SR_TXEDA_SHIFT                   (24U)
/*! TXEDA - Tx Buffer Enough Data Available */
#define QuadSPI_SR_TXEDA(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_TXEDA_SHIFT)) & QuadSPI_SR_TXEDA_MASK)

#define QuadSPI_SR_TXWA_MASK                     (0x2000000U)
#define QuadSPI_SR_TXWA_SHIFT                    (25U)
/*! TXWA - TX Buffer watermark Available */
#define QuadSPI_SR_TXWA(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_TXWA_SHIFT)) & QuadSPI_SR_TXWA_MASK)

#define QuadSPI_SR_TXDMA_MASK                    (0x4000000U)
#define QuadSPI_SR_TXDMA_SHIFT                   (26U)
/*! TXDMA - TXDMA */
#define QuadSPI_SR_TXDMA(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_TXDMA_SHIFT)) & QuadSPI_SR_TXDMA_MASK)

#define QuadSPI_SR_TXFULL_MASK                   (0x8000000U)
#define QuadSPI_SR_TXFULL_SHIFT                  (27U)
#define QuadSPI_SR_TXFULL(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_SR_TXFULL_SHIFT)) & QuadSPI_SR_TXFULL_MASK)
/*! @} */

/*! @name FR - Flag Register */
/*! @{ */

#define QuadSPI_FR_TFF_MASK                      (0x1U)
#define QuadSPI_FR_TFF_SHIFT                     (0U)
#define QuadSPI_FR_TFF(x)                        (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_TFF_SHIFT)) & QuadSPI_FR_TFF_MASK)

#define QuadSPI_FR_IPIEF_MASK                    (0x40U)
#define QuadSPI_FR_IPIEF_SHIFT                   (6U)
#define QuadSPI_FR_IPIEF(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_IPIEF_SHIFT)) & QuadSPI_FR_IPIEF_MASK)

#define QuadSPI_FR_IPAEF_MASK                    (0x80U)
#define QuadSPI_FR_IPAEF_SHIFT                   (7U)
#define QuadSPI_FR_IPAEF(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_IPAEF_SHIFT)) & QuadSPI_FR_IPAEF_MASK)

#define QuadSPI_FR_ABOF_MASK                     (0x1000U)
#define QuadSPI_FR_ABOF_SHIFT                    (12U)
#define QuadSPI_FR_ABOF(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_ABOF_SHIFT)) & QuadSPI_FR_ABOF_MASK)

#define QuadSPI_FR_AIBSEF_MASK                   (0x2000U)
#define QuadSPI_FR_AIBSEF_SHIFT                  (13U)
#define QuadSPI_FR_AIBSEF(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_AIBSEF_SHIFT)) & QuadSPI_FR_AIBSEF_MASK)

#define QuadSPI_FR_AITEF_MASK                    (0x4000U)
#define QuadSPI_FR_AITEF_SHIFT                   (14U)
#define QuadSPI_FR_AITEF(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_AITEF_SHIFT)) & QuadSPI_FR_AITEF_MASK)

#define QuadSPI_FR_ABSEF_MASK                    (0x8000U)
#define QuadSPI_FR_ABSEF_SHIFT                   (15U)
#define QuadSPI_FR_ABSEF(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_ABSEF_SHIFT)) & QuadSPI_FR_ABSEF_MASK)

#define QuadSPI_FR_RBDF_MASK                     (0x10000U)
#define QuadSPI_FR_RBDF_SHIFT                    (16U)
#define QuadSPI_FR_RBDF(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_RBDF_SHIFT)) & QuadSPI_FR_RBDF_MASK)

#define QuadSPI_FR_RBOF_MASK                     (0x20000U)
#define QuadSPI_FR_RBOF_SHIFT                    (17U)
#define QuadSPI_FR_RBOF(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_RBOF_SHIFT)) & QuadSPI_FR_RBOF_MASK)

#define QuadSPI_FR_ILLINE_MASK                   (0x800000U)
#define QuadSPI_FR_ILLINE_SHIFT                  (23U)
#define QuadSPI_FR_ILLINE(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_ILLINE_SHIFT)) & QuadSPI_FR_ILLINE_MASK)

#define QuadSPI_FR_TBUF_MASK                     (0x4000000U)
#define QuadSPI_FR_TBUF_SHIFT                    (26U)
#define QuadSPI_FR_TBUF(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_TBUF_SHIFT)) & QuadSPI_FR_TBUF_MASK)

#define QuadSPI_FR_TBFF_MASK                     (0x8000000U)
#define QuadSPI_FR_TBFF_SHIFT                    (27U)
#define QuadSPI_FR_TBFF(x)                       (((uint32_t)(((uint32_t)(x)) << QuadSPI_FR_TBFF_SHIFT)) & QuadSPI_FR_TBFF_MASK)
/*! @} */

/*! @name RSER - Interrupt and DMA Request Select and Enable Register */
/*! @{ */

#define QuadSPI_RSER_TFIE_MASK                   (0x1U)
#define QuadSPI_RSER_TFIE_SHIFT                  (0U)
/*! TFIE
 *  0b0..No TFF interrupt will be generated
 *  0b1..TFF interrupt will be generated
 */
#define QuadSPI_RSER_TFIE(x)                     (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_TFIE_SHIFT)) & QuadSPI_RSER_TFIE_MASK)

#define QuadSPI_RSER_IPIEIE_MASK                 (0x40U)
#define QuadSPI_RSER_IPIEIE_SHIFT                (6U)
/*! IPIEIE
 *  0b0..No IPIEF interrupt will be generated
 *  0b1..IPIEF interrupt will be generated
 */
#define QuadSPI_RSER_IPIEIE(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_IPIEIE_SHIFT)) & QuadSPI_RSER_IPIEIE_MASK)

#define QuadSPI_RSER_IPAEIE_MASK                 (0x80U)
#define QuadSPI_RSER_IPAEIE_SHIFT                (7U)
/*! IPAEIE
 *  0b0..No IPAEF interrupt will be generated
 *  0b1..IPAEF interrupt will be generated
 */
#define QuadSPI_RSER_IPAEIE(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_IPAEIE_SHIFT)) & QuadSPI_RSER_IPAEIE_MASK)

#define QuadSPI_RSER_ABOIE_MASK                  (0x1000U)
#define QuadSPI_RSER_ABOIE_SHIFT                 (12U)
/*! ABOIE
 *  0b0..No ABOF interrupt will be generated
 *  0b1..ABOF interrupt will be generated
 */
#define QuadSPI_RSER_ABOIE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_ABOIE_SHIFT)) & QuadSPI_RSER_ABOIE_MASK)

#define QuadSPI_RSER_AIBSIE_MASK                 (0x2000U)
#define QuadSPI_RSER_AIBSIE_SHIFT                (13U)
/*! AIBSIE
 *  0b0..No AIBSEF interrupt will be generated
 *  0b1..AIBSEF interrupt will be generated
 */
#define QuadSPI_RSER_AIBSIE(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_AIBSIE_SHIFT)) & QuadSPI_RSER_AIBSIE_MASK)

#define QuadSPI_RSER_AITIE_MASK                  (0x4000U)
#define QuadSPI_RSER_AITIE_SHIFT                 (14U)
/*! AITIE
 *  0b0..No AITEF interrupt will be generated
 *  0b1..AITEF interrupt will be generated
 */
#define QuadSPI_RSER_AITIE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_AITIE_SHIFT)) & QuadSPI_RSER_AITIE_MASK)

#define QuadSPI_RSER_ABSEIE_MASK                 (0x8000U)
#define QuadSPI_RSER_ABSEIE_SHIFT                (15U)
/*! ABSEIE
 *  0b0..No ABSEF interrupt will be generated
 *  0b1..ABSEF interrupt will be generated
 */
#define QuadSPI_RSER_ABSEIE(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_ABSEIE_SHIFT)) & QuadSPI_RSER_ABSEIE_MASK)

#define QuadSPI_RSER_RBDIE_MASK                  (0x10000U)
#define QuadSPI_RSER_RBDIE_SHIFT                 (16U)
/*! RBDIE
 *  0b0..No RBDF interrupt will be generated
 *  0b1..RBDF Interrupt will be generated
 */
#define QuadSPI_RSER_RBDIE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_RBDIE_SHIFT)) & QuadSPI_RSER_RBDIE_MASK)

#define QuadSPI_RSER_RBOIE_MASK                  (0x20000U)
#define QuadSPI_RSER_RBOIE_SHIFT                 (17U)
/*! RBOIE
 *  0b0..No RBOF interrupt will be generated
 *  0b1..RBOF interrupt will be generated
 */
#define QuadSPI_RSER_RBOIE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_RBOIE_SHIFT)) & QuadSPI_RSER_RBOIE_MASK)

#define QuadSPI_RSER_RBDDE_MASK                  (0x200000U)
#define QuadSPI_RSER_RBDDE_SHIFT                 (21U)
/*! RBDDE
 *  0b0..No DMA request will be generated
 *  0b1..DMA request will be generated
 */
#define QuadSPI_RSER_RBDDE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_RBDDE_SHIFT)) & QuadSPI_RSER_RBDDE_MASK)

#define QuadSPI_RSER_ILLINIE_MASK                (0x800000U)
#define QuadSPI_RSER_ILLINIE_SHIFT               (23U)
/*! ILLINIE
 *  0b0..No ILLINE interrupt will be generated
 *  0b1..ILLINE interrupt will be generated
 */
#define QuadSPI_RSER_ILLINIE(x)                  (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_ILLINIE_SHIFT)) & QuadSPI_RSER_ILLINIE_MASK)

#define QuadSPI_RSER_TBFDE_MASK                  (0x2000000U)
#define QuadSPI_RSER_TBFDE_SHIFT                 (25U)
/*! TBFDE - TX Buffer Fill DMA Enable
 *  0b0..No DMA request will be generated
 *  0b1..DMA request will be generated
 */
#define QuadSPI_RSER_TBFDE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_TBFDE_SHIFT)) & QuadSPI_RSER_TBFDE_MASK)

#define QuadSPI_RSER_TBUIE_MASK                  (0x4000000U)
#define QuadSPI_RSER_TBUIE_SHIFT                 (26U)
/*! TBUIE
 *  0b0..No TBUF interrupt will be generated
 *  0b1..TBUF interrupt will be generated
 */
#define QuadSPI_RSER_TBUIE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_TBUIE_SHIFT)) & QuadSPI_RSER_TBUIE_MASK)

#define QuadSPI_RSER_TBFIE_MASK                  (0x8000000U)
#define QuadSPI_RSER_TBFIE_SHIFT                 (27U)
/*! TBFIE
 *  0b0..No TBFF interrupt will be generated
 *  0b1..TBFF interrupt will be generated
 */
#define QuadSPI_RSER_TBFIE(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_RSER_TBFIE_SHIFT)) & QuadSPI_RSER_TBFIE_MASK)
/*! @} */

/*! @name SPNDST - Sequence Suspend Status Register */
/*! @{ */

#define QuadSPI_SPNDST_SUSPND_MASK               (0x1U)
#define QuadSPI_SPNDST_SUSPND_SHIFT              (0U)
#define QuadSPI_SPNDST_SUSPND(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SPNDST_SUSPND_SHIFT)) & QuadSPI_SPNDST_SUSPND_MASK)

#define QuadSPI_SPNDST_SPDBUF_MASK               (0xC0U)
#define QuadSPI_SPNDST_SPDBUF_SHIFT              (6U)
#define QuadSPI_SPNDST_SPDBUF(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SPNDST_SPDBUF_SHIFT)) & QuadSPI_SPNDST_SPDBUF_MASK)

#define QuadSPI_SPNDST_DATLFT_MASK               (0xFE00U)
#define QuadSPI_SPNDST_DATLFT_SHIFT              (9U)
#define QuadSPI_SPNDST_DATLFT(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SPNDST_DATLFT_SHIFT)) & QuadSPI_SPNDST_DATLFT_MASK)
/*! @} */

/*! @name SPTRCLR - Sequence Pointer Clear Register */
/*! @{ */

#define QuadSPI_SPTRCLR_BFPTRC_MASK              (0x1U)
#define QuadSPI_SPTRCLR_BFPTRC_SHIFT             (0U)
#define QuadSPI_SPTRCLR_BFPTRC(x)                (((uint32_t)(((uint32_t)(x)) << QuadSPI_SPTRCLR_BFPTRC_SHIFT)) & QuadSPI_SPTRCLR_BFPTRC_MASK)

#define QuadSPI_SPTRCLR_IPPTRC_MASK              (0x100U)
#define QuadSPI_SPTRCLR_IPPTRC_SHIFT             (8U)
#define QuadSPI_SPTRCLR_IPPTRC(x)                (((uint32_t)(((uint32_t)(x)) << QuadSPI_SPTRCLR_IPPTRC_SHIFT)) & QuadSPI_SPTRCLR_IPPTRC_MASK)
/*! @} */

/*! @name SFA1AD - Serial Flash A1 Top Address */
/*! @{ */

#define QuadSPI_SFA1AD_TPADA1_MASK               (0xFFFFFC00U)
#define QuadSPI_SFA1AD_TPADA1_SHIFT              (10U)
#define QuadSPI_SFA1AD_TPADA1(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SFA1AD_TPADA1_SHIFT)) & QuadSPI_SFA1AD_TPADA1_MASK)
/*! @} */

/*! @name SFA2AD - Serial Flash A2 Top Address */
/*! @{ */

#define QuadSPI_SFA2AD_TPADA2_MASK               (0xFFFFFC00U)
#define QuadSPI_SFA2AD_TPADA2_SHIFT              (10U)
#define QuadSPI_SFA2AD_TPADA2(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SFA2AD_TPADA2_SHIFT)) & QuadSPI_SFA2AD_TPADA2_MASK)
/*! @} */

/*! @name SFB1AD - Serial Flash B1 Top Address */
/*! @{ */

#define QuadSPI_SFB1AD_TPADB1_MASK               (0xFFFFFC00U)
#define QuadSPI_SFB1AD_TPADB1_SHIFT              (10U)
#define QuadSPI_SFB1AD_TPADB1(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SFB1AD_TPADB1_SHIFT)) & QuadSPI_SFB1AD_TPADB1_MASK)
/*! @} */

/*! @name SFB2AD - Serial Flash B2 Top Address */
/*! @{ */

#define QuadSPI_SFB2AD_TPADB2_MASK               (0xFFFFFC00U)
#define QuadSPI_SFB2AD_TPADB2_SHIFT              (10U)
#define QuadSPI_SFB2AD_TPADB2(x)                 (((uint32_t)(((uint32_t)(x)) << QuadSPI_SFB2AD_TPADB2_SHIFT)) & QuadSPI_SFB2AD_TPADB2_MASK)
/*! @} */

/*! @name RBDR - RX Buffer Data Register */
/*! @{ */

#define QuadSPI_RBDR_RXDATA_MASK                 (0xFFFFFFFFU)
#define QuadSPI_RBDR_RXDATA_SHIFT                (0U)
#define QuadSPI_RBDR_RXDATA(x)                   (((uint32_t)(((uint32_t)(x)) << QuadSPI_RBDR_RXDATA_SHIFT)) & QuadSPI_RBDR_RXDATA_MASK)
/*! @} */

/*! @name LUTKEY - LUT Key Register */
/*! @{ */

#define QuadSPI_LUTKEY_KEY_MASK                  (0xFFFFFFFFU)
#define QuadSPI_LUTKEY_KEY_SHIFT                 (0U)
#define QuadSPI_LUTKEY_KEY(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_LUTKEY_KEY_SHIFT)) & QuadSPI_LUTKEY_KEY_MASK)
/*! @} */

/*! @name LCKCR - LUT Lock Configuration Register */
/*! @{ */

#define QuadSPI_LCKCR_LOCK_MASK                  (0x1U)
#define QuadSPI_LCKCR_LOCK_SHIFT                 (0U)
#define QuadSPI_LCKCR_LOCK(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_LCKCR_LOCK_SHIFT)) & QuadSPI_LCKCR_LOCK_MASK)

#define QuadSPI_LCKCR_UNLOCK_MASK                (0x2U)
#define QuadSPI_LCKCR_UNLOCK_SHIFT               (1U)
#define QuadSPI_LCKCR_UNLOCK(x)                  (((uint32_t)(((uint32_t)(x)) << QuadSPI_LCKCR_UNLOCK_SHIFT)) & QuadSPI_LCKCR_UNLOCK_MASK)
/*! @} */

/*! @name LUT - Look-up Table register */
/*! @{ */

#define QuadSPI_LUT_OPRND0_MASK                  (0xFFU)
#define QuadSPI_LUT_OPRND0_SHIFT                 (0U)
/*! OPRND0 - Operand for INSTR0. */
#define QuadSPI_LUT_OPRND0(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_LUT_OPRND0_SHIFT)) & QuadSPI_LUT_OPRND0_MASK)

#define QuadSPI_LUT_PAD0_MASK                    (0x300U)
#define QuadSPI_LUT_PAD0_SHIFT                   (8U)
/*! PAD0 - Pad information for INSTR0.
 *  0b00..1 Pad
 *  0b01..2 Pads
 *  0b10..4 Pads
 *  0b11..8 Pads
 */
#define QuadSPI_LUT_PAD0(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_LUT_PAD0_SHIFT)) & QuadSPI_LUT_PAD0_MASK)

#define QuadSPI_LUT_INSTR0_MASK                  (0xFC00U)
#define QuadSPI_LUT_INSTR0_SHIFT                 (10U)
/*! INSTR0 - Instruction 0 */
#define QuadSPI_LUT_INSTR0(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_LUT_INSTR0_SHIFT)) & QuadSPI_LUT_INSTR0_MASK)

#define QuadSPI_LUT_OPRND1_MASK                  (0xFF0000U)
#define QuadSPI_LUT_OPRND1_SHIFT                 (16U)
/*! OPRND1 - Operand for INSTR1. */
#define QuadSPI_LUT_OPRND1(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_LUT_OPRND1_SHIFT)) & QuadSPI_LUT_OPRND1_MASK)

#define QuadSPI_LUT_PAD1_MASK                    (0x3000000U)
#define QuadSPI_LUT_PAD1_SHIFT                   (24U)
/*! PAD1 - Pad information for INSTR1.
 *  0b00..1 Pad
 *  0b01..2 Pads
 *  0b10..4 Pads
 *  0b11..8 Pads
 */
#define QuadSPI_LUT_PAD1(x)                      (((uint32_t)(((uint32_t)(x)) << QuadSPI_LUT_PAD1_SHIFT)) & QuadSPI_LUT_PAD1_MASK)

#define QuadSPI_LUT_INSTR1_MASK                  (0xFC000000U)
#define QuadSPI_LUT_INSTR1_SHIFT                 (26U)
/*! INSTR1 - Instruction 1 */
#define QuadSPI_LUT_INSTR1(x)                    (((uint32_t)(((uint32_t)(x)) << QuadSPI_LUT_INSTR1_SHIFT)) & QuadSPI_LUT_INSTR1_MASK)
/*! @} */


/*!
 * @}
 */ /* end of group QuadSPI_Register_Masks */


/*!
 * @}
 */ /* end of group QuadSPI_Peripheral_Access_Layer */


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


#endif  /* PERI_QUADSPI_H_ */


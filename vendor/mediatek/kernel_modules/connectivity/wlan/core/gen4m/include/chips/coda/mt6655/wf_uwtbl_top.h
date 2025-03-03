/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

//[File]            : wf_uwtbl_top.h
//[Revision time]   : Fri Mar 25 09:45:29 2022
//[Description]     : This file is auto generated by CODA
//[Copyright]       : Copyright (C) 2022 Mediatek Incorportion. All rights reserved.

#ifndef __WF_UWTBL_TOP_REGS_H__
#define __WF_UWTBL_TOP_REGS_H__

#include "hal_common.h"

#ifdef __cplusplus
extern "C" {
#endif


//****************************************************************************
//
//                     WF_UWTBL_TOP CR Definitions                     
//
//****************************************************************************

#define WF_UWTBL_TOP_BASE                                      0x820c4000

#define WF_UWTBL_TOP_KTVLBR0_ADDR                              (WF_UWTBL_TOP_BASE + 0x0000) // 4000
#define WF_UWTBL_TOP_UCR_ADDR                                  (WF_UWTBL_TOP_BASE + 0x0100) // 4100
#define WF_UWTBL_TOP_WDUCR_ADDR                                (WF_UWTBL_TOP_BASE + 0x0104) // 4104
#define WF_UWTBL_TOP_KTCR_ADDR                                 (WF_UWTBL_TOP_BASE + 0x0108) // 4108
#define WF_UWTBL_TOP_WIUCR_ADDR                                (WF_UWTBL_TOP_BASE + 0x0110) // 4110
#define WF_UWTBL_TOP_WMUDR_ADDR                                (WF_UWTBL_TOP_BASE + 0x0114) // 4114
#define WF_UWTBL_TOP_WMUMR_ADDR                                (WF_UWTBL_TOP_BASE + 0x0118) // 4118
#define WF_UWTBL_TOP_PICR0_ADDR                                (WF_UWTBL_TOP_BASE + 0x0120) // 4120
#define WF_UWTBL_TOP_PICR1_ADDR                                (WF_UWTBL_TOP_BASE + 0x0124) // 4124
#define WF_UWTBL_TOP_ITCR_ADDR                                 (WF_UWTBL_TOP_BASE + 0x0130) // 4130
#define WF_UWTBL_TOP_ITDR0_ADDR                                (WF_UWTBL_TOP_BASE + 0x0138) // 4138
#define WF_UWTBL_TOP_ITDR1_ADDR                                (WF_UWTBL_TOP_BASE + 0x013C) // 413C
#define WF_UWTBL_TOP_PSCCR_ADDR                                (WF_UWTBL_TOP_BASE + 0x0140) // 4140
#define WF_UWTBL_TOP_DSCR00_ADDR                               (WF_UWTBL_TOP_BASE + 0x0200) // 4200
#define WF_UWTBL_TOP_DSCR01_ADDR                               (WF_UWTBL_TOP_BASE + 0x0204) // 4204
#define WF_UWTBL_TOP_DSCR02_ADDR                               (WF_UWTBL_TOP_BASE + 0x0208) // 4208
#define WF_UWTBL_TOP_DSCR03_ADDR                               (WF_UWTBL_TOP_BASE + 0x020C) // 420C
#define WF_UWTBL_TOP_DSCR_EXT_00_ADDR                          (WF_UWTBL_TOP_BASE + 0x0280) // 4280
#define WF_UWTBL_TOP_SRCR00_ADDR                               (WF_UWTBL_TOP_BASE + 0x0300) // 4300
#define WF_UWTBL_TOP_SRCR01_ADDR                               (WF_UWTBL_TOP_BASE + 0x0304) // 4304
#define WF_UWTBL_TOP_SRCR02_ADDR                               (WF_UWTBL_TOP_BASE + 0x0308) // 4308
#define WF_UWTBL_TOP_SRCR03_ADDR                               (WF_UWTBL_TOP_BASE + 0x030C) // 430C
#define WF_UWTBL_TOP_SRCR_EXT_00_ADDR                          (WF_UWTBL_TOP_BASE + 0x0380) // 4380
#define WF_UWTBL_TOP_NSEPPCR00_ADDR                            (WF_UWTBL_TOP_BASE + 0x0400) // 4400
#define WF_UWTBL_TOP_NSEPPCR01_ADDR                            (WF_UWTBL_TOP_BASE + 0x0404) // 4404
#define WF_UWTBL_TOP_NSEPPCR02_ADDR                            (WF_UWTBL_TOP_BASE + 0x0408) // 4408
#define WF_UWTBL_TOP_NSEPPCR03_ADDR                            (WF_UWTBL_TOP_BASE + 0x040C) // 440C
#define WF_UWTBL_TOP_NSEPPCR_EXT_00_ADDR                       (WF_UWTBL_TOP_BASE + 0x0480) // 4480
#define WF_UWTBL_TOP_DFR_ADDR                                  (WF_UWTBL_TOP_BASE + 0x1FD0) // 5FD0
#define WF_UWTBL_TOP_WTBLSNO_ADDR                              (WF_UWTBL_TOP_BASE + 0x1FE0) // 5FE0
#define WF_UWTBL_TOP_WTBLENO_ADDR                              (WF_UWTBL_TOP_BASE + 0x1FE4) // 5FE4
#define WF_UWTBL_TOP_KTBLNO_ADDR                               (WF_UWTBL_TOP_BASE + 0x1FE8) // 5FE8
#define WF_UWTBL_TOP_DMY0_ADDR                                 (WF_UWTBL_TOP_BASE + 0x1FF0) // 5FF0
#define WF_UWTBL_TOP_DMY1_ADDR                                 (WF_UWTBL_TOP_BASE + 0x1FF4) // 5FF4
#define WF_UWTBL_TOP_WTBL_BASE_ADDR                            (WF_UWTBL_TOP_BASE + 0x1FFC) // 5FFC




/* =====================================================================================

  ---KTVLBR0 (0x820c4000 + 0x0000)---

    VLB[31..0]                   - (RW) Valid Lookaside Buffer for key table
                                     Each bit is mapped to 1 entry in key table
                                     The max # = # of wlan_entry + (# of MBSS+ # of BSS) * (# of band)

 =====================================================================================*/
#define WF_UWTBL_TOP_KTVLBR0_VLB_ADDR                          WF_UWTBL_TOP_KTVLBR0_ADDR
#define WF_UWTBL_TOP_KTVLBR0_VLB_MASK                          0xFFFFFFFF                // VLB[31..0]
#define WF_UWTBL_TOP_KTVLBR0_VLB_SHFT                          0

/* =====================================================================================

  ---UCR (0x820c4000 + 0x0100)---

    PN_INCR_MODE[0]              - (RW) The condition of PN+1
    RESERVED1[15..1]             - (RO) Reserved bits
    MAX_BIPN_RANGE[31..16]       - (RW) Max BIPN range
                                     Replay check is checked by the following condition
                                     oldBIPN < newBIPN <= oldBIPN+ MAX_BIPN_RANGE

 =====================================================================================*/
#define WF_UWTBL_TOP_UCR_MAX_BIPN_RANGE_ADDR                   WF_UWTBL_TOP_UCR_ADDR
#define WF_UWTBL_TOP_UCR_MAX_BIPN_RANGE_MASK                   0xFFFF0000                // MAX_BIPN_RANGE[31..16]
#define WF_UWTBL_TOP_UCR_MAX_BIPN_RANGE_SHFT                   16
#define WF_UWTBL_TOP_UCR_PN_INCR_MODE_ADDR                     WF_UWTBL_TOP_UCR_ADDR
#define WF_UWTBL_TOP_UCR_PN_INCR_MODE_MASK                     0x00000001                // PN_INCR_MODE[0]
#define WF_UWTBL_TOP_UCR_PN_INCR_MODE_SHFT                     0

/* =====================================================================================

  ---WDUCR (0x820c4000 + 0x0104)---

    GROUP[5..0]                  - (RW) The selected group of key table/uwtbl
                                     128 entries for each group
    RESERVED6[30..6]             - (RO) Reserved bits
    TARGET[31]                   - (RW) select the target of table

 =====================================================================================*/
#define WF_UWTBL_TOP_WDUCR_TARGET_ADDR                         WF_UWTBL_TOP_WDUCR_ADDR
#define WF_UWTBL_TOP_WDUCR_TARGET_MASK                         0x80000000                // TARGET[31]
#define WF_UWTBL_TOP_WDUCR_TARGET_SHFT                         31
#define WF_UWTBL_TOP_WDUCR_GROUP_ADDR                          WF_UWTBL_TOP_WDUCR_ADDR
#define WF_UWTBL_TOP_WDUCR_GROUP_MASK                          0x0000003F                // GROUP[5..0]
#define WF_UWTBL_TOP_WDUCR_GROUP_SHFT                          0

/* =====================================================================================

  ---KTCR (0x820c4000 + 0x0108)---

    INDEX[12..0]                 - (RW) Target index if write command of KTCR is performed
                                     if OPERATION = search, the free key entry is shown in this field
    RESERVED13[13]               - (RO) Reserved bits
    OPERATION[15..14]            - (RW) The operation if write command of KTCR is performed
    RESERVED16[29..16]           - (RO) Reserved bits
    REASON[31..30]               - (RO) The reason code for the operation of KTCR

 =====================================================================================*/
#define WF_UWTBL_TOP_KTCR_REASON_ADDR                          WF_UWTBL_TOP_KTCR_ADDR
#define WF_UWTBL_TOP_KTCR_REASON_MASK                          0xC0000000                // REASON[31..30]
#define WF_UWTBL_TOP_KTCR_REASON_SHFT                          30
#define WF_UWTBL_TOP_KTCR_OPERATION_ADDR                       WF_UWTBL_TOP_KTCR_ADDR
#define WF_UWTBL_TOP_KTCR_OPERATION_MASK                       0x0000C000                // OPERATION[15..14]
#define WF_UWTBL_TOP_KTCR_OPERATION_SHFT                       14
#define WF_UWTBL_TOP_KTCR_INDEX_ADDR                           WF_UWTBL_TOP_KTCR_ADDR
#define WF_UWTBL_TOP_KTCR_INDEX_MASK                           0x00001FFF                // INDEX[12..0]
#define WF_UWTBL_TOP_KTCR_INDEX_SHFT                           0

/* =====================================================================================

  ---WIUCR (0x820c4000 + 0x0110)---

    INDEX[11..0]                 - (RW) Target index for indirect update
    RESERVED12[12]               - (RO) Reserved bits
    PEERINFO_UPDATE[13]          - (W1) Update peer info according to the value of WIUDR0n
    RESERVED14[17..14]           - (RO) Reserved bits
    PNSN_CLEAR[18]               - (W1) Clear PN/SN* to 0
    RESERVED19[19]               - (RO) Reserved bits
    MASK_UPDATE[20]              - (W1) Mask Update
                                     WTBL loads target wlan_idx & dw and update the target field by WMUDR & WMUMR
    RESERVED21[23..21]           - (RO) Reserved bits
    DW[27..24]                   - (RW) Target DW for indirect update
    RESERVED28[30..28]           - (RO) Reserved bits
    IU_BUSY[31]                  - (RO) Indirect update status
                                     HW will set up this bit when it is updating WTBL.

 =====================================================================================*/
#define WF_UWTBL_TOP_WIUCR_IU_BUSY_ADDR                        WF_UWTBL_TOP_WIUCR_ADDR
#define WF_UWTBL_TOP_WIUCR_IU_BUSY_MASK                        0x80000000                // IU_BUSY[31]
#define WF_UWTBL_TOP_WIUCR_IU_BUSY_SHFT                        31
#define WF_UWTBL_TOP_WIUCR_DW_ADDR                             WF_UWTBL_TOP_WIUCR_ADDR
#define WF_UWTBL_TOP_WIUCR_DW_MASK                             0x0F000000                // DW[27..24]
#define WF_UWTBL_TOP_WIUCR_DW_SHFT                             24
#define WF_UWTBL_TOP_WIUCR_MASK_UPDATE_ADDR                    WF_UWTBL_TOP_WIUCR_ADDR
#define WF_UWTBL_TOP_WIUCR_MASK_UPDATE_MASK                    0x00100000                // MASK_UPDATE[20]
#define WF_UWTBL_TOP_WIUCR_MASK_UPDATE_SHFT                    20
#define WF_UWTBL_TOP_WIUCR_PNSN_CLEAR_ADDR                     WF_UWTBL_TOP_WIUCR_ADDR
#define WF_UWTBL_TOP_WIUCR_PNSN_CLEAR_MASK                     0x00040000                // PNSN_CLEAR[18]
#define WF_UWTBL_TOP_WIUCR_PNSN_CLEAR_SHFT                     18
#define WF_UWTBL_TOP_WIUCR_PEERINFO_UPDATE_ADDR                WF_UWTBL_TOP_WIUCR_ADDR
#define WF_UWTBL_TOP_WIUCR_PEERINFO_UPDATE_MASK                0x00002000                // PEERINFO_UPDATE[13]
#define WF_UWTBL_TOP_WIUCR_PEERINFO_UPDATE_SHFT                13
#define WF_UWTBL_TOP_WIUCR_INDEX_ADDR                          WF_UWTBL_TOP_WIUCR_ADDR
#define WF_UWTBL_TOP_WIUCR_INDEX_MASK                          0x00000FFF                // INDEX[11..0]
#define WF_UWTBL_TOP_WIUCR_INDEX_SHFT                          0

/* =====================================================================================

  ---WMUDR (0x820c4000 + 0x0114)---

    UPDATE_DATA[31..0]           - (RW) Data to update wlan entry

 =====================================================================================*/
#define WF_UWTBL_TOP_WMUDR_UPDATE_DATA_ADDR                    WF_UWTBL_TOP_WMUDR_ADDR
#define WF_UWTBL_TOP_WMUDR_UPDATE_DATA_MASK                    0xFFFFFFFF                // UPDATE_DATA[31..0]
#define WF_UWTBL_TOP_WMUDR_UPDATE_DATA_SHFT                    0

/* =====================================================================================

  ---WMUMR (0x820c4000 + 0x0118)---

    UPDATE_MASK[31..0]           - (RW) Mask of data to update wlan entry

 =====================================================================================*/
#define WF_UWTBL_TOP_WMUMR_UPDATE_MASK_ADDR                    WF_UWTBL_TOP_WMUMR_ADDR
#define WF_UWTBL_TOP_WMUMR_UPDATE_MASK_MASK                    0xFFFFFFFF                // UPDATE_MASK[31..0]
#define WF_UWTBL_TOP_WMUMR_UPDATE_MASK_SHFT                    0

/* =====================================================================================

  ---PICR0 (0x820c4000 + 0x0120)---

    DW0_WTBL[31..0]              - (RW) The value of DW in WTBL for updating
                                     the data are updated to WTBL when WIUCR.PEERINFO_UPDATE is asserted.

 =====================================================================================*/
#define WF_UWTBL_TOP_PICR0_DW0_WTBL_ADDR                       WF_UWTBL_TOP_PICR0_ADDR
#define WF_UWTBL_TOP_PICR0_DW0_WTBL_MASK                       0xFFFFFFFF                // DW0_WTBL[31..0]
#define WF_UWTBL_TOP_PICR0_DW0_WTBL_SHFT                       0

/* =====================================================================================

  ---PICR1 (0x820c4000 + 0x0124)---

    DW1_WTBL[31..0]              - (RW) The value of DW in WTBL for updating
                                     the data are updated to WTBL when WIUCR.PEERINFO_UPDATE is asserted.

 =====================================================================================*/
#define WF_UWTBL_TOP_PICR1_DW1_WTBL_ADDR                       WF_UWTBL_TOP_PICR1_ADDR
#define WF_UWTBL_TOP_PICR1_DW1_WTBL_MASK                       0xFFFFFFFF                // DW1_WTBL[31..0]
#define WF_UWTBL_TOP_PICR1_DW1_WTBL_SHFT                       0

/* =====================================================================================

  ---ITCR (0x820c4000 + 0x0130)---

    INDEX[5..0]                  - (RW) index
    RESERVED6[15..6]             - (RO) Reserved bits
    OP[16]                       - (RW) Operation
    RESERVED17[23..17]           - (RO) Reserved bits
    SELECT[25..24]               - (RW) indirect table select
    RESERVED26[30..26]           - (RO) Reserved bits
    EXECUTE[31]                  - (WO) execute the command

 =====================================================================================*/
#define WF_UWTBL_TOP_ITCR_EXECUTE_ADDR                         WF_UWTBL_TOP_ITCR_ADDR
#define WF_UWTBL_TOP_ITCR_EXECUTE_MASK                         0x80000000                // EXECUTE[31]
#define WF_UWTBL_TOP_ITCR_EXECUTE_SHFT                         31
#define WF_UWTBL_TOP_ITCR_SELECT_ADDR                          WF_UWTBL_TOP_ITCR_ADDR
#define WF_UWTBL_TOP_ITCR_SELECT_MASK                          0x03000000                // SELECT[25..24]
#define WF_UWTBL_TOP_ITCR_SELECT_SHFT                          24
#define WF_UWTBL_TOP_ITCR_OP_ADDR                              WF_UWTBL_TOP_ITCR_ADDR
#define WF_UWTBL_TOP_ITCR_OP_MASK                              0x00010000                // OP[16]
#define WF_UWTBL_TOP_ITCR_OP_SHFT                              16
#define WF_UWTBL_TOP_ITCR_INDEX_ADDR                           WF_UWTBL_TOP_ITCR_ADDR
#define WF_UWTBL_TOP_ITCR_INDEX_MASK                           0x0000003F                // INDEX[5..0]
#define WF_UWTBL_TOP_ITCR_INDEX_SHFT                           0

/* =====================================================================================

  ---ITDR0 (0x820c4000 + 0x0138)---

    INDIRECT_TABLE_DATA[31..0]   - (RW) data for updating indirect table

 =====================================================================================*/
#define WF_UWTBL_TOP_ITDR0_INDIRECT_TABLE_DATA_ADDR            WF_UWTBL_TOP_ITDR0_ADDR
#define WF_UWTBL_TOP_ITDR0_INDIRECT_TABLE_DATA_MASK            0xFFFFFFFF                // INDIRECT_TABLE_DATA[31..0]
#define WF_UWTBL_TOP_ITDR0_INDIRECT_TABLE_DATA_SHFT            0

/* =====================================================================================

  ---ITDR1 (0x820c4000 + 0x013C)---

    INDIRECT_TABLE_DATA[31..0]   - (RW) data for updating indirect table

 =====================================================================================*/
#define WF_UWTBL_TOP_ITDR1_INDIRECT_TABLE_DATA_ADDR            WF_UWTBL_TOP_ITDR1_ADDR
#define WF_UWTBL_TOP_ITDR1_INDIRECT_TABLE_DATA_MASK            0xFFFFFFFF                // INDIRECT_TABLE_DATA[31..0]
#define WF_UWTBL_TOP_ITDR1_INDIRECT_TABLE_DATA_SHFT            0

/* =====================================================================================

  ---PSCCR (0x820c4000 + 0x0140)---

    MLD_ID[11..0]                - (RW) MLD_ID
    RESERVED12[15..12]           - (RO) Reserved bits
    TARGET[18..16]               - (RW) target per-STA config
    RESERVED19[23..19]           - (RO) Reserved bits
    OP[24]                       - (RW) Operation
                                     set or unset the per-STA config for the peer indicating by MLD_ID
    RESERVED25[30..25]           - (RO) Reserved bits
    EXECUTE[31]                  - (W1) MLD_ID is executed when this bit is set
                                     (No need to poll the busy event)

 =====================================================================================*/
#define WF_UWTBL_TOP_PSCCR_EXECUTE_ADDR                        WF_UWTBL_TOP_PSCCR_ADDR
#define WF_UWTBL_TOP_PSCCR_EXECUTE_MASK                        0x80000000                // EXECUTE[31]
#define WF_UWTBL_TOP_PSCCR_EXECUTE_SHFT                        31
#define WF_UWTBL_TOP_PSCCR_OP_ADDR                             WF_UWTBL_TOP_PSCCR_ADDR
#define WF_UWTBL_TOP_PSCCR_OP_MASK                             0x01000000                // OP[24]
#define WF_UWTBL_TOP_PSCCR_OP_SHFT                             24
#define WF_UWTBL_TOP_PSCCR_TARGET_ADDR                         WF_UWTBL_TOP_PSCCR_ADDR
#define WF_UWTBL_TOP_PSCCR_TARGET_MASK                         0x00070000                // TARGET[18..16]
#define WF_UWTBL_TOP_PSCCR_TARGET_SHFT                         16
#define WF_UWTBL_TOP_PSCCR_MLD_ID_ADDR                         WF_UWTBL_TOP_PSCCR_ADDR
#define WF_UWTBL_TOP_PSCCR_MLD_ID_MASK                         0x00000FFF                // MLD_ID[11..0]
#define WF_UWTBL_TOP_PSCCR_MLD_ID_SHFT                         0

/* =====================================================================================

  ---DSCR00 (0x820c4000 + 0x0200)---

    DSCR00[7..0]                 - (RW) Each bit control disable one station function.
                                     SW write one to this CR bits to let HW disable the station TX.
                                     If wlan_idx > 32, DSCR+0x4*i is disable station control for wlan_idx 32*i ~32*(i+1)-1
    DSCR00_01[15..8]             - (RW) The same as DSCR00
    DSCR00_02[31..16]            - (RW) The same as DSCR00

 =====================================================================================*/
#define WF_UWTBL_TOP_DSCR00_DSCR00_02_ADDR                     WF_UWTBL_TOP_DSCR00_ADDR
#define WF_UWTBL_TOP_DSCR00_DSCR00_02_MASK                     0xFFFF0000                // DSCR00_02[31..16]
#define WF_UWTBL_TOP_DSCR00_DSCR00_02_SHFT                     16
#define WF_UWTBL_TOP_DSCR00_DSCR00_01_ADDR                     WF_UWTBL_TOP_DSCR00_ADDR
#define WF_UWTBL_TOP_DSCR00_DSCR00_01_MASK                     0x0000FF00                // DSCR00_01[15..8]
#define WF_UWTBL_TOP_DSCR00_DSCR00_01_SHFT                     8
#define WF_UWTBL_TOP_DSCR00_DSCR00_ADDR                        WF_UWTBL_TOP_DSCR00_ADDR
#define WF_UWTBL_TOP_DSCR00_DSCR00_MASK                        0x000000FF                // DSCR00[7..0]
#define WF_UWTBL_TOP_DSCR00_DSCR00_SHFT                        0

/* =====================================================================================

  ---DSCR01 (0x820c4000 + 0x0204)---

    DSCR01[31..0]                - (RW) The same as DSCR00

 =====================================================================================*/
#define WF_UWTBL_TOP_DSCR01_DSCR01_ADDR                        WF_UWTBL_TOP_DSCR01_ADDR
#define WF_UWTBL_TOP_DSCR01_DSCR01_MASK                        0xFFFFFFFF                // DSCR01[31..0]
#define WF_UWTBL_TOP_DSCR01_DSCR01_SHFT                        0

/* =====================================================================================

  ---DSCR02 (0x820c4000 + 0x0208)---

    DSCR02[31..0]                - (RW) The same as DSCR00

 =====================================================================================*/
#define WF_UWTBL_TOP_DSCR02_DSCR02_ADDR                        WF_UWTBL_TOP_DSCR02_ADDR
#define WF_UWTBL_TOP_DSCR02_DSCR02_MASK                        0xFFFFFFFF                // DSCR02[31..0]
#define WF_UWTBL_TOP_DSCR02_DSCR02_SHFT                        0

/* =====================================================================================

  ---DSCR03 (0x820c4000 + 0x020C)---

    DSCR03[31..0]                - (RW) The same as DSCR00

 =====================================================================================*/
#define WF_UWTBL_TOP_DSCR03_DSCR03_ADDR                        WF_UWTBL_TOP_DSCR03_ADDR
#define WF_UWTBL_TOP_DSCR03_DSCR03_MASK                        0xFFFFFFFF                // DSCR03[31..0]
#define WF_UWTBL_TOP_DSCR03_DSCR03_SHFT                        0

/* =====================================================================================

  ---DSCR_EXT_00 (0x820c4000 + 0x0280)---

    DSCR_EXT_00[3..0]            - (RW) The same as DSCR00
    DSCR_EXT_00_01[7..4]         - (RW) The same as DSCR00
    RESERVED8[31..8]             - (RO) Reserved bits

 =====================================================================================*/
#define WF_UWTBL_TOP_DSCR_EXT_00_DSCR_EXT_00_01_ADDR           WF_UWTBL_TOP_DSCR_EXT_00_ADDR
#define WF_UWTBL_TOP_DSCR_EXT_00_DSCR_EXT_00_01_MASK           0x000000F0                // DSCR_EXT_00_01[7..4]
#define WF_UWTBL_TOP_DSCR_EXT_00_DSCR_EXT_00_01_SHFT           4
#define WF_UWTBL_TOP_DSCR_EXT_00_DSCR_EXT_00_ADDR              WF_UWTBL_TOP_DSCR_EXT_00_ADDR
#define WF_UWTBL_TOP_DSCR_EXT_00_DSCR_EXT_00_MASK              0x0000000F                // DSCR_EXT_00[3..0]
#define WF_UWTBL_TOP_DSCR_EXT_00_DSCR_EXT_00_SHFT              0

/* =====================================================================================

  ---SRCR00 (0x820c4000 + 0x0300)---

    SRCR00[7..0]                 - (RW) Each bit control redirection station TX function.
                                     SW write one to this CR bits to let HW redirection the station TXD.
                                     If wlan_idx > 32, SRCR+0x4*i is station TXD redirection control for wlan_idx 32*i ~32*(i+1)-1
    SRCR00_01[15..8]             - (RW) The same as SRCR00
    SRCR00_02[31..16]            - (RW) The same as SRCR00

 =====================================================================================*/
#define WF_UWTBL_TOP_SRCR00_SRCR00_02_ADDR                     WF_UWTBL_TOP_SRCR00_ADDR
#define WF_UWTBL_TOP_SRCR00_SRCR00_02_MASK                     0xFFFF0000                // SRCR00_02[31..16]
#define WF_UWTBL_TOP_SRCR00_SRCR00_02_SHFT                     16
#define WF_UWTBL_TOP_SRCR00_SRCR00_01_ADDR                     WF_UWTBL_TOP_SRCR00_ADDR
#define WF_UWTBL_TOP_SRCR00_SRCR00_01_MASK                     0x0000FF00                // SRCR00_01[15..8]
#define WF_UWTBL_TOP_SRCR00_SRCR00_01_SHFT                     8
#define WF_UWTBL_TOP_SRCR00_SRCR00_ADDR                        WF_UWTBL_TOP_SRCR00_ADDR
#define WF_UWTBL_TOP_SRCR00_SRCR00_MASK                        0x000000FF                // SRCR00[7..0]
#define WF_UWTBL_TOP_SRCR00_SRCR00_SHFT                        0

/* =====================================================================================

  ---SRCR01 (0x820c4000 + 0x0304)---

    SRCR01[31..0]                - (RW) The same as SRCR00

 =====================================================================================*/
#define WF_UWTBL_TOP_SRCR01_SRCR01_ADDR                        WF_UWTBL_TOP_SRCR01_ADDR
#define WF_UWTBL_TOP_SRCR01_SRCR01_MASK                        0xFFFFFFFF                // SRCR01[31..0]
#define WF_UWTBL_TOP_SRCR01_SRCR01_SHFT                        0

/* =====================================================================================

  ---SRCR02 (0x820c4000 + 0x0308)---

    SRCR02[31..0]                - (RW) The same as SRCR00

 =====================================================================================*/
#define WF_UWTBL_TOP_SRCR02_SRCR02_ADDR                        WF_UWTBL_TOP_SRCR02_ADDR
#define WF_UWTBL_TOP_SRCR02_SRCR02_MASK                        0xFFFFFFFF                // SRCR02[31..0]
#define WF_UWTBL_TOP_SRCR02_SRCR02_SHFT                        0

/* =====================================================================================

  ---SRCR03 (0x820c4000 + 0x030C)---

    SRCR03[31..0]                - (RW) The same as SRCR00

 =====================================================================================*/
#define WF_UWTBL_TOP_SRCR03_SRCR03_ADDR                        WF_UWTBL_TOP_SRCR03_ADDR
#define WF_UWTBL_TOP_SRCR03_SRCR03_MASK                        0xFFFFFFFF                // SRCR03[31..0]
#define WF_UWTBL_TOP_SRCR03_SRCR03_SHFT                        0

/* =====================================================================================

  ---SRCR_EXT_00 (0x820c4000 + 0x0380)---

    SRCR_EXT_00[3..0]            - (RW) The same as SRCR00
    SRCR_EXT_00_01[7..4]         - (RW) The same as SRCR00
    RESERVED8[31..8]             - (RO) Reserved bits

 =====================================================================================*/
#define WF_UWTBL_TOP_SRCR_EXT_00_SRCR_EXT_00_01_ADDR           WF_UWTBL_TOP_SRCR_EXT_00_ADDR
#define WF_UWTBL_TOP_SRCR_EXT_00_SRCR_EXT_00_01_MASK           0x000000F0                // SRCR_EXT_00_01[7..4]
#define WF_UWTBL_TOP_SRCR_EXT_00_SRCR_EXT_00_01_SHFT           4
#define WF_UWTBL_TOP_SRCR_EXT_00_SRCR_EXT_00_ADDR              WF_UWTBL_TOP_SRCR_EXT_00_ADDR
#define WF_UWTBL_TOP_SRCR_EXT_00_SRCR_EXT_00_MASK              0x0000000F                // SRCR_EXT_00[3..0]
#define WF_UWTBL_TOP_SRCR_EXT_00_SRCR_EXT_00_SHFT              0

/* =====================================================================================

  ---NSEPPCR00 (0x820c4000 + 0x0400)---

    NSEPPCR00[7..0]              - (RW) Each bit control NSEP(National Security and Emergency Preparedness) priority for STA.
                                     SW write one to this CR bits to rise the NSEP priority for STA
                                     If wlan_idx > 32, NSEPPCR+0x4*i is NSEP priority control for wlan_idx 32*i ~32*(i+1)-1
    NSEPPCR00_01[15..8]          - (RW) The same as NSEPPCR00
    NSEPPCR00_02[31..16]         - (RW) The same as NSEPPCR00

 =====================================================================================*/
#define WF_UWTBL_TOP_NSEPPCR00_NSEPPCR00_02_ADDR               WF_UWTBL_TOP_NSEPPCR00_ADDR
#define WF_UWTBL_TOP_NSEPPCR00_NSEPPCR00_02_MASK               0xFFFF0000                // NSEPPCR00_02[31..16]
#define WF_UWTBL_TOP_NSEPPCR00_NSEPPCR00_02_SHFT               16
#define WF_UWTBL_TOP_NSEPPCR00_NSEPPCR00_01_ADDR               WF_UWTBL_TOP_NSEPPCR00_ADDR
#define WF_UWTBL_TOP_NSEPPCR00_NSEPPCR00_01_MASK               0x0000FF00                // NSEPPCR00_01[15..8]
#define WF_UWTBL_TOP_NSEPPCR00_NSEPPCR00_01_SHFT               8
#define WF_UWTBL_TOP_NSEPPCR00_NSEPPCR00_ADDR                  WF_UWTBL_TOP_NSEPPCR00_ADDR
#define WF_UWTBL_TOP_NSEPPCR00_NSEPPCR00_MASK                  0x000000FF                // NSEPPCR00[7..0]
#define WF_UWTBL_TOP_NSEPPCR00_NSEPPCR00_SHFT                  0

/* =====================================================================================

  ---NSEPPCR01 (0x820c4000 + 0x0404)---

    NSEPPCR01[31..0]             - (RW) The same as NSEPPCR00

 =====================================================================================*/
#define WF_UWTBL_TOP_NSEPPCR01_NSEPPCR01_ADDR                  WF_UWTBL_TOP_NSEPPCR01_ADDR
#define WF_UWTBL_TOP_NSEPPCR01_NSEPPCR01_MASK                  0xFFFFFFFF                // NSEPPCR01[31..0]
#define WF_UWTBL_TOP_NSEPPCR01_NSEPPCR01_SHFT                  0

/* =====================================================================================

  ---NSEPPCR02 (0x820c4000 + 0x0408)---

    NSEPPCR02[31..0]             - (RW) The same as NSEPPCR00

 =====================================================================================*/
#define WF_UWTBL_TOP_NSEPPCR02_NSEPPCR02_ADDR                  WF_UWTBL_TOP_NSEPPCR02_ADDR
#define WF_UWTBL_TOP_NSEPPCR02_NSEPPCR02_MASK                  0xFFFFFFFF                // NSEPPCR02[31..0]
#define WF_UWTBL_TOP_NSEPPCR02_NSEPPCR02_SHFT                  0

/* =====================================================================================

  ---NSEPPCR03 (0x820c4000 + 0x040C)---

    NSEPPCR03[31..0]             - (RW) The same as NSEPPCR00

 =====================================================================================*/
#define WF_UWTBL_TOP_NSEPPCR03_NSEPPCR03_ADDR                  WF_UWTBL_TOP_NSEPPCR03_ADDR
#define WF_UWTBL_TOP_NSEPPCR03_NSEPPCR03_MASK                  0xFFFFFFFF                // NSEPPCR03[31..0]
#define WF_UWTBL_TOP_NSEPPCR03_NSEPPCR03_SHFT                  0

/* =====================================================================================

  ---NSEPPCR_EXT_00 (0x820c4000 + 0x0480)---

    NSEPPCR_EXT_00[3..0]         - (RW) The same as NSEPPCR00
    NSEPPCR_EXT_00_01[7..4]      - (RW) The same as NSEPPCR00
    RESERVED8[31..8]             - (RO) Reserved bits

 =====================================================================================*/
#define WF_UWTBL_TOP_NSEPPCR_EXT_00_NSEPPCR_EXT_00_01_ADDR     WF_UWTBL_TOP_NSEPPCR_EXT_00_ADDR
#define WF_UWTBL_TOP_NSEPPCR_EXT_00_NSEPPCR_EXT_00_01_MASK     0x000000F0                // NSEPPCR_EXT_00_01[7..4]
#define WF_UWTBL_TOP_NSEPPCR_EXT_00_NSEPPCR_EXT_00_01_SHFT     4
#define WF_UWTBL_TOP_NSEPPCR_EXT_00_NSEPPCR_EXT_00_ADDR        WF_UWTBL_TOP_NSEPPCR_EXT_00_ADDR
#define WF_UWTBL_TOP_NSEPPCR_EXT_00_NSEPPCR_EXT_00_MASK        0x0000000F                // NSEPPCR_EXT_00[3..0]
#define WF_UWTBL_TOP_NSEPPCR_EXT_00_NSEPPCR_EXT_00_SHFT        0

/* =====================================================================================

  ---DFR (0x820c4000 + 0x1FD0)---

    DEBUG_FSM[31..0]             - (RO) debug fsm

 =====================================================================================*/
#define WF_UWTBL_TOP_DFR_DEBUG_FSM_ADDR                        WF_UWTBL_TOP_DFR_ADDR
#define WF_UWTBL_TOP_DFR_DEBUG_FSM_MASK                        0xFFFFFFFF                // DEBUG_FSM[31..0]
#define WF_UWTBL_TOP_DFR_DEBUG_FSM_SHFT                        0

/* =====================================================================================

  ---WTBLSNO (0x820c4000 + 0x1FE0)---

    WTBL_STD_NO[31..0]           - (RO)  xxx 

 =====================================================================================*/
#define WF_UWTBL_TOP_WTBLSNO_WTBL_STD_NO_ADDR                  WF_UWTBL_TOP_WTBLSNO_ADDR
#define WF_UWTBL_TOP_WTBLSNO_WTBL_STD_NO_MASK                  0xFFFFFFFF                // WTBL_STD_NO[31..0]
#define WF_UWTBL_TOP_WTBLSNO_WTBL_STD_NO_SHFT                  0

/* =====================================================================================

  ---WTBLENO (0x820c4000 + 0x1FE4)---

    WTBL_EXT_NO[31..0]           - (RO)  xxx 

 =====================================================================================*/
#define WF_UWTBL_TOP_WTBLENO_WTBL_EXT_NO_ADDR                  WF_UWTBL_TOP_WTBLENO_ADDR
#define WF_UWTBL_TOP_WTBLENO_WTBL_EXT_NO_MASK                  0xFFFFFFFF                // WTBL_EXT_NO[31..0]
#define WF_UWTBL_TOP_WTBLENO_WTBL_EXT_NO_SHFT                  0

/* =====================================================================================

  ---KTBLNO (0x820c4000 + 0x1FE8)---

    KTBL_NO[31..0]               - (RO)  xxx 

 =====================================================================================*/
#define WF_UWTBL_TOP_KTBLNO_KTBL_NO_ADDR                       WF_UWTBL_TOP_KTBLNO_ADDR
#define WF_UWTBL_TOP_KTBLNO_KTBL_NO_MASK                       0xFFFFFFFF                // KTBL_NO[31..0]
#define WF_UWTBL_TOP_KTBLNO_KTBL_NO_SHFT                       0

/* =====================================================================================

  ---DMY0 (0x820c4000 + 0x1FF0)---

    DMY0[31..0]                  - (RW) Dummy Register with default value 0 for ECO purpose

 =====================================================================================*/
#define WF_UWTBL_TOP_DMY0_DMY0_ADDR                            WF_UWTBL_TOP_DMY0_ADDR
#define WF_UWTBL_TOP_DMY0_DMY0_MASK                            0xFFFFFFFF                // DMY0[31..0]
#define WF_UWTBL_TOP_DMY0_DMY0_SHFT                            0

/* =====================================================================================

  ---DMY1 (0x820c4000 + 0x1FF4)---

    DMY1[31..0]                  - (RW) Dummy Register with default value 1 for ECO purpose

 =====================================================================================*/
#define WF_UWTBL_TOP_DMY1_DMY1_ADDR                            WF_UWTBL_TOP_DMY1_ADDR
#define WF_UWTBL_TOP_DMY1_DMY1_MASK                            0xFFFFFFFF                // DMY1[31..0]
#define WF_UWTBL_TOP_DMY1_DMY1_SHFT                            0

/* =====================================================================================

  ---WTBL_BASE (0x820c4000 + 0x1FFC)---

    WTBL_BASE[31..0]             - (RO)  xxx 

 =====================================================================================*/
#define WF_UWTBL_TOP_WTBL_BASE_WTBL_BASE_ADDR                  WF_UWTBL_TOP_WTBL_BASE_ADDR
#define WF_UWTBL_TOP_WTBL_BASE_WTBL_BASE_MASK                  0xFFFFFFFF                // WTBL_BASE[31..0]
#define WF_UWTBL_TOP_WTBL_BASE_WTBL_BASE_SHFT                  0

#ifdef __cplusplus
}
#endif

#endif // __WF_UWTBL_TOP_REGS_H__

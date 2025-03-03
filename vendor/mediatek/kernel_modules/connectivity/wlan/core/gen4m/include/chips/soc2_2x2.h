/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifdef SOC2_2X2

#ifndef _SOC2_2X2_H
#define _SOC2_2X2_H

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */
#include "gl_os.h"

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */
#define SOC2_2X2_CHIP_ID			(0x0001)
#define CONNAC_CHIP_ADIE_INFO			(0x6635)
#define SOC2_2X2_SW_SYNC0			CONN_CFG_ON_CONN_ON_MISC_ADDR
#define SOC2_2X2_SW_SYNC0_RDY_OFFSET \
	CONN_CFG_ON_CONN_ON_MISC_DRV_FM_STAT_SYNC_SHFT
#define SOC2_2X2_PATCH_START_ADDR		(0x0001C000)
#define SOC2_2X2_TOP_CFG_BASE			NIC_CONNAC_CFG_BASE
#define SOC2_2X2_TX_DESC_APPEND_LENGTH		32
#define SOC2_2X2_RX_DESC_LENGTH			20
#define SOC2_2X2_RX_INIT_EVENT_LENGTH		8
#define SOC2_2X2_RX_EVENT_HDR_LENGTH		12
#define MTK_CUSTOM_OID_INTERFACE_VERSION \
	0x00000200 /* for WPDWifi DLL */
#define MTK_EM_INTERFACE_VERSION		0x0001

/*******************************************************************************
 *                         D A T A   T Y P E S
 *******************************************************************************
 */

/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */

/*******************************************************************************
 *                  F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */

#endif /* _SOC2_2X2_H */

#endif /* SOC2_2X2 */


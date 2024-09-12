/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_display_common.c
** Description : oplus common feature
** Version : 1.0
** Date : 2020/07/1
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  JianBin.Zhang   2020/07/01        1.0           Build this moudle
******************************************************************/
#ifndef _OPLUS_DISPLAY_COMMON_H_
#define _OPLUS_DISPLAY_COMMON_H_

#include <oplus_display_private_api.h>

int oplus_display_panel_get_max_brightness(void *buf);
int oplus_display_panel_get_serial_number(void *buf);
int oplus_display_panel_set_closebl_flag(void *buf);
int oplus_display_panel_get_closebl_flag(void *buf);
int oplus_display_get_brightness(void *buf);
int oplus_display_set_brightness(void *buf);
int oplus_display_panel_set_cabc(void *buf);
int oplus_display_panel_get_cabc(void *buf);
int oplus_display_panel_set_esd(void *buf);
int oplus_display_panel_get_esd(void *buf);
int oplus_display_get_dp_support(void *buf);
int oplus_display_panel_get_pq_trigger(void *buf);
int oplus_display_panel_set_pq_trigger(void *buf);
void oplus_te_check(struct mtk_drm_crtc *mtk_crtc, unsigned long long te_time_diff);
int oplus_display_panel_get_panel_bpp(void *buf);
int oplus_display_panel_get_panel_type(void *data);
int oplus_display_panel_set_hbm_max(void *data);
int oplus_display_panel_get_hbm_max(void *data);
ssize_t oplus_get_hbm_max_debug(struct kobject *obj,
	struct kobj_attribute *attr, char *buf);
ssize_t oplus_set_hbm_max_debug(struct kobject *obj,
		struct kobj_attribute *attr,
		const char *buf, size_t count);
void oplus_mtk_read_ddic_v2(u8 ddic_reg, int ret_num, char ret_val[10]);
void oplus_ddic_dsi_send_cmd(unsigned int cmd_num, char val[20]);
#endif /*_OPLUS_DISPLAY_COMMON_H_*/

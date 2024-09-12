/**
 * Copyright (C) Fourier Semiconductor Inc. 2016-2020. All rights reserved.
 * 2024-04-27 File created.
 */

#include "fsm_public.h"
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)
#include <soc/oplus/system/oplus_mm_kevent_fb.h>
#define OPLUS_AUDIO_EVENTID_SMARTPA_ERR 10041
#endif

#if IS_ENABLED(CONFIG_SND_SOC_OPLUS_PA_MANAGER)
#include "../../mtk/oplus_speaker_manager/oplus_speaker_manager_platform.h"
#include "../../mtk/oplus_speaker_manager/oplus_speaker_manager_codec.h"

#define OPLUS_AUDIO_PA_BOOST_VOLTAGR_MAX_LEVEL 4

enum fsm_spk_mode {
	FSM_MUSIC_MODE = 0,
	FSM_VOICE_MODE,
	FSM_FM_MODE,
	FSM_MODE_NUM,
};

void *oplus_pa_fsm_node = NULL;

static int g_fsm_amp_mode[FSM_DEV_MAX] = { 0 };
static int g_fsm_amp_mute[FSM_DEV_MAX] = { 0 };

void fsm_pa_enable_by_position(uint8_t pos_mask, int enable, int mode)
{
	fsm_dev_t *fsm_dev = NULL;
	int index;
	int next_scene;
	int spk_mode;
	int ret;

	pr_info("%s pos_mask: %04X spk enable: %d mode:%d\n", __func__, pos_mask, enable, mode);

	// mode map to scene
	if (mode == RECV_MODE)
		next_scene = 15;
	else { // SPK_MODE -> mode = 0
		index = fsm_pos_mask_to_index(pos_mask);
		spk_mode = g_fsm_amp_mode[index];
		switch(spk_mode){
			case FSM_MUSIC_MODE:
				next_scene = 0;
				break;
			case FSM_VOICE_MODE:
				next_scene = 1;
				break;
			case FSM_FM_MODE:
				next_scene = 4;
				break;
			default:
				pr_err("%s: Unknown spk_mode(%d)", __func__, spk_mode);
				next_scene = 0;
				break;
		}
	}

	if (next_scene < 0 || next_scene >= FSM_SCENE_MAX) {
		pr_err("invaild next_scene: %d", next_scene);
		return;
	}

	fsm_mutex_lock();
	ret = fsm_try_init();
	if (ret) {
		pr_err("init failed: %d", ret);
		fsm_mutex_unlock();
		return;
	}

	// get fsm_dev to set scene and startup
	fsm_dev = fsm_get_fsm_dev_by_position(pos_mask);
	if (fsm_dev == NULL) {
		pr_err("get fsm_dev of speaker failed\n");
		fsm_mutex_unlock();
		return;
	}

	index = fsm_pos_mask_to_index(fsm_dev->pos_mask);
	ret = fsm_stub_set_scene(fsm_dev, BIT(next_scene));
	ret |= fsm_stub_amp_switch(fsm_dev, enable);
	if (!ret) {
		//if (mode != RECV_MODE)
		//	g_fsm_amp_mode[index] = spk_mode;
		g_fsm_amp_mute[index] = !enable;
	}
	fsm_mutex_unlock();
	return;
}

void fsm_pa_enable_l(int enable, int mode)
{
	fsm_pa_enable_by_position(FSM_POS_LTOP, enable, mode);
	return;
}

void fsm_pa_enable_r(int enable, int mode)
{
	fsm_pa_enable_by_position(FSM_POS_RBTM, enable, mode);
	return;
}

int fsm_mode_get_by_position(uint8_t pos_mask)
{
	int index;
	int ret;

	fsm_mutex_lock();
	ret = fsm_try_init();
	if (ret) {
		pr_err("init failed: %d", ret);
		fsm_mutex_unlock();
		return -EINVAL;
	}
	index = fsm_pos_mask_to_index(pos_mask);
	ret = g_fsm_amp_mode[index];
	fsm_mutex_unlock();

	return ret;
}

int fsm_mode_put_by_position(uint8_t pos_mask, int mode)
{
	struct fsm_dev *fsm_dev;
	int next_scene;
	int index;
	int ret;

	pr_info("%s pos_mask: %04X mode:%d\n", __func__, pos_mask, mode);

	// mode map to scene
	switch(mode){
		case FSM_MUSIC_MODE:
			next_scene = 0;
			break;
		case FSM_VOICE_MODE:
			next_scene = 1;
			break;
		case FSM_FM_MODE:
			next_scene = 4;
			break;
		default:
			pr_err("%s: Unknown spk_mode(%d)", __func__, mode);
			next_scene = 0;
			break;
	}

	pr_info("%s next_scene: %d", __func__, next_scene);
	if (next_scene < 0 || next_scene >= FSM_SCENE_MAX) {
		pr_err("invaild next_scene:%d", next_scene);
		return -EINVAL;
	}

	fsm_mutex_lock();
	ret = fsm_try_init();
	if (ret) {
		pr_err("init failed: %d", ret);
		fsm_mutex_unlock();
		return -EINVAL;
	}

	fsm_dev = fsm_get_fsm_dev_by_position(pos_mask);
	if (fsm_dev == NULL) {
		pr_err("get fsm_dev of speaker failed\n");
		fsm_mutex_unlock();
		return -EINVAL;
	}

	index = fsm_pos_mask_to_index(fsm_dev->pos_mask);
	ret = fsm_stub_set_scene(fsm_dev, BIT(next_scene));
	if (!ret)
		g_fsm_amp_mode[index] = mode;
	fsm_mutex_unlock();

	return 0;
}

int fsm_left_top_mode_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = fsm_mode_get_by_position(FSM_POS_LTOP);
	return 0;
}

int fsm_left_top_mode_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int mode = ucontrol->value.integer.value[0];
	int ret;

	pr_info("left channel spk mode:%d\n", mode);
	ret = fsm_mode_put_by_position(FSM_POS_LTOP, mode);
	if (ret)
		pr_err("%s mode: %d failed", __func__, mode);

	return ret;
}

int fsm_right_buttom_mode_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = fsm_mode_get_by_position(FSM_POS_RBTM);
	return 0;
}

int fsm_right_buttom_mode_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int mode = ucontrol->value.integer.value[0];
	int ret;

	pr_info("right channel spk mode:%d\n", mode);
	ret = fsm_mode_put_by_position(FSM_POS_RBTM, mode);
	if (ret)
		pr_err("%s mode: %d failed", __func__, mode);

	return ret;
}

int fsm_mute_get_by_position(uint8_t pos_mask)
{
	int index;
	int ret;

	fsm_mutex_lock();
	ret = fsm_try_init();
	if (ret) {
		pr_err("init failed: %d", ret);
		fsm_mutex_unlock();
		return -EINVAL;
	}
	index = fsm_pos_mask_to_index(pos_mask);
	ret = g_fsm_amp_mute[index];
	fsm_mutex_unlock();

	return ret;
}

int fsm_mute_put_by_position(uint8_t pos_mask, bool mute)
{
	struct fsm_dev *fsm_dev;
	int index;
	int ret;

	pr_info("%s pos_mask: %04X mute:%d\n", __func__, pos_mask, mute);

	fsm_mutex_lock();
	ret = fsm_try_init();
	if (ret) {
		pr_err("init failed: %d", ret);
		fsm_mutex_unlock();
		return -EINVAL;
	}

	fsm_dev = fsm_get_fsm_dev_by_position(pos_mask);
	if (NULL == fsm_dev) {
		pr_err("get fsm_dev of speaker failed\n");
		fsm_mutex_unlock();
		return -EINVAL;
	}

	index = fsm_pos_mask_to_index(fsm_dev->pos_mask);
	ret = fsm_stub_amp_switch(fsm_dev, !mute);
	if (!ret)
		g_fsm_amp_mute[index] = mute;
	fsm_mutex_unlock();

	return ret;
}

int fsm_left_top_mute_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = fsm_mute_get_by_position(FSM_POS_LTOP);
	return 0;
}

int fsm_left_top_mute_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	bool spkl_mute = !!(ucontrol->value.integer.value[0]);
	int ret;

	pr_info("left top speaker mute: %s", spkl_mute ? "True" : "False");
	ret = fsm_mute_put_by_position(FSM_POS_LTOP, spkl_mute);
	if (ret)
		pr_err("%s %s failed", __func__, spkl_mute ? "True" : "False");

	return ret;
}

int fsm_right_buttom_mute_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = fsm_mute_get_by_position(FSM_POS_RBTM);
	return 0;
}

int fsm_right_buttom_mute_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	bool spkr_mute = !!(ucontrol->value.integer.value[0]);
	int ret;

	pr_info("right buttom speaker mute: %s", spkr_mute ? "True" : "False");
	ret = fsm_mute_put_by_position(FSM_POS_RBTM, spkr_mute);
	if (ret)
		pr_err("%s %s failed", __func__, spkr_mute ? "True" : "False");

	return ret;
}

void fsm_oplus_speaker_pa_init(fsm_dev_t *fsm_dev)
{
	struct oplus_speaker_device *speaker_device = NULL;

	if (NULL == fsm_dev) {
		pr_err("bad parameter");
		return;
	}

	pr_info("%s(): speaker_device == null, oplus_register start\n", __func__);
	speaker_device = kzalloc(sizeof(struct oplus_speaker_device), GFP_KERNEL);
	if ((fsm_dev->channel_num == 0) && (speaker_device != NULL)) {
		speaker_device->chipset = MFR_FSM;
		speaker_device->type = L_SPK;
		speaker_device->vdd_need = 0;
		speaker_device->speaker_enable_set = fsm_pa_enable_l;
		//speaker_device->boost_voltage_set = fsm_volume_boost_set;
		//speaker_device->boost_voltage_get = fsm_volume_boost_get;
		speaker_device->spk_mode_set = fsm_left_top_mode_put;
		speaker_device->spk_mode_get = fsm_left_top_mode_get;
		speaker_device->speaker_mute_set = fsm_left_top_mute_put;
		speaker_device->speaker_mute_get = fsm_left_top_mute_get;
		oplus_pa_fsm_node = oplus_speaker_pa_register(speaker_device);
		pr_info("%s: oplus_register end\n", __func__);
	} else if ((fsm_dev->channel_num == 1) && (speaker_device != NULL)) {
		speaker_device->chipset = MFR_FSM;
		speaker_device->type = R_SPK;
		speaker_device->vdd_need = 0;
		speaker_device->speaker_enable_set = fsm_pa_enable_r;
		//speaker_device->boost_voltage_set = fsm_volume_boost_set;
		//speaker_device->boost_voltage_get = fsm_volume_boost_get;
		speaker_device->spk_mode_set = fsm_right_buttom_mode_put;
		speaker_device->spk_mode_get = fsm_right_buttom_mode_get;
		speaker_device->speaker_mute_set = fsm_right_buttom_mute_put;
		speaker_device->speaker_mute_get = fsm_right_buttom_mute_get;
		oplus_pa_fsm_node = oplus_speaker_pa_register(speaker_device);
		pr_info("%s: oplus_register end\n", __func__);
	}
}

void fsm_oplus_speaker_pa_deinit(fsm_dev_t *fsm_dev)
{
	struct oplus_speaker_device *speaker_device = NULL;

	oplus_speaker_pa_remove(oplus_pa_fsm_node);
	if (NULL == fsm_dev) {
		pr_err("bad parameter");
		return;
	}
	speaker_device = get_speaker_dev(fsm_dev->channel_num + 1);
	if (NULL != speaker_device) {
		kfree(speaker_device);
		speaker_device = NULL;
	}
	pr_info("%s(): oplus_deinit done\n", __func__);
}
#endif
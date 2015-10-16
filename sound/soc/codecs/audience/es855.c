/*
 * es855.c  --  Audience eS855 ALSA SoC Audio driver
 *
 * Copyright 2014 Audience, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "es855.h"
#include "escore.h"
#include "escore-i2c.h"
#include "escore-i2s.h"
#include "escore-spi.h"
#include "escore-slim.h"
#include "escore-cdev.h"
#include "escore-uart.h"
#include "escore-uart-common.h"
#include "es855-access.h"
/* Fix me
#include "es-d400.h"
#include "es-a350-reg.h"
*/
#include <linux/sort.h>
#ifdef CONFIG_QPNP_CLKDIV
#include <linux/qpnp/clkdiv.h>
#else
#include <linux/clk.h>
#endif

/* es855 new sync sequence after firmware downloading */
unsigned char ES855_SYNC_CMD[] = {
	0x10, 0x01, 0x80, 0x01, 0xAB, 0xCD, 0xEF, 0x89
};

unsigned char ES855_SYNC_RESP_READ_CMD[] = {
	0x10, 0x00, 0x90, 0x02
};

unsigned char ES855_SYNC_RESP[] = {
	0x40, 0x00, 0x90, 0x02, 0x40, 0x01, 0x80, 0x01,
	0xAB, 0xCD, 0xEF, 0x89
};

#define ES855_SYNC_CMD_LEN	8

#define ES855_SYNC_RESP_READ_CMD_LEN	4

#define ES855_SYNC_RESP_LEN	12

union es855_accdet_reg {
	u16 value;
	struct {
		u16 res1:1;
		u16 plug_det_fsm:1;
		u16 debounce_timer:2;
		u16 res2:1;
		u16 mic_det_fsm:1;
		u16 mg_sel_force:1;
		u16 mg_select:1;
	} fields;
};
/* codec private data TODO: move to runtime init */
struct escore_priv escore_priv = {
	.pm_state = ES_PM_ACTIVE,
	.probe = es855_core_probe,
	.set_streaming = es855_set_streaming,
	.set_datalogging = es855_set_datalogging,
	.streamdev.no_more_bit = 0,
};

struct snd_soc_dai_driver es855_dai[];

#ifdef CONFIG_ARCH_MSM8974
/*Slimbus channel map for APQ8074*/
static int es855_slim_rx_port_to_ch[ES_SLIM_RX_PORTS] = {
		152, 153, 154, 155, 134, 135
};
static int es855_slim_tx_port_to_ch[ES_SLIM_TX_PORTS] = {
		156, 157, 144, 145, 146, 147
};
#else
/*Slimbus channel map for APQ8060*/
static int es855_slim_rx_port_to_ch[ES_SLIM_RX_PORTS] = {
	152, 153, 154, 155, 134, 135
};
static int es855_slim_tx_port_to_ch[ES_SLIM_TX_PORTS] = {
	156, 157, 138, 139, 143, 144
};

#endif

#ifdef CONFIG_QPNP_CLKDIV
static struct q_clkdiv *codec_clk;
#else
static struct clk *codec_clk;
#endif

static struct escore_i2s_dai_data i2s_dai_data[ES_NUM_CODEC_I2S_DAIS];
static struct escore_slim_dai_data slim_dai_data[ES_NUM_CODEC_SLIM_DAIS];
static struct escore_slim_ch slim_rx[ES_SLIM_RX_PORTS];
static struct escore_slim_ch slim_tx[ES_SLIM_TX_PORTS];
static const u32 es855_streaming_cmds[] = {
	[ES_INVAL_INTF] = 0x00000000,	/* ES_NULL_INTF */
	[ES_SLIM_INTF]  = 0x90250200,	/* ES_SLIM_INTF */
	[ES_I2C_INTF]   = 0x90250000,	/* ES_I2C_INTF  */
	[ES_SPI_INTF]   = 0x90250300,	/* ES_SPI_INTF  */
	[ES_UART_INTF]  = 0x90250100,	/* ES_UART_INTF */
};

#ifdef CONFIG_ARCH_MSM
const struct slim_device_id escore_slim_id[] = {
	{ "earSmart-codec", ESCORE_DEVICE_NONE }, /* for portability */
	{ "earSmart-codec-intf", ESCORE_INTERFACE_DEVICE },
	{ "earSmart-codec-gen0", ESCORE_GENERIC_DEVICE },
	{  }
};
MODULE_DEVICE_TABLE(slim, escore_slim_id);
#endif

static int es855_clk_ctl(int enable)
{
	int ret = 0;

	if (enable)
#ifdef CONFIG_QPNP_CLKDIV
		ret = qpnp_clkdiv_enable(codec_clk);
#else
		ret = clk_enable(codec_clk);
#endif
	else
#ifdef CONFIG_QPNP_CLKDIV
		ret = qpnp_clkdiv_disable(codec_clk);
#else
		clk_disable(codec_clk);
#endif

	if (EINVAL != ret)
		ret = 0;

	return ret;
}
static int es855_channel_dir(int dai_id)
{
	int dir = ES_SLIM_CH_UND;

	if (dai_id == ES_SLIM_1_PB ||
	    dai_id == ES_SLIM_2_PB ||
	    dai_id == ES_SLIM_3_PB) {
		dir = ES_SLIM_CH_RX;
	} else if (dai_id == ES_SLIM_1_CAP ||
		   dai_id == ES_SLIM_2_CAP ||
		   dai_id == ES_SLIM_3_CAP)  {
		dir = ES_SLIM_CH_TX;
	}

	return dir;
}

/* es855 new sync sequence after firmware downloading */
int es855_sync_seq(struct escore_priv *escore, unsigned char *resp, int len)
{
	int rc;
	unsigned char  sync_resp[ES855_SYNC_RESP_LEN];

	if (escore->bus.ops.write == NULL || escore->bus.ops.read == NULL)
		return -ENOSYS;

	rc = escore->bus.ops.write(escore, ES855_SYNC_CMD, ES855_SYNC_CMD_LEN);
	if (rc != 0) {
		pr_debug("%s: ES855_SYNC_CMD write err -- rc =%d!\n",
			__func__, rc);
		return rc;
	}
	msleep(50);

	rc = escore->bus.ops.write(escore, ES855_SYNC_RESP_READ_CMD,
		ES855_SYNC_RESP_READ_CMD_LEN);
	if (rc != 0) {
		pr_debug("%s: ES855_SYNC_RESP_READ_CMD write err -- rc =%d!\n",
			__func__, rc);
		return rc;
	}
	msleep(50);

	rc = escore->bus.ops.read(escore, sync_resp, ES855_SYNC_RESP_LEN);
	if (rc != 0) {
		pr_debug("%s: ES855_SYNC_RESP  read err -- rc =%d!\n",
			__func__, rc);
		return rc;
	}

	rc = memcmp(sync_resp, ES855_SYNC_RESP, (size_t) ES855_SYNC_RESP_LEN);
	if (rc != 0) {
		int i;

		pr_debug("%s: ES855_SYNC_RESP  read err -- rc =%d!\n",
			__func__, rc);
		pr_debug("-----------dump resp[] begin --------------\n\n");
		for (i = 0; i < ES855_SYNC_RESP_LEN; i++)
			pr_debug("[%d]=0x%02x", i, sync_resp[i]);

		pr_debug("-----------dump resp[] end --------------\n\n");
		rc = -EINVAL;
	}

	if (resp != NULL && len >= ES855_SYNC_RESP_LEN)
		memcpy(resp, sync_resp, (size_t) ES855_SYNC_RESP_LEN);

	return rc;
}

static ssize_t es855_route_status_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	int ret = 0;

	return ret;
}

static DEVICE_ATTR(route_status, 0444, es855_route_status_show, NULL);
/* /sys/devices/platform/msm_slim_ctrl.1/es855-codec-gen0/route_status */

static ssize_t es855_get_pm_enable(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", escore_priv.pm_enable ?
			"on" : "off");
}
static ssize_t es855_set_pm_enable(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{

	pr_info("%s(): requested - %s\n", __func__, buf);
	if (!strncmp(buf, "on", 2))
		escore_pm_enable();
	else if (!strncmp(buf, "off", 3))
		escore_pm_disable();

	return count;

}
static DEVICE_ATTR(pm_enable, 0666, es855_get_pm_enable, es855_set_pm_enable);

#define SIZE_OF_VERBUF 256
/* TODO: fix for new read/write. use es855_read() instead of BUS ops */
static ssize_t es855_fw_version_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int idx = 0;
	unsigned int value;
	char versionbuffer[SIZE_OF_VERBUF];
	char *verbuf = versionbuffer;

	memset(verbuf, 0, SIZE_OF_VERBUF);

	value = escore_read(NULL, ES_FW_FIRST_CHAR);
	*verbuf++ = (value & 0x00ff);
	for (idx = 0; idx < (SIZE_OF_VERBUF-2); idx++) {
		value = escore_read(NULL, ES_FW_NEXT_CHAR);
		*verbuf++ = (value & 0x00ff);
		if (!value)
			break;
	}

	/* Null terminate the string*/
	*verbuf = '\0';
	pr_debug("Audience fw ver %s\n", versionbuffer);

	return snprintf(buf, PAGE_SIZE, "FW Version = %s\n", versionbuffer);
}

static DEVICE_ATTR(fw_version, 0444, es855_fw_version_show, NULL);
/* /sys/devices/platform/msm_slim_ctrl.1/es855-codec-gen0/fw_version */

static ssize_t es855_clock_on_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int ret = 0;

	return ret;
}

static DEVICE_ATTR(clock_on, 0444, es855_clock_on_show, NULL);
/* /sys/devices/platform/msm_slim_ctrl.1/es855-codec-gen0/clock_on */

static ssize_t es855_reset_control_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static DEVICE_ATTR(reset_control, 0444, es855_reset_control_show, NULL);

static ssize_t es855_ping_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct escore_priv *es855 = &escore_priv;
	int rc = 0;
	unsigned char  sync_resp[ES855_SYNC_RESP_LEN];
	char *status_name = "Ping";

	rc = es855_sync_seq(es855, sync_resp, ES855_SYNC_RESP_LEN);
	if (rc < 0) {
		pr_err("%s(): firmware load failed sync write\n", __func__);
		goto cmd_err;
	}
	pr_debug("%s(): sync_resp = 0x%08x ...\n", __func__,
		*((u32 *)sync_resp));

	rc = snprintf(buf, PAGE_SIZE, "%s=0x%08x ...\n", status_name,
		*((u32 *)sync_resp));

cmd_err:
	return rc;
}

static DEVICE_ATTR(ping_status, 0444, es855_ping_status_show, NULL);

/* /sys/devices/platform/msm_slim_ctrl.1/es855-codec-gen0/ping_status */
static ssize_t es855_cmd_history_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i = 0, j = 0;

	for (i = cmd_hist_index; i < ES_MAX_ROUTE_MACRO_CMD; i++) {
		if (cmd_hist[i].cmd)
			j += snprintf(buf+j,
					PAGE_SIZE,
					"cmd=0x%04x 0x%04x,tstamp=%lu\n",
					cmd_hist[i].cmd>>16,
					cmd_hist[i].cmd & 0xffff,
					cmd_hist[i].timestamp);
	}
	for (i = 0; i < cmd_hist_index; i++) {
		if (cmd_hist[i].cmd)
			j += snprintf(buf+j,
					PAGE_SIZE,
					"cmd=0x%04x 0x%04x,tstamp=%lu\n",
					cmd_hist[i].cmd>>16,
					cmd_hist[i].cmd & 0xffff,
					cmd_hist[i].timestamp);
	}
	return	j;
}
static ssize_t es855_cmd_history_clear(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	pr_info("%s(): requested - %s\n", __func__, buf);
	if (!strncmp(buf, "clear", 5)) {
		memset(cmd_hist, 0,  ES_MAX_ROUTE_MACRO_CMD *
						sizeof(cmd_hist[0]));
		cmd_hist_index = 0;
	} else {
		pr_err("%s(): Invalid command: %s\n", __func__, buf);
	}

	return count;
}
static DEVICE_ATTR(cmd_history, 0666, es855_cmd_history_show,
						es855_cmd_history_clear);
static struct attribute *core_sysfs_attrs[] = {
	&dev_attr_route_status.attr,
	&dev_attr_reset_control.attr,
	&dev_attr_clock_on.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_ping_status.attr,
	&dev_attr_cmd_history.attr,
	&dev_attr_pm_enable.attr,
	NULL
};

static struct attribute_group core_sysfs = {
	.attrs = core_sysfs_attrs
};

int es855_bootup(struct escore_priv *es855)
{
	int rc;
	u32 cmd, resp;

	pr_debug("%s()\n", __func__);

	BUG_ON(es855->standard->size == 0);

	if (es855->bus.ops.high_bw_open) {
		rc = es855->bus.ops.high_bw_open(es855);
		if (rc) {
			dev_err(es855->dev, "%s(): high_bw_open failed %d\n",
				__func__, rc);
			goto es855_high_bw_open_failed;
		}
	}

	if (es855->boot_ops.setup) {
		pr_debug("%s(): calling bus specific boot setup\n", __func__);
		rc = es855->boot_ops.setup(es855);
		if (rc != 0) {
			pr_err("%s() bus specific boot setup error, rc = %d\n",
			       __func__, rc);
			goto es855_bootup_failed;
		}
	}
	es855->mode = SBL;

	rc = es855->bus.ops.high_bw_write(es855, (char *)es855->standard->data,
			      es855->standard->size);
	if (rc < 0) {
		pr_err("%s(): firmware download failed, rc = %d\n",
		       __func__, rc);
		rc = -EIO;
		goto es855_bootup_failed;
	}

	/* Give the chip some time to become ready after firmware
	 * download. */
	msleep(20);

	if (es855->boot_ops.finish) {
		pr_debug("%s(): calling bus specific boot finish\n", __func__);
		rc = es855->boot_ops.finish(es855);
		if (rc != 0) {
			pr_err("%s() bus specific boot finish error, rc = %d\n",
				__func__, rc);
			goto es855_bootup_failed;
		}
	}
	es855->mode = STANDARD;

	if (es855->cmd_compl_mode == ES_CMD_COMP_INTR) {
		cmd = ((ES_SYNC_CMD | ES_SUPRESS_RESPONSE) << 16) |
					es855->pdata->gpio_a_irq_type;
		rc = escore_cmd(es855, cmd, &resp);
		if (rc < 0) {
			pr_err("%s(): API interrupt config failed:%d\n",
					__func__, rc);

			goto es855_bootup_failed;
		}
	}

es855_bootup_failed:
	if (es855->bus.ops.high_bw_close)
		es855->bus.ops.high_bw_close(es855);

es855_high_bw_open_failed:
	return rc;
}


static int es855_slim_set_channel_map(struct snd_soc_dai *dai,
				      unsigned int tx_num,
				      unsigned int *tx_slot,
				      unsigned int rx_num,
				      unsigned int *rx_slot)
{
	struct snd_soc_codec *codec = dai->codec;
	struct escore_priv *escore = &escore_priv;
	int id = dai->id;
	int i;
	int rc = 0;

	dev_dbg(codec->dev, "%s(): dai->name = %s, dai->id = %d\n", __func__,
		dai->name, dai->id);

	if (id == ES_SLIM_1_PB ||
	    id == ES_SLIM_2_PB ||
	    id == ES_SLIM_3_PB) {
		escore->slim_dai_data[DAI_INDEX(id)].ch_tot = rx_num;
		escore->slim_dai_data[DAI_INDEX(id)].ch_act = 0;
		for (i = 0; i < rx_num; i++)
			escore->slim_dai_data[DAI_INDEX(id)].ch_num[i] =
				rx_slot[i];
	} else if (id == ES_SLIM_1_CAP || id == ES_SLIM_2_CAP ||
							id == ES_SLIM_3_CAP) {
		escore->slim_dai_data[DAI_INDEX(id)].ch_tot = tx_num;
		escore->slim_dai_data[DAI_INDEX(id)].ch_act = 0;
		for (i = 0; i < tx_num; i++)
			escore->slim_dai_data[DAI_INDEX(id)].ch_num[i] =
				tx_slot[i];
	}

	return rc;
}

#if defined(CONFIG_ARCH_MSM)
static int es855_slim_get_channel_map(struct snd_soc_dai *dai,
				      unsigned int *tx_num,
				      unsigned int *tx_slot,
				      unsigned int *rx_num,
				      unsigned int *rx_slot)
{
	struct snd_soc_codec *codec = dai->codec;
	struct escore_priv *escore = &escore_priv;
	struct escore_slim_ch *rx = escore->slim_rx;
	struct escore_slim_ch *tx = escore->slim_tx;
	int id = dai->id;
	int i;

	dev_dbg(codec->dev, "%s(): dai->name = %s, dai->id = %d\n", __func__,
		dai->name, dai->id);

	if (id == ES_SLIM_1_PB) {
		*rx_num = escore->dai[DAI_INDEX(id)].playback.channels_max;
		for (i = 0; i < *rx_num; i++)
			rx_slot[i] = rx[ES_SLIM_1_PB_OFFSET + i].ch_num;
	} else if (id == ES_SLIM_2_PB) {
		*rx_num = escore->dai[DAI_INDEX(id)].playback.channels_max;
		for (i = 0; i < *rx_num; i++)
			rx_slot[i] = rx[ES_SLIM_2_PB_OFFSET + i].ch_num;
	} else if (id == ES_SLIM_3_PB) {
		*rx_num = escore->dai[DAI_INDEX(id)].playback.channels_max;
		for (i = 0; i < *rx_num; i++)
			rx_slot[i] = rx[ES_SLIM_3_PB_OFFSET + i].ch_num;
	} else if (id == ES_SLIM_1_CAP) {
		*tx_num = escore->dai[DAI_INDEX(id)].capture.channels_max;
		for (i = 0; i < *tx_num; i++)
			tx_slot[i] = tx[ES_SLIM_1_CAP_OFFSET + i].ch_num;
	} else if (id == ES_SLIM_2_CAP) {
		*tx_num = escore->dai[DAI_INDEX(id)].capture.channels_max;
		for (i = 0; i < *tx_num; i++)
			tx_slot[i] = tx[ES_SLIM_2_CAP_OFFSET + i].ch_num;
	} else if (id == ES_SLIM_3_CAP) {
		*tx_num = escore->dai[DAI_INDEX(id)].capture.channels_max;
		for (i = 0; i < *tx_num; i++)
			tx_slot[i] = tx[ES_SLIM_3_CAP_OFFSET + i].ch_num;
	}

	return 0;
}
#endif

int es855_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	dev_dbg(codec->dev, "%s() mute %d\n", __func__, mute);

	return 0;
}

#if defined(CONFIG_SND_SOC_ES_I2S)
static void es855_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	int id = DAI_INDEX(dai->id);

	dev_dbg(codec->dev, "%s(): dai->name = %s, dai->id = %d\n", __func__,
		dai->name, dai->id);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		escore->i2s_dai_data[id].rx_ch_tot = 0;
	else
		escore->i2s_dai_data[id].tx_ch_tot = 0;

}

static int es855_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	int id = DAI_INDEX(dai->id);
	int channels;

	dev_dbg(codec->dev, "%s(): dai->name = %s, dai->id = %d\n", __func__,
		dai->name, dai->id);

	channels = params_channels(params);
	switch (channels) {
	case 1:
	case 2:
	case 3:
	case 4:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			escore->i2s_dai_data[id].rx_ch_tot = channels;
		else
			escore->i2s_dai_data[id].tx_ch_tot = channels;

		break;
	default:
		dev_err(codec->dev,
			"%s(): unsupported number of channels, %d\n",
			__func__, channels);
		return -EINVAL;
	}

	dev_dbg(codec->dev, "%s(): params_channels(params) = %d\n", __func__,
		channels);

	return 0;
}
#endif

struct snd_soc_dai_driver es855_dai[] = {
#if defined(CONFIG_SND_SOC_ES_I2S)
	{
		.name = "earSmart-porta",
		.id = ES_I2S_PORTA,
		.playback = {
			.stream_name = "PORTA Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES_RATES,
			.formats = ES_FORMATS,
		},
		.capture = {
			.stream_name = "PORTA Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES_RATES,
			.formats = ES_FORMATS,
		},
		.ops = &escore_i2s_port_dai_ops,
	},
	{
		.name = "earSmart-portb",
		.id = ES_I2S_PORTB,
		.playback = {
			.stream_name = "PORTB Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES_RATES,
			.formats = ES_FORMATS,
		},
		.capture = {
			.stream_name = "PORTB Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES_RATES,
			.formats = ES_FORMATS,
		},
		.ops = &escore_i2s_port_dai_ops,
	},
	{
		.name = "earSmart-portc",
		.id = ES_I2S_PORTC,
		.playback = {
			.stream_name = "PORTC Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES_RATES,
			.formats = ES_FORMATS,
		},
		.capture = {
			.stream_name = "PORTC Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES_RATES,
			.formats = ES_FORMATS,
		},
		.ops = &escore_i2s_port_dai_ops,
	},
#endif
#if defined(CONFIG_SND_SOC_ES_SLIM)
	{
		.name = "es855-slim-rx1",
		.id = ES_SLIM_1_PB,
		.playback = {
			.stream_name = "SLIM_PORT-1 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES_SLIMBUS_RATES,
			.formats = ES_SLIMBUS_FORMATS,
		},
		.ops = &escore_slim_port_dai_ops,
	},
	{
		.name = "es855-slim-tx1",
		.id = ES_SLIM_1_CAP,
		.capture = {
			.stream_name = "SLIM_PORT-1 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES_SLIMBUS_RATES,
			.formats = ES_SLIMBUS_FORMATS,
		},
		.ops = &escore_slim_port_dai_ops,
	},
	{
		.name = "es855-slim-rx2",
		.id = ES_SLIM_2_PB,
		.playback = {
			.stream_name = "SLIM_PORT-2 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES_SLIMBUS_RATES,
			.formats = ES_SLIMBUS_FORMATS,
		},
		.ops = &escore_slim_port_dai_ops,
	},
	{
		.name = "es855-slim-tx2",
		.id = ES_SLIM_2_CAP,
		.capture = {
			.stream_name = "SLIM_PORT-2 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES_SLIMBUS_RATES,
			.formats = ES_SLIMBUS_FORMATS,
		},
		.ops = &escore_slim_port_dai_ops,
	},
	{
		.name = "es855-slim-rx3",
		.id = ES_SLIM_3_PB,
		.playback = {
			.stream_name = "SLIM_PORT-3 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES_SLIMBUS_RATES,
			.formats = ES_SLIMBUS_FORMATS,
		},
		.ops = &escore_slim_port_dai_ops,
	},
	{
		.name = "es855-slim-tx3",
		.id = ES_SLIM_3_CAP,
		.capture = {
			.stream_name = "SLIM_PORT-3 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = ES_SLIMBUS_RATES,
			.formats = ES_SLIMBUS_FORMATS,
		},
		.ops = &escore_slim_port_dai_ops,
	}
#endif
};

int es855_wakeup(void)
{

	dev_dbg(escore_priv.dev, "%s()\n", __func__);

	return 0;
}

/* Power state transition */
int es855_power_transition(int next_power_state,
				unsigned int set_pwr_state_cmd)
{
	int rc = 0;
	int curr_state = escore_priv.escore_power_state;

	dev_info(escore_priv.dev, "%s()\n", __func__);

	/* Power state transition */
	while (next_power_state != curr_state) {
		switch (curr_state) {
		case ES_SET_POWER_STATE_SLEEP:
			/* Wakeup Chip */
			rc = es855_wakeup();
			if (rc) {
				dev_err(escore_priv.dev,
					"%s(): es855 wakeup failed, rc = %d\n",
					__func__, rc);
				goto es855_power_transition_exit;
			}
			curr_state = ES_SET_POWER_STATE_NORMAL;
			break;

		case ES_SET_POWER_STATE_NORMAL:
			/* Either switch to Sleep */
			switch (next_power_state) {
			case ES_SET_POWER_STATE_SLEEP:
				curr_state = ES_SET_POWER_STATE_SLEEP;
				break;
			default:
				curr_state = next_power_state;
				break;
			}
			rc = escore_write(NULL, set_pwr_state_cmd, curr_state);
			if (rc) {
				dev_err(escore_priv.dev,
					"%s(): PwrSt cmd write failed, rc=%d\n",
					__func__, rc);
				curr_state = ES_SET_POWER_STATE_NORMAL;
				goto es855_power_transition_exit;
			}

			break;

		case ES_SET_POWER_STATE_MP_SLEEP:
			rc = es855_wakeup();
			if (rc) {
				dev_err(escore_priv.dev,
					"%s(): es855 wakeup failed, rc = %d\n",
					__func__, rc);
				goto es855_power_transition_exit;
			}
			curr_state = ES_SET_POWER_STATE_MP_CMD;
			break;
		case ES_SET_POWER_STATE_MP_CMD:
			rc = escore_write(NULL, set_pwr_state_cmd,
					  ES_SET_POWER_STATE_NORMAL);
			if (rc) {
				dev_err(escore_priv.dev,
					"%s(): PwrSt cmd write failed, rc=%d\n",
					__func__, rc);
				goto es855_power_transition_exit;
			}
			curr_state = ES_SET_POWER_STATE_NORMAL;
			break;
		default:
			dev_err(escore_priv.dev,
				"%s(): Unsupported state in es855\n",
				__func__);
			rc = -EINVAL;
			goto es855_power_transition_exit;
		}
		dev_dbg(escore_priv.dev, "%s(): Current state = %d, val=%d\n",
			__func__, curr_state, next_power_state);
	}

	dev_dbg(escore_priv.dev, "%s(): Power state change successful\n",
		__func__);
es855_power_transition_exit:
	escore_priv.escore_power_state = curr_state;
	return rc;
}

static int es855_put_preset_value(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int value;
	int rc = 0;

	value = ucontrol->value.integer.value[0];

	rc = escore_write(NULL, reg, value);
	if (rc) {
		dev_err(escore_priv.dev, "%s(): Set Preset failed, rc = %d\n",
			__func__, rc);
		return rc;
	}

	escore_priv.preset = value;

	return rc;
}

static int es855_get_preset_value(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = escore_priv.preset;

	return 0;
}

static int es855_get_rdb_size(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] =
				escore_priv.datablock_dev.rdb_read_count;

	return 0;
}

static int es855_get_event_status(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = escore_priv.escore_event_type;

	/* Reset the event status after read */
	escore_priv.escore_event_type = ES_NO_EVENT;

	return 0;
}

static struct snd_kcontrol_new es855_snd_controls[] = {
	SOC_SINGLE_EXT("Preset",
		       ES_PRESET, 0, 65535, 0, es855_get_preset_value,
		       es855_put_preset_value),
	SOC_SINGLE_EXT("ES855 Get Event Status",
		       SND_SOC_NOPM, 0, 65535, 0, es855_get_event_status, NULL),
	SOC_SINGLE_EXT("Get RDB data size",
		       SND_SOC_NOPM, 0, 65535, 0,
		       es855_get_rdb_size, NULL),
};

static int es855_set_bias_level(struct snd_soc_codec *codec,
				      enum snd_soc_bias_level level)
{
	pr_debug("%s(): Setting bias level to :%d\n", __func__, level);

	codec->dapm.bias_level = level;

	return 0;
}

static int es855_codec_probe(struct snd_soc_codec *codec)
{
	int ret;
	struct escore_priv *es855 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "%s()\n", __func__);

	es855->codec = codec;
	codec->control_data = snd_soc_codec_get_drvdata(codec);

/* Fix me
	ret = es_d400_add_snd_soc_controls(codec);
	if (ret) {
		dev_err(codec->dev,
			"%s(): es_d400_snd_controls failed\n", __func__);
		return ret;
	}
	ret = es_analog_add_snd_soc_controls(codec);
	if (ret) {
		dev_err(codec->dev,
			"%s(): es_analog_snd_controls failed\n", __func__);
		return ret;
	}
	ret = es_d400_add_snd_soc_dapm_controls(codec);
	if (ret) {
		dev_err(codec->dev,
			"%s(): es_d400_dapm_widgets failed\n", __func__);
		return ret;
	}
	ret = es_analog_add_snd_soc_dapm_controls(codec);
	if (ret) {
		dev_err(codec->dev,
			"%s(): es_analog_dapm_widgets failed\n", __func__);
		return ret;
	}
	ret = es_d400_add_snd_soc_route_map(codec);
	if (ret) {
		dev_err(codec->dev,
			"%s(): es_d400_add_routes failed\n", __func__);
		return ret;
	}
	ret = es_analog_add_snd_soc_route_map(codec);
	if (ret) {
		dev_err(codec->dev,
			"%s(): es_analog_add_routes failed\n", __func__);
		return ret;
	}
*/

#if defined(KERNEL_VERSION_3_4_0) || defined(KERNEL_VERSION_3_8_0)
	ret = snd_soc_add_codec_controls(codec, es855_snd_controls,
			ARRAY_SIZE(es855_snd_controls));
#elif defined(KERNEL_VERSION_3_2_0)
	ret = snd_soc_add_controls(codec, es855_snd_controls,
			ARRAY_SIZE(es855_snd_controls));
#endif

/* Fix me
	ret = es_d400_fill_cmdcache(escore_priv.codec);
*/
	if (ret) {
		dev_err(codec->dev,
			"%s(): Cache initialization failed\n", __func__);
		return ret;
	}

	es855_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return ret;
}

static int  es855_codec_remove(struct snd_soc_codec *codec)
{

	pr_debug("%s(): Codec removed\n", __func__);

	return 0;
}

static unsigned int es855_codec_read(struct snd_soc_codec *codec,
				unsigned int reg)
{
	struct escore_priv *escore = &escore_priv;
	struct escore_api_access *api_access;
	u32 cmd;
	u32 resp;
	unsigned int msg_len;
	int rc;

	if (reg > ES_MAX_REGISTER) {
		/*dev_err(codec->dev, "read out of range reg %d", reg);*/
		return 0;
	}

	if (!escore->reg_cache[reg].is_volatile)
		return escore->reg_cache[reg].value & 0xff;

	api_access = &escore->api_access[ES_CODEC_VALUE];
	msg_len = api_access->read_msg_len;
	memcpy((char *)&cmd, (char *)api_access->read_msg, msg_len);

	rc = escore_cmd(escore, cmd | reg<<8, &resp);
	if (rc < 0) {
		dev_err(codec->dev, "codec reg read err %d()", rc);
		return rc;
	}
	cmd = escore->bus.last_response;

	return cmd & 0xff;

}

static int es855_codec_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	struct escore_priv *escore = &escore_priv;
	int ret;

	if (reg > ES_MAX_REGISTER) {
		/*dev_err(codec->dev, "write out of range reg %d", reg);*/
		return 0;
	}

	ret = escore_write(codec, ES_CODEC_VALUE, reg<<8 | value);
	if (ret < 0) {
		dev_err(codec->dev, "codec reg %x write err %d\n",
			reg, ret);
		return ret;
	}

	escore->reg_cache[reg].value = value;
	return 0;
}

struct snd_soc_codec_driver soc_codec_dev_es855 = {
	.probe =	es855_codec_probe,
	.remove =	es855_codec_remove,
	.read =		es855_codec_read,
	.write =	es855_codec_write,
	.set_bias_level =	es855_set_bias_level,
};

int es855_set_streaming(struct escore_priv *escore, int value)
{
	u32 resp;
	return escore_cmd(escore,
		es855_streaming_cmds[escore->streamdev.intf] | value, &resp);
}

int es855_set_datalogging(struct escore_priv *escore, int value)
{
	u32 resp;
	return escore_cmd(escore, value, &resp);
}

void es855_slim_setup(struct escore_priv *escore_priv)
{
	int i;
	int ch_cnt;

	escore_priv->init_slim_slave(escore_priv);

	/* allocate ch_num array for each DAI */
	for (i = 0; i < ARRAY_SIZE(es855_dai); i++) {
		switch (es855_dai[i].id) {
		case ES_SLIM_1_PB:
		case ES_SLIM_2_PB:
		case ES_SLIM_3_PB:
			ch_cnt = es855_dai[i].playback.channels_max;
			break;
		case ES_SLIM_1_CAP:
		case ES_SLIM_2_CAP:
		case ES_SLIM_3_CAP:
			ch_cnt = es855_dai[i].capture.channels_max;
			break;
		default:
				continue;
		}
		escore_priv->slim_dai_data[i].ch_num =
			kzalloc((ch_cnt * sizeof(unsigned int)), GFP_KERNEL);
	}
	/* front end for RX1 */
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_1_PB)].ch_num[0] = 152;
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_1_PB)].ch_num[1] = 153;
	/* back end for RX1 */
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_2_CAP)].ch_num[0] = 138;
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_2_CAP)].ch_num[1] = 139;
	/* front end for TX1 */
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_1_CAP)].ch_num[0] = 156;
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_1_CAP)].ch_num[1] = 157;
	/* back end for TX1 */
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_3_PB)].ch_num[0] = 134;
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_3_PB)].ch_num[1] = 135;
	/* front end for RX2 */
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_2_PB)].ch_num[0] = 154;
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_2_PB)].ch_num[1] = 155;
	/* back end for RX2 */
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_3_CAP)].ch_num[0] = 143;
	escore_priv->slim_dai_data[DAI_INDEX(ES_SLIM_3_CAP)].ch_num[1] = 144;
}

static int es855_config_jack(struct escore_priv *escore)
{
	struct esxxx_accdet_config *accdet_cfg = &escore->pdata->accdet_cfg;
	union es855_accdet_reg accdet_reg;
	u32 cmd;
	u32 resp;
	int rc;

	/* Setup the Event response */
	cmd = (ES_SET_EVENT_RESP << 16) | escore->pdata->gpio_b_irq_type;
	rc = escore->bus.ops.cmd(escore, cmd, &resp);
	if (rc < 0) {
		pr_err("%s(): Error %d in setting event response\n",
		       __func__, rc);
		goto out;
	}

	accdet_reg.value = 0;
	accdet_reg.fields.plug_det_fsm = accdet_cfg->plug_det_enabled & (0x1);
	accdet_reg.fields.debounce_timer = accdet_cfg->debounce_timer & (0x3);

	/* Setup the debounce timer for plug event */
	cmd = (ES_ACCDET_CONFIG_CMD << 16) | (accdet_reg.value);
	rc = escore->bus.ops.cmd(escore, cmd, &resp);
	if (rc < 0)
		pr_err("%s(): Error %d in setting debounce timer\n",
		       __func__, rc);

out:
	return rc;
}

int es855_detect(struct snd_soc_codec *codec, struct snd_soc_jack *jack)
{
	struct escore_priv *escore = snd_soc_codec_get_drvdata(codec);
	int rc;

	rc = escore_pm_get_sync();
	if (rc > -1) {
		rc = es855_config_jack(escore);
		escore_pm_put_autosuspend();
	}

	if (rc >= 0)
		escore->jack = jack;

	return rc;
}
EXPORT_SYMBOL_GPL(es855_detect);


static struct esxxx_platform_data *es855_populate_dt_pdata(
							struct device *dev)
{
	struct esxxx_platform_data *pdata;
	struct property *prop;
	struct es755_btn_cfg  *es855_btn_cfg;

	u8 *temp;
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return NULL;
	}

	es855_btn_cfg = devm_kzalloc(dev, sizeof(*es855_btn_cfg), GFP_KERNEL);
	if (!es855_btn_cfg) {
		dev_err(dev, "could not allocate memory for es855_btn_cfg\n");
		return NULL;
	}

	pdata->priv = (struct es755_btn_cfg *)es855_btn_cfg;
	pdata->reset_gpio = of_get_named_gpio(dev->of_node,
					      "adnc,reset-gpio", 0);
	if (pdata->reset_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
			"adnc,reset-gpio", dev->of_node->full_name,
			pdata->reset_gpio);
		pdata->reset_gpio = -1;
	}
	dev_dbg(dev, "%s: reset gpio %d", __func__, pdata->reset_gpio);

	pdata->wakeup_gpio = of_get_named_gpio(dev->of_node,
						"adnc,wakeup-gpio", 0);
	if (pdata->wakeup_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
			"adnc,wakeup-gpio", dev->of_node->full_name,
			pdata->wakeup_gpio);
		pdata->wakeup_gpio = -1;
	}
	dev_dbg(dev, "%s: wakeup gpio %d", __func__, pdata->wakeup_gpio);

	pdata->uart_gpio = of_get_named_gpio(dev->of_node, "adnc,int-gpio", 0);
	if (pdata->uart_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
			"adnc,uart-gpio", dev->of_node->full_name,
			pdata->uart_gpio);
		pdata->uart_gpio = -1;
	}
	dev_dbg(dev, "%s(): uart gpio %d", __func__, pdata->uart_gpio);

/* API Interrupt registration */
#ifdef CONFIG_SND_SOC_ES_GPIO_A
	dev_dbg(dev, "%s(): gpioa configured\n", __func__);
	pdata->gpioa_gpio = of_get_named_gpio(dev->of_node,
			"adnc,gpioa-gpio", 0);
	if (pdata->gpioa_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"adnc,gpioa-gpio", dev->of_node->full_name,
				pdata->gpioa_gpio);
		pdata->gpioa_gpio = -1;
	}
#endif
	dev_dbg(dev, "%s: gpioa_gpio %d", __func__, pdata->gpioa_gpio);

	pdata->gpiob_gpio = of_get_named_gpio(dev->of_node,
					      "adnc,gpiob-gpio", 0);
	if (pdata->gpiob_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
			"adnc,gpiob-gpio", dev->of_node->full_name,
			pdata->gpiob_gpio);
		pdata->gpiob_gpio = -1;
	}
	dev_dbg(dev, "%s: gpiob_gpio %d", __func__, pdata->gpiob_gpio);

	prop = of_find_property(dev->of_node, "adnc,enable_hs_uart_intf", NULL);
	if (prop != NULL) {
		temp =	(u8 *)prop->value;
		if (temp[3] == 1)
			pdata->enable_hs_uart_intf = true;
		else
			pdata->enable_hs_uart_intf = false;
	}

	prop = of_find_property(dev->of_node, "adnc,ext_clk_rate", NULL);
	if (prop != NULL) {
		temp = (u8 *)prop->value;
		if (temp[3] != 0)
			pdata->ext_clk_rate = temp[3];
	}

	prop = of_find_property(dev->of_node, "adnc,btn_press_settling_time",
			NULL);
	if (prop != NULL) {
		temp = (u8 *)prop->value;
		if (temp[3] != 0)
			es855_btn_cfg->btn_press_settling_time = temp[3];
	}

	prop = of_find_property(dev->of_node, "adnc,btn_press_polling_rate",
			NULL);
	if (prop != NULL) {
		temp = (u8 *)prop->value;
		if (temp[3] != 0)
			es855_btn_cfg->btn_press_polling_rate = temp[3];
	}

	prop = of_find_property(dev->of_node, "adnc,btn_press_det_act", NULL);
	if (prop != NULL) {
		temp = (u8 *)prop->value;
		if (temp[3] != 0)
			es855_btn_cfg->btn_press_det_act = temp[3];
	}

	prop = of_find_property(dev->of_node, "adnc,double_btn_timer", NULL);
	if (prop != NULL) {
		temp = (u8 *)prop->value;
		if (temp[3] != 0)
			es855_btn_cfg->double_btn_timer = temp[3];
	}

	prop = of_find_property(dev->of_node, "adnc,mic_det_settling_timer",
			NULL);
	if (prop != NULL) {
		temp = (u8 *)prop->value;
		if (temp[3] != 0)
			es855_btn_cfg->mic_det_settling_timer = temp[3];
	}

	prop = of_find_property(dev->of_node, "adnc,long_btn_timer", NULL);
	if (prop != NULL) {
		temp = (u8 *)prop->value;
		if (temp[3] != 0)
			es855_btn_cfg->long_btn_timer = temp[3];
	}

	prop = of_find_property(dev->of_node, "adnc,adc_btn_mute", NULL);
	if (prop != NULL) {
		temp = (u8 *)prop->value;
		if (temp[3] != 0)
			es855_btn_cfg->adc_btn_mute = temp[3];
	}

	prop = of_find_property(dev->of_node, "adnc,valid_levels", NULL);
	if (prop != NULL) {
		temp =	(u8 *)prop->value;
		if (temp[3] != 0)
			es855_btn_cfg->valid_levels = temp[3];
	}

	prop = of_find_property(dev->of_node, "adnc,impd_det_timer", NULL);
	if (prop != NULL) {
		temp = (u8 *)prop->value;
		if (temp[3] != 0)
			es855_btn_cfg->impd_det_timer = temp[3];
	}

	prop = of_find_property(dev->of_node, "adnc,gpio_b_irq_type", NULL);
	if (prop != NULL) {
		temp = (u8 *)prop->value;
		if (temp[3] != 0)
			pdata->gpio_b_irq_type = temp[3];
	}

	prop = of_find_property(dev->of_node, "adnc,debounce_timer", NULL);
	if (prop != NULL) {
		temp = (u8 *)prop->value;
		if (temp[3] != 0)
			pdata->accdet_cfg.debounce_timer = temp[3];
	}

	prop = of_find_property(dev->of_node, "adnc,plug_det_enabled", NULL);
	if (prop != NULL) {
		temp = (u8 *)prop->value;
		if (temp[3] != 0)
			pdata->accdet_cfg.plug_det_enabled = temp[3];
	}

	prop = of_find_property(dev->of_node, "adnc,mic_det_enabled", NULL);
	if (prop != NULL) {
		temp = (u8 *)prop->value;
		if (temp[3] != 0)
			pdata->accdet_cfg.mic_det_enabled = temp[3];
	}
#ifdef CONFIG_QPNP_CLKDIV
	codec_clk = qpnp_clkdiv_get(dev, "es855-mclk");
#else
	codec_clk = clk_get(dev, "es855-mclk");
#endif
	if (IS_ERR(codec_clk)) {
		dev_err(dev, "%s: Failed to request es855 mclk from pmic %ld\n",
			__func__, PTR_ERR(codec_clk));
		pdata->esxxx_clk_cb = NULL;
	} else {
		pdata->esxxx_clk_cb = es855_clk_ctl;
		pdata->esxxx_clk_cb(1);
	}

	return pdata;
}


static int es855_mic_config(struct escore_priv *escore)
{
	struct snd_soc_codec *codec = escore->codec;
	union es855_accdet_reg accdet_reg;
	struct esxxx_accdet_config accdet_cfg = escore->pdata->accdet_cfg;

	accdet_reg.value = 0;

	accdet_reg.fields.mic_det_fsm = 1;
	accdet_reg.fields.plug_det_fsm = accdet_cfg.plug_det_enabled & (0x1);
	accdet_reg.fields.debounce_timer = accdet_cfg.debounce_timer & (0x3);

	/* This allows detection of both type of headsets: LRGM and LRMG */
	accdet_reg.fields.mg_sel_force = accdet_cfg.mic_det_enabled & (0x1);

	return escore_write(codec, ES_ACCDET_CONFIG, accdet_reg.value);
}

static int es855_button_config(struct escore_priv *escore)
{
	struct esxxx_platform_data *pdata = escore->pdata;
	struct es755_btn_cfg *btn_cfg;
	union es755_btn_ctl1 btn_ctl1;
	union es755_btn_ctl2 btn_ctl2;
	union es755_btn_ctl3 btn_ctl3;
	union es755_btn_ctl4 btn_ctl4;
	u8 invalid = (u8) -1;
	u8 update = 0;
	int rc = 0;

	btn_cfg = (struct es755_btn_cfg *)pdata->priv;

	btn_ctl1.value = 0;
	btn_ctl2.value = 0;
	btn_ctl3.value = 0;
	btn_ctl4.value = 0;

	/* Config for Button Control 1 */
	if (btn_cfg->btn_press_settling_time != invalid) {
		btn_ctl1.fields.btn_press_settling_time =
			(btn_cfg->btn_press_settling_time & 0x7);
		update = 1;
	}
	if (btn_cfg->btn_press_polling_rate != invalid) {
		btn_ctl1.fields.btn_press_polling_rate =
			(btn_cfg->btn_press_polling_rate & 0x3);
		update = 1;
	}
	if (btn_cfg->btn_press_det_act != invalid) {
		btn_ctl1.fields.btn_press_det =
			(btn_cfg->btn_press_det_act & 0x1);
		update = 1;
	}

	if (update) {
		rc = escore_write(NULL, ES_BUTTON_CTRL1, btn_ctl1.value);
		if (rc < 0) {
			pr_err("%s(): Error setting button control 1, rc %d\n",
			       __func__, rc);
			goto btn_cfg_exit;
		}
		update = 0;
	}

	/* Config for Button Control 2 */
	if (btn_cfg->double_btn_timer != invalid) {
		btn_ctl2.fields.double_btn_timer =
			(btn_cfg->double_btn_timer & 0xf);
		update = 1;
	}
	if (btn_cfg->mic_det_settling_timer != invalid) {
		btn_ctl2.fields.mic_det_settling_timer =
			(btn_cfg->mic_det_settling_timer & 0x3);
		update = 1;
	}
	if (update) {
		rc = escore_write(NULL, ES_BUTTON_CTRL2, btn_ctl2.value);
		if (rc < 0) {
			pr_err("%s(): Error setting button control 2, rc %d\n",
			       __func__, rc);
			goto btn_cfg_exit;
		}
		update = 0;
	}

	/* Config for Button Control 3 */
	if (btn_cfg->long_btn_timer != invalid) {
		btn_ctl3.fields.long_btn_timer =
			(btn_cfg->long_btn_timer & 0xf);
		update = 1;
	}
	if (btn_cfg->adc_btn_mute != invalid) {
		btn_ctl3.fields.adc_btn_mute =
			(btn_cfg->adc_btn_mute & 0x1);
		update = 1;
	}
	if (update) {
		rc = escore_write(NULL, ES_BUTTON_CTRL3, btn_ctl3.value);
		if (rc < 0) {
			pr_err("%s(): Error setting button control 3, rc %d\n",
			       __func__, rc);
			goto btn_cfg_exit;
		}
		update = 0;
	}

	/* Config for Button Control 4 */
	if (btn_cfg->valid_levels != invalid) {
		btn_ctl4.fields.valid_levels =
			(btn_cfg->valid_levels & 0x3f);
		update = 1;
	}
	if (btn_cfg->impd_det_timer != invalid) {
		btn_ctl4.fields.impd_det_timer =
			(btn_cfg->impd_det_timer & 0x3);
		update = 1;
	}
	if (update) {
		rc = escore_write(NULL, ES_BUTTON_CTRL4, btn_ctl4.value);
		if (rc < 0) {
			pr_err("%s(): Error setting button control 4, rc %d\n",
			       __func__, rc);
			goto btn_cfg_exit;
		}
		update = 0;
	}

btn_cfg_exit:
	return rc;

}

static int es855_codec_intr(struct notifier_block *self,
				 unsigned long action, void *dev)
{
	struct escore_priv *escore = (struct escore_priv *)dev;
	union es855_accdet_status_reg accdet_reg;
	int value;
	int rc = 0;
	u8 impd_level;

	pr_debug("%s(): Event: 0x%04x\n", __func__, (u32)action);

	if (action & ES_CODEC_INTR_EVENT) {
		value = escore_read(NULL, ES_GET_SYS_INTERRUPT);
		if (value < 0) {
			pr_err("%s(): Get System Event failed\n", __func__);
			goto intr_exit;
		}

		if (ES_PLUG_EVENT(value)) {

			pr_debug("%s(): Plug event\n", __func__);
			/* Enable MIC Detection */
			rc = es855_mic_config(escore);
			if (rc < 0) {
				pr_err("%s(): MIC config failed, rc %d\n",
				       __func__, rc);
				goto intr_exit;
			}
		} else if (ES_MICDET_EVENT(value)) {

			value = escore_read(NULL, ES_GET_ACCDET_STATUS);
			if (value < 0) {
				pr_err("%s(): Acc detect sts fail, rc %d\n",
				       __func__, rc);
				goto intr_exit;
			}

			accdet_reg.value = value;
			impd_level = accdet_reg.fields.impd_level;

			if (impd_level) {
				pr_debug("%s(): Headset detected\n", __func__);
				if (impd_level < MIC_IMPEDANCE_LEVEL_LRGM) {
					pr_debug("LRMG Headset\n");
				} else if (impd_level ==
						MIC_IMPEDANCE_LEVEL_LRGM) {
					pr_debug("LRGM Headset\n");
				} else {
					pr_err("Invalid type:%d\n", impd_level);
					rc = -EINVAL;
					goto intr_exit;
				}

				snd_soc_jack_report(escore->jack,
						    SND_JACK_HEADSET,
						    JACK_DET_MASK);

				rc = es855_button_config(escore);
				if (rc < 0)
					goto intr_exit;

			} else {
				pr_debug("%s() Headphone detected\n", __func__);
				snd_soc_jack_report(escore->jack,
						    SND_JACK_HEADPHONE,
						    JACK_DET_MASK);
			}

			pr_debug("%s(): AccDet status 0x%04x\n", __func__,
				 value);
		} else if (ES_BUTTON_PRESS_EVENT(value)) {
			pr_debug("%s(): Button event\n", __func__);
		} else if (ES_BUTTON_RELEASE_EVENT(value)) {
			pr_debug("%s(): Button released\n", __func__);
			value = escore_read(NULL, ES_GET_ACCDET_STATUS);
			if (value < 0) {
				pr_err("%s(): Accessory detect status failed\n",
				       __func__);
				goto intr_exit;
			}

			accdet_reg.value = value;

			pr_debug("Impd:%d\n", accdet_reg.fields.impd_level);

			switch (accdet_reg.fields.impd_level) {
			case 0:
				snd_soc_jack_report(escore->jack,
						    SND_JACK_BTN_0,
						    JACK_DET_MASK);
				/* Button release event must be sent */
				snd_soc_jack_report(escore->jack, 0,
						    SND_JACK_BTN_0);

				break;
			case 2:
				snd_soc_jack_report(escore->jack,
						    SND_JACK_BTN_1,
						    JACK_DET_MASK);
				/* Button release event must be sent */
				snd_soc_jack_report(escore->jack, 0,
						    SND_JACK_BTN_1);
				break;
			case 3:
				snd_soc_jack_report(escore->jack,
						    SND_JACK_BTN_2,
						    JACK_DET_MASK);
				/* Button release event must be sent */
				snd_soc_jack_report(escore->jack, 0,
						    SND_JACK_BTN_2);
				break;
			default:
				pr_debug("No report of event\n");
				break;
			}
		} else if (ES_UNPLUG_EVENT(value)) {
			pr_debug("%s(): Unplug detected\n", __func__);
			snd_soc_jack_report(escore->jack, 0, JACK_DET_MASK);
		}
	}

intr_exit:
	return NOTIFY_OK;
}

irqreturn_t es855_cmd_completion_isr(int irq, void *data)
{
	struct escore_priv *escore = (struct escore_priv *)data;

	BUG_ON(!escore);

	pr_debug("%s(): API Rising edge received\n",
			__func__);
	/* Complete if expected */
	if (escore->wait_api_intr) {
		pr_debug("%s(): API Rising edge completion.\n",
				__func__);
		complete(&escore->cmd_compl);
		escore->wait_api_intr = 0;
	}
	return IRQ_HANDLED;
}

irqreturn_t es855_irq_work(int irq, void *data)
{
	struct escore_priv *escore = (struct escore_priv *)data;
	int rc;
	u32 cmd = 0;

	if (!escore) {
		pr_err("%s(): Invalid IRQ data\n", __func__);
		goto irq_exit;
	}

	/* Delay required for firmware to be ready in case of CVQ mode */
	msleep(50);

	/* Power state change:
	 * If the chip is in "MP Sleep", the interrupt from codec will
	 * put the chip into command mode. This requires the codec
	 * configuration to be done again.
	 */
	if (escore->escore_power_state == ES_SET_POWER_STATE_MP_SLEEP) {
		pr_debug("%s(): Power state:%d\n", __func__,
			 escore->escore_power_state);

		switch (escore->escore_power_state) {
		case ES_SET_POWER_STATE_MP_SLEEP:
			escore->escore_power_state = ES_SET_POWER_STATE_NORMAL;
			break;
		default:
			pr_err("%s(): IRQ in invalid power state - %d\n",
			       __func__, escore->escore_power_state);
			break;
		}

		rc = escore_reconfig_intr(escore);
		if (rc < 0) {
			pr_err("%s(): Error in reconfig interupt :%d\n",
			       __func__, rc);
			goto irq_exit;
		}
	}

	cmd = ES_GET_EVENT << 16;

	rc = escore_cmd(escore, cmd, &escore->escore_event_type);
	if (rc < 0) {
		pr_err("%s(): Error reading IRQ event\n", __func__);
		goto irq_exit;
	}

	escore->escore_event_type &= ES_MASK_INTR_EVENT;

	if (escore->escore_event_type != ES_NO_EVENT) {
		pr_debug("%s(): Notify subscribers about 0x%04x event",
			 __func__, escore->escore_event_type);
		blocking_notifier_call_chain(escore->irq_notifier_list,
					     escore->escore_event_type, escore);
	}

irq_exit:
	return IRQ_HANDLED;
}

static BLOCKING_NOTIFIER_HEAD(es855_irq_notifier_list);

static struct notifier_block es855_codec_intr_cb = {
	.notifier_call = es855_codec_intr,
	.priority = ES_NORMAL,
};


int es855_core_probe(struct device *dev)
{
	struct esxxx_platform_data *pdata ;
	int rc = 0;
	unsigned long irq_flags = IRQF_DISABLED;
	const char *fw_filename = "audience/es855/audience-es855-fw.bin";

	if (dev->of_node) {
		dev_info(dev, "Platform data from device tree\n");
		pdata = es855_populate_dt_pdata(dev);
		dev->platform_data = pdata;
	} else {
		dev_info(dev, "Platform data from board file\n");
		pdata = dev->platform_data;
	}

	escore_priv.pdata = pdata;

	mutex_init(&escore_priv.datablock_dev.datablock_read_mutex);
	mutex_init(&escore_priv.pm_mutex);
	mutex_init(&escore_priv.streaming_mutex);
	mutex_init(&escore_priv.msg_list_mutex);
	mutex_init(&escore_priv.datablock_dev.datablock_mutex);

	init_completion(&escore_priv.cmd_compl);
	init_waitqueue_head(&escore_priv.stream_in_q);
	INIT_LIST_HEAD(&escore_priv.msg_list);
	escore_priv.irq_notifier_list = &es855_irq_notifier_list;
	escore_priv.cmd_compl_mode = ES_CMD_COMP_POLL;
	escore_priv.wait_api_intr = 0;


	rc = sysfs_create_group(&escore_priv.dev->kobj, &core_sysfs);
	if (rc) {
		dev_err(escore_priv.dev,
			"failed to create core sysfs entries: %d\n", rc);
	}

	dev_dbg(escore_priv.dev, "%s(): reset_gpio = %d\n", __func__,
		pdata->reset_gpio);
	if (pdata->reset_gpio != -1) {
		rc = gpio_request(pdata->reset_gpio, "es855_reset");
		if (rc < 0) {
			dev_err(escore_priv.dev,
				"%s(): reset_gpio(%d) request failed, rc %d",
				__func__, pdata->reset_gpio, rc);
			goto reset_gpio_request_error;
		}
		rc = gpio_direction_output(pdata->reset_gpio, 1);
		if (rc < 0) {
			dev_err(escore_priv.dev,
				"%s(): reset_gpio direction failed, rc %d",
				__func__, rc);
			goto reset_gpio_direction_error;
		}
		if (!escore_priv.flag.reset_done)
			escore_gpio_reset(&escore_priv);
	} else {
		dev_warn(escore_priv.dev, "%s(): reset_gpio undefined\n",
			 __func__);
	}

	dev_dbg(escore_priv.dev, "%s(): wakeup_gpio = %d\n", __func__,
		pdata->wakeup_gpio);
	if (pdata->wakeup_gpio != -1) {
		rc = gpio_request(pdata->wakeup_gpio, "es855_wakeup");
		if (rc < 0) {
			dev_err(escore_priv.dev,
				"%s(): wakeup_gpio(%d) request failed, rc %d",
				__func__, pdata->wakeup_gpio, rc);
			goto wakeup_gpio_request_error;
		}
		rc = gpio_direction_output(pdata->wakeup_gpio, 1);
		if (rc < 0) {
			dev_err(escore_priv.dev,
				"%s(): wakeup_gpio direction failed, rc %d",
				__func__, rc);
			goto wakeup_gpio_direction_error;
		}
	} else {
		dev_warn(escore_priv.dev, "%s(): wakeup_gpio undefined\n",
			 __func__);
	}

#ifndef CONFIG_SND_SOC_ES_GPIO_A
	dev_dbg(dev, "%s(): gpioa not configured\n", __func__);
	pdata->gpioa_gpio = -1;
#endif
	dev_dbg(escore_priv.dev, "%s(): gpioa_gpio = %d\n", __func__,
		pdata->gpioa_gpio);
	if (pdata->gpioa_gpio != -1) {
		rc = gpio_request(pdata->gpioa_gpio, "es855_gpioa");
		if (rc < 0) {
			dev_err(escore_priv.dev,
				"%s(): gpioa_gpio(%d) request failed, rc %d",
				__func__, pdata->gpioa_gpio, rc);
			goto gpioa_gpio_request_error;
		}
		rc = gpio_direction_input(pdata->gpioa_gpio);
		if (rc < 0) {
			dev_err(escore_priv.dev,
				"%s(): gpioa_gpio direction failed, rc %d",
				__func__, rc);
			goto gpioa_gpio_direction_error;
		}
		/* Fix value to Rising Edge */
		pdata->gpio_a_irq_type = ES_RISING_EDGE;
		escore_priv.cmd_compl_mode = ES_CMD_COMP_INTR;
	} else {
		dev_warn(escore_priv.dev, "%s(): gpioa_gpio undefined\n",
			 __func__);
	}

	dev_dbg(escore_priv.dev, "%s(): gpiob_gpio = %d\n", __func__,
		pdata->gpiob_gpio);
	if (pdata->gpiob_gpio != -1) {
		rc = gpio_request(pdata->gpiob_gpio, "es855_gpiob");
		if (rc < 0) {
			dev_err(escore_priv.dev,
				"%s(): gpiob_gpio(%d) request failed, rc %d",
				__func__, pdata->gpiob_gpio, rc);
			goto gpiob_gpio_request_error;
		}
		rc = gpio_direction_input(pdata->gpiob_gpio);
		if (rc < 0) {
			dev_err(escore_priv.dev,
				"%s(): gpiob_gpio direction failed, rc %d",
				__func__, rc);
			goto gpiob_gpio_direction_error;
		}
	} else {
		dev_warn(escore_priv.dev, "%s(): gpiob_gpio undefined\n",
			 __func__);
	}

	rc = request_firmware((const struct firmware **)&escore_priv.standard,
			      fw_filename, escore_priv.dev);
	if (rc) {
		dev_err(escore_priv.dev,
			"%s(): request_firmware(%s) failed %d\n",
			__func__, fw_filename, rc);
		goto request_firmware_error;
	}

	escore_priv.boot_ops.bootup = es855_bootup;
	escore_priv.soc_codec_dev_escore = &soc_codec_dev_es855;
	escore_priv.dai = es855_dai;
	escore_priv.dai_nr = ES_NUM_CODEC_DAIS;
	escore_priv.api_addr_max = ES_API_ADDR_MAX;
	escore_priv.api_access = es855_api_access;
	escore_priv.reg_cache = a300_reg_cache;
	escore_priv.flag.is_codec = 1;
	escore_priv.escore_power_state = ES_SET_POWER_STATE_NORMAL;
	escore_priv.escore_event_type = ES_NO_EVENT;
	escore_priv.i2s_dai_data = i2s_dai_data;
	escore_priv.config_jack = es855_config_jack;

	if (escore_priv.pri_intf == ES_SLIM_INTF) {
		escore_priv.slim_rx = slim_rx;
		escore_priv.slim_tx = slim_tx;
		escore_priv.slim_dai_data = slim_dai_data;
		escore_priv.slim_setup = es855_slim_setup;

		escore_priv.slim_rx_ports = ES_SLIM_RX_PORTS;
		escore_priv.slim_tx_ports = ES_SLIM_TX_PORTS;
		escore_priv.codec_slim_dais = ES_NUM_CODEC_SLIM_DAIS;

		escore_priv.slim_tx_port_to_ch_map = es855_slim_tx_port_to_ch;
		escore_priv.slim_rx_port_to_ch_map = es855_slim_rx_port_to_ch;

#if defined(CONFIG_ARCH_MSM)
		escore_priv.slim_dai_ops.get_channel_map =
				es855_slim_get_channel_map;
#endif
		escore_priv.slim_dai_ops.set_channel_map =
				es855_slim_set_channel_map;

		/* Initialization of be_id goes here if required */
		escore_priv.slim_be_id = NULL;

		/* Initialization of _remote_ routines goes here if required */
		escore_priv.remote_cfg_slim_rx = NULL;
		escore_priv.remote_cfg_slim_tx = NULL;
		escore_priv.remote_close_slim_rx = NULL;
		escore_priv.remote_close_slim_tx = NULL;

		escore_priv.flag.local_slim_ch_cfg = 1;
		escore_priv.channel_dir = es855_channel_dir;
		escore_priv.slim_setup(&escore_priv);
	}

#if defined(CONFIG_SND_SOC_ES_I2S)
	escore_priv.i2s_dai_ops.digital_mute = es855_digital_mute;
	escore_priv.i2s_dai_ops.hw_params = es855_hw_params;
	escore_priv.i2s_dai_ops.shutdown = es855_shutdown;
#endif
#if defined(CONFIG_SND_SOC_ES_SLIM)
	escore_priv.slim_dai_ops.digital_mute =	es855_digital_mute;
#endif

	/* API Interrupt registration */
	if (pdata->gpioa_gpio != -1) {
		irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
		rc = request_threaded_irq(gpio_to_irq(pdata->gpioa_gpio), NULL,
				es855_cmd_completion_isr, irq_flags,
				"es855-cmd-completion-isr", &escore_priv);
		if (rc < 0) {
			pr_err("%s() API interrupt registration failed :%d",
			       __func__, rc);
			goto api_intr_error;
		}
	}

	/* Event Interrupt registration */
	if (pdata->gpiob_gpio != -1 && pdata->gpio_b_irq_type) {
		switch (pdata->gpio_b_irq_type) {
		case ES_ACTIVE_LOW:
			irq_flags = IRQF_TRIGGER_LOW;
			break;
		case ES_ACTIVE_HIGH:
			irq_flags = IRQF_TRIGGER_HIGH;
			break;
		case ES_FALLING_EDGE:
			irq_flags = IRQF_TRIGGER_FALLING;
			break;
		case ES_RISING_EDGE:
			irq_flags = IRQF_TRIGGER_RISING;
			break;
		}
		rc = request_threaded_irq(gpio_to_irq(pdata->gpiob_gpio), NULL,
				es855_irq_work, irq_flags,
				"escore-irq-work", &escore_priv);
		if (rc < 0) {
			pr_err("Error in registering interrupt :%d", rc);
			goto event_intr_error;
		} else {
			irq_set_irq_wake(gpio_to_irq(pdata->gpiob_gpio), 1);
		}

		/* Disable the interrupt till needed */
		if (escore_priv.pdata->gpiob_gpio != -1)
			disable_irq(gpio_to_irq(pdata->gpiob_gpio));

		escore_register_notify(escore_priv.irq_notifier_list,
				       &es855_codec_intr_cb);

	}

#if defined(CONFIG_SND_SOC_ES_UART_STREAMDEV)
	escore_priv.streamdev = es_uart_streamdev;
#endif
	return rc;

event_intr_error:
	free_irq(gpio_to_irq(pdata->gpioa_gpio), NULL);
api_intr_error:
	release_firmware(escore_priv.standard);
request_firmware_error:
gpiob_gpio_direction_error:
	if (pdata->gpiob_gpio != -1)
		gpio_free(pdata->gpiob_gpio);
gpiob_gpio_request_error:
gpioa_gpio_direction_error:
	if (pdata->gpioa_gpio != -1)
		gpio_free(pdata->gpioa_gpio);
gpioa_gpio_request_error:
wakeup_gpio_direction_error:
	if (pdata->wakeup_gpio != -1)
		gpio_free(pdata->wakeup_gpio);
wakeup_gpio_request_error:
reset_gpio_direction_error:
	if (pdata->reset_gpio != -1)
		gpio_free(pdata->reset_gpio);
reset_gpio_request_error:
	dev_dbg(escore_priv.dev, "%s(): exit with error\n", __func__);

	return rc;
}
EXPORT_SYMBOL_GPL(es855_core_probe);

static __init int es855_init(void)
{
	int rc = 0;

	pr_debug("%s()", __func__);

	escore_priv.device_name  = "elemental-addr";
	escore_priv.interface_device_name  = "slim-ifd";
	escore_priv.interface_device_elem_addr_name  =
					"slim-ifd-elemental-addr";

	mutex_init(&escore_priv.api_mutex);
	mutex_init(&escore_priv.intf_probed_mutex);
	init_completion(&escore_priv.fw_download);
	escore_platform_init();
#if defined(CONFIG_SND_SOC_ES_I2C)
	escore_priv.pri_intf = ES_I2C_INTF;
#elif defined(CONFIG_SND_SOC_ES_SLIM)
	escore_priv.pri_intf = ES_SLIM_INTF;
#elif defined(CONFIG_SND_SOC_ES_UART)
	escore_priv.pri_intf = ES_UART_INTF;
#elif defined(CONFIG_SND_SOC_ES_SPI)
	escore_priv.pri_intf = ES_SPI_INTF;
#endif

#if defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_I2C)
	escore_priv.high_bw_intf = ES_I2C_INTF;
#elif defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_SLIM)
	escore_priv.high_bw_intf = ES_SLIM_INTF;
#elif defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_UART)
	escore_priv.high_bw_intf = ES_UART_INTF;
#elif defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_SPI)
	escore_priv.high_bw_intf = ES_SPI_INTF;
#elif defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_DEFAULT)
	escore_priv.high_bw_intf = escore_priv.pri_intf;
#endif

#if defined(CONFIG_SND_SOC_ES_WAKEUP_UART)
	escore_priv.wakeup_intf = ES_UART_INTF;
#endif

#if defined(CONFIG_SND_SOC_ES_I2C) || defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_I2C)
	rc = escore_i2c_init();
#endif
#if defined(CONFIG_SND_SOC_ES_SLIM) || \
	defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_SLIM)
	rc = escore_slimbus_init();
#endif
#if defined(CONFIG_SND_SOC_ES_SPI) || defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_SPI)
	rc = escore_spi_init();
#endif
#if defined(CONFIG_SND_SOC_ES_UART) || \
	defined(CONFIG_SND_SOC_ES_HIGH_BW_BUS_UART) || \
	defined(CONFIG_SND_SOC_ES_WAKEUP_UART)
	rc = escore_uart_bus_init(&escore_priv);
#endif
	if (rc) {
		pr_debug("Error registering Audience eS855 driver: %d\n", rc);
		goto INIT_ERR;
	}

#if defined(CONFIG_SND_SOC_ES_CDEV)
	rc = escore_cdev_init(&escore_priv);
	if (rc) {
		pr_debug("Error enabling CDEV interface: %d\n", rc);
		goto INIT_ERR;
	}
#endif
INIT_ERR:
	return rc;
}
module_init(es855_init);

static __exit void es855_exit(void)
{
	pr_debug("%s()\n", __func__);

#if defined(CONFIG_SND_SOC_ES_I2C)
	i2c_del_driver(&escore_i2c_driver);
#elif defined(CONFIG_SND_SOC_ES_SPI)
	escore_spi_exit();
#endif

}
module_exit(es855_exit);


MODULE_DESCRIPTION("ASoC ES855 driver");
MODULE_AUTHOR("Greg Clemson <gclemson@audience.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:es855-codec");
MODULE_FIRMWARE("audience-es855-fw.bin");

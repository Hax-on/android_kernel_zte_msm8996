/*
 * es855-access.h  --  ES855 Soc Audio access values
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _ES855_ACCESS_H
#define _ES855_ACCESS_H


/*
 * es-d400.h and es-a350-reg.h are being developed in separated effort
 * these will be include when the headers are stable
*/
/*
#include "es-d400.h"
#include "es-a350-reg.h"
*/
#include "es855.h"

static struct escore_api_access es855_api_access[ES_API_ADDR_MAX] = {
	[ES_CHANGE_STATUS] = {
		.read_msg = { ES_API_WORD(0x804f, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(0x904f, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 4,
	},
	[ES_FW_FIRST_CHAR] = {
		.read_msg = { ES_API_WORD(0x8020, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(0x9020, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 255,
	},
	[ES_FW_NEXT_CHAR] = {
		.read_msg = { ES_API_WORD(0x8021, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(0x9021, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 255,
	},
	[ES_GET_SYS_INTERRUPT] = {
		.read_msg = { ES_API_WORD(0x8053, 0x0000) },
		.read_msg_len = 4,
		.val_shift = 0,
		.val_max = 0,
	},
	[ES_ACCDET_CONFIG] = {
		.read_msg = { ES_API_WORD(0x8055, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(0x9056, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 0xFF,
	},
	[ES_GET_ACCDET_STATUS] = {
		.read_msg = { ES_API_WORD(0x8054, 0x0000) },
		.read_msg_len = 4,
		.val_shift = 0,
		.val_max = 1,
	},
	[ES_BUTTON_DETECTION_ENABLE] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x1f30) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x1f30),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 1,
	},
	[ES_BUTTON_SERIAL_CONFIG] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x1f31) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x1f31),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 1,
	},
	[ES_BUTTON_PARALLEL_CONFIG] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x1f32) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x1f32),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 1,
	},
	[ES_BUTTON_DETECTION_RATE] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x1f33) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x1f33),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 3,
	},
	[ES_BUTTON_PRESS_SETTLING_TIME] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x1f34) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x1f34),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 7,
	},
	[ES_BUTTON_BOUNCE_TIME] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x1f35) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x1f35),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 11,
	},
	[ES_BUTTON_DETECTION_LONG_PRESS_TIME] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x1f36) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x1f36),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 15,
	},
	[ES_BUTTON_CTRL1] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x1631) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x1631),
			       ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_BUTTON_CTRL2] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x1632) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x1632),
			       ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_BUTTON_CTRL3] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x1633) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x1633),
			       ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_BUTTON_CTRL4] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x1634) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x1634),
			       ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_CODEC_VALUE] = {
		.read_msg = { ES_API_WORD(0x807B, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(0x907C, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_POWER_STATE] = {
		.read_msg = { ES_API_WORD(0x800f, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(0x9010, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 6,
	},
	[ES_EVENT_RESPONSE] = {
		.read_msg = { ES_API_WORD(0x801a, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(0x901a, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 4,
	},
	[ES_SET_NS] = {
		.read_msg = { ES_API_WORD(0x8043, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(0x901C, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_PRESET] = {
		.read_msg = { ES_API_WORD(0x8031, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(0x9031, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 65535,
	},

	[ES_STEREO_WIDTH] = {
		.read_msg = {ES_API_WORD(ES_GET_ALGO_PARAM, 0x1006)},
		.read_msg_len = 4,
		.write_msg = {
			ES_API_WORD(ES_SET_ALGO_PARAM_ID, 0x1006),
			ES_API_WORD(ES_SET_ALGO_PARAM, 0x0000),
		},
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},

	[ES_MIC_CONFIG] = {
		.read_msg = {ES_API_WORD(ES_GET_ALGO_PARAM, 0x0002)},
		.read_msg_len = 4,
		.write_msg = {
			ES_API_WORD(ES_SET_ALGO_PARAM_ID, 0x0002),
			ES_API_WORD(ES_SET_ALGO_PARAM, 0x0000),
		},
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},

	[ES_AEC_MODE] = {
		.read_msg = {ES_API_WORD(ES_GET_ALGO_PARAM, 0x0003)},
		.read_msg_len = 4,
		.write_msg = {
			ES_API_WORD(ES_SET_ALGO_PARAM_ID, 0x0003),
			ES_API_WORD(ES_SET_ALGO_PARAM, 0x0000),
		},
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},

	[ES_AZ_MODE] = {
		.read_msg = {ES_API_WORD(ES_GET_ALGO_PARAM, 0x2002)},
		.read_msg_len = 4,
		.write_msg = {
			ES_API_WORD(ES_SET_ALGO_PARAM_ID, 0x2002),
			ES_API_WORD(ES_SET_ALGO_PARAM, 0x0000),
		},
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},

	[ES_PORT_WORD_LEN] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x0000),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_PORT_TDM_SLOTS_PER_FRAME] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x0001) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x0001),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_PORT_TX_DELAY_FROM_FS] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x0002) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x0002),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_PORT_RX_DELAY_FROM_FS] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x0003) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x0003),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_PORT_LATCH_EDGE] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x0004) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x0004),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_PORT_ENDIAN] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x0005) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x0005),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_PORT_TRISTATE] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x0006) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x0006),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_PORT_AUDIO_MODE] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x0007) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x0007),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_PORT_TDM_ENABLED] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x0008) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x0008),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_PORT_CLOCK_CONTROL] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x0009) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x0009),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_PORT_DATA_JUSTIFICATION] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x000a) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x000a),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_PORT_FS_DURATION] = {
		.read_msg = { ES_API_WORD(ES_GET_DEV_PARAM, 0x000b) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(ES_SET_DEV_PARAM_ID, 0x000b),
			ES_API_WORD(ES_SET_DEV_PARAM, 0x0000) },
		.write_msg_len = 8,
		.val_shift = 0,
		.val_max = 65535,
	},
	[ES_FE_STREAMING] = {
		.read_msg = { ES_API_WORD(0x8028, 0x0000) },
		.read_msg_len = 4,
		.write_msg = { ES_API_WORD(0x9028, 0x0000) },
		.write_msg_len = 4,
		.val_shift = 0,
		.val_max = 65535,
	},

};

static struct escore_reg_cache a300_reg_cache[ES_MAX_REGISTER] = {
	{ 0x01, 0 },
};

#endif /* _ES855_ACCESS_H */

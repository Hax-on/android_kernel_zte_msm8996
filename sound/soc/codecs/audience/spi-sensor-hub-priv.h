/*
 * OSP Sensor Hub driver
 *
 * Copyright (C) 2014 Audience Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 */
#if !defined(__OSP_SH_SENSOR_ID_HUB_PRIV_H__)
#define   __OSP_SH_SENSOR_ID_HUB_PRIV_H__

/*---------------------------------------------------------------------------*\
 |    I N C L U D E   F I L E S
\*---------------------------------------------------------------------------*/
#ifndef __KERNEL__
#include <stdint.h>
#endif

#include <linux/osp-sensors.h>


/*---------------------------------------------------------------------------*\
 |    C O N S T A N T S   &   M A C R O S
\*---------------------------------------------------------------------------*/
#ifdef __KERNEL__
# define OSP_SH_SUSPEND_DELAY               100        /* msec suspend delay*/
#endif

#define OSP_SH_SENSOR_ID_FIRST \
		SENSOR_ACCELEROMETER_UNCALIBRATED
#define OSP_SH_SENSOR_ID_COUNT	\
		(SENSOR_ENUM_COUNT-OSP_SH_SENSOR_ID_FIRST)

/*---------------------------------------------------------------------------*\
 |    T Y P E   D E F I N I T I O N S
\*---------------------------------------------------------------------------*/

#define osp_pack (__attribute__ ((__packed__)))

#define OSP_SH_MAX_BROADCAST_BUFFER_SIZE 256


struct osp_pack TimeStamp40 {
		uint32_t    timeStamp32;
		uint8_t     timeStamp40;
};

uint64_t timestamp40_to_timestamp64(struct TimeStamp40 *timeStamp);

void timestamp40_minus_timestamp40(
				struct TimeStamp40 *timeStamp1,
				struct TimeStamp40 *timeStamp2,
				struct TimeStamp40 *timeStampResult);

void timestamp40_plus_timestamp8(
				struct TimeStamp40 *timeStamp1,
				int8_t timeStamp2,
				struct TimeStamp40 *timeStampResult);

void timestamp40_plus_timestamp16(
				struct TimeStamp40 *timeStamp1,
				int16_t timeStamp2,
				struct TimeStamp40 *timeStampResult);

void timestamp40_plus_timestamp32(
				struct TimeStamp40 *timeStamp1,
				int32_t timeStamp2,
				struct TimeStamp40 *timeStampResult);


struct osp_pack osp_sh_motion_sensor_broadcast_node {

		/*
		 * raw time stamp in sensor time capture ticks
		**/
		struct TimeStamp40 timeStamp;
		int16_t Data[3];    /* Raw sensor data */
};

struct osp_pack osp_sh_motion_uncal_sensor_broadcast_node {
		/*
		 * raw time stamp in sensor time capture ticks
		**/
		struct TimeStamp40 timeStamp;
		int16_t Data[3];		/* Raw sensor data */
		int16_t Bias[3];		/* Raw sensor bias */
};

#define SIZE_UNCAL_SBCAST_NODE \
		sizeof(struct osp_pack\
			osp_sh_motion_uncal_sensor_broadcast_node)

struct osp_pack osp_sh_segment_broadcast_node {
		int64_t endTime;		/* in NTTIME  */
		int64_t duration;		/* in NTTIME  */
		uint8_t type;
};


struct osp_pack osp_sh_significant_motion_broadcast_node {
		struct TimeStamp40 timeStamp;
		unsigned char significantMotionDetected;
};

struct osp_pack osp_sh_orientation_broadcast_node {
		/*
		 * raw time stamp in sensor time capture ticks
		**/
		struct TimeStamp40 timeStamp;
		/* Raw sensor data in NTEXTENDED*/
		int32_t Data[3];
};

struct osp_pack osp_sh_step_sensitive_sensor_broadcast {
		struct TimeStamp40 timeStamp;
		uint16_t numStepsTotal;
};

struct osp_pack sh_quaternion_data {
		/*
		 * raw time stamp in sensor time capture ticks
		**/
		struct TimeStamp40 timeStamp;
		int32_t W;
		/* w/x/y/z/e_est Raw sensor data  in NTPRECISE*/
		int32_t X;
		int32_t Y;
		int32_t Z;
		int32_t E_EST;
};

struct osp_pack osp_sh_motion_sensor_broadcast_delta_time4_node {
		uint32_t timeStamp;
		/* raw time stamp in sensor time capture ticks */
		int16_t Data[3];
		/* Raw sensor data */
};

struct osp_pack osp_sh_motion_sensor_broadcast_delta_time2_node {
		uint16_t timeStamp;
		/* raw time stamp in sensor time capture  ticks */
		int16_t Data[3];
		/* Raw sensor data */
};

struct osp_pack osp_sh_motion_sensor_broadcast_delta_time1_node {
		uint8_t timeStamp;
		/* raw time stamp in sensor time capture  ticks */
		int16_t Data[3];
		/* Raw sensor data */
};

struct osp_pack osp_sh_motion_sensor_broadcast_delta_data_node {
		/*
		 * raw time stamp in sensor time capture  ticks
		**/
		struct TimeStamp40 timeStamp;
		int8_t Data[3];
		/* Raw sensor data */
};

struct osp_pack osp_sh_motion_sensor_broadcast_delta_time4_data_node {
		uint32_t timeStamp;
		/* raw time stamp in sensor time capture  ticks */
		int8_t Data[3];
		/* Raw sensor data */
};

struct osp_pack osp_sh_motion_sensor_broadcast_delta_time2_data_node {
		uint16_t timeStamp;
		/* raw time stamp in sensor time capture  ticks */
		int8_t Data[3];
		/* Raw sensor data */
};

struct osp_pack osp_sh_motion_sensor_broadcast_delta_time1_data_node {
		uint8_t timeStamp;
		/* raw time stamp in sensor time capture  ticks */
		int8_t Data[3];
		/* Raw sensor data */
};

struct osp_pack osp_sh_sensor_broadcast_node {
	uint8_t sensorId;
	/* Holds Sensor type enumeration */
	uint8_t compression;
	union osp_pack {
		struct osp_sh_motion_sensor_broadcast_node\
				sensorData;
		struct osp_sh_motion_uncal_sensor_broadcast_node\
				uncal_sensorData;
		struct osp_sh_segment_broadcast_node\
				segmentData;
		struct sh_quaternion_data\
				quaternionData;
		struct osp_sh_orientation_broadcast_node\
				orientationData;
		struct osp_sh_step_sensitive_sensor_broadcast\
				stepSensitiveData;
		struct osp_sh_significant_motion_broadcast_node\
				significantMotionData;
	} data;
};

struct osp_pack osp_sh_sensor_broadcast_delta_time4_node {
	uint8_t sensorId;
	/* Holds Sensor type enumeration */
	struct osp_sh_motion_sensor_broadcast_delta_time4_node\
			data;
};

struct osp_pack osp_sh_sensor_broadcast_delta_time2_node {
	uint8_t sensorId;
	/* Holds Sensor type enumeration */
	struct osp_sh_motion_sensor_broadcast_delta_time2_node\
			data;
};

struct osp_pack osp_sh_sensor_broadcast_delta_time1_node {
	uint8_t sensorId;
	/* Holds Sensor type enumeration */
	struct osp_sh_motion_sensor_broadcast_delta_time1_node\
			data;
};


struct osp_pack osp_sh_sensor_broadcast_delta_data_node {
		uint8_t sensorId;
		/* Holds Sensor type enumeration */
		struct osp_sh_motion_sensor_broadcast_delta_data_node\
				data;
};

struct osp_pack osp_sh_sensor_broadcast_delta_time4_data_node {
		uint8_t sensorId;
		/* Holds Sensor type enumeration */
		struct osp_sh_motion_sensor_broadcast_delta_time4_data_node\
				data;
};

struct osp_pack osp_sh_sensor_broadcast_delta_time2_data_node {
		uint8_t sensorId;
		/* Holds Sensor type enumeration */
		struct osp_sh_motion_sensor_broadcast_delta_time2_data_node\
				data;
};

struct osp_pack osp_sh_sensor_broadcast_delta_time1_data_node {
		uint8_t sensorId;
		/* Holds Sensor type enumeration */
		struct osp_sh_motion_sensor_broadcast_delta_time1_data_node\
				data;
};

struct osp_pack ShCmdGetHeader_get_8bits_param_t {
	uint8_t param;
};

struct osp_pack ShCmdGetHeader_get_16bits_param_t {
	uint16_t param;
};

enum OSP_SH_HUB_COMMANDS {
		OSP_SH_GET_WHO_AM_I = 0x00,
		/* gets 8 bits Device ID */
		OSP_SH_GET_VERSION,
		/* gets 16 bits version number on following read */
		OSP_SH_RESET,
		/* there three commands most be gnerated */
		/* atomicly, in this sequence */
		OSP_SH_GET_BROADCAST_LENGTH,
		/* gets 16 bit of broadcast length */
		OSP_SH_GET_BROADCAST_DATA,
		/* gets as many bytes as broadcast length read */
} ;

struct osp_pack ShHubCmdHeader_t {
		uint8_t command;
		/* enum OSP_SH_HUB_COMMANDS */
};

struct osp_pack ShHubCmdHeader_8bits_param_t {
		uint8_t command;
		/* enum OSP_SH_HUB_COMMANDS */
		uint8_t param;
} ;


enum OSP_SH_SENSOR_COMMANDS {
		OSP_SH_SENSOR_SET_ENABLE = 0x20,
		OSP_SH_SENSOR_GET_ENABLE,
		OSP_SH_SENSOR_SET_DELAY,
		OSP_SH_SENSOR_GET_DELAY,
#if defined(TRANSMIT_CAL_TO_SH)
		OSP_SH_SENSOR_SET_CALIBRATE,
		OSP_SH_SENSOR_GET_CALIBRATE,
#endif
};

struct osp_pack ShSensorCmdHeader_t {
		uint8_t command;
		/* enum OSP_SH_SENSOR_ID_COMMANDS */
		uint8_t sensorId;
		/* Holds Sensor type enumeration */
};

struct osp_pack ShSensorSetCmdHeader_8bits_param_t {
		uint8_t command;
		/* enum OSP_SH_SENSOR_ID_COMMANDS */
		uint8_t sensorId;
		/* Holds Sensor type enumeration */
		uint8_t param;
};

struct osp_pack ShSensorSetCmdHeader_16bits_param_t {
		uint8_t command;
		/* enum OSP_SH_SENSOR_ID_COMMANDS */
		uint8_t sensorId;
		/* Holds Sensor type enumeration */
		uint16_t param;
};

union osp_pack ShCmdHeaderUnion {
		struct ShSensorCmdHeader_t command;
		struct ShSensorSetCmdHeader_8bits_param_t\
				sensor_cmd_8bits_param;
		struct ShSensorSetCmdHeader_16bits_param_t\
				sensor_cmd_16bits_param;
		struct ShHubCmdHeader_t\
				hubCmdHeader;
		struct ShHubCmdHeader_8bits_param_t\
				hub_cmd_8bits_param;
};

/*--------------------------------------------------------------------------*\
 |    E X T E R N A L   V A R I A B L E S   &   F U N C T I O N S
\*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*\
 |    P U B L I C   V A R I A B L E S   D E F I N I T I O N S
\*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*\
 |    P U B L I C   F U N C T I O N   D E C L A R A T I O N S
\*--------------------------------------------------------------------------*/
#endif /* __OSP_SH_SENSOR_ID_HUB_PRIV_H__ */
/*------------------------------------------------*\
 |    E N D   O F   F I L E
\*------------------------------------------------*/


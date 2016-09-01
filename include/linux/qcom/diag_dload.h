/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_DIAG_DLOAD_H__
#define __LINUX_DIAG_DLOAD_H__


#define PID_MAGIC_ID		0x71432909
#define SERIAL_NUM_MAGIC_ID	0x61945374
#define SERIAL_NUMBER_LENGTH	128

struct magic_num_struct {
	uint32_t pid;
	uint32_t serial_num;
};

struct dload_struct {
	uint32_t	pid;
	char		serial_number[SERIAL_NUMBER_LENGTH];
	struct magic_num_struct magic_struct;
	/* Use Qualcomm's usb vid and pid if enters download due to panic,4.1 of 5 */
	uint8_t   dload_info_free[2];
};

/* Use Qualcomm's usb vid and pid if enters download due to panic,4.2 of 5 */
void use_qualcomm_usb_product_id(void);

#endif

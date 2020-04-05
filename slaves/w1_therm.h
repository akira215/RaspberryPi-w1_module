/*
 * Copyright (c) 2020 Akira Corp. <akira215corp@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __W1_THERM_H
#define __W1_THERM_H

#include <asm/types.h>

#include <linux/device.h>
#include <linux/types.h>
#include <linux/w1.h>

/*      --------------Defines-----------------         */

#define W1_THERM_DS18S20	0x10
#define W1_THERM_DS1822		0x22
#define W1_THERM_DS18B20	0x28
#define W1_THERM_DS1825		0x3B
#define W1_THERM_DS28EA00	0x42

#define W1_THERM_MAX_TRY		5	/* Nb of try for an operation */
#define W1_THERM_RETRY_DELAY	20	/* Delay in ms to retry to acquire bus mutex */


/*      --------------Structs-----------------         */

/*
 * w1_therm_family_data 
 * rom : data
 * refcnt : ref count
 * external_powered : 1 - device powered externally, 
 *					 0 - device parasite powered, 
 *					-x - error or undefined
 * resolution : resolution in bit of the device, negative value are error code
*/
struct w1_therm_family_data {
	uint8_t rom[9];
	atomic_t refcnt;
	int external_powered;
	int resolution;
};

struct therm_info {
	u8 rom[9];
	u8 crc;
	u8 verdict;
};

/*      --------------Macros-----------------         */

/* return the address of the refcnt in the family data */
#define THERM_REFCNT(family_data) \
	(&((struct w1_therm_family_data *)family_data)->refcnt)

/* return the power mode of the sl slave : 1-ext, 0-parasite, <0 unknown 
	always test family data existance before*/
#define SLAVE_POWERMODE(sl) \
	(((struct w1_therm_family_data *)(sl->family_data))->external_powered)

/* return the resolution in bit of the sl slave : <0 unknown 
	always test family data existance before*/
#define SLAVE_RESOLUTION(sl) \
	(((struct w1_therm_family_data *)(sl->family_data))->resolution)

/*      --------------Interface sysfs-----------------         */

static ssize_t w1_slave_show(struct device *device,
	struct device_attribute *attr, char *buf);

static ssize_t w1_slave_store(struct device *device,
	struct device_attribute *attr, const char *buf, size_t size);

static ssize_t w1_seq_show(struct device *device,
	struct device_attribute *attr, char *buf);

// ASH new files ///////////////////////////////////////////////////
static ssize_t temperature_show(struct device *device,
	struct device_attribute *attr, char *buf);

static ssize_t ext_power_show(struct device *device,
	struct device_attribute *attr, char *buf);

static ssize_t resolution_show(struct device *device,
	struct device_attribute *attr, char *buf);

static ssize_t resolution_store(struct device *device,
	struct device_attribute *attr, const char *buf, size_t size);

/*      --------------Attributes declarations-----------------         */

static DEVICE_ATTR_RW(w1_slave);
static DEVICE_ATTR_RO(w1_seq);

// ASH //////////////////////////////////////////////////////////////
static DEVICE_ATTR_RO(ext_power);
static DEVICE_ATTR_RW(resolution); // TODO implement
static DEVICE_ATTR_RO(temperature);	// TODO implement
// ASH //////////////////////////////////////////////////////////////


/*      ---------------Functions-----------------         */

/* w1_therm_add_slave() - Called each time a search discover a new device
 * used to initialized slave (family datas)
 * @sl:	slave just discovered
 * return value : 0 - If success, negative kernel code otherwise
 */
static int w1_therm_add_slave(struct w1_slave *sl);

/* w1_therm_remove_slave() - Called each time a slave is removed
 * used to free memory
 * @sl:	slave to be removed
 */
static void w1_therm_remove_slave(struct w1_slave *sl);

/**
 * reset_select_slave() - reset and select a slave
 * @sl:		the slave to select
 *
 * Resets the bus and then selects the slave by sending either a ROM MATCH.  
 * w1_reset_select_slave() from w1_io.c could not be used
 * here because a SKIP ROM command is sent if only one device is on the line.
 * At the beginning of the such process, sl->master->slave_count is 1 even if 
 * more devices are on the line, causing collision on the line.
 * The w1 master lock must be held.
 *
 * Return:	0 if success, negative kernel error code otherwise
 */
static int reset_select_slave(struct w1_slave *sl);

/* read_powermode() - Ask the device to get its power mode {external, parasite}
 * @sl:		slave to be interrogated
 * return value :
 * 0 - parasite powered device
 * 1 - externally powered device
 * <0 - kernel error code
 */
static int read_powermode(struct w1_slave *sl);

/* read_scratchpad()
 * @sl: 	pointer to the slave to read
 * @info: 	pointer to a structure to store the read results
 * return value: 0 if success, -kernel error code otherwise
 */
static int read_scratchpad(struct w1_slave *sl, struct therm_info *info);

/* write_scratchpad()
 * @sl: 				pointer to the slave to read
 * @data: 				pointer to an array of 3 bytes, as 3 bytes MUST be written
 * @pullup_duration: 	duration in ms of pullup, only for parasited powered devices
 * return value: 0 if success, -kernel error code otherwise
 */
static int write_scratchpad(struct w1_slave *sl, const u8 *data);

/* convert_t()
 * @sl: 				pointer to the slave to read
 * @info: 	pointer to a structure to store the read results
 * return value: 0 if success, -kernel error code otherwise
 */
static int convert_t(struct w1_slave *sl, struct therm_info *info);

/* copy_scratchpad() - Copy the content of scratchpad in device EEPROM
 * @sl:		slave involved
 * return value : 0 if success, -kernel error code otherwise
 */
static int copy_scratchpad(struct w1_slave *sl);


#endif  /* __W1_THERM_H */

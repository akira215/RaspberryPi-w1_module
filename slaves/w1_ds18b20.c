 /*
 *	w1_ds18b20.c
 *
 * Copyright (c) 2004 Akira Corp. <akira215corp@gmail.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the therms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/string.h>

#include "w1_ds18b20.h"

/* Allow the strong pullup to be disabled, but default to enabled.
 * If it was disabled a parasite powered device might not get the require
 * current to do a temperature conversion.  If it is enabled parasite powered
 * devices have a better chance of getting the current required.
 * In case the parasite power-detection is not working (seems to be the case
 * for some DS18S20) the strong pullup can also be forced, regardless of the
 * power state of the devices.
 *
 * Summary of options:
 * - strong_pullup = 0	Disable strong pullup completely
 * - strong_pullup = 1	Enable automatic strong pullup detection
 * - strong_pullup = 2	Force strong pullup
 */
static int w1_strong_pullup = 1;
module_param_named(strong_pullup, w1_strong_pullup, int, 0);


/*
 * sysfile interface:
 * w1_slave TODO fill
 * temperature (RO):
 *	. temperature in 1/1000 °
 *
 * ext_power (RO):
 *	. -xx : xx is kernel error refer to /usr/include/asm/errno.h
 *	. 0 : device parasite powered
 *	. 1 : device externally powered
 *
 * resolution (RW):
 *	. -xx 	: xx is kernel error refer to /usr/include/asm/errno.h
 *	. 9..12 : resolution set in bit (or resolution to set in bit)
 *	
 * eeprom (WO): be aware that eeprom writing cycles count is limited
 *	. 'write'	:	save device RAM to EEPROM
 *	. 'read'	:	restore EEPROM data in device RAM 
 *				(device do that automatically on power-up)
 *
*/ 

static struct attribute *w1_therm_attrs[] = {
	&dev_attr_w1_slave.attr,
	&dev_attr_temperature.attr,	
	&dev_attr_ext_power.attr,
	&dev_attr_resolution.attr,	
	&dev_attr_eeprom.attr,
	NULL,
};

static struct attribute *w1_ds28ea00_attrs[] = {
	&dev_attr_w1_slave.attr,
	&dev_attr_w1_seq.attr,
	&dev_attr_ext_power.attr,	//ASH
	NULL,
};

ATTRIBUTE_GROUPS(w1_therm);
ATTRIBUTE_GROUPS(w1_ds28ea00);

#if IS_REACHABLE(CONFIG_HWMON)
static int w1_read_temp(struct device *dev, u32 attr, int channel,
			long *val);

static umode_t w1_is_visible(const void *_data, enum hwmon_sensor_types type,
			     u32 attr, int channel)
{
	return attr == hwmon_temp_input ? 0444 : 0;
}

static int w1_read(struct device *dev, enum hwmon_sensor_types type,
		   u32 attr, int channel, long *val)
{
	switch (type) {
	case hwmon_temp:
		return w1_read_temp(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static const u32 w1_temp_config[] = {
	HWMON_T_INPUT,
	0
};

static const struct hwmon_channel_info w1_temp = {
	.type = hwmon_temp,
	.config = w1_temp_config,
};

static const struct hwmon_channel_info *w1_info[] = {
	&w1_temp,
	NULL
};

static const struct hwmon_ops w1_hwmon_ops = {
	.is_visible = w1_is_visible,
	.read = w1_read,
};

static const struct hwmon_chip_info w1_chip_info = {
	.ops = &w1_hwmon_ops,
	.info = w1_info,
};
#define W1_CHIPINFO	(&w1_chip_info)
#else
#define W1_CHIPINFO	NULL
#endif

static struct w1_family_ops w1_therm_fops = {
	.add_slave	= w1_therm_add_slave,
	.remove_slave	= w1_therm_remove_slave,
	.groups		= w1_therm_groups,
	.chip_info	= W1_CHIPINFO,
};
/*
static struct w1_family_ops w1_ds28ea00_fops = {
	.add_slave	= w1_therm_add_slave,
	.remove_slave	= w1_therm_remove_slave,
	.groups		= w1_ds28ea00_groups,
	.chip_info	= W1_CHIPINFO,
};

static struct w1_family w1_therm_family_DS18S20 = {
	.fid = W1_THERM_DS18S20,
	.fops = &w1_therm_fops,
};
*/
static struct w1_family w1_therm_family_DS18B20 = {
	.fid = W1_THERM_DS18B20,
	.fops = &w1_therm_fops,
};
/*
static struct w1_family w1_therm_family_DS1822 = {
	.fid = W1_THERM_DS1822,
	.fops = &w1_therm_fops,
};

static struct w1_family w1_therm_family_DS28EA00 = {
	.fid = W1_THERM_DS28EA00,
	.fops = &w1_ds28ea00_fops,
};

static struct w1_family w1_therm_family_DS1825 = {
	.fid = W1_THERM_DS1825,
	.fops = &w1_therm_fops,
};
*/
struct w1_therm_family_converter {
	u8			broken;
	u16			reserved;
	struct w1_family	*f;
	int			(*convert)(u8 rom[9]);
	int			(*precision)(struct device *device, int val);
	int			(*set_resolution)(struct w1_slave *sl, int val);
	int			(*get_resolution)(struct w1_slave *sl);
	int			(*eeprom)(struct device *device);
};

/* write configuration to eeprom */
static inline int w1_therm_eeprom(struct device *device);

/* TODO del the precision Set precision for conversion */
static inline int w1_DS18B20_precision(struct device *device, int val);
static inline int w1_DS18S20_precision(struct device *device, int val);


/* The return value is millidegrees Centigrade. */
static inline int w1_DS18B20_convert_temp(u8 rom[9]);
static inline int w1_DS18S20_convert_temp(u8 rom[9]);

static struct w1_therm_family_converter w1_therm_families[] = {
/*
	{
		.f				= &w1_therm_family_DS18S20,
		.convert		= w1_DS18S20_convert_temp,
		.precision		= w1_DS18S20_precision,
		.set_resolution	= w1_DS18S20_set_resolution,
		.get_resolution	= w1_DS18S20_get_resolution,
		.eeprom			= w1_therm_eeprom
	},
	{
		.f				= &w1_therm_family_DS1822,
		.convert		= w1_DS18B20_convert_temp,
		.precision		= w1_DS18S20_precision,
		.set_resolution	= w1_DS18S20_set_resolution,
		.get_resolution	= w1_DS18S20_get_resolution,
		.eeprom			= w1_therm_eeprom
	},
*/
	{
		.f				= &w1_therm_family_DS18B20,
		.convert		= w1_DS18B20_convert_temp,
		.precision		= w1_DS18B20_precision,
		.set_resolution	= w1_DS18B20_set_resolution,
		.get_resolution	= w1_DS18B20_get_resolution,
		.eeprom			= w1_therm_eeprom
	}/*,

	{
		.f				= &w1_therm_family_DS28EA00,
		.convert		= w1_DS18B20_convert_temp,
		.precision		= w1_DS18S20_precision,
		.set_resolution	= w1_DS18S20_set_resolution,
		.get_resolution	= w1_DS18S20_get_resolution,
		.eeprom			= w1_therm_eeprom
	},
	{
		.f				= &w1_therm_family_DS1825,
		.convert		= w1_DS18B20_convert_temp,
		.precision		= w1_DS18S20_precision,
		.set_resolution	= w1_DS18S20_set_resolution,
		.get_resolution	= w1_DS18S20_get_resolution,
		.eeprom			= w1_therm_eeprom
	}
*/
};
/*  device_family() 
 *  @brief Helper function that provide a pointer on the w1_therm_family_converter struct
 *  @param sl represents the device 
 *  @return pointer to the slaves's family converter, NULL if not known
*/
static struct w1_therm_family_converter *device_family(struct w1_slave *sl)
{
	struct w1_therm_family_converter *ret = NULL;
	int i;

	for (i = 0; i < ARRAY_SIZE(w1_therm_families); ++i) {
		if (w1_therm_families[i].f->fid == sl->family->fid) {
				ret = &w1_therm_families[i];
			break;
		}
	}
	return ret;
}

static int w1_therm_add_slave(struct w1_slave *sl)
{
	sl->family_data = kzalloc(sizeof(struct w1_therm_family_data),
		GFP_KERNEL);
	if (!sl->family_data)
		return -ENOMEM;
	atomic_set(THERM_REFCNT(sl->family_data), 1);

	/* Getting the power mode of the device {external, parasite}*/
	SLAVE_POWERMODE(sl) = read_powermode(sl);

	if ( SLAVE_POWERMODE(sl) < 0)
	{	/* no error returned because device has been added, put a non*/
		dev_warn(&sl->dev,
			"%s: Device has been added, but power_mode may be corrupted. err=%d\n",
			 __func__, SLAVE_POWERMODE(sl));
	}

	/* Getting the resolution of the device */
	SLAVE_RESOLUTION(sl) = device_family(sl)->get_resolution(sl);

	if ( SLAVE_RESOLUTION(sl) < 0)
	{	/* no error returned because device has been added, put a non*/
		dev_warn(&sl->dev,
			"%s:Device has been added, but resolution may be corrupted. err=%d\n",
			__func__, SLAVE_RESOLUTION(sl));
	}

	return 0;
}

static void w1_therm_remove_slave(struct w1_slave *sl)
{
	int refcnt = atomic_sub_return(1, THERM_REFCNT(sl->family_data));

	while (refcnt) {
		msleep(1000);
		refcnt = atomic_read(THERM_REFCNT(sl->family_data));
	}
	kfree(sl->family_data);
	sl->family_data = NULL;
}

static inline int w1_therm_eeprom(struct device *device)
{
	struct w1_slave *sl = dev_to_w1_slave(device);
	struct w1_master *dev = sl->master;
	u8 rom[9], external_power;
	int ret, max_trying = 10;
	u8 *family_data = sl->family_data;

	if (!sl->family_data) {
		ret = -ENODEV;
		goto error;
	}

	/* prevent the slave from going away in sleep */
	atomic_inc(THERM_REFCNT(family_data));

	ret = mutex_lock_interruptible(&dev->bus_mutex);
	if (ret != 0)
		goto dec_refcnt;

	memset(rom, 0, sizeof(rom));

	while (max_trying--) {
		if (!w1_reset_select_slave(sl)) {
			unsigned int tm = 10;
			unsigned long sleep_rem;

			/* check if in parasite mode */
			w1_write_8(dev, W1_READ_PSUPPLY);
			external_power = w1_read_8(dev);

			if (w1_reset_select_slave(sl))
				continue;

			/* 10ms strong pullup/delay after the copy command */
			if (w1_strong_pullup == 2 ||
			    (!external_power && w1_strong_pullup))
				w1_next_pullup(dev, tm);

			w1_write_8(dev, W1_COPY_SCRATCHPAD);

			if (external_power) {
				mutex_unlock(&dev->bus_mutex);

				sleep_rem = msleep_interruptible(tm);
				if (sleep_rem != 0) {
					ret = -EINTR;
					goto dec_refcnt;
				}

				ret = mutex_lock_interruptible(&dev->bus_mutex);
				if (ret != 0)
					goto dec_refcnt;
			} else if (!w1_strong_pullup) {
				sleep_rem = msleep_interruptible(tm);
				if (sleep_rem != 0) {
					ret = -EINTR;
					goto mt_unlock;
				}
			}

			break;
		}
	}

mt_unlock:
	mutex_unlock(&dev->bus_mutex);
dec_refcnt:
	atomic_dec(THERM_REFCNT(family_data));
error:
	return ret;
}

/* DS18S20 does not feature configuration register */
static inline int w1_DS18S20_precision(struct device *device, int val)
{
	return 0;
}

static inline int w1_DS18B20_precision(struct device *device, int val)
{
	struct w1_slave *sl = dev_to_w1_slave(device);
	struct w1_master *dev = sl->master;
	u8 rom[9], crc;
	int ret, max_trying = 10;
	u8 *family_data = sl->family_data;
	uint8_t precision_bits;
	uint8_t mask = 0x60;

	if (val > 12 || val < 9) {
		pr_warn("%s: Unsupported precision\n", __func__);
		ret = -EINVAL;
		goto error;
	}

	if (!sl->family_data) {
		ret = -ENODEV;
		goto error;
	}

	/* prevent the slave from going away in sleep */
	atomic_inc(THERM_REFCNT(family_data));

	ret = mutex_lock_interruptible(&dev->bus_mutex);
	if (ret != 0)
		goto dec_refcnt;

	memset(rom, 0, sizeof(rom));

	/* translate precision to bitmask (see datasheet page 9) */
	switch (val) {
	case 9:
		precision_bits = 0x00;
		break;
	case 10:
		precision_bits = 0x20;
		break;
	case 11:
		precision_bits = 0x40;
		break;
	case 12:
	default:
		precision_bits = 0x60;
		break;
	}

	while (max_trying--) {
		crc = 0;

		if (!w1_reset_select_slave(sl)) {
			int count = 0;

			/* read values to only alter precision bits */
			w1_write_8(dev, W1_READ_SCRATCHPAD);
			count = w1_read_block(dev, rom, 9);
			if (count != 9)
				dev_warn(device, "w1_read_block(): returned %u instead of 9.\n",	count);

			crc = w1_calc_crc8(rom, 8);
			if (rom[8] == crc) {
				rom[4] = (rom[4] & ~mask) | (precision_bits & mask);

				if (!w1_reset_select_slave(sl)) {
					w1_write_8(dev, W1_WRITE_SCRATCHPAD);
					w1_write_8(dev, rom[2]);
					w1_write_8(dev, rom[3]);
					w1_write_8(dev, rom[4]);

					break;
				}
			}
		}
	}

	mutex_unlock(&dev->bus_mutex);
dec_refcnt:
	atomic_dec(THERM_REFCNT(family_data));
error:
	return ret;
}

static inline int w1_DS18B20_set_resolution(struct w1_slave *sl, int val)
{
	int ret = -ENODEV;
	u8 new_config_register[3];	/* array of data to be written */
	struct therm_info info;

	/* resolution of DS18B20 is in the range [9..12] bits */
	if ( val < 9 || val > 12 )
		return -EINVAL;

	val -= 9; /* soustract 9 the lowest resolution in bit */
	val = (val << 5); /* shift to position bit 5 & bit 6 */

	/* Read the scratchpad to change only the required bits 
	( bit5 & bit 6 from byte 4) */
	ret = read_scratchpad( sl, &info );
	if (!ret){
		new_config_register[0] = info.rom[2];
		new_config_register[1] = info.rom[3];
		new_config_register[2] = (info.rom[4] & 0b10011111) | \
					(u8) val; /* config register is byte 4 */
	}
	else
		return ret;

	/* Write data in the device RAM */
	ret = write_scratchpad(sl, new_config_register);

	return ret;
}

static inline int w1_DS18S20_set_resolution(struct w1_slave *sl, int val)
{
	// TODO implement 
	return 0;
}

static inline int w1_DS18B20_get_resolution(struct w1_slave *sl)
{
	int ret = -ENODEV;
	u8 config_register;
	struct therm_info info;

	ret = read_scratchpad( sl, &info );

	if (!ret)	{
		config_register = info.rom[4]; // config register is byte 4 
		config_register &= 0b01100000; // keep only bit 5 & 6 
		config_register = (config_register >> 5);	// shift to get 0b00 to 0b11 => 0 to 3 
		config_register += 9; // add 9 the lowest resolution in bit 
		ret = (int) config_register;
	}
	
	return ret;
}


static inline int w1_DS18S20_get_resolution(struct w1_slave *sl)
{
/* TODO Implement :
Resolutions greater than 9 bits can be calculated using the data 
from the temperature, COUNT REMAIN and COUNT PER °C registers 
in the scratchpad. Note that the COUNT PER  °C  register  is  
hard-wired  to  16  (10h).  After  reading  the  scratchpad,  
the  TEMP_READ  value  is  obtained  by  truncating  the  0.5°C  bit  
(bit  0)  from  the  temperature  data  (see Figure  4).  
The  extended  resolution  temperature  can  then be calculated u
sing the following equation:
*/
	
	return 0;
}

static inline int w1_DS18B20_convert_temp(u8 rom[9])
{
	s16 t = le16_to_cpup((__le16 *)rom);

	return t*1000/16;
}

static inline int w1_DS18S20_convert_temp(u8 rom[9])
{
	int t, h;

	if (!rom[7])
		return 0;

	if (rom[1] == 0)
		t = ((s32)rom[0] >> 1)*1000;
	else
		t = 1000*(-1*(s32)(0x100-rom[0]) >> 1);

	t -= 250;
	h = 1000*((s32)rom[7] - (s32)rom[6]);
	h /= (s32)rom[7];
	t += h;

	return t;
}

static inline int w1_convert_temp(u8 rom[9], u8 fid)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(w1_therm_families); ++i)
		if (w1_therm_families[i].f->fid == fid)
			return w1_therm_families[i].convert(rom);

	return 0;
}

static ssize_t w1_slave_store(struct device *device,
			      struct device_attribute *attr, const char *buf,
			      size_t size)
{
	int val, ret;
	struct w1_slave *sl = dev_to_w1_slave(device);
	int i;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(w1_therm_families); ++i) {
		if (w1_therm_families[i].f->fid == sl->family->fid) {
			/* zero value indicates to write current configuration to eeprom */
			if (val == 0)
				ret = w1_therm_families[i].eeprom(device);
			else
				ret = w1_therm_families[i].precision(device, val);
			break;
		}
	}
	return ret ? : size;
}

/*------------------------ Helpers Functions----------------------------*/
/*
static inline bool get_bus_mutex_lock(struct mutex *lock)
{
	int max_trying = W1_THERM_MAX_TRY;
	// try to acquire the mutex, if not, sleep retry_delay before retry) 
	while(mutex_lock_interruptible(lock) != 0 && max_trying > 0 ){
		unsigned long sleep_rem;
		sleep_rem = msleep_interruptible(W1_THERM_RETRY_DELAY);
		if (!sleep_rem)
			max_trying--;
	}

	if (!max_trying)
		return false;	// Didn't acquire the bus mutex //

	return true;
}
*/
static inline int get_convertion_time(struct w1_slave *sl)
{
	// TODO Check the compatibility with other devices
	int ret;
	if (!sl->family_data)
		return -ENODEV;	/* device unknown */
	
	switch( SLAVE_RESOLUTION(sl) ){
		case 9:
			ret = 95;
			break;
		case 10:
			ret = 190;
			break;
		case 11:
			ret = 375;
			break;
		case 12:	
		default:
			ret = 750;
	}
	return ret;
}

/*------------------------Hardware Functions--------------------------*/

/* Safe version of reser_select_slave - avoid using the one in w_io.c */
static int reset_select_slave(struct w1_slave *sl)
{
	u8 match[9] = { W1_MATCH_ROM, };
	u64 rn = le64_to_cpu(*((u64*)&sl->reg_num));

	if (w1_reset_bus(sl->master))
		return -ENODEV;

	memcpy(&match[1], &rn, 8);
	w1_write_block(sl->master, match, 9);

	return 0;
}

static int read_scratchpad(struct w1_slave *sl, struct therm_info *info)
{
	struct w1_master *dev_master = sl->master;
	int max_trying = W1_THERM_MAX_TRY;
	u8 ret = -ENODEV;
	info->verdict = 0;

	if (!sl->family_data) 
		goto error;

	memset(info->rom, 0, sizeof(info->rom));

	/* prevent the slave from going away in sleep */
	atomic_inc(THERM_REFCNT(sl->family_data));

	if (!w1_get_bus_mutex_lock (&dev_master->bus_mutex)){
		ret = -EAGAIN;	// Didn't acquire the mutex
		goto dec_refcnt;
	}

	while ( max_trying-- && ret ) { /* ret should be 0 */

		if (!reset_select_slave(sl)) {	/* safe version to select slave */
			u8 nb_bytes_read;
			w1_write_8(dev_master, W1_READ_SCRATCHPAD);

			nb_bytes_read = w1_read_block(dev_master, info->rom, 9);
			if (nb_bytes_read != 9) {
				dev_warn(&sl->dev, "w1_read_block(): "
					"returned %u instead of 9.\n",
					nb_bytes_read);
				ret = -EIO;
			}

			info->crc = w1_calc_crc8(info->rom, 8);

			if (info->rom[8] == info->crc)
			{
				info->verdict = 1;
				ret = 0;
			}
			else
				ret = -EIO; /* CRC not checked */
		}

	}
	mutex_unlock(&dev_master->bus_mutex);	

dec_refcnt:
	atomic_dec(THERM_REFCNT(sl->family_data));
error:
	return ret;
}

static int write_scratchpad(struct w1_slave *sl, const u8 *data)
{
	struct w1_master *dev_master = sl->master;
	int max_trying = W1_THERM_MAX_TRY;
	u8 ret = -ENODEV;

	if (!sl->family_data) 
		goto error;

	/* prevent the slave from going away in sleep */
	atomic_inc(THERM_REFCNT(sl->family_data));

	if (!w1_get_bus_mutex_lock (&dev_master->bus_mutex)){
		ret = -EAGAIN;	// Didn't acquire the mutex
		goto dec_refcnt;
	}

	while ( max_trying-- && ret ) { /* ret should be 0 */

		if (!reset_select_slave(sl)) {	/* safe version to select slave */
			w1_write_8(dev_master, W1_WRITE_SCRATCHPAD);
			w1_write_block(dev_master, data, 3);
			ret =0;
		}
	}
	mutex_unlock(&dev_master->bus_mutex);	

dec_refcnt:
	atomic_dec(THERM_REFCNT(sl->family_data));
error:
	return ret;
}

static int convert_t(struct w1_slave *sl, struct therm_info *info)
{
	struct w1_master *dev_master = sl->master;
	int max_trying = W1_THERM_MAX_TRY;
	int t_conv, ret = -ENODEV;
	bool strong_pullup;

	if (!sl->family_data) 
		goto error;
	
	t_conv = get_convertion_time(sl); /* conversion duration */
	strong_pullup = (w1_strong_pullup == 2 ||
					(!SLAVE_POWERMODE(sl) && w1_strong_pullup));

	memset(info->rom, 0, sizeof(info->rom));

	// prevent the slave from going away in sleep 
	atomic_inc(THERM_REFCNT(sl->family_data));

	if (!w1_get_bus_mutex_lock (&dev_master->bus_mutex)){
		ret = -EAGAIN;	// Didn't acquire the mutex
		goto dec_refcnt;
	}

	while ( max_trying-- && ret ) { /* ret should be 0 */

		info->verdict = 0;
		info->crc = 0;

		if (!reset_select_slave(sl)) {	/* safe version to select slave */
			unsigned long sleep_rem;

			/* 750ms strong pullup (or delay) after the convert */
			if (strong_pullup)
				w1_next_pullup(dev_master, t_conv);
			
			w1_write_8(dev_master, W1_CONVERT_TEMP);

			if (strong_pullup) {
				sleep_rem = msleep_interruptible(t_conv);
				if (sleep_rem != 0) {
					ret = -EINTR;
					goto mt_unlock;
				}
				mutex_unlock(&dev_master->bus_mutex);
			} else {
				mutex_unlock(&dev_master->bus_mutex);

				sleep_rem = msleep_interruptible(t_conv);
				if (sleep_rem != 0) {
					ret = -EINTR;
					goto dec_refcnt;
				}
			}
			ret = read_scratchpad( sl, info);
		}

	}

mt_unlock:
	mutex_unlock(&dev_master->bus_mutex);	
dec_refcnt:
	atomic_dec(THERM_REFCNT(sl->family_data));
error:
	return ret;
}

static int copy_scratchpad(struct w1_slave *sl)
{
	struct w1_master *dev_master = sl->master;
	int max_trying = W1_THERM_MAX_TRY;
	int t_write, ret = -ENODEV;
	bool strong_pullup;

	if (!sl->family_data) 
		goto error;
	
	t_write = W1_THERM_EEPROM_WRITE_DELAY; // TODO change that define to func
	strong_pullup = (w1_strong_pullup == 2 ||
					(!SLAVE_POWERMODE(sl) && w1_strong_pullup));

	// prevent the slave from going away in sleep 
	atomic_inc(THERM_REFCNT(sl->family_data));

	if (!w1_get_bus_mutex_lock (&dev_master->bus_mutex)){
		ret = -EAGAIN;	// Didn't acquire the mutex
		goto dec_refcnt;
	}

	while ( max_trying-- && ret ) { /* ret should be 0 */

		if (!reset_select_slave(sl)) {	/* safe version to select slave */
			unsigned long sleep_rem;

			/* 750ms strong pullup (or delay) after the convert */
			if (strong_pullup)
				w1_next_pullup(dev_master, t_write);
			
			w1_write_8(dev_master, W1_COPY_SCRATCHPAD);

			if (strong_pullup) {
				sleep_rem = msleep_interruptible(t_write);
				if (sleep_rem != 0) {
					ret = -EINTR;
					goto mt_unlock;
				}
			} 
			ret = 0;	
		}

	}

mt_unlock:
	mutex_unlock(&dev_master->bus_mutex);	
dec_refcnt:
	atomic_dec(THERM_REFCNT(sl->family_data));
error:
	return ret;
}

static int recall_eeprom(struct w1_slave *sl)
{
	struct w1_master *dev_master = sl->master;
	int max_trying = W1_THERM_MAX_TRY;
	int ret = -ENODEV;

	if (!sl->family_data) 
		goto error;

	// prevent the slave from going away in sleep 
	atomic_inc(THERM_REFCNT(sl->family_data));

	if (!w1_get_bus_mutex_lock (&dev_master->bus_mutex)){
		ret = -EAGAIN;	// Didn't acquire the mutex
		goto dec_refcnt;
	}

	while ( max_trying-- && ret ) { /* ret should be 0 */

		if (!reset_select_slave(sl)) {	/* safe version to select slave */

			w1_write_8(dev_master, W1_RECALL_EEPROM);
			
			ret = 1; /* Slave will pull line to 0 during recalling */
			while (ret)
				ret = 1 - w1_touch_bit(dev_master, 1);
		}

	}

	mutex_unlock(&dev_master->bus_mutex);	

dec_refcnt:
	atomic_dec(THERM_REFCNT(sl->family_data));
error:
	return ret;
}

static int read_powermode(struct w1_slave *sl)
{
	struct w1_master *dev_master = sl->master;
	int max_trying = W1_THERM_MAX_TRY;
	int  ret = -ENODEV;

	if (!sl->family_data) 
		goto error;

	/* prevent the slave from going away in sleep */
	atomic_inc(THERM_REFCNT(sl->family_data));

	if (!w1_get_bus_mutex_lock (&dev_master->bus_mutex)){
		ret = -EAGAIN;	// Didn't acquire the mutex
		goto dec_refcnt;
	}

	while ( (max_trying--) && (ret < 0) ) { /* ret should be either 1 either 0 */

		if (!reset_select_slave(sl)) {	/* safe version to select slave */
			w1_write_8(dev_master, W1_READ_PSUPPLY);
			/* Read only one bit, 1 is externally powered, 0 is parasite powered */
			ret = w1_touch_bit(dev_master, 1);
		}
	}
	mutex_unlock(&dev_master->bus_mutex);	

dec_refcnt:
	atomic_dec(THERM_REFCNT(sl->family_data));
error:
	return ret;
}

/*------------------------Interface Functions--------------------------*/

static ssize_t read_therm(struct device *device,
			  struct w1_slave *sl, struct therm_info *info)
{
	struct w1_master *dev = sl->master;
	u8 external_power;
	int ret, max_trying = 10;
	u8 *family_data = sl->family_data;

	if (!family_data) {
		ret = -ENODEV;
		goto error;
	}

	/* prevent the slave from going away in sleep */
	atomic_inc(THERM_REFCNT(family_data));

	ret = mutex_lock_interruptible(&dev->bus_mutex);
	if (ret != 0)
		goto dec_refcnt;

	memset(info->rom, 0, sizeof(info->rom));

	while (max_trying--) {

		info->verdict = 0;
		info->crc = 0;

		if (!w1_reset_select_slave(sl)) {
			int count = 0;
			unsigned int tm = 750;
			unsigned long sleep_rem;

			w1_write_8(dev, W1_READ_PSUPPLY);
			external_power = w1_read_8(dev);

			if (w1_reset_select_slave(sl))
				continue;

			/* 750ms strong pullup (or delay) after the convert */
			if (w1_strong_pullup == 2 ||
					(!external_power && w1_strong_pullup))
				w1_next_pullup(dev, tm);

			w1_write_8(dev, W1_CONVERT_TEMP);

			if (external_power) {
				mutex_unlock(&dev->bus_mutex);

				sleep_rem = msleep_interruptible(tm);
				if (sleep_rem != 0) {
					ret = -EINTR;
					goto dec_refcnt;
				}

				ret = mutex_lock_interruptible(&dev->bus_mutex);
				if (ret != 0)
					goto dec_refcnt;
			} else if (!w1_strong_pullup) {
				sleep_rem = msleep_interruptible(tm);
				if (sleep_rem != 0) {
					ret = -EINTR;
					goto mt_unlock;
				}
			}

			if (!w1_reset_select_slave(sl)) {

				w1_write_8(dev, W1_READ_SCRATCHPAD);
				count = w1_read_block(dev, info->rom, 9);
				if (count != 9) {
					dev_warn(device, "w1_read_block(): "
						"returned %u instead of 9.\n",
						count);
				}

				info->crc = w1_calc_crc8(info->rom, 8);

				if (info->rom[8] == info->crc)
					info->verdict = 1;
			}
		}

		if (info->verdict)
			break;
	}

mt_unlock:
	mutex_unlock(&dev->bus_mutex);
dec_refcnt:
	atomic_dec(THERM_REFCNT(family_data));
error:
	return ret;
}

static ssize_t w1_slave_show(struct device *device,
			     struct device_attribute *attr, char *buf)
{
	struct w1_slave *sl = dev_to_w1_slave(device);
	struct therm_info info;
	u8 *family_data = sl->family_data;
	int ret, i;
	ssize_t c = PAGE_SIZE;
	u8 fid = sl->family->fid;

	ret = read_therm(device, sl, &info);
	if (ret)
		return ret;

	for (i = 0; i < 9; ++i)
		c -= snprintf(buf + PAGE_SIZE - c, c, "%02x ", info.rom[i]);
	c -= snprintf(buf + PAGE_SIZE - c, c, ": crc=%02x %s\n",
		      info.crc, (info.verdict) ? "YES" : "NO");
	if (info.verdict)
		memcpy(family_data, info.rom, sizeof(info.rom));
	else
		dev_warn(device, "Read failed CRC check\n");

	for (i = 0; i < 9; ++i)
		c -= snprintf(buf + PAGE_SIZE - c, c, "%02x ",
			      ((u8 *)family_data)[i]);

	c -= snprintf(buf + PAGE_SIZE - c, c, "t=%d\n",
			w1_convert_temp(info.rom, fid));
	ret = PAGE_SIZE - c;
	return ret;
}


 // ASH //////////////////////////////////////////////////////////////////////////////////

static ssize_t temperature_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct w1_slave *sl = dev_to_w1_slave(device);
	struct therm_info info;
	int ret = 0;


	dev_info(device,
			"%s: SHIMARUS DRIVER RUNNING\n", __func__);


	if ( (!sl->family_data) || (!device_family(sl)) ){
		dev_info(device,
			"%s: Device not supported by the driver\n", __func__);
		return 0;  /* No device family */
	}
	
	/* get the correct function depending on the device */
	ret = convert_t(sl, &info);

	if ( ret < 0 ){
		dev_dbg(device,
			"%s: Temperature data may be corrupted. err=%d\n", __func__,
			ret);
		return 0;
	}
	// TODO Arrange temperature conversion per device
	return sprintf(buf, "%d\n", w1_convert_temp(info.rom, sl->family->fid));
}
	
static ssize_t ext_power_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct w1_slave *sl = dev_to_w1_slave(device);

	if (!sl->family_data){
		dev_info(device,
			"%s: Device not supported by the driver\n", __func__);
		return 0;  /* No device family */
	}
		
	/* Getting the power mode of the device {external, parasite}*/
	SLAVE_POWERMODE(sl) = read_powermode(sl);
	
	if (SLAVE_POWERMODE(sl)<0){
		dev_dbg(device,
			"%s: Power_mode may be corrupted. err=%d\n",
			__func__, SLAVE_POWERMODE(sl));
	}
	return sprintf(buf, "%d\n", SLAVE_POWERMODE(sl));
}

static ssize_t resolution_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct w1_slave *sl = dev_to_w1_slave(device);

	if ( (!sl->family_data) || (!device_family(sl)) ){
		dev_info(device,
			"%s: Device not supported by the driver\n", __func__);
		return 0;  /* No device family */
	}
	
	/* get the correct function depending on the device */
	SLAVE_RESOLUTION(sl) = device_family(sl)->get_resolution(sl);
	if (SLAVE_RESOLUTION(sl)<0){
		dev_dbg(device,
			"%s: Resolution may be corrupted. err=%d\n",
			__func__, SLAVE_RESOLUTION(sl));
	}

	return sprintf(buf, "%d\n", SLAVE_RESOLUTION(sl));
}

static ssize_t resolution_store(struct device *device,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct w1_slave *sl = dev_to_w1_slave(device);
	int val;
	int ret = 0;

	ret = kstrtoint(buf, 10, &val); /* converting user entry to int */

	if (ret) {	/* conversion error */
		dev_info(device, 
			"%s: conversion error. err= %d\n", __func__, ret);
		return size;	/* return size to avoid calling back again the callback*/
	}

	if ( (!sl->family_data) || (!device_family(sl)) ){
		dev_info(device,
			"%s: Device not supported by the driver\n", __func__);
		return size;  /* No device family */
	}

	// TODO Move resolution backup in struct in hardware function ?//////////////////////////
	/* Don't deal with the val enterd by user, 
		only device knows what is correct or not */

	/* get the correct function depending on the device */
	ret = device_family(sl)->set_resolution(sl, val);

	if (ret){
		dev_info(device, 
			"%s: writing error %d\n", __func__, ret);
		return size; /* return size to avoid calling back again the callback*/
	}
	else
		SLAVE_RESOLUTION(sl) = val;
	
	return size;
}

static ssize_t eeprom_store(struct device *device,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct w1_slave *sl = dev_to_w1_slave(device);
	int ret = -EINVAL; // Invalid argument

	if (size == sizeof(EEPROM_CMD_WRITE)){
		if (!strncmp( buf , EEPROM_CMD_WRITE, sizeof(EEPROM_CMD_WRITE)-1 ))
			ret = copy_scratchpad(sl);
	} else if (size == sizeof(EEPROM_CMD_READ)){
		if (!strncmp( buf , EEPROM_CMD_READ, sizeof(EEPROM_CMD_READ)-1 ))
			ret = recall_eeprom(sl);
	}

	if(ret)
		dev_info(device, "%s: error in process %d\n", __func__, ret);

	return size;
}
 // ASH //////////////////////////////////////////////////////////////////////////////////

#if IS_REACHABLE(CONFIG_HWMON)
static int w1_read_temp(struct device *device, u32 attr, int channel,
			long *val)
{
	struct w1_slave *sl = dev_get_drvdata(device);
	struct therm_info info;
	u8 fid = sl->family->fid;
	int ret;

	switch (attr) {
	case hwmon_temp_input:
		ret = read_therm(device, sl, &info);
		if (ret)
			return ret;

		if (!info.verdict) {
			ret = -EIO;
			return ret;
		}

		*val = w1_convert_temp(info.rom, fid);
		ret = 0;
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}
#endif

#define W1_42_CHAIN	0x99
#define W1_42_CHAIN_OFF	0x3C
#define W1_42_CHAIN_OFF_INV	0xC3
#define W1_42_CHAIN_ON	0x5A
#define W1_42_CHAIN_ON_INV	0xA5
#define W1_42_CHAIN_DONE 0x96
#define W1_42_CHAIN_DONE_INV 0x69
#define W1_42_COND_READ	0x0F
#define W1_42_SUCCESS_CONFIRM_BYTE 0xAA
#define W1_42_FINISHED_BYTE 0xFF
static ssize_t w1_seq_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct w1_slave *sl = dev_to_w1_slave(device);
	ssize_t c = PAGE_SIZE;
	int rv;
	int i;
	u8 ack;
	u64 rn;
	struct w1_reg_num *reg_num;
	int seq = 0;

	mutex_lock(&sl->master->bus_mutex);
	/* Place all devices in CHAIN state */
	if (w1_reset_bus(sl->master))
		goto error;
	w1_write_8(sl->master, W1_SKIP_ROM);
	w1_write_8(sl->master, W1_42_CHAIN);
	w1_write_8(sl->master, W1_42_CHAIN_ON);
	w1_write_8(sl->master, W1_42_CHAIN_ON_INV);
	msleep(sl->master->pullup_duration);

	/* check for acknowledgment */
	ack = w1_read_8(sl->master);
	if (ack != W1_42_SUCCESS_CONFIRM_BYTE)
		goto error;

	/* In case the bus fails to send 0xFF, limit*/
	for (i = 0; i <= 64; i++) {
		if (w1_reset_bus(sl->master))
			goto error;

		w1_write_8(sl->master, W1_42_COND_READ);
		rv = w1_read_block(sl->master, (u8 *)&rn, 8);
		reg_num = (struct w1_reg_num *) &rn;
		if (reg_num->family == W1_42_FINISHED_BYTE)
			break;
		if (sl->reg_num.id == reg_num->id)
			seq = i;

		w1_write_8(sl->master, W1_42_CHAIN);
		w1_write_8(sl->master, W1_42_CHAIN_DONE);
		w1_write_8(sl->master, W1_42_CHAIN_DONE_INV);
		w1_read_block(sl->master, &ack, sizeof(ack));

		/* check for acknowledgment */
		ack = w1_read_8(sl->master);
		if (ack != W1_42_SUCCESS_CONFIRM_BYTE)
			goto error;

	}

	/* Exit from CHAIN state */
	if (w1_reset_bus(sl->master))
		goto error;
	w1_write_8(sl->master, W1_SKIP_ROM);
	w1_write_8(sl->master, W1_42_CHAIN);
	w1_write_8(sl->master, W1_42_CHAIN_OFF);
	w1_write_8(sl->master, W1_42_CHAIN_OFF_INV);

	/* check for acknowledgment */
	ack = w1_read_8(sl->master);
	if (ack != W1_42_SUCCESS_CONFIRM_BYTE)
		goto error;
	mutex_unlock(&sl->master->bus_mutex);

	c -= snprintf(buf + PAGE_SIZE - c, c, "%d\n", seq);
	return PAGE_SIZE - c;
error:
	mutex_unlock(&sl->master->bus_mutex);
	return -EIO;
}


static int __init w1_ds18b20_init(void)
{
	int err = request_module("w1_therm_base");;
	// TODO : remove kmseg 
	printk(KERN_INFO "w1_ds18b20: Entering Module, "
					" request loading module result=%d\n",err);

	err = w1_register_family(&w1_therm_family_DS18B20);

	// TODO : remove kmseg 
	printk(KERN_INFO "w1_ds18b20: Register family result=%d\n", err);

	return 0;
}

static void __exit w1_ds18b20_exit(void)
{
	// TODO : remove kmseg 
	printk(KERN_INFO "w1_ds18b20: Leaving module, unregistring module\n");

	w1_unregister_family(&w1_therm_family_DS18B20);
			
}

module_init(w1_ds18b20_init);
module_exit(w1_ds18b20_exit);

//module_w1_family(w1_therm_family_DS18B20);


MODULE_AUTHOR("Akira Corp. <akira215corp@gmail.com>");
MODULE_DESCRIPTION("1-wire Driver for Maxim/Dallas DS18b20 digital thermometer ");
MODULE_LICENSE("GPL");
//MODULE_ALIAS("w1-family-" __stringify(W1_THERM_DS18B20));


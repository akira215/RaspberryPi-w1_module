# Enhanced w1_therm Linux Kernel Module
Source files to build enhanced module for One-wire Dallas thermometer devices. It is an enhacement from the existing module from Evgeniy Polyakov <zbr@ioremap.net>, adding some features:
* Bulk read : send one command for all the slaves on the bus to trigger temperature conversion
* Optimized conversion time regarding to resolution
* Dedicated sysfs entry for powering read, resolution set/get, eeprom save/restore
* Alarms settings and reading
* Code optimization to mitigate bus traffic
## Supported devices:
```
DS18S20
DS18B20
DS1822
DS1825
DS28EA00
```
## Dependencies
The module is build against `wire` module, and it is called directly if a compliant device is found on the bus (i.e., if the family ROM code of the device is matching one of the supported device).

## Building module

It should be build within the kernel tree : 
```
drivers/w1/
```
and overwrite existing `w1_therm.c` file in `slave/` subdir. 
The only parameter is the strong pullup which can take the following values:
 * strong_pullup = 0	Disable strong pullup completely
 * strong_pullup = 1	Enable automatic strong pullup detection (default)
 * strong_pullup = 2	Force strong pullup

On automatic detection, strong pullup will be applied on the bus line only if the device required it. If you have 2 devices on the line, one parasite powered and the second externally powered, strong pullup will be applied only when required (conversion operation and eeprom write) during communication with the parasite powered device. It will not be applied during communication with the externally powered device. If you trigger a bulk read on the line, strong pullup will be applied as soon as there is one parasite powered device on the line.

If you want to build only the `w1_therm` module, you first have to put the kernel source files in a directory and build it. Then you can put a Makefile in the `slave/` subdir like this one
```
#Compiler path
PREFIX := /path/to/the/arm/compiler/arm-bcm2708-linux-gnueabi-
#Kernel Source Path
KERNEL_SRC := /path/to/the/kernel/files/linux-X.XX.XX-vXX/

# some warnings about bad code
ccflags-y := -Wall

# Only build the w1_therm.ko module
obj-m += w1_therm.o

all:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(KERNEL_SRC) M=$(shell pwd) modules

clean:
	make ARCH=arm CROSS_COMPILE=$(PREFIX) -C $(KERNEL_SRC) M=$(shell pwd) clean
```
And then run make in the subdir. It will build a w1_therm.ko file. You can add a symlink on the module directory to this file to run it. For example you can do :
```
$ cd /lib/modules/X.XX.XX-vXX/kernel/drivers/w1/slaves
$ sudo mv w1_therm.ko w1_therm.ko.official
$ sudo ln -s /path/to/your/file/w1_therm.ko w1_therm.ko
```
This way you the module will run on the `w1_therm` module you have build. Be aware that you need to build the module on the same kernel that the one you will run on. You also need to rebuild the module each time you will change your kernel.

## Sysfs entry

The sysfs interface of the module is :

* **w1_slave (RW)** : Old driver way, kept for compatibility
  * *read* : return 2 lines with the hexa output of the device return the CRC check return temperature in 1/1000°
  * *write* :
    * `0` : save the 2 or 3 bytes to the device EEPROM (i.e. TH, TL and config register)
  	* `9..12` : set the device resolution in RAM (if supported)
  	* Anything else : do nothing 
  
 * **temperature (RO)** : return the temperature in 1/1000°. 
   * If a bulk read has been triggered, it will directly return the temperature computed when the bulk read occured, if available. If not yet available, nothing is returned (a debug kernel message is sent), you should retry later on.
   * If no bulk read has been triggered, it will trigger a conversion and send the result. Note that the conversion duration depend on the resolution (if device support this feature). It takes 94ms in 9bits resolution, 750ms for 12bits.
 
 * **ext_power (RO)** : return the power status by asking the device
   * `0` : device parasite powered
   * `1` : device externally powered
   * `-xx` : xx is kernel error refer to /usr/include/asm/errno.h
 
 * **resolution (RW)** : get or set the device resolution (on supported devices, if not the entry is not present). Note that the resolution will be changed only in device RAM, so it will be cleared when power is lost. You should trigger a `save` to EEPROM command to keep your values after power-on. Read or write are :
   * `9..12` : resolution set in bit (or resolution to set in bit)
   * `-xx` 	: xx is kernel error refer to /usr/include/asm/errno.h
   * Anything else : do nothing 
 	
 * **eeprom (WO)** : writing that file will either trigger a save of the device data to its embedded EEPROM, either restore data embedded in device EEPROM. Be aware that devices support limited EEPROM writing cycles (typical 50k)
   * `save` :	save device RAM to EEPROM
   * `restore` :	restore EEPROM data in device RAM 
 				(device do that automatically on power-up)

 * **alarms (RW)** : read or write TH and TL (Temperature High an Low) alarms. Values shall be space separated and in the device range (typical -55° to 125°). Values are integer as they are store in a 8bit field in the device. Lowest value is automatically put to TL.
 
 * **therm_bulk_read (RW)** : Attribute at master level.
  * *write* : `trigger`	: trigger a bulk read on all supporting device on the bus
  * *read* :
    * `-1` : conversion is in progress for at least 1 sensor
    * `1` :	conversion complete but at least one sensor value has not been read
    * `0` :	no bulk operation. Reading temperature will trigger a conversion
  * *caveat* : if a bulk read is sent but one sensor is not read immediately,
  			the next access to temperature will return the temperature measured 
  			at the time of issue of the bulk read command (not the current temperature)
			
  * *limitation* : up to now in multi w1_master - systems "therm_bulk_read" is only available for the first w1_master
  

## Example of use
You don't need to directly the module. The `wire` module will load it directly as soon as it will detect compliant devices on the bus. To load the `wire` module, ou may need an overlay:
```
$ sudo dtoverlay w1-gpio
```

and to detect that the  module is correctly loaded
```
$ lsmod
w1_therm               24576  0
w1_gpio                16384  0
wire                   40960  2 w1_gpio,w1_therm
.....
```

Herebelow some CLI usage example of the w1_therm module.
You should either be logged as root either have added a udev rules in `/etc/udev/rules.d` directory to run write commands.

### Checking online devices
```
$ cd /sys/bus/w1/devices
/sys/bus/w1/devices $ ls 
28-000002453b80  28-000002453d4f  28-03119779df08  w1_bus_master1
```
We can see here 3 devices (28 header indicate DS18B20) and the bus master

### Reading informations on a device
```
/sys/bus/w1/devices $ cd 28-03119779df08
/sys/bus/w1/devices/28-03119779df08 $ cat ext_power 
1
/sys/bus/w1/devices/28-03119779df08 $ cat resolution 
10
/sys/bus/w1/devices/28-03119779df08 $ cat alarms 
5 20
/sys/bus/w1/devices/28-03119779df08 $ cat temperature 
21500
/sys/bus/w1/devices/28-03119779df08 $ cat w1_slave 
58 01 14 05 3f a5 a5 66 e7 : crc=e7 YES
58 01 14 05 3f a5 a5 66 e7 t=21500
```
 * Device is externally powered
 * Resolution is set to 10bits
 * Low alarm (Tl) is set to 5°C and High alarm (Th) to 20°C
 * Actual temperature is 21,500°
 * Last command is the old way driver return, still implemented

### Settings a device
```
/sys/bus/w1/devices/28-03119779df08 $ sudo su
[sudo] Password : 
/sys/bus/w1/devices/28-03119779df08# echo 12  > resolution 
/sys/bus/w1/devices/28-03119779df08# cat resolution 
12
/sys/bus/w1/devices/28-03119779df08# echo -5 55 > alarms
/sys/bus/w1/devices/28-03119779df08# cat alarms
-5 55
/sys/bus/w1/devices/28-03119779df08# echo save > eeprom
```
 * Device resolution is set to 12 (in device RAM)
 * We check that resolution has been correctly set
 * Low alarm (Tl) is set to -5°C and High alarm (Th) to 55°C
 * We check that alarms are correctly set
 * Last command trigger a save to device EEPROM. Once that done, the resolution and the alarms settings will be set to our values at power-up

### Bulk read example
 ```
/sys/bus/w1/devices/28-03119779df08# cd ../w1_bus_master1
/sys/bus/w1/devices/w1_bus_master1# echo trigger > therm_bulk_read 
/sys/bus/w1/devices/w1_bus_master1# cd ../28-03119779df08
/sys/bus/w1/devices/28-03119779df08# cat temperature
21625
```
 * We trigger a bulk read on the bus
 * Then, when we read the temperature on each device, it directly return the conversion temperaure initiate by the bulk read command. The caveat is that if you don't read all the devices on the line after that command, next attempt to read the temperature will directy return the value of conversion issued after the bulk read command

## Contributing

Please read post issues on this repository and/or submit pull requests.

## Authors

* **Akira Shimahara** <akira215corp@gmail.com> - from *Initial work* - Evgeniy Polyakov <zbr@ioremap.net>


## License
Copyright (c) 2020 Akira Shimahara <akira215corp@gmail.com>

This program is free software; you can redistribute it and/or modify it under the therms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.
 
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.



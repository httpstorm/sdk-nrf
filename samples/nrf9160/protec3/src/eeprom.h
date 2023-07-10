#ifndef GUARD_EEPROM_H
#define GUARD_EEPROM_H

#include <zephyr/drivers/eeprom.h>

#define OK 0
#define FAIL -1

extern const struct device * eeprom_dev;
extern struct eeprom_apn_config apn_config;

// EEPROM layout
enum EEPROM_ADDRESS
{
	EEPROM_ADDRESS_STARTUP_COUNT = 0,
	EEPROM_ADDRESS_UPDATE_STATUS = 4,
	EEPROM_ADDRESS_APN = 5,
};

#endif /*GUARD_EEPROM_H*/

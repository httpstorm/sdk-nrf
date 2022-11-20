/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <init.h>
//-#include <adp536x.h>
#include <adp536x-mod.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(board_secure, CONFIG_BOARD_LOG_LEVEL);

#define ADP536X_I2C_DEV_NAME	DT_LABEL(DT_NODELABEL(i2c2))
#define LC_MAX_READ_LENGTH	128

uint8_t nrf91_dev_id[9];

static int power_mgmt_init(void)
{
	int err;

	err = adp536x_init(ADP536X_I2C_DEV_NAME);
	if (err) {
		LOG_ERR("ADP536X failed to initialize, error: %d\n", err);
		return err;
	}

	err = adp536x_buck_1v8_set();
	if (err) {
		LOG_ERR("Could not set buck to 1.8 V, error: %d\n", err);
		return err;
	}

	err = adp536x_buckbst_3v3_set();
	if (err) {
		LOG_ERR("Could not set buck/boost to 3.3 V, error: %d\n", err);
		return err;
	}

	err = adp536x_buckbst_enable(true);
	if (err) {
		LOG_ERR("Could not enable buck/boost output, error: %d\n", err);
		return err;
	}

	/* Enables discharge resistor for buck regulator that brings the voltage
	 * on its output faster down when it's inactive. Needed because some
	 * components require to boot up from ~0V.
	 */
	err = adp536x_buck_discharge_set(true);
	if (err) {
		return err;
	}

	/* Sets the VBUS current limit to 500 mA. */
	err = adp536x_vbus_current_set(ADP536X_VBUS_ILIM_500mA);
	if (err) {
		LOG_ERR("Could not set VBUS current limit, error: %d\n", err);
		return err;
	}

	/* Sets the charging current to 320 mA. */
	err = adp536x_charger_current_set(ADP536X_CHG_CURRENT_320mA);
	if (err) {
		LOG_ERR("Could not set charging current, error: %d\n", err);
		return err;
	}

	/* Sets the charge current protection threshold to 400 mA. */
	err = adp536x_oc_chg_current_set(ADP536X_OC_CHG_THRESHOLD_400mA);
	if (err) {
		LOG_ERR("Could not set charge current protection, error: %d\n",
			err);
		return err;
	}

	err = adp536x_charging_enable(true);
	if (err) {
		LOG_ERR("Could not enable charging: %d\n", err);
		return err;
	}

	/* This is custom made */
	(void)adp536x_charger_ldo_enable(true);

	uint8_t tmp;

	err = adp536x_reg_write(0x20, 0xff);
	if (err == 0) {
	  err = adp536x_reg_read(0x27, &tmp);
	  if (err == 0) {
		adp536x_reg_write(0x27, tmp | 1);
	  }
	}

	adp536x_reg_write(0x0a, 0xC0);

	err = adp536x_reg_read(0x34, &tmp);
	if (err) {
	  LOG_ERR("Could not read INT1 flags register: %d", err);
	  tmp = 0xff;
	}
	else {
	  LOG_INF("INT1 flags are set to 0x%02x", tmp);
	}

	adp536x_reg_write(0x34, tmp);

	/* Set the INTERRUPT_ENABLE1 register */
	err = adp536x_reg_write(0x32, 0x01);
	if (err) {
		LOG_ERR("Could not enable INT1: %d\n", err);
		return err;
	}
	/* End of the custom made */

	return 0;
}

static int protec3_board_init(const struct device *dev)
{
	int err;

	err = power_mgmt_init();
	if (err) {
		LOG_ERR("power_mgmt_init failed with error: %d", err);
		return err;
	}


#if defined(CONFIG_HWINFO)
	ssize_t length;

	(void)memset(nrf91_dev_id, 0x0, sizeof(nrf91_dev_id));

	/* Obtain the device id */
	length = hwinfo_get_device_id(nrf91_dev_id, sizeof(nrf91_dev_id) - 1);

	if (length == 8) {
		LOG_INF("unique deviceId is 0x%02x%02x%02x%02x%02x%02x%02x%02x", nrf91_dev_id[0], nrf91_dev_id[1], nrf91_dev_id[2], nrf91_dev_id[3], nrf91_dev_id[4], nrf91_dev_id[5], nrf91_dev_id[6], nrf91_dev_id[7]);
	}
	else {
		LOG_ERR("Could not get device Id");
	}

#endif

	return 0;
}

SYS_INIT(protec3_board_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

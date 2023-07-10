/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdio.h>
#include <device.h>
#include <inttypes.h>
#include <string.h>
#include <bsd.h>
#include <sys/util.h>
#include <sys/printk.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <drivers/i2c.h>
#include <drivers/eeprom.h>
#include <drivers/sensor.h>
#include <modem/bsdlib.h>
#include <modem/lte_lc.h>
#include <modem/modem_info.h>
#include <modem/at_cmd.h>
#include <net/socket.h>
#include <nrf_socket.h>

#include <adp536x-mod.h>
#include <buzzer.h>

LOG_MODULE_REGISTER(protec3, CONFIG_PROTEC3_LOG_LEVEL);

struct gpio_struct {
	const char *gpio_dev_name;
	const char *gpio_pin_name;
	unsigned int gpio_pin;
	unsigned int gpio_flags;
};

#define I2C_BUS_2_DEV_NAME      DT_LABEL(DT_NODELABEL(i2c2))
#define RTC_I2C_DEV_NAME        DT_LABEL(DT_NODELABEL(i2c3))
#define EEPROM0_DEVICE_NAME     DT_LABEL(DT_NODELABEL(eeprom0))
#define UART_DEVICE_NAME        DT_LABEL(DT_NODELABEL(uart1))

#define SHTC3_I2C_ADDR 0x70
#define RTC_I2C_ADDR 0x68
#define EEPROM_I2C_BASE_ADDRESS 0x50

#define FLAGS_OR_ZERO(node)						\
	COND_CODE_1(DT_PHA_HAS_CELL(node, gpios, flags),		\
		    (DT_GPIO_FLAGS(node, gpios)),			\
		    (0))

#define PIN_STRUCT(alias, type) {                                       \
        .gpio_dev_name = DT_GPIO_LABEL(DT_ALIAS(alias), gpios),         \
        .gpio_pin_name = DT_LABEL(DT_ALIAS(alias)),                     \
        .gpio_pin = DT_GPIO_PIN(DT_ALIAS(alias), gpios),                \
        .gpio_flags = type | FLAGS_OR_ZERO(DT_ALIAS(alias)),            \
}

#define PIN_OUTPUT(alias) PIN_STRUCT(alias, GPIO_OUTPUT)
#define PIN_INPUT(alias) PIN_STRUCT(alias, GPIO_INPUT)

const struct gpio_struct button_node = PIN_INPUT(button);
const struct gpio_struct gps_en_node = PIN_OUTPUT(gps_enable);
const struct gpio_struct alarm_loud_node = PIN_OUTPUT(alarm_loud);
const struct gpio_struct rtc_node = PIN_INPUT(rtc_int);
const struct gpio_struct pir_node = PIN_INPUT(pir_int);
const struct gpio_struct pmic_node = PIN_INPUT(pmic_int);

const static struct device *pin_init(const struct gpio_struct *gpio)
{
	const struct device *gpio_dev;
	int ret;

	gpio_dev = device_get_binding(gpio->gpio_dev_name);
	if (gpio_dev == NULL) {
		printk("Error: didn't find %s device\n", gpio->gpio_dev_name);
		return NULL;
	}

	ret = gpio_pin_configure(gpio_dev, gpio->gpio_pin, gpio->gpio_flags);
	if (ret != 0) {
		printk("Error %d: failed to configure pin %d '%s'\n", ret, gpio->gpio_pin, gpio->gpio_pin_name);
		return NULL;
	}

        return gpio_dev;
}

const static struct device *pin_int_init(const struct gpio_struct *gpio, struct gpio_callback *cb_data, void (*cb_func)(const struct device *, struct gpio_callback *,uint32_t)) {
	const struct device *gpio_dev;
	int ret;

        gpio_dev = pin_init(gpio);
        if (gpio_dev == NULL) {
                return NULL;
        }

        ret = gpio_pin_interrupt_configure(gpio_dev, gpio->gpio_pin, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret != 0) {
                printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, gpio->gpio_pin_name, gpio->gpio_pin);
                return NULL;
        }

        gpio_init_callback(cb_data, cb_func, BIT(gpio->gpio_pin));
        gpio_add_callback(gpio_dev, cb_data);

        printk("Set up button at %s pin %d\n", gpio->gpio_pin_name, gpio->gpio_pin);

        return gpio_dev;
}


volatile bool button_int_event = false;
volatile bool pir_int_event = false;

static struct gpio_callback button_cb_data;
static struct gpio_callback rtc_int_cb_data;
static struct gpio_callback pir_int_cb_data;
static struct gpio_callback pmic_int_cb_data;


const struct device *shtc3_dev;
const struct device *alarm_loud;

struct service_info {
    struct k_work work;
	const struct device *device;
    const struct gpio_struct *gpio_node;
    char data[256];
} beep_service, button_debounce_service, pir_debounce_service, pmic_service, gnss_uart_service;


void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
        k_work_submit(&button_debounce_service.work);
}

void button_debounce_worker(struct k_work *item) {
    k_msleep(50);
    struct service_info *the_service = CONTAINER_OF(item, struct service_info, work);
    if (gpio_pin_get(the_service->device, the_service->gpio_node->gpio_pin)) {
        button_int_event = true;
    }
}

void rtc_int_occured(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
      printk("RTC Intterupt\n");
}

void pir_int_occured(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
      k_work_submit(&pir_debounce_service.work);
}

void pir_debounce_worker(struct k_work *item) {
    k_msleep(50);
    struct service_info *the_service = CONTAINER_OF(item, struct service_info, work);
    if (gpio_pin_get(the_service->device, the_service->gpio_node->gpio_pin)) {
        pir_int_event = true;
    }
}


void pmic_int_occured(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
      k_work_submit(&pmic_service.work);
}

void pmic_worker(struct k_work *item)
{
      uint8_t int1_f, int2_f;

      adp536x_reg_read(0x34, &int1_f);
      adp536x_reg_read(0x34, &int2_f);

      printk("PMIC Interrupt: 0x%02x 0x%02x\n", int1_f, int2_f);
}

static uint8_t nmea_message[128];
static uint8_t nmea_message_pos = 0;

void gnss_uart_cb(const struct device *x, void * user_data)
{
        static uint8_t uart_buf[1024];

        static uint8_t counter = 0;

	uart_irq_update(x);
	int data_length = 0;

	if (uart_irq_rx_ready(x)) {
		data_length = uart_fifo_read(x, uart_buf, sizeof(uart_buf));
		uart_buf[data_length] = 0;
	}
        for (int i = 0; i < data_length; i++) {
            if (uart_buf[i] == '$' || uart_buf[i] == '!') {
                nmea_message_pos = 0;
            }

            if (nmea_message_pos < 127  || uart_buf[i] != '\r') {
                nmea_message[nmea_message_pos++] = uart_buf[i];
            }

            if (uart_buf[i] == '\n') {
                nmea_message[nmea_message_pos - 1] = 0;
                if (nmea_message[0] == '$' && strncmp(nmea_message + 3, "GGA", 3) == 0) {
                        if (counter++ >= 10) {
                            sprintf(gnss_uart_service.data, "%s", nmea_message);
                            k_work_submit(&gnss_uart_service.work);
                            //client_send(nmea_message);
                            counter = 0;
                        }
                }
            }
        }
}


void gnss_uart_worker(struct k_work *item) {
    struct service_info *the_service = CONTAINER_OF(item, struct service_info, work);
    printk("%s\n", the_service->data);
}

static uint8_t sht3xd_compute_crc(uint16_t value)
{
	uint8_t buf[2] = { value >> 8, value & 0xFF };
	uint8_t crc = 0xFF;
	uint8_t polynom = 0x31;
	int i, j;

	for (i = 0; i < 2; ++i) {
		crc = crc ^ buf[i];
		for (j = 0; j < 8; ++j) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ polynom;
			} else {
				crc = crc << 1;
			}
		}
	}

	return crc;
}


int sht3xd_write_command(struct device *dev, uint16_t cmd)
{
	uint8_t tx_buf[2] = { cmd >> 8, cmd & 0xFF };

	return i2c_write(shtc3_dev, tx_buf, sizeof(tx_buf),
			 SHTC3_I2C_ADDR);
}

int shtc3_init(const char *dev_name)
{
	int err = 0;

	shtc3_dev = device_get_binding(dev_name);
	if (err) {
		err = -ENODEV;
	}

	return err;
}

void bcd_print(const char *buffer) {
        int seconds = (((buffer[0] & (0x70)) >> 4) * 10) + (buffer[0] & 0x0f);
        int minutes = (((buffer[1] & (0x70)) * 10) >> 4) + (buffer[1] & 0x0f);
        int hours = (((buffer[2] & (0x30)) * 10) >> 4) + (buffer[2] & 0x0f);

        int date = (((buffer[4] & (0x30)) >> 4) * 10) + (buffer[4] & 0x0f);
        int month = (((buffer[5] & (0x10)) >> 4) * 10) + (buffer[5] & 0x0f);
        int year = 2000 + ((buffer[5] >> 6) * 100) + ((buffer[6] >> 4) * 10) + (buffer[6] & 0x0f);

        printk("%02d:%02d:%02d, %d/%d/%d\n", hours, minutes, seconds, date, month, year);

}

K_SEM_DEFINE(thread_sync_sem, 0, 2);

void beep_worker(struct k_work *item)
{
    ui_buzzer_set_frequency(2000, 100);
    k_msleep(100);
    ui_buzzer_set_frequency(0, 0);
}


static bool led_on = false;


static struct modem_param_info modem_param;

#define CGSN_RESP_LEN 19
#define IMEI_LEN 15
#define CLIENT_ID_LEN (IMEI_LEN + sizeof("nrf-"))

static uint8_t client_id_buf[CLIENT_ID_LEN] = {0};

struct rsrp_data {
	uint16_t value;
	uint16_t offset;
};

static struct rsrp_data rsrp = {
	.value = 0,
	.offset = MODEM_INFO_RSRP_OFFSET_VAL,
};


static int client_id_get(char *id_buf, size_t len)
{
	enum at_cmd_state at_state;
	char imei_buf[CGSN_RESP_LEN] = {0};
	int err = at_cmd_write("AT+CGSN", imei_buf, sizeof(imei_buf),
				&at_state);

	if (err) {
		printk("Error when trying to do at_cmd_write: %d, at_state: %d",
			err, at_state);
	}

	snprintf(id_buf, len, "nrf-%.*s", IMEI_LEN, imei_buf);
	return 0;
}

void main(void) {
        const struct device *gnss_uart_dev;
        const struct device *button;
        const struct device *gps_en;
        const struct device *rtc_dev;
        const struct device *rtc_int;
        const struct device *pir_int;
        const struct device *i2c2_dev;
        const struct device *eeprom_dev;
        const struct device *pmic_int;

	int adp_err, err;

        bool loud_on = false;

        k_work_init(&beep_service.work, beep_worker);
        k_work_init(&button_debounce_service.work, button_debounce_worker);
        k_work_init(&pir_debounce_service.work, pir_debounce_worker);
        k_work_init(&pmic_service.work, pmic_worker);
        k_work_init(&gnss_uart_service.work, gnss_uart_worker);

        alarm_loud = pin_init(&alarm_loud_node);
        gps_en = pin_init(&gps_en_node);

        button = pin_int_init(&button_node, &button_cb_data, button_pressed);
        button_debounce_service.device = button;
        button_debounce_service.gpio_node = &button_node;

        rtc_int = pin_int_init(&rtc_node, &rtc_int_cb_data, rtc_int_occured);

        pir_int = pin_int_init(&pir_node, &pir_int_cb_data, pir_int_occured);
        pir_debounce_service.device = pir_int;
        pir_debounce_service.gpio_node = &pir_node;

        pmic_int = pin_int_init(&pmic_node, &pmic_int_cb_data, pmic_int_occured);
        pmic_service.device = pmic_int;
        pmic_service.gpio_node = &pmic_node;


	adp_err = adp536x_init(I2C_BUS_2_DEV_NAME);
	if (adp_err) {
              printk("ADP536X failed to initialize, error: %d\n", adp_err);
	}
        else {
              printk("ADP536X initialized successfully\n");
        };

        gnss_uart_dev = device_get_binding(UART_DEVICE_NAME);
        if (gnss_uart_dev) {
              printk("Uart %s initialized successfully\n", UART_DEVICE_NAME);

              uart_irq_callback_set(gnss_uart_dev, gnss_uart_cb);
              uart_irq_rx_enable(gnss_uart_dev);

              gnss_uart_service.device = gnss_uart_dev;
        }
        else {
              printk("Error initialzing uart %s\n", UART_DEVICE_NAME);
        }

        err = shtc3_init(I2C_BUS_2_DEV_NAME);
        if (err) {
              printk("SHTC3 sensor failed to initialize, error: %d\n", err);
        }
        else {
              printk("SHTC3 sensor initialized successfully\n");

              uint8_t rx_buf[3];
              uint8_t tx_buf[2] = { 0xef, 0xc8 };

              if (i2c_write_read(shtc3_dev, SHTC3_I2C_ADDR, tx_buf, sizeof(tx_buf),
                    rx_buf, sizeof(rx_buf)) < 0) {
                    printk("SHTC3 Failed to read data sample!\n");
              }
              else {
                    printk("SHTC3 device id: 0x%02x 0x%02x 0x%02x -> 0x%02x\n", rx_buf[0], rx_buf[1], rx_buf[2], sht3xd_compute_crc((rx_buf[0] << 8) + rx_buf[1]));
              }
        }

        rtc_dev = device_get_binding(RTC_I2C_DEV_NAME);
        if (rtc_dev) {
              printk("RTC on %s bus initialized successfully\n", RTC_I2C_DEV_NAME);

              uint8_t rx_buf[8];
              uint8_t tx_buf[1] = { 0 };

              if (i2c_write_read(rtc_dev, RTC_I2C_ADDR, tx_buf, sizeof(tx_buf),
                    rx_buf, sizeof(rx_buf)) < 0) {
                    printk("RTC Failed to read data sample!\n");
              }
              else {
                    for (int i = 0; i < 8; i++) {
                          printk("RTC reg: 0x%02x -> 0x%02x\n", i, rx_buf[i]);
                    }
                    bcd_print(rx_buf);
              }


        }
        else {
              printk("Error initializing RTC on %s\n", RTC_I2C_DEV_NAME);

        }

        client_id_get(client_id_buf, sizeof(client_id_buf));

        printk("Client id: %s\n", client_id_buf);

        union startup_counter_union {
            uint32_t value;
            uint8_t buffer[4];
        };

        union startup_counter_union startup_counter;

        i2c2_dev = device_get_binding(I2C_BUS_2_DEV_NAME);
        if (i2c2_dev) {
            uint8_t val = 0;
            i2c_reg_read_byte(i2c2_dev, 0x18, 0x0f, &val);
            printk("Accelerometer WHO_AM_I: 0x%02x\n", val);
        }

        eeprom_dev = device_get_binding(EEPROM0_DEVICE_NAME);
        if (eeprom_dev) {
            printk("EEPROM device %s initialized successfully\n", EEPROM0_DEVICE_NAME);
            err = eeprom_read(eeprom_dev, 0, startup_counter.buffer, sizeof(uint32_t));
            printk("Startup counter is: %d\n", startup_counter.value);
            startup_counter.value++;
            eeprom_write(eeprom_dev, 0, startup_counter.buffer, sizeof(uint32_t));

        }

        if (ui_buzzer_init() == 0) {
              printk("Buzzer initialized in silent mode\n");
        }
        else {
              printk("Buzzer initialization failed\n");
        }

        k_sem_give(&thread_sync_sem);
        k_sem_give(&thread_sync_sem);

	while (1) {
            k_msleep(1);
            if (pir_int_event) {
                  printk("PIR Sensor Intterupt\n");
                  pir_int_event = false;
            }
            if (button_int_event) {
                    if (led_on) {
                        if (loud_on) {
                            loud_on = false;
                            gpio_pin_set(alarm_loud, alarm_loud_node.gpio_pin, 0);

                            led_on = false;

                            i2c_reg_write_byte(rtc_dev, RTC_I2C_ADDR, 0x07, 0x00);
                        }
                        else {
                            loud_on = true;
                            gpio_pin_set(alarm_loud, alarm_loud_node.gpio_pin, 1);

                            gpio_pin_set(gps_en, gps_en_node.gpio_pin, 0);

                        }

                    }
                    else {
                        led_on = true;
                        gpio_pin_set(gps_en, gps_en_node.gpio_pin, 1);
                        i2c_reg_write_byte(rtc_dev, RTC_I2C_ADDR, 0x07, 0x80);
                    }

                    k_work_submit(&beep_service.work);

                    uint8_t rx_buf[8];
                    uint8_t tx_buf[1] = { 0 };

                    if (i2c_write_read(rtc_dev, RTC_I2C_ADDR, tx_buf, sizeof(tx_buf),
                          rx_buf, sizeof(rx_buf)) < 0) {
                          printk("RTC Failed to read data sample!\n");
                    }
                    else {
                          for (int i = 7; i < 8; i++) {
                                printk("RTC reg: 0x%02x -> 0x%02x\n", i, rx_buf[i]);
                          }
                          bcd_print(rx_buf);
                    }

                    if (!adp_err) {
                    err = 0;

                    uint8_t
                      cs1 = 0,
                      cs2 = 0,
                      soc = 0,
                      vbh = 0,
                      vbl = 0,
                      pgs = 0;

                    if (adp536x_reg_read(0x08, &cs1) > 0) { err += 1; }
                    if (adp536x_reg_read(0x09, &cs2) > 0) { err += 2; }
                    if (adp536x_reg_read(0x21, &soc) > 0) { err += 4; }
                    if (adp536x_reg_read(0x25, &vbh) > 0) { err += 8; }
                    if (adp536x_reg_read(0x26, &vbl) > 0) { err += 16; }
                    if (adp536x_reg_read(0x2f, &pgs) > 0) { err += 32; }

                    if (!err) {
                        uint16_t vb = (vbh << 5) | (vbl >> 3);
                        printk("adp536x status: 0x%02x, 0x%02x, 0x%02x, %04d mV, %d%%\n", cs1, cs2, pgs, vb, soc);
                    }
                }
                button_int_event = false;
            }
	}
}

#define CONFIG_COAP_SERVER_PORT 61002
#define CONFIG_COAP_SERVER_HOSTNAME "84.40.94.129"

static struct sockaddr_storage server;

static int server_resolve(void)
{
	int err;
	struct addrinfo *result;
	struct addrinfo hints = {
                .ai_flags = AI_NUMERICHOST,
		.ai_family = AF_INET,
		.ai_socktype = SOCK_DGRAM
	};
	char ipv4_addr[NET_IPV4_ADDR_LEN];

	err = getaddrinfo(CONFIG_COAP_SERVER_HOSTNAME, NULL, &hints, &result);
	if (err != 0) {
		LOG_ERR("ERROR: getaddrinfo failed %d\n", err);
		return -EIO;
	}

	if (result == NULL) {
		LOG_ERR("ERROR: Address not found\n");
		return -ENOENT;
	}

	struct sockaddr_in *server4 = ((struct sockaddr_in *)&server);

	server4->sin_addr.s_addr =
		((struct sockaddr_in *)result->ai_addr)->sin_addr.s_addr;
	server4->sin_family = AF_INET;
	server4->sin_port = htons(CONFIG_COAP_SERVER_PORT);

	inet_ntop(AF_INET, &server4->sin_addr.s_addr, ipv4_addr,
		  sizeof(ipv4_addr));
        printk("IPv4 Address found %s\n", ipv4_addr);

	freeaddrinfo(result);

	return 0;
}


static void gray_thread() {
    const struct gpio_struct leds[3] = {
          PIN_OUTPUT(led0),
          PIN_OUTPUT(led1),
          PIN_OUTPUT(led2),
    };

	const struct device *led_dev[3];

    k_sem_take(&thread_sync_sem, K_FOREVER);

    for (int i = 0; i < 3; i++) {
        led_dev[i] = pin_init(&leds[i]);
    }

    static uint8_t gray_code[8] = { 0, 1, 3, 2, 6, 7, 5, 4 };
    int position = 0;
    printk("LED drivers initialized successfully\n");

    while (1) {
        if (led_on) {
              for (int i = 0; i < 3; i++) {
                  gpio_pin_set(led_dev[i], leds[i].gpio_pin, ((gray_code[position] & (1 << i)) ? 1 : 0));

              }

              position++;
              if (position > 7) {
                  position = 0;
              }
              k_msleep(1000);
        }
        else {
            if (position != 0) {
                for (int i = 0; i < 3; i++) {
                      gpio_pin_set(led_dev[i], leds[i].gpio_pin, 0);
                }
                position = 0;
            }
            k_msleep(100);
        }
    };

}

static void modem_rsrp_handler(char rsrp_value)
{
	if (rsrp_value > 97) {
		return;
	}

	rsrp.value = rsrp_value;

	char buffer[CONFIG_MODEM_INFO_BUFFER_SIZE] = {0};
	int32_t rsrp_current;
	size_t len;
	rsrp_current = rsrp.value - rsrp.offset;

	len = snprintf(buffer, CONFIG_MODEM_INFO_BUFFER_SIZE,
		       "signal strength: %d dBm", rsrp_current);
        printk("%s\n", buffer);

}

static void modem_data_init(void)
{
	int err;
	err = modem_info_init();
	if (err) {
		printk("Modem info could not be established: %d\n", err);
		return;
	}

	modem_info_params_init(&modem_param);
	modem_info_rsrp_register(modem_rsrp_handler);
}


void modem_thread(void)
{
	struct modem_param_info *modem_ptr = NULL;
	int ret;

        char buffer[CONFIG_MESSAGE_BUFFER_SIZE] = {0};
        k_sem_take(&thread_sync_sem, K_FOREVER);

        printk("Starting LTE link initialization\n");

        lte_lc_init_and_connect();
        server_resolve();
        printk("LTE link initialization done\n");

        modem_data_init();

	while (1) {
                    ret = modem_info_params_get(&modem_param);
                    if (ret < 0) {
                          printk("Unable to obtain modem parameters: %d\n", ret);
                    } else {

                          modem_ptr = &modem_param;

                          if (IS_ENABLED(CONFIG_MODEM_INFO_ADD_NETWORK)) {
                                snprintf(buffer, CONFIG_MESSAGE_BUFFER_SIZE, "operator id: %s, area code: %s (%d), cell id: %s (%d), band: %d, ip: %s, apn: %s, mode: %s",
                                  modem_ptr->network.current_operator.value_string,
                                  modem_ptr->network.area_code.value_string,
                                  modem_ptr->network.area_code.value,
                                  modem_ptr->network.cellid_hex.value_string,
                                  (int)modem_ptr->network.cellid_dec,
                                  modem_ptr->network.current_band.value,
                                  modem_ptr->network.ip_address.value_string,
                                  modem_ptr->network.apn.value_string,
                                    (modem_ptr->network.nbiot_mode.value == 1 && modem_ptr->network.gps_mode.value == 1 ? "NB-IoT, GPS" :
                                      (modem_ptr->network.lte_mode.value == 1 && modem_ptr->network.gps_mode.value == 1 ? "LTE-M, GPS" :
                                        (modem_ptr->network.nbiot_mode.value == 0 && modem_ptr->network.lte_mode.value == 0 && modem_ptr->network.gps_mode.value == 1 ? "GPS" :
                                          "unknown"
                                        )
                                      )
                                    )
                                );

                                printk("%s\n", buffer);

                                if (IS_ENABLED(CONFIG_MODEM_INFO_ADD_DATE_TIME)) {
                                    snprintf(buffer, CONFIG_MESSAGE_BUFFER_SIZE, "network time: %s", modem_ptr->network.date_time.value_string);
                                    printk("%s\n", buffer);
                                }
                          }
                    }
		k_sleep(K_SECONDS(30));
	}
}


K_THREAD_DEFINE(modem_thread_id, 4096, modem_thread, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(gray_thread_id, 1024, gray_thread, NULL, NULL, NULL, 7, 0, 0);

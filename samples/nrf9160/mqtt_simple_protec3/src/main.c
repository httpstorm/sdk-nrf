/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <stdio.h>
#include <device.h>
#include <drivers/gpio.h>

#include <string.h>

#include <net/mqtt.h>
#include <net/socket.h>
#include <nrf_socket.h>
#include <modem/lte_lc.h>

//#include <modem/nrf_modem_lib.h>
#include <net/tls_credentials.h>
#include <modem/at_cmd.h>
#include <modem/at_notif.h>
#include <modem/modem_key_mgmt.h>

#if defined(CONFIG_LWM2M_CARRIER)
#include <lwm2m_carrier.h>
#endif

/* Buffers for MQTT client. */
static uint8_t rx_buffer[CONFIG_MQTT_MESSAGE_BUFFER_SIZE];
static uint8_t tx_buffer[CONFIG_MQTT_MESSAGE_BUFFER_SIZE];
static uint8_t payload_buf[CONFIG_MQTT_PAYLOAD_BUFFER_SIZE + 1];

/* The mqtt client struct */
static struct mqtt_client client;

/* MQTT Broker details. */
static struct sockaddr_storage broker;

/* Connected flag */
static bool connected;

/* File descriptor */
static struct pollfd fds;

#if defined(CONFIG_MQTT_LIB_TLS)
static sec_tag_t sec_tag_list[] = { CONFIG_SEC_TAG };
#endif /* defined(CONFIG_MQTT_LIB_TLS) */

struct gpio_struct
{
	const char * gpio_dev_name;
	const char * gpio_pin_name;
	unsigned int gpio_pin;
	unsigned int gpio_flags;
};

#define FLAGS_OR_ZERO(node)                     \
	COND_CODE_1(                                \
		DT_PHA_HAS_CELL(node, gpios, flags),    \
		(DT_GPIO_FLAGS(node, gpios)),           \
		(0)                                     \
	)

#define PIN_STRUCT(alias, type)                             \
{                                                           \
	.gpio_dev_name = DT_GPIO_LABEL(DT_ALIAS(alias), gpios), \
	.gpio_pin_name = DT_LABEL(DT_ALIAS(alias)),             \
	.gpio_pin = DT_GPIO_PIN(DT_ALIAS(alias), gpios),        \
	.gpio_flags = type | FLAGS_OR_ZERO(DT_ALIAS(alias)),    \
}

#define PIN_OUTPUT(alias) PIN_STRUCT(alias, GPIO_OUTPUT)
#define PIN_INPUT(alias) PIN_STRUCT(alias, GPIO_INPUT)


static const char ca_cert[] =
{
	#include "../certs/ca.crt"
};

static const char private_cert[] =
{
	#include "../certs/client.crt"
};

static const char private_key[] =
{
	#include "../certs/client.key"
};


//const struct gpio_struct bt_int_node = PIN_INPUT(bt_module_int);
//const struct device *bt_int;

const struct gpio_struct button0_node = PIN_INPUT(button);
const struct device * button0;

const struct gpio_struct led_nodes[3] =
{
	PIN_OUTPUT(led0),
	PIN_OUTPUT(led1),
	PIN_OUTPUT(led2),
};

const struct device * leds[3];

#define IMEI_LEN 15

char imei[IMEI_LEN + 1] = { 0 };


volatile bool button_int_event = false;

static struct gpio_callback button_cb_data;

struct service_info
{
	struct k_work work;
	const  struct device * device;
	const struct gpio_struct * gpio_node;
	char data[256];
} button_debounce_service;


void button_pressed(const struct device * dev, struct gpio_callback * cb, uint32_t pins)
{
	k_work_submit(&button_debounce_service.work);
}

static const struct device * pin_init(const struct gpio_struct * gpio)
{
	const struct device * gpio_dev;
	int ret;

	gpio_dev = device_get_binding(gpio->gpio_dev_name);

	if (gpio_dev == NULL)
	{
		printk("Error: didn't find %s device\n", gpio->gpio_dev_name);
		return NULL;
	}

	ret = gpio_pin_configure(gpio_dev, gpio->gpio_pin, gpio->gpio_flags);

	if (ret != 0)
	{
		printk(
			"Error %d: failed to configure pin %d '%s'\n",
			ret,
			gpio->gpio_pin,
			gpio->gpio_pin_name
		);

		return NULL;
	}

	return gpio_dev;
}

static const struct device * pin_int_init(
	const struct gpio_struct * gpio,
	struct gpio_callback * cb_data,
	void (*cb_func)(const struct device *, struct gpio_callback *, uint32_t)
)
{
	const struct device * gpio_dev;
	int ret;

	gpio_dev = pin_init(gpio);

	if (gpio_dev == NULL)
	{
		return NULL;
	}

	ret = gpio_pin_interrupt_configure(gpio_dev, gpio->gpio_pin, GPIO_INT_EDGE_TO_ACTIVE);

	if (ret != 0)
	{
		printk(
			"Error %d: failed to configure interrupt on %s pin %d\n",
			ret,
			gpio->gpio_pin_name,
			gpio->gpio_pin
		);

		return NULL;
	}

	gpio_init_callback(cb_data, cb_func, BIT(gpio->gpio_pin));
	gpio_add_callback(gpio_dev, cb_data);

	printk("Set up button at %s pin %d\n", gpio->gpio_pin_name, gpio->gpio_pin);

	return gpio_dev;
}

static int remove_whitespace(char * buf)
{
	size_t i, j = 0, len;

	len = strlen(buf);

	for (i = 0; i < len; i++)
	{
		if (buf[i] >= 32 && buf[i] <= 126)
		{
			if (j != i)
			{
				buf[j] = buf[i];
			}

			j++;
		}
	}

	if (j < len)
	{
		buf[j] = '\0';
	}

	return 0;
}

static int query_modem(const char * cmd, char * buf, size_t buf_len)
{
	int ret;
	enum at_cmd_state at_state;

	ret = at_cmd_write(cmd, buf, buf_len, &at_state);

	if (ret)
	{
		snprintf(buf, buf_len, "at_state: %d", at_state);
		//strncpy(buf, "error", buf_len);

		return ret;
	}

	remove_whitespace(buf);

	return 0;
}

static int client_id_get()
{
	int ret;
	//enum at_cmd_state at_state;
	char buf[32] = { 0 };


	ret = query_modem("AT+CGSN", buf, sizeof(buf));

	if (ret)
	{
		//printk("Error when trying to do at_cmd_write: %d, at_state: %d", err, at_state);
		//printk("ERROR: Failed to read IMEI.\n");
		//goto finish;
	}
	else
	{
		//printk("Modem IMEI read.\n");
		strncpy(imei, buf, IMEI_LEN);
	}

	return 0;
}

/*
static int read_ficr_word(uint32_t *result, const volatile uint32_t *addr)
{
	printk("Read FICR (address 0x%08x):\n", (uint32_t)addr);

	int ret = spm_request_read(result, (uint32_t)addr, sizeof(uint32_t));

	if (ret != 0)
	{
		printk("Could not read FICR (err: %d)\n", ret);
	}

	return ret;
}
*/

#if defined(CONFIG_BSD_LIBRARY)

/**@brief Recoverable BSD library error. */
void bsd_recoverable_error_handler(uint32_t err)
{
	printk("bsdlib recoverable error: %u\n", (unsigned int)err);
}

#endif /* defined(CONFIG_BSD_LIBRARY) */

#if defined(CONFIG_LWM2M_CARRIER)
K_SEM_DEFINE(carrier_registered, 0, 1);

void lwm2m_carrier_event_handler(const lwm2m_carrier_event_t * event)
{
	switch (event->type)
	{
	case LWM2M_CARRIER_EVENT_BSDLIB_INIT:
		printk("LWM2M_CARRIER_EVENT_BSDLIB_INIT\n");
		break;

	case LWM2M_CARRIER_EVENT_CONNECT:
		printk("LWM2M_CARRIER_EVENT_CONNECT\n");
		break;

	case LWM2M_CARRIER_EVENT_DISCONNECT:
		printk("LWM2M_CARRIER_EVENT_DISCONNECT\n");
		break;

	case LWM2M_CARRIER_EVENT_READY:
		printk("LWM2M_CARRIER_EVENT_READY\n");
		k_sem_give(&carrier_registered);
		break;

	case LWM2M_CARRIER_EVENT_FOTA_START:
		printk("LWM2M_CARRIER_EVENT_FOTA_START\n");
		break;

	case LWM2M_CARRIER_EVENT_REBOOT:
		printk("LWM2M_CARRIER_EVENT_REBOOT\n");
		break;
	}
}
#endif /* defined(CONFIG_LWM2M_CARRIER) */

/**@brief Function to print strings without null-termination
 */
static void data_print(uint8_t * prefix, uint8_t * data, size_t len)
{
	char buf[len + 1];

	memcpy(buf, data, len);
	buf[len] = 0;
	printk("%s%s\n", prefix, buf);
}

/**@brief Function to publish data on the configured topic
 */
 /*
 static int data_publish(
	struct mqtt_client * c,
	enum mqtt_qos qos,
	uint8_t * data,
	size_t len
)
{
	struct mqtt_publish_param param;

	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = CONFIG_MQTT_PUB_TOPIC;
	param.message.topic.topic.size = strlen(CONFIG_MQTT_PUB_TOPIC);
	param.message.payload.data = data;
	param.message.payload.len = len;
	param.message_id = sys_rand32_get();
	param.dup_flag = 0;
	param.retain_flag = 0;

	data_print("Publishing: " topic, data, len);

	printk(
		"to topic: %s len: %u\n",
		CONFIG_MQTT_PUB_TOPIC,
		(unsigned int)strlen(CONFIG_MQTT_PUB_TOPIC)
	);

	return mqtt_publish(c, &param);
 }
 */

 /* TODO should be a libary function, but I couldn't get the library included
	cf. https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/reference/random/index.html?highlight=sys_rand#c.sys_rand32_get
  */
uint32_t sys_rand32_get(void)
{
	/* algorithm from EUROS soruces */
	static uint32_t seed = 0;

	if (!seed)
	{
		seed = k_uptime_get_32();
	}

	seed = seed * 1103515245L + 12345L;

	return seed >> 16;
}

static int data_publish1(
	struct mqtt_client * c,
	enum mqtt_qos qos,
	uint8_t * subtopic,
	uint8_t * data
)
{
	uint8_t topic[64];
	struct mqtt_publish_param param;

	snprintf(topic, sizeof(topic), "/%s%s", imei, subtopic);

	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = topic;
	param.message.topic.topic.size = strlen(topic);
	param.message.payload.data = data;
	param.message.payload.len = strlen(data);
	param.message_id = sys_rand32_get();
	param.dup_flag = 0;
	param.retain_flag = 0;

	data_print("Publishing: ", data, strlen(data));

	printk(
		"to topic: %s len: %u\n",
		topic,
		(unsigned int)strlen(data)
	);

	return mqtt_publish(c, &param);
}


#define MQTT_TEST_TOPICS 1

/**@brief Function to subscribe to the configured topic
 */
static int subscribe(void)
{
	uint8_t buf[64];

	snprintf(buf, sizeof(buf), "/%s/led", imei);

#if MQTT_TEST_TOPICS
	uint8_t alarm[64];
	uint8_t beep[64];
	uint8_t led_[64];
	uint8_t user[64];
	uint8_t uptime[64];

	snprintf(alarm, sizeof(alarm), "/nrf-%s/alarm/+", imei);
	snprintf(beep, sizeof(beep), "/nrf-%s/beep/+", imei);
	snprintf(led_, sizeof(led_), "/nrf-%s/led/+", imei);
	snprintf(user, sizeof(user), "/nrf-%s/user/+", imei);
	snprintf(uptime, sizeof(uptime), "/nrf-%s/uptime", imei);
#endif

	struct mqtt_topic subscribe_topics[] =
	{
		{
			.topic =
			{
				.utf8 = CONFIG_MQTT_SUB_TOPIC,
				.size = strlen(CONFIG_MQTT_SUB_TOPIC)
			},
			.qos = MQTT_QOS_1_AT_LEAST_ONCE
		},
		{
			.topic =
			{
				.utf8 = buf,
				.size = strlen(buf)
			},
			.qos = MQTT_QOS_1_AT_LEAST_ONCE
		},
#if MQTT_TEST_TOPICS
		{
			.topic =
			{
				.utf8 = alarm,
				.size = strlen(alarm)
			},
			.qos = MQTT_QOS_1_AT_LEAST_ONCE
		},
		{
			.topic =
			{
				.utf8 = beep,
				.size = strlen(beep)
			},
			.qos = MQTT_QOS_1_AT_LEAST_ONCE
		},
		{
			.topic =
			{
				.utf8 = led_,
				.size = strlen(led_)
			},
			.qos = MQTT_QOS_1_AT_LEAST_ONCE
		},
		{
			.topic =
			{
				.utf8 = user,
				.size = strlen(user)
			},
			.qos = MQTT_QOS_1_AT_LEAST_ONCE
		},
		{
			.topic =
			{
				.utf8 = uptime,
				.size = strlen(uptime)
			},
			.qos = MQTT_QOS_1_AT_LEAST_ONCE
		},
#endif
	};

	size_t count = sizeof(subscribe_topics) / sizeof(subscribe_topics[0]);

	const struct mqtt_subscription_list subscription_list =
	{
		.list = subscribe_topics,
		.list_count = count,
		.message_id = 1234
	};

	printk("\nMQTT subscribing to %u topics:\n", count);

	size_t i = 0;

	for (i = 0; i < count; i++)
	{
		printk("  %s\n", subscribe_topics[i].topic.utf8);
	}

	printk("\n");

	return mqtt_subscribe(&client, &subscription_list);


	// snptinf(buf, sizeof(buf), "/%s/led", imei);
	//
	// subscribe_topic.topic.utf8 = buf;
	// subscribe_topic.topic.size = strlen(buf);
	// subscribe_topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
}

/**@brief Function to read the published payload.
 */
static int publish_get_payload(struct mqtt_client * c, size_t length)
{
	uint8_t * buf = payload_buf;
	uint8_t * end = buf + length;

	if (length > sizeof(payload_buf))
	{
		return -EMSGSIZE;
	}

	while (buf < end)
	{
		int ret = mqtt_read_publish_payload(c, buf, end - buf);

		if (ret < 0)
		{
			int err;

			if (ret != -EAGAIN)
			{
				return ret;
			}

			printk("mqtt_read_publish_payload: EAGAIN\n");

			err = poll(&fds, 1, CONFIG_MQTT_KEEPALIVE * MSEC_PER_SEC);

			if (err > 0 && (fds.revents & POLLIN) == POLLIN)
			{
				continue;
			}
			else
			{
				return -EIO;
			}
		}

		if (ret == 0)
		{
			return -EIO;
		}

		buf += ret;
	}

	return 0;
}

/**@brief MQTT client event handler
 */
void mqtt_evt_handler(struct mqtt_client * const c, const struct mqtt_evt * evt)
{
	int err;

	switch (evt->type)
	{
	case MQTT_EVT_CONNACK:
		if (evt->result != 0)
		{
			printk("MQTT connect failed %d\n", evt->result);
			break;
		}

		connected = true;
		printk("[%s:%d] MQTT client connected!\n", __func__, __LINE__);
		int b = subscribe();

		printk(
			"MQTT mqtt_subscribe %i  %s\n",
			b,
			(b == -12) ? "FAIL: check MQTT_MESSAGE_BUFFER_SIZE" : b ? "FAIL" : "OK"
		);

		data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/status", "ONLINE");

		break;

	case MQTT_EVT_DISCONNECT:
		printk(
			"[%s:%d] MQTT client disconnected %d\n",
			__func__,
			__LINE__,
			evt->result
		);

		connected = false;
		break;

	case MQTT_EVT_PUBLISH:
	{
		const struct mqtt_publish_param * p = &evt->param.publish;

		uint8_t topic[p->message.topic.topic.size + 1];
		snprintf(topic, sizeof(topic), "%s", p->message.topic.topic.utf8);

		printk(
			"[%s:%d] MQTT PUBLISH result=%d len=%d\n",
			__func__,
			__LINE__,
			evt->result,
			p->message.payload.len
		);

		err = publish_get_payload(c, p->message.payload.len);

		if (err >= 0)
		{
			printk("+++ SUB  %s = %s\n", topic, payload_buf);

			char buf[64];
			snprintf(buf, sizeof(buf), "/%s/led", imei);

			if (strncmp(topic, buf, strlen(topic)) == 0)
			{
				if (strncmp(payload_buf, "ON", p->message.payload.len) == 0)
				{
					gpio_pin_set(leds[0], led_nodes[0].gpio_pin, 1);
				}
				else if (strncmp(payload_buf, "OFF", p->message.payload.len) == 0)
				{
					gpio_pin_set(leds[0], led_nodes[0].gpio_pin, 0);
				}
			}

			data_print("Received: ", payload_buf, p->message.payload.len);
			printk("to topic: %s len: %u\n", topic, p->message.payload.len);

			/* Echo back received data */
			payload_buf[p->message.payload.len] = '\0';

			data_publish1(
				&client,
				MQTT_QOS_1_AT_LEAST_ONCE,
				"/echo",
				payload_buf
				/*, p->message.payload.len*/
			);
		}
		else
		{
			printk("mqtt_read_publish_payload: Failed! %d\n", err);
			printk("Disconnecting MQTT client...\n");

			err = mqtt_disconnect(c);

			if (err)
			{
				printk("Could not disconnect: %d\n", err);
			}
		}
	}

	break;

	case MQTT_EVT_PUBACK:
		if (evt->result != 0)
		{
			printk("MQTT PUBACK error %d\n", evt->result);
			break;
		}

		printk(
			"[%s:%d] PUBACK packet id: %u\n",
			__func__,
			__LINE__,
			evt->param.puback.message_id
		);

		break;

	case MQTT_EVT_SUBACK:
		if (evt->result != 0)
		{
			printk("MQTT SUBACK error %d\n", evt->result);
			break;
		}

		printk(
			"[%s:%d] SUBACK packet id: %u\n",
			__func__,
			__LINE__,
			evt->param.suback.message_id
		);

		break;

	default:
		printk(
			"[%s:%d] default: %d\n",
			__func__,
			__LINE__,
			evt->type
		);

		break;
	}
}

/**@brief Resolves the configured hostname and
 * initializes the MQTT broker structure
 */
static void broker_init(void)
{
	int err;
	struct addrinfo * result;
	struct addrinfo * addr;
	struct addrinfo hints =
	{
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM
	};


	struct nrf_in_addr dns;
	dns.s_addr = 0x08080808; // Google DNS, 8.8.8.8
	err = nrf_setdnsaddr(NRF_AF_INET, &dns);
	printk("LTE| set DNS: %d\n", err);

	err = getaddrinfo(CONFIG_MQTT_BROKER_HOSTNAME, NULL, &hints, &result);

	if (err)
	{
		printk("ERROR: getaddrinfo failed %d\n", err);

		return;
	}

	addr = result;
	err = -ENOENT;

	/* Look for address of the broker. */
	while (addr != NULL)
	{
		/* IPv4 Address. */
		if (addr->ai_addrlen == sizeof(struct sockaddr_in))
		{
			struct sockaddr_in * broker4 = ((struct sockaddr_in *)&broker);
			char ipv4_addr[NET_IPV4_ADDR_LEN];

			broker4->sin_addr.s_addr = ((struct sockaddr_in *)addr->ai_addr)->sin_addr.s_addr;
			broker4->sin_family = AF_INET;
			broker4->sin_port = htons(CONFIG_MQTT_BROKER_PORT);

			inet_ntop(AF_INET, &broker4->sin_addr.s_addr, ipv4_addr, sizeof(ipv4_addr));
			printk("IPv4 Address found %s\n", ipv4_addr);

			break;
		}
		else
		{
			printk(
				"ai_addrlen = %u should be %u or %u\n",
				(unsigned int)addr->ai_addrlen,
				(unsigned int)sizeof(struct sockaddr_in),
				(unsigned int)sizeof(struct sockaddr_in6)
			);
		}

		addr = addr->ai_next;

		break;
	}

	/* Free the address. */
	freeaddrinfo(result);
}

/**@brief Initialize the MQTT client structure
 */
static void client_init(struct mqtt_client * client)
{
	static struct mqtt_utf8 password;
	static struct mqtt_utf8 user_name;
	static struct mqtt_utf8 will_message;
	static struct mqtt_topic will_topic;
	static struct mqtt_utf8 will_topic_topic;

	static uint8_t client_id[64];
	static uint8_t topic[64];

	mqtt_client_init(client);

	broker_init();

	/* MQTT client configuration */
	client->broker = &broker;
	client->evt_cb = mqtt_evt_handler;

	snprintf(client_id, sizeof(client_id), "protec3-%s", imei);
	client->client_id.utf8 = (uint8_t *)client_id;
	client->client_id.size = strlen(client_id);

	password.utf8 = CONFIG_MQTT_CLIENT_PASSWORD;
	password.size = strlen(password.utf8);
	client->password = &password;

	user_name.utf8 = (uint8_t *)CONFIG_MQTT_CLIENT_USERNAME;
	user_name.size = strlen(user_name.utf8);
	client->user_name = &user_name;

	will_message.utf8 = (uint8_t *)"OFFLINE";
	will_message.size = strlen(will_message.utf8);
	client->will_message = &will_message;

	snprintf(topic, sizeof(topic), "/%s/status", imei);
	will_topic_topic.utf8 = topic;
	will_topic_topic.size = strlen(topic);

	will_topic.qos = MQTT_QOS_2_EXACTLY_ONCE;
	will_topic.topic = will_topic_topic;
	client->will_topic = &will_topic;

	client->protocol_version = MQTT_VERSION_3_1_1;

	//printk("username: %s, password: %s\n", client->user_name->utf8, client->password->utf8);

	/* MQTT buffers configuration */
	client->rx_buf = rx_buffer;
	client->rx_buf_size = sizeof(rx_buffer);
	client->tx_buf = tx_buffer;
	client->tx_buf_size = sizeof(tx_buffer);

	/* MQTT transport configuration */
#if defined(CONFIG_MQTT_LIB_TLS)
	struct mqtt_sec_config * tls_config = &client->transport.tls.config;

	client->transport.type = MQTT_TRANSPORT_SECURE;

	tls_config->peer_verify = CONFIG_PEER_VERIFY;
	tls_config->cipher_count = 0;
	tls_config->cipher_list = NULL;
	tls_config->sec_tag_count = ARRAY_SIZE(sec_tag_list);
	tls_config->sec_tag_list = sec_tag_list;
	tls_config->hostname = CONFIG_MQTT_BROKER_HOSTNAME;
#else /* MQTT transport configuration */
	client->transport.type = MQTT_TRANSPORT_NON_SECURE;
#endif /* defined(CONFIG_MQTT_LIB_TLS) */

}

/**@brief Initialize the file descriptor structure used by poll.
 */
static int fds_init(struct mqtt_client * c)
{
	if (c->transport.type == MQTT_TRANSPORT_NON_SECURE)
	{
		fds.fd = c->transport.tcp.sock;
	}
	else
	{
#if defined(CONFIG_MQTT_LIB_TLS)
		fds.fd = c->transport.tls.sock;
#else
		return -ENOTSUP;
#endif
	}

	fds.events = POLLIN;

	return 0;
}

/**@brief Configures modem to provide LTE link. Blocks until link is
 * successfully established.
 */
static void modem_configure(void)
{
#if defined(CONFIG_LTE_LINK_CONTROL)
	if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT))
	{
		/* Do nothing, modem is already turned on
		 * and connected.
		 */
	}
	else
	{
#if defined(CONFIG_LWM2M_CARRIER)
		/* Wait for the LWM2M_CARRIER to configure the modem and
		 * start the connection.
		 */
		printk("Waitng for carrier registration...\n");
		k_sem_take(&carrier_registered, K_FOREVER);
		printk("Registered!\n");
#else /* defined(CONFIG_LWM2M_CARRIER) */
		int err;

		printk("LTE Link Connecting ...\n");
		err = lte_lc_init_and_connect();
		__ASSERT(err == 0, "LTE link could not be established.");
		printk("LTE Link Connected!\n");
#endif /* defined(CONFIG_LWM2M_CARRIER) */
	}
#endif /* defined(CONFIG_LTE_LINK_CONTROL) */
}

/* Initialize AT communications */
int at_comms_init(void)
{
	int err;

	err = at_cmd_init();

	if (err)
	{
		printk("Failed to initialize AT commands, err %d\n", err);
		return err;
	}

	err = at_notif_init();

	if (err)
	{
		printk("Failed to initialize AT notifications, err %d\n", err);
		return err;
	}

	return 0;
}

/* Provision certificate to modem */
int cert_provision(void)
{
	int err;
	bool exists;
	uint8_t unused;

	err = modem_key_mgmt_exists(
		sec_tag_list[0],
		MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN,
		&exists,
		&unused
	);

	if (err)
	{
		printk("Failed to check for server certificates err %d\n", err);
		return err;
	}

	if (exists)
	{

		/* For the sake of simplicity we delete what is provisioned
		 * with our security tag and reprovision our certificate.
		 */
		err = modem_key_mgmt_delete(
			sec_tag_list[0],
			MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN
		);

		if (err)
		{
			printk("Failed to delete existing certificate, err %d\n", err);
		}
	}

	err = modem_key_mgmt_exists(
		sec_tag_list[0],
		MODEM_KEY_MGMT_CRED_TYPE_PRIVATE_CERT,
		&exists,
		&unused
	);

	if (err)
	{
		printk("Failed to check for server certificates err %d\n", err);
		return err;
	}

	if (exists)
	{

		/* For the sake of simplicity we delete what is provisioned
		 * with our security tag and reprovision our certificate.
		 */
		err = modem_key_mgmt_delete(
			sec_tag_list[0],
			MODEM_KEY_MGMT_CRED_TYPE_PRIVATE_CERT
		);

		if (err)
		{
			printk("Failed to delete existing certificate, err %d\n", err);
		}
	}


	//	if (!exists)
	//	{
	//		printk("Provisioning server certificate\n");
	//
	//		/*  Provision certificate to the modem */
	//		err = modem_key_mgmt_write(
	//			sec_tag_list[0],
	//			MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN,
	//			server_cert, sizeof(server_cert) - 1
	//		);
	//
	//		if (err)
	//		{
	//			printk("Failed to provision server certificate, err %d\n", err);
	//			return err;
	//		}
	//	}
	//	else
	//	{
	//			printk("Server certificate already provisioned\n");
	//	}



	err = modem_key_mgmt_exists(
		sec_tag_list[0],
		MODEM_KEY_MGMT_CRED_TYPE_PUBLIC_CERT,
		&exists,
		&unused
	);

	if (err)
	{
		printk("Failed to check for private certificates err %d\n", err);
		return err;
	}


	//	if (exists)
	//	{
	//
	//		/* For the sake of simplicity we delete what is provisioned
	//		 * with our security tag and reprovision our certificate.
	//		 */
	//		err = modem_key_mgmt_delete(
	//			sec_tag_list[0],
	//			MODEM_KEY_MGMT_CRED_TYPE_PUBLIC_CERT
	//		);
	//
	//		if (err)
	//		{
	//			printk("Failed to delete existing certificate, err %d\n", err);
	//		}
	//	}

	if (!exists)
	{
		printk("Provisioning private certificate\n");

		/*  Provision certificate to the modem */
		err = modem_key_mgmt_write(
			sec_tag_list[0],
			MODEM_KEY_MGMT_CRED_TYPE_PUBLIC_CERT,
			ca_cert,
			sizeof(ca_cert) - 1
		);

		if (err)
		{
			printk("Failed to provision private certificate, err %d\n", err);
			return err;
		}
	}
	else
	{
		printk("Private certificate already provisioned\n");
	}


	return 0;
}


int cert_provision1(
	nrf_sec_tag_t sec_tag,
	enum modem_key_mgmt_cred_type cred_type,
	const void * buf,
	size_t len
)
{
	int err;
	bool exists;
	uint8_t unused;

	err = modem_key_mgmt_exists(
		sec_tag,
		cred_type,
		&exists,
		&unused
	);

	if (err)
	{
		printk("Failed to check for certificates err %d\n", err);
		return err;
	}

	//	if (exists)
	//	{
	//		/* For the sake of simplicity we delete what is provisioned
	//		 * with our security tag and reprovision our certificate.
	//		 */
	//		err = modem_key_mgmt_delete(sec_tag, cred_type);
	//
	//		if (err)
	//		{
	//			printk("Failed to delete existing certificate, err %d\n", err);
	//		}
	//	}


	if (!exists)
	{
		printk("Provisioning certificate\n");

		/*  Provision certificate to the modem */
		err = modem_key_mgmt_write(
			sec_tag,
			cred_type,
			buf,
			len - 1
		);

		if (err)
		{
			printk("Failed to provision certificate, err %d\n", err);
			return err;
		}
	}
	else
	{
		printk("Certificate already provisioned\n");
	}


	return 0;
}

void button_debounce_worker(struct k_work * item)
{
	k_msleep(50);
	struct service_info * the_service = CONTAINER_OF(item, struct service_info, work);

	if (gpio_pin_get(the_service->device, the_service->gpio_node->gpio_pin))
	{
		//button_int_event = true;

		data_publish1(&client, MQTT_QOS_1_AT_LEAST_ONCE, "/button", "ON");
	}
}

void main(void)
{
	int err;


	for (int i = 0; i < 3; i++)
	{
		leds[i] = pin_init(&led_nodes[i]);
	}

	button0 = pin_init(&button0_node);

	k_work_init(&button_debounce_service.work, button_debounce_worker);

	//bt_int = pin_init(&bt_int_node);

	while (gpio_pin_get(button0, button0_node.gpio_pin) == 0)
	{
		gpio_pin_set(leds[0], led_nodes[0].gpio_pin, 1);
		k_sleep(K_MSEC(500));
		gpio_pin_set(leds[0], led_nodes[0].gpio_pin, 0);
		k_sleep(K_MSEC(500));
	}

	printk("The MQTT simple sample started\n");

	/*
		err = nrf_modem_lib_init();
		if (err)
		{
			printk("Failed to initialize modem library!");
			return;
		}
	*/

	lte_lc_power_off();

	err = client_id_get();

	if (err)
	{
		printk("ERROR: Failed to read IMEI.\n");
		return;
	}

	printk("Client Id: %s\n", imei);

	/* Initialize AT comms in order to provision the certificate */
	err = at_comms_init();

	if (err)
	{
		return;
	}

	/* Provision certificates before connecting to the LTE network */
	err = cert_provision1(
		sec_tag_list[0],
		MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN,
		ca_cert,
		sizeof(ca_cert)
	);

	if (err)
	{
		return;
	}

	err = cert_provision1(
		sec_tag_list[0],
		MODEM_KEY_MGMT_CRED_TYPE_PUBLIC_CERT,
		private_cert,
		sizeof(private_cert)
	);

	if (err)
	{
		return;
	}

	err = cert_provision1(
		sec_tag_list[0],
		MODEM_KEY_MGMT_CRED_TYPE_PRIVATE_CERT,
		private_key,
		sizeof(private_key)
	);

	if (err)
	{
		return;
	}


	modem_configure();

	client_init(&client);

	button0 = pin_int_init(&button0_node, &button_cb_data, button_pressed);
	button_debounce_service.device = button0;
	button_debounce_service.gpio_node = &button0_node;


	while (1)
	{
		err = mqtt_connect(&client);

		if (err != 0)
		{
			printk("ERROR: mqtt_connect %d\n", err);
			//return;
		}
		else
		{

			err = fds_init(&client);

			if (err != 0)
			{
				printk("ERROR: fds_init %d\n", err);
				//return;
			}
			else
			{

				while (1)
				{
					err = poll(&fds, 1, mqtt_keepalive_time_left(&client));

					if (err < 0)
					{
						printk("ERROR: poll %d\n", errno);
						break;
					}

					err = mqtt_live(&client);

					if ((err != 0) && (err != -EAGAIN))
					{
						printk("ERROR: mqtt_live %d\n", err);
						break;
					}

					if ((fds.revents & POLLIN) == POLLIN)
					{
						err = mqtt_input(&client);

						if (err != 0)
						{
							printk("ERROR: mqtt_input %d\n", err);
							break;
						}
					}

					if ((fds.revents & POLLERR) == POLLERR)
					{
						printk("POLLERR\n");
						break;
					}

					if ((fds.revents & POLLNVAL) == POLLNVAL)
					{
						printk("POLLNVAL\n");
						break;
					}

				}

				printk("Disconnecting MQTT client...\n");

				err = mqtt_disconnect(&client);

				if (err)
				{
					printk("Could not disconnect MQTT client. Error: %d\n", err);
				}
			}
		}

		k_sleep(K_SECONDS(10));
	}
}

/*
static void bt_int_thread()
{
	while (1)
	{
		gpio_pin_set(leds[0], led_nodes[0].gpio_pin, gpio_pin_get(bt_int, bt_int_node.gpio_pin));
		k_sleep(K_MSEC(100));
	};
}

K_THREAD_DEFINE(bt_int_thread, 1024, bt_int_thread, NULL, NULL, NULL, 7, 0, 0);
*/
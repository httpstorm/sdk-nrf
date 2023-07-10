/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <string.h>
#include <zephyr.h>
#include <stdlib.h>
#include <net/socket.h>
#include <modem/bsdlib.h>
#include <net/tls_credentials.h>
#include <modem/lte_lc.h>
#include <modem/at_cmd.h>
#include <modem/at_notif.h>
#include <modem/modem_key_mgmt.h>
#include <modem/modem_info.h>

#include <device.h>
#include <drivers/flash.h>
#include <fs/nvs.h>
#include <stdio.h>
#include <power/reboot.h>

#include <net/aws_iot.h>
#include <data/json.h>
#include <date_time.h>

#include <drivers/gpio.h>
#include <crypto/cipher.h>

#include "devicetree_legacy_unfixed.h"

#include "protec3.h"

/******************** Macros *******************************/

#define HTTPS_PORT 443

#define EXECUTE_API \
        "POST /prod/DynamoDBManager HTTP/1.1\r\n" \
        "Host: smf93sjfp7.execute-api.eu-central-1.amazonaws.com\r\n" \
        "Authorization: %s\r\n" \
        "Content-Length: %d\r\n" \
        "\r\n" \
        "%s"

#define ALLOWED_CLOCK_DIFF 30

#define HTTP_HDR_END "\r\n\r\n"

#define BUF_SIZE 2048
#define TLS_SEC_TAG CONFIG_AWS_IOT_SEC_TAG

#define NVS_ID_USER 1
#define NVS_ID_SECRET 2

/* The devicetree node identifier for the "led0" alias. */
#define LED_R_NODE DT_ALIAS(led0)
#define LED_R	DT_GPIO_LABEL(LED_R_NODE, gpios)
#define PIN_R	DT_GPIO_PIN(LED_R_NODE, gpios)
#define FLAGS_R	DT_GPIO_FLAGS(LED_R_NODE, gpios)

#define LED_G_NODE DT_ALIAS(led1)
#define LED_G	DT_GPIO_LABEL(LED_G_NODE, gpios)
#define PIN_G	DT_GPIO_PIN(LED_G_NODE, gpios)
#define FLAGS_G	DT_GPIO_FLAGS(LED_G_NODE, gpios)

#define LED_B_NODE DT_ALIAS(led2)
#define LED_B	DT_GPIO_LABEL(LED_B_NODE, gpios)
#define PIN_B	DT_GPIO_PIN(LED_B_NODE, gpios)
#define FLAGS_B	DT_GPIO_FLAGS(LED_B_NODE, gpios)

#define CRYPTO_DRV_NAME CONFIG_CRYPTO_TINYCRYPT_SHIM_DRV_NAME
/******************** Types ***********************/
typedef enum
{
	uninitialized,
	initialized,
	paired
} tDeviceState;

typedef struct
{
        char *data;
} tAWSData;

/******************* Global variables **************/


static char sim[21];
static char id[16];
static char delta_topic[48];

/* Amazon-Root CA 1 */
static const char cacert[] = {
	#include "../cert/Amazon-Root"
};
/* My certificate */
static const char cert[] = {
        #include "../cert/1df19e7fd5.cert"
};
/* My key */
static const char key[] = {
        #include "../cert/1df19e7fd5.private"
};

static struct nvs_fs fs;

static tDeviceState deviceState = uninitialized;
static u32_t targetState = 0;
static char recv_buf[BUF_SIZE];
static char send_buf[BUF_SIZE];

static struct pollfd fds;

static bool aws_iot_ready = false;

static struct device *led_dev[3];

static struct json_obj_descr delta_state[] = {
        JSON_OBJ_DESCR_PRIM(tState, TargetState, JSON_TOK_STRING),
        JSON_OBJ_DESCR_PRIM(tState, Pairing, JSON_TOK_NUMBER),
        JSON_OBJ_DESCR_PRIM(tState, auth, JSON_TOK_STRING),
        JSON_OBJ_DESCR_PRIM(tState, data, JSON_TOK_STRING),
        JSON_OBJ_DESCR_PRIM(tState, iv, JSON_TOK_STRING)
};
struct json_obj_descr delta[] = {
        JSON_OBJ_DESCR_OBJECT(tDelta, state, delta_state)
};

static struct json_obj_descr pair_request[] = {
        JSON_OBJ_DESCR_PRIM(tPairRequest, user, JSON_TOK_STRING),
        JSON_OBJ_DESCR_PRIM(tPairRequest, device, JSON_TOK_STRING),
        JSON_OBJ_DESCR_PRIM(tPairRequest, time, JSON_TOK_STRING)
};

char __aligned(4) aws_msgq_buffer[10 * sizeof(tAWSData)];
struct k_msgq aws_msgq;

BUILD_ASSERT(sizeof(cacert) < KB(4), "CA Certificate too large");
BUILD_ASSERT(sizeof(cert) < KB(4), "Certificate too large");
BUILD_ASSERT(sizeof(key) < KB(4), "Key too large");

static u8_t ccm_hdr[8] = {
	'P', 'r', 'o', 't', 'e', 'c', '3', 0
};

/**************** Functions **********************/

int ccm_mode(struct device *dev, u8_t *key, size_t keylen, u8_t *iv,
              u8_t *cipher, size_t cipher_len, u8_t *plain, size_t plain_len)
{
        //int i;
        int ret = -1;
	struct cipher_ctx ini = {
		.keylen = keylen,
		.key.bit_stream = key,
		.mode_params.ccm_info = {
			.nonce_len = 13,
			.tag_len = 16,
		},
		.flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS,
	};

	struct cipher_pkt decrypt = {
		.in_buf = cipher,
		.in_len = cipher_len,
		.out_buf = plain,
		.out_buf_max = plain_len,
	};
	struct cipher_aead_pkt ccm_op = {
		.ad = ccm_hdr,
		.ad_len = sizeof(ccm_hdr),
		.pkt = &decrypt,
		/* TinyCrypt always puts the tag at the end of the ciphered
		 * text, but other library such as mbedtls might be more
		 * flexible and can take a different buffer for it.  So to
		 * make sure test passes on all backends: enforcing the tag
		 * buffer to be after the ciphered text.
		 */
		.tag = cipher + cipher_len,
	};

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_CCM,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		return -1;
	}

#if 0
	ccm_op.pkt = &decrypt;
	ccm_op.tag = challenge + 67;
#endif
	if (cipher_ccm_op(&ini, &ccm_op, /*ccm_nonce*/ iv)) {
		printk("CCM mode DECRYPT - Failed");
		goto out;
	}

	printk("Output length (decryption): %d", decrypt.out_len);
        ret = 0;
out:
	cipher_free_session(dev, &ini);
        return ret;
}


/* Initialize AT communications */
int at_comms_init(void)
{
	int err;

	err = at_cmd_init();
	if (err) {
		printk("Failed to initialize AT commands, err %d\n", err);
		return err;
	}

	err = at_notif_init();
	if (err) {
		printk("Failed to initialize AT notifications, err %d\n", err);
		return err;
	}

	return 0;
}

/* Provision certificate to modem */
int cert_provision(void)
{
	int err;
	bool exists = false;
	u8_t unused;
        int i;
        size_t len;
        enum modem_key_mgnt_cred_type types[] = {
                MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN,
                MODEM_KEY_MGMT_CRED_TYPE_PRIVATE_CERT,
                MODEM_KEY_MGMT_CRED_TYPE_PUBLIC_CERT,
        };
        const char *certs[] = {cacert, key, cert};
        enum at_cmd_state state;
        err = at_cmd_write("AT+CFUN=4", send_buf, sizeof(send_buf), &state);
        if (err)
        {
                printk("at_cmd_write failed %d\n", errno);
        }

        for (i = 0; 3 != i; ++i)
        {
                exists = false;

                err = modem_key_mgmt_exists(TLS_SEC_TAG,
                                            types[i],
                                            &exists, &unused);
                if (err)
                {
                        printk("Failed to check for certificates err %d\n", err);
                        return err;
                }

                len = strlen(certs[i]);

                if (exists)
                {
                        /* nothing to do, if our certificate is already installed */
                        if (0 == modem_key_mgmt_cmp(TLS_SEC_TAG, types[i],
                                        certs[i], len))
                        {
                                  continue;
                        }
                        /* Delete old certificate
                         */
                        err = modem_key_mgmt_delete(TLS_SEC_TAG, types[i]);
                        if (err)
                        {
                                  printk("Failed to delete certificate, err %d\n", err);
                                  return err;
                        }
                }

                /*  Provision certificate to the modem */
                err = modem_key_mgmt_write(TLS_SEC_TAG,
                                           types[i],
                                           certs[i], len);
                if (err)
                {
                        printk("Failed to provision certificate, err %d\n", err);
                        return err;
                }
        }

	return 0;
}

static void aws_publish(char *payload, size_t len)
{
        int err;
        struct aws_iot_tx_data tx_data;
        tx_data.str = payload;
        tx_data.len = len;
        tx_data.qos = MQTT_QOS_0_AT_MOST_ONCE;
        tx_data.topic.type = AWS_IOT_SHADOW_TOPIC_UPDATE;
        err = aws_iot_send(&tx_data);
        if (err)
        {
                printk("aws_iot_send failed %d\n", errno);
        } else
        {
                printk("%s\n", payload);
        }
}

void aws_handler(const struct aws_iot_evt *evt)
{
        printk("aws_handler: type = %d, topic type = %d, topic = %s, data = %s\n",
                evt->type,
                evt->topic.type,
                evt->topic.str ? evt->topic.str : "(NULL)",
                evt->ptr ? evt->ptr : "(NULL)");

        switch (evt->type)
        {
        case AWS_IOT_EVT_READY:
                aws_iot_ready = true;
                break;
        case AWS_IOT_EVT_DISCONNECTED:
                aws_iot_ready = false;
                break;
        case AWS_IOT_EVT_DATA_RECEIVED:
                if (evt->ptr)
                {
                        tAWSData msg;
                        msg.data = strncpy(recv_buf, evt->ptr, BUF_SIZE);
                        /* send data to consumers */
                        while (k_msgq_put(&aws_msgq, &msg, K_NO_WAIT) != 0)
                        {
                                /* message queue is full: purge old data & try again */
                                k_msgq_purge(&aws_msgq);
                        }
                }
                break;
        default:
                break;
        }
}

/* converts a number to decimal string
 * str - buffer for output
 * n - size of buffer
 * nummber - number to convert
 * return: -1 on failure, number of digits on success
 * str will always be '\0' terminated, if neccassary
 * digits are dropped (from left)
 */
static int s64_to_str(char *str, size_t n, s64_t number)
{
    char tmp;
    size_t j, i = 0;
    if (!n)
    {
        return -1;
    }
    if (1 == n)
    {
        str[0] = 0;
        return 0;
    }
    if (0LL == number)
    {
        str[0] = '0';
        str[1] = 0;
        return 1;
    }
    while ((0LL != number) && (i < n - 1))
    {
        str[i++] = (number % 10) + '0';
        number /= 10;
    }
    str[i] = 0;
    for (j = 0; j < i/2; ++j)
    {
        tmp = str[i-1-j];
        str[i-1-j] = str[j];
        str[j] = tmp;
    }
    return i;
}

/* converts a hex digit character to an u8_t value,
   undefined for illegal digit
 */
static u8_t hexdigit(char digit)
{
    if (('0' <= digit) && ('9' >= digit))
    {
        return digit - '0';
    } if (('a' <= digit) && ('f' >= digit))
    {
        return digit - 'a' + 10;
    } else
    {
        return digit - 'A' + 10;
    }
}

/* converts a hex string to a binary array
 * bin - [output] produced binary array
 * hex - [input] hex string to be processed
 * len - available length of bin array
 * return: number of generated bytes
 */
static size_t hex_to_bin(char *bin, const char *hex, size_t len)
{
    size_t i;
    for (i = 0; (strlen(hex)/2 != i) && (i != len); ++i)
    {
        bin[i] = hexdigit(hex[2*i]) * 16 + hexdigit(hex[2*i+1]);
    }
    return i;
}

void network_thread(void)
{
    int err;
    //int fd;
    struct flash_pages_info info;
    struct aws_iot_config aws_config;
    struct aws_iot_topic_data subs[1];
    struct device *crypto_dev;

    printk("PROTEC3 started\n");

    k_msgq_init(&aws_msgq, aws_msgq_buffer, sizeof(tAWSData), 10);

    crypto_dev = device_get_binding(CRYPTO_DRV_NAME);
    //ccm_mode(crypto_dev);

    /* non volatile storage */
    fs.offset = DT_FLASH_AREA_STORAGE_OFFSET;
    err = flash_get_page_info_by_offs(
                device_get_binding(DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL),
                fs.offset, &info);
    if (err)
    {
        printk("Unable to get page info");
    }
    fs.sector_size = info.size;
    fs.sector_count = 3;
    err = nvs_init(&fs, DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL);
    if (err)
    {
        printk("Flash Init failed");
    }

    err = nvs_read(&fs, NVS_ID_SECRET, &recv_buf, 33);
    if (0 < err)
    {
        err = nvs_read(&fs, NVS_ID_USER, &recv_buf, sizeof(recv_buf));
	if (0 < err)
	{
            deviceState = paired;
	} else
	{
            deviceState = initialized;
	}
    }

    printk("deviceState = %d\n", deviceState);

    err = bsdlib_init();
    if (err)
    {
        printk("Failed to initialize bsdlib!");
	goto cleanup;
    }

    /* Initialize AT comms in order to provision the certificate */
    err = at_comms_init();
    if (err)
    {
        goto cleanup;
    }


    enum at_cmd_state state;
	err = at_cmd_write("AT+CFUN=4", send_buf, sizeof(send_buf), &state);
	if (err)
	{
			printk("at_cmd_write failed %d\n", errno);
	}

	err = at_cmd_write("AT%XSYSTEMMODE=0,1,0,0", recv_buf, BUF_SIZE, &state);
	printk("at_cmd_write XSYSTEMMODE: %d %d\n", err, state);

	err = at_cmd_write("AT+CFUN=1", send_buf, sizeof(send_buf), &state);
	if (err)
	{
			printk("at_cmd_write failed %d\n", errno);
	}

    /* Provision certificates before connecting to the LTE network */
    if (uninitialized == deviceState)
    {
        /* TODO get certificates */
        err = cert_provision();
        if (err)
        {
            goto cleanup;
        }
    }
#if 0
    err = modem_info_init();
    if (err)
    {
        return;
    }
    err =  modem_info_string_get(MODEM_INFO_IMEI, id, 16);

    /* TODO only for testing purposes in production keep id unchanged,
     * i. e. comment the following line */
    strncpy(id, "thingy91", 16);
#endif

    printk("Waiting for network.. ");
    err = lte_lc_init_and_connect();
    if (err)
    {
        printk("Failed to connect to the LTE network, err %d\n", err);
	goto cleanup;
    }
    printk("OK\n");
    init_device(NULL, NULL);

    err = at_cmd_write("AT%XMODEMTRACE=1,2", recv_buf, BUF_SIZE, &state);
    printk("at_cmd_write: %d %d\n", err, state);
    /*err = at_cmd_write("AT+CFUN=0", recv_buf, BUF_SIZE, &state);
    printk("at_cmd_write: %d %d\n", err, state);*/

    err =  modem_info_string_get(MODEM_INFO_ICCID, sim, 21);
    if (err)
    {
        sim[19] = 0;
    }

    err = 1;
    while (err)
    {
        /* white for time ready */
        s64_t n;
        err = date_time_now(&n);
        if (err)
        {
            printk("dtn: %d\n", errno);
        } else
        {
            s64_to_str(recv_buf, sizeof(recv_buf), n);
            printk("%s\n", recv_buf);
        }
        k_msleep(1000);
    }

    aws_config.client_id = id;
    aws_config.client_id_len = strlen(id);

    err = aws_iot_init(&aws_config, aws_handler);
    if (err)
    {
        printk("aws_iot_init failed %d\n", errno);
    }

reconnect:
    snprintf(delta_topic, 48, "$aws/things/%s/shadow/update/delta", id);
    subs[0].str = delta_topic;
    subs[0].len = strlen(subs[0].str);

    err = aws_iot_subscription_topics_add(subs, 1);
    if (err)
    {
        printk("aws_iot_subscription_topics_add failed %d\n", errno);
    }

    err = aws_iot_connect(&aws_config);
    if (err)
    {
        printk("aws_iot_connect failed %d %d\n", err, errno);
    } else
    {
        printk("aws_iot_connect done, sleeping\n");
    }

    fds.fd = aws_config.socket;
    fds.events = POLLIN;

    while (!aws_iot_ready)
    {
        err = poll(&fds, 1, 1000);
        if (0 > err)
        {
            printk("poll failed %d\n", errno);
        }
        if (POLLIN == (fds.revents & POLLIN))
        {
            err = aws_iot_input();
            if (err)
            {
                printk("aws_iot_input failed %d\n", errno);
            }
        }
        err = aws_iot_ping();
        if (err)
        {
            printk("aws_iot_ping failed %d\n", errno);
        }
    }

    if (uninitialized == deviceState)
    {
        err = snprintf(send_buf, sizeof(send_buf),
                       "{\"state\": {\"reported\": {\"KundenId\": \"%s\", \"WhoAmI\": \"%s\"}}}",
                       id, sim);
        if (0 < err)
        {
            aws_publish(send_buf, err);
        }
        err = nvs_write(&fs, NVS_ID_SECRET, sim, sizeof(sim));
        if (0 > err)
        {
            printk("nvs_write SECRET failed: %d\n", errno);
            goto cleanup;
        }
        deviceState = initialized;
    }

    while (1)
    {
        tAWSData data;
        err = poll(&fds, 1, 1000);
        if (0 > err)
        {
            printk("poll failed %d\n", errno);
        }
        if (POLLIN == (fds.revents & POLLIN))
        {
            err = aws_iot_input();
            if (err)
            {
                printk("aws_iot_input failed %d\n", errno);
            }
        }
        err = aws_iot_ping();
        if (err)
        {
            printk("aws_iot_ping failed %d\n", errno);
        }
        if (!aws_iot_ready)
        {
            printk("AWS reconnect\n");
            goto reconnect;
        }
        if (0 == k_msgq_get(&aws_msgq, &data, K_NO_WAIT))
        {
            tDelta delta_info;
            memset(&delta_info, 0, sizeof(tDelta));
            json_obj_parse(data.data, strlen(data.data), delta, 1, &delta_info);
            if (1 == delta_info.state.Pairing)
            {
                size_t i, j, cipher_len;
                u8_t *iv, *key;
#if 0
                err = snprintf(send_buf, sizeof(send_buf),
                               "{\"state\": {\"reported\": {\"Pairing\": 1, "
                               "\"auth\": \"%s\", \"iv\": \"%s\", \"data\": \"%s\"}}}",
                               delta_info.state.auth,
                               delta_info.state.iv,
                               delta_info.state.data);
                if (0 < err)
                {
                    aws_publish(send_buf, err);
                }
                err = snprintf(send_buf, sizeof(send_buf),
                              "{\"state\": {\"reported\": {\"Pairing\": null, "
                              "\"auth\": null, \"iv\": null, \"data\": null}}"
                              "}");
                if (0 < err)
                {
                    aws_publish(send_buf, err);
                }
#endif
                err = snprintf(send_buf, sizeof(send_buf),
                              "{\"state\": {\"desired\": {\"Pairing\": null, "
                              "\"auth\": null, \"iv\": null, \"data\": null}}"
                              "}");
                if (0 < err)
                {
                    aws_publish(send_buf, err);
                }
                if (initialized == deviceState)
                {
                    i = hex_to_bin(send_buf, delta_info.state.data, BUF_SIZE);
                    cipher_len = i;
                    i += hex_to_bin(&send_buf[i], delta_info.state.auth, BUF_SIZE - i);
                    iv = &send_buf[i];
                    i += hex_to_bin(&send_buf[i], delta_info.state.iv, BUF_SIZE - i);
                    key = &send_buf[i];
                    /* TODO get key from BLE */
                    for (j = 0; (16 != j) && (BUF_SIZE > i); ++j, ++i)
                    {
                        send_buf[i] = (u8_t) j;
                    }
                    if (0 == ccm_mode(crypto_dev, key, 16, iv,
                                      send_buf, cipher_len, recv_buf, BUF_SIZE))
                    {
                        tPairRequest req;
                        memset(&req, 0, sizeof(tPairRequest));
                        if (0 <= json_obj_parse(recv_buf, strlen(recv_buf),
                                                pair_request, 3, &req))
                        {
                            s64_t n;
                            err = date_time_now(&n);
                            n /= 1000; // seconds instead of milliseconds
                            if ((0 == strcmp(id, req.device)) && (!err) &&
                                (abs(n-strtol(req.time, NULL, 10)) < ALLOWED_CLOCK_DIFF))
                            {
                                err = nvs_write(&fs, NVS_ID_USER, req.user, strlen(req.user));
                                if (0 > err)
                                {
                                    printk("nvs_write USER failed: %d\n", errno);
                                } else
                                {
                                    err = snprintf(send_buf, sizeof(send_buf),
                                                   "{\"state\": {\"reported\": "
                                                   "{\"KundenId\": \"%s\", \"SIM\": \"%s\", "
                                                   "\"Accepted\": \"%s\"}}}",
                                                   id,
                                                   sim,
                                                   req.user);
                                    if (0 < err)
                                    {
                                        aws_publish(send_buf, err);
                                        deviceState = paired;
                                    }
                                }
                            }
                        }
                    }
                }
            } else if (delta_info.state.TargetState)
            {
                u32_t ts;
                char *endptr;
                err = snprintf(send_buf, 128,
                               "{\"state\": {\"reported\": {\"TargetState\": \"%s\"}}}",
                               delta_info.state.TargetState);
                ts = strtol(delta_info.state.TargetState, &endptr, 10);
                if (endptr != delta_info.state.TargetState)
                {
                    targetState = ts;
                    printk("TargetState = %d\n", ts);
                }
                if (0 < err)
                {
                    aws_publish(send_buf, err);
                }
            }
            printk("data = %s\n", data.data);
        }
        k_msleep(1000);
        if (paired == deviceState)
        {
            if (targetState & TS_SEND_GPS)
            {
                s64_t n;
                err = date_time_now(&n);
                if (err)
                {
                    err = snprintf(send_buf, sizeof(send_buf),
                                   "{\"state\": {\"reported\": {\"KundenId\": \"%s\", "
                                   "\"payload\": {\"lat\": %f, \"long\": %f}}}}",
                                   id, 10.8, 100.7);
                } else
                {
                    err = snprintf(send_buf, sizeof(send_buf),
                                   "{\"state\": {\"reported\": {\"KundenId\": \"%s\", "
                                   "\"payload\": {\"Timestamp\": \"%lld\", "
                                   "\"lat\": %f, \"long\": %f}}}}",
                                   id,
                                   n/1000 /*seconds instead of milliseconds */,
                                   1.9,
                                   12.3);
                }
                if (0 < err)
                {
                    aws_publish(send_buf, err);
                }
            }
        }
    }

    printk("Finished, closing socket.\n");

cleanup:
#if 0
	//freeaddrinfo(res);
	(void)close(fd);
#endif
    sys_reboot(0);
}

void blink_thread(void)
{
        int i, state;
        gpio_pin_t pins[] = {PIN_R, PIN_G, PIN_B};
        const char *leds[] = {LED_R, LED_G, LED_B};

        for (i = 0; 3 != i; ++i)
        {
            led_dev[i] = device_get_binding(leds[i]);

            gpio_pin_configure(led_dev[i], pins[i], GPIO_OUTPUT_INACTIVE);

            /*gpio_pin_set(led_dev[i], pins[i], 1);
            gpio_pin_set(led_dev[i], pins[i], 0);*/
        }

        state = 1;
        while (1)
        {
            gpio_pin_set(led_dev[1], pins[1], state);
            state = !state;
            k_msleep(1000);
        }
}

K_THREAD_DEFINE(network_thread_id, /* Stack size */ 1024, network_thread,
		/* 1st parameter, void* */ NULL,
		/* 2nd parameter, void* */ NULL,
		/* 3rd parameter, void* */ NULL,
		/* priority */ 7,
		/* Thread options */ 0,
		/* delay before startup */ 0);

K_THREAD_DEFINE(blink_thread_id, /* Stack size */ 1024, blink_thread,
		/* 1st parameter, void* */ NULL,
		/* 2nd parameter, void* */ NULL,
		/* 3rd parameter, void* */ NULL,
		/* priority */ 6,
		/* Thread options */ 0,
		/* delay before startup */ 0);
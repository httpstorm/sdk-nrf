/*
 * Copyright (c) 2020 EUROS Embedded Systems GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <logging/log.h>
#include "protec3.h"
#include "../gpio/gpio_t.h"

#define MODULE protec3
LOG_MODULE_REGISTER(MODULE, CONFIG_BRIDGE_UART_LOG_LEVEL);


 // TODO: disable forwarding by default
 // nRF9160 should request forwarding explicitly
int uart_forward = 1;


static int send_mac(void)
{
	char mac[13];
	char msg[20];

	if (ble_get_mac(mac))
	{
		snprintk(msg, sizeof(msg) - 1, "?BLE %s\n", mac);
	}
	else
	{
		snprintk(msg, sizeof(msg) - 1, "?BLE none\n");
	}

	return uart_tx_enqueue_ext(msg, strlen(msg), 0);
}

int uart_process(const char * msg, int count)
{
	int ret = 0;
	int i = 0;
	static int index = 0;
	const char cmd[] = "BLE";
	const char * cmd_s[] = { "invalid", "down", "up", "query" };
	enum UART_LINK_CMD_E cmd_e = UART_LINK_CMD_INVALID;

#define CMD_SIZE (sizeof(cmd) - 1)


	if (!uart_forward || pin_get(gpio.nrf9160_int))
	{
		// internal commands between nRF9160 and nRF52840
		for (i = 0; i < count; i++)
		{
			if ((msg[i] == '\r') || (msg[i] == '\n'))
			{
				if (index == (CMD_SIZE + 1))
				{
					// match
					LOG_INF("cmd: %s", cmd_s[cmd_e + 1]);

					pin_set(gpio.nrf9160_int, 0);

					switch (cmd_e)
					{
					case UART_LINK_CMD_DOWN:
						uart_forward = 0;
						ret = 1;
						send_mac();
						break;

					case UART_LINK_CMD_UP:
						uart_forward = 1;
						ret = 1;
						send_mac();
						break;

					case UART_LINK_CMD_QUERY:
						ret = 1;
						send_mac();
						break;

					default:
						break;
					}
				}

				index = 0;
				continue;
			}

			if ((index >= 0) && (index < CMD_SIZE) && (cmd[index] == msg[i]))
			{
				index++;
			}
			else if (index == CMD_SIZE)
			{
				index++;

				switch (msg[i])
				{
				case '-':
					cmd_e = UART_LINK_CMD_DOWN;
					break;

				case '+':
					cmd_e = UART_LINK_CMD_UP;
					break;

				case '?':
					cmd_e = UART_LINK_CMD_QUERY;
					break;

				default:
					cmd_e = UART_LINK_CMD_INVALID;
					index = -1;
					break;
				}
			}
			else
			{
				cmd_e = UART_LINK_CMD_INVALID;
				index = -1;
			}
		}
	}

	return ret;
}

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

#include <sys/types.h>
#include <net/socket.h>
#include <posix/netinet/in.h>
#include <posix/unistd.h>

#include "protec3.h"

/******************** Macros *******************************/

#define HTTPS_PORT 443
#define SERVER_NAME "smf93sjfp7.execute-api.eu-central-1.amazonaws.com"
//#define SERVER_NAME "cognito-idp.eu-central-1.amazonaws.com"

#define INIT \
        "POST /prod/DynamoDBManager HTTP/1.1\r\n" \
        "Host: smf93sjfp7.execute-api.eu-central-1.amazonaws.com\r\n" \
        "tokenHeader: allow\r\n" \
        "Content-Length: %d\r\n" \
        "\r\n" \
        "{\"ID\": \"%s\", \"SIM\": \"%s\"}"

#define BUF_SIZE 3072

/******************* Global variables **************/

#if USE_HW_TLS
static char send_buf[] = COGNITO;
static char recv_buf[RECV_BUF_SIZE];
#endif

int init_device(const char *ID, const char *SIM)
{
	int err;
	int fd;
        int bytes;
	size_t off, len;
        char *send_buf, *recv_buf;
	struct addrinfo *res;
	struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM,
	};

        err = getaddrinfo(SERVER_NAME, NULL, &hints, &res);
	//err = getaddrinfo("smf93sjfp7.execute-api.eu-central-1.amazonaws.com", NULL, &hints, &res);
	if (err)
        {
		printk("getaddrinfo() failed, err %d\n", errno);
		goto error;
	}

	((struct sockaddr_in *)res->ai_addr)->sin_port = htons(HTTPS_PORT);

	fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TLS_1_2);
	if (fd == -1)
        {
		printk("Failed to open socket!\n");
		goto error;
	}

        static sec_tag_t sec_tag_list[] = {142};
        u32_t sec_tag_count = ARRAY_SIZE(sec_tag_list);

        err = setsockopt(fd, SOL_TLS,
                          TLS_SEC_TAG_LIST, sec_tag_list,
                          sizeof(sec_tag_t) * sec_tag_count);
        printk("%d, %d\n", err, errno);

#if 0
        int peer_verify = 0; // 0 disable, 1 optional, 2 mandatory
        err = setsockopt(fd, SOL_TLS, TLS_PEER_VERIFY,
		 &peer_verify,
		 sizeof(peer_verify));
        printk("%d, %d\n", err, errno);
#endif

	/* Setup TLS socket options */
	/*err = tls_setup(fd);
	if (err)
        {
		goto error;
	}*/

	printk("Connecting to %s\n", SERVER_NAME);
	err = connect(fd, res->ai_addr, sizeof(struct sockaddr_in));
	if (err)
        {
		printk("connect() failed, err: %d\n", errno);
		goto error;
	}

        send_buf = (char*) k_malloc(192);
        if (!send_buf)
        {
                printk("malloc failed for send buffer\n");
		goto error;
        }
        recv_buf = (char*) k_malloc(BUF_SIZE);
        if (!recv_buf)
        {
                printk("malloc failed for receive buffer\n");
                k_free(send_buf);
		goto error;
        }

        snprintf(send_buf, 192, INIT, 21 + strlen(ID) + strlen(SIM), ID, SIM);
        len = strlen(send_buf);

	off = 0;
	do {
		bytes = send(fd, &send_buf[off], len - off, 0);
		if (bytes < 0) {
			printk("send() failed, err %d\n", errno);
			goto clean_up;
		}
		off += bytes;
	} while (off < len);

	printk("Sent %d bytes\n", off);

	off = 0;
        memset(recv_buf, 0,BUF_SIZE);
	do {
		bytes = recv(fd, &recv_buf[off], BUF_SIZE - off, 0);
		if (bytes < 0) {
			printk("recv() failed, err %d\n", errno);
			goto clean_up;
		} else
                {
                  off += bytes;
                }
	} while (bytes != 0 /* peer closed connection */);

	printk("Received %d bytes\n", off);
	if (off) {
		//off = p - recv_buf;
		recv_buf[off + 1] = '\0';
		printk("\n>\t %s\n\n", recv_buf);
	}
clean_up:
        k_free(recv_buf);
        k_free(send_buf);
error:
        return -1;
}
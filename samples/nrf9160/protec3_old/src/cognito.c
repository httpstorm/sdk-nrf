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

#include <mbedtls/net.h>
#include <mbedtls/ssl.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/debug.h>

#include <fcntl.h>
//#include <mbedtls/x509_crt.h>

#include "protec3.h"

/******************** Macros *******************************/

#define USE_HW_TLS 1

#if USE_HW_TLS

# define HTTPS_PORT 8443
#define SERVER_NAME "euros-embedded.com"
//#define HTTPS_PORT 443
//#define SERVER_NAME "cognito-idp.eu-central-1.amazonaws.com"

# define COGNITO \
	"POST / HTTP/1.1\r\n" \
	"Host: cognito-idp.eu-central-1.amazonaws.com\r\n" \
	"Content-Type: application/x-amz-json-1.1\r\n" \
	"X-Amz-Target: AWSCognitoIdentityProviderService.InitiateAuth\r\n" \
	"Content-Length: 149\r\n" \
	"\r\n" \
	"{\"AuthParameters\": {\"USERNAME\": \"apeltauer\", \"PASSWORD\": \"Bb123456789!\"}, \"AuthFlow\": \"USER_PASSWORD_AUTH\", \"ClientId\": \"1099ismabrj4865jhkjfp8sc4e\"}"
//# define COGNITO "GET / HTTP/1.0\r\n\r\n"
# define COGNITO_LEN (sizeof(COGNITO) - 1)

//#define SERVER_NAME "smf93sjfp7.execute-api.eu-central-1.amazonaws.com"

# define RECV_BUF_SIZE 4072

#else

# define SERVER_PORT 8443
//#define SERVER_NAME "cognito-idp.eu-central-1.amazonaws.com"
# define SERVER_NAME "euros-embedded.com"
# define GET_REQUEST "GET / HTTP/1.0\r\n\r\n"

#endif

/******************* Global variables **************/

#if USE_HW_TLS
static char send_buf[] = COGNITO;
static char recv_buf[RECV_BUF_SIZE];
#else

static void my_debug( void *ctx, int level, const char *file, int line, const char *str )
{
  ((void) level);
  printk( "my debug> %s:%04d: %s\n", file, line, str );
}

static int net_would_block( const mbedtls_net_context *ctx )
{
    int err = errno;

    /*
     * Never return 'WOULD BLOCK' on a blocking socket
     */
    if( ( fcntl( ctx->fd, F_GETFL, 0 ) & O_NONBLOCK ) != O_NONBLOCK )
    {
        errno = err;
        return( 0 );
    }

    switch( errno = err )
    {
#if defined EAGAIN
        case EAGAIN:
#endif
#if defined EWOULDBLOCK && EWOULDBLOCK != EAGAIN
        case EWOULDBLOCK:
#endif
            return( 1 );
    }
    return( 0 );
}

/*
 * Read at most 'len' characters
 */
int mbedtls_net_recv( void *ctx, unsigned char *buf, size_t len )
{
    int ret;
    int fd = ((mbedtls_net_context *) ctx)->fd;

    printk("!!!!! mbedtls_net_recv(%d) = ", len);

    if( fd < 0 )
        return( MBEDTLS_ERR_NET_INVALID_CONTEXT );

    ret = (int) recv( fd, buf, len, 0 );

    printk("%d\n", ret);

    //if (0x24D == len)
    {
      unsigned char peek_buf[5];
      int r = (int) recv(fd, peek_buf, 5, ZSOCK_MSG_PEEK);
      printk("peek = %d, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n", r, peek_buf[0], peek_buf[1], peek_buf[2], peek_buf[3], peek_buf[4]);
    }

    /*if (0 < ret)
    {
        printk("recv = 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
    }*/

    if( ret < 0 )
    {
        printk("recv fail: %d %d\n", ret, errno);
        if( net_would_block( ctx ) != 0 )
            return( MBEDTLS_ERR_SSL_WANT_READ );

#if ( defined(_WIN32) || defined(_WIN32_WCE) ) && !defined(EFIX64) && \
    !defined(EFI32)
        if( WSAGetLastError() == WSAECONNRESET )
            return( MBEDTLS_ERR_NET_CONN_RESET );
#else
        if( errno == EPIPE || errno == ECONNRESET )
            return( MBEDTLS_ERR_NET_CONN_RESET );

        if( errno == EINTR )
            return( MBEDTLS_ERR_SSL_WANT_READ );
#endif

        return( MBEDTLS_ERR_NET_RECV_FAILED );
    }

    return( ret );
}

/*
 * Write at most 'len' characters
 */
int mbedtls_net_send( void *ctx, const unsigned char *buf, size_t len )
{
    int ret;
    int fd = ((mbedtls_net_context *) ctx)->fd;

    printk("!!!!! mbedtls_net_send(%d) = ", len);

    if( fd < 0 )
        return( MBEDTLS_ERR_NET_INVALID_CONTEXT );

    ret = (int) send( fd, buf, len, 0 );

    printk("%d\n", ret);

    //printk("send: %d %d\n", ret, errno);

    if( ret < 0 )
    {
        printk("send fail: %d %d\n", ret, errno);
        if( net_would_block( ctx ) != 0 )
            return( MBEDTLS_ERR_SSL_WANT_WRITE );

#if ( defined(_WIN32) || defined(_WIN32_WCE) ) && !defined(EFIX64) && \
    !defined(EFI32)
        if( WSAGetLastError() == WSAECONNRESET )
            return( MBEDTLS_ERR_NET_CONN_RESET );
#else
        if( errno == EPIPE || errno == ECONNRESET )
            return( MBEDTLS_ERR_NET_CONN_RESET );

        if( errno == EINTR )
            return( MBEDTLS_ERR_SSL_WANT_WRITE );
#endif

        return( MBEDTLS_ERR_NET_SEND_FAILED );
    }

    return( ret );
}
#endif

void get_cognito_token()
{
#if USE_HW_TLS
	int err;
	int fd;
        int bytes;
	size_t off;
	struct addrinfo *res;
	struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM,
	};

        err = getaddrinfo(SERVER_NAME, NULL, &hints, &res);
	//err = getaddrinfo("smf93sjfp7.execute-api.eu-central-1.amazonaws.com", NULL, &hints, &res);
	if (err) {
		printk("getaddrinfo() failed, err %d\n", errno);
		return;
	}

	((struct sockaddr_in *)res->ai_addr)->sin_port = htons(HTTPS_PORT);

	fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TLS_1_2);
	if (fd == -1) {
		printk("Failed to open socket!\n");
		goto clean_up;
	}

	/* Setup TLS socket options */
	err = tls_setup(fd);
	if (err) {
		goto clean_up;
	}

	printk("Connecting to %s\n", SERVER_NAME);
	err = connect(fd, res->ai_addr, sizeof(struct sockaddr_in));
	if (err) {
		printk("connect() failed, err: %d\n", errno);
		goto clean_up;
	}

	off = 0;
	do {
		bytes = send(fd, &send_buf[off], COGNITO_LEN - off, 0);
		if (bytes < 0) {
			printk("send() failed, err %d\n", errno);
			goto clean_up;
		}
		off += bytes;
	} while (off < COGNITO_LEN);

	printk("Sent %d bytes\n", off);

	off = 0;
        memset(recv_buf, 0, RECV_BUF_SIZE);
	do {
		bytes = recv(fd, &recv_buf[off], RECV_BUF_SIZE - off, 0);
		if (bytes < 0) {
			printk("recv() failed, err %d\n", errno);
			//goto clean_up;
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
        return;
#else /* USE_HW_TLS */
    mbedtls_net_context server_fd;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_ssl_context ssl;
    mbedtls_ssl_config conf;
    mbedtls_x509_crt cacert;
    int ret, len, err;
    unsigned char buf[256];
    struct addrinfo *res;
    struct addrinfo hints = {
                    .ai_family = AF_INET,
                    .ai_socktype = SOCK_STREAM,
    };

    server_fd.fd = -1; // mbedtls_net_init( &server_fd );
    mbedtls_ssl_init( &ssl );
    mbedtls_ssl_config_init( &conf );
    mbedtls_x509_crt_init( &cacert );
    mbedtls_ctr_drbg_init( &ctr_drbg );

    mbedtls_entropy_init( &entropy );

    mbedtls_debug_set_threshold(0/*xFF*/);
    if( ( ret = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy,
                           /*(const unsigned char *) pers*/ NULL,
                           /*strlen( pers )*/ 0 ) ) != 0 )
    {
        printk( " failed\n  ! mbedtls_ctr_drbg_seed returned %d\n", ret );
        goto exit;
    }

    /*
     * Start the connection
     */
    printk( "\n  . Connecting to tcp/%s/%4d...", SERVER_NAME,
                                                 SERVER_PORT );

    err = getaddrinfo(SERVER_NAME, NULL, &hints, &res);
    //err = getaddrinfo("smf93sjfp7.execute-api.eu-central-1.amazonaws.com", NULL, &hints, &res);
    if (err) {
      printk("getaddrinfo() failed, err %d\n", errno);
      return;
    }

    ((struct sockaddr_in *)res->ai_addr)->sin_port = htons(SERVER_PORT);


    if( ( server_fd.fd = socket( AF_INET, SOCK_STREAM, IPPROTO_TCP) ) < 0 )
    {
        printk( " failed\n  ! socket returned %d\n\n", errno );
        goto exit;
    }


    err = connect(server_fd.fd, res->ai_addr, sizeof(struct sockaddr_in));
    if (err) {
      printk("connect() failed, err: %d\n", errno);
      goto exit;
    }

    printk( " ok\n" );

    if( ( ret = mbedtls_ssl_config_defaults( &conf,
                                             MBEDTLS_SSL_IS_CLIENT,
                                             MBEDTLS_SSL_TRANSPORT_STREAM,
                                             MBEDTLS_SSL_PRESET_DEFAULT ) ) != 0 )
    {
        printk( " failed\n ! mbedtls_ssl_config_defaults returned %d\n\n", ret );
        goto exit;
    }
    mbedtls_ssl_conf_authmode( &conf, MBEDTLS_SSL_VERIFY_NONE );
    mbedtls_ssl_conf_rng( &conf, mbedtls_ctr_drbg_random, &ctr_drbg );
    mbedtls_ssl_conf_dbg( &conf, my_debug, stdout );

    if ((ret = mbedtls_ssl_setup( &ssl, &conf)) != 0) {
        printk("mbedtls_ssl_setup() returned -0x%04X\n", -ret);
        return;
    }

    if( ( ret = mbedtls_ssl_set_hostname( &ssl, SERVER_NAME ) ) != 0 )
    {
        printk( " failed\n ! mbedtls_ssl_set_hostname returned %d\n\n", ret );
        goto exit;
    }

    mbedtls_ssl_set_bio( &ssl, &server_fd, mbedtls_net_send, mbedtls_net_recv, NULL );
    /*
     * Write the GET request
     */
    printk( "  > Write to server:" );

    len = sprintf( (char *) buf, GET_REQUEST );

    while( ( ret = mbedtls_ssl_write( &ssl, buf, len ) ) <= 0 )
    {
        if( ret != 0 )
        {
            printk( " failed\n  ! write returned %d\n\n", ret );
            goto exit;
        }
    }

    len = ret;
    printk( " %d bytes written\n\n%s", len, (char *) buf );

    /*
     * Read the HTTP response
     */
    printk( "  < Read from server:" );
    do
    {
        len = sizeof( buf ) - 1;
        memset( buf, 0, sizeof( buf ) );
        ret = mbedtls_ssl_read( &ssl, buf, len );

        if( ret <= 0 )
        {
            printk( "failed\n  ! ssl_read returned %d\n\n", ret );
            break;
        }

        len = ret;
        printk( " %d bytes read\n\n%s", len, (char *) buf );
    }
    while( 1 );

exit:
    if( -1 != server_fd.fd )
    {
        shutdown( server_fd.fd, 2 );
        close( server_fd.fd );
        server_fd.fd = -1;
    }
    mbedtls_ssl_free( &ssl );
    mbedtls_ssl_config_free( &conf );
    mbedtls_ctr_drbg_free( &ctr_drbg );
    mbedtls_entropy_free( &entropy );
    //close( server_fd );
#endif /* USE_HW_TLS */
}
#include <string.h>
#include "bionic.h"
#include "memio.h"
#include "serial.h"
#include "utils.h"

#include "fernvale-pmic.h"
#include "fernvale-spi.h"
#include "scriptic.h"

extern struct scriptic set_plls;
extern struct scriptic spi_run;
extern struct scriptic spi_init;

void usb_uart_read(void *buffer, int bytes, int timeout);
void usb_uart_write(const void *data, int bytes, int timeout);
void usb_uart_flush(void);

int serial_putc(uint8_t c) {
	usb_uart_write(&c, 1, 0);
	return 0;
}

uint8_t serial_getc(void) {
	uint8_t d;
	usb_uart_read(&d, 1, 0);
	return d;
}

void serial_write(const void *d, int bytes)
{
	usb_uart_write(d,bytes,0);
}

int serial_puts(const void *s)
{
	const char *str = s;
	while(*str) {
		/* Fix up linefeeds */
		if (*str == '\n')
			serial_putc('\r');
		serial_putc(*str++);
	}
	usb_uart_flush();
	return 0;
}

int serial_read(void *data, int bytes)
{
	int i;
	uint8_t *d = data;
	for (i=0;i<bytes;i++) d[i] = serial_getc();
	return 0;
}


static void spi_cmd_txrx(uint8_t tx_size, uint8_t rx_size,
		 uint8_t *tx_buf, uint8_t *rx_buf)
{
	memcpy(SPI_DATA, tx_buf, tx_size);
	writel(tx_size, SPI_WRITE_COUNT);
	writel(rx_size, SPI_READ_COUNT);
	scriptic_execute(&spi_run);
	memcpy(rx_buf, SPI_DATA + tx_size, rx_size);
}


#define FRSER_NAME "Fernly SPI"

#define CHIP_BUSTYPE_SPI (1 << 3)

#define S_ACK 0x06
#define S_NAK 0x15

#define S_CMD_O_DELAY           0x0E            /* Write opbuf: udelay                          */
#define S_CMD_S_BUSTYPE		0x12		/* Set used bustype(s).				*/
#define S_CMD_O_SPIOP		0x13		/* Perform SPI operation.			*/

#define S_MAXCMD 0x13
#define S_MAXLEN 0x06

const char ca_iface[3] = { S_ACK, 0x01, 0x00 };
const char ca_bitmap[33] = { S_ACK, 0xBF, 0xC9, 0xF, 0, 0 };
const char ca_pgmname[17] = "\x06" FRSER_NAME; /* Small hack to include S_ACK in the name. */
const char ca_serbuf[3] = { S_ACK, 2, 0 };
const char ca_syncnop[2] = { S_NAK, S_ACK };

const char ca_opbufsz[3] = { S_ACK, 128, 0 };
const char ca_wrnlen[4] = { S_ACK, 128, 0, 0 };
const char ca_rdnmaxlen[4] = { S_ACK, 128, 0, 0 };
const char ca_bustypes[2] = { S_ACK, CHIP_BUSTYPE_SPI };

struct constanswer {
	uint8_t len;
	const char* data;
};

/* Commands with a const answer cannot have parameters */
static const struct constanswer const_table[S_MAXCMD+1] = {
	{ 1, ca_iface },	// NOP
	{ 3, ca_iface },	// IFACE V
	{ 33, ca_bitmap },	// op bitmap
	{ 17, ca_pgmname },	// programmer name
	{ 3, ca_serbuf },	// serial buffer size
	{ 2, ca_bustypes },
	{ 0, NULL },
	{ 3, ca_opbufsz },	// operation buffer size
	{ 4, ca_wrnlen },	// write-n max len
	{ 0, NULL },		// read byte
	{ 0, NULL },		// read n
	{ 1, ca_iface },	// init opbuf
	{ 0, NULL },		// opbuf, write-1
	{ 0, NULL },		// opbuf, write-n
	{ 0, NULL },		// opbuf, delay
	{ 1, ca_iface },	// exec opbuf
	{ 2, ca_syncnop },	// sync nop
	{ 4, ca_rdnmaxlen },	// Read-n maximum len
	{ 0, NULL },		// Set bustype
	{ 0, NULL }		// SPI operation
};

static const uint8_t op2len[S_MAXCMD+1] = {
	/* A table to get  parameter length from opcode if possible (if not 0) */
	0x00, 0x00, 0x00,	/* NOP, iface, bitmap */
	0x00, 0x00, 0x00,	/* progname, serbufsize, bustypes */
	0x00, 0x00, 0x00,	/* chipsize, opbufsz, query-n maxlen */
	0x03, 0x06, 0x00,	/* read byte, read n, init opbuf */
	0x04, 0x00, 0x04,	/* write byte, write n, write delay */
	0x00, 0x00, 0x00,	/* Exec opbuf, syncnop, max read-n */
	0x01, 0x06	/* Set used bustype, SPI op */
};


static void do_cmd_spiop(uint8_t *parbuf) {
	uint8_t sbytes = parbuf[0];
	uint8_t rbytes = parbuf[3];
	uint8_t out[256];
	uint8_t in[256];
	serial_read(out, sbytes);
	spi_cmd_txrx(sbytes, rbytes, out, in);
	serial_putc(S_ACK);
	serial_write(in, rbytes);
}

static void frser_operation(uint8_t op) {
	uint8_t parbuf[S_MAXLEN]; /* Parameter buffer */
	uint8_t a_len,p_len;

	if (op > S_MAXCMD) {
		/* Protect against out-of-bounds array read. */
		serial_putc(S_NAK);
		return;
	}

	a_len = const_table[op].len;
	/* NOTE: Operations that have a const answer cannot have parameters !!!    */
	if (a_len) {
		const char *data = const_table[op].data;
		serial_write(data, a_len);
		return;
	}

	p_len = op2len[op];
	if (p_len) serial_read(parbuf, p_len);

	/* These are the operations that need real acting upon: */
	switch (op) {
		default:
			serial_putc(S_NAK);
			break;

		case S_CMD_O_DELAY:
		case S_CMD_S_BUSTYPE:
			serial_putc(S_ACK);
			break;

		case S_CMD_O_SPIOP:
			do_cmd_spiop(parbuf);
			break;
	}
}

int main()
{
	uint8_t out[1] = { 0x9F };
	uint8_t in[3];
	serial_puts(FRSER_NAME "\n");

	/* Disable system watchdog */
	writel(0x2200, 0xa0030000);
	/* Enable USB Download mode (required for no-battery operation) */
	writew(0x8000, PMIC_CTRL10);
	/* Disable battery watchdog */
	writew(0x2, PMIC_CTRL9);

	scriptic_execute(&set_plls);
	scriptic_execute(&spi_init);
	serial_puts("Initialized.\n");
	spi_cmd_txrx(1, 3, out, in);
	serial_puts("RDID:");
	serial_puth(in[0], 2);
	serial_puth(in[1], 2);
	serial_puth(in[2], 2);
	serial_puts("\n> ");
	for(;;) {
		frser_operation(serial_getc());
		usb_uart_flush();
	}
	return 0;
}


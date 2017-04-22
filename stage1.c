#include <string.h>
#include "bionic.h"
#include "memio.h"
#include "printf.h"
#include "serial.h"
#include "utils.h"

#include "fernvale-pmic.h"

char welcome_banner1[] = "Fernly stage 1 loader for 6261\r\n";
char welcome_banner2[] = "Write four bytes of program size, then write program data...\r\n>";

#define LOADADDR 0x70000000
// for osmocom gen'd firmware use 0x40000000
//#define LOADADDR 0x40000000

int main()
{
	uint32_t i;
	uint32_t psize;
	uint8_t *p;
	void (*jumpaddr)(void);

	serial_init();
	serial_puts(welcome_banner1);
	serial_puts(welcome_banner2);

	p = (uint8_t *) &psize;
	for (i=0; i<4; ++i)
		p[i] = serial_getc();

	p = (uint8_t *) LOADADDR;
	for (i=0; i<psize; ++i)
		p[i] = serial_getc();

	jumpaddr = (void (*)(void)) LOADADDR;
	jumpaddr();

	return 0;
}


/*
 * USBasp - USB in-circuit programmer for Atmel AVR controllers
 *
 * Thomas Fischl <tfischl@gmx.de>
 *
 * License........: GNU GPL v2 (see Readme.txt)
 * Target.........: ATMega8 at 12 MHz
 * Creation Date..: 2005-02-20
 * Last change....: 2009-02-28
 *
 * PC2 SCK speed option.
 * GND  -> slow (8khz SCK),
 * open -> software set speed (default is 375kHz SCK)
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/boot.h>
#include <avr/wdt.h>

#include "usbasp.h"
#include "usbdrv.h"
#include "clock.h"
#include "uart.h"

#define MODULE_NAME "btld"
#define LOGGING_ENABLE 1
#include "logging.h"

static uchar replyBuffer[8];

static uchar prog_state = PROG_STATE_IDLE;
static uchar prog_sck = USBASP_ISP_SCK_AUTO;

static uchar prog_address_newmode = 0;
static unsigned long prog_address;
static unsigned int prog_nbytes = 0;
static unsigned int prog_pagesize;
static uchar prog_blockflags;
static uchar prog_pagecounter;

uchar usbFunctionSetup(uchar data[8]) {

	uchar len = 0;

	if (data[1] == USBASP_FUNC_CONNECT) {
		log_print("connecting");


		/* set compatibility mode of address delivering */
		prog_address_newmode = 0;

		ledRedOn();

	} else if (data[1] == USBASP_FUNC_DISCONNECT) {
		log_print("disconnecting");
		ledRedOff();

	} else if (data[1] == USBASP_FUNC_TRANSMIT) {
		log_print("transmit request: %02x %02x %02x %02x \n", data[2], data[3], data[4], data[5]);

		// replyBuffer[0] = ispTransmit(data[2]);
		// replyBuffer[1] = ispTransmit(data[3]);
		// replyBuffer[2] = ispTransmit(data[4]);
		// replyBuffer[3] = ispTransmit(data[5]);

		// [0x30, 0x00, [byte], 0x00] - respond with signature bytes

		replyBuffer[0] = 0;
		replyBuffer[1] = 0;
		replyBuffer[2] = 0;
		replyBuffer[3] = boot_signature_byte_get(data[4]*2);
		len = 4;

	} else if (data[1] == USBASP_FUNC_READFLASH) {
		log_print("read flash");

		if (!prog_address_newmode)
			prog_address = (data[3] << 8) | data[2];

		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_READFLASH;
		len = 0xff; /* multiple in */

	} else if (data[1] == USBASP_FUNC_READEEPROM) {
		log_print("read EEPROM");

		if (!prog_address_newmode)
			prog_address = (data[3] << 8) | data[2];

		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_READEEPROM;
		len = 0xff; /* multiple in */

	} else if (data[1] == USBASP_FUNC_ENABLEPROG) {
		log_print("enable prog");
		replyBuffer[0] = 0;//ispEnterProgrammingMode();
		len = 1;

	} else if (data[1] == USBASP_FUNC_WRITEFLASH) {
		log_print("write flash");

		if (!prog_address_newmode)
			prog_address = (data[3] << 8) | data[2];

		prog_pagesize = data[4];
		prog_blockflags = data[5] & 0x0F;
		prog_pagesize += (((unsigned int) data[5] & 0xF0) << 4);
		if (prog_blockflags & PROG_BLOCKFLAG_FIRST) {
			prog_pagecounter = prog_pagesize;
		}
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_WRITEFLASH;
		len = 0xff; /* multiple out */

	} else if (data[1] == USBASP_FUNC_WRITEEEPROM) {
		log_print("write eeprom");

		if (!prog_address_newmode)
			prog_address = (data[3] << 8) | data[2];

		prog_pagesize = 0;
		prog_blockflags = 0;
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_WRITEEEPROM;
		len = 0xff; /* multiple out */

	} else if (data[1] == USBASP_FUNC_SETLONGADDRESS) {
		log_print("set Long address");

		/* set new mode of address delivering (ignore address delivered in commands) */
		prog_address_newmode = 1;
		/* set new address */
		prog_address = *((unsigned long*) &data[2]);

	} else if (data[1] == USBASP_FUNC_SETISPSCK) {
		log_print("set spi clock");

		/* set sck option */
		prog_sck = data[2];
		replyBuffer[0] = 0;
		len = 1;

	} else if (data[1] == USBASP_FUNC_TPI_CONNECT) {
		log_print("TPI connect");
		// tpi_dly_cnt = data[2] | (data[3] << 8);

		/* RST high */
		// ISP_OUT |= (1 << ISP_RST);
		// ISP_DDR |= (1 << ISP_RST);

		clockWait(3);

		/* RST low */
		// ISP_OUT &= ~(1 << ISP_RST);
		ledRedOn();

		clockWait(16);
		// tpi_init();
	
	} else if (data[1] == USBASP_FUNC_TPI_DISCONNECT) {
		log_print("TPI disconnect");

		// tpi_send_byte(TPI_OP_SSTCS(TPISR));
		// tpi_send_byte(0);

		clockWait(10);

		/* pulse RST */
		// ISP_OUT |= (1 << ISP_RST);
		clockWait(5);
		// ISP_OUT &= ~(1 << ISP_RST);
		clockWait(5);

		/* set all ISP pins inputs */
		// ISP_DDR &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));
		/* switch pullups off */
		// ISP_OUT &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));

		ledRedOff();
	
	} else if (data[1] == USBASP_FUNC_TPI_RAWREAD) {
		log_print("TPI raw read");
		// replyBuffer[0] = tpi_recv_byte();
		len = 1;
	
	} else if (data[1] == USBASP_FUNC_TPI_RAWWRITE) {
		log_print("TPI raw write");
		// tpi_send_byte(data[2]);
	
	} else if (data[1] == USBASP_FUNC_TPI_READBLOCK) {
		log_print("TPI readblock");
		prog_address = (data[3] << 8) | data[2];
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_TPI_READ;
		len = 0xff; /* multiple in */
	
	} else if (data[1] == USBASP_FUNC_TPI_WRITEBLOCK) {
		log_print("TPI writeblock");
		prog_address = (data[3] << 8) | data[2];
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_TPI_WRITE;
		len = 0xff; /* multiple out */
	
	} else if (data[1] == USBASP_FUNC_GETCAPABILITIES) {
		// log_print("get capabilities asked");
		replyBuffer[0] = 1;
		replyBuffer[1] = 0;
		replyBuffer[2] = 0;
		replyBuffer[3] = 0;
		len = 4;
	}

	usbMsgPtr = replyBuffer;

	return len;
}

uchar usbFunctionRead(uchar *data, uchar len) {

	uchar i;

	/* check if programmer is in correct read state */
	if ((prog_state != PROG_STATE_READFLASH) && (prog_state
			!= PROG_STATE_READEEPROM) && (prog_state != PROG_STATE_TPI_READ)) {
		return 0xff;
	}

	/* fill packet TPI mode */
	if(prog_state == PROG_STATE_TPI_READ)
	{
		// tpi_read_block(prog_address, data, len);
		prog_address += len;
		return len;
	}

	/* fill packet ISP mode */
	for (i = 0; i < len; i++) {
		if (prog_state == PROG_STATE_READFLASH) {
			// data[i] = ispReadFlash(prog_address);
		} else {
			// data[i] = ispReadEEPROM(prog_address);
		}
		prog_address++;
	}

	/* last packet? */
	if (len < 8) {
		prog_state = PROG_STATE_IDLE;
	}

	return len;
}

uchar usbFunctionWrite(uchar *data, uchar len) {

	uchar retVal = 0;
	uchar i;

	/* check if programmer is in correct write state */
	if ((prog_state != PROG_STATE_WRITEFLASH) && (prog_state
			!= PROG_STATE_WRITEEEPROM) && (prog_state != PROG_STATE_TPI_WRITE)) {
		return 0xff;
	}

	if (prog_state == PROG_STATE_TPI_WRITE)
	{
		// tpi_write_block(prog_address, data, len);
		prog_address += len;
		prog_nbytes -= len;
		if(prog_nbytes <= 0)
		{
			prog_state = PROG_STATE_IDLE;
			return 1;
		}
		return 0;
	}

	for (i = 0; i < len; i++) {

		if (prog_state == PROG_STATE_WRITEFLASH) {
			/* Flash */

			if (prog_pagesize == 0) {
				/* not paged */
				// ispWriteFlash(prog_address, data[i], 1);
			} else {
				/* paged */
				// ispWriteFlash(prog_address, data[i], 0);
				prog_pagecounter--;
				if (prog_pagecounter == 0) {
					// ispFlushPage(prog_address, data[i]);
					prog_pagecounter = prog_pagesize;
				}
			}

		} else {
			/* EEPROM */
			// ispWriteEEPROM(prog_address, data[i]);
		}

		prog_nbytes--;

		if (prog_nbytes == 0) {
			prog_state = PROG_STATE_IDLE;
			if ((prog_blockflags & PROG_BLOCKFLAG_LAST) && (prog_pagecounter
					!= prog_pagesize)) {

				/* last block and page flush pending, so flush it now */
				// ispFlushPage(prog_address, data[i]);
			}

			retVal = 1; // Need to return 1 when no more data is to be received
		}

		prog_address++;
	}

	return retVal;
}

int main(void) {
	uchar i, j;

	init_debug_uart0();

	// /* no pullups on USB and ISP pins */
	// PORTD = 0;
	// /* all outputs except PD2 = INT0 */
	// DDRD = ~(1 << 2);

	/* output SE0 for USB reset */
	// DDRB = ~0;
	j = 0;
	/* USB Reset by device only required on Watchdog Reset */
	while (--j) {
		i = 0;
		/* delay >10ms for USB reset */
		while (--i)
			;
	}
	/* all USB and ISP pins inputs */
	// DDRB = 0;

	// /* all inputs except PC0, PC1 */
	// DDRC = 0x03;
	// PORTC = 0xfe;

	/* init timer */
	clockInit();

	// DDRB |= _BV(PB7);
	// PORTB &= !_BV(PB7);

	// while(1){
	// 	clockWait(255);
	// 	clockWait(255);
	// 	clockWait(255);
	// 	__asm("sbi 0x05, 7");
	// 	clockWait(255);
	// 	clockWait(255);
	// 	clockWait(255);
	// 	__asm("cbi 0x05, 7");
	// }

	/* main event loop */
	usbInit();
	log_print("bootloader initted");
	sei();
	for (;;) {
		usbPoll();
	}
	return 0;
}


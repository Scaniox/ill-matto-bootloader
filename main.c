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

const char ram_usbDescriptorString0[] = { /* language descriptor */
	4,          /* sizeof(usbDescriptorString0): length of descriptor in bytes */
	3,          /* descriptor type */
	0x09, 0x04, /* language index (0x0409 = US-English) */
};

const int ram_usbDescriptorStringVendor[] = {
	USB_STRING_DESCRIPTOR_HEADER(13),
	'w', 'w', 'w', '.', 'f', 'i', 's', 'c', 'h', 'l', '.', 'd', 'e'
};

const int ram_usbDescriptorStringDevice[] = {
	USB_STRING_DESCRIPTOR_HEADER(6),
	'U', 'S', 'B', 'a', 's', 'p'
};

const char ram_usbDescriptorDevice[] = {    /* USB device descriptor */
	18,         /* sizeof(usbDescriptorDevice): length of descriptor in bytes */
	USBDESCR_DEVICE,        /* descriptor type */
	0x10, 0x01,             /* USB version supported */
	USB_CFG_DEVICE_CLASS,
	USB_CFG_DEVICE_SUBCLASS,
	0,                      /* protocol */
	8,                      /* max packet size */
	/* the following two casts affect the first byte of the constant only, but
	 * that's sufficient to avoid a warning with the default values.
	 */
	(char)USB_CFG_VENDOR_ID,/* 2 bytes */
	(char)USB_CFG_DEVICE_ID,/* 2 bytes */
	USB_CFG_DEVICE_VERSION, /* 2 bytes */
	USB_CFG_DESCR_PROPS_STRING_VENDOR != 0 ? 1 : 0,         /* manufacturer string index */
	USB_CFG_DESCR_PROPS_STRING_PRODUCT != 0 ? 2 : 0,        /* product string index */
	USB_CFG_DESCR_PROPS_STRING_SERIAL_NUMBER != 0 ? 3 : 0,  /* serial number string index */
	1,          /* number of configurations */
};

const char ram_usbDescriptorConfiguration[] = {    /* USB configuration descriptor */
	9,          /* sizeof(usbDescriptorConfiguration): length of descriptor in bytes */
	USBDESCR_CONFIG,    /* descriptor type */
	18 + 7 * USB_CFG_HAVE_INTRIN_ENDPOINT + 7 * USB_CFG_HAVE_INTRIN_ENDPOINT3 +
				(USB_CFG_DESCR_PROPS_HID & 0xff), 0,
	/* total length of data returned (including inlined descriptors) */
1,          /* number of interfaces in this configuration */
1,          /* index of this configuration */
0,          /* configuration name string index */
#if USB_CFG_IS_SELF_POWERED
	(1 << 7) | USBATTR_SELFPOWER,       /* attributes */
#else
	(1 << 7),                           /* attributes */
#endif
	USB_CFG_MAX_BUS_POWER / 2,            /* max USB current in 2mA units */
	/* interface descriptor follows inline: */
		9,          /* sizeof(usbDescrInterface): length of descriptor in bytes */
		USBDESCR_INTERFACE, /* descriptor type */
		0,          /* index of this interface */
		0,          /* alternate setting for this interface */
		USB_CFG_HAVE_INTRIN_ENDPOINT + USB_CFG_HAVE_INTRIN_ENDPOINT3, /* endpoints excl 0: number of endpoint descriptors to follow */
		USB_CFG_INTERFACE_CLASS,
		USB_CFG_INTERFACE_SUBCLASS,
		USB_CFG_INTERFACE_PROTOCOL,
		0,          /* string index for interface */
	#if (USB_CFG_DESCR_PROPS_HID & 0xff)    /* HID descriptor */
		9,          /* sizeof(usbDescrHID): length of descriptor in bytes */
		USBDESCR_HID,   /* descriptor type: HID */
		0x01, 0x01, /* BCD representation of HID version */
		0x00,       /* target country code */
		0x01,       /* number of HID Report (or other HID class) Descriptor infos to follow */
		0x22,       /* descriptor type: report */
		USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH, 0,  /* total length of report descriptor */
	#endif
	#if USB_CFG_HAVE_INTRIN_ENDPOINT    /* endpoint descriptor for endpoint 1 */
		7,          /* sizeof(usbDescrEndpoint) */
		USBDESCR_ENDPOINT,  /* descriptor type = endpoint */
		(char)0x81, /* IN endpoint number 1 */
		0x03,       /* attrib: Interrupt endpoint */
		8, 0,       /* maximum packet size */
		USB_CFG_INTR_POLL_INTERVAL, /* in ms */
	#endif
	#if USB_CFG_HAVE_INTRIN_ENDPOINT3   /* endpoint descriptor for endpoint 3 */
		7,          /* sizeof(usbDescrEndpoint) */
		USBDESCR_ENDPOINT,  /* descriptor type = endpoint */
		(char)(0x80 | USB_CFG_EP3_NUMBER), /* IN endpoint number 3 */
		0x03,       /* attrib: Interrupt endpoint */
		8, 0,       /* maximum packet size */
		USB_CFG_INTR_POLL_INTERVAL, /* in ms */
	#endif
};


usbMsgLen_t getStringDescriptor(struct usbRequest* rq) {
	switch (rq->wValue.bytes[0])
	{
	case 0: // usbDescriptorString0
		usbMsgPtr = ram_usbDescriptorString0;
		return sizeof(ram_usbDescriptorString0);
		break;
	case 1: // usbDescriptorStringVendor
		usbMsgPtr = ram_usbDescriptorStringVendor;
		return sizeof(ram_usbDescriptorStringVendor);
		break;
	case 2: // usbDescriptorStringDevice
		usbMsgPtr = ram_usbDescriptorStringDevice;
		return sizeof(ram_usbDescriptorStringDevice);
		break;
	case 3: // usbDescriptorStringSerialNumber
		break;

	default:
		break;
	}
}

usbMsgLen_t usbFunctionDescriptor(struct usbRequest* rq) {
	switch (rq->wValue.bytes[1]){
	case USBDESCR_DEVICE:
		usbMsgPtr = ram_usbDescriptorDevice;
		return sizeof(ram_usbDescriptorDevice);
		break;
	case USBDESCR_CONFIG:
		usbMsgPtr = ram_usbDescriptorConfiguration;
		return sizeof(ram_usbDescriptorConfiguration);
		break;
	case USBDESCR_STRING:
		return getStringDescriptor(rq);
		break;
	
	default:
	log_print("asking for unknown descriptor")
		break;
	}
}

uchar usbFunctionSetup(uchar* data) {

	usbRequest_t* rq = (void*)data;

	uchar len = 0;

	// log_print("request type: %x", rq->bmRequestType);

	if (rq->bRequest == USBASP_FUNC_CONNECT) {
		log_print("connecting");

		/* set compatibility mode of address delivering */
		prog_address_newmode = 0;

		ledRedOn();

	} else if (rq->bRequest == USBASP_FUNC_DISCONNECT) {
		log_print("disconnecting");
		ledRedOff();

	} else if (rq->bRequest == USBASP_FUNC_TRANSMIT) {
		log_print("transmit request: %02x %02x %02x %02x ", data[2], data[3], data[4], data[5]);

		// [0x30, 0x00, [byte], 0x00] - respond with signature bytes
		switch (data[2])
		{
		case 0x30:
			/* code */
			replyBuffer[3] = boot_signature_byte_get(data[4] * 2);
			len = 4;
			break;
		case 0x58:
			switch (data[3]) {
			case 0x00:
				replyBuffer[3] = boot_lock_fuse_bits_get(GET_LOCK_BITS);
				break;
			case 0x08:
				replyBuffer[3] = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
				break;
			default:
				break;
			}
			len = 4;
			break;
		case 0x50:
			switch (data[3]) {
			case 0x00:
				replyBuffer[3] = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
				break;
			case 0x08:
				replyBuffer[3] = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
				break;

			default:
				break;
			}
			len = 4;
			break;
		default:
			break;
		}


	} else if (rq->bRequest == USBASP_FUNC_READFLASH) {

		if (!prog_address_newmode)
			prog_address = (data[3] << 8) | data[2];

		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_READFLASH;
		len = 0xff; /* multiple in */
		log_print("read flash from 0x%lx", prog_address);

	} else if (rq->bRequest == USBASP_FUNC_READEEPROM) {

		if (!prog_address_newmode)
			prog_address = (data[3] << 8) | data[2];

		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_READEEPROM;
		len = 0xff; /* multiple in */
		log_print("read EEPROM 0x%lx", prog_address);

	} else if (rq->bRequest == USBASP_FUNC_ENABLEPROG) {
		log_print("enable prog");
		replyBuffer[0] = 0;//ispEnterProgrammingMode();
		len = 1;

	} else if (rq->bRequest == USBASP_FUNC_WRITEFLASH) {
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
		log_print("write flash 0x%lx", prog_address);

	} else if (rq->bRequest == USBASP_FUNC_WRITEEEPROM) {

		if (!prog_address_newmode)
			prog_address = (data[3] << 8) | data[2];

		prog_pagesize = 0;
		prog_blockflags = 0;
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_WRITEEEPROM;
		len = 0xff; /* multiple out */
		log_print("write eeprom 0x%lx", prog_address);

	} else if (rq->bRequest == USBASP_FUNC_SETLONGADDRESS) {

		/* set new mode of address delivering (ignore address delivered in commands) */
		prog_address_newmode = 1;
		/* set new address */
		prog_address = *((unsigned long*) &data[2]);
		log_print("set Long address to 0x%lx", prog_address);

	} else if (rq->bRequest == USBASP_FUNC_SETISPSCK) {
		log_print("set spi clock");

		/* set sck option */
		prog_sck = data[2];
		replyBuffer[0] = 0;
		len = 1;
	
	} else if (rq->bRequest == USBASP_FUNC_GETCAPABILITIES) {
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

uchar usbFunctionRead(uchar* data, uchar len) {

	uchar i;

	/* check if programmer is in correct read state */
	if ((prog_state != PROG_STATE_READFLASH) && (prog_state
			!= PROG_STATE_READEEPROM)) {
		return 0xff;
	}

	/* fill packet ISP mode */
	for (i = 0; i < len; i++) {
		if (prog_state == PROG_STATE_READFLASH) {
			// data[i] = ispReadFlash(prog_address);
			// data[i] = (prog_address > UINT16_MAX) ? pgm_read_byte_far(prog_address) : pgm_read_byte_near(prog_address);
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

uchar usbFunctionWrite(uchar* data, uchar len) {

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

	MCUCR = _BV(IVCE);
	MCUCR = _BV(IVSEL); // clear IVCE 

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
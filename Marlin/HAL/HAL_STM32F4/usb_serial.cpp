/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @brief USB virtual serial terminal
 */

#include "usb_serial.h"

#include "string.h"
#include "stdint.h"
#include <stdio.h>

/*
 * Hooks used for bootloader reset signalling
 */

//USBSerial SerialUSB;
#define BOARD_HAVE_SERIALUSB    1
#if BOARD_HAVE_SERIALUSB
static void rxHook(unsigned, void*);
static void ifaceSetupHook(unsigned, void*);
#endif

/*
 * USBSerial interface
 */

#define USB_TIMEOUT 50
#if BOARD_HAVE_SERIALUSB
bool USBSerial::_hasBegun = false;
#endif

#define RX_BUFFER_SIZE 256
static struct ring_buffer {
  unsigned char buffer[RX_BUFFER_SIZE];
  int head;
  int tail;
} rx_buffer;

extern "C" void store_char_usb(unsigned char c)
{
  int i = (unsigned int)(rx_buffer.head + 1) % RX_BUFFER_SIZE;

  if (i != rx_buffer.tail) {
    rx_buffer.buffer[rx_buffer.head] = c;
    rx_buffer.head = i;
  }
}

extern "C" void store_char(unsigned char c)
{
  int i = (unsigned int)(rx_buffer.head + 1) % RX_BUFFER_SIZE;

  if (i != rx_buffer.tail) {
    rx_buffer.buffer[rx_buffer.head] = c;
    rx_buffer.head = i;
  }
}

USBSerial::USBSerial(void) {
    rx_buffer.head=0;
    rx_buffer.tail=0;
}

void USBSerial::begin(void) {
}

//Roger Clark. Two new begin functions has been added so that normal Arduino Sketches that use Serial.begin(xxx) will compile.
void USBSerial::begin(unsigned long ignoreBaud)
{
}
void USBSerial::begin(unsigned long ignoreBaud, uint8_t ignore)
{
}

void USBSerial::end(void) {
}

size_t USBSerial::write(uint8 ch) {
size_t n = 0;
    this->write(&ch, 1);
		return n;
}

size_t USBSerial::write(const char *str) {
size_t n = 0;
    this->write((const uint8*)str, strlen(str));
	return n;
}

extern "C" uint8_t CDC_Send_DATA(uint8_t *ptrBuffer, uint16_t Send_length);
uint8_t com_opened=0;
size_t USBSerial::write(const uint8 *buf, uint32 len)
{
//size_t n = 0;
    /*if (!(bool) *this || !buf) {
        return 0;
    }*/
    #if SIMULATE_DEBUG
        for (uint32_t i=0;i<len;i++) putchar(buf[i]);
        return len;
    #endif
    
    uint16_t retry=1024;
    while (retry!=0)
    {
        //if (com_opened==0) return len;
        uint8_t ret = CDC_Send_DATA((uint8_t *)buf, len);
        if (ret==0 || ret==2) return len;
        retry--;
        //osDelay(10);
    }

    return len;
    /*
    uint32 txed = 0;
    while (txed < len) {
        txed += usb_cdcacm_tx((const uint8*)buf + txed, len - txed);
    }
    return n;*/
}

int USBSerial::available(void) {
    return (unsigned int)(RX_BUFFER_SIZE + rx_buffer.head - rx_buffer.tail) % RX_BUFFER_SIZE;
    //return usb_cdcacm_data_available();
}

int USBSerial::peek(void)
{
    if (rx_buffer.head == rx_buffer.tail) {
        return -1;
    }
    else {
        return rx_buffer.buffer[rx_buffer.tail];
    }
    /*
    uint8 b;
	if (usb_cdcacm_peek(&b, 1)==1)
	{
		return b;
	}
	else
	{
		return -1;
	}*/
}

void USBSerial::flush(void)
{
    rx_buffer.head = rx_buffer.tail;
/*Roger Clark. Rather slow method. Need to improve this */
    /*uint8 b;
	while(usb_cdcacm_data_available())
	{
		this->read(&b, 1);
	}*/
    return;
}

uint32 USBSerial::read(uint8 * buf, uint32 len) {
    // if the head isn't ahead of the tail, we don't have any characters

    /*uint32 rxed = 0;
    while (rxed < len) {
        rxed += usb_cdcacm_rx(buf + rxed, len - rxed);
    }

    return rxed;*/
}

size_t USBSerial::readBytes(char *buf, const size_t& len)
{
    /*size_t rxed=0;
    unsigned long startMillis;
    startMillis = millis();
    if (len <= 0) return 0;
    do {
        rxed += usb_cdcacm_rx((uint8 *)buf + rxed, len - rxed);
        if (rxed == len) return rxed;
    } while(millis() - startMillis < _timeout);
    return rxed;*/
}

/* Blocks forever until 1 byte is received */
int USBSerial::read(void) {
    if (rx_buffer.head == rx_buffer.tail) {
        return -1;
    }
    else {
        unsigned char c = rx_buffer.buffer[rx_buffer.tail];
        rx_buffer.tail = (unsigned int)(rx_buffer.tail + 1) % RX_BUFFER_SIZE;
        return c;
    }
    /*uint8 b;

	    this->read(&b, 1);
    return b;
	*/

	/*if (usb_cdcacm_rx(&b, 1)==0)
	{
		return -1;
	}
	else
	{
		return b;
	}*/
}

uint8 USBSerial::pending(void) {
    return 0;
    //return usb_cdcacm_get_pending();
}

uint8 USBSerial::getDTR(void) {
    return false;
    //return usb_cdcacm_get_dtr();
}

uint8 USBSerial::getRTS(void) {
    return false;
    //return usb_cdcacm_get_rts();
}

USBSerial::operator bool() {
    return true;
    //return usb_is_connected(USBLIB) && usb_is_configured(USBLIB) && usb_cdcacm_get_dtr();
}

USBSerial SerialUSB;

/*
 * Bootloader hook implementations
 */

#if BOARD_HAVE_SERIALUSB

enum reset_state_t {
    DTR_UNSET,
    DTR_HIGH,
    DTR_NEGEDGE,
    DTR_LOW
};

static reset_state_t reset_state = DTR_UNSET;

static void ifaceSetupHook(unsigned hook __attribute__((unused)), void *requestvp) {
}

#define STACK_TOP 0x20000800
#define EXC_RETURN 0xFFFFFFF9
#define DEFAULT_CPSR 0x61000000
static void rxHook(unsigned hook __attribute__((unused)), void *ignored __attribute__((unused))) {
    /* FIXME this is mad buggy; we need a new reset sequence. E.g. NAK
     * after each RX means you can't reset if any bytes are waiting. */
}

#endif  // BOARD_HAVE_SERIALUSB

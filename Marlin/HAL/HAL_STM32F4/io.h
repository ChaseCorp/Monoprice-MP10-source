#ifndef _WIRISH_IO_H_
#define _WIRISH_IO_H_

#include "mm32_types.h"
#include <boards.h>

/**
 * Specifies a GPIO pin behavior.
 * @see pinMode()
 */
typedef enum WiringPinMode {
    OUTPUT,
    OUTPUT_OPEN_DRAIN,
    INPUT,
    INPUT_ANALOG,
    INPUT_PULLUP,
    INPUT_PULLDOWN,
    INPUT_FLOATING,
    PWM,
    PWM_OPEN_DRAIN,
} WiringPinMode;

extern void analogWrite(uint8 pin, uint8 value);
extern void digitalWrite(uint8 pin, uint8 value);
extern void pinMode(uint8 pin, WiringPinMode mode);
extern uint32 digitalRead(uint8 pin);
extern void watchdog_reset();
extern void watchdog_init();

#define HIGH 0x1
#define LOW  0x0

/**
 * Writes a (digital) value to a pin.  The pin must have its
 * mode set to OUTPUT or OUTPUT_OPEN_DRAIN.
 *
 * @param pin Pin to write to.
 * @param value Either LOW (write a 0) or HIGH (write a 1).
 * @see pinMode()
 */
void digitalWrite(uint8 pin, uint8 value);

/**
 * Read a digital value from a pin.  The pin must have its mode set to
 * one of INPUT, INPUT_PULLUP, and INPUT_PULLDOWN.
 *
 * @param pin Pin to read from.
 * @return LOW or HIGH.
 * @see pinMode()
 */
uint32 digitalRead(uint8 pin);

/**
 * Read an analog value from pin.  This function blocks during ADC
 * conversion, and has 12 bits of resolution.  The pin must have its
 * mode set to INPUT_ANALOG.
 *
 * @param pin Pin to read from.
 * @return Converted voltage, in the range 0--4095, (i.e. a 12-bit ADC
 *         conversion).
 * @see pinMode()
 */
uint16 analogRead(uint8 pin);

/**
 * Shift out a byte of data, one bit at a time.
 *
 * This function starts at either the most significant or least
 * significant bit in a byte value, and shifts out each byte in order
 * onto a data pin.  After each bit is written to the data pin, a
 * separate clock pin is pulsed to indicate that the new bit is
 * available.
 *
 * @param dataPin  Pin to shift data out on
 * @param clockPin Pin to pulse after each bit is shifted out
 * @param bitOrder Either MSBFIRST (big-endian) or LSBFIRST (little-endian).
 * @param value    Value to shift out
 */
void shiftOut(uint8 dataPin, uint8 clockPin, uint8 bitOrder, uint8 value);

uint32 shiftIn( uint32 ulDataPin, uint32 ulClockPin, uint32 ulBitOrder );

#endif

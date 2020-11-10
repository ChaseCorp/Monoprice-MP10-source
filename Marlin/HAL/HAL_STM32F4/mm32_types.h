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
 *  @file libmaple/include/libmaple/libmaple_types.h
 *
 *  @brief libmaple's types, and operations on types.
 */

#ifndef _LIBMAPLE_LIBMAPLE_TYPES_H_
#define _LIBMAPLE_LIBMAPLE_TYPES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef bool boolean;

//typedef unsigned char uint8;
#define uint8    uint8_t
#define uint16   uint16_t
#define uint32   uint32_t
#define uint64   uint64_t

#define int8    int8_t
#define int16   int16_t
#define int32   int32_t
#define int64   int64_t

typedef void (*voidFuncPtr)(void);
typedef void (*voidArgumentFuncPtr)(void *);

#define __io volatile
#define __attr_flash __attribute__((section (".USER_FLASH")))
#define __packed __attribute__((__packed__))
#define __deprecated __attribute__((__deprecated__))
#define __weak __attribute__((weak))
#ifndef __always_inline
#define __always_inline __attribute__((always_inline))
#endif
#ifndef __unused
#define __unused __attribute__((unused))
#endif

#ifndef NULL
#define NULL 0
#endif

#ifndef offsetof
#define offsetof(type, member) __builtin_offsetof(type, member)
#endif

#ifdef __cplusplus
}
#endif

#endif

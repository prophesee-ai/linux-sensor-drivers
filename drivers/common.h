/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __DRIVERS_COMMON_H
#define __DRIVERS_COMMON_H
#ifdef __KERNEL__
// include for msleep
#include <linux/delay.h>
// include for ARRAY_SIZE
#include <linux/kernel.h>
#else
#error "environment not supported. Please add msleep implementation"
#endif

#define RET_ON(operation) do { int r = operation; if (r != 0) return r; } while (0)

#endif // __DRIVERS_COMMON_H__

/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __GENX320_IO_H__
#define __GENX320_IO_H__
#include "../api.h"

/// 0 - Standalone, 1 - Master, 2 - Slave
int genx320_io_sync_mode(struct psee_controls *controls, enum sync_mode mode);
#endif // __GENX320_IO_H__


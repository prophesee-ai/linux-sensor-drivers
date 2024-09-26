/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __GENX320_SRAM_H__
#define __GENX320_SRAM_H__
#include "genx320_registers.h"

#define is_powered_up_dyn(ctrl, sram_cut) ({ \
	u32 sram_cut##_initn; \
	u32 sram_cut##_pd; \
	read_field(ctrl, sram_initn, sram_cut##_initn, &sram_cut##_initn); \
	read_field(ctrl, sram_pd1, sram_cut##_pd, &sram_cut##_pd); \
	(!(sram_cut##_pd) && sram_cut##_initn); \
})

#endif // __GENX320_SRAM_H__


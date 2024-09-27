/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __GENX320_MIPI_H__
#define __GENX320_MIPI_H__
#include "../api.h"

int genx320_mipi_configure(struct psee_controls *controls);
int genx320_mipi_start_pattern(struct psee_controls *controls);
#endif // __GENX320_MIPI_H__


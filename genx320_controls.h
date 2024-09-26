/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __GENX320_CONTROLS_H
#define __GENX320_CONTROLS_H
#include "genx320.h"
#include "psee_controls.h"

int genx320_init_controls(struct genx320 *genx320, const struct psee_ctrl_ops*);
#endif /* __GENX320_CONTROLS_H */

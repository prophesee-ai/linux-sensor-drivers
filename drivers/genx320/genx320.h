/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __GENX320__H__
#define __GENX320__H__
#include "../api.h"

int genx320_set_event_format(struct psee_controls *controls, enum event_format fmt);
int genx320_start_streaming(struct psee_controls *controls);
int genx320_stop_streaming(struct psee_controls *controls);
int genx320_get_height(struct psee_controls *controls, u32 *height);
int genx320_get_width(struct psee_controls *controls, u32 *width);

#endif /* __PSEE_CONTROLS_H */


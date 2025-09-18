/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __DRIVERS_API_H
#define __DRIVERS_API_H
#include <linux/types.h>

enum RoiMode {
	ROI = 0,
	RONI,
};

struct roi_window_config {
	u32 roi_count;
	enum RoiMode mode;

};

struct erc_config {
	u32 td_target_event_count;
	u32 reference_period;
};

struct roi {
	u32 x;
	u32 y;
	u32 width;
	u32 height;
};

struct roi_set {
	u32 n;
	struct roi rois[18];
};

struct bias_config {
};

enum sensor_if {
	SENSOR_IF_MIPI = 0,
	SENSOR_IF_PARALLEL,
	SENSOR_IF_INVALID,
};

enum sensor_source {
	SENSOR_SOURCE_PIXEL_ARRAY = 0,
	SENSOR_SOURCE_RO_PATTERN,
	SENSOR_SOURCE_IF_PATTERN, // CPI/MIPI patterns
	SENSOR_SOURCE_TS_PATTERN,
};

enum event_format {
	EVENT_FORMAT_EVT2 = 0,
	EVENT_FORMAT_EVT3,
	EVENT_FORMAT_EVT21,
	EVENT_FORMAT_INVALID,
};

enum sync_mode {
	SYNC_MODE_STANDALONE = 0,
	SYNC_MODE_MASTER = 1,
	SYNC_MODE_SLAVE = 2,
};

struct core_config {
	enum sensor_source source;
	enum sensor_if sensor_if;
	enum event_format format;
};

typedef int (*psee_write_reg)(void *hdl, u32 reg, const u32 val);
typedef int (*psee_read_reg)(void *hdl, u32 reg, u32 *val);

struct psee_ctrl_ops {
	psee_write_reg write_reg;
	psee_read_reg read_reg;
	void *hdl;
};

enum mipi_frame_format {
	VARIABLE_SIZE,
	VARIABLE_CONTENT_SIZE,
	VARIABLE_TIMING,
};


struct mipi_config {
	u32 num_lanes;
	u32 bit_rate;
	bool stats_en;
	bool eof_marker;
	enum mipi_frame_format format;
};

struct row {
	bool dirty;
	// TODO: convert to flexible array
	u32 vectors[10];
};

struct grid {
	u32 width;
	u32 height;
	struct row rows[320];
};

struct roi_pixel_config {
	struct grid grid;
};

struct psee_controls {
	// driver configurations
	struct bias_config bias; // Bias
	struct roi_window_config roi_window;   // ROI/CROP
	struct erc_config erc;   // Event Rate Controller
	struct core_config core;
	struct mipi_config mipi;
	struct roi_pixel_config roi_pixel;

	// hardware control
	struct psee_ctrl_ops dev_ctrl;
};
#endif // __DRIVERS_API_H__

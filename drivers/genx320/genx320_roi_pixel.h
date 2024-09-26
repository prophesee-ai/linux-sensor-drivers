/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __GENX320_ROI_LATCH_H__
#define __GENX320_ROI_LATCH_H__
#include "../api.h"

/// @brief Initializer the ROI pixel driver
///
/// @param config the ROI pixel driver main configuration
/// @return 0 on success
int genx320_roi_pixel_init(struct psee_controls *controls);

/// @brief Reset the ROI configuration
/// @note this function will enable the entire pixel array and apply.
///
/// @param config the ROI pixel driver main configuration
/// @return 0 on success
int genx320_roi_pixel_reset(struct psee_controls *controls);

/// @brief Apply current pixel grid
///
/// @param config the ROI pixel driver main configuration
/// @return 0 on success
int genx320_roi_pixel_set_array(struct psee_controls *controls, struct grid *grid);

/// @brief Read current pixel grid
///
/// @param config the ROI pixel driver main configuration
/// @return 0 on success
int genx320_roi_pixel_get_array(struct psee_controls *controls, struct grid *grid);

/// @brief Get the state of a single pixel from the pixel grid
///
/// @param config the ROI pixel driver main configuration
/// @return 0 on success
int genx320_roi_pixel_get_pixel(struct psee_controls *controls, u32 x, u32 y, bool *enabled);

/// @brief Set the state of a single pixel from the pixel grid
///
/// @param config the ROI pixel driver main configuration
/// @return 0 on success
int genx320_roi_pixel_set_pixel(struct psee_controls *controls, u32 x, u32 y, bool enable);

/// @brief Update the underlying grid with roi_window rois
///
/// @param config the ROI pixel driver main configuration
/// @param rois the ROIs to apply to the grid
/// @return 0 on success
int genx320_roi_pixel_update_windows(struct psee_controls *controls, struct roi *rois, u32 n);

#endif // __GENX320_ROI_H__

/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __GENX320_ROI_H__
#define __GENX320_ROI_H__
#include "../api.h"

/// @brief Initializer the ROI window driver
///
/// @param config the ROI window driver main configuration
/// @return 0 on success
int genx320_roi_window_init(struct psee_controls *controls);

/// @brief Append a single window
///
/// The window will be applied according to the current mode (ROI or RONI)
/// In ROI mode, enabled pixels are those inside the provided rectangles.
/// In RONI mode, enabled pixels are those where row OR column are covered by the provided
/// rectangle.
///
/// @param config the ROI window driver main configuration
/// @param roi The winodow coordinates
/// @param single The configuration will be applied to the driver if single is true,
/// @return 0 on success
int genx320_roi_window_append(struct psee_controls *controls, struct roi roi, bool single);

/// @brief Updates a single window
///
/// The window will be applied according to the current mode (ROI or RONI)
/// In ROI mode, enabled pixels are those inside the provided rectangles.
/// In RONI mode, enabled pixels are those where row OR column are covered by the provided
/// rectangle.
///
/// @param config the ROI window driver main configuration
/// @param roi The winodow coordinates
/// @param index The winodow index
/// @return 0 on success
int genx320_roi_window_update(struct psee_controls *controls, struct roi roi, u32 index);

/// @brief Sets multiple windows
///
/// The windows will be applied according to the current mode (ROI or RONI)
/// In ROI mode, enabled pixels are those inside the provided rectangles.
/// In RONI mode, enabled pixels are those where row OR column are covered by the provided
/// rectangle.
///
/// @param config the ROI window driver main configuration
/// @param rois An array of windows to set
/// @return 0 on success
int genx320_roi_window_set(struct psee_controls *controls, struct roi *rois, u32 n);

/// @brief Reset the ROI configuration
/// @note this function with configure a single full resolution ROI and set n to 0 any
/// previous configuraiton will be lost
///
/// @param config the ROI window driver main configuration
/// @return 0 on success
int genx320_roi_window_reset(struct psee_controls *controls);

/// @brief Enable RONI (Region Of No Interest)
///
/// @param config the ROI window driver main configuration
/// @param en If set to true RONI mode will be enable, otherwise ROI mode will be enabled
/// @note this applies globaly to all confitured windows
/// @return 0 on success
int genx320_roi_window_enable_roni(struct psee_controls *controls, bool en);
#endif // __GENX320_ROI_H__

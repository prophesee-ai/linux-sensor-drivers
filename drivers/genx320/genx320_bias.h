/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __GENX320_BIAS_H__
#define __GENX320_BIAS_H__
#include "../api.h"

#define BIAS_NAME_MAX_SIZE 32

/// @brief Initializer the bias driver
///
/// @param config the bias driver main configuration
/// @return 0 on success
int genx320_bias_init(struct psee_controls *controls);

/// @brief Set the value for a specific bias
///
/// @param config the bias driver main configuration
/// @param id ID of the bias to query
/// @param value Value to set
/// @return 0 on success
int genx320_bias_set(struct psee_controls *controls, u32 id, u8 value);

/// @brief Get the current value for a specific bias
///
/// @param config the bias driver main configuration
/// @param id ID of the bias to query
/// @return the bias current value
int genx320_bias_get(struct psee_controls *controls, u32 id);

/// @brief Get the maximum allowed value for a specific bias
///
/// @param config the bias driver main configuration
/// @param id ID of the bias to query
/// @return the bias maximum allowed value
int genx320_bias_get_max(struct psee_controls *controls, u32 id);

/// @brief Get the minimum allowed value for a specific bias
///
/// @param config the bias driver main configuration
/// @param id ID of the bias to query
/// @return the bias minimum allowed value
int genx320_bias_get_min(struct psee_controls *controls, u32 id);

/// @brief Get the default value for a specific bias
///
/// @param config the bias driver main configuration
/// @param id ID of the bias to query
/// @return the bias default value
int genx320_bias_get_default(struct psee_controls *controls, u32 id);

/// @brief Get the bias name
///
/// @param config the bias driver main configuration
/// @param id ID of the bias to query
/// @param name a pointer to a string pointer that will be set to the bias name
/// @return 0 on success
int genx320_bias_get_name(struct psee_controls *controls, u32 id, char **name);

#endif // __GENX320_BIAS_H__

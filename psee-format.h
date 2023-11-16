/* SPDX-License-Identifier: GPL-2.0-only */

#include <linux/videodev2.h>
#include <linux/media-bus-format.h>

/* Define pixel and media types if the definition does not exist in this kernel
 * This allows kernel module to build, but does not include the definition in the UAPI,
 * thus core kernel code and userspace may complain about unknown pixel/media types
 *
 * Media types values are chosen to be in the Vendor specific formats,
 * far enough from currently defines formats to avoid collisions.
 */
#ifndef V4L2_PIX_FMT_PSEE_EVT2
#define V4L2_PIX_FMT_PSEE_EVT2 v4l2_fourcc('P', 'S', 'E', 'E')
#define MEDIA_BUS_FMT_PSEE_EVT2 0x5300
#endif

#ifndef V4L2_PIX_FMT_PSEE_EVT21ME
#define V4L2_PIX_FMT_PSEE_EVT21ME v4l2_fourcc('P', 'S', 'E', '1')
#define MEDIA_BUS_FMT_PSEE_EVT21ME 0x5301
#endif

#ifndef V4L2_PIX_FMT_PSEE_EVT21
#define V4L2_PIX_FMT_PSEE_EVT21 v4l2_fourcc('P', 'S', 'E', '2')
#define MEDIA_BUS_FMT_PSEE_EVT21 0x5303
#endif

#ifndef V4L2_PIX_FMT_PSEE_EVT3
#define V4L2_PIX_FMT_PSEE_EVT3 v4l2_fourcc('P', 'S', 'E', '3')
#define MEDIA_BUS_FMT_PSEE_EVT3 0x5302
#endif

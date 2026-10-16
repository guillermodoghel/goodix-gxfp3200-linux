/*
 * Correlation matcher for the Goodix Milan F SPI sensor (108x88 pixels).
 *
 * The sensor area is too small for minutiae matching. Instead, each image is
 * contrast-normalized and compared against every enrolled image with a masked
 * normalized cross-correlation (Padfield, 2012) over a small range of
 * rotations and all translations, computed with FFTs.
 *
 * Copyright (C) 2026 guillermodoghel
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#pragma once

#include <glib.h>

#define GXM_SENSOR_W 108
#define GXM_SENSOR_H 88
#define GXM_SENSOR_PIXELS (GXM_SENSOR_W * GXM_SENSOR_H)

typedef struct _GxmImage GxmImage;
typedef struct _GxmProbe GxmProbe;

/* @diff: background minus frame, GXM_SENSOR_PIXELS values */
GxmImage *gxm_image_new (const gint16 *diff);
void      gxm_image_free (GxmImage *image);
/* fraction of the image where the finger made usable contact */
double    gxm_image_coverage (const GxmImage *image);

GxmProbe *gxm_probe_new (const GxmImage *image);
void      gxm_probe_free (GxmProbe *probe);
/* best correlation (0..1) of the probe against one enrolled image */
double    gxm_probe_score (GxmProbe *probe, const GxmImage *reference);
/* Best correlation against several enrolled images, using a few threads and
 * stopping early once one reaches @threshold. @first is tried first. */
double    gxm_probe_search (GxmProbe        *probe,
                            GxmImage *const *refs,
                            guint            n_refs,
                            guint            first,
                            double           threshold,
                            guint           *best_idx);

G_DEFINE_AUTOPTR_CLEANUP_FUNC (GxmImage, gxm_image_free);
G_DEFINE_AUTOPTR_CLEANUP_FUNC (GxmProbe, gxm_probe_free);

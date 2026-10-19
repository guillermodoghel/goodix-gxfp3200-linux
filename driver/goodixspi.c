/*
 * Goodix "Milan F" SPI fingerprint sensor driver (GXFP3200, chip id 0x2202xx)
 *
 * Copyright (C) 2026 guillermodoghel
 *
 * The register protocol was worked out from the vendor's Windows driver, for
 * interoperability. Register access:
 *   write: F0 <reg_hi> <reg_lo> 00 01 <val_hi> <val_lo>
 *   read:  F0 <reg_hi> <reg_lo>   then   F1 + rx(n)
 *
 * The sensor delivers a raw 108x88 12-bit image. It is too small for minutiae
 * matching, so enrollment stores raw images and matching is done on the host
 * with an image correlation matcher (goodixspi-match.c).
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#define FP_COMPONENT "goodixspi"

#include "drivers_api.h"
#include "fpi-spi-transfer.h"
#include "goodixspi-match.h"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define GX_CHIP_MILANF      0x2202
#define GX_IMG_W            GXM_SENSOR_W
#define GX_IMG_H            GXM_SENSOR_H
#define GX_IMG_PIXELS       GXM_SENSOR_PIXELS
#define GX_FRAME_LEN        0x37b4      /* 12-bit packed pixels + 4 bytes CRC32 */
/* Register access always uses the ACPI speed (4 MHz). The frame read is
 * protected by a CRC32 and works at 8 MHz on this sensor (10 MHz fails);
 * after any CRC mismatch the driver falls back to 4 MHz for the session. */
#define GX_SPEED_KHZ        4000
#define GX_FRAME_SPEED_KHZ  8000
#define GX_FRAME_DATA_LEN   (GX_FRAME_LEN - 4)
#define GX_OTP_LEN          32
#define GX_IRQ_IMAGE        0x0008
#define GX_POLL_TIMEOUT_US  (500 * 1000)
#define GX_CRC_RETRIES      3
#define GX_OTP_RETRIES      10
#define GX_CALIB_FRAMES     4
#define GX_BORDER           4           /* edge pixels ignored for finger detection */
#define GX_FRAME_DELAY_MS   8
#define GX_DEBOUNCE         1
/* Capture once the finger has settled: the contact score rose less than
 * GX_SETTLE_PCT percent (with at least GX_SETTLE_MIN frames on the sensor),
 * or after GX_SETTLE_MAX frames. */
#define GX_SETTLE_MIN       2
#define GX_SETTLE_MAX       6
#define GX_SETTLE_PCT       3

/* Finger detection: mean |frame - background| must exceed
 * max (GX_THR_FLOOR, GX_THR_NOISE_K * calibration noise), capped. */
#define GX_THR_FLOOR        60
#define GX_THR_CEIL         150
#define GX_THR_NOISE_K      8

/* Calibration needs an empty, settled sensor: frame-to-frame noise above
 * this means something (a finger) is on it, so calibration is retried. */
#define GX_CALIB_MAX_NOISE  15
#define GX_CALIB_RETRY_MS   100
#define GX_CALIB_RETRIES    50

/* While waiting for an empty sensor, a reading that stays perfectly stable
 * for this long means the background is stale: take the current one. */
#define GX_STALE_BG_US      (3 * G_USEC_PER_SEC)

/* Enrollment / matching */
#define GX_ENROLL_STAGES    30
#define GX_MIN_COVERAGE     0.45
#define GX_MATCH_THRESHOLD  0.88
#define GX_PRINT_VERSION    1

#define GX_MAX_OPS          24

typedef enum {
  GX_OP_WREG,   /* reg, val */
  GX_OP_CMD,    /* val = command byte */
  GX_OP_SLEEP,  /* val = ms */
  GX_OP_ADDR,   /* reg */
  GX_OP_READ,   /* val = length, into self->rx */
  GX_OP_SPEED,  /* val = SPI clock in kHz for the following transfers */
} GxOpType;

typedef struct
{
  GxOpType type;
  guint16  reg;
  guint16  val;
} GxOp;

typedef enum {
  GX_SCAN_WAIT_EMPTY,
  GX_SCAN_WAIT_DOWN,
} GxScanPhase;

struct _FpiDeviceGoodixSpi
{
  FpDevice      parent;

  int           spi_fd;

  /* register operation runner */
  GxOp          ops[GX_MAX_OPS];
  guint         n_ops;
  guint         op_idx;
  guint8       *rx;                     /* GX_FRAME_LEN */

  /* sensor state */
  gboolean      have_otp;
  gboolean      have_bg;
  guint8        otp[GX_OTP_LEN];
  guint         otp_idx;
  guint         otp_try;
  guint16       reg_5c;
  gint64        poll_start;
  guint         crc_try;
  guint         frame_speed_khz;

  guint16      *frame;                  /* GX_IMG_PIXELS each */
  guint16      *prev_frame;
  gint32       *bg;                     /* background, fixed point x16 */
  guint32      *calib_acc;
  guint         calib_n;
  guint         calib_retries;
  guint64       calib_noise_acc;
  guint         threshold;

  /* scan */
  GxScanPhase   phase;
  guint         debounce;
  guint         best_score;
  guint         last_score;
  guint         frames_down;
  gint16       *best_diff;              /* background minus frame */
  gint64        stable_since;

  /* enroll */
  GPtrArray    *enroll_images;          /* GBytes of gint16 diffs */

  /* enrolled image that matched last time: tried first */
  guint         last_match_idx;

  /* stored prints already prepared for matching, and the preparation
   * running in the background while waiting for the finger */
  GPtrArray    *ref_cache;              /* GxCacheEntry */
  struct _GxPrep *prep;
};

G_DECLARE_FINAL_TYPE (FpiDeviceGoodixSpi, fpi_device_goodixspi, FPI, DEVICE_GOODIXSPI, FpDevice);
G_DEFINE_TYPE (FpiDeviceGoodixSpi, fpi_device_goodixspi, FP_TYPE_DEVICE);

/* ------------------------------------------------------------------------ */
/* Register operation runner                                                */

static void
gx_ops_reset (FpiDeviceGoodixSpi *self)
{
  self->n_ops = 0;
  self->op_idx = 0;
}

static void
gx_op (FpiDeviceGoodixSpi *self, GxOpType type, guint16 reg, guint16 val)
{
  g_assert (self->n_ops < GX_MAX_OPS);
  self->ops[self->n_ops++] = (GxOp) { type, reg, val };
}

static void
gx_op_read (FpiDeviceGoodixSpi *self, guint16 reg, guint16 len)
{
  gx_op (self, GX_OP_ADDR, reg, 0);
  gx_op (self, GX_OP_READ, 0, len);
}

enum {
  GX_OPS_RUN,
  GX_OPS_NEXT,
  GX_OPS_NSTATES,
};

static void
gx_ops_handler (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceGoodixSpi *self = FPI_DEVICE_GOODIXSPI (dev);
  FpiSpiTransfer *xfer;
  const GxOp *op;

  if (fpi_ssm_get_cur_state (ssm) == GX_OPS_NEXT)
    {
      fpi_ssm_jump_to_state (ssm, GX_OPS_RUN);
      return;
    }

  if (self->op_idx >= self->n_ops)
    {
      fpi_ssm_mark_completed (ssm);
      return;
    }

  op = &self->ops[self->op_idx++];
  if (op->type == GX_OP_SLEEP)
    {
      fpi_ssm_jump_to_state_delayed (ssm, GX_OPS_RUN, op->val);
      return;
    }
  if (op->type == GX_OP_SPEED)
    {
      /* transfers are sequential, so this applies to the next ones only */
      guint32 hz = op->val * 1000;

      if (ioctl (self->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0)
        fp_warn ("could not set SPI speed to %u Hz", hz);
      fpi_ssm_jump_to_state (ssm, GX_OPS_RUN);
      return;
    }

  xfer = fpi_spi_transfer_new (dev, self->spi_fd);
  switch (op->type)
    {
    case GX_OP_WREG:
      fpi_spi_transfer_write (xfer, 7);
      xfer->buffer_wr[0] = 0xf0;
      xfer->buffer_wr[1] = op->reg >> 8;
      xfer->buffer_wr[2] = op->reg & 0xff;
      xfer->buffer_wr[3] = 0x00;
      xfer->buffer_wr[4] = 0x01;
      xfer->buffer_wr[5] = op->val >> 8;
      xfer->buffer_wr[6] = op->val & 0xff;
      break;

    case GX_OP_CMD:
      fpi_spi_transfer_write (xfer, 1);
      xfer->buffer_wr[0] = op->val;
      break;

    case GX_OP_ADDR:
      fpi_spi_transfer_write (xfer, 3);
      xfer->buffer_wr[0] = 0xf0;
      xfer->buffer_wr[1] = op->reg >> 8;
      xfer->buffer_wr[2] = op->reg & 0xff;
      break;

    case GX_OP_READ:
      g_assert (op->val <= GX_FRAME_LEN);
      fpi_spi_transfer_write (xfer, 1);
      xfer->buffer_wr[0] = 0xf1;
      fpi_spi_transfer_read_full (xfer, self->rx, op->val, NULL);
      break;

    case GX_OP_SLEEP:
    case GX_OP_SPEED:
    default:
      g_assert_not_reached ();
    }

  xfer->ssm = ssm;
  fpi_spi_transfer_submit (xfer, NULL, fpi_ssm_spi_transfer_cb, NULL);
}

/* Runs the queued ops as a child of @parent; @parent advances when done. */
static void
gx_ops_run (FpiDeviceGoodixSpi *self, FpiSsm *parent)
{
  FpiSsm *child = fpi_ssm_new (FP_DEVICE (self), gx_ops_handler, GX_OPS_NSTATES);

  fpi_ssm_silence_debug (child);
  fpi_ssm_start_subsm (parent, child);
}

static guint16
gx_rx16 (FpiDeviceGoodixSpi *self)
{
  return self->rx[0] << 8 | self->rx[1];
}

/* ------------------------------------------------------------------------ */
/* Checksums and image helpers                                              */

/* CRC-8, poly 0x07, init 0, xorout 0xff */
static guint8
gx_crc8 (const guint8 *data, gsize len)
{
  guint8 c = 0;

  for (gsize i = 0; i < len; i++)
    {
      c ^= data[i];
      for (int b = 0; b < 8; b++)
        c = (c & 0x80) ? (c << 1) ^ 0x07 : c << 1;
    }
  return c ^ 0xff;
}

/* CRC-32/MPEG-2: poly 0x04c11db7, init 0xffffffff, no reflection/xorout */
static guint32
gx_crc32 (const guint8 *data, gsize len)
{
  guint32 c = 0xffffffff;

  for (gsize i = 0; i < len; i++)
    {
      c ^= (guint32) data[i] << 24;
      for (int b = 0; b < 8; b++)
        c = (c & 0x80000000) ? (c << 1) ^ 0x04c11db7 : c << 1;
    }
  return c;
}

/* Port of Milan_OTPCRCCheck for chip type 0 */
static gboolean
gx_otp_valid (const guint8 *otp)
{
  static const guint8 blank_idx[] = { 8, 9, 10, 11, 12, 13, 14, 15, 0x10, 0x11, 0x1e };
  guint8 buf[25];
  guint8 sum = 0;
  gboolean blank = TRUE;

  for (gsize i = 0; i < G_N_ELEMENTS (blank_idx); i++)
    blank &= otp[blank_idx[i]] == 0;
  if (blank)
    return TRUE;

  for (int i = 0; i < 0x14; i++)
    sum += otp[i];
  sum += otp[0x1d] + otp[0x1f];
  if (sum == otp[0x1e])
    return TRUE;

  memcpy (buf, otp, 0x14);
  buf[20] = otp[0x1d];
  buf[21] = otp[0x1f];
  if (gx_crc8 (buf, 22) == otp[0x1e])
    return TRUE;

  if (otp[0x1c] == 0xc0)
    {
      buf[20] = otp[0x1c];
      buf[21] = otp[0x1d];
      buf[22] = otp[0x1f];
      return gx_crc8 (buf, 23) == otp[0x1e];
    }

  buf[20] = otp[0x1a];
  buf[21] = otp[0x1b];
  buf[22] = otp[0x1c];
  buf[23] = otp[0x1d];
  buf[24] = otp[0x1f];
  return gx_crc8 (buf, 25) == otp[0x1e];
}

/* 6 bytes -> 4 pixels of 12 bits */
static void
gx_unpack (const guint8 *raw, guint16 *px)
{
  for (int i = 0, o = 0; o < GX_IMG_PIXELS; i += 6, o += 4)
    {
      px[o + 0] = (raw[i + 0] & 0x0f) << 8 | raw[i + 1];
      px[o + 1] = raw[i + 3] << 4 | raw[i + 0] >> 4;
      px[o + 2] = (raw[i + 5] & 0x0f) << 8 | raw[i + 2];
      px[o + 3] = raw[i + 4] << 4 | raw[i + 5] >> 4;
    }
}

/* mean |a - ref| over the sensor interior; ref is bg16 (x16) or b */
static guint
gx_mean_absdiff (const guint16 *a, const gint32 *bg16, const guint16 *b)
{
  guint64 acc = 0;
  guint n = 0;

  for (int y = GX_BORDER; y < GX_IMG_H - GX_BORDER; y++)
    for (int x = GX_BORDER; x < GX_IMG_W - GX_BORDER; x++)
      {
        int i = y * GX_IMG_W + x;
        int ref = bg16 ? bg16[i] / 16 : b[i];
        acc += ABS ((int) a[i] - ref);
        n++;
      }
  return acc / n;
}

/* mean (background - frame) over the interior: a finger lowers the reading,
 * so a clearly negative value means the background still contains a finger */
static int
gx_mean_signed (const guint16 *a, const gint32 *bg16)
{
  gint64 acc = 0;
  guint n = 0;

  for (int y = GX_BORDER; y < GX_IMG_H - GX_BORDER; y++)
    for (int x = GX_BORDER; x < GX_IMG_W - GX_BORDER; x++)
      {
        int i = y * GX_IMG_W + x;
        acc += bg16[i] / 16 - (int) a[i];
        n++;
      }
  return acc / (gint64) n;
}

static void
gx_dump_diff (const gint16 *diff)
{
  const char *dir = g_getenv ("GOODIXSPI_DUMP_DIR");
  g_autofree char *path = NULL;
  g_autofree char *header = NULL;
  g_autoptr(GByteArray) out = NULL;
  static guint counter = 0;
  int lo = G_MAXINT, hi = G_MININT;

  if (!dir)
    return;

  for (int i = 0; i < GX_IMG_PIXELS; i++)
    {
      lo = MIN (lo, diff[i]);
      hi = MAX (hi, diff[i]);
    }
  path = g_strdup_printf ("%s/goodixspi-%04u.pgm", dir, counter++);
  header = g_strdup_printf ("P5\n%d %d\n255\n", GX_IMG_W, GX_IMG_H);
  out = g_byte_array_new ();
  g_byte_array_append (out, (guint8 *) header, strlen (header));
  for (int i = 0; i < GX_IMG_PIXELS; i++)
    {
      guint8 v = (diff[i] - lo) * 255 / MAX (hi - lo, 1);
      g_byte_array_append (out, &v, 1);
    }
  if (!g_file_set_contents (path, (char *) out->data, out->len, NULL))
    fp_warn ("could not write %s", path);
}

static gboolean
gx_check_cancelled (FpDevice *dev, FpiSsm *ssm)
{
  GError *error = NULL;

  if (g_cancellable_set_error_if_cancelled (fpi_device_get_cancellable (dev), &error))
    {
      fpi_ssm_mark_failed (ssm, error);
      return TRUE;
    }
  return FALSE;
}

/* ------------------------------------------------------------------------ */
/* Single frame capture                                                     */

enum {
  GX_CAP_PREPARE,
  GX_CAP_POLL,
  GX_CAP_POLL_CHECK,
  GX_CAP_FRAME_CHECK,
  GX_CAP_NSTATES,
};

static void
gx_capture_handler (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceGoodixSpi *self = FPI_DEVICE_GOODIXSPI (dev);
  guint16 status;
  guint32 crc_chip;
  const guint8 *crc_raw;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case GX_CAP_PREPARE:
      /* Milan_ReadImagePrepare (Milan F) */
      gx_ops_reset (self);
      gx_op (self, GX_OP_CMD, 0, 0xc0);
      gx_op (self, GX_OP_SLEEP, 0, 1);
      gx_op (self, GX_OP_WREG, 0x124, 0x00ff);
      /* Milan_DlCfg (5) */
      gx_op (self, GX_OP_WREG, 0x05c, 0x0080);
      gx_op (self, GX_OP_WREG, 0x120, 0x0408);
      gx_op (self, GX_OP_WREG, 0x052, 0x0008);
      gx_op (self, GX_OP_WREG, 0x050, 0x0501);
      if (self->reg_5c)
        gx_op (self, GX_OP_WREG, 0x05c, self->reg_5c);
      gx_op (self, GX_OP_CMD, 0, 0xc2);
      gx_op (self, GX_OP_SLEEP, 0, 1);
      gx_op (self, GX_OP_WREG, 0x054, 0x0110);
      self->poll_start = g_get_monotonic_time ();
      gx_ops_run (self, ssm);
      break;

    case GX_CAP_POLL:
      gx_ops_reset (self);
      gx_op_read (self, 0x126, 2);
      gx_ops_run (self, ssm);
      break;

    case GX_CAP_POLL_CHECK:
      status = gx_rx16 (self);
      if ((status & 0x1ff) != GX_IRQ_IMAGE)
        {
          if (g_get_monotonic_time () - self->poll_start > GX_POLL_TIMEOUT_US)
            {
              fpi_ssm_mark_failed (ssm,
                                   fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                             "timeout waiting for image, status 0x%04x",
                                                             status));
              return;
            }
          fpi_ssm_jump_to_state_delayed (ssm, GX_CAP_POLL, 1);
          return;
        }
      /* ack like GxF_InterruptIrq, then Milan_ReadImageItem */
      gx_ops_reset (self);
      gx_op (self, GX_OP_WREG, 0x124, status);
      gx_op (self, GX_OP_ADDR, 0xaaaa, 0);
      gx_op (self, GX_OP_SPEED, 0, self->frame_speed_khz);
      gx_op (self, GX_OP_READ, 0, GX_FRAME_LEN);
      gx_op (self, GX_OP_SPEED, 0, GX_SPEED_KHZ);
      gx_ops_run (self, ssm);
      break;

    case GX_CAP_FRAME_CHECK:
      crc_raw = self->rx + GX_FRAME_DATA_LEN;
      crc_chip = (guint32) crc_raw[2] << 24 | (guint32) crc_raw[3] << 16 |
                 (guint32) crc_raw[0] << 8 | crc_raw[1];
      if (crc_chip != gx_crc32 (self->rx, GX_FRAME_DATA_LEN))
        {
          if (self->frame_speed_khz != GX_SPEED_KHZ)
            {
              fp_info ("frame CRC mismatch at %u kHz, falling back to %u kHz",
                       self->frame_speed_khz, GX_SPEED_KHZ);
              self->frame_speed_khz = GX_SPEED_KHZ;
            }
          if (++self->crc_try < GX_CRC_RETRIES)
            {
              fp_dbg ("frame CRC mismatch, retrying");
              fpi_ssm_jump_to_state (ssm, GX_CAP_PREPARE);
              return;
            }
          fpi_ssm_mark_failed (ssm,
                               fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                         "frame CRC mismatch"));
          return;
        }
      self->crc_try = 0;
      gx_unpack (self->rx, self->frame);
      fpi_ssm_mark_completed (ssm);
      break;
    }
}

static void
gx_capture_start (FpiDeviceGoodixSpi *self, FpiSsm *parent)
{
  FpiSsm *child = fpi_ssm_new (FP_DEVICE (self), gx_capture_handler, GX_CAP_NSTATES);

  self->crc_try = 0;
  fpi_ssm_silence_debug (child);
  fpi_ssm_start_subsm (parent, child);
}

/* ------------------------------------------------------------------------ */
/* Open: identify, read OTP, enable, calibrate background                   */

enum {
  GX_INIT_CHIPID,
  GX_INIT_CHIPID_CHECK,
  GX_INIT_OTP_BYTE,
  GX_INIT_OTP_STORE,
  GX_INIT_ENABLE,
  GX_INIT_CALIB_CAPTURE,
  GX_INIT_CALIB_STORE,
  GX_INIT_QUICK_CHECK,
  GX_INIT_NSTATES,
};

static void
gx_calib_retry (FpiDeviceGoodixSpi *self, FpiSsm *ssm, const char *why)
{
  self->calib_n = 0;
  self->calib_noise_acc = 0;

  if (++self->calib_retries < GX_CALIB_RETRIES)
    {
      fp_dbg ("calibration: %s, retrying", why);
      fpi_ssm_jump_to_state_delayed (ssm, GX_INIT_CALIB_CAPTURE, GX_CALIB_RETRY_MS);
      return;
    }
  fpi_ssm_mark_failed (ssm, fpi_device_retry_new_msg (FP_DEVICE_RETRY_REMOVE_FINGER,
                                                      "remove the finger to calibrate"));
}

static void
gx_init_handler (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceGoodixSpi *self = FPI_DEVICE_GOODIXSPI (dev);
  guint32 chip_id;
  guint noise;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case GX_INIT_CHIPID:
      gx_ops_reset (self);
      gx_op_read (self, 0x0000, 4);
      gx_ops_run (self, ssm);
      break;

    case GX_INIT_CHIPID_CHECK:
      chip_id = (guint32) self->rx[2] << 24 | (guint32) self->rx[3] << 16 |
                (guint32) self->rx[0] << 8 | self->rx[1];
      fp_dbg ("chip id 0x%08x", chip_id);
      if ((chip_id >> 8) != GX_CHIP_MILANF)
        {
          fpi_ssm_mark_failed (ssm,
                               fpi_device_error_new_msg (FP_DEVICE_ERROR_NOT_SUPPORTED,
                                                         "unsupported chip id 0x%08x", chip_id));
          return;
        }
      self->otp_idx = 0;
      self->otp_try = 0;
      /* the OTP does not change: read it only on the first open */
      if (self->have_otp)
        fpi_ssm_jump_to_state (ssm, GX_INIT_ENABLE);
      else
        fpi_ssm_next_state (ssm);
      break;

    case GX_INIT_OTP_BYTE:
      /* OTP byte i, read through the 0x102 (control) / 0x104 (address) /
       * 0x106 (data) window, same sequence as the vendor driver */
      gx_ops_reset (self);
      gx_op (self, GX_OP_WREG, 0x102, 0x0020);
      gx_op (self, GX_OP_WREG, 0x104, self->otp_idx >> 2);
      gx_op (self, GX_OP_WREG, 0x102, 0x0021);
      gx_op (self, GX_OP_WREG, 0x102, 0x0020);
      gx_op (self, GX_OP_WREG, 0x104, (self->otp_idx & 3) << 8);
      gx_op (self, GX_OP_WREG, 0x102, 0x0022);
      gx_op (self, GX_OP_WREG, 0x102, 0x0020);
      gx_op_read (self, 0x106, 2);
      gx_ops_run (self, ssm);
      break;

    case GX_INIT_OTP_STORE:
      self->otp[self->otp_idx++] = gx_rx16 (self) & 0xff;
      if (self->otp_idx < GX_OTP_LEN)
        {
          fpi_ssm_jump_to_state (ssm, GX_INIT_OTP_BYTE);
          return;
        }
      if (!gx_otp_valid (self->otp))
        {
          if (++self->otp_try < GX_OTP_RETRIES)
            {
              self->otp_idx = 0;
              fpi_ssm_jump_to_state_delayed (ssm, GX_INIT_OTP_BYTE, 5);
              return;
            }
          fpi_ssm_mark_failed (ssm,
                               fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                         "OTP checksum mismatch"));
          return;
        }
      /* Milan_CheckSensor: value for register 0x5c derived from the OTP */
      if (self->otp[0x16] != 0 && (guint8) (self->otp[0x16] + self->otp[0x17]) == 0xff)
        self->reg_5c = ((self->otp[0x16] >> 4) + 1) * 0x10;
      else
        self->reg_5c = 0;
      fp_dbg ("OTP ok, reg 0x5c = 0x%04x", self->reg_5c);
      self->have_otp = TRUE;
      fpi_ssm_next_state (ssm);
      break;

    case GX_INIT_ENABLE:
      /* device_enable, without the GPIO reset pulse */
      gx_ops_reset (self);
      gx_op (self, GX_OP_WREG, 0x124, 0x0d00);
      gx_op (self, GX_OP_WREG, 0x204, 0x0000);
      gx_op (self, GX_OP_CMD, 0, 0xc0);
      gx_op (self, GX_OP_SLEEP, 0, 10);
      self->calib_n = 0;
      self->calib_noise_acc = 0;
      self->calib_retries = 0;
      gx_ops_run (self, ssm);
      break;

    case GX_INIT_CALIB_CAPTURE:
      gx_capture_start (self, ssm);
      break;

    case GX_INIT_CALIB_STORE:
      /* Reopening with a known background: never recalibrate with a finger
       * on the sensor. One frame: if it matches the background, refresh it
       * slightly; otherwise keep it (the scan waits for an empty sensor). */
      if (self->have_bg)
        {
          fpi_ssm_jump_to_state (ssm, GX_INIT_QUICK_CHECK);
          return;
        }
      if (self->calib_n == 0)
        {
          for (int i = 0; i < GX_IMG_PIXELS; i++)
            self->calib_acc[i] = self->frame[i];
        }
      else
        {
          noise = gx_mean_absdiff (self->frame, NULL, self->prev_frame);
          if (noise > GX_CALIB_MAX_NOISE)
            {
              gx_calib_retry (self, ssm, "sensor not settled");
              return;
            }
          self->calib_noise_acc += noise;
          for (int i = 0; i < GX_IMG_PIXELS; i++)
            self->calib_acc[i] += self->frame[i];
        }
      memcpy (self->prev_frame, self->frame, GX_IMG_PIXELS * sizeof (guint16));

      if (++self->calib_n < GX_CALIB_FRAMES)
        {
          fpi_ssm_jump_to_state (ssm, GX_INIT_CALIB_CAPTURE);
          return;
        }
      for (int i = 0; i < GX_IMG_PIXELS; i++)
        self->bg[i] = self->calib_acc[i] * 16 / GX_CALIB_FRAMES;
      noise = self->calib_noise_acc / (GX_CALIB_FRAMES - 1);
      self->threshold = CLAMP (GX_THR_NOISE_K * noise, GX_THR_FLOOR, GX_THR_CEIL);
      fp_dbg ("calibrated: noise %u, finger threshold %u", noise, self->threshold);
      self->have_bg = TRUE;
      fpi_ssm_mark_completed (ssm);
      break;

    case GX_INIT_QUICK_CHECK:
      noise = gx_mean_absdiff (self->frame, self->bg, NULL);
      if (noise < self->threshold / 2)
        {
          for (int i = 0; i < GX_IMG_PIXELS; i++)
            self->bg[i] += (self->frame[i] * 16 - self->bg[i]) / 4;
          fp_dbg ("reopen: background ok (score %u)", noise);
        }
      else
        {
          fp_dbg ("reopen: sensor busy (score %u), keeping background", noise);
        }
      fpi_ssm_mark_completed (ssm);
      break;
    }
}

/* ------------------------------------------------------------------------ */
/* Scan: wait for an empty sensor, then for a finger; keep the best frame   */

enum {
  GX_SCAN_CAPTURE,
  GX_SCAN_PROCESS,
  GX_SCAN_NSTATES,
};

static void
gx_scan_handler (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceGoodixSpi *self = FPI_DEVICE_GOODIXSPI (dev);
  guint score, noise;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case GX_SCAN_CAPTURE:
      if (gx_check_cancelled (dev, ssm))
        return;
      memcpy (self->prev_frame, self->frame, GX_IMG_PIXELS * sizeof (guint16));
      gx_capture_start (self, ssm);
      break;

    case GX_SCAN_PROCESS:
      score = gx_mean_absdiff (self->frame, self->bg, NULL);

      if (self->phase == GX_SCAN_WAIT_EMPTY)
        {
          if (score < self->threshold)
            {
              self->stable_since = 0;
              if (++self->debounce >= GX_DEBOUNCE)
                {
                  fp_dbg ("sensor empty (score %u), waiting for finger", score);
                  fpi_device_report_finger_status_changes (dev, FP_FINGER_STATUS_NEEDED,
                                                           FP_FINGER_STATUS_PRESENT);
                  self->phase = GX_SCAN_WAIT_DOWN;
                  self->debounce = 0;
                }
            }
          else
            {
              /* a stale background (e.g. drift) would wait forever: if the
               * reading is perfectly still for a while, adopt it */
              noise = gx_mean_absdiff (self->frame, NULL, self->prev_frame);
              self->debounce = 0;
              if (noise > GX_CALIB_MAX_NOISE)
                self->stable_since = 0;
              else if (self->stable_since == 0)
                self->stable_since = g_get_monotonic_time ();
              else if (g_get_monotonic_time () - self->stable_since > GX_STALE_BG_US)
                {
                  fp_dbg ("background stale (score %u), resetting", score);
                  for (int i = 0; i < GX_IMG_PIXELS; i++)
                    self->bg[i] = self->frame[i] * 16;
                  self->stable_since = 0;
                }
            }
        }
      else
        {
          if (score >= self->threshold && gx_mean_signed (self->frame, self->bg) < 0)
            {
              /* the reading went up, not down: the background had a finger
               * in it (calibrated while touching). Take the empty sensor now. */
              fp_dbg ("background contained a finger (score %u), resetting", score);
              for (int i = 0; i < GX_IMG_PIXELS; i++)
                self->bg[i] = self->frame[i] * 16;
              self->frames_down = 0;
              self->best_score = 0;
            }
          else if (score >= self->threshold)
            {
              gboolean rising = score * 100 > self->last_score * (100 + GX_SETTLE_PCT);

              self->frames_down++;
              self->last_score = score;
              if (score > self->best_score)
                {
                  self->best_score = score;
                  for (int i = 0; i < GX_IMG_PIXELS; i++)
                    self->best_diff[i] = self->bg[i] / 16 - (int) self->frame[i];
                }
              if ((!rising && self->frames_down >= GX_SETTLE_MIN) ||
                  self->frames_down >= GX_SETTLE_MAX)
                {
                  fp_dbg ("finger captured (score %u, %u frames)", self->best_score,
                          self->frames_down);
                  fpi_device_report_finger_status_changes (dev, FP_FINGER_STATUS_PRESENT,
                                                           FP_FINGER_STATUS_NEEDED);
                  gx_dump_diff (self->best_diff);
                  fpi_ssm_mark_completed (ssm);
                  return;
                }
            }
          else
            {
              self->frames_down = 0;
              self->best_score = 0;
              self->last_score = 0;
              /* follow slow background drift while the sensor is empty */
              if (score < self->threshold / 2)
                for (int i = 0; i < GX_IMG_PIXELS; i++)
                  self->bg[i] += (self->frame[i] * 16 - self->bg[i]) / 16;
            }
        }
      fpi_ssm_jump_to_state_delayed (ssm, GX_SCAN_CAPTURE, GX_FRAME_DELAY_MS);
      break;
    }
}

/* Starts a scan; @done is called with self->best_diff filled in. */
static void
gx_scan_start (FpiDeviceGoodixSpi *self, FpiSsmCompletedCallback done)
{
  FpiSsm *ssm = fpi_ssm_new (FP_DEVICE (self), gx_scan_handler, GX_SCAN_NSTATES);

  self->phase = GX_SCAN_WAIT_EMPTY;
  self->debounce = 0;
  self->best_score = 0;
  self->last_score = 0;
  self->frames_down = 0;
  self->stable_since = 0;
  fpi_ssm_silence_debug (ssm);
  fpi_ssm_start (ssm, done);
}

/* ------------------------------------------------------------------------ */
/* Print data: (u aay) = version, list of raw diffs (gint16, little endian) */

static GVariant *
gx_print_data_new (GPtrArray *images)
{
  GVariantBuilder b;

  g_variant_builder_init (&b, G_VARIANT_TYPE ("aay"));
  for (guint i = 0; i < images->len; i++)
    {
      GBytes *img = g_ptr_array_index (images, i);
      g_variant_builder_add_value (&b, g_variant_new_from_bytes (G_VARIANT_TYPE ("ay"), img, TRUE));
    }
  return g_variant_new ("(u@aay)", GX_PRINT_VERSION, g_variant_builder_end (&b));
}

/* Prepared matcher images of stored print data, or NULL if the data is invalid */
static GPtrArray *
gx_print_data_load (GVariant *data)
{
  g_autoptr(GVariant) list = NULL;
  GPtrArray *out;
  guint32 version;

  if (!data || !g_variant_is_of_type (data, G_VARIANT_TYPE ("(uaay)")))
    return NULL;
  g_variant_get (data, "(u@aay)", &version, &list);
  if (version != GX_PRINT_VERSION || g_variant_n_children (list) == 0)
    return NULL;

  out = g_ptr_array_new_with_free_func ((GDestroyNotify) gxm_image_free);
  for (gsize i = 0; i < g_variant_n_children (list); i++)
    {
      g_autoptr(GVariant) child = g_variant_get_child_value (list, i);
      gsize len;
      const guint8 *raw = g_variant_get_fixed_array (child, &len, 1);
      gint16 diff[GX_IMG_PIXELS];

      if (len != sizeof (diff))
        {
          g_ptr_array_free (out, TRUE);
          return NULL;
        }
      for (int k = 0; k < GX_IMG_PIXELS; k++)
        diff[k] = (gint16) (raw[2 * k] | raw[2 * k + 1] << 8);
      g_ptr_array_add (out, gxm_image_new (diff));
    }
  return out;
}

static GBytes *
gx_diff_to_bytes (const gint16 *diff)
{
  guint8 *raw = g_malloc (GX_IMG_PIXELS * 2);

  for (int k = 0; k < GX_IMG_PIXELS; k++)
    {
      raw[2 * k] = diff[k] & 0xff;
      raw[2 * k + 1] = (diff[k] >> 8) & 0xff;
    }
  return g_bytes_new_take (raw, GX_IMG_PIXELS * 2);
}

/* ------------------------------------------------------------------------ */
/* Template preparation (background, while waiting for the finger) + cache  */

#define GX_CACHE_MAX 10

typedef struct
{
  GVariant  *data;
  GPtrArray *refs;                      /* GxmImage */
} GxCacheEntry;

static void
gx_cache_entry_free (GxCacheEntry *e)
{
  g_variant_unref (e->data);
  g_ptr_array_unref (e->refs);
  g_free (e);
}

typedef struct _GxPrep
{
  GPtrArray *prints;                    /* FpPrint */
  GPtrArray *datas;                     /* GVariant, one per print (may be NULL) */
  GPtrArray *refs;                      /* GPtrArray of GxmImage, or NULL if invalid */
  GThread   *thread;
} GxPrep;

static GPtrArray *
gx_cache_lookup (FpiDeviceGoodixSpi *self, GVariant *data)
{
  if (!data || !self->ref_cache)
    return NULL;
  for (guint i = 0; i < self->ref_cache->len; i++)
    {
      GxCacheEntry *e = g_ptr_array_index (self->ref_cache, i);
      if (g_variant_equal (e->data, data))
        return g_ptr_array_ref (e->refs);
    }
  return NULL;
}

static gpointer
gx_prep_thread (gpointer data)
{
  GxPrep *prep = data;

  for (guint p = 0; p < prep->prints->len; p++)
    if (!g_ptr_array_index (prep->refs, p))
      prep->refs->pdata[p] = gx_print_data_load (g_ptr_array_index (prep->datas, p));
  return NULL;
}

/* Starts preparing @prints; cached ones are reused immediately. */
static void
gx_prep_start (FpiDeviceGoodixSpi *self, GPtrArray *prints)
{
  GxPrep *prep = g_new0 (GxPrep, 1);
  gboolean missing = FALSE;

  prep->prints = g_ptr_array_ref (prints);
  prep->datas = g_ptr_array_new_with_free_func ((GDestroyNotify) g_variant_unref);
  prep->refs = g_ptr_array_new_with_free_func ((GDestroyNotify) g_ptr_array_unref);
  for (guint p = 0; p < prints->len; p++)
    {
      GVariant *data = NULL;
      GPtrArray *refs;

      g_object_get (g_ptr_array_index (prints, p), "fpi-data", &data, NULL);
      g_ptr_array_add (prep->datas, data);
      refs = gx_cache_lookup (self, data);
      missing |= refs == NULL;
      g_ptr_array_add (prep->refs, refs);
    }
  if (missing)
    prep->thread = g_thread_new ("goodixspi-prep", gx_prep_thread, prep);
  self->prep = prep;
}

static void
gx_prep_wait (GxPrep *prep)
{
  if (prep->thread)
    g_thread_join (g_steal_pointer (&prep->thread));
}

/* Waits for the preparation, stores new results in the cache and frees it. */
static void
gx_prep_finish (FpiDeviceGoodixSpi *self)
{
  GxPrep *prep = g_steal_pointer (&self->prep);

  if (!prep)
    return;
  gx_prep_wait (prep);
  if (!self->ref_cache)
    self->ref_cache = g_ptr_array_new_with_free_func ((GDestroyNotify) gx_cache_entry_free);
  for (guint p = 0; p < prep->prints->len; p++)
    {
      GVariant *data = g_ptr_array_index (prep->datas, p);
      GPtrArray *refs = g_ptr_array_index (prep->refs, p);
      g_autoptr(GPtrArray) cached = gx_cache_lookup (self, data);
      GxCacheEntry *e;

      if (!data || !refs || cached)
        continue;
      if (self->ref_cache->len >= GX_CACHE_MAX)
        g_ptr_array_remove_index (self->ref_cache, 0);
      e = g_new0 (GxCacheEntry, 1);
      e->data = g_variant_ref (data);
      e->refs = g_ptr_array_ref (refs);
      g_ptr_array_add (self->ref_cache, e);
    }
  g_ptr_array_unref (prep->prints);
  g_ptr_array_unref (prep->datas);
  g_ptr_array_unref (prep->refs);
  g_free (prep);
}

/* ------------------------------------------------------------------------ */
/* Matching (in a worker thread)                                            */

typedef struct
{
  gint16     diff[GX_IMG_PIXELS];
  GxPrep    *prep;                      /* borrowed from the device */
  guint      first;                     /* enrolled image to try first */
  FpPrint   *match;                     /* result */
  guint      match_idx;
  double     best;
} GxMatchJob;

static void
gx_match_job_free (GxMatchJob *job)
{
  g_free (job);
}

static void
gx_match_thread (GTask *task, gpointer source, gpointer task_data, GCancellable *cancellable)
{
  GxMatchJob *job = task_data;
  g_autoptr(GxmImage) probe_img = gxm_image_new (job->diff);
  g_autoptr(GxmProbe) probe = NULL;
  gint64 t0 = g_get_monotonic_time ();

  if (gxm_image_coverage (probe_img) < GX_MIN_COVERAGE)
    {
      g_task_return_new_error (task, FP_DEVICE_RETRY, FP_DEVICE_RETRY_CENTER_FINGER,
                               "Not enough finger contact (%.0f%%)",
                               100 * gxm_image_coverage (probe_img));
      return;
    }

  probe = gxm_probe_new (probe_img);
  gx_prep_wait (job->prep);
  for (guint p = 0; p < job->prep->prints->len && !job->match; p++)
    {
      FpPrint *print = g_ptr_array_index (job->prep->prints, p);
      GPtrArray *refs = g_ptr_array_index (job->prep->refs, p);
      guint idx;
      double s;

      if (!refs)
        {
          g_task_return_new_error (task, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID,
                                   "Invalid print data");
          return;
        }
      s = gxm_probe_search (probe, (GxmImage *const *) refs->pdata, refs->len,
                            job->first, GX_MATCH_THRESHOLD, &idx);

      job->best = MAX (job->best, s);
      if (s >= GX_MATCH_THRESHOLD)
        {
          job->match = print;
          job->match_idx = idx;
        }
      if (g_task_return_error_if_cancelled (task))
        return;
    }
  fp_dbg ("match: best %.3f (threshold %.2f) in %.0f ms", job->best, GX_MATCH_THRESHOLD,
          (g_get_monotonic_time () - t0) / 1000.0);
  g_task_return_boolean (task, TRUE);
}

static void
gx_match_start (FpiDeviceGoodixSpi *self, GAsyncReadyCallback cb)
{
  g_autoptr(GTask) task = g_task_new (self, fpi_device_get_cancellable (FP_DEVICE (self)), cb, NULL);
  GxMatchJob *job = g_new0 (GxMatchJob, 1);

  memcpy (job->diff, self->best_diff, sizeof (job->diff));
  job->prep = self->prep;
  job->first = self->last_match_idx;
  g_task_set_task_data (task, job, (GDestroyNotify) gx_match_job_free);
  g_task_run_in_thread (task, gx_match_thread);
}

/* ------------------------------------------------------------------------ */
/* Verify / identify                                                        */

static void
gx_verify_matched (GObject *source, GAsyncResult *res, gpointer user_data)
{
  FpDevice *dev = FP_DEVICE (source);
  GxMatchJob *job = g_task_get_task_data (G_TASK (res));
  GError *error = NULL;

  gx_prep_finish (FPI_DEVICE_GOODIXSPI (dev));
  if (!g_task_propagate_boolean (G_TASK (res), &error))
    {
      if (error->domain == FP_DEVICE_RETRY)
        {
          fpi_device_verify_report (dev, FPI_MATCH_ERROR, NULL, error);
          fpi_device_verify_complete (dev, NULL);
        }
      else
        {
          fpi_device_verify_complete (dev, error);
        }
      return;
    }
  if (job->match)
    FPI_DEVICE_GOODIXSPI (dev)->last_match_idx = job->match_idx;
  fpi_device_verify_report (dev, job->match ? FPI_MATCH_SUCCESS : FPI_MATCH_FAIL, NULL, NULL);
  fpi_device_verify_complete (dev, NULL);
}

static void
gx_identify_matched (GObject *source, GAsyncResult *res, gpointer user_data)
{
  FpDevice *dev = FP_DEVICE (source);
  GxMatchJob *job = g_task_get_task_data (G_TASK (res));
  GError *error = NULL;

  gx_prep_finish (FPI_DEVICE_GOODIXSPI (dev));
  if (!g_task_propagate_boolean (G_TASK (res), &error))
    {
      if (error->domain == FP_DEVICE_RETRY)
        {
          fpi_device_identify_report (dev, NULL, NULL, error);
          fpi_device_identify_complete (dev, NULL);
        }
      else
        {
          fpi_device_identify_complete (dev, error);
        }
      return;
    }
  if (job->match)
    FPI_DEVICE_GOODIXSPI (dev)->last_match_idx = job->match_idx;
  fpi_device_identify_report (dev, job->match, NULL, NULL);
  fpi_device_identify_complete (dev, NULL);
}

static void
gx_verify_scanned (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  if (error)
    {
      gx_prep_finish (FPI_DEVICE_GOODIXSPI (dev));
      fpi_device_verify_complete (dev, error);
      return;
    }
  gx_match_start (FPI_DEVICE_GOODIXSPI (dev), gx_verify_matched);
}

static void
gx_identify_scanned (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  if (error)
    {
      gx_prep_finish (FPI_DEVICE_GOODIXSPI (dev));
      fpi_device_identify_complete (dev, error);
      return;
    }
  gx_match_start (FPI_DEVICE_GOODIXSPI (dev), gx_identify_matched);
}

/* The stored prints are prepared in the background while the scan waits
 * for the finger, so matching can start as soon as the image is captured. */
static void
gx_verify (FpDevice *dev)
{
  FpiDeviceGoodixSpi *self = FPI_DEVICE_GOODIXSPI (dev);
  g_autoptr(GPtrArray) prints = g_ptr_array_new ();
  FpPrint *print;

  fpi_device_get_verify_data (dev, &print);
  g_ptr_array_add (prints, print);
  gx_prep_start (self, prints);
  gx_scan_start (self, gx_verify_scanned);
}

static void
gx_identify (FpDevice *dev)
{
  FpiDeviceGoodixSpi *self = FPI_DEVICE_GOODIXSPI (dev);
  GPtrArray *prints;

  fpi_device_get_identify_data (dev, &prints);
  gx_prep_start (self, prints);
  gx_scan_start (self, gx_identify_scanned);
}

/* ------------------------------------------------------------------------ */
/* Enroll                                                                   */

static void
gx_enroll_scanned (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceGoodixSpi *self = FPI_DEVICE_GOODIXSPI (dev);
  g_autoptr(GxmImage) img = NULL;
  FpPrint *print;
  double coverage;

  if (error)
    {
      g_clear_pointer (&self->enroll_images, g_ptr_array_unref);
      fpi_device_enroll_complete (dev, NULL, error);
      return;
    }

  img = gxm_image_new (self->best_diff);
  coverage = gxm_image_coverage (img);
  if (coverage < GX_MIN_COVERAGE)
    {
      fp_dbg ("enroll: contact %.0f%% too low", 100 * coverage);
      fpi_device_enroll_progress (dev, self->enroll_images->len, NULL,
                                  fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER));
      gx_scan_start (self, gx_enroll_scanned);
      return;
    }

  g_ptr_array_add (self->enroll_images, gx_diff_to_bytes (self->best_diff));
  fp_dbg ("enroll: stage %u/%d (contact %.0f%%)", self->enroll_images->len,
          GX_ENROLL_STAGES, 100 * coverage);
  fpi_device_enroll_progress (dev, self->enroll_images->len, NULL, NULL);
  if (self->enroll_images->len < GX_ENROLL_STAGES)
    {
      gx_scan_start (self, gx_enroll_scanned);
      return;
    }

  fpi_device_get_enroll_data (dev, &print);
  fpi_print_set_type (print, FPI_PRINT_RAW);
  g_object_set (print, "fpi-data", gx_print_data_new (self->enroll_images), NULL);
  g_clear_pointer (&self->enroll_images, g_ptr_array_unref);
  fpi_device_enroll_complete (dev, g_object_ref (print), NULL);
}

static void
gx_enroll (FpDevice *dev)
{
  FpiDeviceGoodixSpi *self = FPI_DEVICE_GOODIXSPI (dev);

  g_clear_pointer (&self->enroll_images, g_ptr_array_unref);
  self->enroll_images = g_ptr_array_new_with_free_func ((GDestroyNotify) g_bytes_unref);
  gx_scan_start (self, gx_enroll_scanned);
}

/* ------------------------------------------------------------------------ */
/* Open / close                                                             */

static void
gx_open_finish (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceGoodixSpi *self = FPI_DEVICE_GOODIXSPI (dev);

  if (error && self->spi_fd >= 0)
    {
      close (self->spi_fd);
      self->spi_fd = -1;
    }
  fpi_device_open_complete (dev, error);
}

static void
gx_open (FpDevice *dev)
{
  FpiDeviceGoodixSpi *self = FPI_DEVICE_GOODIXSPI (dev);
  const char *path = fpi_device_get_udev_data (dev, FPI_DEVICE_UDEV_SUBTYPE_SPIDEV);

  self->spi_fd = open (path, O_RDWR);
  if (self->spi_fd < 0)
    {
      fpi_device_open_complete (dev, g_error_new (G_IO_ERROR, g_io_error_from_errno (errno),
                                                  "unable to open %s", path));
      return;
    }
  self->frame_speed_khz = GX_FRAME_SPEED_KHZ;
  {
    guint32 hz = GX_SPEED_KHZ * 1000;
    ioctl (self->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz);
  }
  fpi_ssm_start (fpi_ssm_new (dev, gx_init_handler, GX_INIT_NSTATES), gx_open_finish);
}

static void
gx_close (FpDevice *dev)
{
  FpiDeviceGoodixSpi *self = FPI_DEVICE_GOODIXSPI (dev);

  if (self->spi_fd >= 0)
    {
      close (self->spi_fd);
      self->spi_fd = -1;
    }
  fpi_device_close_complete (dev, NULL);
}

static const FpIdEntry gx_id_table[] = {
  { .udev_types = FPI_DEVICE_UDEV_SUBTYPE_SPIDEV, .spi_acpi_id = "GXFP3200" },
  { .udev_types = 0 }
};

static void
fpi_device_goodixspi_init (FpiDeviceGoodixSpi *self)
{
  self->spi_fd = -1;
  self->rx = g_malloc0 (GX_FRAME_LEN);
  self->frame = g_new0 (guint16, GX_IMG_PIXELS);
  self->prev_frame = g_new0 (guint16, GX_IMG_PIXELS);
  self->bg = g_new0 (gint32, GX_IMG_PIXELS);
  self->calib_acc = g_new0 (guint32, GX_IMG_PIXELS);
  self->best_diff = g_new0 (gint16, GX_IMG_PIXELS);
}

static void
fpi_device_goodixspi_finalize (GObject *object)
{
  FpiDeviceGoodixSpi *self = FPI_DEVICE_GOODIXSPI (object);

  g_clear_pointer (&self->rx, g_free);
  g_clear_pointer (&self->frame, g_free);
  g_clear_pointer (&self->prev_frame, g_free);
  g_clear_pointer (&self->bg, g_free);
  g_clear_pointer (&self->calib_acc, g_free);
  g_clear_pointer (&self->best_diff, g_free);
  g_clear_pointer (&self->enroll_images, g_ptr_array_unref);
  gx_prep_finish (self);
  g_clear_pointer (&self->ref_cache, g_ptr_array_unref);

  G_OBJECT_CLASS (fpi_device_goodixspi_parent_class)->finalize (object);
}

static void
fpi_device_goodixspi_class_init (FpiDeviceGoodixSpiClass *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);

  dev_class->id = FP_COMPONENT;
  dev_class->full_name = "Goodix Milan F SPI Fingerprint Sensor";
  dev_class->type = FP_DEVICE_TYPE_UDEV;
  dev_class->id_table = gx_id_table;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  dev_class->nr_enroll_stages = GX_ENROLL_STAGES;
  /* a passive capacitive sensor, polled over SPI: nothing heats up. With the
   * default model, libfprint stops it after ~3 minutes of continuous scanning
   * ("Device disabled to prevent overheating"), i.e. a lock screen left on. */
  dev_class->temp_hot_seconds = -1;

  dev_class->open = gx_open;
  dev_class->close = gx_close;
  dev_class->enroll = gx_enroll;
  dev_class->verify = gx_verify;
  dev_class->identify = gx_identify;

  fpi_device_class_auto_initialize_features (dev_class);

  G_OBJECT_CLASS (klass)->finalize = fpi_device_goodixspi_finalize;
}

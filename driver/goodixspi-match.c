/*
 * Correlation matcher for the Goodix Milan F SPI sensor.
 * See goodixspi-match.h.
 *
 * Copyright (C) 2026 guillermodoghel
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include "goodixspi-match.h"

#include <complex.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define CROP          3
#define W             (GXM_SENSOR_W - 2 * CROP)
#define H             (GXM_SENSOR_H - 2 * CROP)
#define PIXELS        (W * H)
#define BOX           11            /* window for local contrast normalization */
#define N             256           /* FFT size; >= 2 * max(W, H) - 1, no wrap */
#define NN            (N * N)
#define MIN_OVERLAP   0.35          /* fraction of the probe that must overlap */

static const int rotations[] = { -12, -6, 0, 6, 12 };
#define N_ROT G_N_ELEMENTS (rotations)

struct _GxmImage
{
  double img[PIXELS];
  double mask[PIXELS];
};

typedef double complex cplx;

struct _GxmProbe
{
  /* per rotation: conjugated spectra of mask, img*mask and img^2*mask */
  cplx  *mm[N_ROT];
  cplx  *mf[N_ROT];
  cplx  *mf2[N_ROT];
  double mask_sum[N_ROT];
};

/* ------------------------------------------------------------------------ */
/* Complex helpers                                                          */

/* Plain complex arithmetic: the C operator '*' on complex numbers calls
 * __muldc3 (Annex G inf/nan handling), which dominates the FFT otherwise. */
static inline cplx
cmul (cplx a, cplx b)
{
  double ar = creal (a), ai = cimag (a), br = creal (b), bi = cimag (b);

  return (ar * br - ai * bi) + (ar * bi + ai * br) * I;
}

/* i * a */
static inline cplx
cmul_i (cplx a)
{
  return -cimag (a) + creal (a) * I;
}

/* ------------------------------------------------------------------------ */
/* FFT (radix-2, in place)                                                  */

static cplx twiddle[N / 2];
static int  bitrev[N];

static void
fft_init (void)
{
  static gsize done = 0;

  if (!g_once_init_enter (&done))
    return;

  for (int i = 0; i < N / 2; i++)
    twiddle[i] = cexp (-2.0 * M_PI * I * i / N);
  for (int i = 0; i < N; i++)
    {
      int r = 0;
      for (int b = 1, v = i; b < N; b <<= 1, v >>= 1)
        r = (r << 1) | (v & 1);
      bitrev[i] = r;
    }
  g_once_init_leave (&done, 1);
}

static void
fft_1d (cplx *x, int stride)
{
  for (int i = 0; i < N; i++)
    {
      int j = bitrev[i];
      if (j > i)
        {
          cplx t = x[i * stride];
          x[i * stride] = x[j * stride];
          x[j * stride] = t;
        }
    }
  for (int len = 2; len <= N; len <<= 1)
    {
      int half = len / 2, step = N / len;
      for (int i = 0; i < N; i += len)
        for (int k = 0; k < half; k++)
          {
            cplx w = twiddle[k * step];
            cplx a = x[(i + k) * stride];
            cplx b = cmul (x[(i + k + half) * stride], w);
            x[(i + k) * stride] = a + b;
            x[(i + k + half) * stride] = a - b;
          }
    }
}

/* columns are transformed in blocks so memory is read in whole cache lines */
#define COL_BLOCK 8

/* @rows: only the first @rows rows may be non-zero (the rest are skipped) */
static void
fft_2d_rows (cplx *x, int rows)
{
  cplx blk[N * COL_BLOCK];

  for (int r = 0; r < rows; r++)
    fft_1d (x + r * N, 1);
  for (int c0 = 0; c0 < N; c0 += COL_BLOCK)
    {
      for (int r = 0; r < N; r++)
        memcpy (blk + r * COL_BLOCK, x + r * N + c0, COL_BLOCK * sizeof (cplx));
      for (int j = 0; j < COL_BLOCK; j++)
        fft_1d (blk + j, COL_BLOCK);
      for (int r = 0; r < N; r++)
        memcpy (x + r * N + c0, blk + r * COL_BLOCK, COL_BLOCK * sizeof (cplx));
    }
}

static void
fft_2d (cplx *x)
{
  fft_2d_rows (x, N);
}

/* inverse via conjugation; result scaled by 1/NN */
static void
ifft_2d (cplx *x)
{
  for (int i = 0; i < NN; i++)
    x[i] = conj (x[i]);
  fft_2d (x);
  for (int i = 0; i < NN; i++)
    x[i] = conj (x[i]) / NN;
}

/* Z = FFT(a + i b) of two real arrays -> A, B (either may be NULL) */
static void
unpack (const cplx *z, cplx *a, cplx *b)
{
  for (int r = 0; r < N; r++)
    for (int c = 0; c < N; c++)
      {
        cplx zk = z[r * N + c];
        cplx zn = conj (z[((N - r) % N) * N + (N - c) % N]);
        if (a)
          a[r * N + c] = 0.5 * (zk + zn);
        if (b)
          b[r * N + c] = -0.5 * cmul_i (zk - zn);
      }
}

/* zero-padded FFTs of two real H x W arrays */
static void
fft_pair (const double *a, const double *b, cplx *z, cplx *out_a, cplx *out_b)
{
  memset (z, 0, NN * sizeof (cplx));
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      z[y * N + x] = a[y * W + x] + (b ? b[y * W + x] : 0.0) * I;
  fft_2d_rows (z, H);
  unpack (z, out_a, out_b);
}

/* ------------------------------------------------------------------------ */
/* Preprocessing                                                            */

static int
reflect (int i, int n)
{
  if (i < 0)
    return -i;
  if (i >= n)
    return 2 * (n - 1) - i;
  return i;
}

/* mean over a BOX x BOX window with reflected borders */
static void
box_mean (const double *in, double *out)
{
  double tmp[PIXELS];
  const int r = BOX / 2;

  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      {
        double s = 0;
        for (int k = -r; k <= r; k++)
          s += in[y * W + reflect (x + k, W)];
        tmp[y * W + x] = s;
      }
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      {
        double s = 0;
        for (int k = -r; k <= r; k++)
          s += tmp[reflect (y + k, H) * W + x];
        out[y * W + x] = s / (BOX * BOX);
      }
}

static int
cmp_double (const void *a, const void *b)
{
  double x = *(const double *) a, y = *(const double *) b;

  return (x > y) - (x < y);
}

/* linear-interpolated percentile, like numpy's default */
static double
percentile (const double *v, double q)
{
  double sorted[PIXELS];
  double pos, frac;
  int lo;

  memcpy (sorted, v, sizeof (sorted));
  qsort (sorted, PIXELS, sizeof (double), cmp_double);
  pos = q / 100.0 * (PIXELS - 1);
  lo = (int) floor (pos);
  frac = pos - lo;
  if (lo + 1 >= PIXELS)
    return sorted[PIXELS - 1];
  return sorted[lo] * (1 - frac) + sorted[lo + 1] * frac;
}

GxmImage *
gxm_image_new (const gint16 *diff)
{
  g_autofree double *d = g_new (double, PIXELS);
  g_autofree double *d2 = g_new (double, PIXELS);
  g_autofree double *m = g_new (double, PIXELS);
  g_autofree double *m2 = g_new (double, PIXELS);
  g_autofree double *sd = g_new (double, PIXELS);
  GxmImage *out = g_new0 (GxmImage, 1);
  double sd_med, m_p90;

  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      {
        double v = diff[(y + CROP) * GXM_SENSOR_W + x + CROP];
        d[y * W + x] = v;
        d2[y * W + x] = v * v;
      }
  box_mean (d, m);
  box_mean (d2, m2);
  for (int i = 0; i < PIXELS; i++)
    {
      sd[i] = sqrt (MAX (m2[i] - m[i] * m[i], 1e-6));
      out->img[i] = (d[i] - m[i]) / (sd[i] + 1e-3);
    }

  /* contact: ridge contrast present and the finger lowers the reading */
  sd_med = percentile (sd, 50);
  m_p90 = percentile (m, 90);
  for (int i = 0; i < PIXELS; i++)
    out->mask[i] = (sd[i] > 0.35 * sd_med && m[i] > 0.25 * m_p90) ? 1.0 : 0.0;

  return out;
}

void
gxm_image_free (GxmImage *image)
{
  g_free (image);
}

double
gxm_image_coverage (const GxmImage *image)
{
  double s = 0;

  for (int i = 0; i < PIXELS; i++)
    s += image->mask[i];
  return s / PIXELS;
}

/* bilinear rotation about the image center, like the prototype */
static void
rotate (const GxmImage *src, int degrees, double *img, double *mask)
{
  double a = degrees * M_PI / 180.0, ca = cos (a), sa = sin (a);
  double cx = (W - 1) / 2.0, cy = (H - 1) / 2.0;

  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      {
        double xs = ca * (x - cx) + sa * (y - cy) + cx;
        double ys = -sa * (x - cx) + ca * (y - cy) + cy;
        int x0 = (int) floor (xs), y0 = (int) floor (ys);
        double fx = xs - x0, fy = ys - y0;
        int i = y * W + x;

        if (x0 < 0 || y0 < 0 || x0 >= W - 1 || y0 >= H - 1)
          {
            img[i] = 0;
            mask[i] = 0;
            continue;
          }
#define BL(arr) (((arr)[y0 * W + x0] * (1 - fx) + (arr)[y0 * W + x0 + 1] * fx) * (1 - fy) + \
                 ((arr)[(y0 + 1) * W + x0] * (1 - fx) + (arr)[(y0 + 1) * W + x0 + 1] * fx) * fy)
        img[i] = BL (src->img);
        mask[i] = BL (src->mask) > 0.99 ? 1.0 : 0.0;
#undef BL
      }
}

/* ------------------------------------------------------------------------ */
/* Matching                                                                 */

typedef struct
{
  const GxmImage *image;
  GxmProbe       *probe;
  guint           r;
} RotJob;

/* spectra of one rotation of the probe */
static gpointer
probe_rotation (gpointer data)
{
  RotJob *job = data;
  GxmProbe *p = job->probe;
  guint r = job->r;
  g_autofree double *img = g_new (double, PIXELS);
  g_autofree double *mask = g_new (double, PIXELS);
  g_autofree double *f = g_new (double, PIXELS);
  g_autofree double *f2 = g_new (double, PIXELS);
  g_autofree cplx *z = g_new (cplx, NN);

  rotate (job->image, rotations[r], img, mask);
  p->mask_sum[r] = 0;
  for (int i = 0; i < PIXELS; i++)
    {
      f[i] = img[i] * mask[i];
      f2[i] = f[i] * img[i];
      p->mask_sum[r] += mask[i];
    }
  fft_pair (mask, f, z, p->mm[r], p->mf[r]);
  fft_pair (f2, NULL, z, p->mf2[r], NULL);
  for (int i = 0; i < NN; i++)
    {
      p->mm[r][i] = conj (p->mm[r][i]);
      p->mf[r][i] = conj (p->mf[r][i]);
      p->mf2[r][i] = conj (p->mf2[r][i]);
    }
  return NULL;
}

GxmProbe *
gxm_probe_new (const GxmImage *image)
{
  GxmProbe *p = g_new0 (GxmProbe, 1);
  RotJob jobs[N_ROT];
  GThread *threads[N_ROT] = { NULL };

  fft_init ();
  for (guint r = 0; r < N_ROT; r++)
    {
      p->mm[r] = g_new (cplx, NN);
      p->mf[r] = g_new (cplx, NN);
      p->mf2[r] = g_new (cplx, NN);
      jobs[r] = (RotJob) { image, p, r };
    }
  /* one thread per rotation (the first runs on the calling thread) */
  for (guint r = 1; r < N_ROT; r++)
    threads[r] = g_thread_new ("gxm-probe", probe_rotation, &jobs[r]);
  probe_rotation (&jobs[0]);
  for (guint r = 1; r < N_ROT; r++)
    g_thread_join (threads[r]);
  return p;
}

void
gxm_probe_free (GxmProbe *probe)
{
  if (!probe)
    return;
  for (guint r = 0; r < N_ROT; r++)
    {
      g_free (probe->mm[r]);
      g_free (probe->mf[r]);
      g_free (probe->mf2[r]);
    }
  g_free (probe);
}

double
gxm_probe_score (GxmProbe *p, const GxmImage *ref)
{
  g_autofree double *f = g_new (double, PIXELS);
  g_autofree double *f2 = g_new (double, PIXELS);
  g_autofree cplx *fm = g_new (cplx, NN);
  g_autofree cplx *ff = g_new (cplx, NN);
  g_autofree cplx *ff2 = g_new (cplx, NN);
  g_autofree cplx *z = g_new (cplx, NN);
  g_autofree cplx *z2 = g_new (cplx, NN);
  g_autofree cplx *z3 = g_new (cplx, NN);
  double best = 0;

  for (int i = 0; i < PIXELS; i++)
    {
      f[i] = ref->img[i] * ref->mask[i];
      f2[i] = f[i] * ref->img[i];
    }
  fft_pair (ref->mask, f, z, fm, ff);
  fft_pair (f2, NULL, z, ff2, NULL);

  for (guint r = 0; r < N_ROT; r++)
    {
      double min_overlap = MIN_OVERLAP * p->mask_sum[r];

      /* each product is the spectrum of a real correlation: pack two per
       * inverse transform (real part, imaginary part) */
      for (int i = 0; i < NN; i++)
        {
          z[i] = cmul (fm[i], p->mm[r][i]) + cmul_i (cmul (ff[i], p->mm[r][i]));     /* overlap, sF */
          z2[i] = cmul (fm[i], p->mf[r][i]) + cmul_i (cmul (ff2[i], p->mm[r][i]));   /* sM, sF2 */
          z3[i] = cmul (fm[i], p->mf2[r][i]) + cmul_i (cmul (ff[i], p->mf[r][i]));   /* sM2, sFM */
        }
      ifft_2d (z);
      ifft_2d (z2);
      ifft_2d (z3);

      for (int i = 0; i < NN; i++)
        {
          double overlap = round (creal (z[i]));
          double sF, sM, sF2, sM2, sFM, num, den;

          if (overlap < min_overlap)
            continue;
          overlap = MAX (overlap, 1);
          sF = cimag (z[i]);
          sM = creal (z2[i]);
          sF2 = cimag (z2[i]);
          sM2 = creal (z3[i]);
          sFM = cimag (z3[i]);
          num = sFM - sF * sM / overlap;
          den = (sF2 - sF * sF / overlap) * (sM2 - sM * sM / overlap);
          if (den > 1e-3 && num / sqrt (den) > best)
            best = num / sqrt (den);
        }
    }
  return best;
}

/* ------------------------------------------------------------------------ */
/* Parallel search over the enrolled images                                 */

typedef struct
{
  GxmProbe        *probe;
  GxmImage *const *refs;
  guint            n_refs;
  guint            first;          /* tried first (e.g. the last match) */
  double           threshold;
  gint             next;           /* atomic: next position to take */
  gint             found;          /* atomic: index that matched, or -1 */
  GMutex           lock;
  double           best;
  guint            best_idx;
} SearchState;

static gpointer
search_worker (gpointer data)
{
  SearchState *st = data;

  while (g_atomic_int_get (&st->found) < 0)
    {
      guint pos = g_atomic_int_add (&st->next, 1);
      guint idx;
      double s;

      if (pos >= st->n_refs)
        break;
      /* visit st->first, then the rest in order */
      idx = pos == 0 ? st->first : (pos <= st->first ? pos - 1 : pos);
      s = gxm_probe_score (st->probe, st->refs[idx]);

      g_mutex_lock (&st->lock);
      if (s > st->best)
        {
          st->best = s;
          st->best_idx = idx;
        }
      g_mutex_unlock (&st->lock);
      if (s >= st->threshold)
        g_atomic_int_compare_and_exchange (&st->found, -1, (gint) idx);
    }
  return NULL;
}

double
gxm_probe_search (GxmProbe *probe, GxmImage *const *refs, guint n_refs, guint first,
                  double threshold, guint *best_idx)
{
  SearchState st = {
    .probe = probe, .refs = refs, .n_refs = n_refs,
    .first = first < n_refs ? first : 0, .threshold = threshold,
    .next = 0, .found = -1, .best = 0, .best_idx = 0,
  };
  guint n_threads = CLAMP (g_get_num_processors (), 1, 4);
  GThread *threads[4] = { NULL };

  if (n_refs == 0)
    return 0;
  g_mutex_init (&st.lock);
  for (guint t = 1; t < n_threads; t++)
    threads[t] = g_thread_new ("gxm-search", search_worker, &st);
  search_worker (&st);
  for (guint t = 1; t < n_threads; t++)
    g_thread_join (threads[t]);
  g_mutex_clear (&st.lock);

  if (best_idx)
    *best_idx = st.best_idx;
  return st.best;
}

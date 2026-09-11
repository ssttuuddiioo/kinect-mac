// Camera-agnostic depth pipeline.
//
// Feed it one depth frame in millimetres plus (optionally) colour already
// registered to the depth grid, at any resolution, and it produces the same
// three outputs the Kinect shim does:
//
//   grey  - depth gated to a near/far slab, near-bright / far-dim
//   rgb   - colour blacked out wherever depth falls outside the slab
//   cloud - depth mm packed R=high byte, G=low byte, B=255 where valid
//
// This is what lets a second camera (Orbbec Femto Mega) reuse every filter we
// tuned on the Kinect without the Kinect shim changing. The filter maths lives
// in kfilters.h and is shared with k2shim.cpp.

#include <stdlib.h>
#include <string.h>
#include "kfilters.h"

namespace {
struct KProc {
    int w = 0, h = 0;
    float *prev = NULL;
    float *smoothed = NULL;
    unsigned char *scratch = NULL;
    int f_temporal = 0, f_median = 0, f_erode = 0;
};
}

extern "C" {

void *kproc_create(int w, int h) {
    if (w <= 0 || h <= 0) return NULL;
    KProc *p = new KProc();
    p->w = w; p->h = h;
    const size_t n = (size_t)w * h;
    p->prev = (float *)calloc(n, sizeof(float));
    p->smoothed = (float *)malloc(n * sizeof(float));
    p->scratch = (unsigned char *)malloc(n);
    if (!p->prev || !p->smoothed || !p->scratch) {
        free(p->prev); free(p->smoothed); free(p->scratch);
        delete p;
        return NULL;
    }
    return p;
}

// temporal 0-90 (% weight of previous frame), median radius 0-3, erode 0-5 px.
void kproc_set_filters(void *h, int temporal, int median, int erode) {
    KProc *p = static_cast<KProc *>(h);
    if (!p) return;
    p->f_temporal = temporal < 0 ? 0 : (temporal > 90 ? 90 : temporal);
    p->f_median = median < 0 ? 0 : (median > 3 ? 3 : median);
    p->f_erode = erode < 0 ? 0 : (erode > 5 ? 5 : erode);
}

// depth_mm : w*h floats, millimetres, <=0 means no reading
// colour   : w*h*3 RGB registered to depth, or NULL
// mirror   : 1 flips horizontally so the image reads like a webcam
// outputs  : grey w*h (required); rgb w*h*3 and cloud w*h*3 may be NULL
// Returns a bitmask: 1 grey, 2 rgb, 4 cloud.
int kproc_run(void *h, const float *depth_mm, const unsigned char *colour,
              int near_mm, int far_mm, int mirror,
              unsigned char *grey, unsigned char *rgb, unsigned char *cloud) {
    KProc *p = static_cast<KProc *>(h);
    if (!p || !depth_mm || !grey) return 0;
    const int w = p->w, hgt = p->h;
    const size_t n = (size_t)w * hgt;
    const float span = (float)(far_mm - near_mm);
    int wrote = 0;

    // Pass 1 - temporal smoothing, in source space. Only blend where both
    // frames have depth, so holes never smear across the image.
    const float a = p->f_temporal / 100.0f;
    for (size_t i = 0; i < n; ++i) {
        float cur = depth_mm[i];
        if (a > 0.0f && cur > 0.0f && p->prev[i] > 0.0f)
            cur = a * p->prev[i] + (1.0f - a) * cur;
        p->prev[i] = cur;
        p->smoothed[i] = cur;
    }

    // Pass 2 - gate to the slab, optionally mirrored.
    for (int y = 0; y < hgt; ++y)
        for (int x = 0; x < w; ++x) {
            const int dx = mirror ? (w - 1 - x) : x;
            grey[y * w + dx] = shade(p->smoothed[y * w + x], near_mm, far_mm, span);
        }

    // Pass 3 - clean the mask.
    if (p->f_median > 0) median_n(grey, p->scratch, w, hgt, p->f_median);
    if (p->f_erode > 0) open_mask(grey, p->scratch, w, hgt, p->f_erode);
    wrote |= 1;

    // Pass 4 - packed depth for point clouds. Re-check z against the gate:
    // median and dilate can switch pixels on from their neighbours, which
    // would otherwise emit points outside the slab.
    if (cloud) {
        for (int y = 0; y < hgt; ++y)
            for (int x = 0; x < w; ++x) {
                const int si = y * w + x;
                const int di = y * w + (mirror ? (w - 1 - x) : x);
                unsigned char *c = cloud + (size_t)di * 3;
                const float z = p->smoothed[si];
                if (grey[di] && z >= near_mm && z <= far_mm) {
                    const int mm = (int)(z + 0.5f);
                    c[0] = (unsigned char)((mm >> 8) & 0xFF);
                    c[1] = (unsigned char)(mm & 0xFF);
                    c[2] = 255;
                } else {
                    c[0] = c[1] = c[2] = 0;
                }
            }
        wrote |= 4;
    }

    // Pass 5 - colour, masked by the *filtered* depth so every output shares
    // one silhouette.
    if (rgb && colour) {
        for (int y = 0; y < hgt; ++y)
            for (int x = 0; x < w; ++x) {
                const int si = y * w + x;
                const int di = y * w + (mirror ? (w - 1 - x) : x);
                const unsigned char *s = colour + (size_t)si * 3;
                unsigned char *d = rgb + (size_t)di * 3;
                if (grey[di]) { d[0] = s[0]; d[1] = s[1]; d[2] = s[2]; }
                else          { d[0] = d[1] = d[2] = 0; }
            }
        wrote |= 2;
    }
    return wrote;
}

void kproc_destroy(void *h) {
    KProc *p = static_cast<KProc *>(h);
    if (!p) return;
    free(p->prev); free(p->smoothed); free(p->scratch);
    delete p;
}
}

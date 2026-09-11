// Depth filtering shared by every camera backend.
//
// One implementation, used by both the Kinect v2 shim and the generic kproc
// pipeline, so the Femto Mega gets exactly the filtering we tuned on the
// Kinect rather than a second copy that drifts. All functions work on an
// 8-bit mask where 0 means "no depth / removed".

#pragma once
#include <string.h>
#include <algorithm>


// Median despeckle with a selectable kernel: kills salt-and-pepper without
// rounding off edges the way a blur would. radius 1/2/3 = 3x3 / 5x5 / 7x7.
// Bigger kernels swallow bigger noise clumps but cost more and erase fine
// detail. nth_element is O(n) average, which beats sorting all 49 values.
static inline void median_n(unsigned char *img, unsigned char *tmp, int w, int h, int radius) {
    if (radius < 1) return;
    if (radius > 3) radius = 3;
    memcpy(tmp, img, (size_t)w * h);
    const int k = (2 * radius + 1) * (2 * radius + 1);
    unsigned char v[49];
    for (int y = radius; y < h - radius; ++y) {
        for (int x = radius; x < w - radius; ++x) {
            int n = 0;
            for (int dy = -radius; dy <= radius; ++dy)
                for (int dx = -radius; dx <= radius; ++dx)
                    v[n++] = tmp[(y + dy) * w + (x + dx)];
            std::nth_element(v, v + k / 2, v + k);
            img[y * w + x] = v[k / 2];
        }
    }
}

// Morphological opening on the mask: erode then dilate. Erode peels off the
// noisy fringe that haloes every depth edge; the dilate puts the real size back.
static inline void open_mask(unsigned char *img, unsigned char *tmp, int w, int h, int radius) {
    for (int pass = 0; pass < radius; ++pass) {     // erode
        memcpy(tmp, img, (size_t)w * h);
        for (int y = 1; y < h - 1; ++y)
            for (int x = 1; x < w - 1; ++x) {
                if (!tmp[y * w + x]) continue;
                bool edge = false;
                for (int dy = -1; dy <= 1 && !edge; ++dy)
                    for (int dx = -1; dx <= 1; ++dx)
                        if (!tmp[(y + dy) * w + (x + dx)]) { edge = true; break; }
                if (edge) img[y * w + x] = 0;
            }
    }
    for (int pass = 0; pass < radius; ++pass) {     // dilate
        memcpy(tmp, img, (size_t)w * h);
        for (int y = 1; y < h - 1; ++y)
            for (int x = 1; x < w - 1; ++x) {
                if (tmp[y * w + x]) continue;
                unsigned char best = 0;
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        unsigned char n = tmp[(y + dy) * w + (x + dx)];
                        if (n > best) best = n;
                    }
                img[y * w + x] = best;
            }
    }
}

static inline unsigned char shade(float v, int near_mm, int far_mm, float span) {
    if (!(v > 0.0f) || v < near_mm || v > far_mm || span <= 0.0f) return 0;
    float t = 1.0f - (v - near_mm) / span;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return static_cast<unsigned char>(t * 255.0f);
}

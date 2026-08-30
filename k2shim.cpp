// C shim over libfreenect2's C++ API, for ctypes.
//
// Produces two co-registered 512x424 outputs from one frame:
//   grey - depth, gated to a near/far slab, near-bright / far-dim
//   rgb  - colour pixels aligned to the depth camera, blacked out wherever
//          depth falls outside the slab (a depth matte, not chroma key)
//
// Colour is optional at open time: skipping it roughly triples throughput,
// which matters when the sensor sits behind a USB hub.

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <string>
#include <string.h>
#include <algorithm>
#include <stdlib.h>

namespace lf = libfreenect2;

namespace {
struct K2 {
    lf::Freenect2 ctx;
    lf::Freenect2Device *dev = nullptr;
    lf::SyncMultiFrameListener *listener = nullptr;
    lf::Registration *reg = nullptr;
    lf::Frame *undistorted = nullptr;
    lf::Frame *registered = nullptr;
    std::string serial;
    bool colour = false;

    // filter state
    float *prev = NULL;          // previous depth frame, for temporal smoothing
    unsigned char *scratch = NULL;  // ping-pong buffer for median / morphology
    float *smoothed = NULL;      // depth after temporal smoothing
    int f_temporal = 0;          // 0-90, weight of the previous frame as a percent
    int f_median = 0;            // despeckle radius: 0 off, 1=3x3, 2=5x5, 3=7x7
    int f_erode = 0;             // 0-5, morphological opening radius in pixels

    // point-cloud output: depth in mm packed as R=high byte, G=low byte,
    // B=255 where valid. 8-bit shading is far too coarse to unproject from.
    unsigned char *cloud = NULL;
    int want_cloud = 0;
};

// Median despeckle with a selectable kernel: kills salt-and-pepper without
// rounding off edges the way a blur would. radius 1/2/3 = 3x3 / 5x5 / 7x7.
// Bigger kernels swallow bigger noise clumps but cost more and erase fine
// detail. nth_element is O(n) average, which beats sorting all 49 values.
void median_n(unsigned char *img, unsigned char *tmp, int w, int h, int radius) {
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
void open_mask(unsigned char *img, unsigned char *tmp, int w, int h, int radius) {
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

inline unsigned char shade(float v, int near_mm, int far_mm, float span) {
    if (!(v > 0.0f) || v < near_mm || v > far_mm || span <= 0.0f) return 0;
    float t = 1.0f - (v - near_mm) / span;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return static_cast<unsigned char>(t * 255.0f);
}
}

extern "C" {

int k2_width(void) { return 512; }
int k2_height(void) { return 424; }

void *k2_open(int want_colour) {
    lf::setGlobalLogger(lf::createConsoleLogger(lf::Logger::Error));
    K2 *k = new K2();
    k->colour = want_colour != 0;

    if (k->ctx.enumerateDevices() == 0) { delete k; return nullptr; }
    k->serial = k->ctx.getDefaultDeviceSerialNumber();
    k->dev = k->ctx.openDevice(k->serial);
    if (!k->dev) { delete k; return nullptr; }

    const unsigned int types = k->colour
        ? (lf::Frame::Color | lf::Frame::Depth)
        : lf::Frame::Depth;
    k->listener = new lf::SyncMultiFrameListener(types);
    if (k->colour) k->dev->setColorFrameListener(k->listener);
    k->dev->setIrAndDepthFrameListener(k->listener);

    if (!k->dev->startStreams(k->colour, true)) { delete k; return nullptr; }

    if (k->colour) {
        k->reg = new lf::Registration(k->dev->getIrCameraParams(),
                                      k->dev->getColorCameraParams());
        k->undistorted = new lf::Frame(512, 424, 4);
        k->registered = new lf::Frame(512, 424, 4);
    }
    return k;
}

const char *k2_serial(void *h) {
    K2 *k = static_cast<K2 *>(h);
    return k ? k->serial.c_str() : "";
}

int k2_has_colour(void *h) {
    K2 *k = static_cast<K2 *>(h);
    return (k && k->colour) ? 1 : 0;
}

// Fills grey (w*h) and, when colour is on and rgb is non-null, rgb (w*h*3).
// Returns a bitmask: 1 = grey written, 2 = rgb written, 0 = timed out.
int k2_frame(void *h, unsigned char *grey, unsigned char *rgb,
             int near_mm, int far_mm) {
    K2 *k = static_cast<K2 *>(h);
    if (!k) return 0;

    lf::FrameMap frames;
    if (!k->listener->waitForNewFrame(frames, 5000)) return 0;

    lf::Frame *depth = frames[lf::Frame::Depth];
    const int w = static_cast<int>(depth->width), hgt = static_cast<int>(depth->height);
    const float span = static_cast<float>(far_mm - near_mm);
    int wrote = 0;

    bool want_rgb = k->colour && rgb != nullptr;
    lf::Frame *colour = want_rgb ? frames[lf::Frame::Color] : nullptr;
    if (want_rgb && colour) {
        k->reg->apply(colour, depth, k->undistorted, k->registered);
    } else {
        want_rgb = false;
    }

    // Depth used for gating: the undistorted map lines up with the registered
    // colour, so both outputs are masked by exactly the same pixels.
    const float *dz = reinterpret_cast<const float *>(
        want_rgb ? k->undistorted->data : depth->data);
    const unsigned char *cz = want_rgb ? k->registered->data : nullptr;
    const bool bgr = want_rgb && colour->format == lf::Frame::BGRX;

    const size_t n = (size_t)w * hgt;
    if (!k->scratch) {
        k->scratch = (unsigned char *)malloc(n);
        k->prev = (float *)calloc(n, sizeof(float));
        k->smoothed = (float *)malloc(n * sizeof(float));
    }
    if (!k->scratch || !k->prev || !k->smoothed) {
        k->listener->release(frames);
        return 0;
    }

    // Pass 1 - temporal smoothing, in source space. Blending only where both
    // frames have valid depth, so holes never smear across the image.
    const float a = k->f_temporal / 100.0f;
    for (size_t i = 0; i < n; ++i) {
        float cur = dz[i];
        if (a > 0.0f && cur > 0.0f && k->prev[i] > 0.0f)
            cur = a * k->prev[i] + (1.0f - a) * cur;
        k->prev[i] = cur;
        k->smoothed[i] = cur;
    }

    // Pass 2 - gate to the near/far slab, mirrored so it reads like a webcam.
    for (int y = 0; y < hgt; ++y)
        for (int x = 0; x < w; ++x)
            grey[y * w + (w - 1 - x)] =
                shade(k->smoothed[y * w + x], near_mm, far_mm, span);

    // Pass 3 - clean the mask.
    if (k->f_median > 0) median_n(grey, k->scratch, w, hgt, k->f_median);
    if (k->f_erode > 0) open_mask(grey, k->scratch, w, hgt, k->f_erode);

    // Pass 3b - packed 16-bit depth for point-cloud use, gated by the same
    // filtered mask so the cloud matches the other feeds exactly.
    if (k->want_cloud) {
        if (!k->cloud) k->cloud = (unsigned char *)malloc(n * 3);
        if (k->cloud) {
            for (int y = 0; y < hgt; ++y) {
                for (int x = 0; x < w; ++x) {
                    const int si = y * w + x;
                    const int di = y * w + (w - 1 - x);
                    unsigned char *p = k->cloud + di * 3;
                    const float z = k->smoothed[si];
                    // The mask can grow past the slab: median and dilate both
                    // switch pixels on from their neighbours. Re-check z so the
                    // cloud never carries points outside the gate.
                    if (grey[di] && z >= near_mm && z <= far_mm) {
                        const int mm = (int)(z + 0.5f);
                        p[0] = (unsigned char)((mm >> 8) & 0xFF);
                        p[1] = (unsigned char)(mm & 0xFF);
                        p[2] = 255;
                    } else {
                        p[0] = p[1] = p[2] = 0;
                    }
                }
            }
        }
    }

    // Pass 4 - colour, masked by the *filtered* depth so both outputs still
    // share one silhouette.
    if (want_rgb) {
        for (int y = 0; y < hgt; ++y) {
            for (int x = 0; x < w; ++x) {
                const int si = y * w + x;
                const int di = y * w + (w - 1 - x);
                unsigned char r = 0, gg = 0, b = 0;
                if (grey[di]) {
                    const unsigned char *p = cz + si * 4;
                    if (bgr) { b = p[0]; gg = p[1]; r = p[2]; }
                    else     { r = p[0]; gg = p[1]; b = p[2]; }
                }
                rgb[di * 3 + 0] = r;
                rgb[di * 3 + 1] = gg;
                rgb[di * 3 + 2] = b;
            }
        }
    }
    wrote |= 1;
    if (want_rgb) wrote |= 2;

    k->listener->release(frames);
    return wrote;
}

// temporal: 0-90 (percent weight of the previous frame; higher = smoother but
// smears motion). median: despeckle radius 0-3 (off / 3x3 / 5x5 / 7x7).
// erode: 0-5 px mask opening.
void k2_set_filters(void *h, int temporal, int median, int erode) {
    K2 *k = static_cast<K2 *>(h);
    if (!k) return;
    k->f_temporal = temporal < 0 ? 0 : (temporal > 90 ? 90 : temporal);
    k->f_median = median < 0 ? 0 : (median > 3 ? 3 : median);
    k->f_erode = erode < 0 ? 0 : (erode > 5 ? 5 : erode);
}

// Enable the packed-depth cloud buffer (costs one extra pass per frame).
void k2_set_cloud(void *h, int enable) {
    K2 *k = static_cast<K2 *>(h);
    if (k) k->want_cloud = enable ? 1 : 0;
}

// Copy the last packed-depth frame out. Buffer must be width*height*3.
int k2_get_cloud(void *h, unsigned char *out) {
    K2 *k = static_cast<K2 *>(h);
    if (!k || !k->cloud || !out) return 0;
    memcpy(out, k->cloud, (size_t)512 * 424 * 3);
    return 1;
}

// IR camera intrinsics - TouchDesigner needs these to unproject depth to XYZ.
int k2_intrinsics(void *h, float *fx, float *fy, float *cx, float *cy) {
    K2 *k = static_cast<K2 *>(h);
    if (!k || !k->dev) return 0;
    libfreenect2::Freenect2Device::IrCameraParams p = k->dev->getIrCameraParams();
    *fx = p.fx; *fy = p.fy; *cx = p.cx; *cy = p.cy;
    return 1;
}

void k2_close(void *h) {
    K2 *k = static_cast<K2 *>(h);
    if (!k) return;
    if (k->dev) { k->dev->stop(); k->dev->close(); }
    free(k->cloud);
    free(k->scratch);
    free(k->prev);
    free(k->smoothed);
    delete k->registered;
    delete k->undistorted;
    delete k->reg;
    delete k->listener;
    delete k;
}
}

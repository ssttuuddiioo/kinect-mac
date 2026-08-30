// Syphon publisher: exposes the Kinect feeds as Syphon servers, so
// TouchDesigner (and Resolume, MadMapper, VDMX...) can pull them GPU-side
// with none of the latency MJPEG-over-HTTP introduces.
//
// Syphon's OpenGL server needs a CGL context; we make a headless one. All GL
// work happens inside publish, which makes the context current on whatever
// thread calls it, so a background grab thread is fine.

#import <Foundation/Foundation.h>
#import <OpenGL/OpenGL.h>
#import <OpenGL/gl.h>
#import <Syphon/Syphon.h>
#include <stdlib.h>
#include <string.h>

namespace {
CGLContextObj gCtx = NULL;
SyphonOpenGLServer *gServer[3] = {nil, nil, nil};
GLuint gTex[3] = {0, 0, 0};
int gW[3] = {0, 0, 0}, gH[3] = {0, 0, 0};
unsigned char *gScratch = NULL;
size_t gScratchSize = 0;
}

extern "C" {

int syphon_init(const char *depth_name, const char *colour_name,
                const char *cloud_name) {
    @autoreleasepool {
        CGLPixelFormatAttribute attrs[] = {
            kCGLPFAAccelerated,
            (CGLPixelFormatAttribute)0,
        };
        CGLPixelFormatObj pix = NULL;
        GLint npix = 0;
        if (CGLChoosePixelFormat(attrs, &pix, &npix) != kCGLNoError || !pix) {
            // Fall back to a software renderer rather than failing outright.
            CGLPixelFormatAttribute soft[] = { (CGLPixelFormatAttribute)0 };
            if (CGLChoosePixelFormat(soft, &pix, &npix) != kCGLNoError || !pix) return 0;
        }
        CGLError err = CGLCreateContext(pix, NULL, &gCtx);
        CGLDestroyPixelFormat(pix);
        if (err != kCGLNoError || !gCtx) return 0;

        CGLSetCurrentContext(gCtx);
        glGenTextures(3, gTex);

        gServer[0] = [[SyphonOpenGLServer alloc]
            initWithName:[NSString stringWithUTF8String:depth_name]
                 context:gCtx options:nil];
        gServer[1] = [[SyphonOpenGLServer alloc]
            initWithName:[NSString stringWithUTF8String:colour_name]
                 context:gCtx options:nil];
        gServer[2] = [[SyphonOpenGLServer alloc]
            initWithName:[NSString stringWithUTF8String:cloud_name]
                 context:gCtx options:nil];
        return (gServer[0] && gServer[1] && gServer[2]) ? 1 : 0;
    }
}

// channels: 1 = greyscale (expanded to RGB here), 3 = RGB.
int syphon_publish(int idx, const unsigned char *data, int w, int h, int channels) {
    if (idx < 0 || idx > 2 || !gCtx || !gServer[idx] || !data) return 0;
    @autoreleasepool {
        CGLSetCurrentContext(gCtx);

        const unsigned char *rgb = data;
        if (channels == 1) {
            size_t need = (size_t)w * h * 3;
            if (gScratchSize < need) {
                free(gScratch);
                gScratch = (unsigned char *)malloc(need);
                gScratchSize = gScratch ? need : 0;
            }
            if (!gScratch) return 0;
            const size_t n = (size_t)w * h;
            for (size_t i = 0; i < n; ++i) {
                const unsigned char v = data[i];
                gScratch[i * 3 + 0] = v;
                gScratch[i * 3 + 1] = v;
                gScratch[i * 3 + 2] = v;
            }
            rgb = gScratch;
        }

        glBindTexture(GL_TEXTURE_2D, gTex[idx]);
        if (gW[idx] != w || gH[idx] != h) {
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, w, h, 0,
                         GL_RGB, GL_UNSIGNED_BYTE, rgb);
            gW[idx] = w; gH[idx] = h;
        } else {
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h,
                            GL_RGB, GL_UNSIGNED_BYTE, rgb);
        }
        glBindTexture(GL_TEXTURE_2D, 0);
        glFlush();

        [gServer[idx] publishFrameTexture:gTex[idx]
                            textureTarget:GL_TEXTURE_2D
                              imageRegion:NSMakeRect(0, 0, w, h)
                        textureDimensions:NSMakeSize(w, h)
                                  flipped:NO];
        return 1;
    }
}

// Syphon announces servers over distributed notifications, and answers client
// discovery requests on the runloop. A tight publish loop never services it, so
// the servers stay invisible. Call this each frame - it drains pending events
// without blocking.
void syphon_pump(void) {
    @autoreleasepool {
        [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode
                                 beforeDate:[NSDate date]];
    }
}

int syphon_has_clients(int idx) {
    if (idx < 0 || idx > 2 || !gServer[idx]) return 0;
    return gServer[idx].hasClients ? 1 : 0;
}

// Lists Syphon servers visible on this machine. Announcements arrive as
// distributed notifications, so the runloop has to spin before we look.
int syphon_servers(char *out, int len) {
    @autoreleasepool {
        SyphonServerDirectory *dir = [SyphonServerDirectory sharedDirectory];
        [[NSRunLoop currentRunLoop] runUntilDate:
            [NSDate dateWithTimeIntervalSinceNow:1.5]];
        NSMutableString *acc = [NSMutableString string];
        for (NSDictionary *s in dir.servers) {
            [acc appendFormat:@"%@|%@\n",
                s[SyphonServerDescriptionAppNameKey] ?: @"?",
                s[SyphonServerDescriptionNameKey] ?: @"?"];
        }
        const char *utf8 = acc.UTF8String;
        if (!utf8) return 0;
        strncpy(out, utf8, len - 1);
        out[len - 1] = '\0';
        return (int)dir.servers.count;
    }
}

void syphon_shutdown(void) {
    @autoreleasepool {
        for (int i = 0; i < 3; ++i) {
            if (gServer[i]) { [gServer[i] stop]; gServer[i] = nil; }
        }
        if (gCtx) {
            CGLSetCurrentContext(gCtx);
            glDeleteTextures(3, gTex);
            CGLSetCurrentContext(NULL);
            CGLDestroyContext(gCtx);
            gCtx = NULL;
        }
        free(gScratch); gScratch = NULL; gScratchSize = 0;
    }
}
}

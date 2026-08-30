# Kinect v2 point cloud in TouchDesigner

The `Kinect Cloud` Syphon feed carries **16-bit depth in millimetres**, packed
into an 8-bit RGB texture:

| Channel | Meaning                      |
|---------|------------------------------|
| R       | depth mm, high byte          |
| G       | depth mm, low byte           |
| B       | 255 where the point is valid |

Depth is sent rather than XYZ because X and Y are derivable from pixel position
plus depth, so one channel pair carries everything and keeps full 1 mm
precision. Sending XYZ as 8-bit would quantise to ~16 mm steps and look like
stair-stepping. The cloud uses the same near/far gate and the same despeckle /
erode filtering as the other feeds, so all three stay in register.

## Intrinsics for this sensor

    fx = 365.357   fy = 365.357
    cx = 261.003   cy = 207.894

These are printed on startup by `syphon_out.py`. They are per-sensor - if you
swap Kinects, read the new ones off the console.

## Network

    Syphon Spout In TOP        server = "Kinect Cloud" (app: Python)
      -> GLSL TOP              the shader below; output format 32-bit float RGBA
        -> Geometry COMP       instancing, positions sampled from the GLSL TOP
          -> Render TOP

On the GLSL TOP: set **Output Format** to *32-bit float (RGBA)*, or the XYZ you
just computed gets crushed back to 8 bits. Add a vec4 uniform named
`uIntrinsics` on the Vectors page = `(365.357, 365.357, 261.003, 207.894)`.

On the Geometry COMP: Instance Count = 512 * 424 = 217088, and point Translate
X/Y/Z at the GLSL TOP's R/G/B channels.

## The shader

```glsl
// Kinect v2 depth -> XYZ point cloud.
// in : R/G = depth mm (high/low byte), B = validity
// out: RGB = position in metres, A = 1 for valid points
//
// texelFetch, not texture(): depth is split across two bytes, so ANY
// interpolation blends a high byte with a low byte and produces nonsense
// distances. texelFetch reads the exact texel and ignores filtering.

uniform vec4 uIntrinsics;   // fx, fy, cx, cy

out vec4 fragColor;

void main()
{
    ivec2 res = textureSize(sTD2DInputs[0], 0);
    ivec2 pix = ivec2(gl_FragCoord.xy);

    vec4 t = texelFetch(sTD2DInputs[0], pix, 0);

    if (t.b < 0.5) {                 // no depth here
        fragColor = vec4(0.0);
        return;
    }

    float mm = t.r * 255.0 * 256.0 + t.g * 255.0;
    float z  = mm / 1000.0;          // metres

    // DEBUG: uncomment for a smooth grey ramp that proves depth decoded.
    // Near things dark, far things bright. If you see harsh banding or
    // random speckle instead, the bytes are being filtered somewhere.
    // fragColor = vec4(vec3(z / 4.0), 1.0); return;

    // The feed is mirrored so it reads like a webcam, so texel column
    // res.x-1-x is camera column x. Row order is unchanged.
    float u = float(res.x - 1 - pix.x);
    float v = float(pix.y);

    float x = (u - uIntrinsics.z) * z / uIntrinsics.x;
    float y = (v - uIntrinsics.w) * z / uIntrinsics.y;

    // Camera space is X right, Y down, Z forward. TouchDesigner wants Y up
    // and -Z forward, hence the two negations.
    fragColor = vec4(x, -y, -z, 1.0);
}
```

## If it looks wrong

- **Stair-stepped depth** - the GLSL TOP is still 8-bit. Set Output Format to
  32-bit float RGBA.
- **Upside down** - drop a Flip TOP between the Syphon TOP and the GLSL TOP.
  Whether this is needed depends on how your TD version handles texture origin.
- **Mirrored geometry** - remove the `1.0 -` on the sample, and use
  `u = (1.0 - vUV.s) * res.x`.
- **Everything at the origin** - `uIntrinsics` is missing or zero, so
  `z / fx` divides by zero.
- **Sparse or speckled** - raise Despeckle to 1 and Erode to 1-2 in the
  publisher window; those clean the mask the cloud is gated by.

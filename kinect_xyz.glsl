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

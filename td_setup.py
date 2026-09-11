"""Build the Kinect point-cloud network inside TouchDesigner.

NOT run from the terminal - paste this into TouchDesigner's Textport
(Dialogs -> Textport and DATs, or Alt+T) and press Enter.

It creates:  Syphon Spout In TOP -> GLSL TOP (+ shader DAT) -> Geometry COMP
             (instanced) -> Camera / Light -> Render TOP

Parameter names differ between TD builds, so every parameter set here is
attempted defensively: anything that does not exist is reported at the end
rather than throwing, and you can set those few by hand.
"""

INTRINSICS = (365.357, 365.357, 261.003, 207.894)   # fx, fy, cx, cy
# ^ per-sensor. The app prints yours at startup - replace these.
SERVER = "Kinect Cloud"
# Output resolution differs per camera: Kinect v2 512x424, Femto Mega 640x576
# (or 1024x1024 in wide-FOV). Read from the live Syphon feed when possible;
# these are only the fallback if nothing is streaming yet.
W, H = 640, 576

SHADER = '''uniform vec4 uIntrinsics;   // fx, fy, cx, cy
out vec4 fragColor;

void main()
{
    vec2 res = vec2(textureSize(sTD2DInputs[0], 0));
    vec2 uv  = vec2(1.0 - vUV.s, vUV.t);
    vec4 t   = texture(sTD2DInputs[0], uv);

    if (t.b < 0.5) { fragColor = vec4(0.0); return; }

    float mm = t.r * 255.0 * 256.0 + t.g * 255.0;
    float z  = mm / 1000.0;

    float u = vUV.s * res.x;
    float v = (1.0 - vUV.t) * res.y;

    float x = (u - uIntrinsics.z) * z / uIntrinsics.x;
    float y = (v - uIntrinsics.w) * z / uIntrinsics.y;

    fragColor = vec4(x, -y, -z, 1.0);
}
'''

missing = []


def setpar(operator, names, value):
    """Set the first parameter that exists from a list of candidate names."""
    for name in names:
        par = getattr(operator.par, name, None)
        if par is not None:
            try:
                par.val = value
                return True
            except Exception as exc:
                missing.append("%s.%s = %r (%s)" % (operator.name, name, value, exc))
                return False
    missing.append("%s: none of %s exist - set it by hand" % (operator.name, names))
    return False


parent_comp = op('/project1')
for name in ("kinect_syphon", "kinect_xyz", "kinect_shader",
             "kinect_geo", "kinect_cam", "kinect_light", "kinect_render"):
    existing = parent_comp.op(name)
    if existing:
        existing.destroy()

syphon = parent_comp.create(syphonspoutinTOP, "kinect_syphon")
setpar(syphon, ["Signalsource", "signalsource", "Sourcetype"], "Syphon")
setpar(syphon, ["Syphonservername", "syphonservername", "Servername"], SERVER)
setpar(syphon, ["Syphonappname", "syphonappname", "Appname"], "Python")
try:
    syphon.cook(force=True)
    if syphon.width > 1 and syphon.height > 1:
        W, H = syphon.width, syphon.height
except Exception:
    pass
print("using resolution %dx%d" % (W, H))

shader_dat = parent_comp.create(textDAT, "kinect_shader")
shader_dat.text = SHADER

xyz = parent_comp.create(glslTOP, "kinect_xyz")
xyz.inputConnectors[0].connect(syphon)
setpar(xyz, ["pixeldat", "Pixeldat"], shader_dat.name)
setpar(xyz, ["resolutionw", "Resolutionw"], W)
setpar(xyz, ["resolutionh", "Resolutionh"], H)
# 32-bit float, or the XYZ we just computed is crushed back to 8 bits
setpar(xyz, ["format", "Format"], "rgba32float")
setpar(xyz, ["uniname0", "Uniname0"], "uIntrinsics")
for axis, val in zip("xyzw", INTRINSICS):
    setpar(xyz, ["unival0" + axis, "Unival0" + axis], val)

geo = parent_comp.create(geometryCOMP, "kinect_geo")
setpar(geo, ["instancing", "Instancing"], True)
setpar(geo, ["instanceop", "Instanceop"], xyz.path)
setpar(geo, ["numinstances", "Numinstances"], W * H)
for axis, chan in (("tx", "r"), ("ty", "g"), ("tz", "b")):
    setpar(geo, ["instance" + axis, "Instance" + axis], chan)

cam = parent_comp.create(cameraCOMP, "kinect_cam")
setpar(cam, ["tz", "Tz"], 2.0)
light = parent_comp.create(lightCOMP, "kinect_light")

render = parent_comp.create(renderTOP, "kinect_render")
setpar(render, ["camera", "Camera"], cam.path)
setpar(render, ["geometry", "Geometry"], geo.path)
setpar(render, ["lights", "Lights"], light.path)

for i, o in enumerate([syphon, shader_dat, xyz, geo, cam, light, render]):
    o.nodeX, o.nodeY = 0, -150 * i

print("=" * 60)
print("Created: kinect_syphon -> kinect_xyz -> kinect_geo -> kinect_render")
if missing:
    print("\\nSet these by hand (parameter names vary by TD version):")
    for m in missing:
        print("   " + m)
    print("\\nTip: click the operator, then hover a parameter to see its name.")
else:
    print("All parameters set. Look at kinect_render.")
print("=" * 60)

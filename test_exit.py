"""The app process must actually exit when asked - with tracking running.

Regression test for a real bug: MediaPipe runs non-daemon dispatcher threads,
and after the window closed one of them kept the process alive. Kinect.app's
launcher waits on that process, so its PID file stayed and the next launch
reported the camera as already in use.

Runs the real app, as a separate process, the way the launchers do.
    .venv/bin/python test_exit.py        # needs the GPU: normal terminal
"""
import os, signal, socket, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
PORT = 9131
fails = 0
def check(name, ok, detail=""):
    global fails
    print("  %s  %s%s" % ("PASS" if ok else "FAIL", name, ("  (%s)" % detail) if detail else ""))
    fails += 0 if ok else 1

for sig_name, sig in (("SIGTERM", signal.SIGTERM), ("SIGINT (Ctrl-C)", signal.SIGINT)):
    print("\n[%s with tracking running]" % sig_name)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", PORT)); rx.settimeout(0.5)
    log = open("/tmp/test_exit_app.log", "w")
    p = subprocess.Popen([os.path.join(HERE, ".venv/bin/python"), os.path.join(HERE, os.environ.get("APP", "kinect_app.py")),
                          "--camera", "synthetic", "--image",
                          os.path.join(HERE, "models/test/pose_640x576.jpg"),
                          "--track", "both", "--osc-port", str(PORT)],
                         stdout=log, stderr=subprocess.STDOUT, cwd=HERE)
    got, end = 0, time.time() + 30
    while time.time() < end and got < 60:
        try: rx.recvfrom(65535); got += 1
        except socket.timeout: pass
    check("tracking was live (OSC arriving)", got >= 60, "%d datagrams" % got)
    t0 = time.time(); p.send_signal(sig)
    try:
        code = p.wait(timeout=20); took = time.time() - t0
    except subprocess.TimeoutExpired:
        code, took = None, None
        p.kill(); p.wait()
    check("process exits", code is not None,
          "gone in %.1f s" % took if took is not None else "STILL RUNNING after 20 s - killed")
    check("exit code 0", code == 0, "code=%s" % code)
    rx.close(); log.close()
    time.sleep(1.0)

print("\n%s" % ("ALL PASSED" if not fails else "%d FAILED" % fails))
sys.exit(1 if fails else 0)

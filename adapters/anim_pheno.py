#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Loop a real recorded animation into the native polyscope viewer (anny_demo) over OSC, so a chosen
# ANNY phenotype performs real AddBiomechanics motion on repeat.  dump_anim.py produces the frames
# (clean bone world rotations per /sinew channel from a B3D); this only streams them — no procedural
# posing (hand-authored joint motion reads uncanny).
#   1. viewer, phenotype pinned:  SOMA_IDENT=<11 floats> ./anny_demo
#   2. stream the looping motion:  python viz_native/anim_pheno.py [frames.json]
# anny_demo treats /sinew/tracker quaternions as bone WORLD rotations (passthrough), which is exactly
# what dump_anim emits.
import socket, struct, time, json, sys, os

ADDR = ("127.0.0.1", 39539)
FPS = 30.0
# Default to the committed test walk (Falisse2017 subject_3, real overground gait); override with a
# path argument to play any dump_anim.py output.
PATH = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_walk.json")

# Phenotypes to pin on the viewer side (SOMA_IDENT=...).  obese_demo (BMI ~31.7, from
# generate_humans.py) is a body type absent from the real calibration data.
PRESETS = {
    "default": [0.5, 0.7, 0.5, 0.5, 0.6, 0, 0, 0, 0, 0, 0],
    "obese_demo": [0.0027, 0.8574, 0.0336, 0.7297, 0.1757, 0.8632, 0.5415, 0.2997, 0.4227, 0.0283, 0.1243],
}


def _pad(b):  # OSC: pad to a multiple of 4, adding nothing when already aligned
    return b + b"\x00" * ((4 - len(b) % 4) % 4)


def osc(name, q, conf=1.0):  # /sinew/tracker ,sffffd  (big-endian)
    msg = _pad(b"/sinew/tracker\x00") + _pad(b",sffffd\x00") + _pad(name.encode() + b"\x00")
    msg += struct.pack(">ffff", float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    msg += struct.pack(">d", conf)
    return msg


def osc_root(p):  # /sinew/root ,fff  — hip translation delta (m)
    return _pad(b"/sinew/root\x00") + _pad(b",fff\x00") + struct.pack(">fff", *(float(x) for x in p))


def main():
    data = json.load(open(PATH))
    dt = data["dt"] if isinstance(data, dict) else 1.0 / FPS  # trial's native seconds/frame
    frames = data["frames"] if isinstance(data, dict) else data
    n = len(frames)
    print(f"real animation: {n} frames, dt={dt:.4f}s ({1/dt:.0f} Hz, {n*dt:.1f}s) from {PATH}", flush=True)
    print("pin a phenotype on the viewer, e.g.  SOMA_IDENT=%s ./anny_demo" %
          ",".join(map(str, PRESETS["obese_demo"])), flush=True)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t0 = time.time()
    while True:
        # index by wall-clock so velocity is real-time regardless of send/render rate
        fr = frames[int((time.time() - t0) / dt) % n]
        for ch, q in fr.items():
            if ch == "_root":
                s.sendto(osc_root(q), ADDR)
            else:
                s.sendto(osc(ch, q), ADDR)
        time.sleep(1.0 / 60.0)  # send at ~60 Hz, picking the time-correct frame


if __name__ == "__main__":
    main()

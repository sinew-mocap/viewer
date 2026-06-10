# Synthetic /sinew/tracker feed for testing anny_demo without the dongle.
# Sends the same OSC wire format the C driver emits (big-endian float32 quats)
# to 127.0.0.1:39539, animating the shoulders so the body visibly moves.
#   python viz_native/osc_test_sender.py
import math
import socket
import struct
import time

PORT = 39539


def osc_string(s: str) -> bytes:
    b = s.encode() + b"\0"
    return b + b"\0" * (-len(b) % 4)


def tracker(name: str, q) -> bytes:
    msg = osc_string("/sinew/tracker") + osc_string(",sffffd") + osc_string(name)
    msg += b"".join(struct.pack(">f", c) for c in q)  # big-endian float32, OSC order
    msg += struct.pack(">d", time.monotonic())
    return msg


def qx(a):  # rotation by angle a about X
    return (math.cos(a / 2), math.sin(a / 2), 0.0, 0.0)


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(f"sending synthetic trackers to 127.0.0.1:{PORT} (Ctrl-C to stop)")
# Only the upper arms are sent; everything below them (forearm, hand) is left
# untracked so it rides along rigidly.  A coherent feed — no joint contradicts a
# neighbour, so the skin never pinches.  Real trackers report every segment, also
# coherently.
t0 = time.monotonic()
while True:
    a = 0.8 * math.sin((time.monotonic() - t0) * 1.5)
    for nm in ("LeftUpperArm", "RightUpperArm"):
        sock.sendto(tracker(nm, qx(a)), ("127.0.0.1", PORT))
    time.sleep(1 / 60)

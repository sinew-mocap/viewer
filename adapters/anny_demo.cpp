// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// Drive the real ANNY body (embedded SoA rig, soma_rig.h) from the live sinew
// tracker feed and show it in polyscope.  A UDP/OSC listener on port 39539 reads
// the driver's `/sinew/tracker` quaternions; FK over the joint hierarchy + LBS
// over each vertex's K influences re-skins the 18056-vertex mesh every frame.  The
// viser UI is ported to a polyscope ImGui panel.  No Python, no ONNX runtime.
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using sock_t = SOCKET;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using sock_t = int;
#endif

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include "imgui.h"
#include "polyscope/curve_network.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "soma_pheno_api.h"
#include "vk_lbs_host.h"
#include "soma_rig.h"
#include "channel_map.h"  // viewer/core: the neutral /sinew channel -> SOMA name map
#include "tracker_source.h"    // /sinew OSC-in port (39539)
#include "vr_source.h"         // /vr OSC-in port (39541)
#include "vr_tracker_sink.h"   // 6DOF OSC-out port (39542)
#include "pose_sink.h"         // solved-body render port (the PoseSink the solve drives)

// ── small 4x4 row-major math ─────────────────────────────────────────────────
static void mul4(const float *A, const float *B, float *C) {
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			float s = 0;
			for (int k = 0; k < 4; k++) {
				s += A[r * 4 + k] * B[k * 4 + c];
			}
			C[r * 4 + c] = s;
		}
	}
}
static void rigid_inv(const float *M, float *O) {  // (R|t) -> (Rᵀ | -Rᵀt)
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			O[r * 4 + c] = M[c * 4 + r];
		}
	}
	for (int r = 0; r < 3; r++) {
		O[r * 4 + 3] = -(O[r * 4] * M[3] + O[r * 4 + 1] * M[7] + O[r * 4 + 2] * M[11]);
	}
	O[12] = O[13] = O[14] = 0;
	O[15] = 1;
}
static void quatMat(const float *q, float *M) {  // q = (w,x,y,z) -> 4x4 rotation
	float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (n < 1e-9f) {
		float I[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
		memcpy(M, I, sizeof I);
		return;
	}
	float w = q[0] / n, x = q[1] / n, y = q[2] / n, z = q[3] / n;
	float R[16] = {1 - 2 * (y * y + z * z),
	               2 * (x * y - z * w),
	               2 * (x * z + y * w),
	               0,
	               2 * (x * y + z * w),
	               1 - 2 * (x * x + z * z),
	               2 * (y * z - x * w),
	               0,
	               2 * (x * z - y * w),
	               2 * (y * z + x * w),
	               1 - 2 * (x * x + y * y),
	               0,
	               0,
	               0,
	               0,
	               1};
	memcpy(M, R, sizeof R);
}
static int jointByName(const char *name) {
	for (int j = 0; j < SOMA_J; j++) {
		if (strcmp(soma_jointNames[j], name) == 0) {
			return j;
		}
	}
	return -1;
}

// ── tracker pose state (written by the OSC thread, read by the renderer) ──────
static std::mutex g_mu;
static float g_curQ[SOMA_J][4];   // latest quat per joint (w,x,y,z)
static bool g_tracked[SOMA_J];    // a tracker has been seen for this joint
static std::atomic<int> g_count;  // distinct joints seen (for the UI)
static float g_rootDelta[3];      // hip translation delta (m) from /sinew/root — root motion
// Live-driver calibration (non-passthrough): a hips-referenced sensor-world -> model-world align G
// and per-joint sensor->bone offsets, so a knee-flex delta rotates about the anatomical axis.
//   bone_world = G · R_live · off,   off = R_calᵀ · Gᵀ · REST_W,   G = REST_W[Hips] · R_hipsᵀ
static float g_G[16];
static float g_calOff[SOMA_J * 16];
static bool g_haveCalib = false;
static std::atomic<int> g_calCountdown{0};  // T-pose countdown (seconds; 0 = idle)
static float g_jointPos[SOMA_J * 3];        // per-joint world position (m), filled by skin()
static float g_jointRot[SOMA_J * 9];        // per-joint world rotation (row-major 3x3), filled by skin()
static float g_rootOffset[3];               // applied root translation this frame (foot-lock/HMD/streamed)
// Foot-lock root-motion state (planted lower foot's XZ stays anchored as the body steps).
static int g_planted = -1;
static float g_anchorX = 0, g_anchorZ = 0, g_ox = 0, g_oz = 0;
static float g_prevHipX = 1e9f, g_prevHipZ = 1e9f;  // detect a pose teleport (clip loop) to reset root
// SteamVR layer (in: /vr/* on UDP 39541 from hmd_reader; out: 6DOF on 39542 to driver_sinew).
static float g_hmdPos[3];
static float g_hmdQuat[4] = {1, 0, 0, 0};
static bool g_haveHmd = false;
static float g_handPos[2][3];          // 0 = left, 1 = right controller (OpenVR m)
static float g_handQuat[2][4] = {{1, 0, 0, 0}, {1, 0, 0, 0}};
static bool g_haveHand[2] = {false, false};
struct VrDev {
	int cls;
	float pos[3], quat[4];
};
static std::map<int, VrDev> g_vrDev;   // all SteamVR devices, by index (for gizmos)
static float g_restJointPos[SOMA_J * 3];  // rest FK joint positions — VR-IK anchors
static float g_ikWorld[SOMA_J * 16];      // VR-IK-solved arm world rotations
static bool g_ikActive[SOMA_J] = {false};
static std::atomic<bool> g_driveVr{false};  // feed 15 virtual trackers to driver_sinew

// play_log streams already-calibrated bone *world* orientations (calibration is
// solved offline in Lean), so the viewer applies the incoming quat directly as the
// bone world rotation instead of treating it as a raw sensor reading.
static bool g_passthrough = true;

// Channel->SOMA-name mapping is the neutral viewer/core (channel_map); this
// adapter just resolves the SOMA name to a rig index.
static int somaIndexForChannel(const char *channel) {
	const char *soma = soma_for_channel(channel);
	return soma ? jointByName(soma) : -1;
}

static float bef(const uint8_t *p) {  // big-endian float32 (OSC wire order)
	uint8_t b[4] = {p[3], p[2], p[1], p[0]};
	float f;
	memcpy(&f, b, 4);
	return f;
}
static int pad4(int n) {
	return (n + 3) & ~3;
}
static int bei(const uint8_t *p) {  // big-endian int32 (OSC wire order)
	return (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
}

// Parse one `/sinew/tracker ,sffffd` message: name + (qw,qx,qy,qz).
static void parseTracker(const uint8_t *buf, int len) {
	if (len < 16 || memcmp(buf, "/sinew/tracker", 14) != 0) {
		return;
	}
	int o = pad4((int)strlen((const char *)buf) + 1);  // past address
	if (o + 8 > len || buf[o] != ',') {
		return;
	}
	o = pad4(o + (int)strlen((const char *)(buf + o)) + 1);  // past type tag ",sffffd"
	const char *name = (const char *)(buf + o);
	o = pad4(o + (int)strlen(name) + 1);  // past the string arg
	if (o + 16 > len) {
		return;
	}
	int idx = somaIndexForChannel(name);
	if (idx < 0) {
		return;
	}
	float q[4] = {bef(buf + o), bef(buf + o + 4), bef(buf + o + 8), bef(buf + o + 12)};
	std::lock_guard<std::mutex> lk(g_mu);
	if (!g_tracked[idx]) {
		g_count++;
	}
	g_tracked[idx] = true;
	memcpy(g_curQ[idx], q, sizeof q);
}

// Parse one `/sinew/root ,fff` message: the hip translation delta (m) for root motion.
static void parseRoot(const uint8_t *buf, int len) {
	int o = pad4((int)strlen((const char *)buf) + 1);  // past "/sinew/root"
	if (o + 8 > len || buf[o] != ',') {
		return;
	}
	o = pad4(o + (int)strlen((const char *)(buf + o)) + 1);  // past ",fff"
	if (o + 12 > len) {
		return;
	}
	float p[3] = {bef(buf + o), bef(buf + o + 4), bef(buf + o + 8)};
	std::lock_guard<std::mutex> lk(g_mu);
	memcpy(g_rootDelta, p, sizeof p);
}

// Parse one `/sinew/pheno ,f×11` message: a new 11-dim ANNY identity → live phenotype switch.
// The OSC thread only stashes it; the render thread re-runs applyPheno() (touches GPU bind buffers).
static float g_identNext[11];
static std::atomic<bool> g_phenoUpdate{false};
static void parsePheno(const uint8_t *buf, int len) {
	int o = pad4((int)strlen((const char *)buf) + 1);  // past "/sinew/pheno"
	if (o + 13 > len || buf[o] != ',') {
		return;
	}
	o = pad4(o + (int)strlen((const char *)(buf + o)) + 1);  // past ",f…f"
	if (o + 44 > len) {  // 11 floats
		return;
	}
	std::lock_guard<std::mutex> lk(g_mu);
	for (int i = 0; i < 11; i++) {
		g_identNext[i] = bef(buf + o + 4 * i);
	}
	g_phenoUpdate.store(true);
}

// ── Port adapters: the three UDP boundaries as source/sink ports ─────────────
// Thin wrappers over the sockets so each boundary reads as a declared port; the
// parse/solve logic is unchanged.  UdpSourceCtx backs both TrackerSource
// (/sinew 39539) and VrSource (/vr 39541) — same recvfrom shape.
struct UdpSourceCtx {
	sock_t s;
};
static int udp_source_next(void *ctx, uint8_t *osc, size_t cap, size_t *len) {
	UdpSourceCtx *c = (UdpSourceCtx *)ctx;
	int n = recvfrom(c->s, (char *)osc, (int)cap, 0, nullptr, nullptr);
	if (n > 0) {
		*len = (size_t)n;
		return 1;
	}
	return n == 0 ? 0 : -1;
}
static void udp_source_close(void *) {
}

// VrTrackerSink: the 6DOF body-out to driver_sinew (one 29-byte datagram per
// tracker: wire index + position(12) + quaternion(16)).
struct VrTrackerSinkCtx {
	sock_t s;
	sockaddr_in dst;
};
static void vr_tracker_send(void *ctx, int joint, const float pos[3], const float quat[4]) {
	VrTrackerSinkCtx *c = (VrTrackerSinkCtx *)ctx;
	unsigned char pkt[29];
	pkt[0] = (unsigned char)joint;
	memcpy(pkt + 1, pos, 12);
	memcpy(pkt + 13, quat, 16);
	sendto(c->s, (const char *)pkt, 29, 0, (sockaddr *)&c->dst, sizeof c->dst);
}
static void vr_tracker_close(void *) {
}

static void oscThread() {
#ifdef _WIN32
	WSADATA wsa;
	WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
	sock_t s = socket(AF_INET, SOCK_DGRAM, 0);
	sockaddr_in a{};
	a.sin_family = AF_INET;
	a.sin_port = htons(39539);
	a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if (bind(s, (sockaddr *)&a, sizeof a) != 0) {
		std::fprintf(stderr, "anny_demo: cannot bind UDP 39539 (driver already on it?)\n");
		return;
	}
	UdpSourceCtx srcctx{s};
	TrackerSource src{&srcctx, udp_source_next, udp_source_close};
	uint8_t buf[2048];
	for (;;) {
		size_t len = 0;
		if (src.next(src.ctx, buf, sizeof buf, &len) == 1) {
			int n = (int)len;
			if (n >= 11 && memcmp(buf, "/sinew/root", 11) == 0) {
				parseRoot(buf, n);
			} else if (n >= 12 && memcmp(buf, "/sinew/pheno", 12) == 0) {
				parsePheno(buf, n);
			} else {
				parseTracker(buf, n);
			}
		}
	}
}

// ── geometry / skinning ──────────────────────────────────────────────────────
static std::vector<float> g_restLocal(SOMA_J * 16);

// Phenotype-driven body, recomputed from the 11-dim ident by the native SOMA port
// (soma_pheno.c).  Seeded from the baked rig so the viewer still runs if the asset
// is absent.  skin() reads these instead of the static soma_rig.h arrays.
static std::vector<float> g_bindV(SOMA_V * 3);     // current bind / rest vertices
static std::vector<float> g_invbind(SOMA_J * 16);  // inverse-bind per joint
static std::vector<float> g_bwP(SOMA_J * 16);      // current bind world transforms
static float g_ident[11] = {0.5f, 0.7f, 0.5f, 0.5f, 0.6f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
static bool g_havePheno = false;
static bool g_useVk = false;         // per-frame LBS runs on the GPU (Vulkan)?
static std::vector<float> g_denseW;  // dense V*J skinning weights for the lbs kernel

// Recompute bind verts + skeleton (and rest-local for FK) from g_ident.
static void applyPheno() {
	pheno_eval(g_ident, g_bindV.data(), g_bwP.data());
	for (int j = 0; j < SOMA_J; j++) {
		rigid_inv(&g_bwP[j * 16], &g_invbind[j * 16]);  // invbind = SE3 inverse(bind world)
	}
	for (int j = 0; j < SOMA_J; j++) {
		int p = soma_parents[j];
		if (p < 0 || p == j) {
			memcpy(&g_restLocal[j * 16], &g_bwP[j * 16], 64);
		} else {
			float inv[16];
			rigid_inv(&g_bwP[p * 16], inv);
			mul4(inv, &g_bwP[j * 16], &g_restLocal[j * 16]);
		}
	}
	if (g_useVk) {
		vk_lbs_set_bind(g_bindV.data());  // reshaped bind → GPU
	}
}

// ── SteamVR math + VR IK (ported from live_body.py) ──────────────────────────
static void matToQuat(const float *m, float *q) {  // row-major 3x3 (9) -> (w,x,y,z)
	float t = m[0] + m[4] + m[8];
	if (t > 0) {
		float s = std::sqrt(t + 1.f) * 2;
		q[0] = 0.25f * s; q[1] = (m[7] - m[5]) / s; q[2] = (m[2] - m[6]) / s; q[3] = (m[3] - m[1]) / s;
	} else if (m[0] > m[4] && m[0] > m[8]) {
		float s = std::sqrt(1.f + m[0] - m[4] - m[8]) * 2;
		q[0] = (m[7] - m[5]) / s; q[1] = 0.25f * s; q[2] = (m[1] + m[3]) / s; q[3] = (m[2] + m[6]) / s;
	} else if (m[4] > m[8]) {
		float s = std::sqrt(1.f + m[4] - m[0] - m[8]) * 2;
		q[0] = (m[2] - m[6]) / s; q[1] = (m[1] + m[3]) / s; q[2] = 0.25f * s; q[3] = (m[5] + m[7]) / s;
	} else {
		float s = std::sqrt(1.f + m[8] - m[0] - m[4]) * 2;
		q[0] = (m[3] - m[1]) / s; q[1] = (m[2] + m[6]) / s; q[2] = (m[5] + m[7]) / s; q[3] = 0.25f * s;
	}
}
static void v3sub(const float *a, const float *b, float *o) {
	o[0] = a[0] - b[0]; o[1] = a[1] - b[1]; o[2] = a[2] - b[2];
}
static float v3len(const float *a) { return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }
static void v3cross(const float *a, const float *b, float *o) {
	o[0] = a[1] * b[2] - a[2] * b[1]; o[1] = a[2] * b[0] - a[0] * b[2]; o[2] = a[0] * b[1] - a[1] * b[0];
}
static float v3dot(const float *a, const float *b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

static void rotAxis(const float *ax, float ang, float *M) {  // axis*angle -> 4x4 rotation (transl 0)
	float n = v3len(ax) + 1e-9f, x = ax[0] / n, y = ax[1] / n, z = ax[2] / n;
	float c = std::cos(ang), s = std::sin(ang), C = 1 - c;
	float r[16] = {c + x * x * C, x * y * C - z * s, x * z * C + y * s, 0,
	               y * x * C + z * s, c + y * y * C, y * z * C - x * s, 0,
	               z * x * C - y * s, z * y * C + x * s, c + z * z * C, 0, 0, 0, 0, 1};
	memcpy(M, r, sizeof r);
}
static void shortestArc(const float *a, const float *b, float *M) {  // rot unit(a)->unit(b) -> 4x4
	float an[3] = {a[0], a[1], a[2]}, bn[3] = {b[0], b[1], b[2]};
	float la = v3len(an) + 1e-9f, lb = v3len(bn) + 1e-9f;
	for (int i = 0; i < 3; i++) { an[i] /= la; bn[i] /= lb; }
	float d = v3dot(an, bn);
	if (d > 0.99999f) {
		float I[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
		memcpy(M, I, sizeof I);
		return;
	}
	float ax[3];
	if (d < -0.99999f) {
		float t[3] = {1, 0, 0};
		v3cross(an, t, ax);
		if (v3len(ax) < 1e-6f) { float u[3] = {0, 1, 0}; v3cross(an, u, ax); }
		rotAxis(ax, 3.14159265f, M);
		return;
	}
	v3cross(an, bn, ax);
	rotAxis(ax, std::acos(d < -1 ? -1 : (d > 1 ? 1 : d)), M);
}
static void twoBoneElbow(const float *S, const float *T, float Lu, float Lf, const float *pole, float *E) {
	float to[3];
	v3sub(T, S, to);
	float d = v3len(to);
	if (d < 1e-6f) { E[0] = S[0]; E[1] = S[1] - Lu; E[2] = S[2]; return; }
	float u[3] = {to[0] / d, to[1] / d, to[2] / d};
	float lo = std::fabs(Lu - Lf) + 1e-3f, hi = Lu + Lf - 1e-3f;
	d = d < lo ? lo : (d > hi ? hi : d);
	float cosA = (Lu * Lu + d * d - Lf * Lf) / (2 * Lu * d);
	cosA = cosA < -1 ? -1 : (cosA > 1 ? 1 : cosA);
	float bax[3];
	v3cross(u, pole, bax);
	if (v3len(bax) < 1e-6f) { float z[3] = {0, 0, 1}; v3cross(u, z, bax); }
	float R[16];
	rotAxis(bax, std::acos(cosA), R);
	float ru[3] = {R[0] * u[0] + R[1] * u[1] + R[2] * u[2], R[4] * u[0] + R[5] * u[1] + R[6] * u[2],
	               R[8] * u[0] + R[9] * u[1] + R[10] * u[2]};
	E[0] = S[0] + Lu * ru[0]; E[1] = S[1] + Lu * ru[1]; E[2] = S[2] + Lu * ru[2];
}

// Parse one `/vr/*` message (big-endian OSC): /vr/hmd, /vr/hand/{left,right} (,fffffff) and
// /vr/device (,iifffffff).  Formats per steamvr/hmd_reader.cpp.
static void parseVr(const uint8_t *buf, int len) {
	if (len < 8) {
		return;
	}
	const char *addr = (const char *)buf;
	int o = pad4((int)strlen(addr) + 1);
	if (o + 4 > len || buf[o] != ',') {
		return;
	}
	o = pad4(o + (int)strlen((const char *)(buf + o)) + 1);  // past the type tag
	std::lock_guard<std::mutex> lk(g_mu);
	if (strcmp(addr, "/vr/device") == 0) {
		if (o + 36 > len) {
			return;
		}
		int idx = bei(buf + o), cls = bei(buf + o + 4);
		VrDev d{cls, {bef(buf + o + 8), bef(buf + o + 12), bef(buf + o + 16)},
		        {bef(buf + o + 20), bef(buf + o + 24), bef(buf + o + 28), bef(buf + o + 32)}};
		g_vrDev[idx] = d;
		return;
	}
	if (o + 28 > len) {
		return;
	}
	float p[3] = {bef(buf + o), bef(buf + o + 4), bef(buf + o + 8)};
	float q[4] = {bef(buf + o + 12), bef(buf + o + 16), bef(buf + o + 20), bef(buf + o + 24)};
	if (strcmp(addr, "/vr/hmd") == 0) {
		memcpy(g_hmdPos, p, sizeof p);
		memcpy(g_hmdQuat, q, sizeof q);
		g_haveHmd = true;
	} else if (strcmp(addr, "/vr/hand/left") == 0) {
		memcpy(g_handPos[0], p, sizeof p);
		memcpy(g_handQuat[0], q, sizeof q);
		g_haveHand[0] = true;
	} else if (strcmp(addr, "/vr/hand/right") == 0) {
		memcpy(g_handPos[1], p, sizeof p);
		memcpy(g_handQuat[1], q, sizeof q);
		g_haveHand[1] = true;
	}
}

static void vrOscThread() {  // second listener: /vr/* on UDP 39541 (SteamVR via hmd_reader)
	sock_t s = socket(AF_INET, SOCK_DGRAM, 0);
	sockaddr_in a{};
	a.sin_family = AF_INET;
	a.sin_port = htons(39541);
	a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if (bind(s, (sockaddr *)&a, sizeof a) != 0) {
		std::fprintf(stderr, "anny_demo: cannot bind UDP 39541 (hmd_reader port)\n");
		return;
	}
	UdpSourceCtx srcctx{s};
	VrSource src{&srcctx, udp_source_next, udp_source_close};
	uint8_t buf[2048];
	for (;;) {
		size_t len = 0;
		if (src.next(src.ctx, buf, sizeof buf, &len) == 1) {
			int n = (int)len;
			if (n >= 4 && memcmp(buf, "/vr/", 4) == 0) {
				parseVr(buf, n);
			}
		}
	}
}

// Per-joint world transform (rotation + chained position, root at rest) from the live trackers.
// Pure FK (no LBS); both skin() and the 62.5 Hz VR feed call it, so the feed computes its own pose
// and stays in sync regardless of the polyscope render rate.
static void computeWorld(const float snap[SOMA_J][4], const bool tr[SOMA_J], float world[SOMA_J * 16]) {
	float Wr[SOMA_J * 16];  // per-joint world rotation (translation 0)
	for (int j = 0; j < SOMA_J; j++) {
		int p = soma_parents[j];
		bool root = (p < 0 || p == j);
		float RW[16];
		memcpy(RW, &g_bwP[j * 16], 64);
		RW[3] = RW[7] = RW[11] = 0;
		float Dr[16];
		if (g_ikActive[j]) {  // VR IK drives this (unsensored) arm bone
			memcpy(Dr, &g_ikWorld[j * 16], 64);
			Dr[3] = Dr[7] = Dr[11] = 0;
		} else if (tr[j] && g_passthrough) {
			quatMat(snap[j], Dr);  // play_log streams the calibrated bone world orientation
			Dr[3] = Dr[7] = Dr[11] = 0;
		} else if (tr[j] && g_haveCalib) {
			float R[16], tmp[16];
			quatMat(snap[j], R);
			R[3] = R[7] = R[11] = 0;
			mul4(R, &g_calOff[j * 16], tmp);
			mul4(g_G, tmp, Dr);  // bone_world = G · R_live · off
		} else if (root) {
			memcpy(Dr, RW, 64);
		} else {
			float RL[16];  // untracked: follow rest, relative to the parent
			memcpy(RL, &g_restLocal[j * 16], 64);
			RL[3] = RL[7] = RL[11] = 0;
			mul4(&Wr[p * 16], RL, Dr);
		}
		float local[16];
		if (root) {
			memcpy(local, Dr, 64);
			local[3] = g_bwP[j * 16 + 3];   // root at rest; translation applied to verts/feed later
			local[7] = g_bwP[j * 16 + 7];
			local[11] = g_bwP[j * 16 + 11];
			memcpy(&world[j * 16], local, 64);
		} else {
			float WpT[16], localRot[16];
			rigid_inv(&Wr[p * 16], WpT);
			mul4(WpT, Dr, localRot);
			memcpy(local, localRot, 64);
			local[3] = g_restLocal[j * 16 + 3];
			local[7] = g_restLocal[j * 16 + 7];
			local[11] = g_restLocal[j * 16 + 11];
			mul4(&world[p * 16], local, &world[j * 16]);
		}
		memcpy(&Wr[j * 16], Dr, 64);
	}
}

// Feed the body's FK pose to driver_sinew as 15 Vive-tracker 6DOF updates at 62.5 Hz (the 29-byte
// LE packet of driver_sinew.cpp).  kMap order == driver_sinew's kJointNames / live_body VR_NODE_ORDER.
static void driveVrThread() {
	sock_t s = socket(AF_INET, SOCK_DGRAM, 0);
	sockaddr_in dst{};
	dst.sin_family = AF_INET;
	dst.sin_port = htons(39542);
	dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	VrTrackerSinkCtx sinkctx{s, dst};
	VrTrackerSink sink{&sinkctx, vr_tracker_send, vr_tracker_close};
	int nt = CHANNEL_MAP_COUNT;
	std::vector<int> idx(nt);
	for (int i = 0; i < nt; i++) {
		idx[i] = jointByName(channel_map_soma(i));
	}
	static float world[SOMA_J * 16];
	for (;;) {
		if (g_driveVr.load()) {
			float snap[SOMA_J][4];
			bool tr[SOMA_J];
			{
				std::lock_guard<std::mutex> lk(g_mu);
				memcpy(snap, g_curQ, sizeof snap);
				memcpy(tr, g_tracked, sizeof tr);
			}
			computeWorld(snap, tr, world);  // own FK -> render-rate-independent, no desync
			for (int i = 0; i < nt && i < 255; i++) {
				int j = idx[i];
				if (j < 0) {
					continue;
				}
				const float *m = &world[j * 16];
				float p[3] = {m[3] + g_rootOffset[0], m[7] + g_rootOffset[1], m[11] + g_rootOffset[2]};
				float rr[9] = {m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]}, q[4];
				matToQuat(rr, q);
				sink.send(sink.ctx, i, p, q);  // wire index i -> driver_sinew
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(16));  // 62.5 Hz
	}
}

// T-pose capture: solve the hips-referenced G and per-joint offsets from the current trackers
// (ported from live_body.py's _countdown_capture).  Off-axis sensors stay uncalibrated.
static void captureCalibration() {
	int hips = jointByName("Hips");
	std::lock_guard<std::mutex> lk(g_mu);
	if (hips < 0 || !g_tracked[hips]) {
		return;
	}
	float Rh[16], RhT[16], RWh[16];
	quatMat(g_curQ[hips], Rh);
	Rh[3] = Rh[7] = Rh[11] = 0;
	rigid_inv(Rh, RhT);                       // rotation-only -> transpose
	memcpy(RWh, &g_bwP[hips * 16], 64);
	RWh[3] = RWh[7] = RWh[11] = 0;
	mul4(RWh, RhT, g_G);                       // G = REST_W[Hips] · R_hipsᵀ
	float GT[16];
	rigid_inv(g_G, GT);
	for (int j = 0; j < SOMA_J; j++) {
		if (!g_tracked[j]) {
			continue;
		}
		float Rj[16], RjT[16], RWj[16], tmp[16];
		quatMat(g_curQ[j], Rj);
		Rj[3] = Rj[7] = Rj[11] = 0;
		rigid_inv(Rj, RjT);
		memcpy(RWj, &g_bwP[j * 16], 64);
		RWj[3] = RWj[7] = RWj[11] = 0;
		mul4(GT, RWj, tmp);                    // Gᵀ · REST_W
		mul4(RjT, tmp, &g_calOff[j * 16]);     // off = R_calᵀ · Gᵀ · REST_W
	}
	g_haveCalib = true;
}

// Skin into `verts` from the live trackers only — no canned animation (the viewer's sole job is to
// display the tracker-driven body; with no trackers it holds the rest pose).
static void skin(std::vector<std::array<float, 3>> &verts) {
	static float world[SOMA_J * 16], bone[SOMA_J * 16];
	float snap[SOMA_J][4];
	bool tr[SOMA_J];
	{
		std::lock_guard<std::mutex> lk(g_mu);
		memcpy(snap, g_curQ, sizeof snap);
		memcpy(tr, g_tracked, sizeof tr);
	}
	computeWorld(snap, tr, world);
	for (int j = 0; j < SOMA_J; j++) {  // bone = world·invbind for LBS; export pose for gizmos
		mul4(&world[j * 16], &g_invbind[j * 16], &bone[j * 16]);
		g_jointPos[j * 3] = world[j * 16 + 3];
		g_jointPos[j * 3 + 1] = world[j * 16 + 7];
		g_jointPos[j * 3 + 2] = world[j * 16 + 11];
		const float *m = &world[j * 16];
		float r[9] = {m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]};
		memcpy(&g_jointRot[j * 9], r, sizeof r);
	}
	if (g_useVk) {  // GPU: dispatch the lbs Slang kernel (Vulkan), read verts back
		vk_lbs_dispatch(bone, &verts[0][0]);
		return;
	}
	for (int v = 0; v < SOMA_V; v++) {  // CPU fallback (no Vulkan device)
		const float *b = &g_bindV[v * 3];
		float ax = 0, ay = 0, az = 0;
		for (int k = 0; k < SOMA_K; k++) {
			float w = soma_wval[v * SOMA_K + k];
			if (w == 0) {
				continue;
			}
			const float *m = &bone[soma_widx[v * SOMA_K + k] * 16];
			ax += w * (m[0] * b[0] + m[1] * b[1] + m[2] * b[2] + m[3]);
			ay += w * (m[4] * b[0] + m[5] * b[1] + m[6] * b[2] + m[7]);
			az += w * (m[8] * b[0] + m[9] * b[1] + m[10] * b[2] + m[11]);
		}
		verts[v] = {ax, ay, az};
	}
}

// ── PoseSink adapter: render a solved frame in polyscope ─────────────────────
// The solve emits the neutral per-joint world poses here each frame; this
// adapter presents the skinned body (the frame's deformed verts).  Making the
// render flow through the port revives the previously-dead PoseSink and is the
// seam the doc's other sinks (SteamVR, VMC) can hang off the same solve pass.
struct PolyscopePoseSinkCtx {
	polyscope::SurfaceMesh *mesh;
	std::vector<std::array<float, 3>> *verts;
	const bool *showMesh;
};
static void polyscope_pose_emit(void *ctx, const SinewJointPose *joints, int n, double time_s) {
	(void)joints;  // the skeleton; the deformed mesh (verts) is this frame's solved body
	(void)n;
	(void)time_s;
	PolyscopePoseSinkCtx *c = (PolyscopePoseSinkCtx *)ctx;
	c->mesh->updateVertexPositions(*c->verts);
	c->mesh->setEnabled(*c->showMesh);
}
static void polyscope_pose_close(void *) {
}

int main() {
	polyscope::options::verbosity = 3;
	polyscope::options::programName = "sinew — ANNY body";
	polyscope::options::userGuiIsOnRightSide = false;   // panel on the left
	polyscope::options::buildDefaultGuiPanels = false;  // make it the primary left panel (far-left edge)
	polyscope::init();

	std::vector<std::array<uint32_t, 3>> faces(SOMA_F);
	for (int i = 0; i < SOMA_F; i++) {
		faces[i] = {(uint32_t)soma_faces[3 * i], (uint32_t)soma_faces[3 * i + 1],
		            (uint32_t)soma_faces[3 * i + 2]};
	}

	// Seed the body from the baked rig (fallback if the phenotype asset is absent).
	memcpy(g_bindV.data(), soma_bind, sizeof(float) * SOMA_V * 3);
	memcpy(g_invbind.data(), soma_invbind, sizeof(float) * SOMA_J * 16);
	memcpy(g_bwP.data(), soma_restWorld, sizeof(float) * SOMA_J * 16);
	for (int j = 0; j < SOMA_J; j++) {  // rest-local of each joint
		int p = soma_parents[j];
		if (p < 0 || p == j) {
			memcpy(&g_restLocal[j * 16], &g_bwP[j * 16], 16 * sizeof(float));
		} else {
			float inv[16];
			rigid_inv(&g_bwP[p * 16], inv);
			mul4(inv, &g_bwP[j * 16], &g_restLocal[j * 16]);
		}
	}
	// Load the phenotype asset; if present, recompute the body from g_ident.
	g_havePheno = (pheno_load("soma_pheno.bin") == 0);
	// A specific phenotype can be pinned at startup via SOMA_IDENT (11 comma-separated floats),
	// so a chosen body loads without touching the sliders (e.g. for a looped animation demo).
	if (const char *env = std::getenv("SOMA_IDENT")) {
		int i = 0;
		for (const char *p = env; *p && i < 11; i++) {
			g_ident[i] = (float)atof(p);
			const char *c = strchr(p, ',');
			if (!c) {
				break;
			}
			p = c + 1;
		}
		std::fprintf(stderr, "anny_demo: SOMA_IDENT pinned the phenotype\n");
	}
	if (g_havePheno) {
		applyPheno();
	} else {
		std::fprintf(stderr, "anny_demo: soma_pheno.bin not found — phenotype sliders disabled\n");
	}

	// Dense weights for the lbs kernel, then bring up the GPU path (CPU fallback).
	g_denseW.assign((size_t)SOMA_V * SOMA_J, 0.f);
	for (int v = 0; v < SOMA_V; v++) {
		for (int k = 0; k < SOMA_K; k++) {
			float w = soma_wval[v * SOMA_K + k];
			if (w != 0) {
				g_denseW[(size_t)v * SOMA_J + soma_widx[v * SOMA_K + k]] = w;
			}
		}
	}
	g_useVk = (vk_lbs_init("lbs.spv", g_bindV.data(), g_denseW.data(), SOMA_V, SOMA_J) == 0);
	std::fprintf(stderr, "anny_demo: per-frame LBS on %s\n", g_useVk ? "GPU (Vulkan)" : "CPU");

	for (int j = 0; j < SOMA_J; j++) {
		g_curQ[j][0] = 1.f;  // identity quats
	}

#ifdef _WIN32
	{
		WSADATA w;
		WSAStartup(MAKEWORD(2, 2), &w);  // once for all socket threads
	}
#endif
	if (std::getenv("SINEW_DRIVE_VR")) {
		g_driveVr.store(true);
	}
	std::thread(oscThread).detach();
	std::thread(vrOscThread).detach();   // /vr/* in (SteamVR)
	std::thread(driveVrThread).detach();  // 6DOF out to driver_sinew

	std::vector<std::array<float, 3>> verts(SOMA_V);
	skin(verts);  // rest pose (no trackers yet)
	memcpy(g_restJointPos, g_jointPos, sizeof g_restJointPos);  // rest FK joints for VR-IK anchors
	polyscope::SurfaceMesh *mesh = polyscope::registerSurfaceMesh("anny", verts, faces);

	// Tracker-axis gizmos: one curve network, 3 colored segments (X red / Y green / Z blue) per
	// tracker, drawn at each tracked joint with its bone-world orientation.
	const int NTRK = CHANNEL_MAP_COUNT;
	std::vector<int> trkIdx(NTRK);
	for (int i = 0; i < NTRK; i++) {
		trkIdx[i] = jointByName(channel_map_soma(i));
	}
	std::vector<std::array<float, 3>> axPts((size_t)NTRK * 6, {0.f, 0.f, 0.f});
	polyscope::CurveNetwork *axes = polyscope::registerCurveNetworkSegments("tracker axes", axPts);
	std::vector<std::array<float, 3>> axCol((size_t)NTRK * 3);
	for (int i = 0; i < NTRK; i++) {
		axCol[i * 3] = {1, 0, 0};
		axCol[i * 3 + 1] = {0, 1, 0};
		axCol[i * 3 + 2] = {0, 0, 1};
	}
	axes->addEdgeColorQuantity("axis", axCol)->setEnabled(true);
	axes->setRadius(0.0035f, false);

	// SteamVR device gizmos (HMD / controllers / base stations), drawn in the OpenVR frame.
	const int NDEV = 16;
	std::vector<std::array<float, 3>> devPts((size_t)NDEV * 6, {0.f, 0.f, 0.f});
	polyscope::CurveNetwork *devGiz = polyscope::registerCurveNetworkSegments("vr devices", devPts);
	std::vector<std::array<float, 3>> devCol((size_t)NDEV * 3);
	for (int i = 0; i < NDEV; i++) {
		devCol[i * 3] = {1, 0, 1};
		devCol[i * 3 + 1] = {1, 1, 0};
		devCol[i * 3 + 2] = {0, 1, 1};
	}
	devGiz->addEdgeColorQuantity("axis", devCol)->setEnabled(true);
	devGiz->setRadius(0.0035f, false);
	devGiz->setEnabled(false);

	// ── viewer UI (polyscope ImGui panel) ────────────────────────────────────
	bool rootFootLock = true, showAxes = true, showMesh = true, showDevices = true;
	bool hmdRoot = std::getenv("SINEW_HMD_ROOT") != nullptr;  // parity with live_body's SINEW_* envs
	bool vrIk = std::getenv("SINEW_VR_IK") != nullptr;

	// The solved frame is presented through the PoseSink port (polyscope adapter).
	PolyscopePoseSinkCtx posectx{mesh, &verts, &showMesh};
	PoseSink poseSink{&posectx, polyscope_pose_emit, polyscope_pose_close};

	polyscope::state::userCallback = [&]() {
		if (g_phenoUpdate.exchange(false)) {  // a /sinew/pheno message arrived → live phenotype switch
			{
				std::lock_guard<std::mutex> lk(g_mu);
				memcpy(g_ident, g_identNext, sizeof g_ident);
			}
			applyPheno();  // recompute bind mesh + skeleton from the new ident
		}
		int n = g_count.load();
		ImGui::Text("ANNY body — trackers: %d/15 (live OSC :39539)", n);
		ImGui::SeparatorText("Calibration");
		ImGui::Checkbox("Pre-calibrated stream (play_log)", &g_passthrough);
		ImGui::TextDisabled("play_log solves the per-tracker offset offline (Lean align_vectors)");
		if (!g_passthrough) {
			if (ImGui::Button("T-pose calibrate (3s)")) {
				std::thread([] {
					for (int s = 3; s > 0; s--) {
						g_calCountdown.store(s);
						std::this_thread::sleep_for(std::chrono::seconds(1));
					}
					g_calCountdown.store(0);
					captureCalibration();
				}).detach();
			}
			ImGui::SameLine();
			int cd = g_calCountdown.load();
			ImGui::TextDisabled(cd > 0 ? "hold T-pose..." : (g_haveCalib ? "calibrated (G)" : "raw sensor input"));
			if (g_haveCalib && ImGui::Button("clear calibration")) {
				g_haveCalib = false;
			}
		}
		ImGui::Checkbox("root motion (foot lock)", &rootFootLock);
		ImGui::Checkbox("show ANNY mesh", &showMesh);
		ImGui::Checkbox("show tracker axes", &showAxes);

		ImGui::SeparatorText("SteamVR");
		ImGui::Checkbox("HMD root (SteamVR)", &hmdRoot);
		ImGui::Checkbox("VR IK (HMD + controllers)", &vrIk);
		ImGui::Checkbox("show VR devices (HMD/controllers/base)", &showDevices);
		bool dv = g_driveVr.load();
		if (ImGui::Checkbox("drive SteamVR trackers", &dv)) {
			g_driveVr.store(dv);
		}

		if (ImGui::CollapsingHeader("Phenotype")) {
			if (g_havePheno) {
				static const char *pn[11] = {"gender",  "age",         "muscle",   "weight",
				                             "height",  "proportions", "cupsize",  "firmness",
				                             "african", "asian",       "caucasian"};
				bool ch = false;
				for (int i = 0; i < 11; i++) {
					ch |= ImGui::SliderFloat(pn[i], &g_ident[i], 0.f, 1.f);
				}
				if (ch) {
					applyPheno();  // recompute bind mesh + skeleton from the new ident
				}
			} else {
				ImGui::TextDisabled("soma_pheno.bin missing.");
			}
		}

		// VR IK: reach each unsensored arm to its controller (before skin, so it feeds the LBS).
		for (int j = 0; j < SOMA_J; j++) {
			g_ikActive[j] = false;
		}
		if (vrIk) {
			std::lock_guard<std::mutex> lk(g_mu);
			int Head = jointByName("Head");
			const char *armN[2][3] = {{"LeftArm", "LeftForeArm", "LeftHand"},
			                          {"RightArm", "RightForeArm", "RightHand"}};
			for (int side = 0; side < 2 && Head >= 0; side++) {
				int aA = jointByName(armN[side][0]), aF = jointByName(armN[side][1]), aH = jointByName(armN[side][2]);
				if (!g_haveHmd || !g_haveHand[side] || aA < 0 || aF < 0 || aH < 0) {
					continue;
				}
				if (g_tracked[aA] || g_tracked[aF]) {
					continue;  // a real arm sensor drives this limb -> trust it
				}
				const float *S = &g_restJointPos[aA * 3], *Er = &g_restJointPos[aF * 3];
				const float *Hr = &g_restJointPos[aH * 3], *headR = &g_restJointPos[Head * 3];
				float d[3];
				v3sub(Er, S, d);
				float Lu = v3len(d);
				v3sub(Hr, Er, d);
				float Lf = v3len(d);
				float tgt[3] = {headR[0] + g_handPos[side][0] - g_hmdPos[0],
				                headR[1] + g_handPos[side][1] - g_hmdPos[1],
				                headR[2] + g_handPos[side][2] - g_hmdPos[2]};
				float pole[3] = {0, -1, 0}, E[3], a1[3], b1[3], arc[16], RW[16];
				twoBoneElbow(S, tgt, Lu, Lf, pole, E);
				v3sub(Er, S, a1);
				v3sub(E, S, b1);
				shortestArc(a1, b1, arc);
				memcpy(RW, &g_bwP[aA * 16], 64);
				RW[3] = RW[7] = RW[11] = 0;
				mul4(arc, RW, &g_ikWorld[aA * 16]);
				g_ikActive[aA] = true;
				v3sub(Hr, Er, a1);
				v3sub(tgt, E, b1);
				shortestArc(a1, b1, arc);
				memcpy(RW, &g_bwP[aF * 16], 64);
				RW[3] = RW[7] = RW[11] = 0;
				mul4(arc, RW, &g_ikWorld[aF * 16]);
				g_ikActive[aF] = true;
			}
		}
		skin(verts);
		// Root translation applied to the skinned verts: HMD root (if on) else foot-lock else the
		// streamed /sinew/root delta.  g_jointPos is at rest-root.
		float off3[3] = {0, 0, 0};
		bool usedHmd = false;
		if (hmdRoot) {
			std::lock_guard<std::mutex> lk(g_mu);
			int Head = jointByName("Head");
			if (g_haveHmd && Head >= 0) {
				float o[3] = {g_hmdPos[0] - g_jointPos[Head * 3], g_hmdPos[1] - g_jointPos[Head * 3 + 1],
				              g_hmdPos[2] - g_jointPos[Head * 3 + 2]};
				if (std::fabs(o[0]) < 10 && std::fabs(o[1]) < 10 && std::fabs(o[2]) < 10) {
					memcpy(off3, o, sizeof o);
					usedHmd = true;
				}
			}
		}
		if (!usedHmd && rootFootLock) {
			int LF = jointByName("LeftFoot"), RF = jointByName("RightFoot"), HP = jointByName("Hips");
			if (HP >= 0) {  // hips XZ jump > 0.5 m => the clip looped/teleported: reset root to origin
				float hx = g_jointPos[HP * 3], hz = g_jointPos[HP * 3 + 2];
				float dx = hx - g_prevHipX, dz = hz - g_prevHipZ;
				if (dx * dx + dz * dz > 0.25f) {
					g_planted = -1;
					g_ox = g_oz = 0;
				}
				g_prevHipX = hx;
				g_prevHipZ = hz;
			}
			if (LF >= 0 && RF >= 0) {
				const float *lf = &g_jointPos[LF * 3], *rf = &g_jointPos[RF * 3];
				int planted = (lf[1] < rf[1]) ? 0 : 1;   // lower foot stays planted
				const float *foot = planted == 0 ? lf : rf;
				if (planted != g_planted) {
					g_anchorX = foot[0] + g_ox;
					g_anchorZ = foot[2] + g_oz;
					g_planted = planted;
				}
				g_ox = g_anchorX - foot[0];
				g_oz = g_anchorZ - foot[2];
				off3[0] = g_ox;
				off3[1] = -(lf[1] < rf[1] ? lf[1] : rf[1]);  // ground the lower foot
				off3[2] = g_oz;
			}
		} else if (!usedHmd) {
			std::lock_guard<std::mutex> lk(g_mu);
			off3[0] = g_rootDelta[0];
			off3[1] = g_rootDelta[1];
			off3[2] = g_rootDelta[2];
		}
		memcpy(g_rootOffset, off3, sizeof off3);
		for (auto &v : verts) {
			v[0] += off3[0];
			v[1] += off3[1];
			v[2] += off3[2];
		}
		// Present the solved frame through the PoseSink: build the neutral joint
		// poses (world-frame quat + position incl. root offset) and emit; the
		// polyscope adapter renders the skinned body.
		SinewJointPose joints[SOMA_J];
		for (int j = 0; j < SOMA_J; j++) {
			float q[4];
			matToQuat(&g_jointRot[j * 9], q);
			joints[j].qw = q[0];
			joints[j].qx = q[1];
			joints[j].qy = q[2];
			joints[j].qz = q[3];
			joints[j].x = g_jointPos[j * 3] + g_rootOffset[0];
			joints[j].y = g_jointPos[j * 3 + 1] + g_rootOffset[1];
			joints[j].z = g_jointPos[j * 3 + 2] + g_rootOffset[2];
		}
		poseSink.emit(poseSink.ctx, joints, SOMA_J, ImGui::GetTime());

		// Tracker-axis gizmos: each tracked joint gets 3 axis segments at its position + orientation.
		if (showAxes) {
			const float L = 0.10f;
			for (int i = 0; i < NTRK; i++) {
				int j = trkIdx[i];
				bool t = (j >= 0) && g_tracked[j];
				float px = j >= 0 ? g_jointPos[j * 3] + off3[0] : 0.f;
				float py = j >= 0 ? g_jointPos[j * 3 + 1] + off3[1] : 0.f;
				float pz = j >= 0 ? g_jointPos[j * 3 + 2] + off3[2] : 0.f;
				const float *R = j >= 0 ? &g_jointRot[j * 9] : nullptr;
				for (int a = 0; a < 3; a++) {
					axPts[i * 6 + a * 2] = {px, py, pz};
					axPts[i * 6 + a * 2 + 1] = t ? std::array<float, 3>{px + R[a] * L, py + R[3 + a] * L, pz + R[6 + a] * L}
					                              : std::array<float, 3>{px, py, pz};
				}
			}
			axes->updateNodePositions(axPts);
		}
		axes->setEnabled(showAxes);

		// SteamVR device gizmos — HMD, controllers, base stations (OpenVR frame), shown whenever
		// /vr/device data is arriving.
		int ndev = 0;
		{
			std::lock_guard<std::mutex> lk(g_mu);
			const float L = 0.12f;
			auto putDev = [&](const float *p, const float *quat) {
				if (ndev >= NDEV) {
					return;
				}
				float M[16];
				quatMat(quat, M);
				for (int a = 0; a < 3; a++) {
					devPts[ndev * 6 + a * 2] = {p[0], p[1], p[2]};
					devPts[ndev * 6 + a * 2 + 1] = {p[0] + M[a] * L, p[1] + M[4 + a] * L, p[2] + M[8 + a] * L};
				}
				ndev++;
			};
			if (g_haveHmd) {
				putDev(g_hmdPos, g_hmdQuat);  // HMD + controllers arrive on /vr/hmd, /vr/hand
			}
			if (g_haveHand[0]) {
				putDev(g_handPos[0], g_handQuat[0]);
			}
			if (g_haveHand[1]) {
				putDev(g_handPos[1], g_handQuat[1]);
			}
			for (auto &kv : g_vrDev) {  // base stations + real trackers, as reported (our own driven
				putDev(kv.second.pos, kv.second.quat);  // trackers are filtered out by hmd_reader serial)
			}
			for (int slot = ndev; slot < NDEV; slot++) {
				for (int a = 0; a < 6; a++) {
					devPts[slot * 6 + a] = {0, 0, 0};
				}
			}
		}
		devGiz->updateNodePositions(devPts);
		devGiz->setEnabled(showDevices && ndev > 0);
	};

	polyscope::show();  // blocking — close the window to exit
	vk_lbs_shutdown();
	polyscope::shutdown();
	return 0;
}

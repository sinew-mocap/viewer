// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// PoseSink — a DRIVEN (secondary) port of the server hexagon: a sink for solved
// body poses (the output of core/solve after core/calib).  Adapters (server
// cluster): the polyscope viewer (host/server/viewer), the SteamVR bridge
// (host/server/steamvr), and VMC-out (host/server/vmc, future).  Fanning the
// solve out through one port is what lets the server drive all three from a
// single solve pass.  (Declared now; wired load-bearing in the ports step.)
#pragma once
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// One solved joint: world-frame orientation (quaternion w,x,y,z) + position.
typedef struct SinewJointPose {
	float qw, qx, qy, qz;
	float x, y, z;
} SinewJointPose;

typedef struct PoseSink {
	void *ctx;
	// Emit one solved frame: `joints` is `n` joint poses in the rig's joint order.
	void (*emit)(void *ctx, const SinewJointPose *joints, int n, double time_s);
	void (*close)(void *ctx);
} PoseSink;

#ifdef __cplusplus
}
#endif

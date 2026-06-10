// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// VrTrackerSink — a DRIVEN (secondary) port of the viewer hexagon: a sink for
// the solved 6DOF body, sent on to SteamVR as virtual trackers (UDP 39542 ->
// the vr_bridge cluster's driver_sinew).  It is the write twin of VrSource: the
// viewer reads HMD/hands in, solves, and pushes the 15 trackers back out here.
// Adapter (viewer cluster): the UDP send socket (anny_demo's driveVrThread).
// (Declared now; wired load-bearing in the ports step.)
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct VrTrackerSink {
	void *ctx;
	// Emit joint `joint`'s 6DOF pose: position (metres) + rotation quaternion
	// (w,x,y,z), in OpenVR space (+y up, -z forward, RH).
	void (*send)(void *ctx, int joint, const float pos[3], const float quat[4]);
	void (*close)(void *ctx);
} VrTrackerSink;

#ifdef __cplusplus
}
#endif

// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// VrSource — a DRIVEN (secondary) port of the viewer hexagon: a source of
// inbound /vr OSC packets (HMD / hands / devices on UDP 39541 from the vr_bridge
// cluster's hmd_reader).  The viewer reads these to re-root the body at the HMD
// and draw the rig.  It is the read end whose paired sink is VrTrackerSink (the
// 6DOF back-out).  Adapter (viewer cluster): the UDP receive socket
// (anny_demo's vrOscThread).  (Declared now; wired load-bearing in the ports step.)
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct VrSource {
	void *ctx;
	// Fill `osc` (capacity `cap`) with the next inbound /vr packet, set *len.
	// Returns 1 on a packet, 0 if none available this poll, <0 on error/close.
	int (*next)(void *ctx, uint8_t *osc, size_t cap, size_t *len);
	void (*close)(void *ctx);
} VrSource;

#ifdef __cplusplus
}
#endif

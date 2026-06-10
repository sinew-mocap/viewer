// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// TrackerSource — a DRIVEN (secondary) port of the viewer hexagon: a source of
// inbound /sinew OSC packets (the body feed on UDP 39539 from the driver:
// /sinew/tracker, /sinew/root, /sinew/pheno).  It is the read end of the pipe
// whose sink is PoseSink (solve -> render): the core reads /sinew here, solves,
// and emits through PoseSink.  Adapter (viewer cluster): the UDP receive socket
// (anny_demo's oscThread).  (Declared now; wired load-bearing in the ports step.)
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct TrackerSource {
	void *ctx;
	// Fill `osc` (capacity `cap`) with the next inbound /sinew packet, set *len.
	// Returns 1 on a packet, 0 if none available this poll, <0 on error/close.
	int (*next)(void *ctx, uint8_t *osc, size_t cap, size_t *len);
	void (*close)(void *ctx);
} TrackerSource;

#ifdef __cplusplus
}
#endif

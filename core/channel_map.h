// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// viewer/core — the neutral domain: the /sinew OSC channel name -> SOMA joint
// name mapping.  No polyscope, no socket, no rig lookup (that's the adapter).
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define CHANNEL_MAP_COUNT 15  // mapped joints, in NodeNumber order

// Returns the SOMA joint name for a /sinew tracker channel name, or NULL if the
// channel is not mapped.  (The adapter resolves the SOMA name to a rig index.)
const char *soma_for_channel(const char *channel);

// The SOMA joint name for NodeNumber index i (0..CHANNEL_MAP_COUNT-1), or NULL.
// The index order matches the /sinew NodeNumber order (driver_sinew kJointNames).
const char *channel_map_soma(int i);

#ifdef __cplusplus
}
#endif

// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// Minimal extern "C" shim over polyscope so Lean (@[extern]) and C drive a native
// skinned-mesh viewer without Python or a game engine.  Lean computes the
// deformed vertices (Slang LBS on Vulkan/Metal/CPU) and hands them here each
// frame; polyscope owns the window, camera, and ImGui.
#ifndef SINEW_VIEW_H
#define SINEW_VIEW_H

#ifdef __cplusplus
extern "C" {
#endif

// Open the window and register the mesh: verts = n_verts*3 floats (xyz),
// faces = n_faces*3 ints (triangle vertex indices).
void sinew_view_init(const float *verts, int n_verts, const int *faces, int n_faces);

// Replace the vertex positions (n_verts*3 floats) — the per-frame deform output.
void sinew_view_update(const float *verts, int n_verts);

// One non-blocking UI iteration (call from the driver loop).
void sinew_view_frame(void);

// Blocking loop until the window closes.
void sinew_view_show(void);

void sinew_view_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif  // SINEW_VIEW_H

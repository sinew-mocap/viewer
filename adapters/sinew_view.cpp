// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// polyscope implementation of the sinew viewer shim (see sinew_view.h).
#include "sinew_view.h"

#include <array>
#include <cstdint>
#include <vector>

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

static polyscope::SurfaceMesh *g_mesh = nullptr;

static std::vector<std::array<float, 3>> pack_verts(const float *v, int n) {
	std::vector<std::array<float, 3>> out((size_t)n);
	for (int i = 0; i < n; ++i)
		out[(size_t)i] = {v[3 * i], v[3 * i + 1], v[3 * i + 2]};
	return out;
}

extern "C" void sinew_view_init(const float *verts, int n_verts, const int *faces, int n_faces) {
	polyscope::options::verbosity = 3;  // log the chosen backend on init
	polyscope::init();
	std::vector<std::array<uint32_t, 3>> F((size_t)n_faces);
	for (int i = 0; i < n_faces; ++i)
		F[(size_t)i] = {(uint32_t)faces[3 * i], (uint32_t)faces[3 * i + 1],
		                (uint32_t)faces[3 * i + 2]};
	g_mesh = polyscope::registerSurfaceMesh("anny", pack_verts(verts, n_verts), F);
}

extern "C" void sinew_view_update(const float *verts, int n_verts) {
	if (g_mesh)
		g_mesh->updateVertexPositions(pack_verts(verts, n_verts));
}

extern "C" void sinew_view_frame(void) {
	polyscope::frameTick();
}

extern "C" void sinew_view_show(void) {
	polyscope::show();
}

extern "C" void sinew_view_shutdown(void) {
	polyscope::shutdown();
}

// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// Lean FFI adapter for the polyscope viewer (see Sinew.Viewer).  Plain C on
// purpose: it is compiled into the Lean executable, so it must pull no C++
// runtime.  All C++ lives behind the sinew_view_* C ABI in a self-contained DLL.
// FloatArray vertices (doubles) are narrowed to float; faces are a ByteArray of
// little-endian int32 indices.
#include <stdlib.h>

#include <lean/lean.h>

#include "sinew_view.h"

// Loads sinew_view.dll and binds the stub dispatchers (sinew_view_stub.c).
extern int sinew_view_load(void);

static float *narrow(b_lean_obj_arg verts, uint32_t n_verts) {
	const double *vd = lean_float_array_cptr(verts);
	size_t m = (size_t)n_verts * 3;
	float *vf = (float *)malloc(m * sizeof(float));
	for (size_t i = 0; i < m; i++)
		vf[i] = (float)vd[i];
	return vf;
}

LEAN_EXPORT lean_object *sinew_view_init_lean(b_lean_obj_arg verts, uint32_t n_verts,
                                              b_lean_obj_arg faces, uint32_t n_faces,
                                              lean_object *world) {
	(void)world;
	sinew_view_load();  // load the DLL + bind the stubs on first use
	float *vf = narrow(verts, n_verts);
	sinew_view_init(vf, (int)n_verts, (const int *)lean_sarray_cptr(faces), (int)n_faces);
	free(vf);
	return lean_io_result_mk_ok(lean_box(0));
}

LEAN_EXPORT lean_object *sinew_view_update_lean(b_lean_obj_arg verts, uint32_t n_verts,
                                                lean_object *world) {
	(void)world;
	float *vf = narrow(verts, n_verts);
	sinew_view_update(vf, (int)n_verts);
	free(vf);
	return lean_io_result_mk_ok(lean_box(0));
}

LEAN_EXPORT lean_object *sinew_view_frame_lean(lean_object *world) {
	(void)world;
	sinew_view_frame();
	return lean_io_result_mk_ok(lean_box(0));
}

LEAN_EXPORT lean_object *sinew_view_show_lean(lean_object *world) {
	(void)world;
	sinew_view_show();
	return lean_io_result_mk_ok(lean_box(0));
}

LEAN_EXPORT lean_object *sinew_view_shutdown_lean(lean_object *world) {
	(void)world;
	sinew_view_shutdown();
	return lean_io_result_mk_ok(lean_box(0));
}

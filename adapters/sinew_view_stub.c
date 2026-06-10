// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// Host side of the core/host split: a pure-C runtime loader + dispatchers for the
// self-contained sinew_view.dll (the "core").  This is the same pattern as
// generate_stubs (steamvr/gen), but hand-written in C on purpose: the generated
// posix_stubs output uses GNU std::map, whose libstdc++ symbols can't link into a
// Lean executable (lean's clang links libc++, not libstdc++).  Keeping the host
// pure C means zero C++ runtime crosses the FFI boundary.
#include <windows.h>

#include "sinew_view.h"

typedef void (*fn_init)(const float *, int, const int *, int);
typedef void (*fn_update)(const float *, int);
typedef void (*fn_void)(void);

static HMODULE g_dll = NULL;
static fn_init p_init = NULL;
static fn_update p_update = NULL;
static fn_void p_frame = NULL, p_show = NULL, p_shutdown = NULL;

int sinew_view_load(void) {
	if (g_dll)
		return 1;
	g_dll = LoadLibraryA("sinew_view.dll");
	if (!g_dll)
		return 0;
	p_init = (fn_init)(void *)GetProcAddress(g_dll, "sinew_view_init");
	p_update = (fn_update)(void *)GetProcAddress(g_dll, "sinew_view_update");
	p_frame = (fn_void)(void *)GetProcAddress(g_dll, "sinew_view_frame");
	p_show = (fn_void)(void *)GetProcAddress(g_dll, "sinew_view_show");
	p_shutdown = (fn_void)(void *)GetProcAddress(g_dll, "sinew_view_shutdown");
	return p_init && p_update && p_frame && p_show && p_shutdown;
}

// Dispatchers — same names as the DLL exports, so callers (the Lean adapter) link
// against these and reach the DLL through the loaded pointers.
void sinew_view_init(const float *v, int nv, const int *f, int nf) {
	if (sinew_view_load())
		p_init(v, nv, f, nf);
}
void sinew_view_update(const float *v, int nv) {
	if (sinew_view_load())
		p_update(v, nv);
}
void sinew_view_frame(void) {
	if (sinew_view_load())
		p_frame();
}
void sinew_view_show(void) {
	if (sinew_view_load())
		p_show();
}
void sinew_view_shutdown(void) {
	if (sinew_view_load())
		p_shutdown();
}

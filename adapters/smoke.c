// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// Link smoke test for the polyscope viewer shim.  With no argument it only
// exercises the link (no window — safe headless / in CI).  Pass any argument to
// actually open the viewer on a machine with a display + GPU.
#include <stdio.h>
#include <string.h>

#include "sinew_view.h"

int main(int argc, char **argv) {
	float v[9] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
	int f[3] = {0, 1, 2};
	const char *mode = argc > 1 ? argv[1] : "";
	if (strcmp(mode, "show") == 0) {  // blocking viewer — needs an interactive display
		sinew_view_init(v, 3, f, 1);
		sinew_view_update(v, 3);
		sinew_view_show();
		sinew_view_shutdown();
	} else if (strcmp(mode, "tick") == 0) {  // bounded: init + 3 frames + exit
		sinew_view_init(v, 3, f, 1);
		sinew_view_update(v, 3);
		for (int i = 0; i < 3; i++)
			sinew_view_frame();
		sinew_view_shutdown();
	} else {  // no arg / --help: link-only, no window
		printf("usage: sinew_view_smoke [show|tick]\n");
		printf("  show   open the viewer (blocking)\n");
		printf("  tick   init + 3 frames + exit (smoke)\n");
	}
	return 0;
}

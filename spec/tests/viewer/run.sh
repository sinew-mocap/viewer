#!/usr/bin/env bash
# Build and run the Lean-driven native viewer demo using the core/host stub split:
#   core  = sinew_view.dll (polyscope + C++ runtime bundled; extern "C" only)
#   host  = generate_stubs dispatchers + a tiny loader, compiled with leanc and
#           linked into the Lean exe (no C++ runtime crosses the FFI boundary).
# Headless this exits 0 on polyscope's mock backend; on a desktop it opens the
# real window.
set -e
here="$(cd "$(dirname "$0")" && pwd)"
spec="$here/../.."
root="$spec/.."

# 1. Self-contained viewer DLL (polyscope + MinGW C++ runtime bundled).
if [ ! -f "$root/viz_native/build/build.ninja" ]; then
	cmake -S "$root/viz_native" -B "$root/viz_native/build" -G Ninja -DCMAKE_BUILD_TYPE=Release
fi
cmake --build "$root/viz_native/build" --target sinew_view

# 2. Compile the pure-C FFI adapter + runtime stub loader with MinGW (it has the
#    C headers leanc's clang lacks); -I lean's include resolves lean.h.  No C++ →
#    nothing to clash with lean's libc++ at the exe link.
leaninc="$(lean --print-prefix)/include"
gcc -c "$root/viz_native/sinew_view_lean.c" \
	-I "$leaninc" -I "$root/viz_native" -o "$root/viz_native/sinew_view_lean.o"
gcc -c "$root/viz_native/sinew_view_stub.c" \
	-I "$root/viz_native" -o "$root/viz_native/sinew_view_stub.o"

# 3. Link the Lean exe (see lakefile moreLinkArgs).
(cd "$spec" && lake build viewer_demo)

# 4. Put the DLL next to the exe and run.
cp -u "$root/viz_native/build/sinew_view.dll" "$spec/.lake/build/bin/"
"$spec/.lake/build/bin/viewer_demo"

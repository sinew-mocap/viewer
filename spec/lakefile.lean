-- SPDX-License-Identifier: MIT
-- SinewServerSpec — the server host-adapter spec: the Lean-driven native viewer
-- (Viewer) and its demo exe.  Self-contained.
--
-- viewer_demo links only two pure-C objects — the FFI adapter and the runtime
-- stub loader (built at host/server/ by tests/viewer/run.sh) — so no C++ runtime
-- crosses into the Lean exe (polyscope is g++/libstdc++, isolated in the
-- self-contained sinew_view.dll loaded at runtime).
import Lake
open Lake DSL

package "SinewServerSpec" where
  version := v!"0.1.0"

lean_lib SinewServerSpec where
  srcDir := "."
  globs  := #[Glob.one `Sinew.Viewer]

lean_exe viewer_demo where
  root := `ViewerDemo
  moreLinkArgs := #[
    "../adapters/sinew_view_lean.o",
    "../adapters/sinew_view_stub.o"
  ]

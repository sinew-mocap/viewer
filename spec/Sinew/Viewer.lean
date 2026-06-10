-- SPDX-License-Identifier: MIT
-- Copyright (c) 2026-present K. S. Ernest (iFire) Lee
--
-- Lean `@[extern]` bindings to the native polyscope viewer (viz_native/sinew_view),
-- so a Lean program drives the window directly — the same FFI pattern as
-- `Sinew.Serial` / `Sinew.Udp`.  Vertices are a FloatArray (xyz per vertex); the
-- C++ adapter narrows the doubles to float.  Faces are a ByteArray of little-endian
-- int32 triangle indices.

namespace Sinew.Viewer

/-- Open the window and register the mesh. -/
@[extern "sinew_view_init_lean"]
opaque init (verts : FloatArray) (nVerts : UInt32) (faces : ByteArray) (nFaces : UInt32) : IO Unit

/-- Replace the vertex positions (the per-frame deform output). -/
@[extern "sinew_view_update_lean"]
opaque update (verts : FloatArray) (nVerts : UInt32) : IO Unit

/-- One non-blocking UI iteration. -/
@[extern "sinew_view_frame_lean"]
opaque frame : IO Unit

/-- Blocking UI loop until the window closes. -/
@[extern "sinew_view_show_lean"]
opaque showWindow : IO Unit

/-- Tear down the viewer. -/
@[extern "sinew_view_shutdown_lean"]
opaque shutdown : IO Unit

end Sinew.Viewer

-- SPDX-License-Identifier: MIT
-- Copyright (c) 2026-present K. S. Ernest (iFire) Lee
--
-- Lean program that drives the native polyscope viewer through `Sinew.Viewer`,
-- showing linear-blend skinning end to end: a vertical ribbon rigged to two bones
-- (root + tip) bends as Lean re-skins it every frame and pushes the new vertices.
--   viewer_demo skin   animate the bend in the real window (close it to exit)
--   viewer_demo        headless smoke: 3 frames + exit (mock backend, exit 0)
-- Build: spec/tests/viewer/run.sh
import Sinew.Viewer

def segs : Nat := 8                 -- ribbon height segments (segs+1 levels)
def nVerts : UInt32 := (UInt32.ofNat segs + 1) * 2
def nFaces : UInt32 := UInt32.ofNat segs * 2

-- Rest mesh: two columns (x = ±0.15) rising y = 0..2; index = level*2 + column.
def restVerts : Array (Float × Float × Float) := Id.run do
  let mut a := #[]
  for lvl in [0:segs + 1] do
    let y := (Float.ofNat lvl) * (2.0 / Float.ofNat segs)
    a := a.push (-0.15, y, 0.0)
    a := a.push (0.15, y, 0.0)
  return a

-- Tip-bone weight: 0 at the base, 1 at the top (smooth blend up the ribbon).
def weights : Array Float := restVerts.map (fun (_, y, _) => min 1.0 (max 0.0 (y / 2.0)))

-- Triangulated quads between consecutive levels.
def faces : ByteArray := Id.run do
  let mut b := ByteArray.empty
  let push32 := fun (b : ByteArray) (x : UInt32) =>
    b.push (x &&& 0xff).toUInt8 |>.push ((x >>> 8) &&& 0xff).toUInt8
      |>.push ((x >>> 16) &&& 0xff).toUInt8 |>.push ((x >>> 24) &&& 0xff).toUInt8
  for lvl in [0:segs] do
    let a := UInt32.ofNat (2 * lvl)
    for idx in #[a, a + 1, a + 3, a, a + 3, a + 2] do
      b := push32 b idx
  return b

-- Linear-blend skinning for a tip-bone rotation of `theta` about the base (z-axis):
-- bone0 = identity, bone1 = Rz(theta); v' = (1-w)·v + w·(Rz(theta)·v).
def skin (theta : Float) : FloatArray := Id.run do
  let c := Float.cos theta
  let s := Float.sin theta
  let mut fa := FloatArray.empty
  for (v, w) in restVerts.zip weights do
    let (x, y, z) := v
    let rx := x * c - y * s
    let ry := x * s + y * c
    fa := fa.push ((1.0 - w) * x + w * rx)
    fa := fa.push ((1.0 - w) * y + w * ry)
    fa := fa.push z
  return fa

def main (args : List String) : IO Unit := do
  Sinew.Viewer.init (skin 0.0) nVerts faces nFaces
  if args.contains "skin" then
    IO.println "viewer_demo: skinning a 2-bone ribbon (~48 s), or close the window…"
    for k in [0:3000] do
      let theta := 0.9 * Float.sin ((Float.ofNat k) * 0.04)
      Sinew.Viewer.update (skin theta) nVerts
      Sinew.Viewer.frame
      IO.sleep 16
  else
    for _ in [0:3] do
      Sinew.Viewer.frame
    IO.println "viewer_demo: Lean drove init + 3 skinned frames + shutdown OK"
  Sinew.Viewer.shutdown

# viewer — sinew-mocap

A hexagon cluster (`core/` + `ports/` + `adapters/`) of the Sinew mocap stack:
the polyscope PoseSink — binds /sinew OSC, runs the body solve, and renders the ANNY body.

## Build

```
cmake -B build && cmake --build build
```

## Dependencies

See `ports/sibling-repos.txt` — clone the listed `sinew-mocap` repos side-by-side in `~/`.

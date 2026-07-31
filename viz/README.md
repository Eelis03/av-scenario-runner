# Scenario playback

A self-contained TypeScript and Canvas playback of a scenario trace exported by
the Python package. It has no runtime dependencies, loads nothing from a
content delivery network, and makes no network request of any kind. The only
development dependency is the TypeScript compiler.

The Python package and its test suite do not import anything here, and never
need this directory to be built. `src/` and `tests/` in the repository root
contain no reference to `viz/`.

## Build

```bash
cd viz
npm install
npm run build
```

`npm run build` compiles `src/*.ts` to ES modules in `dist/`, which
`index.html` loads directly. There is no bundler.

## Run

A browser will not `fetch` a local file from a `file://` URL, so serve the
directory:

```bash
cd viz
python -m http.server 8000
```

then open `http://localhost:8000/`. The trace selector lists the recordings in
`traces/`. The file picker works from a `file://` URL as well, so the page is
usable without a server if you open a trace by hand.

## Producing a trace

```bash
uv run python examples/export_viz_trace.py --scenario aggressive_cut_in
```

writes `viz/traces/aggressive_cut_in.json`. The schema is documented in
`src/types.ts` and carries a `trace_version`; the viewer refuses a version it
does not recognise rather than drawing a picture that looks authoritative and
is wrong.

## What is drawn

The camera follows the ego vehicle and rotates with its heading, so a curved
road reads the same as a straight one. Vehicle footprints are the rectangles
the Python collision check covers with discs, not the disc cover itself, so the
gap seen on screen is the true bumper to bumper gap and is slightly larger than
the conservative clearance the assertions use. Both vehicles turn red on the
step where clearance reaches zero.

The side panel shows the live readout at the current step and the verdict of
every assertion, with the worst value each observed and the time it occurred.

## Layout

| File | Responsibility |
| --- | --- |
| `src/types.ts` | Trace schema, version check, and narrowing from parsed JSON |
| `src/geometry.ts` | Road frame conversions and footprint corners |
| `src/renderer.ts` | Canvas drawing of one step |
| `src/player.ts` | Wall clock playback, independent of display refresh rate |
| `src/main.ts` | DOM wiring only |

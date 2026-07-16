# winglevlr Agent Notes

This file is a local handoff for future assistant work in this repository.

## Current Navigation Work

- `winglevlr` is used by `autotrim` for the revived ILS simulator and the new placeholder synthetic VOR simulator.
- Keep the existing heading convention for now: normalized headings use `(0, 360]`, so exact north is represented as `360`, not `0`.
- The recent angle-wrap cleanup is committed as `75d938d` (`Fix nav angle wrap handling`):
  - `WaypointNav.h::constrain360()` now uses `fmodf()` while preserving `(0, 360]`.
  - `WaypointNav.h::angularDiff(a, b)` handles wrap with modulo math.
  - `IlsSimulator::courseErr()` uses `angularDiff(...)` instead of raw subtraction, fixing the north-crossing localizer bug.
  - `RollAHRS.h` received matching loop-free wrap cleanup.
- The `TinyGPSPlus` dependency is intentionally a git checkout at
  `~/Arduino/libraries/TinyGPSPlus`, cloned from
  `https://github.com/mikalhart/TinyGPSPlus.git`.
  - Do not rely on `arduino-cli lib install TinyGPSPlus@1.1.0`; the Arduino
    index only exposes `1.0.3`.
  - The practical standardization path is to keep the git clone in sync across
    machines (`git fetch`, `git checkout master`, `git pull --ff-only`) or pin
    both machines to the same commit.
- `make BOARD=csim` in `winglevlr` depends on the host-side `lv_port_linux`
  checkout and its built artifacts.
  - The repo-root `make -j2` path is not the right way to build the LVGL host
    pieces.
  - Build the sibling host project with:
    - `cd ~/src/lv_port_linux`
    - `cmake -B build`
    - `make -C build -j2`
  - If csim link errors mention unresolved X11 symbols, that usually means the
    build system is using the wrong entry point or missing the CMake-built host
    artifacts, not that the X11 dev packages are absent.

## Synthetic VOR Context

- `VorSimulator` lives next to `IlsSimulator` in `WaypointNav.h`.
- `autotrim` mode 6 currently creates a synthetic VOR one nautical mile ahead of the aircraft's current course.
- VOR CDI full-scale deflection is modeled as 10 degrees.
- Cone of confusion is intentionally a simple fixed `0.2 NM` radius from the station, not a full angular/altitude model.
- Until the SL30 wrapper exposes proper validity flags, `autotrim` uses placeholder needle behavior inside the cone.

## SL30 Protocol Context

- The reference PDF is committed in `autotrim`:
  - `docs/reference/SL30_Installation_Manual_560-0404-03_Rev_A.pdf`
- Appendix E documents `$PMRRV21`.
- Relevant flag byte bits:
  - bit 1: back-course enabled
  - bit 2: localizer detect
  - bit 3: FROM flag
  - bit 4: TO flag
  - bit 5: GSI superflag valid
  - bit 6: GSI valid
  - bit 7: NAV superflag valid
  - bit 8: NAV valid
- For VOR simulation, the next likely improvement is to expose these flags through the SL30 wrapper so VOR can represent TO/FROM, no glideslope, and cone-of-confusion/invalid behavior directly.

## CSIM / LVGL Caution

- Do not casually re-apply the UBUNTU-to-CSIM/LVGL repair attempt.
- That work is intentionally stashed for Jim to debug manually because it appears to break the LVGL path involving `~/src/lv_port_pc_vscode`.
- Current stash pointers at the time this file was written:
  - `winglevlr`: `stash@{0}` / `WIP csim LVGL build repair`
  - `lvglConfigPanel`: `stash@{0}` / `WIP csim LVGL build repair`
- The normal `make -j2` build passed after stashing the CSIM/LVGL work. Do not claim `make BOARD=csim` is fixed unless that stash is intentionally revisited and revalidated.
- The newer csim recovery path that actually worked was to keep the TinyGPSPlus
  git checkout in place and use `lv_port_linux`'s CMake build (`cmake -B build`
  then `make -C build -j2`).

## Local Artifacts

- Unrelated untracked `.vscode/*.log` files may exist in this repo. Leave them alone unless Jim asks.

## Commit Convention

- Assistant-made commits should include an `AI generated` annotation unless Jim explicitly asks to omit it.

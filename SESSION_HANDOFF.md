# Session Handoff — winglevlr

Updated 2026-07-17.

## Current repository state

- `master` is synchronized with `origin/master`.
- The current tip is `8fd2922` — `Add vscode log files to gitignore`.
- This repository is also the navigation/simulator dependency used by
  `autotrim`.

## Navigation rules to preserve

- Normalized headings intentionally use `(0, 360]`; exact north is `360`, not
  `0`.
- Keep angle wrapping modulo-based and loop-free. The `WaypointNav` and
  `RollAHRS` wrap fixes prevent north-crossing localizer errors.
- `VorSimulator` lives beside `IlsSimulator` in `WaypointNav.h`; its current
  behavior is deliberately a placeholder, not a complete radio model.
- Mode 6 in `autotrim` creates a synthetic VOR one NM ahead of current course;
  full-scale deflection is 10 degrees and the cone radius is 0.2 NM.

## Build and dependency recovery

- Build the host LVGL support through `~/src/lv_port_linux` CMake:

  ```sh
  cmake -B build
  make -C build -j2
  ```

- Do not use the old repo-root make path for `lv_port_linux`; it can omit the
  correct X11 link setup.
- Keep `~/Arduino/libraries/TinyGPSPlus` as a synchronized git checkout. The
  Arduino library index does not provide the needed 1.1.0 package.
- The CSIM/LVGL repair stash is intentionally not applied. Do not claim the
  CSIM LVGL path is fixed unless that work is deliberately revisited and
  revalidated.
- Leave unrelated `.vscode/*.log` artifacts alone.

## Likely next improvements

Expose SL30 NAV validity, TO/FROM, no-glideslope, and cone-of-confusion state
so the VOR simulation can represent invalid/TO/FROM behavior directly instead
of using placeholder needle behavior.

## Detailed navigation context

### Heading and angle behavior

The `(0, 360]` heading convention is a project-wide compatibility boundary.
Many callers and displays expect north to appear as `360`; changing it to
`0` would be mathematically conventional but behaviorally disruptive. Use the
shared modulo helpers for subtraction and normalization rather than adding
special cases around north.

The angle-wrap fix was important because raw subtraction fails when one course
is just below 360 and the other is just above 0. The corresponding `RollAHRS`
cleanup should remain aligned with `WaypointNav` rather than growing a second
wrap convention.

### ILS/VOR simulator boundaries

- `IlsSimulator` computes angular ILS-like guidance, not GPS-style discrete CDI
  scaling. Its output is scaled by 2.0 before going through the SL30 path.
- Localizer and glideslope intentionally use different reference points. Do
  not simplify them to one point without rechecking approach geometry.
- The approach database does not yet carry runway-specific lengths or explicit
  localizer antenna coordinates; the current 6,000 ft plus 1,000 ft projection
  is an approximation.
- `VorSimulator` is synthetic test infrastructure. It places a station one NM
  ahead, uses a 10-degree full-scale lateral model, and treats 0.2 NM as a
  simple cone-of-confusion radius.
- The next protocol-facing improvement is to expose SL30 flag state: back
  course, localizer detect, FROM, TO, GSI superflag/valid, and NAV
  superflag/valid. This would replace placeholder pegged-needle behavior with
  explicit validity and no-glideslope semantics.

### Dependency and build history

The TinyGPSPlus Arduino registry package is not the expected version. The
known-good solution is a git checkout at `~/Arduino/libraries/TinyGPSPlus`,
kept synchronized across machines. Similarly, the host LVGL project must be
built via `~/src/lv_port_linux` CMake; repo-root make can compile part of the
tree while producing misleading unresolved X11 link failures.

The LVGL repair attempt is intentionally stashed. It was not proven safe to
apply to the existing `~/src/lv_port_pc_vscode` path. A future repair should
be tested against both normal `make` and the full CSIM pipe workflow before
being described as fixed.

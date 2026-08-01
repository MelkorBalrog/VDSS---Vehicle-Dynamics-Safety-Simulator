# AGENTS.md

## Cursor Cloud specific instructions

This repository contains **two components**:

1. **VDSS MATLAB simulator (primary product)** — `VDSS.m`, `Source/`, `Scripts/`,
   `tests/`, `Curves/`, `codegen/`. A desktop scientific application (App Designer /
   `uifigure` GUI, class-based toolboxes) for vehicle dynamics & crash-severity
   simulation. There are no servers, ports, databases, or secrets.
2. **Python Tkinter GUI prototype** — `python/vdss_gui/`. An early UI skeleton that
   mirrors the MATLAB GUI. See `python/README.md`.

### MATLAB simulator (requires licensed MATLAB — not available in the cloud VM)
- The main product and its test suite need a **licensed MATLAB** install, which cannot
  be installed via a package manager in this VM. Run steps are in `README.md`
  (`addpath(genpath(pwd))`, then `VDSS`; tests via `runtests('tests')`).
- Tests use the `matlab.unittest` framework (`functiontests` / `verifyEqual`) and the
  GUI uses App Designer widgets, so **GNU Octave cannot run the tests or the GUI**.
- **Octave (8.x) is pre-installed** as a convenience for sanity-checking *pure-numeric*
  core algorithms only (e.g. `Source/Physics/VehicleCollisionSeverity.m` collision
  dynamics). Caveat: Octave treats MATLAB double-quoted strings as `char` arrays, so
  code that relies on the MATLAB `string` class (e.g. `"S" + num2str(...)` severity
  labels) produces numeric results in Octave — do not trust those outputs. Octave is
  **not** a substitute for MATLAB.
  - Example: `octave --no-gui --quiet` then `addpath('Source/Physics')` and construct
    `VehicleCollisionSeverity(...)`, call `PerformCollision()` / `CalculateSeverity()`.

### Python GUI prototype (the runnable app in this VM)
- Requires `python3-tk` (system package, pre-installed in the VM snapshot) plus an X
  display. This VM exposes a desktop on `DISPLAY=:1`.
- Run from the `python/` directory: `python3 -m vdss_gui.main`. If launching from
  another directory, set `PYTHONPATH` to the `python/` folder.
- It is a **skeleton**: the Setup/Vehicle/Run/Results tabs are intentionally empty
  frames and there is no simulation backend yet. The implemented functionality is the
  window/tab/menu layout plus an async background-task → `queue` → `_poll_queue`
  pump (`start_background_task`). Blank tab content is expected, not a bug.
- Uses only the Python standard library — there are no `pip` dependencies.

### Dependencies
- There are **no package-manager dependency files** (no `requirements.txt`,
  `package.json`, etc.). The only environment needs are the system packages
  `python3-tk` and `octave`, which are baked into the VM snapshot; the startup update
  script is therefore effectively a no-op.

# VDSS Python GUI

This directory contains the early Python implementation of the Vehicle Dynamics
Safety Simulator user interface.  It mirrors the MATLAB prototype but focuses
on a production-ready architecture with parallel execution in mind.

The GUI is written with `tkinter` so it runs without external dependencies.  No
backend model is provided yet; the window layout and asynchronous task queue are
in place so computation-heavy modules can be integrated later.

Run the GUI with:

```bash
python3 -m vdss_gui.main
```

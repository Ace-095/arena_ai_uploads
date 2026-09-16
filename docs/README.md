# Setup docs — bench → hardware → air

Three docs, in the order you should read them. Each one is self-contained and
each one ends with an acceptance checklist; do not skip a checklist.

| # | Doc | What it covers | Vehicle | Hardware needed |
|---|---|---|---|---|
| 01 | [`01_TWO_LAPTOP_SITL_GAZEBO_SETUP.md`](01_TWO_LAPTOP_SITL_GAZEBO_SETUP.md) | `git clone` → working hunt on two laptops: Linux (SITL + Gazebo + `mission_pi`) and Windows (Mission Planner + bridge + UI) | SITL / Gazebo | 2 laptops, a webcam, a printed A3 QR |
| 02 | [`02_PI5_AI_HAT_CAMERAS_BENCH.md`](02_PI5_AI_HAT_CAMERAS_BENCH.md) | Raspberry Pi 5 + Hailo AI HAT + two CSI cameras brought up against the **simulated** vehicle | SITL | Pi 5, AI HAT+, Pi Cam 3, IMX477, field router |
| 03 | [`03_REAL_FLIGHT_INTEGRATION.md`](03_REAL_FLIGHT_INTEGRATION.md) | Everything physically connected: Pixhawk, telemetry radio, Pi onboard, failsafes, flight-day procedure | real airframe | the full aircraft |

## arena/test1 — Polygon + Configurable Height + FOV Coverage + A3 QR at 10/15 m

New branch `arena/test1` adds polygon primary, fence inclusion conversion, configurable max height 5/10/15 m, FOV-optimal coverage per cam.txt, and tuned detection for A3 at 10/15 m.

| # | Doc | What it covers |
|---|-----|----------------|
| 00 | [`arena_test1/00_DETECTION_AT_10_15M_A3_QR_HEF.md`](arena_test1/00_DETECTION_AT_10_15M_A3_QR_HEF.md) | **Why detection fails at 15 m, HEF impact, A3 pixel budget, and how to make it work every time** |
| 01 | [`arena_test1/01_SITL_GAZEBO_POP_OS_WINDOWS11.md`](arena_test1/01_SITL_GAZEBO_POP_OS_WINDOWS11.md) | **Method 1:** Pop!_OS SITL + Gazebo + mission_pi, Windows 11 MP + UI bridge (two-laptop) |
| 02 | [`arena_test1/02_PI5_AI_HAT_BENCH.md`](arena_test1/02_PI5_AI_HAT_BENCH.md) | **Method 2:** Real bench test Pi 5 + Hailo AI HAT + cams, no props |
| 03 | [`arena_test1/03_REAL_FLIGHT.md`](arena_test1/03_REAL_FLIGHT.md) | **Method 3:** Real flight integration, preflight, flight day |

Quick start: [`arena_test1/README.md`](arena_test1/README.md) — index of new features + which doc to read.


## Which one do I want?

* *"The UI can't see the Pi / nothing works and I don't know which side is
  broken"* → **01 §3** (zero-hardware smoke test, 5 minutes, isolates software
  from everything else).
* *"I have the Pi and the cameras in a box"* → **02**.
* *"We're flying"* → **03**, and only after 01 and 02 are green.

## Companion docs that already exist

| Path | What it is |
|---|---|
| `workspace-*/mission_pi/SIM_GUIDE.md` | the bench guide's guts: why the ladder looks like this, the full troubleshooting table, the Gazebo phase |
| `workspace-*/mission_pi/RUN_GAZEBO.md` | paste-and-go Gazebo run sheet (four terminals, exact paths) |
| `workspace-*/mission_pi/README.md` | `mission_pi` reference: endpoints, config keys, mission FSM |
| `workspace-*/mission_pi/training/README.md` + `compile_hef.md` | the x86 fine-tune → HEF loop for the Hailo HAT |
| `workspace-*/mission-ui/docs/LAPTOP_SETUP_MP_UI_OFFLINE_MAPS.md` | the Windows laptop in detail: MP mirror, both offline-map systems, field-day checklist |
| `workspace-*/mission-ui/docs/api-contract.md` | every channel, endpoint and invariant between the Pi, the bridge and the UI |
| `workspace-*/our-mission.md` | the mission spec + the camera/optics math the configs encode |

`workspace-*/hehe/docs/*` is the older reference implementation's paperwork —
history, not instructions for this stack.

## Status of these docs

Written 2026-09-15 against `main`. Software claims were checked by running the
stack, not by reading it: the zero-hardware chain (`tools/fake_sitl.py` +
`config.demo.yaml` + `bridge/mp_bridge.py`) ran end to end and latched
`MISSION-QR-001` in the bridge, `tools/link_doctor.py` reported `8 checks,
0 FAIL`, all 17 test suites passed, and `mp_bridge.py --selftest` printed
`SELFTEST PASS`. Hardware steps (Hailo install, CSI cameras, Pixhawk wiring,
telemetry radio) are documented from the vendors' current documentation and
this project's own notes — they are the parts that still need a real bench to
tick off, which is what the checklists are for.

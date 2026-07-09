# CARLA ADAS Simulation Test Platform

A Python-based automated testing platform for ADAS (Advanced Driver Assistance Systems) validation built on CARLA 0.9.16. Supports Euro NCAP scenario testing with CAN bus data simulation output.

## Scenarios

| Scenario | Script | Description | NCAP Category |
|---|---|---|---|
| AEB Car-to-Car | `scenario_aeb_test.py` | Ego follows lead vehicle; lead brakes hard | Car-to-Car AEB |
| Cut-In | `scenario_cutin.py` | Adjacent-lane vehicle cuts into ego's lane | Cut-In Response |
| Pedestrian AEB | `scenario_pedestrian.py` | Pedestrian crosses road in front of ego | Car-to-Pedestrian (CPNA) |
| Batch Testing | `batch_test.py` | Headless parameter sweep (80 combos) | Regression / CI |

## Features

- **Real-time visualization** with Pygame (camera view, HUD, real-time charts)
- **CAN bus simulation** — 7 standard CAN frames (speed, steering, brake, throttle, yaw, acceleration, radar) output in Vector `.asc` format compatible with CANalyzer/CANoe
- **Batch testing framework** — parameter sweep across 80 combinations, heatmap reports + CSV export
- **Persistent AEB braking** — state-locked braking for pedestrian scenarios to prevent ACC re-acceleration
- **Lane-keeping controller** — waypoint-based pure pursuit with PI yaw/cross-track correction
- **Automatic report generation** — 4-panel matplotlib reports with speed, distance, TTC charts and verdict

## Quick Start

### Prerequisites
- CARLA 0.9.16 running on `localhost:2000`
- Python 3.12 with `carla`, `pygame`, `numpy`, `matplotlib`

### Run a Scenario

```bash
# Start CARLA server first
cd E:\CARLA
CarlaUE4.exe

# Run a scenario
python scenario_aeb_test.py    # 2-vehicle AEB (interactive)
python scenario_cutin.py       # 3-vehicle cut-in (interactive)
python scenario_pedestrian.py  # Pedestrian crossing (interactive)
python batch_test.py           # 80-test parameter sweep (headless)
```

**Controls:** `SPACE` = start scenario, `R` = reset, `Q` = quit

### Run Tests

```bash
pip install pytest
python -m pytest test_can_simulator.py -v
```

55 tests covering: CAN ID constants, signal encoding/decoding (uint16/int16/uint8), frame construction, .asc format output, binary round-trip, boundary value clamping, and CANDatalogger file I/O. Tests run without a CARLA server and are automatically executed on every push via GitHub Actions.

### Batch Testing

```bash
python batch_test.py
# Output: test_report/batch_report.png + batch_results.csv
# 80 combinations: 5 speeds x 4 gaps x 4 deceleration rates
```

## CAN Bus Output

The `can_simulator.py` module generates Vector `.asc` format CAN logs with 7 standard CAN IDs at 20 Hz:

| CAN ID | Signal | Resolution |
|---|---|---|
| 0x0C0 | Vehicle Speed | km/h x 100, uint16 |
| 0x0C4 | Steering Angle | deg x 10, int16 |
| 0x1A0 | Brake Pressure | bar x 10, uint16 |
| 0x1A4 | Throttle Position | 0-255, uint8 |
| 0x200 | Yaw Rate | deg/s x 10, int16 |
| 0x220 | Longitudinal Accel | m/s2 x 100, int16 |
| 0x300 | Radar Target | dist(m)x10 + rel_spd(m/s)x10 |

Open `.asc` files in Vector CANalyzer, CANoe, or python-can.

## Project Structure

```
.
├── can_simulator.py           # CAN bus simulation library (reusable)
├── scenario_aeb_test.py       # AEB car-to-car scenario
├── scenario_cutin.py          # Cut-in scenario + CAN logging
├── scenario_pedestrian.py     # Pedestrian AEB + CAN logging
├── batch_test.py              # Headless parameter sweep framework
├── test_can_simulator.py      # pytest unit tests for CAN module
├── .github/workflows/test.yml  # CI: auto-run pytest on push
├── demo_drive.py              # Manual driving demo
├── demo_multisensor.py        # Multi-sensor demo (RGB + semantic + LiDAR)
├── spawn_test.py              # Vehicle spawn diagnostic tool
├── run_*.bat                  # Windows launcher scripts
└── test_report/               # Generated reports and CAN logs
```

## Key Technical Details

- **Spawn fix**: `get_transform().location` returns `(0,0,0)` immediately after `spawn_actor` — CARLA requires ~10 physics ticks for transform sync. All scenarios include this fix.
- **AEB controller**: TTC-based graded braking (light brake < 4s TTC, hard brake < 2s TTC, emergency full brake when distance < safety margin + 3m)
- **Persistent AEB**: State-locked braking in pedestrian scenario prevents ACC from re-accelerating after partial slowdown
- **Vehicle placement**: Ego-first spawn strategy with geometric forward projection for lead, spawn-point filtering for cut-in

## License

MIT

#  Implementation of TRI-axis Attitude Determination, and experimental implementation of Nonlinear Complementary Filter using LSM9DS1 (Acclerometer, Magnetometer, gYro)
## Overview
- The ino files implements TRIAD and multiple complementary filters and prints quaternions to serial.
  - Primary: [triad.ino](triad.ino) and [5filters_unstable.ino](5filters_unstable.ino)
  - Working calibration variant: [triad_working.ino](triad_working.ino)
- PC-side tools read serial data and visualize quaternions as 3D cubes or log raw output:
  - Visualizers: [tp_modifed.py](tp_modifed.py), [tp_monitor.py](tp_monitor.py), [tp_modified_ 5filters.py](tp_modified_ 5filters.py)
  - Serial logger: [IMUserial.py](IMUserial.py)
  - Offline sample data: [serial_output.txt](serial_output.txt)
- Utilities: a small Kalman example in [kalman.py](kalman.py)

## Files & Key Symbols
- [triad.ino](triad.ino)
  - Output helper: [`printQuaternionFlat`](triad.ino)
  - TRIAD implementation: [`getTriadRotation`](triad.ino)
  - Rodrigues + vex helpers: [`rodrigues_exp`](triad.ino), [`vex`](triad.ino)
- [5filters_unstable.ino](5filters_unstable.ino)
  - TRIAD-based multi-filter printing and TRIAD implementation: [`getTriadRotation`](5filters_unstable.ino)
  - Helpers: [`rodrigues_exp`](5filters_unstable.ino), [`vex`](5filters_unstable.ino)
- [triad_working.ino](triad_working.ino)
  - alternative working example and `gyroIntegrate` helper: [`gyroIntegrate`](triad_working.ino)
- [tp_modifed.py](tp_modifed.py)
  - serial input reader: [`serial_read`](tp_modifed.py)
- [tp_modified_ 5filters.py](tp_modified_ 5filters.py)
  - multi-filter visual tool: [`serial_read`](tp_modified_ 5filters.py), [`get_rotation_matrix`](tp_modified_ 5filters.py), [`draw_cube`](tp_modified_ 5filters.py)
- [IMUserial.py](IMUserial.py)
  - simple logger that writes incoming serial to [serial_output.txt](serial_output.txt)
- [kalman.py](kalman.py)
  - `KalmanFilter` class: [`KalmanFilter`](kalman.py)
- [serial_output.txt](serial_output.txt)
  - Example logged numeric output (comma/tab separated) — useful for offline visualization and debugging.

## Quickstart

1. Build and upload firmware to your target (here esp32):
   - Use [triad.ino](triad.ino) or [5filters_unstable.ino](5filters_unstable.ino) depending on desired output mode.
   - The firmware prints quaternions to Serial at 115200 bps using `printQuaternionFlat`.

2. Visualize in realtime:
   - Ensure Python packages installed:
     ```sh
     pip install numpy pygame pyserial
     ```
   - Open one of the visualizers:
     - Single-cube monitor: run  or 
     - 5-filter comparison: run [tp_modified_ 5filters.py](tp_modified_ 5filters.py)
   - The visualizers expect comma-separated quaternion(s) from the serial device. See , , and [`serial_read`](tp_modified_ 5filters.py) for parsing details.

Kalman demo:
   -  includes a small  class  and an example main loop that reads comma-separated accel/gyro lines.

## Data format notes
- Firmware prints quaternions as  (comma separated). Multi-filter firmware prints multiple quaternion sets in a single line (see  print logic).
- See the captured example data in .

## Tips & Troubleshooting
- Serial port: change the port string in Python scripts (`/dev/ttyUSB0`) to match the system.
- If visualization shows flipped axes or inverted cubes, inspect the DCM creation in  / [`get_rotation_matrix`](tp_modified_ 5filters.py) and remapping in the Arduino TRIAD (e.g., body remap in triad.ino).
- Use the logged  for offline plotting and debugging.

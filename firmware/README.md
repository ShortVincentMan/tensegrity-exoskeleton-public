# Firmware projects

Each project has its own `platformio.ini` and entry point. The organization preserves firmware behavior; historical code is retained separately.

| Project | Purpose | USB serial baud |
| --- | --- | --- |
| `controller` | Current modular controller, including the existing local changes | 921600 |
| `imu-trial-logger` | SURP IMU collection and controller logging sketch | 115200 |
| `emg-trial-logger` | Preserved Day 41 logger from the existing repository | 921600 |
| `calibration/hinge-axis` | Historical hinge-axis calibration sketch | 115200 |
| `examples/dual_imu_angle_reader_test` | Independent dual-IMU communication test | 115200 |

Build from the repository root with `pio run -d firmware/<project>`, or open that project folder in PlatformIO. Install PlatformIO and use the configured ESP32 board target. Generated `.pio` outputs are ignored.

The controller headers and sources moved together. `day41.cpp` now lives as `emg-trial-logger/src/main.cpp`; it remains separate from the modular controller. The example's second historical entry point is preserved under `archive/firmware/dual-imu-controller-snapshot/` rather than compiled alongside the sensor test.

The SURP Day 33 sketch differs from the repository's archived version. Both remain under `archive/firmware/`. The original SURP Day 41 copy is also retained there; its checked difference from the repo version is a title comment.

Historical collection firmware is a candidate source association, not a proven exact binary used for each trial. Record future firmware hashes in trial metadata. Detailed wiring is in the [original README](../archive/documentation/README-before-research-organization.md).

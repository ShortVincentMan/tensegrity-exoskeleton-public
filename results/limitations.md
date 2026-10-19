# Measurement and interpretation limits

The prototype demonstrates system integration. Controller activation did not establish effective physical assistance, and the existing EMG exports do not establish reduced muscle effort.

## EMG

Exported timing implies approximately 51.2 Hz where usable, with constant/shared timestamps in some files. This is not verified acquisition rate or bandwidth. Channel placement, signal type, gain, electrode contact, clipping, and motion artifacts remain unresolved. Whole-record means combine unequal rest/movement intervals. The 100 ms CH2 processor and poster comparison are distinct outputs; preserve their method notes.

## IMU and motor output

Relative rotation is an unvalidated elbow-angle proxy. Sensor freshness does not establish anatomical accuracy. ASSIST is a controller state, and emitted steps count firmware pulses rather than measured motor rotation or torque. Participants can voluntarily move during recorded assistance. The logs do not isolate motor-only lifting effectiveness.

Trial 4 contains an abrupt orientation change; trials 5 and 6 contain brief excursions. Existing analysis retains all samples without assigning a cause. An incomplete ASSIST interval has unknown duration.

## Experimental scope

The EMG and IMU recordings are separate, not synchronized. This is pilot prototype research, not a clinical efficacy result. Future work should verify timing and signal bandwidth, measure cable force, improve actuation and pretension, validate anatomical angles, and standardize synchronized recording intervals.

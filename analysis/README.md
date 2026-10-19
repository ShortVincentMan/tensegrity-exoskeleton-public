# Research analysis

The EMG and IMU studies are independent. Their files are not synchronized, and their analysis pipelines must not be combined into a single time series.

## Python

Install `analysis/requirements.txt` into a virtual environment. The recorder uses pyserial; the IMU analysis uses NumPy, pandas, and Matplotlib.

```sh
python analysis/imu/analyze_imu.py --output analysis/reproduced/imu
```

Its default inputs resolve relative to the repository, so the command also works from another working directory. Optional `--input` and `--output` paths override the defaults. The broad dependency ranges support installation; they are not an exact historical environment lock.

For new acquisition, read the [IMU protocol](../experiments/imu-controller-demo/protocol.txt). The recorder requires `--port`, `--out`, `--trial`, and `--load`; its load argument is recorded in pounds. Choose a new output filename and preserve the `.raw.log` sidecar. USB baud differs from the older EMG logger.

## MATLAB

`emg/process_all_emg.m` is the CH2 pipeline. Select `experiments/emg-condition-comparison/raw/spreadsheets/condition-trials` when prompted. The folder contains exactly the 12 condition spreadsheets. The separate Assist_Trial is not part of that four-condition comparison.

The script creates `emg_processed` beside the chosen inputs. Imported historical CH2 outputs are under the experiment's `processed/ch2-rms/` directory; new outputs should be compared before replacing historical results. Whole-record DC removal and rolling RMS run with base MATLAB. Optional bandpass filtering requires Signal Processing Toolbox and a verified suitable signal rate.

`emg/combined-channel-preview/` preserves the separate `Process_EMG.m` and `Plot_EMG_Comparison.m` pipeline. Its inputs are under the experiment's `derived/combined-channel-preview/` directory. Copy that pipeline's scripts into a temporary copy of its input folder to run them together as originally instructed; keep reproduction outputs separate. Read its original `READ_ME.txt` first.

The `.mlx` files in `legacy/emg/` are earlier methods, not the preferred pipeline. Their assumptions about timing, filtering, and duplicate timestamps require review.

from pathlib import Path
import argparse
import zipfile

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D


REPO_ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=REPO_ROOT / "experiments/imu-controller-demo/raw")
    parser.add_argument("--output", type=Path, default=REPO_ROOT / "experiments/imu-controller-demo/processed")
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)

    paths = sorted(args.input.glob("t*AS_NW*.csv"))

    if len(paths) != 6:
        raise ValueError(
            f"Expected six AS_NW CSV files in {args.input}; found {len(paths)}."
        )

    data = []
    summaries = []
    prepared = []

    for trial, path in enumerate(paths, 1):
        d = pd.read_csv(path)

        if len(d) < 2:
            raise ValueError(f"Not enough readings: {path.name}")

        if (
            d.time_ms.isna().any()
            or (d.time_ms.diff().dropna() <= 0).any()
        ):
            raise ValueError(f"Invalid timestamps: {path.name}")

        # Convert milliseconds to seconds, starting at zero.
        t = (d.time_ms - d.time_ms.iloc[0]) / 1000

        # Find the logged ASSIST interval.
        active = d.assist_active.eq(1)

        edges = np.diff(
            np.r_[False, active.to_numpy(), False].astype(int)
        )

        starts = np.where(edges == 1)[0]
        stops = np.where(edges == -1)[0]

        if len(starts) != 1:
            raise ValueError(
                f"Expected one ASSIST interval: {path.name}"
            )

        start = starts[0]
        stop = stops[0]
        dt = d.time_ms.diff().dropna()

        # If the recording ends during ASSIST, its end time is unknown.
        assist_off = t.iloc[stop] if stop < len(d) else np.nan
        assist_duration = (
            assist_off - t.iloc[start]
            if stop < len(d)
            else np.nan
        )

        if trial == 4:
            review_note = (
                "Abrupt upper IMU orientation change near 19.5 s; "
                "global peak includes later motion"
            )
        elif trial == 5:
            review_note = (
                "Brief angle excursion near 9.2 s; "
                "inspect movement/sensor mounting"
            )
        elif trial == 6:
            review_note = (
                "Brief angle excursion near 7.5 s; "
                "inspect movement/sensor mounting"
            )
        else:
            review_note = ""

        summaries.append(
            dict(
                trial=trial,
                source_file=path.name,
                samples=len(d),
                elapsed_span_s=t.iloc[-1],
                logged_rate_hz=1000 / dt.median(),
                min_log_interval_ms=dt.min(),
                max_log_interval_ms=dt.max(),
                missing_cells=int(d.isna().sum().sum()),
                stale_sensor_rows=int(
                    (d.sensors_fresh != 1).sum()
                ),
                upper_max_age_ms=d.upper_age_ms.max(),
                forearm_max_age_ms=d.forearm_age_ms.max(),
                skipped_log_rows_counter_max=(
                    d.skipped_log_rows.max()
                ),
                filtered_min_deg=d.filtered_angle_deg.min(),
                filtered_peak_deg=d.filtered_angle_deg.max(),
                filtered_span_deg=(
                    d.filtered_angle_deg.max()
                    - d.filtered_angle_deg.min()
                ),
                final_filtered_angle_deg=(
                    d.filtered_angle_deg.iloc[-1]
                ),
                assist_intervals=len(starts),
                assist_on_s=t.iloc[start],
                assist_off_s=assist_off,
                assist_logged_duration_s=assist_duration,
                assist_start_angle_deg=(
                    d.filtered_angle_deg.iloc[start]
                ),
                assist_last_active_angle_deg=(
                    d.filtered_angle_deg.iloc[stop - 1]
                ),
                assist_angle_change_deg=(
                    d.filtered_angle_deg.iloc[stop - 1]
                    - d.filtered_angle_deg.iloc[start]
                ),
                assist_target_deg=(
                    d.assist_target_deg.iloc[start]
                ),
                largest_filtered_step_deg=(
                    d.filtered_angle_deg.diff().abs().max()
                ),
                review_note=review_note,
            )
        )

        q = d.copy()
        q.insert(0, "source_file", path.name)
        q.insert(1, "file_trial", trial)
        q.insert(2, "elapsed_s", t)

        prepared.append(q)
        data.append((trial, d, t, start, stop))

    # Save numerical results.
    summary = pd.DataFrame(summaries)
    combined = pd.concat(prepared, ignore_index=True)

    summary.to_csv(
        args.output / "imu_trial_summary.csv", index=False
    )
    combined.to_csv(
        args.output / "imu_all_trials.csv", index=False
    )

    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 11,
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )

    blue = "#225a95"
    green = "#42a576"
    brown = "#8b5b23"

    # ---------------------------------------------------------
    # Figure 1: all six recordings
    # ---------------------------------------------------------
    fig, axes = plt.subplots(
        3, 2,
        figsize=(12, 9),
        sharex=True,
        sharey=True,
    )

    for ax, (trial, d, t, start, stop) in zip(
        axes.flat, data
    ):
        interval_end = (
            t.iloc[stop] if stop < len(d) else t.iloc[-1]
        )

        ax.plot(
            t,
            d.filtered_angle_deg,
            color=blue,
            linewidth=1.8,
        )

        ax.axvspan(
            t.iloc[start],
            interval_end,
            color=green,
            alpha=0.18,
            linewidth=0,
        )

        ax.set_title(
            f"Trial {trial}",
            loc="left",
            fontweight="bold",
        )
        ax.grid(alpha=0.16)

        if trial == 4:
            ax.annotate(
                "Abrupt sensor change",
                xy=(19.48, 48.43),
                xytext=(14.4, 85),
                fontsize=9,
                arrowprops={
                    "arrowstyle": "->",
                    "color": "#a94442",
                },
                color="#a94442",
            )

        ax.set_xlim(0, max(30, combined.elapsed_s.max()))
        ax.set_ylim(0, 90)

    for ax in axes[-1]:
        ax.set_xlabel("Elapsed time (s)")

    for ax in axes[:, 0]:
        ax.set_ylabel("Relative IMU angle (degrees)")

    fig.suptitle(
        "Arm motion while wearing the exoskeleton | "
        "No external weight",
        fontsize=16,
        fontweight="bold",
        y=0.985,
    )

    fig.legend(
        handles=[
            Line2D(
                [0], [0],
                color=blue,
                linewidth=2,
                label="Logged filtered angle",
            ),
            Patch(
                facecolor=green,
                alpha=0.18,
                label="Controller ASSIST state",
            ),
        ],
        loc="upper center",
        bbox_to_anchor=(0.5, 0.954),
        ncol=2,
        frameon=False,
    )

    fig.text(
        0.5,
        0.018,
        "Six separate recordings. Relative to the logged zero; "
        "anatomical elbow calibration has not been verified.",
        ha="center",
        fontsize=10,
        color="#555555",
    )

    fig.tight_layout(rect=(0, 0.04, 1, 0.91))
    fig.savefig(
        args.output / "imu_six_trials.png",
        dpi=220,
    )
    plt.close(fig)

    # ---------------------------------------------------------
    # Figure 2: compact poster chart, Trial 3
    # ---------------------------------------------------------
    trial, d, t, start, stop = data[2]

    compact_style = {
        "font.size": 8,
        "axes.titlesize": 9,
        "axes.labelsize": 8,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "legend.fontsize": 6,
    }

    # Apply font settings before creating the chart.
    with plt.rc_context(compact_style):
        fig, ax = plt.subplots(figsize=(5, 3.2))

        ax.plot(
            t,
            d.filtered_angle_deg,
            color=blue,
            linewidth=1.8,
        )

        interval_end = (
            t.iloc[stop] if stop < len(d) else t.iloc[-1]
        )

        ax.axvspan(
            t.iloc[start],
            interval_end,
            color=green,
            alpha=0.18,
            linewidth=0,
        )

        # Show the controller target only during ASSIST.
        target = d.assist_target_deg.iloc[start]

        ax.plot(
            [t.iloc[start], interval_end],
            [target, target],
            linestyle="--",
            color=brown,
            linewidth=1.2,
        )

        ax.set(
            xlabel="Time (s)",
            ylabel="Relative IMU angle (°)",
            xlim=(0, max(28, t.iloc[-1])),
            ylim=(0, 85),
        )

        ax.set_title(
            "Arm motion and controller activation",
            loc="left",
            fontweight="bold",
            pad=8,
        )

        ax.grid(alpha=0.16)

        ax.legend(
            handles=[
                Line2D(
                    [0], [0],
                    color=blue,
                    linewidth=1.8,
                    label="Filtered angle",
                ),
                Patch(
                    facecolor=green,
                    alpha=0.18,
                    label="ASSIST state",
                ),
                Line2D(
                    [0], [0],
                    color=brown,
                    linestyle="--",
                    label="Controller target",
                ),
            ],
            loc="upper right",
            frameon=False,
        )

        # The caption is saved separately to keep the chart compact.
        fig.tight_layout(pad=0.6)

        fig.savefig(
            args.output / "imu_poster_trial3.png",
            dpi=300,
        )
        fig.savefig(
            args.output / "imu_poster_trial3.svg",
        )
        plt.close(fig)

    # ---------------------------------------------------------
    # Notes and poster caption
    # ---------------------------------------------------------
    notes = """IMU ANALYSIS - SIX ARM-ON, NO-WEIGHT RECORDINGS

Sources:
The six supplied t1-t6 AS_NW CSVs. Source filenames distinguish
trials; internal trial_id values are repeated. No external weight
was added.

Processing:
All rows retained. The logged filtered_angle_deg column was used
without additional smoothing, interpolation, or zero correction.
Elapsed time = (time_ms - first time_ms) / 1000.
Trials were not averaged or aligned by movement phase.

Controller state:
ASSIST shading identifies the logged controller state. It does not
establish delivered force or effective physical assistance.
Angle change uses the first and last active samples.
If recording ends during ASSIST, its duration is left unknown.

Timing and angle:
CSV logging rate does not establish each IMU's native update rate.
Relative IMU angle is not independently calibrated anatomical
elbow angle. Sensor freshness does not establish angle accuracy.

Review:
Trial 4 has an abrupt upper-IMU orientation change near 19.5 s.
Its global peak includes later motion.
Trials 5 and 6 have brief angle excursions.
No readings were removed and no cause was assigned.

Participant observations:
The participant reported no noticeable lifting assistance.
The participant deliberately lowered the arm during spool release
to maintain string tension. The decreasing angle therefore cannot
be treated as an isolated measurement of assistance effectiveness.

Interpretation:
The recordings document motion sensing and controller activation.
They do not establish reduced muscle effort or effective lifting
assistance. Emitted steps are firmware pulse counts, not measured
motor rotation or delivered torque.

Poster chart:
Trial 3 is displayed individually, not as an ensemble mean.
The six-trial overview preserves every recording for review.
"""

    caption = (
        "Trial 3: arm-on motion with no external weight. "
        "Shading marks the controller ASSIST state; the dashed "
        "line marks its target. Angle is relative to the logged "
        "zero, not independently calibrated anatomical elbow angle. "
        "The participant deliberately lowered the arm during spool "
        "release to maintain cable tension."
    )

    (args.output / "imu_analysis_notes.txt").write_text(
        notes, encoding="utf-8"
    )
    (args.output / "imu_poster_caption.txt").write_text(
        caption, encoding="utf-8"
    )

    # ---------------------------------------------------------
    # ZIP: include the running script from its actual location
    # ---------------------------------------------------------
    output_names = [
        "imu_trial_summary.csv",
        "imu_all_trials.csv",
        "imu_six_trials.png",
        "imu_poster_trial3.png",
        "imu_poster_trial3.svg",
        "imu_analysis_notes.txt",
        "imu_poster_caption.txt",
    ]

    with zipfile.ZipFile(
        args.output / "imu_analysis_bundle.zip",
        "w",
        zipfile.ZIP_DEFLATED,
    ) as archive:
        archive.write(
            Path(__file__).resolve(),
            arcname="analyze_imu.py",
        )

        for name in output_names:
            archive.write(
                args.output / name,
                arcname=name,
            )

    print(
        summary[
            [
                "trial",
                "elapsed_span_s",
                "filtered_peak_deg",
                "assist_logged_duration_s",
                "assist_angle_change_deg",
            ]
        ].to_string(index=False)
    )

    print(f"\nTotal samples: {len(combined)}")
    print(f"Outputs saved to: {args.output.resolve()}")


if __name__ == "__main__":
    main()
# Research methods and data provenance

Two studies are maintained independently: an EMG condition comparison and an IMU controller demonstration. Original recordings remain separate from derived tables, processing spreadsheets, and presentation figures.

## Data lineage

Each inventoried SURP file has a SHA-256 source hash, source-relative path, import destination or retention decision, and publication status in `provenance/surp-import-manifest.json`. A destination hash records any explicit organizational adaptations. `exact-duplicates.json` contains byte-identical file groups; shared filenames alone do not establish duplication.

Each experiment's metadata retains filename-based identity and condition labels as recorded. Unknown participant identity, exact firmware association, acquisition settings, and setup details remain unknown rather than being inferred from timestamps or folder names. Session export folders are source provenance, not independently verified acquisition dates.

## Analysis

The IMU script reads six CSVs and reports motion, logging intervals, controller state, and quality notes. It does not claim calibrated anatomy or force measurement. The CH2 MATLAB pipeline processes 12 original condition spreadsheets without cross-trial alignment or averaging; its time interpretation and RMS windows are documented beside the outputs.

The two-channel preview and poster comparison have their own derived inputs and methods. Neither is a replacement for original Shimmer acquisition. See each experiment README and the original analysis notes before using a result.

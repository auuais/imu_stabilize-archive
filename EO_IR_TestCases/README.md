# EO/IR Panorama Stabilization Test Cases

This directory contains various test scenarios and configurations for both Electro-Optical (EO) and Infrared (IR) panorama stabilization and blending algorithms.

## Running the Tests

To run any individual test case:
1. Navigate into the specific test case directory (e.g., `EO_Test2_...`).
2. Copy the `parameters_unified.ini` file located inside that test folder. **change the folder paths to match local pathcs
3. Paste it into the `Release` folder (or wherever the main executable is located) prior to running the application.

## Test Setup and Methodology

* **Alignment Verification**: Tests were performed by placing the marker target exactly at the parallax point in most cases. This ensures proper geometry for verifying both vertical and horizontal alignment correctness.
* **Seam Evaluation**: Targets were specifically placed at the inter-camera seam limits to test the handling of the blending boundaries and continuity.

## High Tilt Angles & Adaptive Clamping Logic

The following directories contain edge-cases where the starting tilt angle is greater than $5^\circ$:
* `EO_IR_TestCases\IR_Test5_R_0.75_P_6.25`
* `EO_IR_TestCases\EO_Test6_R_1.25_P_6.5`

**Stabilization Logic (`stabilization=true`)**:
Extreme structural shifts have the potential to violate bounded projection limits. When stabilization is activated, the application enforces a robust $5^\circ$ clamping safety constraint:
1. The raw IMU configuration (roll, pitch, yaw) is read without immediate truncation.
2. The logic simulates projecting the rotation matrices to all individual cameras to find the per-camera effective pitch and roll shifts.
3. The absolute maximum individual angle across all 6 views is determined.
4. If this highest per-camera angle exceeds $5.0^\circ$, a unified scaling factor is calculated as `5.0 / max_camera_angle`.
5. The raw sensor measurements are proportionally scaled down by this factor. This guarantees that the geometric proportion of the 3D plane shifts is preserved while strictly ensuring no single individual camera exceeds exactly $5^\circ$ of drift.

## Blending Region Fixes

Minor code modifications have been built into the rendering process to fix a known bug in the blending cross-dissolve areas. Previously, a slight color tint was visible across overlapping regions due to the mishandling of chroma (color) accumulations. The logic now safely restricts chroma writing over even pixels properly aligning with the strict YUV 4:2:2 downsampling specifications, eliminating color bleeding/tinting in the seam limits.

## Reprojection Accuracies

To guarantee the quality of the algorithmic projection models, the reprojection Root Mean Square Error (RMSE) values have been successfully assessed for the underlying distortion correction passes.

* **Electro-Optical (EO)**: Reprojection RMSE (px): **`0.198`**
* **Infrared (IR)**: Reprojection RMSE (px): **`0.128`**

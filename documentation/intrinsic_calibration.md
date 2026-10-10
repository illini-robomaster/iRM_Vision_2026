# Browser-based camera intrinsic calibration

This tool requires a camera and a symmetric circle-grid target, **not a gimbal,
IMU, CBoard, neural network, or TensorRT engine**. It supports the repository's
MindVision, HikRobot, and video-file camera backends. Existing calibration
programs are unchanged.

## Build and start

```bash
cmake -S /home/irm/iRM-Vision-2026 -B /home/irm/iRM-Vision-2026/build
make -C /home/irm/iRM-Vision-2026/build intrinsic_calibration_gui intrinsic_calibration_test -j2
/home/irm/iRM-Vision-2026/build/intrinsic_calibration_gui \
  -c=/home/irm/iRM-Vision-2026/configs/mv_sua133gc.yaml \
  -b=127.0.0.1 -p=8081 \
  -o=/home/irm/iRM-Vision-2026/records/calibration
```

Open `http://127.0.0.1:8081/` in a browser. For a remote Jetson, preferably use
SSH forwarding from the laptop:

```bash
ssh -L 8081:127.0.0.1:8081 irm@JETSON_IP
```

Then open the same localhost address on the laptop. Alternatively bind with
`-b=0.0.0.0` and open `http://JETSON_IP:8081/` on a **trusted LAN only**.
There is no authentication or TLS; do not expose this service to the Internet.
The default bind address is loopback. Stop with **Ctrl+C** to release the camera;
do not force-kill a live MindVision process. Only one process may open the camera.

The configuration needs camera keys only. Optional `pattern_cols`,
`pattern_rows`, and `center_distance_mm` initialize the UI; defaults are 10, 7,
and 40 mm. These can be changed on the page. A MindVision configuration needs
`camera_name`, `exposure_ms`, `gamma`, and `vid_pid`; retain the desired fixed
pipeline settings from the actual camera configuration.

## Workflow

1. Use a flat, black-on-white **symmetric circle grid**, not a chessboard. Enter
   circle counts and measured center-to-center spacing (not circle diameter).
2. Confirm the actual resolution and camera settings. Keep resolution, ROI,
   focus, and lens configuration fixed throughout acquisition and later use.
3. Move the board across the image, including the edges, and vary distance and
   tilt. Let the board settle before clicking **采集当前帧**.
4. Capture 15–25 diverse sharp views. The application permits solving from 5
   views, but that is only an input minimum, not a quality guarantee. Duplicate
   views are not automatically rejected. Position coverage is only a heuristic.
5. Click **开始标定**. Computation runs in a background job; sample mutation is
   locked while solving. The model matches the existing program's `CALIB_FIX_K3`.
6. Inspect RMS, mean Euclidean point error, and per-view errors. Delete unsuitable
   samples and solve again. Try the undistortion preview; no automatic accuracy
   threshold declares a real camera calibration valid.
7. Download `intrinsics.yaml`. Copy `camera_matrix` and `distort_coeffs` into the
   actual robot configuration only after verification. No configuration is
   overwritten automatically. The existing Solver expects nine row-major matrix
   elements and five coefficients `[k1,k2,p1,p2,k3]`.

The browser uses periodically refreshed JPEG snapshots, not a full-rate MJPEG
stream. Preview images may be resized; detection, calibration, and saved PNG
images use original image resolution. Sample thumbnails can be enlarged by
clicking them. The single HTTP request worker has bounded receive/send timeouts;
this is a local calibration tool, not a production multi-user web server.

## Outputs and validity

Each launch creates a unique session directory under the output root:

```text
records/calibration/YYYY-MM-DD_HH-MM-SS_N/
  1.png, 2.png, ...   # raw images; IDs are stable and need not be continuous
  samples.yaml        # active samples, pattern, image timestamps, optional pose
  intrinsics.yaml     # current result, resolution, overall and per-view errors
```

Deleting a sample removes its PNG. Changing the target specification requires
confirmation and clears the current samples. Any sample change invalidates the
result and removes `intrinsics.yaml`, so stale results cannot be downloaded.
Samples already saved on disk are not automatically reloaded on restart.
Use `-i=<directory>` to review/import images into a new session; filenames are
sorted lexicographically (zero-pad numeric names). This mode opens no camera.
Click **下一张离线图片** to advance, then capture valid views manually.

The output YAML includes `camera_matrix`, `distort_coeffs`, `image_width`,
`image_height`, `rms_px`, `mean_error_px`, `per_view_mean_px`, and
`per_view_rms_px`. Per-view arrays follow the active sample order in
`samples.yaml`.

## Hand–eye extension contract

`/home/irm/iRM-Vision-2026/calibration/intrinsic_session.hpp` defines a
`PoseProvider` with `orientation_at(image_timestamp)` returning an optional
body-to-reference quaternion in **wxyz** order. `Sample` contains the image,
detected points, timestamp, and optional quaternion. The GUI's provider is null
and `handeye_available` is false: no IMU connection or hand–eye solve is enabled.

The manifest records host `steady_clock` nanoseconds, meaningful only within
the acquisition process, not wall-clock time or a cross-machine sync protocol.
Missing orientations are YAML null, never synthetic identity quaternions.
A future adapter must implement time alignment, coordinate conventions,
timeouts and availability reporting before enabling hand–eye controls. This
manifest is not directly consumable by the legacy numbered JPG/TXT loader;
a future export adapter is needed. See
`/home/irm/iRM-Vision-2026/documentation/cboard_requirements.md` for the current
CAN interface requirements.

## Camera-free verification

```bash
/home/irm/iRM-Vision-2026/build/intrinsic_calibration_test
python3 /home/irm/iRM-Vision-2026/scripts/test_intrinsic_gui.py
node --check /home/irm/iRM-Vision-2026/calibration/web/app.js
```

The C++ test recovers known intrinsics from synthetic projected points, checks
error metrics/YAML/input rejection, and detects a rendered circle grid. The
standard-library Python test generates offline fixtures, starts the server,
and exercises acquisition, deletion, target settings, background calibration,
undistortion, export, stale-result invalidation, and normal SIGINT shutdown.
These tests do not establish the accuracy of a real camera calibration.
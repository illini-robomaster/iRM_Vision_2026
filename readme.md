# sp_vision_25 - Camera Pipeline Bring-up (MindVision)

> **Scope of this file.** For now this README documents exactly **one** path: bringing up the
> **MindVision camera pipeline** and proving that the imaging pipeline is *pinned* (every ISP
> parameter explicitly written, then read back). The complete project documentation - architecture,
> auto-aim theory, WSL/x86_64 replay setup, inference backends (TensorRT / ONNX Runtime / OpenCV
> DNN), every other test program - lives in [`readme_zh.md`](readme_zh.md) (Chinese), which is the
> current full Chinese README. More sections will be ported here over time.
>
> Engineering rules for this port live in [`AGENTS.md`](AGENTS.md): section 8.7 (detection-frequency
> test tool + MindVision keys) and section 8.8 (headless streaming + pinned pipeline).

## 1. The one pipeline

```
MindVision USB camera (MV-SUA133GC-T1V-C, f622:0001, 1280x1024 Bayer8)
  -> io::MindVision::open()          io/mindvision/mindvision.cpp   (MindVision SDK)
  -> pinned ISP pipeline             explicit write + readback of 18 parameters
  -> io::Camera::read() -> cv::Mat   consumers: detect_freq_visual_test, camera_test, uav, ...
```

| Piece | Value |
|---|---|
| Camera backend | `camera_name: "mindvision"` in the yaml |
| Config file | `configs/mv_sua133gc.yaml` |
| Driver | `io/mindvision/mindvision.cpp` (`io::MindVision`) |
| Bring-up program | `./build/detect_freq_visual_test` (`tests/detect_freq_visual_test.cpp`) |

`configs/mv_sua133gc.yaml` is a **standalone** config for this USB2.0 test camera: camera section +
detector section only. The end-to-end programs (`uav`, `minimum_vision_system`) additionally need
calibration / planner / cboard keys, so use a full config (`configs/demo.yaml` etc.) and swap in the
camera section from `configs/mv_sua133gc.yaml`.

## 2. Prerequisites

- Linux **x86_64 or aarch64** (verified on a Jetson Orin Nano, JetPack 5.1.3).
- MindVision SDK: nothing to install, it is vendored in this repo - headers in
  `io/mindvision/include/` and `io/mindvision/lib/arm64/libMVSDK.so` (or `lib/amd64/` on x86_64),
  selected by `io/CMakeLists.txt` from `CMAKE_SYSTEM_PROCESSOR`. Only libusb has to be installed.
- Base libraries:

  ```bash
  sudo apt install -y libopencv-dev libfmt-dev libeigen3-dev libspdlog-dev libyaml-cpp-dev \
      libusb-1.0-0-dev
  ```

- Camera plugged in. A USB2.0 port is enough: at 1280x1024 Bayer8 the stream is roughly 1.25 MB per
  frame (~45 MB/s), which already saturates a 480 Mbps link and still yields ~35.7 fps. Exactly one
  process may hold the camera at a time.

## 3. Build

```bash
cmake -B build                                   # only needed once; build/ is usually configured
make -C build/ detect_freq_visual_test -j$(nproc)
make -C build/ camera_test -j$(nproc)            # optional: capture-only minimal program
```

## 4. Start the pipeline

```bash
# with a display (X / WSLg):
./build/detect_freq_visual_test -c=configs/mv_sua133gc.yaml -m=live -d

# on a headless machine (Jetson over SSH) - stream the image, see section 5:
./build/detect_freq_visual_test -c=configs/mv_sua133gc.yaml -m=live -stream=8080 -no-yolo
```

Flags used above and nearby (full table in `AGENTS.md` 8.7):

| Flag | Meaning |
|---|---|
| `-m=live` | run until stopped with Ctrl+C |
| `-m=bench -n=60` | process 60 frames, print the stage summary, exit |
| `-no-yolo` | do not load the detector: measure capture / draw only |
| `-stream=<port>` | MJPEG live view (section 5) |
| `-save=/tmp/shot.png` | write the canvas (image + frequency curves) to a file |
| `-w=<frames>`, `-interval=<frames>` | statistics window length, log period |

Warning: every program in this repo parses arguments with `cv::CommandLineParser`, so a short option
must be written as `-c=<path>`. A space (`-c <path>`) parses as an empty value and aborts with
`[YAML] Failed to load file: bad file`.

## 5. Watching the image without a monitor (headless MJPEG)

```bash
./build/detect_freq_visual_test -c=configs/mv_sua133gc.yaml -m=live -stream=8080
curl -s -o /tmp/snap.jpg http://127.0.0.1:8080/snapshot.jpg     # script self-check
# browser / VLC / ffplay on any machine that can reach the board:
#   http://<board-ip>:8080/
```

The startup log lists every local IPv4 address together with its URL. The server lives in
`tools/mjpeg_server.{hpp,cpp}` (POSIX sockets + `cv::imencode`, no third-party dependency):

| Path | Behaviour |
|---|---|
| `/` or `/index.html` | `multipart/x-mixed-replace; boundary=frame` live stream |
| `/snapshot.jpg` | single JPEG of the latest frame |
| anything else (including `/favicon.ico`) | `404` |
| more than 8 clients | `503` |

- JPEG encoding happens only while at least one client is connected, and each client has its own
  thread sharing the newest frame, so a slow client drops frames instead of stalling the pipeline.
- `-stream` without an explicit `-d` disables `cv::imshow` automatically. If `imshow` throws anyway
  (no `DISPLAY`), it is caught, logged as a warning, and the program keeps running.
- Measured on a 1280x1194 canvas: 10.2 ms/frame with no client, 16.1 ms/frame with one client
  (JPEG quality 80 costs about 6 ms). Both stay below the 28 ms camera frame interval, so throughput
  remains camera-bound: 35.7 fps captured, 34.8 fps end to end, no dropped frames.

## 6. Verify that the pipeline is pinned

`io::MindVision::open()` writes every `mv_*` parameter present in the yaml, in a fixed order, and
then reads them all back into the startup log (search the log for the readback block; its labels are
Chinese on the device). With `configs/mv_sua133gc.yaml` the readback must show:

| Parameter | Expected value |
|---|---|
| Resolution | 1280x1024 (camera preset) |
| Frame speed mode | 1 |
| Output format | BGR8 (index listed in the format list printed at startup) |
| Trigger mode | 0 (continuous) |
| Target frame rate | 0 Hz (unlimited) |
| Auto exposure | off |
| Exposure time | 1996.2 us (from `exposure_ms: 2`) |
| Gamma | 50 (from `gamma: 0.5`, expressed in hundredths) |
| Analog gain | 64 (`mv_analog_gain`) |
| Digital gain | R 100 G 100 B 100 |
| White balance mode | manual |
| Colour temperature mode | 1 (preset) |
| Colour temperature gain | R 100 G 100 B 100 (`mv_clr_temp_gain`) |
| Sharpness | 0 (`mv_sharpness`) |
| Contrast | 100 (`mv_contrast`) |
| Saturation | 100 (`mv_saturation`) |
| Anti-flicker | off (`mv_anti_flick`) |
| Light frequency | 50 Hz (`mv_light_frequency`) |

**Consistency check** - run twice and compare the startup block. The readback block sits inside the
first 40 lines, before the first per-frame line containing `fps`, so a fixed-width head is enough:

```bash
./build/detect_freq_visual_test -c=configs/mv_sua133gc.yaml -m=bench -n=60 -no-yolo > /tmp/rb1.log 2>&1
./build/detect_freq_visual_test -c=configs/mv_sua133gc.yaml -m=bench -n=60 -no-yolo > /tmp/rb2.log 2>&1
for f in 1 2; do sed -E 's/^\[[^]]*\] \[[^]]*\] //' /tmp/rb$f.log | head -40 > /tmp/rb$f.txt; done
diff /tmp/rb1.txt /tmp/rb2.txt && echo IDENTICAL
```

On the Jetson this prints `IDENTICAL` (empty diff). If your camera prints more startup lines, raise
`head -40` until it covers the whole readback block, but keep it before the first `fps` line. Any
difference between two runs means something else is still changing the imaging.

**Rules that came out of measurement - do not undo them:**

- Ordering: `CameraSetParameterMode` makes the SDK reload its parameter sheet, which silently resets
  the exposure side to the defaults (auto exposure on, about 10 ms, gamma 100). Therefore
  `setup_parameter_source()` runs *before* the exposure settings in `io::MindVision::open()`, while
  auto-exposure / exposure / gamma run *after* the structural settings (output format, trigger,
  frame speed, gains, resolution). The readback block exposes any regression immediately.
- Unsupported on this camera model: `CameraSetWbMode` and `CameraSetFrameRate` return `-4` on the
  MV-SUA133GC, so `mv_wb_mode` and `mv_frame_rate` must stay commented out (every start would log a
  warning). Both still appear in the readback (manual white balance, 0 Hz = unlimited), because
  their values are read, not written.
- Golden sheet (copy the imaging to another machine): set
  `mv_parameter_save_file: "assets/mv_sua133gc_pipeline.config"` to dump the effective parameters
  on exit (about 74 KB) and `mv_parameter_file` to the same path to restore them at startup. Both go
  through the same reload path as above, so the ordering rule applies. That `.config` file is not
  committed to the repository.
- To test whether a write actually took effect, use a temporary yaml with deliberately different
  values (`mv_analog_gain: 96`, `mv_sharpness: 5`, `mv_light_frequency: 1`, ...) and check the
  readback. Keeping a single key in the file is enough to tell "write rejected" apart from
  "overwritten by a later call".

## 7. MindVision config keys used by this pipeline

Every `mv_*` key has a default, so configs that do not mention them behave exactly as before.

| Key | Default | Meaning |
|---|---|---|
| `camera_name` | required | selects the backend: `"mindvision"` (this file), `"hikrobot"`, or `"video"` (replay a file) |
| `vid_pid` | required | expected VID:PID; a mismatch is logged as a warning together with the real value (this camera is `f622:0001`, while the repo default in `configs/camera.yaml` is `f622:d13a`) |
| `exposure_ms` | required | exposure in ms (sent to the SDK in us) |
| `gamma` | required | gamma, sent to the SDK in hundredths |
| `mv_device_index` | `0` | which enumerated device to open (several cameras plugged in) |
| `mv_friendly_name` | `""` | pick the device by nickname; non-empty takes precedence over `mv_device_index` |
| `mv_frame_speed` | `1` | SDK frame-speed mode: 0 low, 1 normal, 2 high |
| `mv_resolution_width` / `mv_resolution_height` | `-1` | `-1` keeps the camera preset; changing it invalidates the calibration |
| `mv_media_type` | `-1` | raw output format index (list printed at startup), `-1` leaves it unchanged |
| `mv_gain` | `-1` | digital gain in SDK units (100 = 1.0x), `-1` leaves it unchanged |
| `mv_frame_timeout_ms` | `1000` | frame wait timeout; the old code hard-coded 100 ms and mistook slow USB2.0 frames for a dead camera |
| `mv_usb_reset` | `true` | reset the device over libusb before reconnecting after a dropout |
| `mv_analog_gain` | `-1` | analogue (sensor-side) gain |
| `mv_clr_temp_mode` | `-1` | colour temperature mode: 0 auto, 1 preset, 2 custom |
| `mv_clr_temp_gain` | `""` | `"R,G,B"` gains for the preset mode (100 = 1.0x, range 0..400) |
| `mv_once_wb` | `false` | run the one-shot white balance (overwrites `mv_clr_temp_gain`) |
| `mv_sharpness` | `-1` | sharpness, 0 = off |
| `mv_contrast` | `-1` | contrast, 100 = original |
| `mv_saturation` | `-1` | saturation, 100 = original |
| `mv_anti_flick` | `-1` | anti-flicker; when on, exposure is locked to multiples of the mains period |
| `mv_light_frequency` | `-1` | 0 = 50 Hz, 1 = 60 Hz |
| `mv_wb_mode`, `mv_frame_rate` | `-1` | unsupported on this model (returns -4): keep them commented out |
| `mv_parameter_mode`, `mv_parameter_mask`, `mv_parameter_load_group`, `mv_parameter_save_group`, `mv_parameter_file`, `mv_parameter_save_file`, `mv_data_dir` | empty | SDK parameter-sheet source and "golden sheet" file handling (section 6) |

## 8. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `CameraInit` returns `-18` | Another process still holds the camera: `pgrep -a` and kill it |
| Collect statistics show valid 0 / dropped N after a forced kill | Do **not** `SIGTERM` a camera process: `tools::Exiter` only handles `SIGINT`, so `timeout` or `kill` skips `CameraUnInit` and leaves this USB2.0 camera half-streaming. Wait 1-2 minutes, replug, or move it to a USB3 port. Use Ctrl+C, `timeout -s INT`, or `-n=<frames>` so the program exits by itself |
| Readback shows auto exposure on, about 10 ms, gamma 100 | The exposure side is applied before the parameter source: see the ordering rule in section 6 |
| Startup warns about `vid_pid` | The camera reports a different VID:PID than the config (check `lsusb`) |
| `[YAML] Failed to load file: bad file` | `-c=<path>` was written with a space |
| `NO YOLO` on the HUD | Detector unavailable (no TensorRT, or OpenCV below 4.9 for the ONNX backend). Capture still runs; drop `-no-yolo` once a backend exists |
| Broken or half frames at 1280x1024 | usbfs memory limit too small; the startup warning prints the fix: `sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'` |

## 9. Measured baseline (Jetson Orin Nano, MV-SUA133GC on a USB2.0 port)

| Item | Value |
|---|---|
| Capture rate, 1280x1024 Bayer8, frame_speed 1 | 35.7 fps, 0 dropped frames |
| End to end (`-m=live -no-yolo`, no stream client) | 34.4 fps |
| `draw` stage, canvas only | 10.2 ms/frame |
| `draw` stage, canvas plus one stream client | 16.1 ms/frame |
| Other resolutions (camera preset) | 1024x768 about 59 fps, 640x480 about 134 fps |
| Frame rate ceiling | the USB2.0 link is already saturated (~45 MB/s), so frame_speed 0 / 1 / 2 all give about 35.7 fps at 1280x1024 |

Changing the resolution invalidates the camera calibration, so keep 1280x1024 unless you redo it.

## 10. Related documents

- [`AGENTS.md`](AGENTS.md) section 8.7: `detect_freq_visual_test` and the MindVision keys;
  section 8.8: headless MJPEG streaming and the pinned pipeline (engineering rules, Chinese).
- [`readme_zh.md`](readme_zh.md): the current full Chinese README of this project (a superset of
  this file - everything not ported here yet, including the inference backends and the local
  regression runner, lives there).
- `configs/mv_sua133gc.yaml`: the config file itself, with the measurement notes inline.

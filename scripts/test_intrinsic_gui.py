#!/usr/bin/env python3
"""Camera-free HTTP integration test. Uses only the Python standard library."""
import json
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import urllib.error
import urllib.request


ROOT = Path(__file__).resolve().parents[1]


def main():
    with tempfile.TemporaryDirectory(prefix="irm-intrinsic-") as directory:
        temp = Path(directory)
        images = temp / "images"
        subprocess.run([str(ROOT / "build/intrinsic_calibration_test"), str(images)], check=True)
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        base = f"http://127.0.0.1:{port}"

        def get(path):
            with urllib.request.urlopen(base + path, timeout=15) as response:
                return response.read()

        def post(path, payload=None, expected=200):
            req = urllib.request.Request(base + path,
                data=json.dumps(payload or {}).encode(),
                headers={"Content-Type": "application/json"}, method="POST")
            try:
                with urllib.request.urlopen(req, timeout=15) as response:
                    assert response.status == expected
                    return json.loads(response.read())
            except urllib.error.HTTPError as error:
                assert error.code == expected, (error.code, error.read())
                return json.loads(error.read())

        def status():
            return json.loads(get("/api/status"))

        with (temp / "server.log").open("w+") as log:
            process = subprocess.Popen([
                str(ROOT / "build/intrinsic_calibration_gui"),
                f"-c={ROOT / 'configs/calibration.yaml'}", f"-p={port}",
                f"-i={images}", f"-o={temp / 'sessions'}"], stdout=log, stderr=log, cwd=ROOT)
            try:
                deadline = time.monotonic() + 20
                while True:
                    if process.poll() is not None:
                        raise RuntimeError("Server exited during startup")
                    try:
                        first = status()
                        break
                    except (OSError, urllib.error.URLError):
                        if time.monotonic() > deadline:
                            raise
                        time.sleep(0.1)
                assert first["found"] and first["offline"]
                assert not first["handeye_available"] and not first["pose_provider_available"]
                assert "相机内参标定".encode() in get("/")
                assert b"/api/capture" in get("/app.js")
                assert get("/preview.jpg").startswith(b"\xff\xd8")
                # Form-style cross-origin mutations must not be accepted.
                form = urllib.request.Request(base + "/api/capture", data=b"x=1", method="POST")
                try:
                    urllib.request.urlopen(form, timeout=15)
                    raise AssertionError("Non-JSON mutation accepted")
                except urllib.error.HTTPError as error:
                    assert error.code == 400
                assert status()["samples"] == []
                post("/api/calibrate", expected=400)
                post("/api/pattern", {"cols": 0, "rows": 7, "spacing_mm": 40}, 400)
                post("/api/capture")
                assert get("/sample.jpg?id=1").startswith(b"\xff\xd8")
                post("/api/pattern", {"cols": 10, "rows": 7, "spacing_mm": 40}, 400)
                post("/api/delete", {"id": 999}, 400)
                post("/api/delete", {"id": 1})
                for _ in range(8):
                    post("/api/next")
                    assert status()["found"], "Perspective grid must be detected"
                    post("/api/capture")
                assert len(status()["samples"]) == 8
                post("/api/calibrate")
                deadline = time.monotonic() + 60
                while status()["busy"]:
                    assert time.monotonic() < deadline, "Calibration timeout"
                    time.sleep(0.1)
                solved = status()
                assert solved["result"] is not None, solved["error"]
                assert solved["result"]["rms_px"] < 1.0, solved["result"]
                exported = get("/intrinsics.yaml")
                assert b"camera_matrix:" in exported and b"distort_coeffs:" in exported
                session = Path(solved["directory"])
                assert (session / "intrinsics.yaml").read_bytes() == exported
                manifest = (session / "samples.yaml").read_text()
                assert "orientation_wxyz: ~" in manifest or "orientation_wxyz: null" in manifest
                assert len(list(session.glob("*.png"))) == 8
                assert get("/preview.jpg?undistort=1").startswith(b"\xff\xd8")
                post("/api/next")
                assert not status()["found"]
                post("/api/capture", expected=400)
                post("/api/delete", {"id": solved["samples"][0]["id"]})
                assert status()["result"] is None
                assert not (session / "intrinsics.yaml").exists()
                post("/api/pattern", {"cols": 10, "rows": 7, "spacing_mm": 40, "clear": True})
                assert status()["samples"] == []
                try:
                    get("/../../AGENTS.md")
                    raise AssertionError("Traversal must not be served")
                except urllib.error.HTTPError as error:
                    assert error.code == 404
                print("PASS: HTTP page, preview, capture/delete, pattern, async solve, YAML, "
                      "undistort, invalidation, blank rejection, traversal, no CBoard")
                print(f"Rendered-image RMS={solved['result']['rms_px']:.6f} px")
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        process.kill()  # Offline test process only; no hardware camera is opened.
                        process.wait()
                log.seek(0)
                text = log.read()
                if process.returncode != 0:
                    raise RuntimeError(text)
                assert "Closed normally" in text
                assert "Waiting for q" not in text


if __name__ == "__main__":
    main()
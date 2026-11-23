#!/usr/bin/env python
# Copyright 2026 Sophia Marie DeClue
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""LeRobot camera plugin for the Microsoft Kinect v1 (Xbox 360) sensor.

Why this exists: LeRobot ships depth support through vendor SDKs — RealSense, ZED, OAK — and its
third-party list has no consumer depth sensor at all. A Kinect v1 costs about $20 second-hand, uses
open drivers on Linux, needs no CUDA, and gives metric depth that works in the dark. That is a much
lower floor for a robotics hobbyist than any of the commercial options.

It talks to ``libfreenect`` through ctypes, so there is no Python binding to install and nothing to
compile at import time.

Depth is returned as LeRobot expects it: ``(H, W, 1)`` ``uint16``, **millimetres**, where ``0``
means no measurement. libfreenect's ``FREENECT_DEPTH_MM`` format gives that directly, so no scaling
or unit conversion happens here — which is deliberate, because a silent unit bug in a depth stream
is invisible in the images and poisons the dataset.

Linux setup (once)::

    sudo modprobe -r gspca_kinect      # the kernel driver claims the device; libfreenect needs it
    # libfreenect built from source into /usr/local, and udev rules that make the device world-usable

The kernel driver re-binds after a replug or a power cycle. If ``connect()`` fails with "device
busy", that is almost always why, and the message says so.
"""

from __future__ import annotations

import ctypes
import ctypes.util
import threading
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray

from lerobot.cameras.camera import Camera
from lerobot.cameras.configs import Cv2Rotation

from .configuration_kinect import KinectCameraConfig

# libfreenect depth/video formats.
DEPTH_MM = 5  # metric depth, uint16, in millimetres -- exactly LeRobot's contract
VIDEO_RGB = 0

SENSOR_W, SENSOR_H = 640, 480

_LIB_CANDIDATES = (
    "/usr/local/lib/libfreenect_sync.so",
    "/usr/lib/libfreenect_sync.so",
    "libfreenect_sync.so",
)


def _load_sync() -> ctypes.CDLL:
    """Load libfreenect_sync, with a message that names the actual problem."""
    errors: list[str] = []
    for path in _LIB_CANDIDATES:
        try:
            lib = ctypes.CDLL(path)
        except OSError as exc:
            errors.append(f"  {path}: {exc}")
            continue
        lib.freenect_sync_get_depth.restype = ctypes.c_int
        lib.freenect_sync_get_depth.argtypes = [
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.c_int,
            ctypes.c_int,
        ]
        lib.freenect_sync_get_video.restype = ctypes.c_int
        lib.freenect_sync_get_video.argtypes = [
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.c_int,
            ctypes.c_int,
        ]
        lib.freenect_sync_stop.restype = ctypes.c_int
        return lib
    raise RuntimeError(
        "libfreenect_sync could not be loaded. Build libfreenect with the synchronised API:\n"
        "  git clone https://github.com/OpenKinect/libfreenect && cd libfreenect && mkdir build && cd build\n"
        "  cmake .. -DBUILD_EXAMPLES=ON && make && sudo make install && sudo ldconfig\n"
        "Tried:\n" + "\n".join(errors)
    )


class KinectCamera(Camera):
    """A Kinect v1 as a LeRobot camera, with a depth channel.

    Frames are captured on a background thread so that recording is not blocked by the sensor, as
    the other LeRobot cameras do. ``read()`` and ``read_depth()`` consume from that buffer.
    """

    use_depth = True

    def __init__(self, config: KinectCameraConfig):
        super().__init__(config)
        self.config = config
        self.device_index = config.device_index
        self.rotation = config.rotation

        self._lib: ctypes.CDLL | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Condition()

        self._color: NDArray[Any] | None = None
        self._depth: NDArray[Any] | None = None
        self._color_ts: float = 0.0
        self._depth_ts: float = 0.0
        self._consumed = True
        self._connected = False
        self._frames = 0
        self._errors = 0

    # ---------------------------------------------------------------- connection

    @property
    def is_connected(self) -> bool:
        return self._connected

    @staticmethod
    def _present_devices() -> int:
        """How many Kinect v1 cameras the USB bus reports.

        Probing absent indices makes libfreenect print ``Invalid index`` on stderr for each one, so
        count them first and probe only what exists. Vendor 045e, product 02ae is the camera half of
        the sensor; the motor (02b0) and audio (02ad) are separate USB devices.
        """
        import glob
        import os

        count = 0
        for path in glob.glob("/sys/bus/usb/devices/*/idVendor"):
            try:
                with open(path) as fh:
                    if fh.read().strip().lower() != "045e":
                        continue
                with open(os.path.join(os.path.dirname(path), "idProduct")) as fh:
                    if fh.read().strip().lower() == "02ae":
                        count += 1
            except OSError:  # nosec B112 -- a vanished sysfs entry is not an error worth raising
                continue
        return count

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        """Report the Kinects libfreenect can read."""
        try:
            lib = _load_sync()
        except RuntimeError as exc:
            return [{"name": "Kinect v1", "type": "kinect", "id": None, "error": str(exc)}]

        n_present = KinectCamera._present_devices() or 1
        found: list[dict[str, Any]] = []
        for index in range(n_present):
            buf = ctypes.c_void_p()
            ts = ctypes.c_uint32()
            rc = lib.freenect_sync_get_depth(ctypes.byref(buf), ctypes.byref(ts), index, DEPTH_MM)
            if rc == 0 and buf.value:
                found.append(
                    {
                        "name": f"Kinect v1 @ {index}",
                        "type": "kinect",
                        "id": index,
                        "depth": "640x480 uint16 mm",
                        "color": "640x480 RGB",
                        "skeleton": "NiTE2 (optional, separate package)",
                    }
                )
        lib.freenect_sync_stop()
        return found

    def connect(self, warmup: bool = True) -> None:
        if self._connected:
            return
        self._lib = _load_sync()

        # Prove we can read before declaring success, and name the usual cause when we cannot.
        if not self._probe():
            raise RuntimeError(
                f"No frame from Kinect index {self.device_index}. Usual causes, in order:\n"
                "  1. gspca_kinect is still loaded and holding the device -- "
                "`sudo modprobe -r gspca_kinect`\n"
                "  2. the sensor is not powered (it needs its own power brick, not just USB)\n"
                "  3. a stale process still owns it -- `fuser -v /dev/bus/usb/*/*` finds the holder\n"
                "  4. udev permissions -- the udev rule should set MODE=\"0666\" on the device"
            )

        self._connected = True
        self._stop.clear()
        self._thread = threading.Thread(target=self._capture_loop, name="kinect-camera", daemon=True)
        self._thread.start()

        if warmup and self.config.warmup_s > 0:
            # A Kinect's first frames are dark or zero while exposure settles. Waiting here keeps
            # that out of the dataset rather than fixing it later.
            deadline = time.monotonic() + self.config.warmup_s
            while time.monotonic() < deadline:
                if self._depth is not None and self._depth.any():
                    break
                time.sleep(0.05)

    def _probe(self) -> bool:
        assert self._lib is not None
        for _ in range(30):
            buf = ctypes.c_void_p()
            ts = ctypes.c_uint32()
            rc = self._lib.freenect_sync_get_depth(
                ctypes.byref(buf), ctypes.byref(ts), self.device_index, DEPTH_MM
            )
            if rc == 0 and buf.value:
                return True
            time.sleep(0.1)
        return False

    def disconnect(self) -> None:
        self._connected = False
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._lib is not None:
            try:
                # Releases the USB handle. Without this the device stays claimed and the next
                # process to open it fails with a confusing "busy".
                self._lib.freenect_sync_stop()
            except Exception:  # nosec B110 -- cleanup must not raise during teardown
                pass
        with self._lock:
            self._color = None
            self._depth = None
            self._lock.notify_all()

    # ---------------------------------------------------------------- capture

    def _capture_loop(self) -> None:
        period = 1.0 / float(self.config.fps or 30)
        assert self._lib is not None
        while not self._stop.is_set():
            start = time.monotonic()

            dbuf, dts = ctypes.c_void_p(), ctypes.c_uint32()
            rc_d = self._lib.freenect_sync_get_depth(
                ctypes.byref(dbuf), ctypes.byref(dts), self.device_index, DEPTH_MM
            )
            if rc_d == 0 and dbuf.value:
                raw = ctypes.string_at(dbuf, SENSOR_W * SENSOR_H * 2)
                depth = np.frombuffer(raw, dtype=np.uint16).reshape(SENSOR_H, SENSOR_W).copy()
            else:
                depth = None

            vbuf, vts = ctypes.c_void_p(), ctypes.c_uint32()
            rc_c = self._lib.freenect_sync_get_video(
                ctypes.byref(vbuf), ctypes.byref(vts), self.device_index, VIDEO_RGB
            )
            if rc_c == 0 and vbuf.value:
                raw = ctypes.string_at(vbuf, SENSOR_W * SENSOR_H * 3)
                color = np.frombuffer(raw, dtype=np.uint8).reshape(SENSOR_H, SENSOR_W, 3).copy()
            else:
                color = None

            now = time.monotonic()
            if depth is None and color is None:
                self._errors += 1
            else:
                with self._lock:
                    if color is not None:
                        self._color, self._color_ts = self._rotate(color), now
                    if depth is not None:
                        self._depth, self._depth_ts = self._rotate(depth), now
                    self._frames += 1
                    self._consumed = False
                    self._lock.notify_all()

            elapsed = time.monotonic() - start
            if elapsed < period:
                self._stop.wait(period - elapsed)

    def _rotate(self, frame: NDArray[Any]) -> NDArray[Any]:
        if self.rotation == Cv2Rotation.NO_ROTATION:
            return frame
        try:
            import cv2
        except ImportError:
            return frame
        code = {
            Cv2Rotation.ROTATE_90: cv2.ROTATE_90_CLOCKWISE,
            Cv2Rotation.ROTATE_180: cv2.ROTATE_180,
            Cv2Rotation.ROTATE_270: cv2.ROTATE_90_COUNTERCLOCKWISE,
        }.get(self.rotation)
        return cv2.rotate(frame, code) if code is not None else frame

    # ---------------------------------------------------------------- reads

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("KinectCamera is not connected. Call connect() first.")

    def read(self) -> NDArray[Any]:
        self._require_connected()
        with self._lock:
            if self._color is None:
                if not self._lock.wait(timeout=2.0):
                    raise TimeoutError("Kinect delivered no colour frame within 2 s.")
            return self._color.copy()

    def read_depth(self) -> NDArray[Any]:
        """Depth as ``(H, W, 1)`` uint16 in millimetres, 0 meaning no measurement."""
        self._require_connected()
        with self._lock:
            if self._depth is None:
                if not self._lock.wait(timeout=2.0):
                    raise TimeoutError("Kinect delivered no depth frame within 2 s.")
            return self._depth.reshape(SENSOR_H, SENSOR_W, 1).copy()

    def async_read(self, timeout_ms: float = 200) -> NDArray[Any]:
        self._require_connected()
        deadline = time.monotonic() + timeout_ms / 1000.0
        with self._lock:
            while self._consumed:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"No new Kinect frame within {timeout_ms:.0f} ms "
                        f"(connected={self._connected}, frames={self._frames}, errors={self._errors})"
                    )
                self._lock.wait(timeout=remaining)
            self._consumed = True
            return self._color.copy()

    def read_latest(self, max_age_ms: int = 500) -> NDArray[Any]:
        self._require_connected()
        with self._lock:
            if self._color is None:
                raise RuntimeError("Kinect is connected but has captured no frames yet.")
            age_ms = (time.monotonic() - self._color_ts) * 1000.0
            if age_ms > max_age_ms:
                raise TimeoutError(f"Latest Kinect colour frame is {age_ms:.0f} ms old.")
            return self._color.copy()

    def read_latest_depth(self, max_age_ms: int = 500) -> NDArray[Any]:
        self._require_connected()
        with self._lock:
            if self._depth is None:
                raise RuntimeError("Kinect is connected but has captured no depth frames yet.")
            age_ms = (time.monotonic() - self._depth_ts) * 1000.0
            if age_ms > max_age_ms:
                raise TimeoutError(f"Latest Kinect depth frame is {age_ms:.0f} ms old.")
            return self._depth.reshape(SENSOR_H, SENSOR_W, 1).copy()

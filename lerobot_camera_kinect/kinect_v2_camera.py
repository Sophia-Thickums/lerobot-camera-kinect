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
"""A Kinect v2 as a LeRobot camera, with a metric depth channel.

It talks to **OpenNI2** through ctypes — OpenNI2 ships with the Kinect drivers on most
distributions and, through ``libfreenect2-openni2.so``, talks to the v2 — so no Python binding has
to be built and nothing is compiled. That also means the same code path can open a Kinect v1
(``freenect://0``) where the OpenNI2 Freenect driver is installed.

Two facts about this sensor that save an afternoon:

* **Depth is millimetres, as uint16, with 0 meaning "no measurement".** No scaling is needed, and
  0 is the only invalid marker the sensor gives you.
* **The sensor hard-clips at 4,499 mm.** Nothing past 4.5 m is ever returned, however open the room
  is. In a typical room a third of the pixels come back 0 for that reason alone, not because of a
  fault.

Linux notes:
    * the udev rule for the v2 (``90-kinect2.rules``) is required, or the device is root-only;
    * libfreenect2's OpenGL depth processor needs a display session (X11 or Wayland). In a bare
      shell it fails with ``GLFW error 65550`` and no frames arrive. Run it from a desktop session,
      or set ``XDG_RUNTIME_DIR`` / ``WAYLAND_DISPLAY`` before starting Python.

Only one process may hold a v2 at a time; a second consumer sees no frames rather than an error.
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

from .configuration_kinect_v2 import KinectV2CameraConfig

# --- OpenNI2 C API, as declared in /usr/include/openni2/OniCAPI.h --------------------------------
ONI_API_VERSION = 2002  # ONI_CREATE_API_VERSION(2, 2)
SENSOR_IR, SENSOR_COLOR, SENSOR_DEPTH = 1, 2, 3
PROP_VIDEO_MODE = 3
PF_DEPTH_1_MM = 100
PF_RGB888 = 200
STATUS_OK = 0

_CANDIDATE_LIBS = (
    "libOpenNI2.so",
    "/usr/lib/libOpenNI2.so",
    "/usr/local/lib/libOpenNI2.so",
    "libOpenNI2.so.0",
)


class _OniVideoMode(ctypes.Structure):
    _fields_ = [
        ("pixelFormat", ctypes.c_int),
        ("resolutionX", ctypes.c_int),
        ("resolutionY", ctypes.c_int),
        ("fps", ctypes.c_int),
    ]


class _OniFrame(ctypes.Structure):
    # Field order is the ABI: do not reorder, and do not "tidy" the padding.
    _fields_ = [
        ("dataSize", ctypes.c_int),
        ("data", ctypes.c_void_p),
        ("sensorType", ctypes.c_int),
        ("timestamp", ctypes.c_uint64),
        ("frameIndex", ctypes.c_int),
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("videoMode", _OniVideoMode),
        ("croppingEnabled", ctypes.c_int),
        ("cropOriginX", ctypes.c_int),
        ("cropOriginY", ctypes.c_int),
        ("stride", ctypes.c_int),
    ]


class _OniDeviceInfo(ctypes.Structure):
    _fields_ = [
        ("uri", ctypes.c_char * 256),
        ("vendor", ctypes.c_char * 256),
        ("name", ctypes.c_char * 256),
        ("usbVendorId", ctypes.c_uint16),
        ("usbProductId", ctypes.c_uint16),
    ]


class _OniSensorInfo(ctypes.Structure):
    _fields_ = [
        ("sensorType", ctypes.c_int),
        ("numSupportedVideoModes", ctypes.c_int),
        ("pSupportedVideoModes", ctypes.POINTER(_OniVideoMode)),
    ]


_INIT_LOCK = threading.Lock()
_INITIALISED = False


def _load_openni() -> ctypes.CDLL:
    """Load libOpenNI2 and declare the entry points, with a message that names the real problem."""
    last: Exception | None = None
    for name in _CANDIDATE_LIBS:
        try:
            lib = ctypes.CDLL(name)
        except OSError as exc:  # pragma: no cover - depends on the machine
            last = exc
            continue
        try:
            lib.oniInitialize.argtypes, lib.oniInitialize.restype = [ctypes.c_int], ctypes.c_int
            lib.oniGetDeviceList.argtypes = [
                ctypes.POINTER(ctypes.POINTER(_OniDeviceInfo)),
                ctypes.POINTER(ctypes.c_int),
            ]
            lib.oniReleaseDeviceList.argtypes = [ctypes.POINTER(_OniDeviceInfo)]
            lib.oniDeviceOpen.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_void_p)]
            lib.oniDeviceClose.argtypes = [ctypes.c_void_p]
            # ★ returns const OniSensorInfo* — not a status, and not an out-parameter
            lib.oniDeviceGetSensorInfo.argtypes = [ctypes.c_void_p, ctypes.c_int]
            lib.oniDeviceGetSensorInfo.restype = ctypes.POINTER(_OniSensorInfo)
            lib.oniDeviceCreateStream.argtypes = [
                ctypes.c_void_p,
                ctypes.c_int,
                ctypes.POINTER(ctypes.c_void_p),
            ]
            lib.oniStreamSetProperty.argtypes = [
                ctypes.c_void_p,
                ctypes.c_int,
                ctypes.c_void_p,
                ctypes.c_int,
            ]
            lib.oniStreamStart.argtypes = [ctypes.c_void_p]
            lib.oniStreamStop.argtypes = [ctypes.c_void_p]
            lib.oniStreamDestroy.argtypes = [ctypes.c_void_p]
            lib.oniWaitForAnyStream.argtypes = [
                ctypes.POINTER(ctypes.c_void_p),
                ctypes.c_int,
                ctypes.POINTER(ctypes.c_int),
                ctypes.c_int,
            ]
            lib.oniStreamReadFrame.argtypes = [
                ctypes.c_void_p,
                ctypes.POINTER(ctypes.POINTER(_OniFrame)),
            ]
            lib.oniFrameRelease.argtypes = [ctypes.POINTER(_OniFrame)]
        except AttributeError as exc:  # pragma: no cover
            last = exc
            continue
        return lib
    raise RuntimeError(
        "libOpenNI2 could not be loaded. Install OpenNI2 (Debian/Ubuntu: libopenni2-0; Arch: "
        "openni2) and, for the Kinect v2, make sure libfreenect2-openni2.so is in "
        "/usr/lib/OpenNI2/Drivers/. Last error: " + repr(last)
    )


def _initialise(lib: ctypes.CDLL) -> None:
    global _INITIALISED
    with _INIT_LOCK:
        if not _INITIALISED:
            status = lib.oniInitialize(ONI_API_VERSION)
            if status != STATUS_OK:
                raise RuntimeError(f"oniInitialize failed with status {status}")
            _INITIALISED = True


class KinectV2Camera(Camera):
    """A Kinect v2 as a LeRobot camera, with a depth channel.

    Frames are captured on a background thread so that recording is not blocked by the sensor, as
    the other LeRobot cameras do. ``read()`` and ``read_depth()`` consume from that buffer.
    """

    use_depth = True

    def __init__(self, config: KinectV2CameraConfig):
        super().__init__(config)
        self.config = config
        self.device_index = config.device_index
        self.rotation = config.rotation

        self._lib: ctypes.CDLL | None = None
        self._device: ctypes.c_void_p | None = None
        self._depth_stream: ctypes.c_void_p | None = None
        self._colour_stream: ctypes.c_void_p | None = None

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

    # ---------------------------------------------------------------- discovery

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        """Report the Kinect v2 sensors OpenNI2 can see."""
        try:
            lib = _load_openni()
            _initialise(lib)
        except RuntimeError as exc:
            return [{"name": "Kinect v2", "type": "kinect_v2", "id": None, "error": str(exc)}]

        found: list[dict[str, Any]] = []
        for index, dev in enumerate(KinectV2Camera._devices(lib)):
            found.append(
                {
                    "name": f"Kinect v2 @ {index} ({dev['name']})",
                    "type": "kinect_v2",
                    "id": index,
                    "uri": dev["uri"],
                }
            )
        return found

    @staticmethod
    def _devices(lib: ctypes.CDLL, want_depth: tuple[int, int] = (512, 424)) -> list[dict[str, str]]:
        """One entry per physical sensor.

        libfreenect2's OpenNI2 driver publishes the *same* Kinect three times: a bare URI, one with
        ``depth-size=640x480`` and one with ``depth-size=512x424``. Listing all three would make
        ``device_index=1`` open the camera that index 0 already holds, which fails as "no frames"
        rather than as an error. So group by serial and keep the URI whose depth size matches the
        configuration, falling back to the bare one.
        """
        devs = ctypes.POINTER(_OniDeviceInfo)()
        n = ctypes.c_int(0)
        lib.oniGetDeviceList(ctypes.byref(devs), ctypes.byref(n))
        by_serial: dict[str, list[dict[str, str]]] = {}
        try:
            for i in range(n.value):
                info = devs[i]
                uri = info.uri.decode()
                if "freenect2://" not in uri:
                    continue
                serial = uri.split("serial=")[1].split("&")[0] if "serial=" in uri else uri
                by_serial.setdefault(serial, []).append({"uri": uri, "name": info.name.decode()})
        finally:
            if n.value:
                lib.oniReleaseDeviceList(devs)

        wanted = f"depth-size={want_depth[0]}x{want_depth[1]}"
        out: list[dict[str, str]] = []
        for serial, candidates in by_serial.items():
            chosen = next((c for c in candidates if wanted in c["uri"]), candidates[0])
            out.append(chosen)
        return out

    # ---------------------------------------------------------------- connection

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, warmup: bool = True) -> None:
        if self._connected:
            return

        self._lib = _load_openni()
        _initialise(self._lib)
        devices = self._devices(
            self._lib, (self.config.depth_width, self.config.depth_height)
        )
        if not devices:
            raise RuntimeError(
                "No Kinect v2 found through OpenNI2. Check the udev rule (90-kinect2.rules), that "
                "libfreenect2-openni2.so is in /usr/lib/OpenNI2/Drivers/, and that no other process "
                "holds the sensor."
            )
        if self.device_index >= len(devices):
            raise RuntimeError(
                f"Kinect v2 index {self.device_index} requested, but {len(devices)} found."
            )

        uri = devices[self.device_index]["uri"].encode()
        device = ctypes.c_void_p()
        status = self._lib.oniDeviceOpen(uri, ctypes.byref(device))
        if status != STATUS_OK:
            raise RuntimeError(f"oniDeviceOpen({uri!r}) failed with status {status}")
        self._device = device

        self._depth_stream = self._open_stream(
            SENSOR_DEPTH,
            PF_DEPTH_1_MM,
            self.config.depth_width,
            self.config.depth_height,
        )
        self._colour_stream = self._open_stream(
            SENSOR_COLOR,
            PF_RGB888,
            self.config.colour_width,
            self.config.colour_height,
        )

        self._stop.clear()
        self._thread = threading.Thread(target=self._capture_loop, name="kinect-v2", daemon=True)
        self._thread.start()
        self._connected = True

        if warmup and self.config.warmup_s:
            deadline = time.monotonic() + self.config.warmup_s
            while time.monotonic() < deadline:
                with self._lock:
                    if self._color is not None and self._depth is not None:
                        break
                time.sleep(0.05)

    def _open_stream(
        self, sensor: int, pixel_format: int, width: int, height: int
    ) -> ctypes.c_void_p:
        assert self._lib is not None and self._device is not None
        stream = ctypes.c_void_p()
        status = self._lib.oniDeviceCreateStream(self._device, sensor, ctypes.byref(stream))
        if status != STATUS_OK:
            raise RuntimeError(f"oniDeviceCreateStream(sensor={sensor}) failed with status {status}")

        mode = _OniVideoMode(pixel_format, width, height, self.config.fps)
        status = self._lib.oniStreamSetProperty(
            stream, PROP_VIDEO_MODE, ctypes.byref(mode), ctypes.sizeof(_OniVideoMode)
        )
        if status != STATUS_OK:
            raise RuntimeError(
                f"Kinect v2 refused {width}x{height}@{self.config.fps} for sensor {sensor} "
                f"(status {status}). The sensor offers 512x424 / 640x480 depth and 1920x1080 / "
                "512x424 colour."
            )
        status = self._lib.oniStreamStart(stream)
        if status != STATUS_OK:
            raise RuntimeError(f"oniStreamStart failed with status {status}")
        return stream

    def disconnect(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._connected = False

        if self._lib is None:
            return
        for stream in (self._depth_stream, self._colour_stream):
            if stream:
                self._lib.oniStreamStop(stream)
                self._lib.oniStreamDestroy(stream)
        self._depth_stream = None
        self._colour_stream = None
        if self._device:
            self._lib.oniDeviceClose(self._device)
            self._device = None

    # ---------------------------------------------------------------- capture

    def _capture_loop(self) -> None:
        assert self._lib is not None
        lib = self._lib
        streams = [s for s in (self._depth_stream, self._colour_stream) if s]
        array = (ctypes.c_void_p * len(streams))(*streams)

        while not self._stop.is_set():
            index = ctypes.c_int(-1)
            status = lib.oniWaitForAnyStream(array, len(streams), ctypes.byref(index), 200)
            if status != STATUS_OK:
                continue
            for stream in streams:
                frame = ctypes.POINTER(_OniFrame)()
                if lib.oniStreamReadFrame(stream, ctypes.byref(frame)) != STATUS_OK:
                    continue
                try:
                    self._store(frame.contents)
                finally:
                    lib.oniFrameRelease(frame)

    def _store(self, frame: _OniFrame) -> None:
        # ctypes' pointer arithmetic keeps the stride honest: sensors pad rows.
        raw = ctypes.string_at(frame.data, frame.dataSize)
        width, height, stride = frame.width, frame.height, frame.stride

        if frame.sensorType == SENSOR_DEPTH:
            row = width * 2
            arr = np.frombuffer(raw, dtype=np.uint16)
            if stride != row:
                arr = np.lib.stride_tricks.as_strided(
                    arr, shape=(height, width), strides=(stride, 2)
                ).copy()
            else:
                arr = arr[: height * width].reshape(height, width).copy()
            with self._lock:
                self._depth = arr
                self._depth_ts = time.monotonic()
                self._consumed = False
                self._frames += 1
                self._lock.notify_all()
            return

        if frame.sensorType == SENSOR_COLOR:
            row = width * 3
            if stride != row:
                arr = np.lib.stride_tricks.as_strided(
                    np.frombuffer(raw, dtype=np.uint8),
                    shape=(height, width, 3),
                    strides=(stride, 3, 1),
                ).copy()
            else:
                arr = np.frombuffer(raw, dtype=np.uint8)[: height * width * 3].reshape(
                    height, width, 3
                ).copy()
            with self._lock:
                self._color = arr
                self._color_ts = time.monotonic()
                self._consumed = False
                self._frames += 1
                self._lock.notify_all()

    # ---------------------------------------------------------------- reading

    def _rotate(self, frame: NDArray[Any]) -> NDArray[Any]:
        if self.rotation == Cv2Rotation.NO_ROTATION:
            return frame
        try:
            import cv2
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("rotation needs opencv-python-headless") from exc
        code = {
            Cv2Rotation.ROTATE_90: cv2.ROTATE_90_CLOCKWISE,
            Cv2Rotation.ROTATE_180: cv2.ROTATE_180,
            Cv2Rotation.ROTATE_270: cv2.ROTATE_90_COUNTERCLOCKWISE,
        }.get(self.rotation)
        return cv2.rotate(frame, code) if code is not None else frame

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError(f"{self.__class__.__name__} is not connected.")

    def read(self) -> NDArray[Any]:
        frame = self.async_read(timeout_ms=2000)
        return self._rotate(frame)

    def read_depth(self) -> NDArray[Any]:
        self._require_connected()
        with self._lock:
            if self._depth is None:
                raise RuntimeError("No depth frame has arrived from the Kinect v2 yet.")
            depth = self._depth.copy()
            self._consumed = True
        return self._rotate(depth.reshape(depth.shape[0], depth.shape[1], 1))

    def async_read(self, timeout_ms: float = 200) -> NDArray[Any]:
        self._require_connected()
        deadline = time.monotonic() + timeout_ms / 1000.0
        with self._lock:
            while self._color is None or self._consumed:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    if self._color is None:
                        raise TimeoutError("No colour frame has arrived from the Kinect v2.")
                    break
                self._lock.wait(remaining)
            assert self._color is not None
            return self._color.copy()

    def read_latest(self, max_age_ms: int = 500) -> NDArray[Any]:
        self._require_connected()
        with self._lock:
            if self._color is None:
                raise RuntimeError("No colour frame has arrived from the Kinect v2 yet.")
            age = (time.monotonic() - self._color_ts) * 1000.0
            if age > max_age_ms:
                raise TimeoutError(
                    f"The newest Kinect v2 frame is older than {max_age_ms} ms ({age:.0f} ms)."
                )
            return self._color.copy()

    def read_latest_depth(self, max_age_ms: int = 500) -> NDArray[Any]:
        self._require_connected()
        with self._lock:
            if self._depth is None:
                raise RuntimeError("No depth frame has arrived from the Kinect v2 yet.")
            age = (time.monotonic() - self._depth_ts) * 1000.0
            if age > max_age_ms:
                raise TimeoutError(
                    f"The newest Kinect v2 depth frame is older than {max_age_ms} ms ({age:.0f} ms)."
                )
            return self._depth.copy().reshape(self._depth.shape[0], self._depth.shape[1], 1)

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
"""Configuration for the Kinect v1 camera."""

from dataclasses import dataclass

from lerobot.cameras.configs import CameraConfig, Cv2Rotation


@CameraConfig.register_subclass("kinect")
@dataclass
class KinectCameraConfig(CameraConfig):
    """Configuration for a Microsoft Kinect v1 (Xbox 360) sensor.

    The Kinect is a depth sensor first and a camera second: it gives metric depth that works in a
    dark room, plus RGB and infrared. Depth is the reason to use it — a policy that learns from
    distance generalises across lighting, and a 640x480 depth map is cheaper to stream than a
    higher-resolution colour image.

    Attributes:
        device_index: Which Kinect to open when several are attached (0 is the first).
        fps: Target capture rate. The hardware delivers 30 fps; asking for more does nothing.
        width: Frame width. The Kinect v1 delivers 640; other values are rejected by the hardware.
        height: Frame height. 480, likewise fixed by the hardware.
        rotation: Rotation applied to frames on read, as with the OpenCV camera.
        warmup_s: Seconds to wait after opening, because the first frames from a Kinect are often
            dark or zero while auto-exposure settles.
    """

    device_index: int = 0
    fps: int = 30
    width: int = 640
    height: int = 480
    rotation: Cv2Rotation = Cv2Rotation.NO_ROTATION
    warmup_s: float = 2.0

    def __post_init__(self) -> None:
        # The hardware is fixed at 640x480 per stream. Saying so here is friendlier than a shape
        # mismatch three layers down in a dataset writer.
        if self.width != 640 or self.height != 480:
            raise ValueError(
                f"Kinect v1 delivers 640x480 per stream; got {self.width}x{self.height}. "
                "Leave width and height unset, or use 640 and 480."
            )
        if self.fps is not None and self.fps > 30:
            raise ValueError(f"Kinect v1 delivers at most 30 fps, got {self.fps}.")

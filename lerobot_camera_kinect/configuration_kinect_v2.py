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
"""Configuration for the Kinect v2 camera."""

from dataclasses import dataclass

from lerobot.cameras.configs import CameraConfig, Cv2Rotation

# The modes the sensor itself offers, in (pixel format, width, height, fps). Anything else is
# refused by the device, so the config refuses it first and says which ones work.
DEPTH_MODES = ((512, 424), (640, 480))
COLOUR_MODES = ((1920, 1080), (512, 424))


@CameraConfig.register_subclass("kinect_v2")
@dataclass
class KinectV2CameraConfig(CameraConfig):
    """Configuration for a Microsoft Kinect v2 (Xbox One) sensor.

    The v2 is a time-of-flight sensor: depth is metric, works in the dark, and is delivered in
    millimetres as uint16 with 0 meaning "no measurement". Its usable range is 0.5 m to 4.5 m —
    the sensor hard-clips at 4499 mm, which is worth knowing before blaming a driver for missing
    distant geometry.

    Attributes:
        device_index: Which Kinect v2 to open when several are attached (0 is the first).
        fps: Target capture rate. The hardware delivers 30 fps.
        depth_width, depth_height: 512x424 (native) or 640x480.
        colour_width, colour_height: 1920x1080 (native) or 512x424.
        rotation: Rotation applied to frames on read, as with the OpenCV camera.
        warmup_s: Seconds to wait after opening, because the first frames from a Kinect are often
            zeroed or black while the sensor settles.
    """

    device_index: int = 0
    fps: int = 30
    depth_width: int = 512
    depth_height: int = 424
    colour_width: int = 1920
    colour_height: int = 1080
    rotation: Cv2Rotation = Cv2Rotation.NO_ROTATION
    warmup_s: float = 2.0

    def __post_init__(self) -> None:
        if (self.depth_width, self.depth_height) not in DEPTH_MODES:
            raise ValueError(
                f"Kinect v2 depth is {DEPTH_MODES}; got {self.depth_width}x{self.depth_height}."
            )
        if (self.colour_width, self.colour_height) not in COLOUR_MODES:
            raise ValueError(
                f"Kinect v2 colour is {COLOUR_MODES}; got {self.colour_width}x{self.colour_height}."
            )
        if self.fps is not None and self.fps > 30:
            raise ValueError(f"Kinect v2 delivers at most 30 fps, got {self.fps}.")
        # LeRobot's base config carries width/height for the colour stream; keep them honest so a
        # dataset writer that reads them gets the v2's real colour size.
        if self.width is None:
            self.width = self.colour_width
        if self.height is None:
            self.height = self.colour_height

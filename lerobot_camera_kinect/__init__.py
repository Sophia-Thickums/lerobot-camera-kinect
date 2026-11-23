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
"""LeRobot camera plugin for the Microsoft Kinect v1 (Xbox 360 sensor).

Importing this package registers the ``kinect`` camera type with LeRobot's config registry, which
is how a third-party camera plugin is discovered: LeRobot imports any installed package whose name
starts with ``lerobot_camera_``, and registration happens at import time.
"""

from .configuration_kinect import KinectCameraConfig
from .kinect_camera import KinectCamera

__all__ = ["KinectCamera", "KinectCameraConfig"]

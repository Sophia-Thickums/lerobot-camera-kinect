# lerobot-camera-kinect

A **LeRobot camera plugin for the Microsoft Kinect v1** (the Xbox 360 sensor) — metric depth, RGB and infrared, on open drivers, with **no CUDA and no vendor SDK**.

A second-hand Kinect costs about **$15–25**. It gives 640×480 metric depth at a true 30 fps, it reads in a completely dark room, and it has a motor that tilts the sensor — a second degree of freedom most fixed cameras don't have. LeRobot's depth support currently runs through RealSense, ZED and OAK, all of which cost multiples of that and all of which need a vendor stack. This plugin puts a consumer depth sensor in the same list.

## What you get

| | |
|---|---|
| **depth** | 640×480 `uint16`, **millimetres**, `0` = no measurement — LeRobot's exact contract |
| **colour** | 640×480 RGB `uint8` |
| **rate** | 30 fps, captured on a background thread |
| **range** | ~0.5 m to 8 m, usable indoors |
| **dark** | depth is an active IR sensor — it works with the lights off |
| **tilt** | the sensor's motor tilts ±30° (motor API not exposed here; see [Tilt](#tilt)) |
| **drivers** | `libfreenect`, no Python bindings, no CUDA — AMD, Intel and NVIDIA all fine |

## Install

**1. libfreenect with the synchronised API.**

```sh
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect && mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=ON
make -j"$(nproc)" && sudo make install && sudo ldconfig
```

**2. Free the device from the kernel's own driver.**

The kernel module `gspca_kinect` claims the sensor, and libfreenect cannot open it while that is true:

```sh
sudo modprobe -r gspca_kinect
```

> **This is the single most common failure.** The module **re-binds after every replug and every power cycle**, so it comes back on its own. If `connect()` raises, this is almost always why, and the error message says so. To make it permanent, blacklist it in `/etc/modprobe.d/`.

**3. udev rules, so you don't need root.**

libfreenect ships `platform/linux/udev/51-kinect.rules`; install it to `/etc/udev/rules.d/` and set `MODE="0666"` on the sensor's nodes.

**4. This plugin.**

```sh
pip install -e .          # from a checkout
# or, once published:
pip install lerobot-camera-kinect
```

LeRobot imports any installed package prefixed with `lerobot_camera_`, so nothing else is needed — the `kinect` camera type appears automatically.

## Use

```python
from lerobot_camera_kinect import KinectCamera, KinectCameraConfig

with KinectCamera(KinectCameraConfig(device_index=0)) as cam:
    rgb = cam.read()             # (480, 640, 3) uint8
    depth = cam.read_depth()     # (480, 640, 1) uint16, millimetres

    frame = cam.async_read(timeout_ms=200)      # freshest unconsumed frame
    recent = cam.read_latest(max_age_ms=500)    # peek, non-blocking
    d = cam.read_latest_depth(max_age_ms=500)
```

Or straight from the CLI:

```sh
lerobot-record \
  --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=my_follower \
  --robot.cameras="{ front: {type: kinect, width: 640, height: 480, fps: 30} }" \
  --dataset.repo_id=${HF_USER}/my-dataset \
  --dataset.num_episodes=5
```

Discover attached sensors:

```python
from lerobot_camera_kinect import KinectCamera
print(KinectCamera.find_cameras())
```

## Measured

On a Kinect v1 on a desktop, Linux 7.3.5, libfreenect 0.7.5:

```
colour : (480, 640, 3) uint8, mean 106.5
depth  : (480, 640, 1) uint16, 191,401 valid pixels, median 3,659 mm
burst  : 10 async_read calls -> 10 distinct frames (it streams, it does not repeat a cache)
```

Depth is `FREENECT_DEPTH_MM` from libfreenect, which is millimetres already. The plugin does **no unit conversion** — a silent scale error in a depth stream is invisible in the images and poisons the dataset, so the safest thing to do with a correct unit is nothing.

## Tilt

The Kinect v1's motor tilts ±30° and is genuinely useful for framing a work surface. It is **not** exposed by this plugin, because the synchronised capture API holds the USB device and the motor interface needs it released — claiming both from one process takes a `freenect_sync_stop()`, a re-init with the motor subdevice, and a re-open, which would make a camera read block on a motor call. If you want it, the pattern is in `kinect_sense.py` in [Sophia-Thickums/radiance-gfx9](https://github.com/Sophia-Thickums/radiance-gfx9) — a separate context that claims only the motor.

## Notes and honest limits

- **640×480 is the hardware.** `width`/`height` other than 640/480 raise at config time rather than failing three layers down in a dataset writer.
- **`[Stream 70] Invalid magic` on connect is libfreenect resyncing**, not a fault. It comes from the audio subdevice and it settles; frames are unaffected.
- **No skeleton tracking in this package.** NiTE2 does that, is abandoned by its vendor, and carries a licence that would make this plugin non-redistributable. Depth and colour are what a LeRobot policy actually learns from.
- **One process at a time.** The synchronised API holds the device open between calls; two processes reading the same sensor will fight.
- Tested on a **Kinect v1 only**. The Kinect v2 is a different device with a different driver (`libfreenect2`), and would be a separate plugin.

## Licence

Apache-2.0, matching LeRobot. `libfreenect` is Apache-2.0/GPL-2.0 dual; this plugin links against it at runtime through `ctypes` and redistributes none of it.

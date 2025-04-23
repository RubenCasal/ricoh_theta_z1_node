# Ricoh THETA Z1 ROS 2 Node

This package provides a ROS 2 node (`theta_driver::ThetaDriver`) to stream video from the **Ricoh THETA Z1** in dual-fisheye UVC mode, decode it via GStreamer, and publish the image to ROS 2 topics.

## Features

- Support for 1920x960 (FHD) and 3840x1920 (4K) streaming
- Real-time decoding and publishing via GStreamer pipeline
- Publishes stitched RGB image to ROS 2
- Fully integrated with `rclcpp` and `sensor_msgs`

---

## Installation

### 1. System Dependencies

```bash
sudo apt install \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev \
  libglib2.0-dev \
  libusb-1.0-0-dev \
  libopencv-dev \
  gstreamer1.0-vaapi
```

### 2. libuvc (if not installed via apt)

```bash
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake ..
make && sudo make install
```

### 3. Build the ROS 2 Package

```bash
cd ~/ros2_ws/src
git clone https://github.com/RubenCasal/ricoh_theta_z1_node.git
cd ..
colcon build --packages-select ricoh_theta
source install/setup.bash
```

---

## Usage

```bash
ros2 run ricoh_theta theta_node
```

### Available Parameters

| Parameter        | Type    | Description                                  | Default          |
|------------------|---------|----------------------------------------------|------------------|
| `use4k`          | `bool`  | Enable 4K resolution                          | `false`          |
| `serial`         | `str`   | Device serial number to connect              | `""`             |
| `camera_frame`   | `str`   | Frame ID for the image messages              | `"camera_link"`  |
| `pipeline`       | `str`   | Custom GStreamer pipeline string             | *preset default* |

Example:

```bash
ros2 run ricoh_theta theta_node --ros-args -p use4k:=true -p camera_frame:=theta_frame
```

---

## Published Topics

| Topic             | Type                        | Description                     |
|-------------------|-----------------------------|---------------------------------|
| `/stitched_image` | `sensor_msgs/msg/Image`     | Stitched RGB image from camera |

---

## How to Enable Live Streaming Mode on the THETA Z1

Before launching the node, you need to manually switch the camera to UVC (live streaming) mode:

1. **Connect the THETA Z1 via USB** to your computer.
2. **Power on** the camera using the power button.
3. **Press the MODE button** repeatedly until you see a small `LIVE` icon on the screen.

- The icon indicates that the camera is ready to stream in UVC mode
  
<p align="center">
<img src="./readme_images/live_mode.jpg" alt="Live Mode" width="300">
</p>

- When streaming is active, a **larger `LIVE` icon** will appear (see below).
  
<p align="center">
<img src="./readme_images/activated_live_mode.jpg" alt="Activated Live Mode" width="300">
</p>

---

## ⚙️ Optional: Enable GPU-accelerated decoding with `nvh264dec`

To reduce latency and improve FPS, you can enable GPU decoding with NVIDIA's NVDEC:

### 1. Install VAAPI support:

```bash
sudo apt install gstreamer1.0-vaapi
```

### 2. Modify pipeline in `ricoh_theta_node.cpp` (around line 123):

```cpp
pipeline_ = 
    "appsrc name=ap is-live=true do-timestamp=true format=time ! queue ! "
    "h264parse ! nvh264dec ! videoconvert n_threads=8 ! video/x-raw,format=RGB ! "
    "appsink name=appsink emit-signals=true sync=false max-buffers=1 drop=true";
```

### ✅ Fallback for CPU decoding:

```cpp
pipeline_ =
 "appsrc name=ap ! queue ! h264parse ! queue ! "
 "decodebin ! queue ! videoconvert n_threads=8 ! queue ! video/x-raw,format=RGB ! appsink name=appsink emit-signals=true";
```

This change leverages GPU for H.264 decoding, significantly reducing latency on compatible systems.


# Ricoh THETA Z1 ROS 2 Driver

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
  libopencv-dev
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
git clone https://github.com/youruser/ricoh_theta_ros2.git
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

## Extending the Node

### 1. Publish Left and Right Fisheye Images

Split the RGB stitched frame into left and right images (each 1920x960 or 3840x1920 depending on resolution).

```cpp
auto left_img = create_image_msg("left_image", map.data, width / 2, height);
auto right_img = create_image_msg("right_image", map.data + (width / 2) * 3, width / 2, height);
image_left_pub_->publish(*left_img);
image_right_pub_->publish(*right_img);
```

Don't forget to initialize the publishers in `onInit()`:

```cpp
image_left_pub_ = this->create_publisher<sensor_msgs::msg::Image>("left_image", 1);
image_right_pub_ = this->create_publisher<sensor_msgs::msg::Image>("right_image", 1);
```

### 2. Enable image_proc compatibility

Make sure to use the appropriate frame_id and encoding (`"rgb8"`). To use `image_proc`, publish `camera_info` messages:

```cpp
camera_info_pub_ = image_transport::create_camera_publisher(this, "stitched_image");
// and publish CameraInfo along with image
```

Use `camera_info_manager` to load intrinsics from a YAML file.

---

## Resources

- [GStreamer Documentation](https://gstreamer.freedesktop.org/documentation/)
- [libuvc](https://github.com/libuvc/libuvc)
- [THETA Z1 Manual](https://support.theta360.com/en/manual/z1/)
- [Official THETA SDK](https://github.com/ricohapi/theta-uvc-sdk)

---

## License

BSD 3-Clause License - See source headers for details.

---

Maintainer: Your Name (<your.email@example.com>)


# Yaw Tracker (C++ Implementation)

This ROS package tracks and records the yaw (rotation around the vertical axis) of a TurtleBot3 Burger robot using its IMU sensor. The initial orientation when the program starts is considered 0 degrees, and all subsequent yaw measurements are relative to this initial position.

## Features

- Sets the initial robot orientation as 0 degrees
- Continuously tracks yaw throughout operation
- Records yaw history with timestamps
- Publishes real-time yaw data on ROS topics
- Saves yaw history to a CSV file when the node is shut down
- Uses rqt_plot for visualization

## Installation

1. Create the package in your catkin workspace:

```bash
cd ~/catkin_ws/src
mkdir -p yaw_tracker/{src,launch}
```

2. Copy the source files:

   - `src/yaw_tracker_node.cpp`
   - `launch/yaw_tracker.launch`
   - `launch/yaw_visualizer.launch`
   - `CMakeLists.txt`
   - `package.xml`

3. Build the package:

```bash
cd ~/catkin_ws
catkin_make
```

4. Source your workspace:

```bash
source ~/catkin_ws/devel/setup.bash
```

## Usage

### Running the Yaw Tracker

To run the yaw tracker:

```bash
roslaunch yaw_tracker yaw_tracker.launch
```

This will start the `yaw_tracker` node, which will:

- Initialize and wait for the first IMU reading
- Set the initial yaw as 0 degrees
- Track yaw changes relative to the initial position
- Publish yaw data to the `/current_yaw` topic as a `Float64` message
- Publish yaw data to the `/yaw_stamped` topic as a `QuaternionStamped` message
- Save the complete yaw history to a CSV file (`yaw_history.csv`) when terminated

### Visualization

To visualize the yaw in real-time:

```bash
roslaunch yaw_tracker yaw_visualizer.launch
```

This will launch rqt_plot to display the yaw angle over time.

### Checking Raw IMU Data

To verify that IMU data is being published:

```bash
rostopic echo /imu
```

## Topics

- `/current_yaw` (std_msgs/Float64): The current yaw in degrees, relative to the initial position.
- `/yaw_stamped` (geometry_msgs/QuaternionStamped): The current yaw as a quaternion with timestamp.

## Customization

If your TurtleBot3's IMU topic is different from the default `/imu`, you can modify it by changing the parameter in the launch file:

```xml
<param name="imu_topic" value="/your_custom_imu_topic" />
```

## Implementation Details

- The yaw tracker uses the TF2 library to convert IMU quaternion data to Euler angles
- Yaw is calculated in degrees for easier readability
- The node normalizes yaw to keep it in the range of -180 to 180 degrees
- Data is saved to a CSV file when the node shuts down, making it easy to analyze later

## Dependencies

- ROS Noetic (or compatible version)
- C++ compiler with C++11 support
- TurtleBot3 ROS packages
- tf2
- tf2_ros
- tf2_geometry_msgs
- roscpp
- sensor_msgs
- geometry_msgs
- std_msgs

## Troubleshooting

- **No IMU data**: Ensure that your TurtleBot3 is publishing IMU data on the expected topic.

- **Compilation errors**: Make sure you have all the required dependencies installed:

  ```bash
  sudo apt-get install ros-noetic-tf2-geometry-msgs
  ```

  (Replace `noetic` with your ROS distribution)

- **IMU calibration**: If the yaw drifts significantly over time, you may need to calibrate the IMU sensor on your TurtleBot3.

## License

This package is released under the MIT License.

# perfect_velodyne
![ci](https://github.com/amslabtech/perfect_velodyne/workflows/ci/badge.svg)

Normal Estimation For Velodyne

# Requirement

- ROS2 (tested on foxy) 

# Install and build
```
cd your_ros2_ws/src
git clone https://github.com/amslabtech/perfect_velodyne.git
cd your_ros2_ws
rosdep install -i -r -y --from-paths src
colcon build 
```

# Run
```
source your_ros2_ws/install/local_setup.bash
ros2 launch perfect_velodyne normal_estimator_launch.py
```
#!/usr/bin/env bash
# Install the Nav2 navigation stack for Misty (nav2.launch.py) on ROS 2 Lyrical.
#
#   bash scripts/install_nav2.sh
#
# Nav2's core, robot_localization etc. come from apt. cv_bridge and aruco_opencv
# are built from source in src/third_party: their apt packages need Boost 1.83
# (Ubuntu 26.04), and this VM runs 26.10 (Boost 1.90). Not installed (not needed,
# and also broken on 26.10): the navigation2 / nav2_bringup metapackages,
# nav2_waypoint_follower, nav2_smac_planner.
set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"
R=ros-lyrical

sudo apt-get update
sudo apt-get install -y \
  $R-robot-state-publisher $R-tf2-ros $R-tf2-geometry-msgs $R-robot-localization \
  $R-depthimage-to-laserscan $R-nav2-msgs $R-nav2-controller $R-nav2-planner $R-nav2-smoother \
  $R-nav2-behaviors $R-nav2-bt-navigator $R-nav2-velocity-smoother $R-nav2-collision-monitor \
  $R-nav2-lifecycle-manager $R-nav2-regulated-pure-pursuit-controller $R-nav2-navfn-planner \
  $R-nav2-costmap-2d $R-aruco-opencv-msgs $R-image-transport $R-rclcpp-components \
  $R-rclcpp-lifecycle $R-ament-cmake-python $R-twist-mux \
  libboost-python-dev libopencv-dev liborocos-kdl-dev libyaml-cpp-dev python3-colcon-common-extensions

TP="$WS/src/third_party"
mkdir -p "$TP"
[ -d "$TP/vision_opencv" ] || git clone --depth 1 --branch 4.1.0 https://github.com/ros-perception/vision_opencv.git "$TP/vision_opencv"
[ -d "$TP/ros_aruco_opencv" ] || git clone --depth 1 --branch 7.0.0 https://github.com/fictionlab/ros_aruco_opencv.git "$TP/ros_aruco_opencv"
# only cv_bridge and aruco_opencv are built; the rest comes from apt or isn't needed
for d in vision_opencv/image_geometry vision_opencv/opencv_tests vision_opencv/vision_opencv ros_aruco_opencv/aruco_opencv_msgs; do
  touch "$TP/$d/COLCON_IGNORE"
done

set +u; source /opt/ros/lyrical/setup.bash; set -u
cd "$WS"
colcon build --base-paths src --cmake-args -DBUILD_TESTING=OFF
echo
echo "Done. Test without the robot:  source install/setup.bash && ros2 launch misty_ros nav2.launch.py sim:=2d"

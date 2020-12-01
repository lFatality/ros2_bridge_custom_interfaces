# How to call this script:
# source source_ros2.sh

echo "Sourcing ROS2 (Foxy)"
source /opt/ros/foxy/setup.bash
echo "Sourcing local ROS2 workspace"
SCRIPTPATH=$(dirname "$SCRIPT")
SETUPPATH="${SCRIPTPATH}/ros2ws/install/setup.bash"
if [ -f ${SETUPPATH} ]; then
  source ${SETUPPATH}
else
  echo "Couldn't source local ROS2 workspace, build it first!"
fi

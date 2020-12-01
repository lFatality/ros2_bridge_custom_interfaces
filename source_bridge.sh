# How to call this script:
# source source_bridge.sh

# This script is used when you want to run or compile the ros1_bridge.

# Adjust this value if your roscore is located elsewhere
ROS_MASTER_URI=http://localhost:11311

echo "Note: Any warnings about not mixing ROS environments can be ignored."
echo "This is because we actually want to source both in order to bridge them."
echo "Sourcing ROS1 (Noetic)"
source /opt/ros/noetic/setup.bash
echo "Sourcing ROS2 (Foxy)"
source /opt/ros/foxy/setup.bash
echo "Exporting ROS_MASTER_URI to ${ROS_MASTER_URI}"
export ROS_MASTER_URI=${ROS_MASTER_URI}
echo "Sourcing local ROS1 workspace"

SCRIPTPATH=$(dirname "$SCRIPT")
SETUPPATH1="${SCRIPTPATH}/ros1ws/devel/setup.bash"
if [ -f ${SETUPPATH1} ]; then
  source ${SETUPPATH1}
else
  echo -e "ERROR: Couldn't source local ROS1 workspace, build it first!"
fi

echo "Sourcing local ROS2 workspace"
SETUPPATH2="${SCRIPTPATH}/ros2ws/install/setup.bash"
if [ -f ${SETUPPATH2} ]; then
  source ${SETUPPATH2}
else
  echo -e "ERROR: Couldn't source local ROS2 workspace, build it first!"
fi


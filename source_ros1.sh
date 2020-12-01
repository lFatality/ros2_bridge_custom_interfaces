# How to call this script:
# source source_ros1.sh

echo "Sourcing ROS1 (Noetic)"
source /opt/ros/noetic/setup.bash
echo "Sourcing local ROS1 workspace"
SCRIPTPATH=$(dirname "$SCRIPT")
SETUPPATH="${SCRIPTPATH}/ros1ws/devel/setup.bash"
if [ -f ${SETUPPATH} ]; then
  source ${SETUPPATH}
else
  echo -e "ERROR: Couldn't source local ROS1 workspace, build it first!"
fi

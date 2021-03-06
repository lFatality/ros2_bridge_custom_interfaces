# Custom interface bridging between ROS 1 & 2

This repository gives an example how to bridge custom interfaces between ROS 1 & 2 using the ros1_bridge package (https://github.com/ros2/ros1_bridge).
At the time of writing this, it's possible to bridge topics, services & `tf2_msgs`, but not actions.

## Video
I've made a tutorial on how to transfer custom messages between ROS 1&2 that you can find here:  
https://www.youtube.com/watch?v=vBlUFIOHEIo

## TLDR

I'm just here for the commands:

Before you do anything, initialize the git submodule for the ros bridge.  
```
### initialize the ros_bridge submodule

git submodule update --init
```

Then you can continue with building the workspaces.  
Find the extremely condensed version first, after that you can find the same commands with more comments.

### I like to keep it short and simple (no comments)
```
### Installation

# Build ros1 workspace
source source_ros1.sh
cd ros1ws
catkin_make

### Build ros2 workspace
source source_ros2.sh
cd ros2ws
colcon build

### Build bridge workspace
source source_bridge.sh
cd bridge_ws
colcon build --symlink-install --cmake-force-configure
```

```
# Run
# roscore
source source_ros1.sh
roscore

# bridge
source source_bridge.sh
ros2 run ros1_bridge dynamic-bridge

### ros1 talker
source source_ros1.sh
rosrun custom_msg_ros1 talker.py

### ros2 listener
source source_ros2.sh
ros2 run pubsub listener

### ros2 talker
source source_ros2.sh
ros2 run pubsub talker

### Echo the topic in ros1
source source_ros1.sh
rostopic echo /chatter
```

### Could you explain that a bit more? (with comments)

```
### Installation
### start with a new console that does not have any ROS installation sourced
### make sure you don't source any ROS installation in your .bashrc

### Build ros1 workspace
### When sourcing you will get a notification that it can't source the local workspaces.
### that's ok, it was just to source the ROS1 installation.
### we will build the local workspace afterwards.

source source_ros1.sh
cd ros1ws
catkin_make

### Build ros2 workspace
### Open a new console!

source source_ros2.sh
cd ros2ws
colcon build

### Build bridge workspace
### Open a new console!

source source_bridge.sh
cd bridge_ws
colcon build --symlink-install --cmake-force-configure

### To check that it worked

source install/setup.bash ### (in bridge_ws)
ros2 run ros1_bridge dynamic_bridge --print-pairs

### in the output that is printed, search for the custom messages you created
### you can also search for them with grep (note: your message name might be different)

ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i CustomMessage 

### if you can't find them, something went wrong. 
### did you forget to source the bridge_ws after building it?
### make sure you follow the steps exactly.
### remove the devel, build, install folders in all workspaces and start again.
```

```
### Run
### start with a new console that does not have any ROS installation sourced
### make sure you don't source any ROS installation in your .bashrc

### Start roscore

source source_ros1.sh
roscore

### Run bridge
### Open a new console!

source source_bridge.sh
ros2 run ros1_bridge dynamic-bridge

### it might be necessary to run it like this instead if it doesn't work
### ros2 run ros1_bridge dynamic-bridge --bridge-all-topics

### Now you can choose who should send and who should receive

### -- ROS1 talker, ROS2 listener --

### Run ros1 talker
### Open a new console!

source source_ros1.sh
rosrun custom_msg_ros1 talker.py

### Run ros2 listener
### Open a new console!

source source_ros2.sh
ros2 run custom_msg_ros2 listener

### there is also an example for a listener in a different package as the messages

ros2 run pubsub listener

### -- ROS1 listener, ROS2 talker --

### Run ros2 talker
### Open a new console!

source source_ros2.sh
ros2 run pubsub talker

### Echo the topic in ros1
### Open a new console!

source source_ros1.sh
rostopic echo /chatter
```

## Some more info regarding the installation

### Sourcing scripts

To ease sourcing there are scripts that will do it for you.  
You can have a look inside them to see what they do.

The most interesting is probably the one to source the bridge_ws.
Here 6 things are happening:
- source the ROS1 installation
- source the ROS2 installation
- export the ROS_MASTER_URI so that ROS2 can find it
- source the local ROS1 installation
- source the local ROS2 installation (this and the previous step are necessary to let the bridge find the packages with the message definitions and build their mapping)
- source the bridge workspace (if you want to run the bridge, you have to source it's workspace to make custom message transmission work)

Note that the scripts expect a certain folder structure.
If your folder structure is different, you have to adjust the scripts.
Also if your roscore is located elsewhere, you have to adjust your ROS_MASTER_URI in the scripts.

### Verbose build

If you're debugging and want a more verbose output while building:
```
colcon build --symlink-install --cmake-force-configure --event-handlers console_direct+
```
Or even more verbose:
```
VERBOSE=1 colcon build --symlink-install --cmake-force-configure --event-handlers console_direct+
```
While building the bridge_ws you should see in the verbose console output that CMake finds both the ROS 1&2 package (here: `custom_msg_ros1`, `custom_msg_ros2`) and interacts with them.

## Useful documentation  
https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst  
https://github.com/ros2/ros1_bridge

## How to do it yourself

## - Ros1 workspace

### Custom messages & pub/sub node
Create a custom message / service as described here: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv  
If you want, create a publisher or subscriber node as described here: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(c%2B%2B)

Adjust the node so that it publishes / subscribes to a topic with your own custom message.  
You can refer to the code in `ros1ws/src/custom_msg_ros1/scripts/talker.py` and `ros1ws/src/custom_msg_ros1/CMakeLists.txt` for an example how to do this.

## - Ros2 workspace

### Custom messages & pub/sub node
Create a custom message / service as described here:
If you want, create a publisher or subscriber node as described here: https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/

You can also have a look at a repository I've created with examples for ROS2 basics: https://github.com/lFatality/ros2_basics

Adjust the node so that it publishes / subscribes to a topic with your own custom message.  
You can refer to the code in `ros2ws/src/pubsub/src/listener.cpp` and `ros2ws/src/pubsub/CMakeLists.txt` for an example how to do this.

If you want to use a node within the same package as the msg definition, don't forget to add this in the `CMakeLists.txt` of the package.
```
rosidl_target_interfaces(your_executable_name
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
```

### Yaml file for custom mapping

It might be the case that your ROS 1&2 package, message, or message fields are not similar.  
Example:
```
ROS1:
package name: my_ros1_package
message name: myRos1Message.msg
field name: int32 myRos1Value

ROS2:
package name: my_ros2_package
message name: myRos2Message.msg
field name: int32 myRos2Value
```

Still you might want to transfer the message between ROS 1&2.  
For that you need to create a YAML file specifying the mapping.

Note: This mapping file is necessary only if there are differences between your ROS 1&2 messages. If they have the same package name, message name, and field names, you don't need this mapping file. If your messages are not created as expected it still might be worth a shot to try to at least specify the ros package names even if they're similar.

Put the YAML file inside your ROS2 package with the message definitions.

Here is an example for the file structure (source: https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst)
```
.
├─ ros1_msgs_ws
│  └─ src
│     └─ bridge_msgs
│        └─ msg
│           └─ CustomMessage.msg
├─ ros2_msgs_ws
│  └─ src
│     └─ bridge_msgs
│        ├─ msg
│        │  └─ CustomMessage.msg
│        └─ my_mapping.yaml # YAML file if your custom interfaces have non-matching names
└─ bridge_ws
   └─ src
      └─ ros1_bridge
```

For this repo the name of the packages is different. For this mapping the file looks like this:
```
- 
  ros1_package_name: 'custom_msg_ros1'
  ros2_package_name: 'custom_msg_ros2'
```
Note the `-` symbol in the first line and the intendation.

Next to this YAML file, you have to adjust the `CMakeLists.txt` and `package.xml` of the ROS2 package with the message definitions.

### `CMakeLists.txt`
```
install(
  FILES my_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME})
```

### `package.xml`
```
<export>
  <ros1_bridge mapping_rules="my_mapping_rules.yaml"/>
</export>
```

Make sure to build your ROS2 workspace again after adding the `install` statement for the `.yaml` file!  
Otherwise it will not be in the `install` folder and the bridge will not know about the mapping!
If you forgot it, make sure to source your workspace again before building.

For more complex mapping examples for both topics and services as well as additional information, refer to the ros_bridge documentation: https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst

## - Bridge workspace

Create the workspace.
```
mkdir bridge_ws
cd bridge_ws
mkdir src
```

Clone the `ros1_bridge`.
You can either just clone it or use a git submodule.
```
Option A, simple clone:
git clone https://github.com/ros2/ros1_bridge.git
Option B, git submodule:
git submodule add https://github.com/ros2/ros1_bridge.git
```

Be sure that you're on the right branch for your ROS distro.

```
git checkout your_ros_distro
```

Before building the bridge be sure to have sourced both your ROS 1 & 2 installation as well as any workspaces that contain custom messages you want to map.

# UB ROSbot Navigation Project:
by:  
George Knibbs  
Henrique Mira  
Ivan Li Hoi Him  
Luke Sykes  
Youssef Hany

## WHICH FILES GO WHERE:

-In `[workspace name]/src/[package name]`, ie: `~/ros_workspace/src/rosbot_bath`, REPLACE:
	- `package.xml`
	- `CMakeLists.txt`
with the ones in this folder, or you can modify the files yourself, guides below.

-In `[workspace name]/src/[package name]/src`, ie: `~/ros_workspace/src/rosbot_bath/src`, ADD:
	- `distanceHandler.py`
	- `lidarHandler.py`
	- `objectAvoid.py`
	- `terminate.py`
	- `yawHandler.py`
basically all the .py scripts in the "SCRIPTS" folder

-In `[workspace name]/src/[package name]/srv`, ie: `~/ros_workspace/src/rosbot_bath/srv`, ADD:
	- `distanceHandler.srv`

-In `[workspace name]/src/[package name]/msg`, ie: `~/ros_workspace/src/rosbot_bath/msg`, ADD:
	- `lidarHandler.msg`

-In `[workspace name]/src/[package name]/launch`, ie: `~/ros_workspace/src/rosbot_bath/launch`, ADD:
	- `rosbot_test.launch`

### REMINDER: 
1. Make them executable programs (either by checking the option in file properties tab/ do "chmod +x [path to file]/[filename]")
2. Do "catkin_make" (and probably "source /devel.setup.sh") after doing all the modifications required
	  
## TO RUN THE SCRIPTS:

In `[workspace name]/src/[package name]`, ie: `~/ros_workspace/src/rosbot_bath`:
	1: `./run_rosbot.sh`

In separate terminal, in `[workspace name]`, ie: `~/ros_workspace`:
	2: `roslaunch [package name] [launch file name]`, ie: `roslaunch rosbot_bath rosbot_test.launch`

## -Restructured the sensor handling data such that:
* instead of having the only logic script subscribing to all the sensory data and doing all the logic stuff to process them, 
* we now have different nodes subscribed to different sensors, process them into data used by the logic handling program, 
* then publish/respond-when-needed the data to the logic node for all the fancy stuff

- The processing/ logic itself is 99.9% identical to its original version,
only split into different scripts with extra lines of code to link them up.

- If you run the scripts as intended it the rosbot should behave exactly as before.

- However, this assumes the original logic works fine, and you already understand what's going on.

- For a visual on how the structure has changed, please refer to "Ros Structure Comparison.png"

#### NOTE: The way the logic node interacts with the controls of the robot is completely untouched, as I'm tired AF and this is the part where we can add action nodes, which I have 0 clue how they work.

---

## In the new Structure:

- the new TOPICS have the same names as the new NODES and new SCRIPTS because I'm not sure which is which either and I can't be bothered to change them now
- The Proxy Sensors are used directly in the code and they are needed constantly, so they are directly PUBLISHED to the logic node

### new yawHandler.py:
1. takes imu data by subscribing to `/imu`
2. does the euler thing to turn them into understandable orientations
3. change the measurement of `current_yaw` (the only orientation we care about) from radians to degrees
4. measure difference from ideal orientation (0 degrees)
5. PUBLISHES with the topic `yawHandler` of the difference in yaw as that's all we care about
- The odomHandler needs constant update on robot orientation to do fwd_distance measurements, topic is preferred, 
else service would have been fine as the data is only needed during CORRECTION state of the robot

### new distanceHandler.py:
1. takes odom data by SUBSCRIBING to `/odom`
2. does the distance formula things
3. takes yaw error data by SUBSCRIBING to `yawHandler`
4. calculates forward distance by normalizing the distance traveled based on the angle
4.5. this method might not be right, could be modified later but right now, idc
5. responds the data with topic `distanceHandler` as a SERVICE
- The distance logging is just for data logging, just call it occasionally and on code termination

### new distanceHandler.srv:
- new .srv file as distanceHandler returns to distance data:
	1. `total_distance`: total distance the robot has traveled in all 4 directions
	2. `total_fwd_distance`: total distance the robot has traveled FORWARDS

### new lidarHandler.py:
1. takes the massive array of lidar values from `/LaserScan`
2. does the split-lidar-data-into-quadrants thing
3. PUBLISHES the 3 data values with topic `lidarHandler`
- The bot constantly searches stuff with lidar data 24/7, it's better to use topics for constant data updates

### new lidarHandler.msg:
- custom .msg file cause the script publishes 3 float32 datas for:
	- front
	- right
	- left

### modified objectAvoid.py:
- Same Script as the old test.py, but with the sensory logic functions removed
- renamed for convenience

### new termination.py:
- Simply here to request and log distance info from odomHandler on program termination

### new rosbot_test.launch:
- runs all the scripts at once
- you still need a separate terminal to do `./run_rosbot.sh` though

### modified CMakeLists.txt and package.xml:
- needed so that the new .srv and .msg files could work


## Benefits:
- clearer structure now that the big code is separated into smaller ones 
- better resource management as less important functions are now services that only run when needed
- better division of labor as more people can work on different parts of the robot instead of everyone diving into the same code
- probably some sort of more efficient debugging/failsafe that I need time to think about how to BS through


## CMakeLists.txt MODIFICATIONS:

Have something like this:
~~~
find_package(catkin REQUIRED COMPONENTS
  roscpp tf
  rospy
  std_msgs
  message_generation
)

...

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   lidarHandler.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   distanceHandler.srv
#   Service1.srv
#   Service2.srv
 )

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   objectAvoid.action
#   Action1.action
#   Action2.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   	DEPENDENCIES
   	actionlib_msgs
	std_msgs 
	# Or other packages containing msgs
)

~~~
...
~~~

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rosbot_bath
  CATKIN_DEPENDS roscpp
  std_msgs
#  DEPENDS system_lib
)

--------------------------------------------------------------------------

package.xml MODIFICATIONS:
Have something like:
<!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
 <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
  <build_depend>actionlib_msgs</build_depend>
  <exec_depend>actionlib_msgs</exec_depend>

---
commented code stuff
---

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <build_depend>std_msgs</build_depend>
  <exec_depend>std_msgs</exec_depend>

---
uncommented code stuff
---

--------------------------------------------------------------------------
~~~

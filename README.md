# HW11
```
cd ~/catkin_ws/src/hw11/
mkdir msg
cd msg
nano Drive.py
```
Add this to the file:
```
	float32 steering
	float32 speed
```
To Save and exit nano:
```
Cntl-O
Cntl-X
```

```
cd ..
nano package.xml
```
make sure these lines are in the package.xml file:
```
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```
To save and exit from nano:

```
Cntl-O
Cntl-X
```
Next:
```
mkdir srv
nano Service.srv
```
add these lines to the Service file:
```
  float32 steering
	float32 speed
```
to save and exit nano:
```
Cntl-O
Cntl-X
```
next:
```
nano CMakeLists.txt
```
Do not just add this to your CMakeLists.txt, modify the existing text to add message_generation before the closing parenthesis with this:

```
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)

	catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```
  
Also uncomment this and edit

```
  # add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
```
To look like this:
```
add_message_files(
   FILES
   Drive.msg
 )
```
Uncomment this:
```
	# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
```
Change this:
```
	# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
```
To this:
```
add_service_files(
  FILES
  Service.srv
)
```
to save and exit nano:
```
Ctnl-O
Cntl-X
```
Compile:
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
catkin_make install
```

For the sub/pub
http://wiki.ros.org/ROS/Tutorials/CustomMessagePublisherSubscriber%28python%29
	
	

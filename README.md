# HW11 Custom Ros Messages

This assumes you have a ros workspace and package.  Here, the ros workspace is called 'catkin_ws' and the package is called 'hw11'.

### Create the message scheme
In this folder, each .msg file will be a separate message 'type.'

For this scenario, the variables 'speed' and 'steering' will be sent together, so we use a single message, 'Drive.'

```
cd ~/catkin_ws/src/hw11/
mkdir msg
cd msg
nano Drive.py
```

Both 'steering' and 'speed' are floats, so we populate the file with the variable type and its name:

```
float32 steering
float32 speed
```

To Save and exit nano:
```
Cntl-O
Cntl-X
```
### Add the message to the package

Open the package xml file in the package directory

```
cd ~/catkin_ws/src/hw11/
nano package.xml
```

Make sure these lines are in the package.xml file UNCOMMENTED:

```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

To save and exit from nano:

```
Cntl-O
Cntl-X
```
### Creating a Service File

```
cd ~/catkin_ws/src/hm11/
mkdir srv
nano Service.srv
```

Add these lines to the Service file:
```
float32 steering
float32 speed
```

To save and exit nano:
```
Cntl-O
Cntl-X
```
### Adjust the CMakeList file

```
cd ~/catkin_ws/src/hw11/
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
To save and exit nano:
```
Ctnl-O
Cntl-X
```
### Compile

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
catkin_make install
```

#### For more information about creating custom ROS messages:
```
http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
```

For the sub/pub
http://wiki.ros.org/ROS/Tutorials/CustomMessagePublisherSubscriber%28python%29
	
	

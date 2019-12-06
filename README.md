# HW11 Custom Ros Messages

This assumes you have a ros workspace and package.  Here, the ros workspace is called 'catkin_ws' and the package is called 'hw11'.

### Create the message scheme
In this folder, each .msg file will be a separate message 'type.'

For this scenario, the variables 'speed' and 'steering' will be sent together, so we use a single message, 'Drive.'

```
cd ~/catkin_ws/src/hw11/
mkdir msg
cd msg
nano Drive.msg
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
http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv


### Writing a subscriber with custom ros messages
First, declarations at the top, with dependenccies and import statements.

```
#!/usr/bin/env python

#import dependencies
import rospy
from hw11.msg import Drive
import time
```
To import your custom message, use
```
from <package name>.msg import <message name>
```
In this example, that is 
```
from hw11.msg import Drive
```
Next, we have our callback method which is called everytime new data is published
```
def callback(data):
    #log data
    rospy.loginfo(rospy.get_caller_id() + "I heard %f for speed and %f for steering", data.speed, data.steering)
    print("Speed: ", data.speed, " Steering: ", data.steering)
```
Here, the variable 'data' represents our custom message coming in.  To distinguish the two data variables, data.speed and data.steering are used.

Next, the listener listens for the message.  Here, 'Drive' is the message the subscriber is looking for from the topic 'carcontrol.'

```
def listener():
    # see ros wiki for more information
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("carcontrol", Drive, callback)
    rospy.spin()
```
We also have a main definition that simply calls our subscriber:
```
if __name__== '__main__':
    listener()
```

Here is the code all together:
```
#!/usr/bin/env python

#import dependencies
import rospy
from hw11.msg import Drive
import time

#method will be called when new data is published
def callback(data):
    #log data
    rospy.loginfo(rospy.get_caller_id() + "I heard %f for speed and %f for steering", data.speed, data.steering)
    print("Speed: ", data.speed, " Steering: ", data.steering)
def listener():
    # see ros wiki for more information
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("carcontrol", Drive, callback)
    rospy.spin()

if __name__== '__main__':
    listener()
```

### Writing a publisher with custom ros messages

First, we again have the import statements with our dependencies.  Importing the custom message here is the same as the subscriber.
```
#!/usr/bin/env python

#import dependencies
import rospy
from hw11.msg import Drive

import logging
import time
```
Next, we set up the node it will be talking to.
```
def talker():
    #set up publisher
    pub = rospy.Publisher('carcontrol', Drive)
    rospy.init_node('cust_talker', anonymous=True)
    rate = rospy.Rate(1) #1 Hz
```
Within that method, we set up the individual variables within our custom message.  Here, the two variables are hardcoded to 0.16 and 0.17, but this can be easily changed.
```
    msg = Drive()
    msg.steering = 0.16
    msg.speed = 0.17
```
Next, we publish the data like in a normal publisher.
```
    while not rospy.is_shutdown():
        #log data from sensor
        rospy.loginfo(msg)
        #publish data from sensor
        pub.publish(msg)
        #sleep based on Hz from earlier (1 Hz = sleep for 1 second; 2 Hz = sleep for 0.5 seconds; etc)
        rate.sleep()
```
Finally, we have the main method that calls the publisher and exits on an interrupt:
```
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

Here is the code all together:

```
#!/usr/bin/env python

#import dependencies
import rospy
from hw11.msg import Drive

import logging
import time

def talker():
    #set up publisher
    pub = rospy.Publisher('carcontrol', Drive)
    rospy.init_node('cust_talker', anonymous=True)
    rate = rospy.Rate(1) #1 Hz
    msg = Drive()
    msg.steering = 0.16
    msg.speed = 0.17
    while not rospy.is_shutdown():
        #log data from sensor
        rospy.loginfo(msg)
        #publish data from sensor
        pub.publish(msg)
        #sleep based on Hz from earlier (1 Hz = sleep for 1 second; 2 Hz = sleep for 0.5 seconds; etc)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

#### For more information about creating a subscriber and publisher
##### Simple Subscribers/Publishers:
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

##### Custom Subscribers/Publishers:
http://wiki.ros.org/ROS/Tutorials/CustomMessagePublisherSubscriber%28python%29
	
	

# robotcar_plotter


Gives an example how you can use Subscribers for plotting sensor informations from the [RobotCar](https://github.com/Michdo93/robotcar) using matplotlib. At first you have to make sure that the roscore is running and the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) is publishing the sensor informations.

The String variable `robot_host` uses the hostname of one RobotCar. As example it could be `robotcar`.

A [robotcar_subscriber](https://github.com/Michdo93/robotcar_subscriber) could be used as blue print for subscribing informations for an ADAS. The robotcar_plotter could be used as example for visualize this informations.

## Prerequisites

```
cd ~/catkin_ws/src
git clone https://github.com/ros/common_msgs.git
cd .. && catkin_make
```

## FrontIRPlotter Node

It subscribes informations from the FrontInfrared Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the front infrared sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /infrared/front/distance                  | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter frontIRPlotter.py`

## FrontLeftUltrasonicPlotter Node

It subscribes informations from the FrontLeftUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the left front ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/front/left/distance           | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter frontLeftUltrasonicPlotter.py`

## FrontRightUltrasonicPlotter Node

It subscribes informations from the FrontRightUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the right front ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/front/right/distance          | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter frontRightUltrasonicPlotter.py`

## FrontTofPlotter Node

It subscribes informations from the FrontTimeOfFlight Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the front time-of-flight sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /time_of_flight/front/distance            | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter frontToFPlotter.py`

## FrontUltrasonicPlotter Node

It subscribes informations from the FrontUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the front ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/front/distance                | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter frontUltrasonicPlotter.py`

## RearIRPlotter Node

It subscribes informations from the RearInfrared Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the rear infrared sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /infrared/rear/distance                   | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter rearIRPlotter.py`

## RearLeftUltrasonicPlotter Node

It subscribes informations from the RearLeftUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the left rear ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/rear/left/distance            | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter rearLeftUltrasonicPlotter.py`

## RearRightUltrasonicPlotter Node

It subscribes informations from the RearRightUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the right rear ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/rear/right/distance           | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter rearRightUltrasonicPlotter.py`

## RearToFPlotter Node

It subscribes informations from the RearTimeOfFlight Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the rear time-of-flight sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /time_of_flight/rear/distance             | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter rearToFPlotter.py`

## RearUltrasonicPlotter Node

It subscribes informations from the RearUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the rear ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/rear/distance                 | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter rearUltrasonicPlotter.py`

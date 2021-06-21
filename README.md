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

## AccelerometerCurvesPlotter Node

![accelerometerCurvesPlotter](https://raw.githubusercontent.com/Michdo93/robotcar_plotter/main/accelerometerCurvesPlotter.JPG)

It subscribes informations from the IMU Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg).

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /imu/accelerometer/raw                        | [robotcar_msgs/Accelerometer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Accelerometer.md)    |

You can run it with `rosrun robotcar_plotter accelerometerCurvesPlotter.py`

## AccelerometerPlotter Node

![accelerometerPlotter](https://raw.githubusercontent.com/Michdo93/robotcar_plotter/main/accelerometerPlotter.JPG)

It subscribes informations from the IMU Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg).

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /imu/accelerometer                        | [robotcar_msgs/Accelerometer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Accelerometer.md)    |

You can run it with `rosrun robotcar_plotter accelerometerPlotter.py`

## Compass Node

![compass](https://raw.githubusercontent.com/Michdo93/robotcar_plotter/main/compassPlotter.JPG)

It subscribes informations from the IMU Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg).

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /imu/magenetometer                        | [robotcar_msgs/Magnetometer](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Magnetometer.md)     |

You can run it with `rosrun robotcar_plotter compass.py`

## FrontIRPlotter Node

![FrontIRPlotter](https://raw.githubusercontent.com/Michdo93/robotcar_plotter/main/frontIRPlotter.JPG)

It subscribes informations from the FrontInfrared Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the front infrared sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /infrared/front/distance                  | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter frontIRPlotter.py`

## FrontLeftUltrasonicPlotter Node

![FrontLeftUltrasonicPlotter](https://raw.githubusercontent.com/Michdo93/robotcar_plotter/main/frontLeftUltrasonicPlotter.JPG)

It subscribes informations from the FrontLeftUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the left front ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/front/left/distance           | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter frontLeftUltrasonicPlotter.py`

## FrontRightUltrasonicPlotter Node

![FrontRightUltrasonicPlotter](https://raw.githubusercontent.com/Michdo93/robotcar_plotter/main/frontRightUltrasonicPlotter.JPG)

It subscribes informations from the FrontRightUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the right front ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/front/right/distance          | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter frontRightUltrasonicPlotter.py`

## FrontTofPlotter Node

![FrontTofPlotter](https://raw.githubusercontent.com/Michdo93/robotcar_plotter/main/frontToFPlotter.JPG)

It subscribes informations from the FrontTimeOfFlight Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the front time-of-flight sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /time_of_flight/front/distance            | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter frontToFPlotter.py`

## FrontUltrasonicPlotter Node

![FrontUltrasonicPlotter](https://raw.githubusercontent.com/Michdo93/robotcar_plotter/main/frontUltrasonicPlotter.JPG)

It subscribes informations from the FrontUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the front ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/front/distance                | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter frontUltrasonicPlotter.py`

## GyroscopeCurvesPlotter Node

![gyroscopeCurvesPlotter](https://raw.githubusercontent.com/Michdo93/robotcar_plotter/main/gyroscopeCurvesPlotter.JPG)

It subscribes informations from the IMU Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg).

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /imu/gyroscope/raw                        | [robotcar_msgs/Gyroscope](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Gyroscope.md)        |

You can run it with `rosrun robotcar_plotter gyroscopeCurvesPlotter.py`

## GyroscopePlotter Node

![gyroscopePlotter](https://raw.githubusercontent.com/Michdo93/robotcar_plotter/main/gyroscopePlotter.JPG)

It subscribes informations from the IMU Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg).

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /imu/gyroscope                       | [robotcar_msgs/Gyroscope](https://github.com/Michdo93/robotcar_msgs/blob/master/doku/Gyroscope.md)        |

You can run it with `rosrun robotcar_plotter gyroscopePlotter.py`


## RearIRPlotter Node

![RearIRPlotter](https://raw.githubusercontent.com/Michdo93/robotcar_plotter/main/rearIRPlotter.JPG)

It subscribes informations from the RearInfrared Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the rear infrared sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /infrared/rear/distance                   | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter rearIRPlotter.py`

## RearLeftUltrasonicPlotter Node

![RearLeftUltrasonicPlotter](https://raw.githubusercontent.com/Michdo93/robotcar_plotter/main/rearLeftUltrasonicPlotter.JPG)

It subscribes informations from the RearLeftUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the left rear ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/rear/left/distance            | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter rearLeftUltrasonicPlotter.py`

## RearRightUltrasonicPlotter Node

![RearRightUltrasonicPlotter](https://raw.githubusercontent.com/Michdo93/robotcar_plotter/main/rearRightUltrasoncPlotter.JPG)

It subscribes informations from the RearRightUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the right rear ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/rear/right/distance           | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter rearRightUltrasonicPlotter.py`

## RearToFPlotter Node

![RearToFPlotter](https://raw.githubusercontent.com/Michdo93/robotcar_plotter/main/rearTofPlotter.JPG)

It subscribes informations from the RearTimeOfFlight Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the rear time-of-flight sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /time_of_flight/rear/distance             | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter rearToFPlotter.py`

## RearUltrasonicPlotter Node

![RearUltrasonicPlotter](https://raw.githubusercontent.com/Michdo93/robotcar_plotter/main/rearUltrasonicPlotter.JPG)

It subscribes informations from the RearUltrasonic Node of the [robotcar-pkg](https://github.com/Michdo93/robotcar-pkg) respectively the rear ultrasonic sensor.

|                       Topic Address                   |             Message Type        |
|------------------------------------------------------ | --------------------------------|
|robot_host + /ultrasonic/rear/distance                 | [sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)             |

You can run it with `rosrun robotcar_plotter rearUltrasonicPlotter.py`

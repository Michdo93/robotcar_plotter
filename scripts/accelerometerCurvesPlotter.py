#!/usr/bin/env python
import os
import sys
import re
import socket
import rospy
from robotcar_msgs.msg import Accelerometer
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits import mplot3d
plt.style.use('seaborn')
plt.ion()

class AccelerometerCurvesPlotter(object):

    def __init__(self, robot_host):
        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.robot_host = robot_host
        self.sub = rospy.Subscriber(self.robot_host + '/imu/accelerometer/raw', accelerometer, self.callback)

        # Initialize message variables.
        self.enable = False
        self.data = ""

        self.x = []
        self.y = []
        self.z = []
        self.c = []

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.sub = rospy.Subscriber(self.robot_host + '/imu/accelerometer/raw', accelerometer, self.callback)

    def stop(self):
        """Turn off subscriber."""
        self.enable = False
        self.sub.unregister()

    def callback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        self.data = data

        self.x.append(self.data.x)
        self.y.append(self.data.y)
        self.z.append(self.data.z)
        self.c.append(self.data.header.stamp)

        plt.clf()

        plt.subplot(3, 1, 1)
        plt.plot(self.c, self.x, '.-')
        plt.title('A tale of 3 subplots')
        plt.ylabel('X acceleration')

        plt.subplot(3, 1, 2)
        plt.plot(self.c, self.y, '.-')
        plt.xlabel('time (s)')
        plt.ylabel('Y acceleration')

        plt.subplot(3, 1, 3)
        plt.plot(self.c, self.z, '.-')
        plt.xlabel('time (s)')
        plt.ylabel('Z acceleration')

        plt.show()
        plt.pause(.000001)

if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_AccelerometerCurvesPlotter"
    rospy.init_node(node_name, anonymous=False)
    
    accelerometer = AccelerometerCurvesPlotter("robotcar")
    
    # Go to the main loop
    try:
        accelerometer.start()
        # Wait for messages on topic, go to callback function when new messages arrive.
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    # Stop with Ctrl + C
    except KeyboardInterrupt:
        accelerometer.stop()

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill " + node_name)

        
        print("Node stopped")

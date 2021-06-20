#!/usr/bin/env python
import os
import sys
import re
import socket
import rospy
from robotcar_msgs.msg import Accelerometer
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits import mplot3d
import math
plt.style.use('seaborn')
plt.ion()

class AccelerometerPlotter(object):

    def __init__(self, robot_host):
        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.robot_host = robot_host
        self.sub = rospy.Subscriber(self.robot_host + '/imu/accelerometer', Accelerometer, self.callback)

        # Initialize message variables.
        self.enable = False
        self.data = ""

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.sub = rospy.Subscriber(self.robot_host + '/imu/accelerometer', Accelerometer, self.callback)

    def stop(self):
        """Turn off subscriber."""
        self.enable = False
        self.sub.unregister()

    def callback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        self.data = data

        self.ax.clear()

        x1 = 100 * (math.cos(self.data.z) * math.cos(self.data.y))
        y1 = 100 * (math.cos(self.data.x) * math.sin(self.data.y) + math.cos(self.data.y) * math.sin(self.data.x) * math.sin(self.data.z))

        x2 = 100 * (-(math.cos(self.data.z)) * math.sin(self.data.y))
        y2 = 100 * (math.cos(self.data.x) * math.cos(self.data.y) - math.sin(self.data.x) * math.sin(self.data.z) * math.sin(self.data.y))

        x3 = 100 * (math.sin(self.data.z))
        y3 = 100 * (-(math.cos(self.data.z)) * math.sin(self.data.x))

        self.ax.quiver(0.5, 0.5, 0.5, [x1,x2,x3], [y1,y2,y3], [1, 1, 1], colors=['r', 'g', 'b'], length=0.5, normalize=True)

        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])

        plt.show()
        plt.pause(.000001)

if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_AccelerometerPlotter"
    rospy.init_node(node_name, anonymous=False)
    
    accelerometer = AccelerometerPlotter("robotcar")
    
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

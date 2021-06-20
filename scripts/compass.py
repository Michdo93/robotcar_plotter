#!/usr/bin/env python
import os
import sys
import re
import socket
import rospy
from robotcar_msgs.msg import Magnetometer
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

plt.style.use('seaborn')
plt.ion()

class CompassPlotter(object):

    def __init__(self, robot_host):
        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.robot_host = robot_host
        self.sub = rospy.Subscriber(self.robot_host + '/imu/magnetometer/raw', Magnetometer, self.callback)

        # Initialize message variables.
        self.enable = False
        self.data = ""

        self.fig = plt.figure(num='Compass', figsize=[5, 3])
        self.ax = plt.subplot(projection='polar')
        self.i = 0

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.sub = rospy.Subscriber(self.robot_host + '/imu/magnetometer/raw', Magnetometer, self.callback)

    def stop(self):
        """Turn off subscriber."""
        self.enable = False
        self.sub.unregister()

    def callback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        self.data = data
        north = self.data.north
    
        self.ax.clear()
        self.ax.set_theta_zero_location("N")
        self.ax.set_theta_direction(-1)
        self.ax.set_ylim(top=1)
        
        # arrow at 45 degree
        plt.arrow(north/180.*np.pi, 0.0, 0, 1, alpha = 0.5, width = 0.015,
                        edgecolor = 'black', facecolor = 'green', lw = 2, zorder = 5)

        plt.show()
        plt.pause(.000001)

if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_CompassPlotter"
    rospy.init_node(node_name, anonymous=False)
    
    compass = CompassPlotter("robotcar")
    
    # Go to the main loop
    try:
        compass.start()
        # Wait for messages on topic, go to callback function when new messages arrive.
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    # Stop with Ctrl + C
    except KeyboardInterrupt:
        compass.stop()

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill " + node_name)

        
        print("Node stopped")

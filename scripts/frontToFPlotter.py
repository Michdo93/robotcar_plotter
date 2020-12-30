#!/usr/bin/env python
import os
import sys
import re
import socket
import rospy
from sensor_msgs.msg import Range
import numpy as np
import matplotlib.pyplot as plt
plt.style.use('seaborn')
plt.ion()

class FrontToFPlotter(object):

    def __init__(self, robot_host):
        """Configure subscriber."""
        # Create a subscriber with appropriate topic, custom message and name of
        # callback function.
        self.robot_host = robot_host
        self.sub = rospy.Subscriber(self.robot_host + '/time_of_flight/front/distance', Range, self.callback)

        # Initialize message variables.
        self.enable = False
        self.data = ""

        self.x0 = []
        self.y0 = []
        self.i = 0

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        self.enable = True
        self.sub = rospy.Subscriber(self.robot_host + '/time_of_flight/front/distance', Range, self.callback)

    def stop(self):
        """Turn off subscriber."""
        self.enable = False
        self.sub.unregister()

    def callback(self, data):
        """Handle subscriber data."""
        # Simply print out values in our custom message.
        self.data = data
        msg = "Got type %s with FoV %s and Min-Range %s and Max-Range %s and measured Range %s" % (self.data.radiation_type, self.data.field_of_view, self.data.min_range, self.data.max_range, self.data.range)
        rospy.loginfo(rospy.get_caller_id() + msg)

        self.x0.append(self.i)
        self.y0.append(self.data.range)

        self.i = self.i + 1

        plt.ylim(0, 300)
        plt.yticks(np.arange(0, 300, step=20))
        plt.xlim(left=max(0, self.i-50), right=self.i+40)

        plt.ylabel('OBJECT DISTANCE (cm)', fontname='monospace', color='black', fontsize=14)
        plt.title('FRONT TIME-OF-FLIGHT SENSOR - ENVIRONMENT MAP', fontname='monospace', color='black', fontsize=16)

        p1, = plt.plot(self.x0, self.y0, color='b')
        plt.legend([p1], ['TIME-OF-FLIGHT Reading'], prop={'family': 'monospace'}, loc='upper right', frameon=True)

        plt.grid(True)
        plt.show()
        plt.pause(.000001)

if __name__ == '__main__':
    # Initialize the node and name it.
    node_name = re.sub("-", "_", socket.gethostname()) + "_FrontToFPlotter"

    rospy.init_node(node_name, anonymous=False)
    
    tof = FrontToFPlotter("robotcar")
    
    # Go to the main loop
    try:
        tof.start()
        # Wait for messages on topic, go to callback function when new messages arrive.
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    # Stop with Ctrl + C
    except KeyboardInterrupt:
        tof.stop()

        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            os.system("rosnode kill " + node_name)

        
        print("Node stopped")

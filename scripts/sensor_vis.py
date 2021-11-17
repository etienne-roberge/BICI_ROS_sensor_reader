#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from serial_comm.msg import TactileData
import numpy as np


class SensorDataVis:
    def __init__(self, sensor_num):
        self.sensor_num = sensor_num
        self.data_min = np.inf
        self.data_max = -np.inf

        taxels = {}
        self.finger_small = [8, 9, 12, 13, 14, 17, 18, 19, 20, 24, 25, 26, 29]
        self.finger_big = [10, 15, 27]
        self.fingertip = [11, 16, 23, 28]
        self.palm = [21]
        self.boh = [22]
        taxels.update(dict.fromkeys(self.finger_small, (3, 2)))
        taxels.update(dict.fromkeys(self.finger_big, (5, 3)))
        taxels.update(dict.fromkeys(self.fingertip, (7, 2)))
        taxels.update(dict.fromkeys(self.palm, (4, 7)))
        taxels.update(dict.fromkeys(self.boh, (4, 7)))

        self.taxels = taxels

    def norm_tactile(self, data):
        self.data_min = min(self.data_min, min(data))
        self.data_max = max(self.data_max, max(data))
        return ((data - self.data_min)/(self.data_max-self.data_min)).clip(0, 1)

    def transform_finger_small(self, data):
        order = [1, 2, 0, 3, 5, 4]
        sorted_data = np.asarray([data[i] for i in order])
        return np.reshape(self.norm_tactile(sorted_data), self.taxels[self.sensor_num])

    def transform_finger_big(self, data):
        order = [12, 9, 4, 13, 8, 3, 14, 7, 2, 11, 6, 1, 10, 5, 0]
        sorted_data = np.asarray([data[i] for i in order])
        return np.reshape(self.norm_tactile(sorted_data), self.taxels[self.sensor_num])

    def transform_fingertip(self, data):
        order = [13, 6, 12, 5, 11, 4, 10, 3, 9, 2, 8, 1, 7, 0]
        sorted_data = np.asarray([data[i] for i in order])
        return np.reshape(self.norm_tactile(sorted_data), self.taxels[self.sensor_num])

    def transform_palm(self, data):
        data = list(data)
        data.extend([0]*8)
        order = [9, 8, 7, 6, 5, 4, 3, 10, 11, 12, 13, 0, 1, 2, 16, 15, 14, 20, 21, 22, 23, 17, 18, 19, 25, 25, 26, 27]
        sorted_data = np.asarray([data[i] for i in order])
        return np.reshape(self.norm_tactile(sorted_data), self.taxels[self.sensor_num])

    def transform_boh(self, data):
        data = list(data)
        data.extend([0]*3)
        order = [25, 23, 19, 15, 11, 7, 3, 26, 22, 18, 14, 10, 6, 2, 27, 21, 17, 13, 9, 5, 1, 24, 20, 16, 12, 8, 4, 0]
        sorted_data = np.asarray([data[i] for i in order])
        return np.reshape(self.norm_tactile(sorted_data), self.taxels[sensor_num])

def callback(msg, sdv):
    if sdv.sensor_num in sdv.finger_small:
        tactile_data = sdv.transform_finger_small(msg.data)
    elif sensor_num in sdv.finger_big:
        tactile_data = sdv.transform_finger_big(msg.data)
    elif sensor_num in sdv.fingertip:
        tactile_data = sdv.transform_fingertip(msg.data)
    elif sensor_num in sdv.palm:
        tactile_data = sdv.transform_palm(msg.data)
    elif sensor_num in sdv.boh:
        tactile_data = sdv.transform_boh(msg.data)
    else:
        print('issue with sensor number... check it out.')
    # print(tactile_data)
    im.set_array(tactile_data)
    fig.canvas.draw()

def listener(sdv):
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("sensor_" + str(sensor_num) + "_readings", TactileData, callback, sdv)
    # rospy.spin()

if __name__ == '__main__':
    # ask for user input about what sensor number you want to visualize
    sensor_num = int(input("Please enter a sensor number from 8-29: "))
    if (sensor_num > 0) and (sensor_num < 23):
        print("You entered: " + str(sensor_num))
    else:
        sensor_num = int(input("Try it again, friend, something from 8-29: "))

    # initialize a SensorDataVis object
    sdv = SensorDataVis(sensor_num)

    # intialize figure and start the listener node
    fig = plt.figure()
    ax = fig.add_subplot(111)
    im = ax.imshow(np.random.random(sdv.taxels[sdv.sensor_num]), cmap='Oranges')
    listener(sdv)
    plt.show()
    rospy.spin()




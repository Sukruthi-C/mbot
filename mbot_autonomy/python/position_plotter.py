#!/usr/bin/env python
import lcm
import sys
import matplotlib.pyplot as plt
import matplotlib
import time
import math
matplotlib.use('Agg')
sys.path.append('/usr/lib/python3.9/site-packages/')
plot_dir = "/home/mbot/mbot_ws/mbot_autonomy/python/python_plots/"
from mbot_lcm_msgs.pose2D_t import pose2D_t
from mbot_lcm_msgs.path2D_t import path2D_t

class plotter:
    def __init__(self):
        self.start_recording_pose = False

        self.expected_path = {"x":[], "y":[] }
        self.path = {'x': [], 'y': []}
        self.final_location = None
        self.lc = lcm.LCM()
        subscription = self.lc.subscribe("MBOT_ODOMETRY", self.get_pose)
        subscription2 = self.lc.subscribe("CONTROLLER_PATH",  self.get_path)

    def plot_data(self):
        fig, ax = plt.subplots()
        ax.plot(self.path['x'], self.path['y'], label='path taken')
        ax.plot(self.expected_path['x'], self.expected_path['y'], label='expected path')
        ax.legend()
        plt.savefig(plot_dir+'path.png')
        print("data saved to "+plot_dir+'path.png')

    def get_pose(self, channel, data):

        if self.start_recording_pose:
            msg = pose2D_t.decode(data)
            self.path['x'].append(msg.x)
            self.path['y'].append(msg.y)
            if math.sqrt(abs((msg.x-self.final_location[0])**2+(msg.y-self.final_location[1]**2)))<0.05:
                self.start_recording_pose = False
                self.plot_data()
                time.sleep(1000)
        
    def get_path(self, channel, data):
        msg = path2D_t.decode(data)
        for i in msg.path:
            print(i.x, i.y)
            self.expected_path["x"].append(i.x)
            self.expected_path["y"].append(i.y)
        self.final_location = (self.expected_path["x"][-1], self.expected_path["y"][-1])
        
        self.start_recording_pose = True
        
p = plotter()
try:
    while True:
        p.lc.handle()
except KeyboardInterrupt:
    pass

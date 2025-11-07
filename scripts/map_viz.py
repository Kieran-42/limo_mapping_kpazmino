#!/usr/bin/env python3
import numpy as np, matplotlib.pyplot as plt, rospy
from nav_msgs.msg import OccupancyGrid

class MapViewer:
    def __init__(self):
        rospy.Subscriber("/map", OccupancyGrid, self.cb, queue_size=1)
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.im = None

    def cb(self, msg):
        w, h = msg.info.width, msg.info.height
        data = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        img = np.full((h, w), 0.5, dtype=np.float32)   # unknown
        img[data == 0]   = 1.0                         # free
        img[data == 100] = 0.0                         # occupied
        if self.im is None:
            self.im = self.ax.imshow(img, origin='lower', interpolation='nearest')
            self.ax.set_title(f"/map  res={msg.info.resolution:.3f} m/px")
        else:
            self.im.set_data(img)
        self.fig.canvas.draw(); self.fig.canvas.flush_events()

if __name__ == "__main__":
    rospy.init_node("map_viz")
    mv = MapViewer()
    r = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            plt.pause(0.001); r.sleep()
    except rospy.ROSInterruptException:
        pass

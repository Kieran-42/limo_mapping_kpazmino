#!/usr/bin/env python3
import os, sys, signal, subprocess, threading, termios, tty, select
import rospy
from geometry_msgs.msg import Twist

class ExploreAndMap:
    def __init__(self):
        self.pub = rospy.Publisher(rospy.get_param("~cmd_vel_topic", "/cmd_vel"), Twist, queue_size=10)
        self.running = True
        rospy.loginfo("Ready. Keys: [p]=pattern  [s]=save map  [q]=quit")
        threading.Thread(target=self._key_loop, daemon=True).start()

    def move(self, x=0.0, z=0.0, t=2.0, rate_hz=10):
        rate = rospy.Rate(rate_hz)
        cmd = Twist(); cmd.linear.x = x; cmd.angular.z = z
        t_end = rospy.Time.now() + rospy.Duration.from_sec(t)
        while not rospy.is_shutdown() and rospy.Time.now() < t_end:
            self.pub.publish(cmd); rate.sleep()
        self.pub.publish(Twist())

    def demo_pattern(self):
        rospy.loginfo("Running demo pattern…")
        for _ in range(6):
            self.move(x=0.25, z=0.0, t=8.0)
            self.move(x=0.0,   z=1.2, t=2.0)
        rospy.loginfo("Pattern complete.")

    def save_map(self, prefix="~/maps/limo_map"):
        prefix = os.path.expanduser(prefix)
        os.makedirs(os.path.dirname(prefix), exist_ok=True)
        rospy.loginfo("Saving map to %s.[pgm|png]+yaml …", prefix)
        try:
            subprocess.check_call(["rosrun", "map_server", "map_saver", "-f", prefix])
            rospy.loginfo("Map saved.")
        except subprocess.CalledProcessError as e:
            rospy.logerr("map_saver failed: %s", e)

    def _key_loop(self):
        fd = sys.stdin.fileno(); old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self.running and not rospy.is_shutdown():
                if select.select([fd], [], [], 0)[0]:
                    ch = sys.stdin.read(1)
                    if ch == 'q':
                        rospy.signal_shutdown("user quit"); self.running = False
                    elif ch == 'p':
                        self.demo_pattern()
                    elif ch == 's':
                        self.save_map()
                rospy.sleep(0.05)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

def _sigint(sig, frame): rospy.signal_shutdown("SIGINT")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, _sigint)
    rospy.init_node("limo_explore_and_map")
    ExploreAndMap()
    rospy.spin()

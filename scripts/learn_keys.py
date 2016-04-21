#!/usr/bin/env python

import baxter_interface
from baxter_interface import CHECK_VERSION
import rospy, pdb, os, json, time, threading

from artist_performer import Learner

class Shell(threading.Thread):
    def __init__(self,learner):
        threading.Thread.__init__(self)
        self.learner = learner

    def run(self):
        try:
            inp = raw_input("$ ").split(" ")
            while inp[0] != "exit":
                if inp[0] == "left":
                    self.learner.left_note = int(inp[1])
                    self.learner.keys.save_left(self.learner.left_note,self.learner.get_joint_angles())
                    self.learner.send_note(self.learner.left_note)
                elif inp[0] == "right":
                    self.learner.right_note = int(inp[1])
                    self.learner.keys.save_right(self.learner.right_note,self.learner.get_joint_angles())
                    self.learner.send_note(self.learner.right_note)
                elif inp[0] == "neutral":
                    self.learner.keys.save_neutral("left",self.learner.get_joint_angles())
                    self.learner.keys.save_neutral("right",self.learner.get_joint_angles())

                inp = raw_input("$ ").split(" ")
        except EOFError,e:
            pass

def main():
    rospy.loginfo("Initializing node... ")
    rospy.init_node("play_xylophone",log_level=rospy.DEBUG)
    learner = Learner()

    shell = Shell(learner)

    shell.start()
    shell.join()
    
    rospy.signal_shutdown("Finished learning control")
    rospy.loginfo("Done with the learning.")

if __name__ == '__main__':
    main()

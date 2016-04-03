#!/usr/bin/env python

import baxter_interface
from baxter_interface import CHECK_VERSION
import baxter_external_devices
import rospy
import pdb, time, json, copy
import cv2, cv_bridge
import os

from sensor_msgs.msg import (
    Image,
)

with open("./src/baxter_artist/scripts/conf.json") as f:
    CONFIG = json.load(f)

notes = {"B5":"63",
        "C6":"64",
        "D6":"66",
        "E6":"68",
        "F6":"69",
        "G6":"71",
        "A6":"73",
        "B6":"75",
        "C7":"76"}

class Performer(object):
    def __init__(self):
        """
        Performs the instrument like a boss
        """
        self.done = False

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

        # Get the body running
        self.head = baxter_interface.Head()
        self.right_arm = baxter_interface.Limb('right')
        self.left_arm = baxter_interface.Limb('left')
        self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self.right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)

    def clean_shutdown(self):
        """
        Exits example cleanly by moving head to neutral position and
        maintaining start state
        """
        print("\nExiting example...")
        if self.done:
            self.set_neutral()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

    def set_neutral(self):
        self.left_arm.move_to_joint_positions(CONFIG["neutral"]["left"])
        self.right_arm.move_to_joint_positions(CONFIG["neutral"]["right"])

    def flick(self,arm,current_pos):
        down_pos = copy.deepcopy(current_pos)
        down_pos[arm + "_w1"] = current_pos[arm + "_w1"] + 0.08

        arm_obj = (self.left_arm if arm == "left" else self.right_arm)

        arm_obj.set_joint_positions(down_pos)
        time.sleep(0.15)
        arm_obj.set_joint_positions(current_pos)

    def get_joint_angles(self):
        return self.left_arm.joint_angles(), self.right_arm.joint_angles()

    def send_note(self, note):
        path = os.path.join("./src/baxter_artist/scripts/display/img/", str(note) + ".png")

        if not os.path.exists(path):
            rospy.logerr("Not Found: " + path)
            return False

        rospy.logdebug("[send_note] %s", path)

        img = cv2.imread(path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        pub.publish(msg)

    def perform(self, KEYS):
        self.set_neutral()

        inp = raw_input("$ ").split(" ")

        while inp != "exit":

            if inp[0] == "ls":
                print KEYS[inp[1]]

            elif inp[0] == "move":

                self.left_arm.set_joint_position_speed(0.9)
                self.right_arm.set_joint_position_speed(0.9)

                try:
                    start_time = time.time()

                    if inp[1] == "left":
                        self.left_arm.set_joint_positions(KEYS[inp[1]][inp[2]])
                    elif inp[1] == "right":
                        self.right_arm.set_joint_positions(KEYS[inp[1]][inp[2]], raw=True)
                    else:
                        self.set_neutral()

                    delta = time.time() - start_time 
                    print "time", delta

                except KeyError, e:
                    print "KeyError", e

            elif inp[0] == "flick":

                try:
                    start_time = time.time()

                    if inp[1] == "left":
                        self.flick(inp[1],self.get_joint_angles()[0])
                    elif inp[1] == "right":
                        self.flick(inp[1],self.get_joint_angles()[1])

                    delta = time.time() - start_time 
                    print "time", delta

                except KeyError, e:
                    print "KeyError", e
                
            elif inp[0] == "exit":
                break

            elif inp[0] == 'jasonspecial':
                noteNameArray = ["C6", "E6", "G6", "C7", "D6", "B6", "C7", "C6", "C6", "C6", "E6", "G6", "C7", "D6", "B6", "C7", "C6", "C6", "A6", "G6", "F6", "E6", "D6", "G6", "F6", "E6", "D6", "C6", "B5", "C6", "D6", "B5", "D6"]
                noteValArray = [notes[x] for x in noteNameArray]
                print noteValArray

                for num in noteValArray:
                    self.send_note(int(num))
                    self.right_arm.set_joint_positions(KEYS["right"][str(num)], raw=True)
                    time.sleep(0.5)
                    self.flick("right", self.get_joint_angles()[1])
                    time.sleep(0.5)
                    

            inp = raw_input("$ ").split(" ")


        self.done = True



def main():
    print("Initializing node... ")
    rospy.init_node("play_xylophone")
    performer = Performer()
    rospy.on_shutdown(performer.clean_shutdown)

    print("Performing!...")
    with open("./src/baxter_artist/scripts/keys.json") as f:
        KEYS = json.load(f)

    performer.perform(KEYS)

    rospy.signal_shutdown("Finished perform control")
    print("Done with the performance. A+")



if __name__ == '__main__':
    main()
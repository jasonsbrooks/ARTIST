#!/usr/bin/env python

import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_dataflow import wait_for
import baxter_external_devices
import rospy
import pdb, time, json, copy
import cv2, cv_bridge
import os

from controller import BaxterController

from sensor_msgs.msg import (
    Image,
)

notes = {"B5":"63",
        "C6":"64",
        "D6":"66",
        "E6":"68",
        "F6":"69",
        "G6":"71",
        "A6":"73",
        "B6":"75",
        "C7":"76"}

class Performer(BaxterController):
    def __init__(self):
        """
        Performs the instrument like a boss
        """
        BaxterController.__init__(self)

    def flick(self,arm,current_pos):
        down_pos = copy.deepcopy(current_pos)
        down_pos[arm + "_w1"] = current_pos[arm + "_w1"] + 0.08

        arm_obj = (self.left_arm if arm == "left" else self.right_arm)

        wait_for(self.constructJointChecker(down_pos), rate=4, timeout=2.0, body=arm_obj.set_joint_positions(down_pos))
        wait_for(self.constructJointChecker(current_pos), rate=4, timeout=2.0, body=arm_obj.set_joint_positions(current_pos))


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

            elif inp[0] == 'special':
                noteDuration = 1.5
                noteNameArray = ["C6", "E6", "G6", "C7", "D6", "B6", "C7", "C6", "C6", "C6", "E6", "G6", "C7", "D6", "B6", "C7", "C6", "C6", "A6", "G6", "F6", "E6", "D6", "G6", "F6", "E6", "D6", "C6", "B5", "C6", "D6", "B5", "D6"]
                # noteNameArray = ["C7", "B5", "C7","B5", "C7","B5", "C7","B5", "C7"]
                # noteNameArray = ["C7", "E6"]
                noteValArray = [notes[x] for x in noteNameArray]
                print noteValArray
                for num in noteValArray:
                    start_time = time.time()
                    self.send_note(int(num))
                    wait_for(self.constructJointChecker(KEYS["right"][str(num)]), rate=4, raise_on_error=False, timeout=2.0, body=self.right_arm.set_joint_positions(KEYS["right"][str(num)], raw=True))
                    wait_for(self.constructJointChecker(KEYS["right"][str(num)]), rate=4, raise_on_error=False, timeout=2.0, body=self.flick("right", KEYS["right"][str(num)]))
                    delta = time.time() - start_time
                    if noteDuration - delta > 0:
                        time.sleep(noteDuration-delta)
                    print time.time() - start_time
                print "FINSIHED"


            inp = raw_input("$ ").split(" ")

    def checkRightJointPositions(self, target_pos):
        # print "CHECKING"
        current_joint_positions = self.right_arm.joint_angles()
        for key in target_pos:
            difference = abs(target_pos[key] - current_joint_positions[key])
            # print "checked %s with difference %f" %(key, difference)
            if difference > 0.01:
                # print "%s IS TOO FAR OFF!" %(key)
                # print "returning false"
                return False
        # print "returning true"
        return True

    def constructJointChecker(self, target_pos):
        return (lambda: self.checkRightJointPositions(target_pos))

def main():
    print("Initializing node... ")
    rospy.init_node("play_xylophone")
    performer = Performer()

    print("Performing!...")
    with open("./src/baxter_artist/scripts/keys.json") as f:
        KEYS = json.load(f)

    performer.perform(KEYS)

    rospy.signal_shutdown("Finished perform control")
    print("Done with the performance. A+")



if __name__ == '__main__':
    main()
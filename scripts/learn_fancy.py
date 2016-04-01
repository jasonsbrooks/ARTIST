#!/usr/bin/env python

import baxter_interface
from baxter_interface import CHECK_VERSION
import rospy, pdb, os, json

import cv2, cv_bridge

from sensor_msgs.msg import (
    Image,
)

with open("./src/baxter_artist/scripts/conf.json") as f:
    CONFIG = json.load(f)

class Learner(object):
    def __init__(self):
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
        self.right_arm = baxter_interface.Limb('right', CHECK_VERSION)
        self.left_arm = baxter_interface.Limb('left', CHECK_VERSION)
        self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self.right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)
        self.left_navigator = baxter_interface.Navigator('left', CHECK_VERSION)
        self.right_navigator = baxter_interface.Navigator('right', CHECK_VERSION)

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

    def get_joint_angles(self):
        return self.left_arm.joint_angles(), self.right_arm.joint_angles()

    # send the image corresponding to a given note to the display
    def send_note(self, note):
        path = os.path.join("display/img/", str(note) + ".png")
        img = cv2.imread(path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        pub.publish(msg)

def main():
    print("Initializing node... ")
    rospy.init_node("play_xylophone")
    learner = Learner()
    rospy.on_shutdown(learner.clean_shutdown)
    rospy.init_node('rsdk_xdisplay_image', anonymous=True)

    learner.set_neutral()

    print("Learning!...")
    
    if os.path.exists("./src/baxter_artist/scripts/keys.json"):
        with open("./src/baxter_artist/scripts/keys.json") as f:
            data = json.load(f)
            left_arm = data["left"]
            right_arm = data["right"]
    else:
        left_arm = {}
        right_arm = {}

    left_note = 40
    right_note = 40

    def left_wheel_change(change):
        left_note += change
        learner.send_note(left_note)

    # change the left note (and send the image)
    learner.left_navigator.wheel_changed(left_wheel_change)

    def right_wheel_change(change):
        right_note += change
        learner.send_note(right_note)

    # change the right note (and send the image)
    learner.right_navigator.wheel_changed(right_wheel_change)

    # save left joint angles
    def save_left(on=True):
        if not on:
            return False

        pos = learner.get_joint_angles()[0]
        arm = "left"
        left_arm[left_note] = pos
        print arm, left_note, pos 

        return True

    # save right joint angles
    def save_right(on=True):
        if not on:
            return False

        pos = learner.get_joint_angles()[1]
        arm = "right"
        right_arm[right_note] = pos
        print arm, right_note, pos

        return True

    # listen for button presses
    learner.left_navigator.button0_changed(save_left)
    learner.right_navigator.button0_changed(save_right)

    inp = raw_input("$ ").split(" ")
    while inp[0] != "exit":
        if inp[0] == "left":
            save_left()
        elif inp[0] == "right":
            save_right()
        elif inp[0] == "neutral":
            pos = learner.get_joint_angles()
            CONFIG["neutral"]["left"] = pos[0]
            CONFIG["neutral"]["right"] = pos[1]

            with open("./src/baxter_artist/scripts/conf.json", "w") as f:
                json.dump(CONFIG,f)
                print CONFIG

        inp = raw_input("$ ").split(" ")
    
    rospy.signal_shutdown("Finished learning control")
    print("Done with the learning. A+")

    # write keys to file
    with open("./src/baxter_artist/scripts/keys.json", "w") as f:
        data = {"left": left_arm, "right": right_arm}
        json.dump(data, f)

    print left_arm, right_arm

if __name__ == '__main__':
    main()

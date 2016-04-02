#!/usr/bin/env python

import baxter_interface
from baxter_interface import CHECK_VERSION
import rospy, pdb, os, json, time

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
        self.right_arm = baxter_interface.Limb('right')
        self.left_arm = baxter_interface.Limb('left')
        self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self.right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)
        self.left_navigator = baxter_interface.Navigator('left')
        self.right_navigator = baxter_interface.Navigator('right')
        self.left_torso_navigator = baxter_interface.Navigator('torso_left')
        self.right_torso_navigator = baxter_interface.Navigator('torso_right')

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

    def set_neutral(self,on=True):
        if not on:
            return False
        
        self.left_arm.move_to_joint_positions(CONFIG["neutral"]["left"])
        self.right_arm.move_to_joint_positions(CONFIG["neutral"]["right"])

    def get_joint_angles(self):
        return self.left_arm.joint_angles(), self.right_arm.joint_angles()

    # send the image corresponding to a given note to the display
    def send_note(self, note):
        path = os.path.join("./src/baxter_artist/scripts/display/img/", str(note) + ".png")

        if not os.path.exists(path):
            print "Not Found:", path
            return False

        print "Sending:", path

        img = cv2.imread(path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        pub.publish(msg)

NUM_KEYS = 88
NEUTRAL_KEY = 0
MIDDLE_C = 40

left_note = MIDDLE_C
right_note = MIDDLE_C

def main():
    print("Initializing node... ")
    rospy.init_node("play_xylophone")
    learner = Learner()
    rospy.on_shutdown(learner.clean_shutdown)

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

    # change the notes and send images to display
    def left_wheel_change(change):
        global left_note
        left_note += change
        if 0 <= left_note and left_note <= NUM_KEYS:
            learner.send_note(left_note)

    def right_wheel_change(change):
        global right_note
        right_note += change
        if 0 <= right_note and right_note <= NUM_KEYS:
            learner.send_note(right_note)

    # listen for wheel changes
    learner.left_navigator.wheel_changed.connect(left_wheel_change)
    learner.right_navigator.wheel_changed.connect(right_wheel_change)

    # save a new neutral position
    def save_neutral(arm):
        pos = learner.get_joint_angles()
        CONFIG["neutral"][arm] = pos[(0 if arm == "left" else 1)]

        with open("./src/baxter_artist/scripts/conf.json", "w") as f:
            json.dump(CONFIG,f)
            print CONFIG

    global left_note,right_note

    # save left joint angles
    def save_left(on=True):
        if not on:
            return False
        
        learner.left_navigator._outer_led.set_output(True)
        
        if left_note == NEUTRAL_KEY:
            save_neutral("left")

        pos = learner.get_joint_angles()[0]
        left_arm[left_note] = pos
        print "Recording", "left", left_note, pos 

        time.sleep(0.5)
        learner.left_navigator._outer_led.set_output(False)

    # save right joint angles
    def save_right(on=True):
        if not on:
            return False
        
        learner.right_navigator._outer_led.set_output(True)

        if right_note == NEUTRAL_KEY:
            save_neutral("right")

        pos = learner.get_joint_angles()[1]
        right_arm[right_note] = pos
        print "Recording", "right", right_note, pos

        time.sleep(0.5)
        learner.right_navigator._outer_led.set_output(False)

    # listen for button presses
    learner.left_navigator.button0_changed.connect(save_left)
    learner.right_navigator.button0_changed.connect(save_right)

    # torso navigators move to neutral position
    learner.left_torso_navigator.button0_changed.connect(learner.set_neutral)
    learner.right_torso_navigator.button0_changed.connect(learner.set_neutral)

    learner.left_navigator._inner_led.set_output(True)
    learner.left_torso_navigator._inner_led.set_output(True)
    learner.right_navigator._inner_led.set_output(True)
    learner.right_torso_navigator._inner_led.set_output(True)
    learner.left_navigator._outer_led.set_output(False)
    learner.right_navigator._outer_led.set_output(False)


    # send MIDDLE_C
    learner.send_note(NEUTRAL_KEY)

    inp = raw_input("$ ").split(" ")
    while inp[0] != "exit":
        if inp[0] == "left":
            left_note = int(inp[1])
            save_left()
            learner.send_note(left_note)
        elif inp[0] == "right":
            right_note = int(inp[1])
            save_right()
            learner.send_note(right_note)
        elif inp[0] == "neutral":
            save_neutral("left")
            save_neutral("right")

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

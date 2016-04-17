#!/usr/bin/env python

import baxter_interface
from baxter_interface import CHECK_VERSION
import rospy, pdb, os, json, time, threading

from baxter_artist import BaxterController

import cv2, cv_bridge

from sensor_msgs.msg import (
    Image,
)

NUM_KEYS = 88
NEUTRAL_KEY = 0

with open("./src/baxter_artist/scripts/conf.json") as f:
    CONFIG = json.load(f)

class Learner(BaxterController):
    def __init__(self):

        BaxterController.__init__(self)

        # initialize right_note, left_note
        self.left_note = NEUTRAL_KEY
        self.right_note = NEUTRAL_KEY

        self.keys = Keys()

        # listen for wheel changes
        self.left_navigator.wheel_changed.connect(self.left_wheel_change)
        self.right_navigator.wheel_changed.connect(self.right_wheel_change)
        
        # listen for button presses
        self.left_navigator.button0_changed.connect(self.left_button_press)
        self.right_navigator.button0_changed.connect(self.right_button_press)

        # torso navigators move to neutral position
        self.left_torso_navigator.button0_changed.connect(self.set_neutral)
        self.right_torso_navigator.button0_changed.connect(self.set_neutral)

        self.set_neutral()
        self.init_leds()
        self.send_note(NEUTRAL_KEY)

        self.shell = Shell()
   
    def init_leds(self):
        rospy.logdebug("init_leds")
        self.left_navigator._inner_led.set_output(True)
        self.left_torso_navigator._inner_led.set_output(True)
        self.right_navigator._inner_led.set_output(True)
        self.right_torso_navigator._inner_led.set_output(True)
        self.left_navigator._outer_led.set_output(False)
        self.right_navigator._outer_led.set_output(False)

    def clean_shutdown(self):

        self.keys.write()
        BaxterController.clean_shutdown(self)

    def get_joint_angles(self):
        return self.left_arm.joint_angles(), self.right_arm.joint_angles()

    # send the image corresponding to a given note to the display
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

    # change the notes and send images to display
    def left_wheel_change(self,change):
        self.left_note += change
        if 0 <= self.left_note and self.left_note <= NUM_KEYS:
            self.send_note(self.left_note)

        rospy.logdebug("[left_wheel_change] left_note = %d",self.left_note)

    def right_wheel_change(self,change):
        self.right_note += change
        if 0 <= self.right_note and self.right_note <= NUM_KEYS:
            self.send_note(self.right_note)

        rospy.logdebug("[right_wheel_change] right_note = %d",self.right_note)

    def left_button_press(self,on=True):
        if not on:
            return False
        
        rospy.logdebug("[left_button_press]")

        self.left_navigator._outer_led.set_output(True)

        self.keys.save_left(self.left_note,self.get_joint_angles())

        time.sleep(0.5)
        self.left_navigator._outer_led.set_output(False)

    def right_button_press(self,on=True):
        if not on:
            return False

        rospy.logdebug("[right_button_press]")
        
        self.right_navigator._outer_led.set_output(True)

        self.keys.save_right(self.right_note,self.get_joint_angles())

        time.sleep(0.5)
        self.right_navigator._outer_led.set_output(False)

class Keys(object):
    def __init__(self):
        if os.path.exists("./src/baxter_artist/scripts/keys.json"):
            with open("./src/baxter_artist/scripts/keys.json") as f:
                data = json.load(f)

                self.left_arm = data["left"]
                self.right_arm = data["right"]

                rospy.logdebug("Loaded: %s", data)
        else:
            self.left_arm = {}
            self.right_arm = {}

    def save_left(self,note,angles):
        if note <= NEUTRAL_KEY:
            self.save_neutral("left",angles)
        elif note > NUM_KEYS:
            note = NUM_KEYS

        pos = angles[0]
        self.left_arm[note] = pos
        rospy.loginfo("[save_left] %d %s", note, pos) 

    def save_right(self,note,angles):
        if note <= NEUTRAL_KEY:
            self.save_neutral("right",angles)
        elif note > NUM_KEYS:
            note = NUM_KEYS

        pos = angles[1]
        self.right_arm[note] = pos
        rospy.loginfo("[save_right] %d %s", note, pos) 

    # save a new neutral position
    def save_neutral(self,arm,angles):
        CONFIG["neutral"][arm] = angles[(0 if arm == "left" else 1)]

        with open("./src/baxter_artist/scripts/conf.json", "w") as f:
            json.dump(CONFIG,f)
            rospy.loginfo("[save_neutral] %s", CONFIG)

    def write(self):
        # write keys to file
        with open("./src/baxter_artist/scripts/keys.json", "w") as f:
            data = {"left": self.left_arm, "right": self.right_arm}
            json.dump(data, f)
            rospy.loginfo("[write] %s", data)

class Shell(threading.Thread):
    def run(self):
        try:
            inp = raw_input("$ ").split(" ")
            while inp[0] != "exit":
                if inp[0] == "left":
                    self.left_note = int(inp[1])
                    self.keys.save_left(self.left_note,self.get_joint_angles())
                    self.send_note(self.left_note)
                elif inp[0] == "right":
                    self.right_note = int(inp[1])
                    self.keys.save_right(self.right_note,self.get_joint_angles())
                    self.send_note(self.right_note)
                elif inp[0] == "neutral":
                    self.keys.save_neutral("left",self.get_joint_angles())
                    self.keys.save_neutral("right",self.get_joint_angles())

                inp = raw_input("$ ").split(" ")
        except EOFError,e:
            pass

def main():
    rospy.loginfo("Initializing node... ")
    rospy.init_node("play_xylophone",log_level=rospy.DEBUG)
    learner = Learner()

    learner.shell.start()
    learner.shell.join()
    
    rospy.signal_shutdown("Finished learning control")
    rospy.loginfo("Done with the learning.")

if __name__ == '__main__':
    main()

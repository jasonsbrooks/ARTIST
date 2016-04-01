#!/usr/bin/env python

import baxter_interface
from baxter_interface import CHECK_VERSION
import rospy, pdb, os, json

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

    def close_gripper(self,gripper):
        print "open close gripper"
        gripper.calibrate()
        gripper.open()
        gripper.close() 
        
    def grab_mallet(self,gripper):
        gripper.calibrate()

        inp = raw_input("$ ")
        while inp != "cont":
            print "Opening / Closing ", gripper
            gripper.open()
            gripper.close()

            inp = raw_input("$ ")

        print "Successfully grabbed mallet."

    def get_joint_angles(self):
        return self.left_arm.joint_angles(), self.right_arm.joint_angles()

def main():
    print("Initializing node... ")
    rospy.init_node("play_xylophone")
    learner = Learner()
    rospy.on_shutdown(learner.clean_shutdown)

    learner.set_neutral()

    print("Learning!...")
    
    # learner.grab_mallet(learner.right_gripper)
    # learner.grab_mallet(learner.left_gripper)

    right_arm = {}
    left_arm = {}

    inp = raw_input("$ ").split(" ")
    while inp[0] != "exit":
        key = inp[1]

        if inp[0] == "left":
            pos = learner.get_joint_angles()[0]
            arm = "left"
            left_arm[key] = pos
            print arm, key, pos  

        elif inp[0] == "right":
            pos = learner.get_joint_angles()[1]
            arm = "right"
            right_arm[key] = pos
            print arm, key, pos

        inp = raw_input("$ ").split(" ")
    
    rospy.signal_shutdown("Finished learning control")
    print("Done with the learning. A+")

    with open("./src/baxter_artist/scripts/keys.json", "w") as f:
        data = {"left": left_arm, "right": right_arm}
        json.dump(data, f)

    print left_arm, right_arm



if __name__ == '__main__':
    main()

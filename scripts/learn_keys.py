import baxter_interface
from baxter_interface import CHECK_VERSION
import baxter_external_devices
import rospy
import pdb, json

with open('conf.json') as f:
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
            self._set_neutral()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

    def _set_neutral(self):
        self.left_arm.move_to_joint_positions(config["neutral"]["left"])
        self.right_arm.move_to_joint_positions(config["neutral"]["right"])

    def close_gripper(gripper):
        print "open close gripper"
        gripper.calibrate()
        gripper.open()
        gripper.close() 
        
    def grab_mallet(gripper):
        gripper.calibrate()

        inp = input("$ ")
        while inp != "cont":
            print "Opening / Closing " + gripper
            gripper.open()
            gripper.close()

            inp = input("$ ")

        print "Successfully grabbed mallet."

    def get_joint_angles(self):
        return self.left_arm.joint_angles(), self.right_arm.joint_angles()

def main():
    print("Initializing node... ")
    rospy.init_node("play_xylophone")
    learner = Learner()
    rospy.on_shutdown(learner.clean_shutdown)

    print("Learning!...")
    
    learner.grab_mallet(learner.right_gripper)
    learner.grab_mallet(learner.left_gripper)

    inp = input("$ ")
    while inp != "done":
        if inp == "left":
            print "left", learner.get_joint_angles()[0]
        elif inp == "right":
            print "right", learner.get_joint_angles()[1]

        inp = input("$ ")
    
    rospy.signal_shutdown("Finished learning control")
    print("Done with the learning. A+")



if __name__ == '__main__':
    main()

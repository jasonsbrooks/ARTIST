import baxter_interface
from baxter_interface import CHECK_VERSION
import baxter_external_devices
import rospy
import pdb

RIGHT_NEUTRAL = {'right_s0': 0.8943107983154297, 'right_s1': 0.09664078952636719, 'right_w0': -3.048786812438965, 'right_w1': 1.6693545905090332, 'right_w2': -0.0851359336303711, 'right_e0': 0.0851359336303711, 'right_e1': 1.51212155993042}
LEFT_NEUTRAL = {'left_w0': -0.12655341485595703, 'left_w1': -1.5696458394104005, 'left_w2': -3.0591411827453614, 'left_e0': -0.002300971179199219, 'left_e1': 1.5243934062194826, 'left_s0': -0.966407895263672, 'left_s1': 0.04716990917358399}

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

    def perform(self):
        self.set_neutral()
        self.done = True

    def set_neutral(self):
        self.left_arm.move_to_joint_positions(LEFT_NEUTRAL)
        self.right_arm.move_to_joint_positions(RIGHT_NEUTRAL)

    def get_joint_angles(self):
        print "right", self.right_arm.joint_angles()
        print "left", self.left_arm.joint_angles()

    def close_right_gripper(self):
        print "open close right gripper"
        self.right_gripper.calibrate()
        self.right_gripper.open()
        self.right_gripper.close()


def main():
    print("Initializing node... ")
    rospy.init_node("play_xylophone")
    performer = Performer()
    rospy.on_shutdown(performer.clean_shutdown)
    print("Performing!...")
    performer.perform()
    performer.get_joint_angles()
    performer.close_right_gripper()
    rospy.signal_shutdown("Finished perform control")
    print("Done with the performance. A+")



if __name__ == '__main__':
    main()
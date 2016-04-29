#!/usr/bin/env python

import rospy, baxter_interface, json
from baxter_interface import CHECK_VERSION
from . import CONFIG_FILENAME

class BaxterController(object):
    """
    Superclass for controlling Baxter. Initializes various joints, manages shutdown and set_neutral.
    """
    def __init__(self):
        """
        Initialize control with Baxter
        """
        # verify robot is enabled
        rospy.loginfo("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        rospy.loginfo("Enabling robot... ")
        self._rs.enable()
        rospy.on_shutdown(self.clean_shutdown)
        rospy.loginfo("Running...")

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
        Exits example cleanly by disabling robot
        """
        rospy.loginfo("Exiting example...")

        if not self._init_state and self._rs.state().enabled:
            rospy.loginfo("Disabling robot...")
            self._rs.disable()

    def set_neutral(self,on=True):
        """
        Move both arms to the neutral position

        Args:
            on: whether to perform the action (useful in button callbacks)
        """
        if not on:
            return False

        rospy.loginfo("[set_neutral] Moving arms to neutral position.")

        # load the config file
    	with open(CONFIG_FILENAME) as f:
		    CONFIG = json.load(f)

        rospy.loginfo("[set_neutral] CONFIG: %s",CONFIG)

        # and move to neutral position
        self.left_arm.move_to_joint_positions(CONFIG["neutral"]["left"])
        self.right_arm.move_to_joint_positions(CONFIG["neutral"]["right"])

        rospy.loginfo("[set_neutral] Both arms in neutral position.")

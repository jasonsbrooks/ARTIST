import copy, os, rospy, cv2, cv_bridge, json, time

from baxter_dataflow import wait_for

from sensor_msgs.msg import (
    Image,
)

import baxter_artist.msg
from . import BaxterController, KEYS_FILENAME, IMAGE_PATH

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
        path = os.path.join(IMAGE_PATH, "notes/", str(note) + ".png")

        if not os.path.exists(path):
            rospy.logerr("Not Found: " + path)
            return False

        rospy.logdebug("[send_note] %s", path)

        img = cv2.imread(path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        pub.publish(msg)

    def play_note_now(self,note):
        noteDuration = 1.5
        start_time = time.time()
        self.send_note(int(num))
        wait_for(self.constructJointChecker(KEYS["right"][str(num)]), rate=4, raise_on_error=False, timeout=2.0, body=self.right_arm.set_joint_positions(KEYS["right"][str(num)], raw=True))
        wait_for(self.constructJointChecker(KEYS["right"][str(num)]), rate=4, raise_on_error=False, timeout=2.0, body=self.flick("right", KEYS["right"][str(num)]))
        delta = time.time() - start_time
        if noteDuration - delta > 0:
            time.sleep(noteDuration-delta)
        print time.time() - start_time

    def on_receive_note(self,data):
        rospy.loginfo("receive - pitch:" + str(data.pitch))

    def subscribe(self):
        rospy.Subscriber("baxter_artist_notes", baxter_artist.msg.Note, self.on_receive_note)
        rospy.spin()

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

import copy, os, rospy, cv2, cv_bridge, json, time, threading
from Queue import Queue

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

class NoteSubscriber(threading.Thread):
    def __init__(self,q):
        threading.Thread.__init__(self)
        self.q = q

    def on_receive_note(self,data):
        rospy.loginfo("receive - pitch: " + str(data.pitch))
        self.q.put(data)

    def run(self):
        rospy.Subscriber("baxter_artist_notes", baxter_artist.msg.Note, self.on_receive_note)
        rospy.spin()

class Performer(BaxterController):
    def __init__(self):
        """
        Performs the instrument like a boss
        """
        BaxterController.__init__(self)
        self.set_neutral()
        self.q = Queue()

        with open(KEYS_FILENAME) as f:
            self.keys = json.load(f)

        self.right_notes = [int(x) for x in keys['right'].keys()]
        self.left_notes = [int(x) for x in keys['left'].keys()]

    def flick(self,arm,current_pos):
        down_pos = copy.deepcopy(current_pos)
        down_pos[arm + "_w1"] = current_pos[arm + "_w1"] + 0.08

        arm_obj = (self.left_arm if arm == "left" else self.right_arm)

        wait_for(lambda: self.checkJointPositions(down_pos, arm), rate=4, timeout=2.0, body=arm_obj.set_joint_positions(down_pos))
        wait_for(lambda: self.checkJointPositions(current_pos, arm), rate=4, timeout=2.0, body=arm_obj.set_joint_positions(current_pos))


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

    def play_right_note_now(self,note):
        self.send_note(int(note))
        wait_for(lambda: self.checkJointPositions(self.keys["right"][str(note)], "right"), rate=4, raise_on_error=False, timeout=2.0, body=self.right_arm.set_joint_positions(self.keys["right"][str(note)], raw=True))
        wait_for(lambda: self.checkJointPositions(self.keys["right"][str(note)], "right"), rate=4, raise_on_error=False, timeout=2.0, body=self.flick("right", self.keys["right"][str(note)]))

    def play_left_note_now(self, note):
        self.send_note(int(note))
        wait_for(lambda: self.checkJointPositions(self.keys["left"][str(note)], "left"), rate=4, raise_on_error=False, timeout=2.0, body=self.left_arm.set_joint_positions(self.keys["left"][str(note)], raw=True))
        wait_for(lambda: self.checkJointPositions(self.keys["left"][str(note)], "left"), rate=4, raise_on_error=False, timeout=2.0, body=self.flick("left", self.keys["left"][str(note)]))


    def play_note(self,note):
        sleep_time = note.starttime - rospy.Time.now()

        # we've missed the opportunity to play this note
        if sleep_time.to_nsec() < 0:
            return

        # sleep until we can play it
        rospy.sleep(sleep_time)

        rospy.loginfo("playing: " + str(note.pitch))
        if note.pitch in self.right_notes:
            self.play_right_note_now(note.pitch)
        elif note.pitch in self.left_notes:
            self.play_left_note_now(note.pitch)
        else:
            rospy.loginfo("Note with no corresponding key tried to play. Check yo_self.")

    def subscribe(self):
        subscriber = NoteSubscriber(self.q)
        subscriber.daemon = True
        subscriber.start()

        while True:
            note = self.q.get()
            rospy.loginfo("dequeue: " + str(note.pitch))
            self.play_note(note)
            self.q.task_done()

        subscriber.join()

    def checkJointPositions(self, target_pos, arm):
        current_joint_positions = self.left_arm.joint_angles()

        # update if curious about the other arm
        if arm == 'right':
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
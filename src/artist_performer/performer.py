import copy, os, rospy, cv2, cv_bridge, json, time, threading, signal, sys
from Queue import Queue

from baxter_dataflow import wait_for

from sensor_msgs.msg import (
    Image,
)

import baxter_artist.msg
from . import BaxterController, KEYS_FILENAME, IMAGE_PATH

class NoteSubscriber(threading.Thread):
    """
    Subscribe to the baxter_artist_notes rostopic, listening for new notes to play.

    Add them to the queue when received.
    """
    def __init__(self,q):
        """
        Initialize the NoteSubscriber
        Args:
            q: the queue of notes. add here when receiving a new note
        """
        threading.Thread.__init__(self)
        self.q = q

    def on_receive_note(self,data):
        """
        Callback for when a note message is received. Add the note to the queue of notes to play.

        Args:
            data (baxter_artist.msg.Note): the message received (a note)
        """
        rospy.loginfo("receive - pitch: " + str(data.pitch))
        self.q.put(data)

    def run(self):
        """
        Start the thread, subscribing the the baxter_artist_notes topic.
        """
        rospy.Subscriber("baxter_artist_notes", baxter_artist.msg.Note, self.on_receive_note)
        rospy.spin()

class Performer(BaxterController):
    """
    Manage the performance.
    """
    def __init__(self):
        """
        Performs the instrument like a boss
        """
        BaxterController.__init__(self)
        self.q = Queue()
        self.prev_left_note = 52
        self.prev_right_note = 61

        with open(KEYS_FILENAME) as f:
            self.keys = json.load(f)

        self.right_notes = [int(x) for x in self.keys['right'].keys()]
        self.left_notes = [int(x) for x in self.keys['left'].keys()]

    def flick(self,arm,current_pos):
        """
        Flick the arm down to hit a specific key.

        Args:
            arm (str): the arm to flick
            current_pos: the current joint anagles of the arm. return here when done
        """
        down_pos = copy.deepcopy(current_pos)
        down_pos[arm + "_w1"] = current_pos[arm + "_w1"] + 0.8

        arm_obj = (self.left_arm if arm == "left" else self.right_arm)

        arm_obj.set_joint_positions(down_pos)
        time.sleep(0.15)
        arm_obj.set_joint_positions(current_pos)

        # wait_for(lambda: self.checkJointPositions(down_pos, arm), rate=4, timeout=2.0, raise_on_error=False, body=arm_obj.set_joint_positions(down_pos))
        # wait_for(lambda: self.checkJointPositions(current_pos, arm), rate=4, timeout=2.0, raise_on_error=False, body=arm_obj.set_joint_positions(current_pos))

    def avoid_black_key(self, arm, current_pos):
        """
        Avoid the black keys when moving.

        Args:
            arm (str): the arm moving.
            current_pos: current position of the arm.
        """
        inter_pos = copy.deepcopy(current_pos)
        inter_pos[arm + "_s1"] = current_pos[arm + "_s1"] - 1

        arm_obj = (self.left_arm if arm == "left" else self.right_arm)

        arm_obj.set_joint_positions(inter_pos)

    def get_joint_angles(self):
        """
        Get the joint angles of the left / right arm as a tuple.

        Returns (tuple): the arm joint angles.
        """
        return self.left_arm.joint_angles(), self.right_arm.joint_angles()

    def send_note(self, note):
        """
        Send the image corresponding to a given note to the display

        Args:
            note (int): number of the note to send
        """
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
        """
        Play a note with the right arm immediately (no waiting)

        Args:
            note (int): the note to play
        """
        self.send_note(int(note))

        total_sleep = 0.5
        if int(note) in [74, 72, 70, 67, 65, 62, 60, 58] and self.prev_right_note not in [74, 72, 70, 67, 65, 62, 60, 58]:
            self.avoid_black_key("right", self.right_arm.joint_angles())
            time.sleep(0.25)
            total_sleep -= 0.25

        self.right_arm.set_joint_positions(self.keys["right"][str(note)], raw=True)
        if abs(int(note) - int(self.prev_right_note)) > 5:
            rospy.loginfo("Making two motions since notes are too far apart")
            time.sleep(0.25)
            total_sleep -= 0.25
            self.right_arm.set_joint_positions(self.keys["right"][str(note)], raw=True)

        # time.sleep(1)
        # self.right_arm.set_joint_positions(self.keys["right"][str(note)], raw=True)
        time.sleep(total_sleep)
        self.flick("right", self.keys["right"][str(note)])
        self.prev_right_note = int(note)
        # time.sleep(2)
        # wait_for(lambda: self.checkJointPositions(self.keys["right"][str(note)], "right"), rate=4, raise_on_error=False, timeout=2.0, body=self.right_arm.set_joint_positions(self.keys["right"][str(note)], raw=True))
        # wait_for(lambda: self.checkJointPositions(self.keys["right"][str(note)], "right"), rate=4, raise_on_error=False, timeout=2.0, body=self.flick("right", self.keys["right"][str(note)]))

    def play_left_note_now(self, note):
        """
        Play a note with the right arm immediately (no waiting)

        Args:
            note (int): the note to play
        """
        self.send_note(int(note))

        total_sleep = 0.5
        if int(note) in [48, 50, 53, 55, 58, 60, 62] and self.prev_left_note not in [48, 50, 53, 55, 58, 60, 62]:
            self.avoid_black_key("left", self.left_arm.joint_angles())
            time.sleep(0.25)
            total_sleep -= 0.25

        self.left_arm.set_joint_positions(self.keys["left"][str(note)], raw=True)
        if abs(int(note) - int(self.prev_left_note)) > 5:
            rospy.loginfo("Making two motions since notes are too far apart")
            time.sleep(0.25)
            total_sleep -= 0.25
            self.left_arm.set_joint_positions(self.keys["left"][str(note)], raw=True)

        time.sleep(total_sleep)
        self.flick("left", self.keys["left"][str(note)])
        self.prev_left_note = int(note)
        # wait_for(lambda: self.checkJointPositions(self.keys["left"][str(note)], "left"), rate=4, raise_on_error=False, timeout=2.0, body=self.left_arm.set_joint_positions(self.keys["left"][str(note)], raw=True))
        # wait_for(lambda: self.checkJointPositions(self.keys["left"][str(note)], "left"), rate=4, raise_on_error=False, timeout=2.0, body=self.flick("left", self.keys["left"][str(note)]))


    def play_note(self,note):
        """
        Play a note (arm undetermined as of now). Wait until the note should be played.
        Args:
            note (baxter_artist.msg.Note): the note to play
        """
        sleep_time = note.starttime - rospy.Time.now()

        # # we've missed the opportunity to play this note
        # if sleep_time.to_nsec() < 0:
        #     rospy.loginfo("PASSED A NOTE")
        #     return

        # # sleep until we can play it
        rospy.sleep(sleep_time)

        # rospy.loginfo("playing: " + str(note.pitch))
        if note.pitch in self.right_notes:
            self.play_right_note_now(note.pitch)
        elif note.pitch in self.left_notes:
            self.play_left_note_now(note.pitch)
        else:
            rospy.loginfo("Note with no corresponding key tried to play. Check yo_self.")

    def subscribe(self):
        """
        Subscribe to the baxter_artist_notes rostopic.
        """
        subscriber = NoteSubscriber(self.q)
        subscriber.daemon = True
        subscriber.start()
        rospy.loginfo("[subscribe] NoteSubscriber listening for notes.")
        while True:
            note = self.q.get()
            rospy.loginfo("dequeue: " + str(note.pitch))
            self.play_note(note)
            self.q.task_done()

        subscriber.join()

    def checkJointPositions(self, target_pos, arm):
        """
        Verify that the joint positions are with a certain delta of the target.
        Args:
            target_pos: the target position to hit
            arm (str): which arm

        Returns:
            bool: whether the arm is within the desired range.
        """
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
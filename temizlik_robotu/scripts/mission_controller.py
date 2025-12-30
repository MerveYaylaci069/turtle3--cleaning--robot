#!/usr/bin/env python3
import rospy
import yaml
import tf
import actionlib
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyzbar.pyzbar as pyzbar

class TemizlikRobotu:
    def __init__(self):
        rospy.init_node("mission_controller", log_level=rospy.INFO)
        rospy.loginfo("Mission Controller başlatıldı")

        self.bridge = CvBridge()
        self.qr_data = None

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.goal_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("move_base bekleniyor...")
        self.goal_client.wait_for_server()

        rospy.Subscriber("/camera/rgb/image_raw", Image, self.qr_callback)

        mission_path = rospy.get_param(
            "~mission_file",
            rospy.get_param("/mission_file")
        )

        with open(mission_path, "r") as f:
            self.rooms = yaml.safe_load(f)["rooms"]

        rospy.loginfo(f"{len(self.rooms)} oda yüklendi.")

    def qr_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        decoded = pyzbar.decode(image)
        if decoded:
            self.qr_data = decoded[0].data.decode("utf-8")
            rospy.loginfo(f"QR okundu: {self.qr_data}")

    def go_to(self, goal, timeout=90):
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.frame_id = "map"
        mb_goal.target_pose.header.stamp = rospy.Time.now()
        mb_goal.target_pose.pose.position.x = goal["x"]
        mb_goal.target_pose.pose.position.y = goal["y"]
        q = tf.transformations.quaternion_from_euler(0, 0, goal["yaw"])
        mb_goal.target_pose.pose.orientation.x = q[0]
        mb_goal.target_pose.pose.orientation.y = q[1]
        mb_goal.target_pose.pose.orientation.z = q[2]
        mb_goal.target_pose.pose.orientation.w = q[3]

        self.goal_client.send_goal(mb_goal)
        finished = self.goal_client.wait_for_result(rospy.Duration(timeout))
        return finished and self.goal_client.get_state() == 3

    def small_adjustment(self):
        twist = Twist()
        twist.angular.z = 0.3
        self.cmd_pub.publish(twist)
        rospy.sleep(1.0)
        self.cmd_pub.publish(Twist())

    def run(self):
        report = {}

        for room in self.rooms:
            rospy.loginfo(f"{room['name']} girişine gidiliyor")
            if not self.go_to(room["entry_goal"]):
                report[room["name"]] = "FAIL"
                continue

            self.qr_data = None
            qr_ok = False
            rospy.loginfo("QR doğrulama başlatıldı")

            for _ in range(3):
                rospy.sleep(1.5)
                if self.qr_data == room["qr_expected"]:
                    qr_ok = True
                    break
                self.small_adjustment()

            if not qr_ok:
                rospy.logwarn("QR doğrulama başarısız")
                report[room["name"]] = "SKIPPED"
                continue

            rospy.loginfo("Temizlik başlatıldı")
            for wp in room["cleaning_goals"]:
                self.go_to(wp)

            report[room["name"]] = "SUCCESS"

        rospy.loginfo("=== TEMİZLİK RAPORU ===")
        for r, s in report.items():
            rospy.loginfo(f"{r}: {s}")

if __name__ == "__main__":
    rospy.sleep(2)
    TemizlikRobotu().run()


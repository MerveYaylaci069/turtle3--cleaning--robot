#!/usr/bin/env python3
import rospy
import yaml
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# ROS node başlat
rospy.init_node('waypoint_markers')

# Marker publisher
marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=10)

# mission.yaml dosyasını oku
mission_file = rospy.get_param("~mission_file", "../config/mission.yaml")
with open(mission_file, 'r') as file:
    mission_data = yaml.safe_load(file)

rate = rospy.Rate(1)  # 1 Hz

marker_id = 0

while not rospy.is_shutdown():
    for room in mission_data['rooms']:
        # Oda giriş waypoint'i
        entry = room['entry_goal']
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = entry['x']
        marker.pose.position.y = entry['y']
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3  # ok uzunluğu
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker_pub.publish(marker)
        marker_id += 1

        # Mini temizlik waypointleri
        for wp in room['cleaning_goals']:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = wp['x']
            marker.pose.position.y = wp['y']
            marker.pose.position.z = 0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.07
            marker.scale.z = 0.07
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_pub.publish(marker)
            marker_id += 1

    rate.sleep()


#!/usr/bin/env python

# Author: Eduardo Ferrera <eferrera@catec.aero>


# Python
import numpy as np

# ROS
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion, Pose, Point, Vector3
from geometry_msgs.msg import PolygonStamped
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
import tf2_ros
import tf2_py as tf2 
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
# Other imports


class PedestrianDetector(object):
    """
    Pedestrian detector, capable of checking if there is one person un a specific area
    """

    def __init__(self):

        self.n_measurements_consider_valid = 4
        
        # Publishes the area where the pedestrian pass is
        self.pub_pedestrian_area = rospy.Publisher("/pedestrian_area_reference", PolygonStamped, queue_size=1, latch=True)
        self.pedestrian_center = [42.3,78.9, -0.48]
        self.pedestrian_wide = [6.5,4.0]
        self.publish_pedestrian_area()
        self.pedestrian_area = DDynamicReconfigure("pedestrian_area")
        self.pedestrian_area.add_variable('x_pose','x_pose', 
                                    self.pedestrian_center[0], 
                                    min= self.pedestrian_center[0] - 1, 
                                    max = self.pedestrian_center[0] + 1)
        self.pedestrian_area.add_variable('y_pose','y_pose', 
                                    self.pedestrian_center[1], 
                                    min= self.pedestrian_center[1] - 1, 
                                    max = self.pedestrian_center[1] + 1)
        self.pedestrian_area.add_variable('x_size','x_size', 
                                    self.pedestrian_wide[0], 
                                    min= self.pedestrian_wide[0] - 1, 
                                    max = self.pedestrian_wide[0] + 1)
        self.pedestrian_area.add_variable('y_size','y_size', 
                                    self.pedestrian_wide[1], 
                                    min= self.pedestrian_wide[1] - 1, 
                                    max = self.pedestrian_wide[1] + 1)

        self.pub_pedestrian_area_attention = rospy.Publisher("/pedestrian_area_attention",                                      PolygonStamped, queue_size=1, latch=True)
        self.pedestrian_recog_area_c = [self.pedestrian_center[0] + self.pedestrian_wide[0]/2.0,
                                        self.pedestrian_center[1] + self.pedestrian_wide[1]/2.0]
        self.pedestrian_recog_area_wide_x = [-2.4, 3.3]
        self.pedestrian_recog_area_wide_y = self.pedestrian_wide[1]/2 + 0.05
        self.publish_pedestrian_area_attention()
        # Uses as a reference the direction on which the car comes
        self.pedestrian_area.add_variable('x_pedest_neg','x_pedestrian_neg', 
                                    self.pedestrian_recog_area_wide_x[0], 
                                    min=  -5.0, 
                                    max = 0.0)
        self.pedestrian_area.add_variable('x_pedest_pos','x_pedestrian_pos', 
                                    self.pedestrian_recog_area_wide_x[1], 
                                    min=  0.0, 
                                    max = 5.0)
        self.pedestrian_area.start(self.pedestrian_changes_cb)

        self.pointcloud_sub = rospy.Subscriber('/velodyne_downsampled_xyz',
                                                PointCloud2,
                                                self.velodyne_cb,
                                                queue_size=1)
        self.pub_person_found = rospy.Publisher("/pedestrian_detection",
                                                String, queue_size=1)
        self.pub_person_found_area = rospy.Publisher("/pedestrian_area_detection",                                      Marker, queue_size=1, latch=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def publish_pedestrian_area(self):
        msg = PolygonStamped()
        msg.header.frame_id = "/map"
        p1 = Point()
        p1.x = self.pedestrian_center[0]
        p1.y = self.pedestrian_center[1]
        p1.z = self.pedestrian_center[2]
        msg.polygon.points.append(p1)

        p2 = Point()
        p2.x = self.pedestrian_center[0] + self.pedestrian_wide[0]
        p2.y = self.pedestrian_center[1]
        p2.z = self.pedestrian_center[2]
        msg.polygon.points.append(p2)

        p3 = Point()
        p3.x = self.pedestrian_center[0] + self.pedestrian_wide[0]
        p3.y = self.pedestrian_center[1] + self.pedestrian_wide[1]
        p3.z = self.pedestrian_center[2]
        msg.polygon.points.append(p3)

        p4 = Point()
        p4.x = self.pedestrian_center[0] 
        p4.y = self.pedestrian_center[1] + self.pedestrian_wide[1]
        p4.z = self.pedestrian_center[2]
        msg.polygon.points.append(p4)

        self.pub_pedestrian_area.publish(msg)
        return

    def publish_pedestrian_area_attention(self):
        msg = PolygonStamped()
        msg.header.frame_id = "/map"

        p1 = Point()
        # Farer away point to th2 factory
        p1.x = self.pedestrian_recog_area_c[0] + self.pedestrian_recog_area_wide_x[0]
        p1.y = self.pedestrian_recog_area_c[1] + self.pedestrian_recog_area_wide_y
        p1.z = self.pedestrian_center[2] 
        msg.polygon.points.append(p1)

        p2 = Point()
        p2.x = self.pedestrian_recog_area_c[0] + self.pedestrian_recog_area_wide_x[0]
        p2.y = self.pedestrian_recog_area_c[1] - self.pedestrian_recog_area_wide_y
        p2.z = self.pedestrian_center[2]
        msg.polygon.points.append(p2)

        p3 = Point()
        # Closest point to the factory
        p3.x = self.pedestrian_recog_area_c[0] + self.pedestrian_recog_area_wide_x[1]
        p3.y = self.pedestrian_recog_area_c[1] - self.pedestrian_recog_area_wide_y
        p3.z = self.pedestrian_center[2]
        msg.polygon.points.append(p3)

        p4 = Point()
        # The point that is at the left of the car
        p4.x = self.pedestrian_recog_area_c[0] + self.pedestrian_recog_area_wide_x[1]
        p4.y = self.pedestrian_recog_area_c[1] + self.pedestrian_recog_area_wide_y
        p4.z = self.pedestrian_center[2]
        msg.polygon.points.append(p4)

        self.pub_pedestrian_area_attention.publish(msg)
        return

    def pedestrian_changes_cb(self, config, level):
        self.pedestrian_center[0] = config['x_pose']
        self.pedestrian_center[1] = config['y_pose']
        self.pedestrian_wide[0] =   config['x_size']
        self.pedestrian_wide[1] =   config['y_size']
        self.pedestrian_recog_area_wide_x[0] = config['x_pedest_neg']
        self.pedestrian_recog_area_wide_x[1] = config['x_pedest_pos']
        self.publish_pedestrian_area()
        self.publish_pedestrian_area_attention()
        return config

    def velodyne_cb(self, pointcloud):
        # Transforming velodyne to map
        transform = self.tf_buffer.lookup_transform("map", pointcloud.header.frame_id,    
                                                    pointcloud.header.stamp, rospy.Duration(3.0))
        pointcloud = do_transform_cloud(pointcloud, transform)
        #pointcloud_map 
        cloud_points = list(read_points(
            pointcloud, skip_nans=True, field_names=("x", "y", "z")))
        
        person_found = []
        for p in cloud_points:
            px = p[0] - self.pedestrian_recog_area_c[0]
            py = p[1] - self.pedestrian_recog_area_c[1]
            if (abs(py) < self.pedestrian_recog_area_wide_y):
                # Taking points at the right of the pedestrian
                if (self.pedestrian_recog_area_wide_x[0] < px and 
                   px < self.pedestrian_recog_area_wide_x[1]):   
                    person_found.append([p[0], p[1], p[2]])
                    if len(person_found) > self.n_measurements_consider_valid:
                        self.publish_person_found('CROSSING')
                        self.publish_person_found_area(person_found)
                        return
        self.publish_person_found('CLEAR')
        return 
    
    def publish_person_found(self, status):
        rospy.loginfo('Publishing: ' + status)
        self.pub_person_found.publish(status)

    def publish_person_found_area(self, position):
        # Publishes a circle around the area where the person is placed
        msg = PolygonStamped()
        msg.header.frame_id = "/map"

        px = 0
        py = 0
        pz = 0
        for person in position:
            px += person[0]
            py += person[1]
            pz += person[2]
        px /= len(position)
        py /= len(position)
        pz /= len(position)
        r = 0.4
        marker = Marker(
                type=Marker.CUBE,
                id=0,
                lifetime=rospy.Duration(0.3),
                pose=Pose(Point(px,py,pz), Quaternion(0, 0, 0, 1)),
                scale=Vector3(r,r,2.0),
                header=Header(frame_id='map'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                text="text")
        self.pub_person_found_area.publish(marker)

        # p.append(Point())
        # p4.x = self.pedestrian_center[0] 
        # p4.y = self.pedestrian_center[1] + self.pedestrian_wide[1]
        # p4.z = self.pedestrian_center[2]
        # msg.polygon.points.append(p4)



    def run(self):
        rate = rospy.Rate(10)  # Detect at 10hz
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("pedestrian_detector")
    pd = PedestrianDetector()
    pd.run()

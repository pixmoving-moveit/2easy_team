#!/usr/bin/env python

# Author: Eduardo Ferrera <eferrera@catec.aero>


# Python
import numpy as np

# ROS
import rospy


# from std_msgs.msg import String
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

class AreaDetector(object):
    """
    Obstacle detector, capable of checking if there is an object in a specified area
    """
    def __init__(self, name,  x_offset, y_offset, x_size, y_size):
        #     self.n_measurements_consider_valid = 4
        self.pub_avoidance_area = rospy.Publisher("/avoidance_area_" + name, PolygonStamped, queue_size=1, latch=True)
        self.attention_wide   = [x_offset, y_offset] #[1.5, 10.0]
        self.attention_center = [x_size, y_size, 0.0]#[-48.9, 120.0, 0.0]
            
        self.attention_area = DDynamicReconfigure("avoidance_aerea" + name)
        self.attention_area.add_variable('x_offset','x_offset', 
                                    self.attention_center[0], 
                                    min= self.attention_center[0] - 25.0, 
                                    max = self.attention_center[0] + 25.0)
        self.attention_area.add_variable('y_offset','y_offset', 
                                    self.attention_center[1], 
                                    min= self.attention_center[1] - 25.0, 
                                    max = self.attention_center[1] + 25.0)
        self.attention_area.add_variable('x_size','x_size', 
                                    self.attention_wide[0], 
                                    min= 0.0, 
                                    max = self.attention_wide[0] + 20.0)
        self.attention_area.add_variable('y_size','y_size', 
                                    self.attention_wide[1], 
                                    min= 0.0, 
                                    max = self.attention_wide[1] + 5.0)

        self.publish_attention_area()

        self.attention_area.start(self.attention_changes_cb)

        self.points_found = None

    def attention_changes_cb(self, config, level):
        self.attention_center[0] = config['x_offset']
        self.attention_center[1] = config['y_offset']
        self.attention_wide[0] =   config['x_size']
        self.attention_wide[1] =   config['y_size']
        self.publish_attention_area()
        return config

    def publish_attention_area(self):
        msg = PolygonStamped()
        msg.header.frame_id = "/map"
        p1 = Point()
        p1.x = self.attention_center[0] + self.attention_wide[0]
        p1.y = self.attention_center[1] + self.attention_wide[1]
        p1.z = self.attention_center[2]
        msg.polygon.points.append(p1)

        p2 = Point()
        p2.x = self.attention_center[0] + self.attention_wide[0]
        p2.y = self.attention_center[1] - self.attention_wide[1]
        p2.z = self.attention_center[2]
        msg.polygon.points.append(p2)

        p3 = Point()
        p3.x = self.attention_center[0] - self.attention_wide[0]
        p3.y = self.attention_center[1] - self.attention_wide[1]
        p3.z = self.attention_center[2]
        msg.polygon.points.append(p3)

        p4 = Point()
        p4.x = self.attention_center[0] - self.attention_wide[0]
        p4.y = self.attention_center[1] + self.attention_wide[1]
        p4.z = self.attention_center[2]
        msg.polygon.points.append(p4)

        self.pub_avoidance_area.publish(msg)
        return

    def check_point(self,x,y,z):
        # Checks if a point is on the area
        px = x - self.attention_center[0]
        py = y - self.attention_center[1]
        if (-self.attention_wide[0] < px and 
            px < self.attention_wide[0]): 
            if (-self.attention_wide[1] < py and 
                py < self.attention_wide[1]):
                if self.points_found == None:
                    self.points_found = [] 
                # The point is inside the box
                self.points_found.append([x,y,z])
        return

    def get_filtered_position(self):
        # Centeres the position of the measurements
        if self.points_found == None:
            return

        px = 0
        py = 0
        pz = 0
        for p in self.points_found:
            px += p[0]
            py += p[1]
            pz += p[2]
        px /= len(self.points_found)
        py /= len(self.points_found)
        pz /= len(self.points_found)
        return [px, py, pz]

    def get_n_points(self):
        if self.points_found == None:
            return 0
        return len(self.points_found)

class CarObstacleDetector(object):
    """
    Obstacle detector, capable of checking if there is object infront of the car
    """
    def __init__(self):
        self.Area1 = AreaDetector("area1_left", 1.5, 10.0, -48.9, 120.0)
        self.Area2 = AreaDetector("area2_left", 1.5, 10.0, -49.4, 140.2)

        self.pointcloud_sub = rospy.Subscriber('/velodyne_downsampled_xyz',
                                                PointCloud2,
                                                self.velodyne_cb,
                                                queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub_object_found_area = rospy.Publisher("/attention_area_detection",
                                              Marker, queue_size=1, latch=True)

        
    #     self.pub_person_found = rospy.Publisher("/attention_detection",
    #                                             String, queue_size=1)

    
    def velodyne_cb(self, pointcloud):
        # Transforming velodyne to map
        transform = self.tf_buffer.lookup_transform("map", pointcloud.header.frame_id,    
                                                    pointcloud.header.stamp, rospy.Duration(3.0))
        pointcloud = do_transform_cloud(pointcloud, transform)

        self.Area1.points_found = None
        self.Area2.points_found = None
        #pointcloud_map 
        cloud_points = list(read_points(
            pointcloud, skip_nans=True, field_names=("x", "y", "z")))

        print "working"
        
        for p in cloud_points:
            self.Area1.check_point(p[0], p[1], p[2])
            self.Area2.check_point(p[0], p[1], p[2])

        if (self.Area1.get_n_points()):
            pos = self.Area1.get_filtered_position()
            print "Objet found in area 1"
            print pos
            self.publish_object_found_area( pos[0], pos[1], pos[2])

        if (self.Area2.get_n_points()):
            pos = self.Area2.get_filtered_position()
            print "Objet found in area 2"
            print pos
            self.publish_object_found_area( pos[0], pos[1], pos[2])

    def publish_object_found_area(self, px, py, pz):
        # Publishes a circle around the area where the object is placed
        msg = PolygonStamped()
        msg.header.frame_id = "/map"

        r = 3.0
        marker = Marker(
                type=Marker.CUBE,
                id=0,
                lifetime=rospy.Duration(1.0),
                pose=Pose(Point(px,py,pz), Quaternion(0, 0, 0, 1)),
                scale=Vector3(r,r,2.0),
                header=Header(frame_id='map'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                text="text")
        self.pub_object_found_area.publish(marker)

    
    # def publish_person_found(self, status):
    #     rospy.loginfo('Publishing: ' + status)
    #     self.pub_person_found.publish(status)

  



    def run(self):
        rate = rospy.Rate(10)  # Detect at 10hz
        while not rospy.is_shutdown():
            # Publishes the area where the car pays attention
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("car_avoidance_detector")
    pd = CarObstacleDetector()
    pd.run()

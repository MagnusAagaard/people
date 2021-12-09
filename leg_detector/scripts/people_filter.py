#!/usr/bin/python3
import rospy
from people_msgs.msg import PositionMeasurementArray
from nav_msgs.msg import OccupancyGrid
from math import pow, sqrt
# Visulalization
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import ColorRGBA, Header
from tf import TransformListener


class PeopleFilter:
    def __init__(self):
        self.marker = None
        self.wall_points = None
        self.__init_params()
        self.__init_subscribers()
        self.__init_publishers()

    def __init_params(self):
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~static_frame', 'odom_comb')
        self.laser_frame = rospy.get_param('~laser_frame', 'scan')
        self.dist_from_wall_lim = rospy.get_param('~dist_from_wall_lim', 0.2)
        self.listener = TransformListener()
        self.listener.waitForTransform(self.map_frame, self.odom_frame, rospy.Time(), rospy.Duration(60.0))

    def __init_subscribers(self):
        people_measurements = rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, self.__people_cb)
        map_sub = rospy.Subscriber(self.map_frame, OccupancyGrid, self.__map_cb)

    def __init_publishers(self):
        self.vis_pub = rospy.Publisher('~visualisation', Marker, queue_size=1)
        self.filtered_people_vis_pub = rospy.Publisher('~people_visualisation', Marker, queue_size=1)
        filtered_people_topic = rospy.get_param('filtered_people_topic', '~filtered_people_topic')
        self.filtered_people_pub = rospy.Publisher(filtered_people_topic, PositionMeasurementArray, queue_size=1)

    def __people_cb(self, msg):
        now = rospy.Time.now()
        header = Header(stamp=now, frame_id='map')
        COLOR = ColorRGBA(r=1.0, g=0, b=0, a=1)
        marker = Marker(header=header)
        marker.id = 0
        # Points
        marker.type = Marker.POINTS
        # Add/modify
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        pt_array = []
        color_array = []
        filtered_people = []
        for person in msg.people:
            person_pt_stamped = PointStamped(header=person.header, point=person.pos)
            transformed_pt = self.listener.transformPoint(self.map_frame, person_pt_stamped)
            if self.dist_to_wall(transformed_pt.point) > self.dist_from_wall_lim:
                pt_array.append(transformed_pt.point)
                color_array.append(COLOR)
                filtered_people.append(person)
        marker.points = pt_array
        marker.colors = color_array
        if len(pt_array):
            self.filtered_people_vis_pub.publish(marker)
        if len(filtered_people):
            msg_copy = msg
            msg_copy.people = filtered_people
            self.filtered_people_pub.publish(msg_copy)


    def __map_cb(self, msg):
        now = rospy.Time.now()
        header = Header(stamp=now, frame_id='map')
        COLOR = ColorRGBA(r=1.0, g=0, b=0, a=1)
        self.marker = Marker(header=header)
        self.marker.id = 0
        # Points
        self.marker.type = Marker.POINTS
        # Add/modify
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.02
        self.marker.scale.y = 0.02
        pt_array = []
        color_array = []
        w = msg.info.width
        h = msg.info.height
        ori_x, ori_y = msg.info.origin.position.x, msg.info.origin.position.y
        res = msg.info.resolution
        for i, pt in enumerate(msg.data):
            if pt < 0:
                x_offset = (i % w)*res
                y_offset = int(i/w)*res
                pt_array.append(Point(x=ori_x+x_offset, y=ori_y+y_offset, z=0))
                color_array.append(COLOR)
        self.marker.points = pt_array
        self.marker.colors = color_array
        self.wall_points = pt_array
        self.vis_pub.publish(self.marker)

    def dist_between_points(self, pt1, pt2):
        x1, y1 = pt1.x, pt1.y
        x2, y2 = pt2.x, pt2.y
        return sqrt(pow(x2-x1,2) + pow(y2-y1,2))

    def dist_to_wall(self, person):
        if self.wall_points is not None:
            closests_point = 10000.0
            for pt in self.wall_points:
                dist = self.dist_between_points(person, pt)
                if dist < closests_point:
                    closests_point = dist
            return closests_point
        return -1

    def publish_points(self):
        if self.marker is not None:
            self.vis_pub.publish(self.marker)

def main():
    rospy.init_node('people_filter', log_level=rospy.INFO)
    pf = PeopleFilter()
    rospy.spin()
    #r = rospy.Rate(10)
    #while not rospy.is_shutdown():
    #    pf.publish_points()
    #    r.sleep()

if __name__ == "__main__":
    main()

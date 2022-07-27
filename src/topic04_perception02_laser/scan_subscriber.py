#!/usr/bin/env python
from tracemalloc import stop
import rospy
import cv2
from sensor_msgs.msg import LaserScan
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Twist
import math
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

def scan_callback(scan_data):
    # min_value, min_index = min_range_index(scan_data.ranges)
    # print("\nthe minimum range value is: ", min_value)
    # print("the minimum range index is: ", min_index)

    # average2 = average_between_indices(scan_data.ranges, 320, 400)
    # print("\nthe average between 2 indices is: ", average2)
    # print(" Data type is given is ",type(scan_data.ranges))
    # cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    ranges= [x for x in scan_data.ranges if not math.isnan(x)]
    ranges = [x for x in ranges if not math.isinf(x)]
    filtered_input = np.array(ranges)
    print(" Data type is given is before ",filtered_input.shape)
    # print(" filtered input  value ", filtered_input)
    filtered_input = np.reshape(filtered_input, (-1, 2))
    print(" Data type is given is after  ",filtered_input.shape)
    # print(" filtered input  value  now ", filtered_input)

    kmeans = KMeans(n_clusters=2, random_state=0).fit(filtered_input)
    # kmeans.labels_
    # kmeans.predict([[0, 0], [12, 3]])
    
    # print( "kmean output",kmeans.labels_)
    lane1 = np.array(())
    lane2 = np.array(())
    # ITERATIN OVER THE ARRAY 
    iter = 0
    for x in kmeans.labels_:
        if x == 0:
            lane1 = np.append(lane1,filtered_input[iter])
            iter = iter + 1 
            
        else:
            lane2 = np.append(lane2, filtered_input[iter])
            iter = iter + 1
    # print(" lane 1", lane1)
    lane1 = np.reshape(lane1, (-1, 2))    
    lane2 = np.reshape(lane2, (-1, 2))
    print(" lane 1", lane1.shape)
    print("Lane 2", lane2.shape)
    x1 = lane1[:,0]
    y1 = lane1[:,1]
    m, b = np.polyfit(x1, y1, 1)
    print( " m value is ", m)
    print( " b value is ", b)

    # lane1x = x1
    lane1y = np.array(()) 
    i = 0    
    for x in x1:
        lane1y = np.append(lane1y,x*m + b)

    print("Lane 1y shape  = ", lane1y.shape)
    lane1x = x1

    x1_value1 = np.amin(lane1x)
    y1_value1 = np.amin(lane1y)

    x1_value2 = np.amax(lane1x)
    y1_value2 = np.amax(lane1y)

    plt(x1_value1, y1_value1, 'bo')
    plt(x1_value2, y1_value2, 'go')
    # cv_image = np.zeros((800,800,3), dtype=np.uint8)
    # cv2.line(cv_image, (int(x1_value1) , int(y1_value1) ), (int(x1_value2), int(y1_value2)), (255, 0, 0), 2)
    # cv2.imshow('Hough Lines', cv_image)
    # cv2.waitKey(3)
    # heading_image = display_heading_line(
    # cv_image, steering_angle, line_color=(0, 0, 255), line_width=5)
    # cv2.imshow("heading_image", heading_image)
    # plt.plot(x1, y1, 'o')
    # plt.plot(x1, m*x1 + b)
    # plt.plot(x_axis1,y_axis1)
    # plt.plot(lane2[0],lane2[1])


    if average_between_indices(scan_data.ranges, 340, 380)<=0.35:
        print("obstacle found")
        vel_msg.linear.x = 0.0


    elif average_between_indices(scan_data.ranges, 0, 40) <= 0.4:
        print("Robot right side not clear")
        # vel_msg.linear.x = 0.0  
        vel_msg.angular.z = 0.35     

    elif average_between_indices(scan_data.ranges, 660, 720) <= 0.4:
        print("Robot left side not clear")
        # vel_msg.linear.x = 0.0  
        vel_msg.angular.z = -0.35 

    elif scan_data.ranges[360]>=0.1:
        print("clear path")
        vel_msg.angular.z = 0.0
        vel_msg.linear.x = 0.2 
    plt.show()
    # pub.publish(vel_msg)


###########################old code with the call backs

# def scan_callback(scan_data):
#     # Find minimum range
#     min_value, min_index = min_range_index(scan_data.ranges)
#     print("\nthe minimum range value is: ", min_value)
#     print("the minimum range index is: ", min_index)

#     max_value, max_index = max_range_index(scan_data.ranges)
#     print("\nthe maximum range value is: ", max_value)
#     print("the maximum range index is: ", max_index)

#     average_value = average_range(scan_data.ranges)
#     print("\nthe average range value is: ", average_value)

#     average2 = average_between_indices(scan_data.ranges, 2, 7)
#     print("\nthe average between 2 indices is: ", average2)

#     print("the field of view: ", field_of_view(scan_data))

#     print("\n Center scan range: ", center_range(scan_data.ranges))

#     # move()
#     # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#     # rate = rospy.Rate(10)
#     # vel_msg = Twist()

#     # if center_range(scan_data.ranges) >= 0.1:
#     #     print("it should go straight**********************************************************")
#     #     vel_msg.linear.x = 1.0

#     # elif center_range(scan_data.ranges)>=0.1:
# # 		vel_msg.linear.x = vel_msg.linear.x + 0.2
# # rospy.init_node('bot_controller', anonymous=True)
# # stop(vel_msg)

# # print("Use w,s,a,d to move the bot and press q for exit")

# # while not rospy.is_shutdown():
# # keyPressed = input()
# # if center_range<=0.1:
# # 	vel_msg.linear.x = 0.0

# # 	# pub.publish(vel_msg)
# # 	# break
# # elif center_range(scan_data.ranges)>=0.1:
# # 		vel_msg.linear.x = vel_msg.linear.x + 0.2


# def field_of_view(scan_data):
#     return (scan_data.angle_max-scan_data.angle_min)*180.0/3.14

# # find the max range and its index


def min_range_index(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    ranges = [x for x in ranges if not math.isinf(x)]
    return (min(ranges), ranges.index(min(ranges)))

# # find the max range


# def max_range_index(ranges):
#     ranges = [x for x in ranges if not math.isnan(x)]
#     ranges = [x for x in ranges if not math.isinf(x)]
#     return (max(ranges), ranges.index(max(ranges)))

# # find the average range


# def average_range(ranges):
#     ranges = [x for x in ranges if not math.isnan(x)]
#     ranges = [x for x in ranges if not math.isinf(x)]
#     return (sum(ranges) / float(len(ranges)))


def average_between_indices(ranges, i, j):
    ranges = [x for x in ranges if not math.isnan(x)]
    ranges = [x for x in ranges if not math.isinf(x)]
    slice_of_array = ranges[i: j+1]
    # print("slice ", slice_of_array)
    # print("SUM ", sum(slice_of_array))
    if len(slice_of_array) == 0:
        return 1.0
    else:     
        return (sum(slice_of_array) / float(len(slice_of_array)))

# def image_callback(ros_image):
#     global bridge
#     return bridge.imgmsg_to_cv2(ros_image, "bgr8")

# def center_range(ranges):
#     center_index = int(len(ranges)/2)
#     return ranges[center_index]


# def stop():
#     return False


# def move():
# 	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# 	# rospy.init_node('bot_controller', anonymous=True)
# 	rate = rospy.Rate(10)  # 10hz

# 	vel_msg = Twist()
# 	# stop(vel_msg)

# 	# print("Use w,s,a,d to move the bot and press q for exit")
#     # if (True):
#     #     vel_msg.linear.x = 0.2

# 	# while not rospy.is_shutdown():
# 	# keyPressed = input()
# 	if center_range<=0.1:
# 		vel_msg.linear.x = 0.0

# 	# 	# pub.publish(vel_msg)
# 	# 	# break
# 	elif center_range(scan_data.ranges)>=0.1:
# 			vel_msg.linear.x = vel_msg.linear.x + 0.2

# 	# elif keyPressed=='a':
# 	# 		vel_msg.angular.z = vel_msg.angular.z + 0.2

# 	# elif keyPressed=='s':
# 	# 	vel_msg.linear.x = vel_msg.linear.x - 0.2

# 	# elif keyPressed=='d':
# 	# 		vel_msg.angular.z = vel_msg.angular.z - 0.2

# 	# elif keyPressed=='x':
# 	# 		stop(vel_msg)

# 	pub.publish(vel_msg)


if __name__ == '__main__':
    # init new a node and give it a name
    rospy.init_node('scan_node', anonymous=True)
    # subscribe to the topic /scan.
    sub = rospy.Subscriber("scan", LaserScan, scan_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    vel_msg = Twist()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

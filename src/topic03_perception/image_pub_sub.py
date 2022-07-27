#!/usr/bin/env python

import logging
import math
from random import randint, random
from tracemalloc import stop
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np
from geometry_msgs.msg import Twist

bridge = CvBridge()
XL = 0
XR = 800
previous_offset = 0
previous_leftLane = 0
previous_rightLane = 0
smooth = 0
leftLaneSmooth = np.array(())
rightLaneSmooth = np.array(())
centerLaneSmooth = np.array(())
stableLeftLane = np.array(())
stableRightLane = np.array(())
leftLaneAvg = np.array(())
rightLaneAvg = np.array(())
reject = 0
pre_left_x2 = 0
# random.seed(12345)


def image_callback(ros_image):
    print('got an image')
    global bridge

    # INITIALIZE FOR THE LEFT AND RIGHT LANE
    global XL
    global XR
    global previous_offset
    global previous_leftLane
    global previous_rightLane
    global smooth
    global leftLaneSmooth
    global rightLaneSmooth
    global centerLaneSmooth
    global reject
    global stableLeftLane
    global stableRightLane
    global pre_left_x2
    global leftLaneAvg
    global rightLaneAvg
    # convert ros_image into an opencv-compatible image
    # try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    cv_image_path = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    # ORIGINAL IMAGE OUTPUT
    # cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    # HSV IMAGE OUTPUT
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # cv2.imshow("hsv image",hsv_image)

    # find the upper and lower bounds of the yellow color (tennis ball)
    greenLower = (30, 0, 00)
    greenUpper = (55, 220, 220)

    # define a mask using the lower and upper bounds of the green color to get the black and white color
    mask = cv2.inRange(hsv_image, greenLower, greenUpper)
    # cv2.imshow("mask image",mask)

    # MASKING METHOD TWO, (note - two mask are used one for left lane and one for the right lane)
    edges = mask
    height, width = edges.shape
    masker = np.zeros_like(edges)

    # # # only focus bottom half of the screen
    # # polygon = np.array([[
    # #     (0, height * 1 / 2),
    # #     (width, height * 1 / 2),
    # #     (width, height),
    # #     (0, height),
    # # ]], np.int32)

    # polygon = np.array([[
    #     (450, 300),
    #     (800, 150),
    #     (800, 500),
    #     (450, 650),
    # ]], np.int32)

    # cv2.fillPoly(masker, polygon, 255)

    # polygon = np.array([[
    #     (0, 150),
    #     (350, 300),
    #     (350, 650),
    #     (0, 500),
    # ]], np.int32)

    # polygon = np.array([[
    #     (600, 300),
    #     (800, 300),
    #     (800, 450),
    #     (600, 450),
    # ]], np.int32)

    # cv2.fillPoly(masker, polygon, 255)

    polygon = np.array([[
        (0, 0),
        (450, 0),
        (450, 800),
        (0, 800),
    ]], np.int32)

    cv2.fillPoly(masker, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, masker)
    # cv2.imshow("cropped_edges",cropped_edges)

    # TO FIND THE CONTOUR IN THE IMAGE
    # Find contours
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the convex hull object for each contour
    hull_list = []
    for i in range(len(contours)):
        hull = cv2.convexHull(contours[i])
        hull_list.append(hull)
    img = np.zeros((800, 800))

    # # Draw contours + hull results
    drawing = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
    for i in range(len(contours)):
        # color = (randint(0,256), randint(0,256), randint(0,256))
        color = (255, 255, 255)
        # cv2.drawContours(drawing, contours, i, color)
        cv2.drawContours(drawing, hull_list, i, color)
    # print(len(hull_list))
    pt = np.array(contours)
    # cv2.fillPoly(drawing, pts =[pt], color=(255,255,255))
    # # Show in a window
    # cv2.imshow('Contours', drawing)
    # print("contour" , len(contours))

    contour = np.zeros((800, 800))
    cv2.drawContours(mask, hull_list, -1, color=(255,
                     255, 255), thickness=cv2.FILLED)
    # cv2.fillPoly(mask, pts =[contours], color=(255,255,255))
    cv2.imshow(" CONTOUR OUTPUT ", mask)

    # CONTOUR APPROXIMATION
    cnt = contours[0]
    # epsilon = 0.5*cv2.arcLength(cnt,True)
    # approx = cv2.approxPolyDP(cnt,epsilon,True)
    # img = np.zeros( (800,800) )
    # cv2.drawContours(img, approx, -1, color=(255, 255, 255), thickness=cv2.FILLED)
    # # cv2.fillPoly(mask, pts =[contours], color=(255,255,255))
    # cv2.imshow(" ", img)

    # CONTOUR FITTING LINE
#   rows,cols = img.shape[:2]
#   [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
#   lefty = int((-x*vy/vx) + y)
#   righty = int(((cols-x)*vy/vx)+y)
#   cv2.line(img,(cols-1,righty),(0,lefty),(0,255,0),2)
#   cv2.imshow(" IMG ", img)

    # CANNY IMAGE - EDGE DETECTION
    # image_g = cv2.cvtColor(masked_image , cv2.COLOR_BGR2GRAY)
    # image_canny = cv2.Canny(masked_image, 50, 200, apertureSize = 3)
    # image_canny = cv2.Canny(masked_image, 200,400)

    image_canny = cv2.Canny(mask, 200, 400, apertureSize=3)
    cv2.imshow('canny image', image_canny)

    # HOUGH TRANFORM FOR THE EDGE DETECTION
    lines = cv2.HoughLinesP(image_canny, 1, 1 * np.pi/180,
                            70, np.array([]), minLineLength=5, maxLineGap=8)
    # cv2.imshow("Hough image",lines)

    # TO DRAW THE HOUGH TRANSFORM OUTPUT LANE ON THE IMAGE
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]

            # rho = lines[0][0][0]
            # theta = lines[0][0][1]

            x0 = rho * np.cos(theta)
            y0 = rho * np.sin(theta)

            a = np.cos(theta)
            b = np.sin(theta)

            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 2)

    # ####FILTER THE X AXIS PARALLEL LANE
    #           if round(y1) == round(y2) or round(x1) == round(x2):
    #             # print(x1 ," and ", x2)
    #             # print(y1 ," y value ", y2)
    #             continue

    # # ##### TO GET THE RIGHT LANE X AND LEFT LANE X
    # #             if x1 > 400:
    # #               XR = 800 - x1
    # #               # print("XR VALUE IS ", XR)
    # #             else:
    # #               XL = x1
    # #               # print("XL VALUE IS ", XL)

    # # ##### TO GET THE CENTER LANE
    # #             center_lane = XR - XL + 400
    # #             cv2.circle(cv_image, (center_lane,400), radius=0, color=(0, 255, 0), thickness=15) ##MEAN THE CENTER LANE WE NEED TO FOLLOW
    # #             cv2.circle(cv_image, (400,400), radius=0, color=(255, 0, 0), thickness=10) # WHERE WE ARE HEADING TO

    #           #   # print("FINAL OUTPUT FOR THE PID ", center_lane - 400) # BOTH THE MINUS WILL FEED IT PID
    #           else:
    #             cv2.line(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
    else:
        print("Hough is not working")

    # cv2.imshow('Hough Lines', cv_image)
    # cv2.destroyAllWindows()
    # cv2.waitKey(0)

    # TO GENERATE THE LEFT AND RIGHT LANE
    lane_lines = []
    if lines is None:
        logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = cv_image.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    # left lane line segment should be on left 2/3 of the screen
    left_region_boundary = width * (1 - boundary)
    # right lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary
    left_count = 0
    right_count = 0

    for line_segment in lines:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info(
                    'skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
                    left_count += 1
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))
                    right_count += 1
#   print("Left_count = ", left_count)
#   print("Right_count = ", right_count)

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(cv_image, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(cv_image, right_fit_average))

    logging.debug('lane lines: %s' % lane_lines)

    line_image = np.zeros_like(cv_image)
    if lane_lines is not None:
        for line in lane_lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    line_image = cv2.addWeighted(cv_image_path, 0.8, line_image, 1, 1)
    cv2.imshow("lane lines", line_image)

    # GENERATE THE CENTER LANE
    # prev_lane_lines = []
    # # if lane_lines[0][0] is not None and lane_lines[1][0] is not None:  # TO AVOID OUTOF RANGE
    # if len(lane_lines) == 2:
    #   _, _, left_x2, _ = lane_lines[0][0]
    #   _, _, right_x2, _ = lane_lines[1][0]
    #   mid = int(width / 2)
    #   x_offset = (left_x2 + right_x2) / 2 - mid
    #   y_offset = int(height / 2)
    #   x1, _, x2, _ = lane_lines[0][0]
    #   prev_lane_lines = lane_lines
    # else:
    #   _, _, left_x2, _ = prev_lane_lines[0][0]
    #   _, _, right_x2, _ = prev_lane_lines[1][0]
    #   mid = int(width / 2)
    #   x_offset = (left_x2 + right_x2) / 2 - mid
    #   y_offset = int(height / 2)
    #   x1, _, x2, _ = prev_lane_lines[0][0]

    # ALTERNATE GENERATE CENTER LANE
#   print("number of lane  = ",len(lane_lines))
    _, _, left_x2, _ = lane_lines[0][0]
    _, _, right_x2, _ = lane_lines[1][0]
#   print("lane_lines", lane_lines)
    print("LEFT _x2 ", left_x2)
    print("right_x2 ", right_x2)
    mid = int(width / 2)
#   print("mid", mid)
    x_offset = (left_x2 + right_x2) / 2 - mid
    print("x_offset ", x_offset)
    y_offset = int(height / 2)
#   print("y_offset ",y_offset)
    # x1, _, x2, _ = lane_lines[0][0]
    # # prev_lane_lines = lane_lines
    # x_offset = x2 - x1
    # y_offset = int(height / 2)
    # print("x_offset ",x_offset )

    # FILTER THE NEGATIVE LANE
#   if left_x2 > 0 or right_x2 > 0:

# # WHEN ONE LANE DETECTED
#   leftRatio = left_x2/pre_left_x2
#   pre_left_x2 = left_x2
#   leftRatio = round(leftRatio,2)
#   stableLeftLane = np.append(stableLeftLane,leftRatio)
#   constLeftLane = np.sum(stableLeftLane)

#   rightRatio = right_x2/pre_right_x2
#   pre_right_x2 = right_x2
#   rightRatio = round(rightRatio,2)
#   stableRightLane = np.append(stableRightLane,rightRatio)
#   constRightLane = np.sum(stableRightLane)

#   parameterStable = 50  # change it to 2
#   if(constLeftLane < parameterStable):
#     print("ConstLeftLane  = ", constLeftLane)


#   elif (constRightLane < parameterStable):
#     print("ConstRightLane  = ", constRightLane)

# #WHEN TWO LANE ARE DETECTED
#   else:
    # FILTER THE LANE
    # parameterDiff = 80
    # laneDiff = right_x2 - left_x2
    # if laneDiff < parameterDiff:
    #     print("Lane difference ", laneDiff)
# STEERING ANGLE GENERATE

    # angle (in radian) to center vertical line
    # angle_to_mid_radian = math.atan(x_offset / y_offset)
    # # angle (in degrees) to center vertical line
    # angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
    # steering_angle = 400 - x_offset
    # print("STEERING ANGlE  = ", steering_angle)

    # # DISPLAY THE CENTER LANE
    # heading_image = display_heading_line(
    #     cv_image_path, steering_angle, line_color=(0, 0, 255), line_width=5)
    # cv2.imshow("heading_image", heading_image)
    # print("steering_angle", steering_angle)

    #   # Filter the steering
    #   leftLaneSmooth = np.append(leftLaneSmooth,left_x2)
    #   rightLanesmooth = np.append(leftLaneSmooth,right_x2)
    new_offset = filter(left_x2, right_x2,
                        previous_leftLane, previous_rightLane)
    #   new_offset = filter(leftLaneSmooth ,rightLanesmooth,previous_leftLane,previous_rightLane)
    #   print( "NEW STEERING ANGLE IS ",new_offset)
    #   new_offset = x_offset

    # STABILIZE STEERING OUTPUT
    #   stabilize_angle = stabilize_steering_angle(new_offset,previous_offset,len(lane_lines),20,10)
    #   previous_offset = new_offset
    #   previous_leftLane = left_x2
    #   previous_rightLane = right_x2

    #   heading_image = display_heading_line(
    #       cv_image_path, stabilize_angle, line_color=(0, 0, 255), line_width=5)
    #   cv2.imshow("heading_image", heading_image)

    # CENTER LANE SMOOTH
    leftLaneAvg = np.append(leftLaneAvg, left_x2)
    rightLaneAvg = np.append(rightLaneAvg, right_x2)
    centerLaneSmooth = np.append(centerLaneSmooth, new_offset)

    # MOVE THE ROBOT
    parameter = 50
    if smooth == parameter:
        centerLane = np.sum(centerLaneSmooth) / parameter
        leftLaneAvg = np.sum(leftLaneAvg) / parameter
        rightLaneAvg = np.sum(rightLaneAvg) / parameter
        print("Left Lane IN THE LOOP IS GIVEN BY THE VALUE = ", leftLaneAvg)
        print("Right Lane IN THE LOOP IS GIVEN BY THE VALUE = ", rightLaneAvg)
        offset = filter(int(leftLaneAvg), int(rightLaneAvg),
                        previous_leftLane, previous_rightLane)
        stabilize_angle = stabilize_steering_angle(
            offset , previous_offset, len(lane_lines), 20, 10)

        cv2.circle(cv_image_path, (int(leftLaneAvg),400), radius=0, color=(0, 255, 0), thickness=15) ##MEAN THE CENTER LANE WE NEED TO FOLLOW
        cv2.circle(cv_image_path, (int(rightLaneAvg),400), radius=0, color=(0, 255, 0), thickness=15)  
        cv2.imshow('AVG LANE ', cv_image_path)  
        # previous_offset = new_offset
        # previous_leftLane = left_x2
        # previous_rightLane = right_x2

        previous_offset = offset
        previous_leftLane = leftLaneAvg
        previous_rightLane = rightLaneAvg 
        # heading_image = display_heading_line(cv_image_path, stabilize_angle, line_color=(0, 0, 255), line_width=5)
        # cv2.imshow("heading_image", heading_image)
        # print("steering angle in loop is  = ", stabilize_angle)
        move(stabilize_angle)
        # move(offset)
        smooth = 0
        centerLaneSmooth = np.array(())

    smooth = smooth + 1
    # return line_image
# else:
#     parameterRej = 15
#     if reject == parameterRej:
#         move_reject()
#         reject = 0
#         print("REJECTED",laneDiff)
#     reject += 1


def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image


def move(steering_angle):
    print("STEERING IN MOVE IS ", steering_angle)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    if (-50 < steering_angle <= 50):
        vel_msg.linear.x = 0.3
        vel_msg.angular.z = 0.0
        print("GOING STRAINGHT")

    elif (steering_angle > 50):
        vel_msg.angular.z = 0.1
        vel_msg.linear.x = 0.0
        print("Streeing left")

    elif (steering_angle <= -50):
        vel_msg.angular.z = -0.1
        vel_msg.linear.x = 0.0
        print("Streeing right")
    else:
        print("NOT ENTER ANY ")

    pub.publish(vel_msg)


def move_reject():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = 0.01
    vel_msg.angular.z = 0.0
    pub.publish(vel_msg)


def filter(leftLane, rightLane, previousLeftLane, previousRightLane):
    leftLaneShift = leftLane - previousLeftLane
    rightLaneShift = rightLane - previousRightLane

    # AVOID NEGATIVE LANE
    # AVOID IF LEFT OR RIGHT LANE MORE THAN THE OTHER LANE.
    # if (leftLane > rightLane):
    #     leftLane = previousLeftLane

    # if (rightLane < leftLane):
    #     rightLane = previousRightLane

    # if (leftLaneShift) < 50 :
    #     leftLane = leftLane - leftLaneShift

    # if (rightLaneShift) < 50:
    #     rightLane = rightLane - rightLaneShift

    # print("New Left Lane is  = ", leftLane)
    # print("New right Lane is ", rightLane)

    new_offset = (leftLane + rightLane) / 2 - 400
    print("New offset is given by = ", new_offset)
    return new_offset


def stabilize_steering_angle(
        new_steering_angle,
        previous_steering_angle,
        num_of_lane_lines,
        max_angle_deviation_two_lines=5,
        max_angle_deviation_one_lane=1):
    """
    Using last steering angle to stabilize the steering angle
    if new angle is too different from current angle, 
    only turn by max_angle_deviation degrees
    """
    if num_of_lane_lines == 2:
        # if both lane lines detected, then we can deviate more
        max_angle_deviation = max_angle_deviation_two_lines
    else:
        # if only one lane detected, don't deviate too much
        max_angle_deviation = max_angle_deviation_one_lane

    angle_deviation = new_steering_angle - previous_steering_angle
    if angle_deviation > max_angle_deviation:
        stabilized_steering_angle = new_steering_angle
    else:
        stabilized_steering_angle = new_steering_angle - angle_deviation
    return stabilized_steering_angle


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    # for turtlebot3 waffle
    image_topic = "/camera/rgb/image_raw"
    # for usb cam
    # image_topic="/usb_cam/image_raw"
    image_sub = rospy.Subscriber(image_topic, Image, image_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

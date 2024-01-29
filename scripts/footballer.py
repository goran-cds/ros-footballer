#!/usr/bin/env python

from __future__ import print_function
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class CaptureObject:
    def __init__(self):

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.rate = rospy.Rate(1)
        self.rot = Twist()
        
        self.ball_found = False
        self.goal_found = False
        self.ball_in_sight = False

        self.ball_offset = 50
        self.ball_offset_sign = 1

        self.goal_offset = 150

        self.ball_x = 0
        self.ball_y = 0

        self.shoot_spot_direction = ""
        self.ball_direction = ""
        self.goal_direction = "straight"

        self.state = "FIND_BALL"

        self.bridge = CvBridge()

        # Connect to image topic
        img_topic = "/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Allow up to one second to connection
        rospy.sleep(1)

    def callback(self, data):
        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image = cv_image

        # Handling robot's states
        if self.state == "FIND_BALL":
            self.process_image_find_ball(cv_image)
        elif self.state == "GOTO_SHOOTING_SPOT":
            self.process_image_find_ball(cv_image)
            if self.ball_in_sight:
                self.rot.angular.z = 0
                self.rot.linear.x = 0.1
                self.pub.publish(self.rot)
            else:
                print("Reached shooting spot")
                self.stop()
                self.state = "FIND_GOAL"
        elif self.state == "FIND_GOAL":
            self.process_image_find_goal(cv_image)
        
    def process_image_find_ball(self,img):
        # Convert BGR to HSV
        hsv_image_ball = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        hsv_image_ball = cv2.resize(hsv_image_ball, (640,300))

        img = cv2.resize(img, (640,300))

        # HSV color range for red color
        ball_low_HSV = (0, 100, 100)
        ball_high_HSV = (18, 255, 255)

        # Threshold the image to get only red color
        mask_ball = cv2.inRange(hsv_image_ball, ball_low_HSV, ball_high_HSV)

        # Find contours in the ball's mask image
        contours_ball, _ = cv2.findContours(mask_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Reverse until we see the ball
        if not self.ball_found:
            self.rot.linear.x = -0.2
            self.pub.publish(self.rot)

        # Check if any contour is found
        if contours_ball:
            self.ball_in_sight = True
            # Get the largest contour
            largest_contour = max(contours_ball, key=cv2.contourArea)

            # Calculate the center of the largest contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                self.ball_x = cx
                self.ball_y = cy

                if self.state == "FIND_BALL" and self.shoot_spot_direction != "straight":
                    self.move_towards_object(cx - self.ball_offset * self.ball_offset_sign, cy)
                elif self.shoot_spot_direction == "straight" and self.state != "GOTO_SHOOTING_SPOT":
                    self.state = "GOTO_SHOOTING_SPOT"

                # Display ball center on the image
                cv2.circle(img, (cx, cy), 5, (0, 255, 0), -1)

                # DIsplay ball offset center on the image
                cv2.circle(img, (cx - self.ball_offset * self.ball_offset_sign, cy), 5, (255, 255, 255), -1)

                self.check_shoot_spot_pos(cx - self.ball_offset * self.ball_offset_sign)
                self.check_ball_pos(cx)

                if self.ball_direction == "left" and not self.ball_found:
                    self.ball_offset_sign = 1
                elif self.ball_direction == "right" and not self.ball_found:
                    self.ball_offset_sign = -1

                self.ball_found = True
        else:
            self.ball_in_sight = False

        print(f"spot_x: {self.ball_x - self.ball_offset} | spot_y: {self.ball_y}")

        # Display the image
        cv2.imshow('Camera view: detect ball', img)
        cv2.waitKey(1)

    def process_image_find_goal(self,img):
        # Convert BGR to HSV
        hsv_image_goal = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        hsv_image_goal = cv2.resize(hsv_image_goal, (640,300))

        img = cv2.resize(img, (640,300))

        # HSV color range for yellow color
        goal_low_HSV = (25, 100, 100)
        goal_high_HSV = (32, 255, 255)

        # Threshold the image to get only yellow color
        mask_goal = cv2.inRange(hsv_image_goal, goal_low_HSV, goal_high_HSV)

        # Find contours in the goal's mask image
        contours_goal, _ = cv2.findContours(mask_goal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if(not self.goal_found):
            self.rot.angular.z = -0.1 * self.ball_offset_sign
            self.pub.publish(self.rot)

        if contours_goal:
            # Get the largest contour
            largest_contour = max(contours_goal, key=cv2.contourArea)

            # Calculate the center of the largest contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                self.move_towards_object(cx + self.goal_offset * self.ball_offset_sign, cy)
	            
                # Display center on the image
                cv2.circle(img, (cx + self.goal_offset * self.ball_offset_sign, cy), 5, (255, 255, 255), -1)

                self.check_goal_pos(cx + self.goal_offset * self.ball_offset_sign, cy)

                self.goal_found = True

        # Display the image
        cv2.imshow('Camera view: detect goal', img)
        cv2.waitKey(1)

    # Keeps track of the ball's direction when it is in sight
    def check_ball_pos(self, x):
        obj_x = x - 320

        if (obj_x < 5 and obj_x > -5):
            self.ball_direction = "straight"
        elif (obj_x >= 5):
            self.ball_direction = "right"
        elif (obj_x <= -5):
            self.ball_direction = "left"

    # Keeps track of the shooting position's direction when it is in sight
    def check_shoot_spot_pos(self, x):
        obj_x = x - 320

        if (obj_x < 5 and obj_x > -5):
            self.shoot_spot_direction = "straight"
        elif (obj_x >= 5):
            self.shoot_spot_direction = "right"
        elif (obj_x <= -5):
            self.shoot_spot_direction = "left"

    # Keeps track of the goal's direction when it is in sight
    def check_goal_pos(self,x,y):
        obj_x = x - 320
        if obj_x < -15 and obj_x > -55:
            self.goal_direction = "straight"
        elif obj_x >= -15:
            self.goal_direction = "right"
        elif obj_x <= -55 and obj_x >= 0:
            self.goal_direction = "left"

    # Calibrates the robot's rotation and moves forward to the desired object's coordinates on camera image
    def move_towards_object(self, x, y):
        if(self.state == "FIND_GOAL"):
            self.goal_offset = self.goal_offset - 1

        linear_vel = 0.1
        angular_vel = -0.002 * (x - 320)

        angular_vel = max(min(angular_vel, 0.5), -0.5)

        self.rot.linear.x = linear_vel
        self.rot.angular.z = angular_vel

        self.pub.publish(self.rot)

    def stop(self):
        self.rot.angular.z = 0
        self.rot.linear.x = 0
        self.pub.publish(self.rot)

if __name__ == '__main__':
    # Initialize
    rospy.init_node('take_photo', anonymous=False)
    camera = CaptureObject()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        rospy.spin()

    camera.stop


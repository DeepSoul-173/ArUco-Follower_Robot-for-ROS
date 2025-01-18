#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from time import time

# Camera calibration parameters
camera_matrix = np.array([[640, 0, 320],
                         [0, 480, 240],
                         [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5,), dtype=np.float32)

# Speed control parameters
MAX_LINEAR_SPEED = 0.2
MIN_LINEAR_SPEED = 0.05
MAX_ANGULAR_SPEED = 0.7
MIN_ANGULAR_SPEED = 0.2

class ArUcoFollower:
    def __init__(self):
        self.last_command = None
        self.last_marker_id = None
        self.last_distance = 0.5  # Default middle distance
        self.bridge = CvBridge()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.last_turn_time = None  # Timestamp for last turn command

    def calculate_speed(self, distance):
        """Calculate speed based on distance to marker"""
        if distance < 0.3:
            return MIN_LINEAR_SPEED
        elif distance > 1.0:
            return MAX_LINEAR_SPEED
        else:
            return MIN_LINEAR_SPEED + (MAX_LINEAR_SPEED - MIN_LINEAR_SPEED) * (distance - 0.3) / 0.7

    def send_command(self, command, marker_id, distance):
        """Send velocity commands with dynamic speed control"""
        twist = Twist()
        linear_speed = self.calculate_speed(distance)

        # Handle turning commands for a fixed duration
        if command in ["left", "right"] and (self.last_turn_time is None or time() - self.last_turn_time > 1):
            twist.linear.x = 0.0
            twist.angular.z = MAX_ANGULAR_SPEED if command == "left" else -MAX_ANGULAR_SPEED
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(1)  # Perform the turn for 1 second
            self.last_turn_time = time()
            # Continue forward after turning
            command = "forward"

        # Base movement on marker ID
        if command == "forward" and marker_id in [1, None]:
            twist.linear.x = linear_speed
            twist.angular.z = 0.0
        elif marker_id == 2:  # Move Backward
            twist.linear.x = -linear_speed
            twist.angular.z = 0.0
        elif marker_id == 5:  # Stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.last_command = None
            self.last_marker_id = None

        self.last_command = command
        self.cmd_vel_pub.publish(twist)

    def detect_aruco(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and len(corners) > 0:
            cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)

            for i in range(len(ids)):
                try:
                    ret = aruco.estimatePoseSingleMarkers(corners[i], 0.05, camera_matrix, dist_coeffs)
                    rvec = ret[0][0][0]
                    tvec = ret[1][0][0]

                    cv_image = aruco.drawAxis(cv_image, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                    marker_id = ids[i][0]
                    distance = abs(tvec[2])

                    rospy.loginfo("Marker ID: {} Position: {} Distance: {:.2f}m".format(marker_id, tvec, distance))

                    direction_command = None
                    if marker_id == 3:  # Turn Right
                        direction_command = "right"
                    elif marker_id == 4:  # Turn Left
                        direction_command = "left"
                    elif marker_id == 1:  # Move Forward
                        direction_command = "forward"

                    self.send_command(direction_command, marker_id, distance)

                except Exception as e:
                    rospy.logerr("Error in pose estimation for marker ID {}: {}".format(ids[i][0], str(e)))
                    continue
        else:
            if self.last_marker_id is not None:
                self.send_command("forward", None, self.last_distance)
            else:
                rospy.logwarn("No markers detected and no previous state!")
                self.send_command(None, 5, 0)  # Stop if no markers and no previous state

        return cv_image

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detected_image = self.detect_aruco(cv_image)
            cv2.imshow("ArUco Detection", detected_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User exited")
        except Exception as e:
            rospy.logerr("Error processing image: {}".format(str(e)))

def main():
    rospy.init_node('aruco_detector')
    follower = ArUcoFollower()
    rospy.Subscriber('/camera/image_raw', Image, follower.image_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


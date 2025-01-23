# ArUco-follower_robot
"A ROS-based ArUco marker-following robot leveraging computer vision and feedback control for precise navigation. The robot identifies ArUco markers using OpenCV, integrates real-time position tracking, and executes marker-guided movement. This project demonstrates the synergy of robotics and vision systems."
![Screenshot from 2025-01-22 17-11-32](https://github.com/user-attachments/assets/76017787-a6c2-41b0-83ae-0039fbe8cabd)


# ArUco Cube Detection with ROS and OpenCV

This project demonstrates how to detect an ArUco cube using ROS (Robot Operating System), OpenCV, and the cv_bridge package. The cube consists of multiple ArUco markers on its faces, enabling 6-DOF pose estimation (position and orientation) using the camera feed.

---

## Features
- Detect ArUco markers on a cube.
- Publish the cube's pose as a ROS `PoseStamped` message.
- Visualize the cube's pose in RViz using markers.
- Display the camera feed with detected ArUco markers and their axes in a separate OpenCV window.

---

## Prerequisites
1. **ROS Installation**:
   - Install ROS (Noetic or Melodic based on your OS).
   - Ensure `cv_bridge` is installed.

2. **Dependencies**:
   - OpenCV with ArUco support (`cv2.aruco` module).
   - Python packages: `numpy`, `rospy`, `tf`, `cv_bridge`.

3. **Hardware**:
   - A compatible camera.
   - An ArUco-marked cube (or any setup of ArUco markers).

---

## Steps to Set Up the Project

### 1. Clone the GitHub Repository
```bash
cd ~/catkin_ws/src
git clone https://github.com/DeepSoul-173/ArUco-Follower_Robot-using-ROS.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Hardware Setup
- Connect your camera to the system.
- Place the ArUco cube within the camera's field of view.

### 4. Launch RViz for Visualization
- Open RViz to visualize the cube’s pose:
  ```bash
  rviz
  ```
- Add the following markers in RViz:
  - Set "Fixed Frame" to `camera_color_optical_frame`.
  - Add a `Marker` display, subscribing to `/visualization_marker_real_cube`.

### 5. Run the ArUco Detection Node
```bash
rosrun tortoisebot_firmware opencv_node.py
```

---

## Project Code Walkthrough

### 1. ROS Node Initialization
The `ArUcoDetector` class initializes the ROS node, subscribes to camera topics, and publishes pose information.
```python
rospy.init_node('aruco_detector', anonymous=True)
self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
```

### 2. Camera Info Callback
This function retrieves the camera matrix and distortion coefficients.
```python
def camera_info_callback(self, msg):
    self.camera_matrix = np.array(msg.K).reshape((3, 3))
    self.dist_coeffs = np.array(msg.D)
```

### 3. Image Callback
This function processes the camera feed, detects ArUco markers, estimates pose, and publishes it.
```python
def image_callback(self, msg):
    # Convert ROS Image to OpenCV image
    cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)

    # Pose estimation
    _, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.board, self.camera_matrix, self.dist_coeffs, np.empty(1), np.empty(1))

    # Visualize detection
    cv_image = aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size)
    cv2.imshow("ArUco Detection", cv_image)
    cv2.waitKey(1)
```

### 4. Publishing Pose
The pose is published as a `PoseStamped` message to the `/aruco_cube/pose` topic.
```python
cube_pose_msg = PoseStamped()
cube_pose_msg.header.frame_id = 'camera_color_optical_frame'
cube_pose_msg.header.stamp = rospy.Time.now()
cube_pose_msg.pose.position = Point(x=translation[0], y=translation[1], z=translation[2])
cube_pose_msg.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
self.cube_pose_pub.publish(cube_pose_msg)
```

### 5. RViz Marker
A marker is created and published for visualization in RViz.
```python
def create_marker(self, marker, marker_pub, type=1, target_location=None, color=None, scale=None):
    marker.header.frame_id = "camera_color_optical_frame"
    marker.header.stamp = rospy.Time.now()
    marker.type = type
    marker.pose.position = target_location.pose.position
    marker.pose.orientation = target_location.pose.orientation
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0
    marker_pub.publish(marker)
```

---

## Expected Results
1. A real-time OpenCV window displaying the detected ArUco markers and axes.
2. A `PoseStamped` message published to `/aruco_cube/pose` with the cube's position and orientation.
3. A marker visualization in RViz representing the cube's pose.

---

## Debugging Tips
1. **Camera Calibration**:
   Ensure accurate calibration parameters for the camera matrix and distortion coefficients.

2. **Topic Names**:
   Verify the correct topic names for your camera feed in the launch file or node.

3. **Marker Visibility**:
   Ensure all ArUco markers on the cube are visible for accurate pose estimation.

4. **Dependencies**:
   Verify all ROS packages and Python modules are installed and compatible with your ROS distribution.

---

## Future Enhancements
- Add support for multiple ArUco boards.
- Extend the project for dynamic object tracking.
- Integrate machine learning for improved marker detection in low-light conditions.

---




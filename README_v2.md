**Title**: ArUco Marker_follower

**Description:**
This project showcases an ArUco marker-following robot that utilizes computer vision and ROS (Robot Operating System) to track and follow predefined ArUco markers. ArUco markers are lightweight, binary square patterns that serve as unique identifiers for localization and navigation tasks. This robot is built to demonstrate marker-based navigation, making it a versatile tool for applications such as warehouse automation, object transportation, and robotic assistance.

**Features:**
- **ArUco Marker Detection**: Identifies and tracks ArUco markers in real time using OpenCV’s ArUco module.
- **ROS Integration**: Leverages ROS for modular development, allowing seamless communication between perception, planning, and control nodes.
- **Dynamic Control**: Dynamically adjusts robot motion to maintain a fixed distance and alignment with the detected marker.
- **Hardware Compatibility**: Designed for platforms equipped with a LiDAR, camera, motor driver, and onboard computation such as Raspberry Pi.

**Project Highlights:**
- **Customizable**: Easily adaptable for various marker configurations and environments.
- **Simulation Support**: Includes a Gazebo simulation model for testing and development.
- **Predefined Tasks**: Demonstrates tasks like pick-and-place and path-following using ArUco markers.

**Getting Started:**
1. Clone the repository: `
2. Build the ROS workspace.
3. Launch the robot node:

**Prerequisites:**
- ROS Noetic or Melodic
- OpenCV with ArUco module
- Python or C++ development environment

**Acknowledgments:**
This project builds upon foundational robotics and computer vision principles, leveraging previous work on the AMAB Bot and Tortoise Bot.

For detailed setup instructions and technical documentation, refer to the included `README.md` file in the repository.

    

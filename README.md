
# **Odometry and Robot Control Project**

## **Overview**

This project contains two ROS 2 nodes for a differential drive robot. The first node computes the robot's odometry based on wheel encoder data, while the second node controls the robot's motion to drive a specified distance or turn to a specific angle using proportional control.

### **Included Files:**
1. `odom_compute.py`: Computes and publishes the robot's odometry based on wheel encoder data.
2. `drive_robot.py`: Controls the robot's motion to drive a specific distance and turn to a specific angle.

## **Project Structure**
```plaintext
.
├── odom_compute.py    # Odometry computation node
├── drive_robot.py     # Robot control node
├── README.md          # Project description and instructions (this file)
```

## **File Descriptions**

### **1. Odometry Computation (`odom_compute.py`)**

This script subscribes to wheel encoder data from the `joint_states` topic and computes the robot's odometry. It publishes the computed odometry to the `/my_odom` topic.

#### **Key Classes and Methods**:
- **`State` class**: Holds the robot's position (x, y), orientation (theta), and velocities (vx, vy, omega).
- **`OdomCompute` class**: Main ROS 2 node for odometry computation.
  - `subscriber_callback(data)`: Processes encoder data to compute the robot's new position and velocity.
  - `transition_model(x, u)`: Predicts the robot's new state based on the current state and control inputs (wheel changes).

### **2. Robot Control (`drive_robot.py`)**

This script subscribes to the `/odom` topic for odometry data and controls the robot's motion by publishing velocity commands to the `/cmd_vel` topic. It implements proportional control for driving a specific distance and turning the robot to a specific angle.

#### **Key Classes and Methods**:
- **`TurtleBotControl` class**: Main ROS 2 node for robot control.
  - `drive_distance(goal_distance)`: Drives the robot forward for the specified distance using proportional control.
  - `turn_to(angle_rads, current_odom)`: Turns the robot to the specified angle in radians.
  - `odom_callback(data)`: Receives odometry data and updates the robot's current position and orientation.

## **Running the Project**

### **1. Running the Odometry Node**
To run the `odom_compute.py` script, which computes the robot's odometry:

1. **Clone the repository**:
   ```bash
   git clone https://github.com/your_username/your_repo_name.git
   cd your_repo_name
   ```

2. **Build the workspace**:
   ```bash
   colcon build
   ```

3. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

4. **Run the node**:
   ```bash
   ros2 run your_package_name odom_compute
   ```

### **2. Running the Robot Control Node**
To run the `drive_robot.py` script, which controls the robot's motion:

1. **Run the node**:
   ```bash
   ros2 run your_package_name drive_robot
   ```

2. The robot will automatically drive a specified distance and turn to an angle when called.

## **Dependencies**

Ensure you have the following ROS 2 packages installed:

- `rclpy`: The ROS 2 Python client library.
- `nav_msgs`: For publishing `Odometry` messages.
- `geometry_msgs`: For working with `Twist` and `Quaternion` messages.
- `tf_transformations`: For quaternion-to-euler conversions.

Install them with the following commands:
```bash
sudo apt install ros-<your_ros2_distro>-rclpy
sudo apt install ros-<your_ros2_distro>-nav-msgs
sudo apt install ros-<your_ros2_distro>-geometry-msgs
sudo apt install ros-<your_ros2_distro>-tf-transformations
```

Replace `<your_ros2_distro>` with your ROS 2 distribution (e.g., `foxy`, `galactic`, etc.).

## **License**
This project is licensed under the MIT License. See the `LICENSE` file for more details.

## **Contributors**
- **Khaled Mohamed Ali** - Developer

Feel free to contribute or submit any issues you find!

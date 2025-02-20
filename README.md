## Note to anyone using this
The model saved in this repo was trained incorrectly, I don't remember the specifics but the forward speed max of the car was mixed up with the turning speed max resulting in the car never being able to move forwards faster than 3m/s. This is probably why it swerves all over the place as the reward function was scaled with velocity which is highest when the car is turning at max speed.

# ROS PPO Controller

This project, `ros_ppo_controller`, is a Python package that uses the Proximal Policy Optimization (PPO) algorithm to control a robot in a ROS (Robot Operating System) environment.

## Code Structure

The main code for this project is located in [`ros_ppo_controller/controller.py`](ros_ppo_controller/controller.py). This file contains the `PPOController` class, which is a ROS node that uses a trained PPO model to control a robot's movements based on sensor data.

The `PPOController` class subscribes to LaserScan and Odometry messages, processes this data, and publishes Twist messages to control the robot's linear and angular velocities.

## Building and Running the ROS2 Package with Colcon

To build and run the ROS2 package using Colcon, follow these steps:

1. Create a workspace directory for your ROS2 project if you haven't already. For example, you can create a directory called `ros2_ws`:

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```

2. Clone or copy your ROS2 package into the `src` directory of your workspace.

3. Install the necessary dependencies for your package. You can use the `rosdep` tool to install system dependencies:

    ```bash
    rosdep install --from-paths src --ignore-src --rosdistro <ROS_DISTRO> -y
    ```

    Replace `<ROS_DISTRO>` with the appropriate ROS distribution (e.g., `foxy`, `galactic`, etc.).

4. Build the package using Colcon:

    ```bash
    colcon build
    ```

    This command will build all the packages in your workspace.

5. Source the setup file to add the built packages to your environment:

    ```bash
    source install/setup.bash
    ```

6. Run the ROS2 package using the appropriate launch file or command. For example:

    ```bash
    ros2 launch ros_ppo_controller controller
    ```

The entry point for this package is the `main` function in `controller.py`, which is specified in `setup.py` as follows:

```python
entry_points={
    'console_scripts': [
        'controller = ros_ppo_controller.controller:main'
    ],
},
```

# ACT Robot Simulation Project

This project sets up a basic Gazebo simulation environment with a simple robot model. The robot consists of a chassis, two wheels, and a caster wheel. This README provides step-by-step instructions to install dependencies, build the project, and run the simulation.

## Prerequisites

- **ROS 2 Jazzy** (Ensure ROS 2 Jazzy is installed and set up on your system)
- **Gazebo Harmonic** (Test `gz sim` in your terminal to confirm installation)

If you haven’t installed ROS 2 Jazzy or Gazebo, follow the [ROS 2 installation guide](https://docs.ros.org/en/jazzy/Installation.html) and the [Gazebo installation instructions](https://gazebosim.org/docs/latest/install).

## Project Setup

1. **Clone the Repository**

   Open a terminal and clone this repository into your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> act
   ```

2. **Source the ROS 2 Environment**
`source /opt/ros/jazzy/setup.bash`

3. **Set up Resource Paths**
`export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/act/models`

To make this setting persistent, add it to your ~/.bashrc:
```
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/src/act/models' >> ~/.bashrc
source ~/.bashrc
```

4. **Build the Workspace**
```
cd ~/ros2_ws
colcon build
```
After building, source the workspace:
`source ~/ros2_ws/install/setup.bash`

5. **Launch the Simulation**
`ros2 launch act gazebo_launch.py`

6. **Open 2nd Terminal and Launch the Bridge**
`ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$(echo $HOME)/ros2_ws/src/act/bridge.yaml`

## Current Understanding of World and Model Building with Gazebo. 

When modifying models, worlds, or other source files, follow this cycle to reflect updates in Gazebo:

1. **Build** the workspace with `colcon build`
2. **Source** the environment with `source ~/ros2_ws/install/setup.bash`
3. **Relaunch** with `ros2 launch <package> <launch_file>.py`

In Gazebo, you can adjust model settings interactively, and these modifications can then be translated into `.sdf` files for persistent updates. Given the modular structure of `.sdf` files, you can build individual models and then assemble them into a larger world, making it easy to create custom environments.

### Approach for Building Worlds

1. **Start with Standalone Models**: Create individual models as `.sdf` files in the `models` directory, each with a `model.config`.
2. **Combine Models into a World**: Use `<include><uri>model://model_name</uri></include>` in a world file (e.g., `main_world.sdf`) to reference each model and create a complete environment. (examine living_room.sdf)
3. **Test and Iterate**: Launch and test your world in Gazebo, adjusting as needed to create an interactive simulation.

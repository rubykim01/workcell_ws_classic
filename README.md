# workcell_ws_classic

A ROS 2 workspace for industrial workcell simulation and control, featuring Universal Robots (UR) manipulators 
and Denso robots with MoveIt2 integration and Gazebo Classic simulation.

## Project Structure

```
workcell_ws_classic/
├── src/
│   ├── descriptions/         # Robot models (UR + Denso)
│   ├── link_attacher/        # Link attachment utilities
│   ├── moveit_config/        # Inverse kinematics & motion planning
│   └── simulation/           # Gazebo simulation
├── env/
│   ├── env_host.bash          # Environment setup script - Host
│   └── env_docker.bash        # Docker
└── docker-compose.yml        # Docker Compose configuration
```

## Clone and Setup
1. **Clone the repository**:
   ```bash
   git clone https://github.com/gkim0127/workcell_ws_classic.git
   cd workcell_ws_classic
   ```

2. **Pull the pre-built Docker image**:
   ```bash
   docker pull ghcr.io/gkim0127/workcell_image:251010
   ```
3. **Setup env - Host**:
   ```bash
   source env/env_host.bash
   ```
   
## Run the workspace in Docker
1. **Run Docker Container**:
   ```bash
   xhost +local:root     # allow X11
   docker compose run workcell_simuation
   ```

2. **Setup env - Docker**:
   ```bash
   source env/env_docker.bash
   ```


## Usage

1. **Build and source the workspace**:

   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

   ###### **Clean build if needed**:
   ```bash
   rm -rf build/ install/ log/
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Launch simulation**: 

   ##### 2-1) **Launch without MoveIt2**
   ```bash
   ros2 launch simulation ur_sim_control.launch.py
   ```

   ##### 2-2) **Launch with MoveIt2**:
   ```bash
   ros2 launch simulation ur_sim_moveit.launch.py
   ```

3. **Spawn pick objects**:
   ```bash
   cd ~/workcell_ws_classic
   ./src/descriptions/ur_description/urdf/objects/spawn_all_heating_objects.sh 
   ```

## Dependencies

- ROS 2 (Humble)
- Gazebo Classic
- MoveIt2
- Universal Robots ROS 2 packages
- Denso robot packages
- Link attacher packages
- ros-humble-pick-ik (for inverse kinematics)

*This workspace provides a complete simulation environment for industrial workcell operations with multiple robot types, inverse kinematics, and advanced motion planning capabilities.*

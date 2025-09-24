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
├── build/                    # Build artifacts
├── install/                  # Installed packages
└── log/                      # Build logs
```

## Clone and Setup
1. **Clone the repository**:
   ```bash
   git clone https://github.com/gkim0127/workcell_ws_classic.git
   cd workcell_ws_classic
   ```

2. **Pull the pre-built Docker image**:
   ```bash
   docker pull ghcr.io/gkim0127/workcell_simulation:250924
   ```
3. **Setup env**:
   ```bash
   xhost +local:root
   ```
   ```bash
   sudo nano ~/.bashrc
   ```
   copy and paste:
   ```bash
   # Auto-load workcell env only when you're inside the repo
   _workcell_try_source() {
     local d="$PWD"
     while [ "$d" != "/" ]; do
       if [ -f "$d/env/setup_env.bash" ]; then
         # avoid double-loading per shell
         if [ -z "$_WORKCELL_ENV_SOURCED" ]; then
           source "$d/env/setup_env.bash"
           export _WORKCELL_ENV_SOURCED=1
         fi
         return 0
       fi
       d="$(dirname "$d")"
     done
     return 1
   }
   PROMPT_COMMAND="_workcell_try_source; $PROMPT_COMMAND"

   ```
   
## Run the workspace in Docker
   ```bash
   docker run -it --gpus all --privileged \
     -e DISPLAY=$DISPLAY \
     -e QT_X11_NO_MITSHM=1 \
     -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
     -v "$PWD":/root/workcell_ws_classic \
     --hostname $(hostname) \
     --network host \
     --name {container_name} \
     ghcr.io/gkim0127/workcell_simulation:250924 bash
   ```

   **Replace the following variables:**
   - `{/path/to/your/workcell_ws}`: Path to your local workcell workspace directory
   - `{container_name}`: Desired name for your Docker container

**Inside the continer**:
   ```bash
   echo 'source /root/workcell_ws_classic/env/setup_env.bash' >> ~/.bashrc
   ```


## Usage

1. **Build and source the workspace**:

   ```bash
   colcon build --symlink-install
   ```
   ```bash
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

## Installation

Install required ROS packages:

```bash
sudo apt install ros-humble-pick-ik
```

*This workspace provides a complete simulation environment for industrial workcell operations with multiple robot types, inverse kinematics, and advanced motion planning capabilities.*

#!/bin/bash

# Script to spawn all heating plates and covers
# Usage: ./spawn_all_heating_objects.sh

# Fixed positions for each element 
HP1_X=0.5601
HP1_Y=-0.1129
HP1_Z=0.830

HP2_X=0.5601
HP2_Y=-0.2271
HP2_Z=0.830

HPC2_X=0.491
HPC2_Y=0.150
HPC2_Z=0.830

HPC1_X=0.619
HPC1_Y=0.150
HPC1_Z=0.830

HPC31_X=0.369
HPC31_Y=0.0155
HPC31_Z=0.830

HPC32_X=0.369
HPC32_Y=0.1955
HPC32_Z=0.830


echo "Using fixed positions for each element:"
echo "  heating_plate1: ($HP1_X, $HP1_Y, $HP1_Z)"
echo "  heating_plate2: ($HP2_X, $HP2_Y, $HP2_Z)"
echo "  heating_plate_cover1: ($HPC1_X, $HPC1_Y, $HPC1_Z)"
echo "  heating_plate_cover2: ($HPC2_X, $HPC2_Y, $HPC2_Z)"
echo "  heating_plate_cover3-1: ($HPC31_X, $HPC31_Y, $HPC31_Z)"
echo "  heating_plate_cover3-2: ($HPC32_X, $HPC32_Y, $HPC32_Z)"

echo "Spawning all heating plates and covers"

# Get the current workspace directory
OBJECTS_DIR="/root/workcell_ws_classic/src/Universal_Robots_ROS2_Description/urdf/objects"

echo "Objects directory: $OBJECTS_DIR"

# Check if SDF files exist
if [ ! -f "$OBJECTS_DIR/heating_plate1.sdf" ]; then
    echo "Error: heating_plate1.sdf not found at $OBJECTS_DIR/heating_plate1.sdf"
    exit 1
fi

# Spawn heating plates
echo "Spawning heating plates..."
echo "  - heating_plate1 at ($HP1_X, $HP1_Y, $HP1_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate1.sdf" \
    -entity heating_plate1 \
    -x $HP1_X -y $HP1_Y -z $HP1_Z

echo "  - heating_plate2 at ($HP2_X, $HP2_Y, $HP2_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate2.sdf" \
    -entity heating_plate2 \
    -x $HP2_X -y $HP2_Y -z $HP2_Z

# Spawn heating plate covers
echo "Spawning heating plate covers..."
echo "  - heating_plate_cover1 at ($HPC1_X, $HPC1_Y, $HPC1_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate_cover1.sdf" \
    -entity heating_plate_cover1 \
    -x $HPC1_X -y $HPC1_Y -z $HPC1_Z

echo "  - heating_plate_cover2 at ($HPC2_X, $HPC2_Y, $HPC2_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate_cover2.sdf" \
    -entity heating_plate_cover2 \
    -x $HPC2_X -y $HPC2_Y -z $HPC2_Z

echo "  - heating_plate_cover3-1 at ($HPC31_X, $HPC31_Y, $HPC31_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate_cover3-1.sdf" \
    -entity heating_plate_cover3-1 \
    -x $HPC31_X -y $HPC31_Y -z $HPC31_Z

echo "  - heating_plate_cover3-2 at ($HPC32_X, $HPC32_Y, $HPC32_Z)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate_cover3-2.sdf" \
    -entity heating_plate_cover3-2 \
    -x $HPC32_X -y $HPC32_Y -z $HPC32_Z


echo "All heating objects spawned successfully!"
echo "Objects created:"
echo "  - heating_plate1"
echo "  - heating_plate2"
echo "  - heating_plate_cover1"
echo "  - heating_plate_cover2"
echo "  - heating_plate_cover3-1"
echo "  - heating_plate_cover3-2" 
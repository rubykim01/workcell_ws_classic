#!/bin/bash

# Script to spawn all heating plates and covers for ARF test
# Usage: ./spawn_on_arf.sh

# Fixed positions for each element 
HP1_X=0.150541
HP1_Y=0.789223
HP1_Z=1.088485
HP1_ROLL=-0.007418
HP1_PITCH=-0.000775
HP1_YAW=1.572667

HP2_X=-0.019455
HP2_Y=0.788920
HP2_Z=1.087330
HP2_ROLL=-0.007418
HP2_PITCH=-0.000775
HP2_YAW=1.572667

HPC1_X=0.151205
HPC1_Y=0.837834
HPC1_Z=1.081713
HPC1_ROLL=0.002632
HPC1_PITCH=-0.001700
HPC1_YAW=0.001854

HPC2_X=-0.018893
HPC2_Y=0.837618
HPC2_Z=1.081327
HPC2_ROLL=0.002933
HPC2_PITCH=-0.001481
HPC2_YAW=0.001896

HPC31_X=0.369
HPC31_Y=0.1955
HPC31_Z=0.830

HPC32_X=0.369
HPC32_Y=0.0155
HPC32_Z=0.830


echo "Using fixed positions for each element:"
echo "  heating_plate1: ($HP1_X, $HP1_Y, $HP1_Z) with orientation (roll: $HP1_ROLL, pitch: $HP1_PITCH, yaw: $HP1_YAW)"
echo "  heating_plate2: ($HP2_X, $HP2_Y, $HP2_Z) with orientation (roll: $HP2_ROLL, pitch: $HP2_PITCH, yaw: $HP2_YAW)"
echo "  heating_plate_cover1: ($HPC1_X, $HPC1_Y, $HPC1_Z) with orientation (roll: $HPC1_ROLL, pitch: $HPC1_PITCH, yaw: $HPC1_YAW)"
echo "  heating_plate_cover2: ($HPC2_X, $HPC2_Y, $HPC2_Z) with orientation (roll: $HPC2_ROLL, pitch: $HPC2_PITCH, yaw: $HPC2_YAW)"
echo "  heating_plate_cover3-1: ($HPC31_X, $HPC31_Y, $HPC31_Z)"
echo "  heating_plate_cover3-2: ($HPC32_X, $HPC32_Y, $HPC32_Z)"

echo "Spawning all heating plates and covers for ARF test"

# Get the current workspace directory
OBJECTS_DIR="/root/workcell_ws_classic/src/descriptions/ur_description/urdf/objects"

echo "Objects directory: $OBJECTS_DIR"

# Check if SDF files exist
if [ ! -f "$OBJECTS_DIR/heating_plate1.sdf" ]; then
    echo "Error: heating_plate1.sdf not found at $OBJECTS_DIR/heating_plate1.sdf"
    exit 1
fi

# Spawn heating plates
echo "Spawning heating plates..."
echo "  - heating_plate1 at ($HP1_X, $HP1_Y, $HP1_Z) with orientation (roll: $HP1_ROLL, pitch: $HP1_PITCH, yaw: $HP1_YAW)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate1.sdf" \
    -entity heating_plate1 \
    -x $HP1_X -y $HP1_Y -z $HP1_Z \
    -R $HP1_ROLL -P $HP1_PITCH -Y $HP1_YAW

echo "  - heating_plate2 at ($HP2_X, $HP2_Y, $HP2_Z) with orientation (roll: $HP2_ROLL, pitch: $HP2_PITCH, yaw: $HP2_YAW)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate2.sdf" \
    -entity heating_plate2 \
    -x $HP2_X -y $HP2_Y -z $HP2_Z \
    -R $HP2_ROLL -P $HP2_PITCH -Y $HP2_YAW

# Spawn heating plate covers
echo "Spawning heating plate covers..."
echo "  - heating_plate_cover1 at ($HPC1_X, $HPC1_Y, $HPC1_Z) with orientation (roll: $HPC1_ROLL, pitch: $HPC1_PITCH, yaw: $HPC1_YAW)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate_cover1.sdf" \
    -entity heating_plate_cover1 \
    -x $HPC1_X -y $HPC1_Y -z $HPC1_Z \
    -R $HPC1_ROLL -P $HPC1_PITCH -Y $HPC1_YAW

echo "  - heating_plate_cover2 at ($HPC2_X, $HPC2_Y, $HPC2_Z) with orientation (roll: $HPC2_ROLL, pitch: $HPC2_PITCH, yaw: $HPC2_YAW)"
ros2 run gazebo_ros spawn_entity.py \
    -file "$OBJECTS_DIR/heating_plate_cover2.sdf" \
    -entity heating_plate_cover2 \
    -x $HPC2_X -y $HPC2_Y -z $HPC2_Z \
    -R $HPC2_ROLL -P $HPC2_PITCH -Y $HPC2_YAW

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


echo "All heating objects spawned successfully for ARF test!"
echo "Objects created:"
echo "  - heating_plate1"
echo "  - heating_plate2"
echo "  - heating_plate_cover1"
echo "  - heating_plate_cover2"
echo "  - heating_plate_cover3-1"
echo "  - heating_plate_cover3-2"
#!/bin/bash

# Script to spawn mobile trays on feeder base
# Usage: ./spawn_feeder_trays.sh

# Get the current workspace directory dynamically
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OBJECTS_DIR="$SCRIPT_DIR"

# Locate trolley_positions.yaml (same relative path in source tree and install share)
YAML_PATH="$SCRIPT_DIR/../../config/trolley_positions.yaml"
if [ ! -f "$YAML_PATH" ]; then
    echo "Error: trolley_positions.yaml not found at $YAML_PATH"
    exit 1
fi

# Read feeder pose from YAML so object spawn positions follow the configured feeder
read FEEDER_WORLD_X FEEDER_WORLD_Y FEEDER_WORLD_Z FEEDER_YAW <<< "$(python3 -c "
import yaml
with open('$YAML_PATH') as f:
    d = yaml.safe_load(f)['trolley_positions']['feeder']
print(d['x'], d['y'], d['z'], d['yaw'])
")"
echo "Feeder world pose: x=$FEEDER_WORLD_X y=$FEEDER_WORLD_Y z=$FEEDER_WORLD_Z yaw=$FEEDER_YAW"

# Convert a feeder-local offset (lx ly lz) to world coords, applying feeder yaw rotation.
# Roll/pitch are assumed 0 (they are in trolley_positions.yaml); only yaw affects XY.
world_xyz() {
    awk -v fx="$FEEDER_WORLD_X" -v fy="$FEEDER_WORLD_Y" -v fz="$FEEDER_WORLD_Z" -v yaw="$FEEDER_YAW" \
        -v lx="$1" -v ly="$2" -v lz="$3" \
        'BEGIN {
            c = cos(yaw); s = sin(yaw);
            printf "%.6f %.6f %.6f\n", fx + c*lx - s*ly, fy + s*lx + c*ly, fz + lz;
        }'
}

# Compose feeder yaw with an object's feeder-local yaw to get world yaw
world_yaw() {
    awk -v fy="$FEEDER_YAW" -v ly="$1" 'BEGIN { printf "%.6f\n", fy + ly }'
}

# Sum two scalars (for combining feeder-local offsets before rotation)
sum() {
    awk -v a="$1" -v b="$2" 'BEGIN { printf "%.6f\n", a + b }'
}

# Feeder-local tray offsets (meters)
OFFSET_X=-0.361
OFFSET_Y=-0.9685
OFFSET_H1_Z=0.544
OFFSET_H2_Z=0.434
OFFSET_H3_Z=0.324
OFFSET_H4_Z=0.214
OFFSET_H5_Z=0.104

# Object offsets from tray origin (tray shares feeder yaw, so these are feeder-local too)
COVER_OFFSET_X=0.0
COVER_OFFSET_Y=-0.0315
COVER_1ST_LEVEL_Z=0.0
COVER_2ND_LEVEL_Z=0.018
PLATE_OFFSET_X=0.0
PLATE_OFFSET_Y=-0.0335
PLATE_1ST_LEVEL_Z=0.0
PLATE_2ND_LEVEL_Z=0.0055
COVER3_OFFSET_X_1=-0.1
COVER3_OFFSET_X_2=0.0
COVER3_OFFSET_X_3=0.1
COVER3_OFFSET_Y_1=-0.074
COVER3_OFFSET_Y_2=0.006
COVER3_OFFSET_Z=-0.003

# Tray world positions
read TRAY_H1_X TRAY_H1_Y TRAY_H1_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $OFFSET_H1_Z)"
read TRAY_H2_X TRAY_H2_Y TRAY_H2_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $OFFSET_H2_Z)"
read TRAY_H3_X TRAY_H3_Y TRAY_H3_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $OFFSET_H3_Z)"
read TRAY_H4_X TRAY_H4_Y TRAY_H4_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $OFFSET_H4_Z)"
read TRAY_H5_X TRAY_H5_Y TRAY_H5_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $OFFSET_H5_Z)"

echo "Objects directory: $OBJECTS_DIR"

# Check if SDF files exist
if [ ! -f "$OBJECTS_DIR/mobile_tray_h1.sdf" ]; then
    echo "Error: mobile_tray_h1.sdf not found at $OBJECTS_DIR/mobile_tray_h1.sdf"
    exit 1
fi

# Spawn all trays in parallel (each tray inherits feeder yaw)
echo "Spawning trays in parallel..."
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/mobile_tray_h1.sdf" -entity mobile_tray_h1 -x $TRAY_H1_X -y $TRAY_H1_Y -z $TRAY_H1_Z -R 0 -P 0 -Y $FEEDER_YAW &
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/mobile_tray_h2.sdf" -entity mobile_tray_h2 -x $TRAY_H2_X -y $TRAY_H2_Y -z $TRAY_H2_Z -R 0 -P 0 -Y $FEEDER_YAW &
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/mobile_tray_h3.sdf" -entity mobile_tray_h3 -x $TRAY_H3_X -y $TRAY_H3_Y -z $TRAY_H3_Z -R 0 -P 0 -Y $FEEDER_YAW &
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/mobile_tray_h4.sdf" -entity mobile_tray_h4 -x $TRAY_H4_X -y $TRAY_H4_Y -z $TRAY_H4_Z -R 0 -P 0 -Y $FEEDER_YAW &
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/mobile_tray_h5.sdf" -entity mobile_tray_h5 -x $TRAY_H5_X -y $TRAY_H5_Y -z $TRAY_H5_Z -R 0 -P 0 -Y $FEEDER_YAW &
wait
echo "All trays spawned."

# Cover positions for Tray 1 and Tray 4 (combine in feeder-local frame, then rotate)
read COVER_T1_1ST_X COVER_T1_1ST_Y COVER_T1_1ST_Z <<< "$(world_xyz \
    $(sum $OFFSET_X $COVER_OFFSET_X) $(sum $OFFSET_Y $COVER_OFFSET_Y) $(sum $OFFSET_H1_Z $COVER_1ST_LEVEL_Z))"
read COVER_T1_2ND_X COVER_T1_2ND_Y COVER_T1_2ND_Z <<< "$(world_xyz \
    $(sum $OFFSET_X $COVER_OFFSET_X) $(sum $OFFSET_Y $COVER_OFFSET_Y) $(sum $OFFSET_H1_Z $COVER_2ND_LEVEL_Z))"
read COVER_T4_1ST_X COVER_T4_1ST_Y COVER_T4_1ST_Z <<< "$(world_xyz \
    $(sum $OFFSET_X $COVER_OFFSET_X) $(sum $OFFSET_Y $COVER_OFFSET_Y) $(sum $OFFSET_H4_Z $COVER_1ST_LEVEL_Z))"
read COVER_T4_2ND_X COVER_T4_2ND_Y COVER_T4_2ND_Z <<< "$(world_xyz \
    $(sum $OFFSET_X $COVER_OFFSET_X) $(sum $OFFSET_Y $COVER_OFFSET_Y) $(sum $OFFSET_H4_Z $COVER_2ND_LEVEL_Z))"

# Plate positions for Tray 2 and Tray 5
read Plate_T2_X_1ST Plate_T2_Y_1ST Plate_T2_Z_1ST <<< "$(world_xyz \
    $(sum $OFFSET_X $PLATE_OFFSET_X) $(sum $OFFSET_Y $PLATE_OFFSET_Y) $(sum $OFFSET_H2_Z $PLATE_1ST_LEVEL_Z))"
read Plate_T2_X_2ND Plate_T2_Y_2ND Plate_T2_Z_2ND <<< "$(world_xyz \
    $(sum $OFFSET_X $PLATE_OFFSET_X) $(sum $OFFSET_Y $PLATE_OFFSET_Y) $(sum $OFFSET_H2_Z $PLATE_2ND_LEVEL_Z))"
read Plate_T5_X_1ST Plate_T5_Y_1ST Plate_T5_Z_1ST <<< "$(world_xyz \
    $(sum $OFFSET_X $PLATE_OFFSET_X) $(sum $OFFSET_Y $PLATE_OFFSET_Y) $(sum $OFFSET_H5_Z $PLATE_1ST_LEVEL_Z))"
read Plate_T5_X_2ND Plate_T5_Y_2ND Plate_T5_Z_2ND <<< "$(world_xyz \
    $(sum $OFFSET_X $PLATE_OFFSET_X) $(sum $OFFSET_Y $PLATE_OFFSET_Y) $(sum $OFFSET_H5_Z $PLATE_2ND_LEVEL_Z))"

# Cover3 positions for Tray 3
read COVER3_T3_X_1 COVER3_T3_Y_1 COVER3_T3_Z_1 <<< "$(world_xyz \
    $(sum $OFFSET_X $COVER3_OFFSET_X_1) $(sum $OFFSET_Y $COVER3_OFFSET_Y_1) $(sum $OFFSET_H3_Z $COVER3_OFFSET_Z))"
read COVER3_T3_X_2 COVER3_T3_Y_2 COVER3_T3_Z_2 <<< "$(world_xyz \
    $(sum $OFFSET_X $COVER3_OFFSET_X_1) $(sum $OFFSET_Y $COVER3_OFFSET_Y_2) $(sum $OFFSET_H3_Z $COVER3_OFFSET_Z))"
read COVER3_T3_X_3 COVER3_T3_Y_3 COVER3_T3_Z_3 <<< "$(world_xyz \
    $(sum $OFFSET_X $COVER3_OFFSET_X_2) $(sum $OFFSET_Y $COVER3_OFFSET_Y_1) $(sum $OFFSET_H3_Z $COVER3_OFFSET_Z))"
read COVER3_T3_X_4 COVER3_T3_Y_4 COVER3_T3_Z_4 <<< "$(world_xyz \
    $(sum $OFFSET_X $COVER3_OFFSET_X_2) $(sum $OFFSET_Y $COVER3_OFFSET_Y_2) $(sum $OFFSET_H3_Z $COVER3_OFFSET_Z))"
read COVER3_T3_X_5 COVER3_T3_Y_5 COVER3_T3_Z_5 <<< "$(world_xyz \
    $(sum $OFFSET_X $COVER3_OFFSET_X_3) $(sum $OFFSET_Y $COVER3_OFFSET_Y_1) $(sum $OFFSET_H3_Z $COVER3_OFFSET_Z))"
read COVER3_T3_X_6 COVER3_T3_Y_6 COVER3_T3_Z_6 <<< "$(world_xyz \
    $(sum $OFFSET_X $COVER3_OFFSET_X_3) $(sum $OFFSET_Y $COVER3_OFFSET_Y_2) $(sum $OFFSET_H3_Z $COVER3_OFFSET_Z))"

# Cover rotation: feeder-local -90° about Z, composed with feeder yaw for world yaw
COVER_YAW=$(world_yaw -1.5708)
COVER3_YAW=$(world_yaw -1.5708)
# Plates have no local yaw; they inherit the feeder yaw in world frame
PLATE_YAW=$FEEDER_YAW

# Spawn all covers and plates in parallel
echo "Spawning covers and plates in parallel..."
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/heating_plate_cover2.sdf" -entity heating_plate_cover_t1_1st -x $COVER_T1_1ST_X -y $COVER_T1_1ST_Y -z $COVER_T1_1ST_Z -R 0 -P 0 -Y $COVER_YAW &
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/heating_plate_cover1.sdf" -entity heating_plate_cover_t1_2nd -x $COVER_T1_2ND_X -y $COVER_T1_2ND_Y -z $COVER_T1_2ND_Z -R 0 -P 0 -Y $COVER_YAW &
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/heating_plate_cover1.sdf" -entity heating_plate_cover_t4_1st -x $COVER_T4_1ST_X -y $COVER_T4_1ST_Y -z $COVER_T4_1ST_Z -R 0 -P 0 -Y $COVER_YAW &
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/heating_plate_cover2.sdf" -entity heating_plate_cover_t4_2nd -x $COVER_T4_2ND_X -y $COVER_T4_2ND_Y -z $COVER_T4_2ND_Z -R 0 -P 0 -Y $COVER_YAW &
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/heating_plate1.sdf" -entity heating_plate_t2_1st -x $Plate_T2_X_1ST -y $Plate_T2_Y_1ST -z $Plate_T2_Z_1ST -R 0 -P 0 -Y $PLATE_YAW &
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/heating_plate2.sdf" -entity heating_plate_t2_2nd -x $Plate_T2_X_2ND -y $Plate_T2_Y_2ND -z $Plate_T2_Z_2ND -R 0 -P 0 -Y $PLATE_YAW &
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/heating_plate1.sdf" -entity heating_plate_t5_1st -x $Plate_T5_X_1ST -y $Plate_T5_Y_1ST -z $Plate_T5_Z_1ST -R 0 -P 0 -Y $PLATE_YAW &
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/heating_plate2.sdf" -entity heating_plate_t5_2nd -x $Plate_T5_X_2ND -y $Plate_T5_Y_2ND -z $Plate_T5_Z_2ND -R 0 -P 0 -Y $PLATE_YAW &
wait
echo "All covers and plates spawned."

# Spawn cover3 on Tray 3 in parallel
echo "Spawning cover3 on Tray 3 in parallel..."
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/heating_plate_cover3-1.sdf" -entity heating_plate_cover3_1_t3_1 -x $COVER3_T3_X_1 -y $COVER3_T3_Y_1 -z $COVER3_T3_Z_1 -R 0 -P 0 -Y $COVER3_YAW &
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/heating_plate_cover3-2.sdf" -entity heating_plate_cover3_2_t3_2 -x $COVER3_T3_X_2 -y $COVER3_T3_Y_2 -z $COVER3_T3_Z_2 -R 0 -P 0 -Y $COVER3_YAW &
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/heating_plate_cover3-1.sdf" -entity heating_plate_cover3_1_t3_3 -x $COVER3_T3_X_3 -y $COVER3_T3_Y_3 -z $COVER3_T3_Z_3 -R 0 -P 0 -Y $COVER3_YAW &
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/heating_plate_cover3-2.sdf" -entity heating_plate_cover3_2_t3_4 -x $COVER3_T3_X_4 -y $COVER3_T3_Y_4 -z $COVER3_T3_Z_4 -R 0 -P 0 -Y $COVER3_YAW &
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/heating_plate_cover3-1.sdf" -entity heating_plate_cover3_1_t3_5 -x $COVER3_T3_X_5 -y $COVER3_T3_Y_5 -z $COVER3_T3_Z_5 -R 0 -P 0 -Y $COVER3_YAW &
ros2 run gazebo_ros spawn_entity.py -file "$OBJECTS_DIR/heating_plate_cover3-2.sdf" -entity heating_plate_cover3_2_t3_6 -x $COVER3_T3_X_6 -y $COVER3_T3_Y_6 -z $COVER3_T3_Z_6 -R 0 -P 0 -Y $COVER3_YAW &
wait

echo "All mobile trays and covers spawned successfully!"

# Wait for physics to settle before attaching
echo "Waiting for physics to settle before attaching objects to trays..."
sleep 2

attach_to_tray() {
    local obj="$1"
    local tray="$2"
    echo "Attaching $obj to $tray..."
    ros2 service call /ATTACHLINK linkattacher_msgs/srv/AttachLink \
        "{model1_name: '$tray', link1_name: 'link', model2_name: '$obj', link2_name: 'link'}" \
        > /dev/null
}

# Tray 1 covers
attach_to_tray heating_plate_cover_t1_1st mobile_tray_h1
attach_to_tray heating_plate_cover_t1_2nd mobile_tray_h1
# Tray 2 plates
attach_to_tray heating_plate_t2_1st mobile_tray_h2
attach_to_tray heating_plate_t2_2nd mobile_tray_h2
# Tray 3 cover3
attach_to_tray heating_plate_cover3_1_t3_1 mobile_tray_h3
attach_to_tray heating_plate_cover3_2_t3_2 mobile_tray_h3
attach_to_tray heating_plate_cover3_1_t3_3 mobile_tray_h3
attach_to_tray heating_plate_cover3_2_t3_4 mobile_tray_h3
attach_to_tray heating_plate_cover3_1_t3_5 mobile_tray_h3
attach_to_tray heating_plate_cover3_2_t3_6 mobile_tray_h3
# Tray 4 covers
attach_to_tray heating_plate_cover_t4_1st mobile_tray_h4
attach_to_tray heating_plate_cover_t4_2nd mobile_tray_h4
# Tray 5 plates
attach_to_tray heating_plate_t5_1st mobile_tray_h5
attach_to_tray heating_plate_t5_2nd mobile_tray_h5

echo "All heater objects attached to trays."

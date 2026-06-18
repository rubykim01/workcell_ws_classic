#!/bin/bash

# Script to spawn mobile trays E1-E6 on feeder base (for electronics)
# Usage: ./spawn_feeder_elec.sh [CYCLE]
#   CYCLE 1 (default): trays 1,2,3 are loaded with objects; trays 4,5,6 are bare.
#   CYCLE 2          : trays 4,5,6 are loaded with objects; trays 1,2,3 are bare.
# All 6 trays are always spawned either way; only the object set differs.

CYCLE="${1:-1}"
if [ "$CYCLE" != "1" ] && [ "$CYCLE" != "2" ]; then
    echo "Error: CYCLE must be 1 or 2 (got '$CYCLE')"
    exit 1
fi
echo "Elec cycle: $CYCLE"

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
world_xyz() {
    awk -v fx="$FEEDER_WORLD_X" -v fy="$FEEDER_WORLD_Y" -v fz="$FEEDER_WORLD_Z" -v yaw="$FEEDER_YAW" \
        -v lx="$1" -v ly="$2" -v lz="$3" \
        'BEGIN {
            c = cos(yaw); s = sin(yaw);
            printf "%.6f %.6f %.6f\n", fx + c*lx - s*ly, fy + s*lx + c*ly, fz + lz;
        }'
}

# Sum two scalars (for combining feeder-local offsets before rotation)
sum() {
    awk -v a="$1" -v b="$2" 'BEGIN { printf "%.6f\n", a + b }'
}

# Feeder-local tray offsets (meters, converted from mm)
OFFSET_X=-0.361
OFFSET_Y=-0.9685
OFFSET_E1_Z=0.544
OFFSET_E2_Z=0.434
OFFSET_E3_Z=0.324
OFFSET_E4_Z=0.214
OFFSET_E5_Z=0.104
OFFSET_E6_Z=-0.006

# Tray world positions
read TRAY_E1_X TRAY_E1_Y TRAY_E1_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $OFFSET_E1_Z)"
read TRAY_E2_X TRAY_E2_Y TRAY_E2_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $OFFSET_E2_Z)"
read TRAY_E3_X TRAY_E3_Y TRAY_E3_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $OFFSET_E3_Z)"
read TRAY_E4_X TRAY_E4_Y TRAY_E4_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $OFFSET_E4_Z)"
read TRAY_E5_X TRAY_E5_Y TRAY_E5_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $OFFSET_E5_Z)"
read TRAY_E6_X TRAY_E6_Y TRAY_E6_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $OFFSET_E6_Z)"

echo "Objects directory: $OBJECTS_DIR"

# Check if SDF files exist
if [ ! -f "$OBJECTS_DIR/mobile_tray_e1.sdf" ]; then
    echo "Error: mobile_tray_e1.sdf not found at $OBJECTS_DIR/mobile_tray_e1.sdf"
    exit 1
fi

# Collect every object into one batch_spawn.py call (built up across the phases
# below, including the TB_JOTN loops, spawned once at the end) instead of
# launching a process per object. Args per object: FILE ENTITY X Y Z R P Y.
SPAWN_ARGS=()
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_e1.sdf" mobile_tray1 $TRAY_E1_X $TRAY_E1_Y $TRAY_E1_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_e2.sdf" mobile_tray2 $TRAY_E2_X $TRAY_E2_Y $TRAY_E2_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_e3.sdf" mobile_tray3 $TRAY_E3_X $TRAY_E3_Y $TRAY_E3_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_e4.sdf" mobile_tray4 $TRAY_E4_X $TRAY_E4_Y $TRAY_E4_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_e5.sdf" mobile_tray5 $TRAY_E5_X $TRAY_E5_Y $TRAY_E5_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_e6.sdf" mobile_tray6 $TRAY_E6_X $TRAY_E6_Y $TRAY_E6_Z 0 0 $FEEDER_YAW)

# Electrical components offsets from tray origin (in meters, converted from mm)
MCCB_OFFSET_X=-0.0885
MCCB_OFFSET_Y=0.0145
MCCB_OFFSET_Z=-0.003

PDU1_OFFSET_X=0.0345
PDU1_OFFSET_Y=0.031
PDU1_OFFSET_Z=-0.003

PDU2_OFFSET_X=0.0345
PDU2_OFFSET_Y=-0.017
PDU2_OFFSET_Z=-0.003

NOISE_FILTER_OFFSET_X=-0.0907
NOISE_FILTER_OFFSET_Y=-0.066
NOISE_FILTER_OFFSET_Z=0.01

PLUG_SOCKET_OFFSET_X=0.0335
PLUG_SOCKET_OFFSET_Y=-0.0735
PLUG_SOCKET_OFFSET_Z=0.003

BUSBAR_OFFSET_X=0.126
BUSBAR_OFFSET_Y=-0.032
BUSBAR_OFFSET_Z=-0.003

# E2 & E5 tray components offsets
SINGLE_MC1_OFFSET_X=-0.1136
SINGLE_MC1_OFFSET_Y=0.0149
SINGLE_MC1_OFFSET_Z=-0.003

SINGLE_MC2_OFFSET_X=-0.0416
SINGLE_MC2_OFFSET_Y=0.0149
SINGLE_MC2_OFFSET_Z=-0.003

SINGLE_MC3_OFFSET_X=-0.0776
SINGLE_MC3_OFFSET_Y=-0.0723
SINGLE_MC3_OFFSET_Z=-0.003

SMPS_OFFSET_X=0.0797
SMPS_OFFSET_Y=-0.0245
SMPS_OFFSET_Z=0.015

# Component world positions: combine tray offset + component offset in feeder-local
# frame, then rotate into world. Writes a pair of helpers first so each entry is one line.
world_from_tray() {
    # $1..$3 = tray-local offset (OFFSET_X, OFFSET_Y, OFFSET_Ei_Z)
    # $4..$6 = component offset from tray origin
    world_xyz "$(sum $1 $4)" "$(sum $2 $5)" "$(sum $3 $6)"
}

# E1 tray components
read MCCB_E1_X MCCB_E1_Y MCCB_E1_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E1_Z $MCCB_OFFSET_X $MCCB_OFFSET_Y $MCCB_OFFSET_Z)"
read PDU1_E1_X PDU1_E1_Y PDU1_E1_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E1_Z $PDU1_OFFSET_X $PDU1_OFFSET_Y $PDU1_OFFSET_Z)"
read PDU2_E1_X PDU2_E1_Y PDU2_E1_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E1_Z $PDU2_OFFSET_X $PDU2_OFFSET_Y $PDU2_OFFSET_Z)"
read NOISE_FILTER_E1_X NOISE_FILTER_E1_Y NOISE_FILTER_E1_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E1_Z $NOISE_FILTER_OFFSET_X $NOISE_FILTER_OFFSET_Y $NOISE_FILTER_OFFSET_Z)"
read PLUG_SOCKET_E1_X PLUG_SOCKET_E1_Y PLUG_SOCKET_E1_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E1_Z $PLUG_SOCKET_OFFSET_X $PLUG_SOCKET_OFFSET_Y $PLUG_SOCKET_OFFSET_Z)"
read BUSBAR_E1_X BUSBAR_E1_Y BUSBAR_E1_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E1_Z $BUSBAR_OFFSET_X $BUSBAR_OFFSET_Y $BUSBAR_OFFSET_Z)"

# E4 tray components
read MCCB_E4_X MCCB_E4_Y MCCB_E4_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E4_Z $MCCB_OFFSET_X $MCCB_OFFSET_Y $MCCB_OFFSET_Z)"
read PDU1_E4_X PDU1_E4_Y PDU1_E4_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E4_Z $PDU1_OFFSET_X $PDU1_OFFSET_Y $PDU1_OFFSET_Z)"
read PDU2_E4_X PDU2_E4_Y PDU2_E4_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E4_Z $PDU2_OFFSET_X $PDU2_OFFSET_Y $PDU2_OFFSET_Z)"
read NOISE_FILTER_E4_X NOISE_FILTER_E4_Y NOISE_FILTER_E4_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E4_Z $NOISE_FILTER_OFFSET_X $NOISE_FILTER_OFFSET_Y $NOISE_FILTER_OFFSET_Z)"
read PLUG_SOCKET_E4_X PLUG_SOCKET_E4_Y PLUG_SOCKET_E4_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E4_Z $PLUG_SOCKET_OFFSET_X $PLUG_SOCKET_OFFSET_Y $PLUG_SOCKET_OFFSET_Z)"
read BUSBAR_E4_X BUSBAR_E4_Y BUSBAR_E4_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E4_Z $BUSBAR_OFFSET_X $BUSBAR_OFFSET_Y $BUSBAR_OFFSET_Z)"

# E2 tray components
read SINGLE_MC1_E2_X SINGLE_MC1_E2_Y SINGLE_MC1_E2_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E2_Z $SINGLE_MC1_OFFSET_X $SINGLE_MC1_OFFSET_Y $SINGLE_MC1_OFFSET_Z)"
read SINGLE_MC2_E2_X SINGLE_MC2_E2_Y SINGLE_MC2_E2_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E2_Z $SINGLE_MC2_OFFSET_X $SINGLE_MC2_OFFSET_Y $SINGLE_MC2_OFFSET_Z)"
read SINGLE_MC3_E2_X SINGLE_MC3_E2_Y SINGLE_MC3_E2_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E2_Z $SINGLE_MC3_OFFSET_X $SINGLE_MC3_OFFSET_Y $SINGLE_MC3_OFFSET_Z)"
read SMPS_E2_X SMPS_E2_Y SMPS_E2_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E2_Z $SMPS_OFFSET_X $SMPS_OFFSET_Y $SMPS_OFFSET_Z)"

# E5 tray components
read SINGLE_MC1_E5_X SINGLE_MC1_E5_Y SINGLE_MC1_E5_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E5_Z $SINGLE_MC1_OFFSET_X $SINGLE_MC1_OFFSET_Y $SINGLE_MC1_OFFSET_Z)"
read SINGLE_MC2_E5_X SINGLE_MC2_E5_Y SINGLE_MC2_E5_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E5_Z $SINGLE_MC2_OFFSET_X $SINGLE_MC2_OFFSET_Y $SINGLE_MC2_OFFSET_Z)"
read SINGLE_MC3_E5_X SINGLE_MC3_E5_Y SINGLE_MC3_E5_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E5_Z $SINGLE_MC3_OFFSET_X $SINGLE_MC3_OFFSET_Y $SINGLE_MC3_OFFSET_Z)"
read SMPS_E5_X SMPS_E5_Y SMPS_E5_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E5_Z $SMPS_OFFSET_X $SMPS_OFFSET_Y $SMPS_OFFSET_Z)"

# Components inherit feeder yaw so they sit correctly on rotated trays.
# Only the selected cycle's trays are loaded: cycle 1 loads trays 1,2,3 and
# cycle 2 loads trays 4,5,6. Unloaded trays remain spawned but bare.
if [ "$CYCLE" = "1" ]; then
    # E1 tray components
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mccb_abe_32b_30a.sdf" mccb_abe_32b_30a $MCCB_E1_X $MCCB_E1_Y $MCCB_E1_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/pdu_sps25_m66xm4.sdf" pdu_sps25_m66xm4_1 $PDU1_E1_X $PDU1_E1_Y $PDU1_E1_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/pdu_sps25_m66xm4.sdf" pdu_sps25_m66xm4_2 $PDU2_E1_X $PDU2_E1_Y $PDU2_E1_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/noise_filter_rms_2030_din.sdf" noise_filter_rms_2030_din $NOISE_FILTER_E1_X $NOISE_FILTER_E1_Y $NOISE_FILTER_E1_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/plug_socket_drc_220v_16a.sdf" plug_socket_drc_220v_16a $PLUG_SOCKET_E1_X $PLUG_SOCKET_E1_Y $PLUG_SOCKET_E1_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/busbar_6p.sdf" busbar_6p $BUSBAR_E1_X $BUSBAR_E1_Y $BUSBAR_E1_Z 0 0 $FEEDER_YAW)
    # E2 tray components
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" single_mc_gmc_1 $SINGLE_MC1_E2_X $SINGLE_MC1_E2_Y $SINGLE_MC1_E2_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" single_mc_gmc_2 $SINGLE_MC2_E2_X $SINGLE_MC2_E2_Y $SINGLE_MC2_E2_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" single_mc_gmc_3 $SINGLE_MC3_E2_X $SINGLE_MC3_E2_Y $SINGLE_MC3_E2_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/smps_wdr_120_24v.sdf" smps_wdr_120_24v $SMPS_E2_X $SMPS_E2_Y $SMPS_E2_Z 0 0 $FEEDER_YAW)
else
    # E4 tray components
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mccb_abe_32b_30a.sdf" mccb_abe_32b_30a $MCCB_E4_X $MCCB_E4_Y $MCCB_E4_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/pdu_sps25_m66xm4.sdf" pdu_sps25_m66xm4_1 $PDU1_E4_X $PDU1_E4_Y $PDU1_E4_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/pdu_sps25_m66xm4.sdf" pdu_sps25_m66xm4_2 $PDU2_E4_X $PDU2_E4_Y $PDU2_E4_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/noise_filter_rms_2030_din.sdf" noise_filter_rms_2030_din $NOISE_FILTER_E4_X $NOISE_FILTER_E4_Y $NOISE_FILTER_E4_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/plug_socket_drc_220v_16a.sdf" plug_socket_drc_220v_16a $PLUG_SOCKET_E4_X $PLUG_SOCKET_E4_Y $PLUG_SOCKET_E4_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/busbar_6p.sdf" busbar_6p $BUSBAR_E4_X $BUSBAR_E4_Y $BUSBAR_E4_Z 0 0 $FEEDER_YAW)
    # E5 tray components
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" single_mc_gmc_1 $SINGLE_MC1_E5_X $SINGLE_MC1_E5_Y $SINGLE_MC1_E5_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" single_mc_gmc_2 $SINGLE_MC2_E5_X $SINGLE_MC2_E5_Y $SINGLE_MC2_E5_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" single_mc_gmc_3 $SINGLE_MC3_E5_X $SINGLE_MC3_E5_Y $SINGLE_MC3_E5_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/smps_wdr_120_24v.sdf" smps_wdr_120_24v $SMPS_E5_X $SMPS_E5_Y $SMPS_E5_Z 0 0 $FEEDER_YAW)
fi

# TB_JOTN-15A grid for E3 and E6 trays
TB_OFFSET_Z=0.002

# X positions (38.5mm spacing)
TB_X1=-0.13475
TB_X2=-0.09625
TB_X3=-0.05775
TB_X4=-0.01925
TB_X5=0.01925
TB_X6=0.05775
TB_X7=0.09625
TB_X8=0.13475

# Y positions (75mm spacing)
TB_Y1=-0.0675
TB_Y2=0.0075

# TB_JOTN-15A grid: E3 belongs to cycle 1, E6 to cycle 2 (16 units each).
if [ "$CYCLE" = "1" ]; then
    echo ""
    echo "Spawning TB_JOTN-15A grid on E3 tray (2x8 = 16 units)..."
    # Row 1 (Y = -67.5mm)
    for i in 1 2 3 4 5 6 7 8; do
        eval "TB_X=\$TB_X$i"
        read TB_POS_X TB_POS_Y TB_POS_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E3_Z $TB_X $TB_Y1 $TB_OFFSET_Z)"
        SPAWN_ARGS+=(--obj "$OBJECTS_DIR/tb_jotn_15a.sdf" tb_jotn_15a_${i}_1 $TB_POS_X $TB_POS_Y $TB_POS_Z 0 0 $FEEDER_YAW)
    done
    # Row 2 (Y = 7.5mm)
    for i in 1 2 3 4 5 6 7 8; do
        eval "TB_X=\$TB_X$i"
        read TB_POS_X TB_POS_Y TB_POS_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E3_Z $TB_X $TB_Y2 $TB_OFFSET_Z)"
        SPAWN_ARGS+=(--obj "$OBJECTS_DIR/tb_jotn_15a.sdf" tb_jotn_15a_${i}_2 $TB_POS_X $TB_POS_Y $TB_POS_Z 0 0 $FEEDER_YAW)
    done
else
    echo ""
    echo "Spawning TB_JOTN-15A grid on E6 tray (2x8 = 16 units)..."
    # Row 1 (Y = -67.5mm)
    for i in 1 2 3 4 5 6 7 8; do
        eval "TB_X=\$TB_X$i"
        read TB_POS_X TB_POS_Y TB_POS_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E6_Z $TB_X $TB_Y1 $TB_OFFSET_Z)"
        SPAWN_ARGS+=(--obj "$OBJECTS_DIR/tb_jotn_15a.sdf" tb_jotn_15a_${i}_1 $TB_POS_X $TB_POS_Y $TB_POS_Z 0 0 $FEEDER_YAW)
    done
    # Row 2 (Y = 7.5mm)
    for i in 1 2 3 4 5 6 7 8; do
        eval "TB_X=\$TB_X$i"
        read TB_POS_X TB_POS_Y TB_POS_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $OFFSET_E6_Z $TB_X $TB_Y2 $TB_OFFSET_Z)"
        SPAWN_ARGS+=(--obj "$OBJECTS_DIR/tb_jotn_15a.sdf" tb_jotn_15a_${i}_2 $TB_POS_X $TB_POS_Y $TB_POS_Z 0 0 $FEEDER_YAW)
    done
fi

# Spawn all 58 objects from a single process
echo "Spawning all elec objects (single batch)..."
python3 "$OBJECTS_DIR/batch_spawn.py" "${SPAWN_ARGS[@]}"

echo ""
echo "All E trays and electrical components spawned successfully!"

# Objects have gravity disabled, so they don't fall after spawn; a brief pause
# just lets the new models register before the link_attacher references them.
sleep 0.3

# Queue all attachments, then run them from one batch_attach.py process instead
# of a separate `ros2 service call` per object.
ATTACH_ARGS=()
attach_to_tray() {  # $1 = object, $2 = tray (both attach via their 'link')
    ATTACH_ARGS+=(--attach "$2" link "$1" link)
}

if [ "$CYCLE" = "1" ]; then
    # E1 tray components
    attach_to_tray mccb_abe_32b_30a mobile_tray1
    attach_to_tray pdu_sps25_m66xm4_1 mobile_tray1
    attach_to_tray pdu_sps25_m66xm4_2 mobile_tray1
    attach_to_tray noise_filter_rms_2030_din mobile_tray1
    attach_to_tray plug_socket_drc_220v_16a mobile_tray1
    attach_to_tray busbar_6p mobile_tray1
    # E2 tray components
    attach_to_tray single_mc_gmc_1 mobile_tray2
    attach_to_tray single_mc_gmc_2 mobile_tray2
    attach_to_tray single_mc_gmc_3 mobile_tray2
    attach_to_tray smps_wdr_120_24v mobile_tray2
    # E3 terminal blocks
    for i in 1 2 3 4 5 6 7 8; do
        attach_to_tray tb_jotn_15a_${i}_1 mobile_tray3
        attach_to_tray tb_jotn_15a_${i}_2 mobile_tray3
    done
else
    # E4 tray components
    attach_to_tray mccb_abe_32b_30a mobile_tray4
    attach_to_tray pdu_sps25_m66xm4_1 mobile_tray4
    attach_to_tray pdu_sps25_m66xm4_2 mobile_tray4
    attach_to_tray noise_filter_rms_2030_din mobile_tray4
    attach_to_tray plug_socket_drc_220v_16a mobile_tray4
    attach_to_tray busbar_6p mobile_tray4
    # E5 tray components
    attach_to_tray single_mc_gmc_1 mobile_tray5
    attach_to_tray single_mc_gmc_2 mobile_tray5
    attach_to_tray single_mc_gmc_3 mobile_tray5
    attach_to_tray smps_wdr_120_24v mobile_tray5
    # E6 terminal blocks
    for i in 1 2 3 4 5 6 7 8; do
        attach_to_tray tb_jotn_15a_${i}_1 mobile_tray6
        attach_to_tray tb_jotn_15a_${i}_2 mobile_tray6
    done
fi

echo "Attaching all elec objects to trays (single batch)..."
python3 "$OBJECTS_DIR/batch_attach.py" "${ATTACH_ARGS[@]}"
echo "All elec objects attached to trays."

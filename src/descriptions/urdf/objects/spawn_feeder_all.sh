#!/bin/bash

# Script to spawn all 7 loaded trays on the feeder base in one shot.
# Usage: ./spawn_feeder_all.sh
#
# Stacked from the top slot down, each tray carries its own components:
#   slot 1  heater tray 1 (mobile_tray1) - plate covers
#   slot 2  heater tray 2 (mobile_tray2) - heating plates
#   slot 3  heater tray 3 (mobile_tray3) - cover3 pieces
#   slot 4  elec  tray 1 (mobile_tray4) - mccb / pdu / filter / socket / busbar
#   slot 5  elec  tray 2 (mobile_tray5) - single MCs / smps
#   slot 6  elec  tray 3 (mobile_tray6) - tb_jotn terminal-block grid
#   slot 7  kiro  tray   (mobile_tray7) - relay parts
#
# Unlike the per-type scripts (which reuse mobile_tray1..N and overlapping slots),
# this one gives every tray a unique name (mobile_tray1..7) and its own Z slot so
# all seven coexist in the world at once.

echo "Spawning all feeder trays (heater x3, elec x3, kiro x1)"

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

# Combine a tray-local offset with the tray's own feeder-local offset, then rotate into world.
world_from_tray() {
    # $1..$3 = tray feeder-local offset, $4..$6 = component offset from tray origin
    world_xyz "$(sum $1 $4)" "$(sum $2 $5)" "$(sum $3 $6)"
}

# Feeder-local tray XY (shared by every slot) and per-slot Z (0.139 m spacing).
OFFSET_X=-0.361
OFFSET_Y=-0.9685
SLOT1_Z=0.495
SLOT2_Z=0.356
SLOT3_Z=0.217
SLOT4_Z=0.078
SLOT5_Z=-0.061
SLOT6_Z=-0.200
SLOT7_Z=-0.339

# Tray world positions
read TRAY1_X TRAY1_Y TRAY1_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $SLOT1_Z)"
read TRAY2_X TRAY2_Y TRAY2_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $SLOT2_Z)"
read TRAY3_X TRAY3_Y TRAY3_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $SLOT3_Z)"
read TRAY4_X TRAY4_Y TRAY4_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $SLOT4_Z)"
read TRAY5_X TRAY5_Y TRAY5_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $SLOT5_Z)"
read TRAY6_X TRAY6_Y TRAY6_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $SLOT6_Z)"
read TRAY7_X TRAY7_Y TRAY7_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $SLOT7_Z)"

echo "Objects directory: $OBJECTS_DIR"

# Sanity check one SDF per type
for f in mobile_tray_h1.sdf mobile_tray_e1.sdf mobile_tray_r1.sdf; do
    if [ ! -f "$OBJECTS_DIR/$f" ]; then
        echo "Error: $f not found at $OBJECTS_DIR/$f"
        exit 1
    fi
done

# Collect every object into one batch_spawn.py call (built up below, spawned once
# at the end). Args per object: FILE ENTITY X Y Z R P Y.
SPAWN_ARGS=()

# --- Trays (one per slot, unique names mobile_tray1..7) ---
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_h1.sdf" mobile_tray1 $TRAY1_X $TRAY1_Y $TRAY1_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_h2.sdf" mobile_tray2 $TRAY2_X $TRAY2_Y $TRAY2_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_h3.sdf" mobile_tray3 $TRAY3_X $TRAY3_Y $TRAY3_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_e1.sdf" mobile_tray4 $TRAY4_X $TRAY4_Y $TRAY4_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_e2.sdf" mobile_tray5 $TRAY5_X $TRAY5_Y $TRAY5_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_e3.sdf" mobile_tray6 $TRAY6_X $TRAY6_Y $TRAY6_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_r1.sdf" mobile_tray7 $TRAY7_X $TRAY7_Y $TRAY7_Z 0 0 $FEEDER_YAW)

# =====================================================================
# Heater tray 1 (slot 1): plate covers
# =====================================================================
COVER_OFFSET_X=0.0
COVER_OFFSET_Y=-0.0315
COVER_1ST_LEVEL_Z=0.0
COVER_2ND_LEVEL_Z=0.018
COVER_YAW=$(world_yaw -1.5708)

read COVER_1ST_X COVER_1ST_Y COVER_1ST_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT1_Z $COVER_OFFSET_X $COVER_OFFSET_Y $COVER_1ST_LEVEL_Z)"
read COVER_2ND_X COVER_2ND_Y COVER_2ND_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT1_Z $COVER_OFFSET_X $COVER_OFFSET_Y $COVER_2ND_LEVEL_Z)"
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/heating_plate_cover2.sdf" heating_plate_cover_1st $COVER_1ST_X $COVER_1ST_Y $COVER_1ST_Z 0 0 $COVER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/heating_plate_cover1.sdf" heating_plate_cover_2nd $COVER_2ND_X $COVER_2ND_Y $COVER_2ND_Z 0 0 $COVER_YAW)

# =====================================================================
# Heater tray 2 (slot 2): heating plates
# =====================================================================
PLATE_OFFSET_X=0.0
PLATE_OFFSET_Y=-0.0335
PLATE_1ST_LEVEL_Z=0.0
PLATE_2ND_LEVEL_Z=0.0055
PLATE_YAW=$FEEDER_YAW

read PLATE_1ST_X PLATE_1ST_Y PLATE_1ST_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT2_Z $PLATE_OFFSET_X $PLATE_OFFSET_Y $PLATE_1ST_LEVEL_Z)"
read PLATE_2ND_X PLATE_2ND_Y PLATE_2ND_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT2_Z $PLATE_OFFSET_X $PLATE_OFFSET_Y $PLATE_2ND_LEVEL_Z)"
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/heating_plate1.sdf" heating_plate_1st $PLATE_1ST_X $PLATE_1ST_Y $PLATE_1ST_Z 0 0 $PLATE_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/heating_plate2.sdf" heating_plate_2nd $PLATE_2ND_X $PLATE_2ND_Y $PLATE_2ND_Z 0 0 $PLATE_YAW)

# =====================================================================
# Heater tray 3 (slot 3): single cover3 piece (was a 2x3 grid; keep just the
# X_2/Y_2 slot piece, named plainly heating_plate_cover3)
# =====================================================================
COVER3_OFFSET_X=0.0
COVER3_OFFSET_Y=0.006
COVER3_OFFSET_Z=-0.003
COVER3_YAW=$(world_yaw -1.5708)

read COVER3_X COVER3_Y COVER3_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT3_Z $COVER3_OFFSET_X $COVER3_OFFSET_Y $COVER3_OFFSET_Z)"
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/heating_plate_cover3-2.sdf" heating_plate_cover3 $COVER3_X $COVER3_Y $COVER3_Z 0 0 $COVER3_YAW)

# =====================================================================
# Elec tray 1 (slot 4): mccb / pdu / noise filter / plug socket / busbar
# =====================================================================
MCCB_OFFSET_X=-0.0885;  MCCB_OFFSET_Y=0.0145;   MCCB_OFFSET_Z=-0.003
PDU1_OFFSET_X=0.0345;   PDU1_OFFSET_Y=0.031;    PDU1_OFFSET_Z=-0.003
PDU2_OFFSET_X=0.0345;   PDU2_OFFSET_Y=-0.017;   PDU2_OFFSET_Z=-0.003
NOISE_FILTER_OFFSET_X=-0.0907; NOISE_FILTER_OFFSET_Y=-0.066; NOISE_FILTER_OFFSET_Z=0.01
PLUG_SOCKET_OFFSET_X=0.0335;   PLUG_SOCKET_OFFSET_Y=-0.0735; PLUG_SOCKET_OFFSET_Z=0.003
BUSBAR_OFFSET_X=0.126;  BUSBAR_OFFSET_Y=-0.032; BUSBAR_OFFSET_Z=-0.003

read MCCB_X MCCB_Y MCCB_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT4_Z $MCCB_OFFSET_X $MCCB_OFFSET_Y $MCCB_OFFSET_Z)"
read PDU1_X PDU1_Y PDU1_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT4_Z $PDU1_OFFSET_X $PDU1_OFFSET_Y $PDU1_OFFSET_Z)"
read PDU2_X PDU2_Y PDU2_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT4_Z $PDU2_OFFSET_X $PDU2_OFFSET_Y $PDU2_OFFSET_Z)"
read NF_X NF_Y NF_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT4_Z $NOISE_FILTER_OFFSET_X $NOISE_FILTER_OFFSET_Y $NOISE_FILTER_OFFSET_Z)"
read PS_X PS_Y PS_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT4_Z $PLUG_SOCKET_OFFSET_X $PLUG_SOCKET_OFFSET_Y $PLUG_SOCKET_OFFSET_Z)"
read BB_X BB_Y BB_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT4_Z $BUSBAR_OFFSET_X $BUSBAR_OFFSET_Y $BUSBAR_OFFSET_Z)"
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mccb_abe_32b_30a.sdf" mccb_abe_32b_30a $MCCB_X $MCCB_Y $MCCB_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/pdu_sps25_m66xm4.sdf" pdu_sps25_m66xm4_1 $PDU1_X $PDU1_Y $PDU1_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/pdu_sps25_m66xm4.sdf" pdu_sps25_m66xm4_2 $PDU2_X $PDU2_Y $PDU2_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/noise_filter_rms_2030_din.sdf" noise_filter_rms_2030_din $NF_X $NF_Y $NF_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/plug_socket_drc_220v_16a.sdf" plug_socket_drc_220v_16a $PS_X $PS_Y $PS_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/busbar_6p.sdf" busbar_6p $BB_X $BB_Y $BB_Z 0 0 $FEEDER_YAW)

# =====================================================================
# Elec tray 2 (slot 5): single MCs / smps
# =====================================================================
SINGLE_MC1_OFFSET_X=-0.1136; SINGLE_MC1_OFFSET_Y=0.0149;  SINGLE_MC1_OFFSET_Z=-0.003
SINGLE_MC2_OFFSET_X=-0.0416; SINGLE_MC2_OFFSET_Y=0.0149;  SINGLE_MC2_OFFSET_Z=-0.003
SINGLE_MC3_OFFSET_X=-0.0776; SINGLE_MC3_OFFSET_Y=-0.0723; SINGLE_MC3_OFFSET_Z=-0.003
SMPS_OFFSET_X=0.0797;   SMPS_OFFSET_Y=-0.0245;  SMPS_OFFSET_Z=0.015

read MC1_X MC1_Y MC1_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT5_Z $SINGLE_MC1_OFFSET_X $SINGLE_MC1_OFFSET_Y $SINGLE_MC1_OFFSET_Z)"
read MC2_X MC2_Y MC2_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT5_Z $SINGLE_MC2_OFFSET_X $SINGLE_MC2_OFFSET_Y $SINGLE_MC2_OFFSET_Z)"
read MC3_X MC3_Y MC3_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT5_Z $SINGLE_MC3_OFFSET_X $SINGLE_MC3_OFFSET_Y $SINGLE_MC3_OFFSET_Z)"
read SMPS_X SMPS_Y SMPS_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT5_Z $SMPS_OFFSET_X $SMPS_OFFSET_Y $SMPS_OFFSET_Z)"
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" single_mc_gmc_1 $MC1_X $MC1_Y $MC1_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" single_mc_gmc_2 $MC2_X $MC2_Y $MC2_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" single_mc_gmc_3 $MC3_X $MC3_Y $MC3_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/smps_wdr_120_24v.sdf" smps_wdr_120_24v $SMPS_X $SMPS_Y $SMPS_Z 0 0 $FEEDER_YAW)

# =====================================================================
# Elec tray 3 (slot 6): single tb_jotn terminal block (was a 2x8 grid; keep just
# the X5/Y2 slot unit, named plainly tb_jotn_15a)
# =====================================================================
TB_OFFSET_Z=0.002
TB_X=0.01925
TB_Y=0.0075

read TB_POS_X TB_POS_Y TB_POS_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT6_Z $TB_X $TB_Y $TB_OFFSET_Z)"
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/tb_jotn_15a.sdf" tb_jotn_15a $TB_POS_X $TB_POS_Y $TB_POS_Z 0 0 $FEEDER_YAW)

# =====================================================================
# Kiro tray (slot 7): relay parts
# =====================================================================
RELAY01_OFFSET_X=-0.01625; RELAY01_OFFSET_Y=0.0215;   RELAY01_OFFSET_Z=0.0439
RELAY02_OFFSET_X=-0.1065;  RELAY02_OFFSET_Y=0.0145;   RELAY02_OFFSET_Z=0.0521
RELAY02_2_OFFSET_X=-0.1065; RELAY02_2_OFFSET_Y=-0.0655; RELAY02_2_OFFSET_Z=0.0531
RELAY03_OFFSET_X=-0.01625; RELAY03_OFFSET_Y=-0.05925; RELAY03_OFFSET_Z=0.030
RELAY04_OFFSET_X=0.089;    RELAY04_OFFSET_Y=-0.066;   RELAY04_OFFSET_Z=0.026
RELAY04_2_OFFSET_X=0.089;  RELAY04_2_OFFSET_Y=0.004;  RELAY04_2_OFFSET_Z=0.026

read R01_X R01_Y R01_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT7_Z $RELAY01_OFFSET_X $RELAY01_OFFSET_Y $RELAY01_OFFSET_Z)"
read R02_X R02_Y R02_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT7_Z $RELAY02_OFFSET_X $RELAY02_OFFSET_Y $RELAY02_OFFSET_Z)"
read R022_X R022_Y R022_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT7_Z $RELAY02_2_OFFSET_X $RELAY02_2_OFFSET_Y $RELAY02_2_OFFSET_Z)"
read R03_X R03_Y R03_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT7_Z $RELAY03_OFFSET_X $RELAY03_OFFSET_Y $RELAY03_OFFSET_Z)"
read R04_X R04_Y R04_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT7_Z $RELAY04_OFFSET_X $RELAY04_OFFSET_Y $RELAY04_OFFSET_Z)"
read R042_X R042_Y R042_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT7_Z $RELAY04_2_OFFSET_X $RELAY04_2_OFFSET_Y $RELAY04_2_OFFSET_Z)"
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_01.sdf" relay_part_01 $R01_X $R01_Y $R01_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_02.sdf" relay_part_02 $R02_X $R02_Y $R02_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_02_2.sdf" relay_part_02_2 $R022_X $R022_Y $R022_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_03.sdf" relay_part_03 $R03_X $R03_Y $R03_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_04.sdf" relay_part_04 $R04_X $R04_Y $R04_Z 0 0 $FEEDER_YAW)
SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_04.sdf" relay_part_04_2 $R042_X $R042_Y $R042_Z 0 0 $FEEDER_YAW)

# Spawn everything from a single process
echo "Spawning all feeder objects (single batch)..."
python3 "$OBJECTS_DIR/batch_spawn.py" "${SPAWN_ARGS[@]}"

echo "All 7 trays and components spawned successfully!"

# Objects have gravity disabled, so they don't fall after spawn; a brief pause
# just lets the new models register before the link_attacher references them.
sleep 0.3

# Queue all attachments, then run them from one batch_attach.py process.
ATTACH_ARGS=()
attach_to_tray() {  # $1 = object, $2 = tray (both attach via their 'link')
    ATTACH_ARGS+=(--attach "$2" link "$1" link)
}

# Heater tray 1 covers
attach_to_tray heating_plate_cover_1st mobile_tray1
attach_to_tray heating_plate_cover_2nd mobile_tray1
# Heater tray 2 plates
attach_to_tray heating_plate_1st mobile_tray2
attach_to_tray heating_plate_2nd mobile_tray2
# Heater tray 3 cover3 piece
attach_to_tray heating_plate_cover3 mobile_tray3
# Elec tray 1 components
attach_to_tray mccb_abe_32b_30a mobile_tray4
attach_to_tray pdu_sps25_m66xm4_1 mobile_tray4
attach_to_tray pdu_sps25_m66xm4_2 mobile_tray4
attach_to_tray noise_filter_rms_2030_din mobile_tray4
attach_to_tray plug_socket_drc_220v_16a mobile_tray4
attach_to_tray busbar_6p mobile_tray4
# Elec tray 2 components
attach_to_tray single_mc_gmc_1 mobile_tray5
attach_to_tray single_mc_gmc_2 mobile_tray5
attach_to_tray single_mc_gmc_3 mobile_tray5
attach_to_tray smps_wdr_120_24v mobile_tray5
# Elec tray 3 terminal block
attach_to_tray tb_jotn_15a mobile_tray6
# Kiro tray relay parts
attach_to_tray relay_part_01 mobile_tray7
attach_to_tray relay_part_02 mobile_tray7
attach_to_tray relay_part_02_2 mobile_tray7
attach_to_tray relay_part_03 mobile_tray7
attach_to_tray relay_part_04 mobile_tray7
attach_to_tray relay_part_04_2 mobile_tray7

echo "Attaching all objects to their trays (single batch)..."
python3 "$OBJECTS_DIR/batch_attach.py" "${ATTACH_ARGS[@]}"
echo "All objects attached to trays."

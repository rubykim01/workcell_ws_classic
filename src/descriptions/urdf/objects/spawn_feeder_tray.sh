#!/bin/bash

# Spawn ONE feeder tray (with its own components) onto ONE floor/slot of the feeder.
# Usage: ./spawn_feeder_tray.sh <tray_type> <floor>
#   tray_type: h1 h2 h3 e1 e2 e3 r1   (see table below)
#   floor:     1..7                   (slot from the top of the feeder down)
#
#   h1  heater covers      (plate covers)
#   h2  heater plates      (heating plates)
#   h3  heater cover3      (cover3 piece)
#   e1  elec mccb/pdu      (mccb / pdu / filter / socket / busbar)
#   e2  elec mc/smps       (single MCs / smps)
#   e3  elec terminal      (tb_jotn terminal block)
#   r1  kiro relays        (relay parts)
#
# Every entity is named with a "_f<floor>" suffix (tray -> mobile_tray_f<floor>,
# components -> <name>_f<floor>) so the SAME tray type can be spawned on several
# floors at once without name collisions. Geometry mirrors spawn_feeder_all.sh;
# only the slot Z changes per floor, and component offsets are relative to the
# tray so they follow whichever floor is chosen.

set -e

TRAY_TYPE="$1"
FLOOR="$2"

if [ -z "$TRAY_TYPE" ] || [ -z "$FLOOR" ]; then
    echo "Usage: $0 <tray_type: h1|h2|h3|e1|e2|e3|r1> <floor: 1..7>"
    exit 1
fi
if ! [[ "$FLOOR" =~ ^[1-7]$ ]]; then
    echo "Error: floor must be 1..7 (got '$FLOOR')"
    exit 1
fi

SUFFIX="_f${FLOOR}"

echo "Spawning feeder tray '$TRAY_TYPE' on floor $FLOOR"

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
case "$FLOOR" in
    1) SLOT_Z=0.495 ;;
    2) SLOT_Z=0.356 ;;
    3) SLOT_Z=0.217 ;;
    4) SLOT_Z=0.078 ;;
    5) SLOT_Z=-0.061 ;;
    6) SLOT_Z=-0.200 ;;
    7) SLOT_Z=-0.339 ;;
esac

# Tray world position for this floor
read TRAY_X TRAY_Y TRAY_Z <<< "$(world_xyz $OFFSET_X $OFFSET_Y $SLOT_Z)"

echo "Objects directory: $OBJECTS_DIR"

# Collect this tray's objects into one batch_spawn.py call, and its attachments
# into one batch_attach.py call. Args per object: FILE ENTITY X Y Z R P Y.
SPAWN_ARGS=()
ATTACH_ARGS=()

TRAY_ENTITY="mobile_tray${SUFFIX}"
attach_to_tray() {  # $1 = object entity; attaches its 'link' to the tray 'link'
    ATTACH_ARGS+=(--attach "$TRAY_ENTITY" link "$1" link)
}

case "$TRAY_TYPE" in
  # ===================================================================
  h1) # Heater tray: plate covers
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_h1.sdf" $TRAY_ENTITY $TRAY_X $TRAY_Y $TRAY_Z 0 0 $FEEDER_YAW)
    COVER_OFFSET_X=0.0
    COVER_OFFSET_Y=-0.0315
    COVER_1ST_LEVEL_Z=0.0
    COVER_2ND_LEVEL_Z=0.018
    COVER_YAW=$(world_yaw -1.5708)
    read C1_X C1_Y C1_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $COVER_OFFSET_X $COVER_OFFSET_Y $COVER_1ST_LEVEL_Z)"
    read C2_X C2_Y C2_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $COVER_OFFSET_X $COVER_OFFSET_Y $COVER_2ND_LEVEL_Z)"
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/heating_plate_cover2.sdf" heating_plate_cover_1st${SUFFIX} $C1_X $C1_Y $C1_Z 0 0 $COVER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/heating_plate_cover1.sdf" heating_plate_cover_2nd${SUFFIX} $C2_X $C2_Y $C2_Z 0 0 $COVER_YAW)
    attach_to_tray heating_plate_cover_1st${SUFFIX}
    attach_to_tray heating_plate_cover_2nd${SUFFIX}
    ;;
  # ===================================================================
  h2) # Heater tray: heating plates
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_h2.sdf" $TRAY_ENTITY $TRAY_X $TRAY_Y $TRAY_Z 0 0 $FEEDER_YAW)
    PLATE_OFFSET_X=0.0
    PLATE_OFFSET_Y=-0.0335
    PLATE_1ST_LEVEL_Z=0.0
    PLATE_2ND_LEVEL_Z=0.0055
    read P1_X P1_Y P1_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $PLATE_OFFSET_X $PLATE_OFFSET_Y $PLATE_1ST_LEVEL_Z)"
    read P2_X P2_Y P2_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $PLATE_OFFSET_X $PLATE_OFFSET_Y $PLATE_2ND_LEVEL_Z)"
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/heating_plate1.sdf" heating_plate_1st${SUFFIX} $P1_X $P1_Y $P1_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/heating_plate2.sdf" heating_plate_2nd${SUFFIX} $P2_X $P2_Y $P2_Z 0 0 $FEEDER_YAW)
    attach_to_tray heating_plate_1st${SUFFIX}
    attach_to_tray heating_plate_2nd${SUFFIX}
    ;;
  # ===================================================================
  h3) # Heater tray: single cover3 piece
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_h3.sdf" $TRAY_ENTITY $TRAY_X $TRAY_Y $TRAY_Z 0 0 $FEEDER_YAW)
    COVER3_OFFSET_X=0.0
    COVER3_OFFSET_Y=0.006
    COVER3_OFFSET_Z=-0.003
    COVER3_YAW=$(world_yaw -1.5708)
    read C3_X C3_Y C3_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $COVER3_OFFSET_X $COVER3_OFFSET_Y $COVER3_OFFSET_Z)"
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/heating_plate_cover3-2.sdf" heating_plate_cover3${SUFFIX} $C3_X $C3_Y $C3_Z 0 0 $COVER3_YAW)
    attach_to_tray heating_plate_cover3${SUFFIX}
    ;;
  # ===================================================================
  e1) # Elec tray: mccb / pdu / noise filter / plug socket / busbar
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_e1.sdf" $TRAY_ENTITY $TRAY_X $TRAY_Y $TRAY_Z 0 0 $FEEDER_YAW)
    MCCB_OFFSET_X=-0.0885;  MCCB_OFFSET_Y=0.0145;   MCCB_OFFSET_Z=-0.003
    PDU1_OFFSET_X=0.0345;   PDU1_OFFSET_Y=0.031;    PDU1_OFFSET_Z=-0.003
    PDU2_OFFSET_X=0.0345;   PDU2_OFFSET_Y=-0.017;   PDU2_OFFSET_Z=-0.003
    NOISE_FILTER_OFFSET_X=-0.0907; NOISE_FILTER_OFFSET_Y=-0.066; NOISE_FILTER_OFFSET_Z=0.01
    PLUG_SOCKET_OFFSET_X=0.0335;   PLUG_SOCKET_OFFSET_Y=-0.0735; PLUG_SOCKET_OFFSET_Z=0.003
    BUSBAR_OFFSET_X=0.126;  BUSBAR_OFFSET_Y=-0.032; BUSBAR_OFFSET_Z=-0.003
    read MCCB_X MCCB_Y MCCB_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $MCCB_OFFSET_X $MCCB_OFFSET_Y $MCCB_OFFSET_Z)"
    read PDU1_X PDU1_Y PDU1_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $PDU1_OFFSET_X $PDU1_OFFSET_Y $PDU1_OFFSET_Z)"
    read PDU2_X PDU2_Y PDU2_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $PDU2_OFFSET_X $PDU2_OFFSET_Y $PDU2_OFFSET_Z)"
    read NF_X NF_Y NF_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $NOISE_FILTER_OFFSET_X $NOISE_FILTER_OFFSET_Y $NOISE_FILTER_OFFSET_Z)"
    read PS_X PS_Y PS_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $PLUG_SOCKET_OFFSET_X $PLUG_SOCKET_OFFSET_Y $PLUG_SOCKET_OFFSET_Z)"
    read BB_X BB_Y BB_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $BUSBAR_OFFSET_X $BUSBAR_OFFSET_Y $BUSBAR_OFFSET_Z)"
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mccb_abe_32b_30a.sdf" mccb_abe_32b_30a${SUFFIX} $MCCB_X $MCCB_Y $MCCB_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/pdu_sps25_m66xm4.sdf" pdu_sps25_m66xm4_1${SUFFIX} $PDU1_X $PDU1_Y $PDU1_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/pdu_sps25_m66xm4.sdf" pdu_sps25_m66xm4_2${SUFFIX} $PDU2_X $PDU2_Y $PDU2_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/noise_filter_rms_2030_din.sdf" noise_filter_rms_2030_din${SUFFIX} $NF_X $NF_Y $NF_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/plug_socket_drc_220v_16a.sdf" plug_socket_drc_220v_16a${SUFFIX} $PS_X $PS_Y $PS_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/busbar_6p.sdf" busbar_6p${SUFFIX} $BB_X $BB_Y $BB_Z 0 0 $FEEDER_YAW)
    for o in mccb_abe_32b_30a pdu_sps25_m66xm4_1 pdu_sps25_m66xm4_2 noise_filter_rms_2030_din plug_socket_drc_220v_16a busbar_6p; do
        attach_to_tray ${o}${SUFFIX}
    done
    ;;
  # ===================================================================
  e2) # Elec tray: single MCs / smps
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_e2.sdf" $TRAY_ENTITY $TRAY_X $TRAY_Y $TRAY_Z 0 0 $FEEDER_YAW)
    SINGLE_MC1_OFFSET_X=-0.1136; SINGLE_MC1_OFFSET_Y=0.0149;  SINGLE_MC1_OFFSET_Z=-0.003
    SINGLE_MC2_OFFSET_X=-0.0416; SINGLE_MC2_OFFSET_Y=0.0149;  SINGLE_MC2_OFFSET_Z=-0.003
    SINGLE_MC3_OFFSET_X=-0.0776; SINGLE_MC3_OFFSET_Y=-0.0723; SINGLE_MC3_OFFSET_Z=-0.003
    SMPS_OFFSET_X=0.0797;   SMPS_OFFSET_Y=-0.0245;  SMPS_OFFSET_Z=0.015
    read MC1_X MC1_Y MC1_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $SINGLE_MC1_OFFSET_X $SINGLE_MC1_OFFSET_Y $SINGLE_MC1_OFFSET_Z)"
    read MC2_X MC2_Y MC2_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $SINGLE_MC2_OFFSET_X $SINGLE_MC2_OFFSET_Y $SINGLE_MC2_OFFSET_Z)"
    read MC3_X MC3_Y MC3_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $SINGLE_MC3_OFFSET_X $SINGLE_MC3_OFFSET_Y $SINGLE_MC3_OFFSET_Z)"
    read SMPS_X SMPS_Y SMPS_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $SMPS_OFFSET_X $SMPS_OFFSET_Y $SMPS_OFFSET_Z)"
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" single_mc_gmc_1${SUFFIX} $MC1_X $MC1_Y $MC1_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" single_mc_gmc_2${SUFFIX} $MC2_X $MC2_Y $MC2_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/single_mc_gmc_30p2_ac220v.sdf" single_mc_gmc_3${SUFFIX} $MC3_X $MC3_Y $MC3_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/smps_wdr_120_24v.sdf" smps_wdr_120_24v${SUFFIX} $SMPS_X $SMPS_Y $SMPS_Z 0 0 $FEEDER_YAW)
    for o in single_mc_gmc_1 single_mc_gmc_2 single_mc_gmc_3 smps_wdr_120_24v; do
        attach_to_tray ${o}${SUFFIX}
    done
    ;;
  # ===================================================================
  e3) # Elec tray: single tb_jotn terminal block
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_e3.sdf" $TRAY_ENTITY $TRAY_X $TRAY_Y $TRAY_Z 0 0 $FEEDER_YAW)
    TB_OFFSET_Z=0.002
    TB_X=0.01925
    TB_Y=0.0075
    read TB_POS_X TB_POS_Y TB_POS_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $TB_X $TB_Y $TB_OFFSET_Z)"
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/tb_jotn_15a.sdf" tb_jotn_15a${SUFFIX} $TB_POS_X $TB_POS_Y $TB_POS_Z 0 0 $FEEDER_YAW)
    attach_to_tray tb_jotn_15a${SUFFIX}
    ;;
  # ===================================================================
  r1) # Kiro tray: relay parts
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/mobile_tray_r1.sdf" $TRAY_ENTITY $TRAY_X $TRAY_Y $TRAY_Z 0 0 $FEEDER_YAW)
    RELAY01_OFFSET_X=-0.01625; RELAY01_OFFSET_Y=0.0215;   RELAY01_OFFSET_Z=0.0439
    RELAY02_OFFSET_X=-0.1065;  RELAY02_OFFSET_Y=0.0145;   RELAY02_OFFSET_Z=0.0521
    RELAY02_2_OFFSET_X=-0.1065; RELAY02_2_OFFSET_Y=-0.0655; RELAY02_2_OFFSET_Z=0.0531
    RELAY03_OFFSET_X=-0.01625; RELAY03_OFFSET_Y=-0.05925; RELAY03_OFFSET_Z=0.030
    RELAY04_OFFSET_X=0.089;    RELAY04_OFFSET_Y=-0.066;   RELAY04_OFFSET_Z=0.026
    RELAY04_2_OFFSET_X=0.089;  RELAY04_2_OFFSET_Y=0.004;  RELAY04_2_OFFSET_Z=0.026
    read R01_X R01_Y R01_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $RELAY01_OFFSET_X $RELAY01_OFFSET_Y $RELAY01_OFFSET_Z)"
    read R02_X R02_Y R02_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $RELAY02_OFFSET_X $RELAY02_OFFSET_Y $RELAY02_OFFSET_Z)"
    read R022_X R022_Y R022_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $RELAY02_2_OFFSET_X $RELAY02_2_OFFSET_Y $RELAY02_2_OFFSET_Z)"
    read R03_X R03_Y R03_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $RELAY03_OFFSET_X $RELAY03_OFFSET_Y $RELAY03_OFFSET_Z)"
    read R04_X R04_Y R04_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $RELAY04_OFFSET_X $RELAY04_OFFSET_Y $RELAY04_OFFSET_Z)"
    read R042_X R042_Y R042_Z <<< "$(world_from_tray $OFFSET_X $OFFSET_Y $SLOT_Z $RELAY04_2_OFFSET_X $RELAY04_2_OFFSET_Y $RELAY04_2_OFFSET_Z)"
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_01.sdf" relay_part_01${SUFFIX} $R01_X $R01_Y $R01_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_02.sdf" relay_part_02${SUFFIX} $R02_X $R02_Y $R02_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_02_2.sdf" relay_part_02_2${SUFFIX} $R022_X $R022_Y $R022_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_03.sdf" relay_part_03${SUFFIX} $R03_X $R03_Y $R03_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_04.sdf" relay_part_04${SUFFIX} $R04_X $R04_Y $R04_Z 0 0 $FEEDER_YAW)
    SPAWN_ARGS+=(--obj "$OBJECTS_DIR/relay_part_04.sdf" relay_part_04_2${SUFFIX} $R042_X $R042_Y $R042_Z 0 0 $FEEDER_YAW)
    for o in relay_part_01 relay_part_02 relay_part_02_2 relay_part_03 relay_part_04 relay_part_04_2; do
        attach_to_tray ${o}${SUFFIX}
    done
    ;;
  *)
    echo "Error: unknown tray_type '$TRAY_TYPE' (expected h1|h2|h3|e1|e2|e3|r1)"
    exit 1
    ;;
esac

echo "Spawning tray objects (single batch)..."
python3 "$OBJECTS_DIR/batch_spawn.py" "${SPAWN_ARGS[@]}"

# Objects have gravity disabled, so they don't fall after spawn; a brief pause
# just lets the new models register before the link_attacher references them.
sleep 0.3

echo "Attaching objects to tray (single batch)..."
python3 "$OBJECTS_DIR/batch_attach.py" "${ATTACH_ARGS[@]}"

echo "Tray '$TRAY_TYPE' spawned on floor $FLOOR and attached."

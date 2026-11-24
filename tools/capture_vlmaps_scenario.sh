#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 2 ]; then
    echo "Uso:"
    echo "  $0 <objeto> <condicion>"
    echo
    echo "Ejemplo:"
    echo "  $0 boxes I0"
    exit 1
fi

OBJECT="$1"
CONDITION="$2"
SCENARIO="${OBJECT}_${CONDITION}"

INDOOR="$HOME/indoor"
DATA_ROOT="$HOME/vlmaps_data_benchmark"
OUTPUT_DIR="$DATA_ROOT/$SCENARIO"

POSITIONS_FILE="$INDOOR/evaluation/config/vlmaps_mapping_positions.yaml"

SCENE_RESULT="$INDOOR/evaluation/benchmark_clipseg_results/$OBJECT/$CONDITION/scene/warehouse_${OBJECT}_${CONDITION}.usd"
SCENE_SOURCE="$INDOOR/src/isaac_indoor/isaac_description/isaac_description/warehouse_${OBJECT}_${CONDITION}.usd"

if [ -f "$SCENE_RESULT" ]; then
    SCENE="$SCENE_RESULT"
elif [ -f "$SCENE_SOURCE" ]; then
    SCENE="$SCENE_SOURCE"
else
    echo "ERROR: no existe escena para $SCENARIO"
    exit 1
fi

if [ -e "$OUTPUT_DIR" ]; then
    echo "ERROR: ya existe:"
    echo "$OUTPUT_DIR"
    echo "No se sobrescribe un dataset existente."
    exit 1
fi

echo
echo "========================================"
echo " VLMAPS MAPPING ACQUISITION"
echo "========================================"
echo "Scenario: $SCENARIO"
echo
echo "ABRE EN ISAAC SIM:"
echo "$SCENE"
echo
echo "Presiona Play y verifica que el Jackal esté cargado."
echo "========================================"
echo

read -r -p "Presiona ENTER cuando Isaac esté listo..."

conda deactivate 2>/dev/null || true

cd "$INDOOR"

# ROS setup scripts are not safe under bash nounset.
set +u
source /opt/ros/jazzy/setup.bash
source "$INDOOR/install/setup.bash"
set -u

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

for service in \
    /set_simulation_state \
    /get_entity_state \
    /set_entity_state
do
    if ! ros2 service list | grep -qx "$service"; then
        echo "ERROR: falta $service"
        exit 1
    fi
done

EXPECTED=$(
python3 - "$POSITIONS_FILE" <<'PY'
import sys
import yaml

with open(sys.argv[1]) as f:
    data = yaml.safe_load(f)

print(len(data["positions"]))
PY
)

MAP_IDS=$(
python3 - "$POSITIONS_FILE" <<'PY'
import sys
import yaml

with open(sys.argv[1]) as f:
    data = yaml.safe_load(f)

print(" ".join(data["positions"].keys()))
PY
)

mkdir -p "$DATA_ROOT"

LOG="/tmp/vlmaps_recorder_${SCENARIO}.log"

ros2 run \
vlmaps_dataset_recorder \
vlmaps_dataset_recorder \
--ros-args \
-p use_sim_time:=true \
-p capture_mode:=trigger \
-p output_dir:="$OUTPUT_DIR" \
-p global_frame:=odom \
-p camera_frame:=Camera \
-p sync_slop_s:=0.10 \
> "$LOG" 2>&1 &

RECORDER_PID=$!

cleanup() {
    kill -INT "$RECORDER_PID" 2>/dev/null || true
    wait "$RECORDER_PID" 2>/dev/null || true
}

trap cleanup EXIT

READY=0

for _ in $(seq 1 50); do

    if ros2 service list \
        | grep -qx \
        /vlmaps_dataset_recorder/capture
    then
        READY=1
        break
    fi

    sleep 0.2
done

if [ "$READY" -ne 1 ]; then
    echo "ERROR: recorder no inició."
    cat "$LOG"
    exit 1
fi

echo
echo "Capturando $EXPECTED poses..."
echo

ros2 run \
vlmaps_dataset_recorder \
mapping_pose_runner \
--positions-file "$POSITIONS_FILE" \
--positions $MAP_IDS \
--settle-time 3.0 \
--capture-timeout 10.0

cleanup
trap - EXIT

RGB=$(
find "$OUTPUT_DIR/rgb" \
-maxdepth 1 \
-name '*.png' \
| wc -l
)

DEPTH=$(
find "$OUTPUT_DIR/depth" \
-maxdepth 1 \
-name '*.npy' \
| wc -l
)

POSES=$(
wc -l < "$OUTPUT_DIR/poses.txt"
)

TIMESTAMPS=$(
grep -vc '^#' "$OUTPUT_DIR/timestamps.txt"
)

MAX_DELTA=$(
awk '
!/^#/ {
    if ($8 > max) max=$8
}
END {
    printf "%.9f", max
}
' "$OUTPUT_DIR/timestamps.txt"
)

echo
echo "========================================"
echo " VALIDACION"
echo "========================================"
echo "Esperado:   $EXPECTED"
echo "RGB:        $RGB"
echo "Depth:      $DEPTH"
echo "Poses:      $POSES"
echo "Timestamps: $TIMESTAMPS"
echo "Max dt:     $MAX_DELTA s"
echo

if \
    [ "$RGB" -ne "$EXPECTED" ] || \
    [ "$DEPTH" -ne "$EXPECTED" ] || \
    [ "$POSES" -ne "$EXPECTED" ] || \
    [ "$TIMESTAMPS" -ne "$EXPECTED" ]
then
    echo "DATASET INVALIDO"
    exit 1
fi

echo "DATASET VALIDO:"
echo "$OUTPUT_DIR"

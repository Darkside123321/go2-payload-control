#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
output=${1:?Usage: ./check_ros_loop.sh OUTPUT_DIR timeout|zero}
stop=${2:?Specify timeout or zero}
case "$stop" in timeout|zero) ;; *) echo "Specify timeout or zero" >&2; exit 2 ;; esac
mkdir -p "$output"
if [[ -e "$output/summary.json" || -e "$output/dds.json" || -e "$output/trace.csv" ]]; then
    echo "Output already contains a run" >&2
    exit 2
fi
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-72}
if [[ "$stop" == timeout ]]; then
    ports=(--command-port 26371 --telemetry-port 26372)
else
    ports=(--command-port 26373 --telemetry-port 26374)
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/configs/fastdds_loopback.xml"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python=${GO2_PYTHON:-.venv/bin/python}

"$python" run_sensor_control.py --ros --load 0.20 --seconds 10 --output "$output" "${ports[@]}" \
    >"$output/sim.log" 2>&1 &
sim=$!
bridge=
cleanup() {
    if [[ -n "$bridge" ]]; then kill "$bridge" 2>/dev/null || true; fi
    kill "$sim" 2>/dev/null || true
    if [[ -n "$bridge" ]]; then wait "$bridge" 2>/dev/null || true; fi
    wait "$sim" 2>/dev/null || true
}
trap cleanup EXIT
source /opt/ros/humble/setup.bash
/usr/bin/python3 ros_bridge.py "${ports[@]}" >"$output/bridge.log" 2>&1 &
bridge=$!
if ! /usr/bin/python3 check_ros_loop.py --stop "$stop" --output "$output/dds.json"; then
    tail -n 20 "$output/bridge.log" "$output/sim.log"
    exit 1
fi
if ! wait "$sim"; then
    tail -n 20 "$output/sim.log"
    exit 1
fi
tail -1 "$output/sim.log"

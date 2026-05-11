#!/usr/bin/env bash
set -euo pipefail

LIVOX_DELAY_SEC="${LIVOX_DELAY_SEC:-2}"
FAST_LIO_DELAY_SEC="${FAST_LIO_DELAY_SEC:-2}"
STATIC_TF_DELAY_SEC="${STATIC_TF_DELAY_SEC:-2}"
WITH_FAST_LIO_RVIZ="${WITH_FAST_LIO_RVIZ:-0}"
WITH_BROADCASTERS_RVIZ="${WITH_BROADCASTERS_RVIZ:-0}"
LOG_DIR="${LOG_DIR:-/tmp/run_mapping_terminals_logs}"
TERMINAL_PIDS=()
CLEANING_UP=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [--no-rviz]

Environment overrides:
  LIVOX_DELAY_SEC      Seconds to wait after starting Livox before FAST-LIO. Default: 2
  FAST_LIO_DELAY_SEC   Seconds to wait after starting FAST-LIO before static TF. Default: 2
  STATIC_TF_DELAY_SEC  Seconds to wait after starting static TF before broadcasters. Default: 2
  WITH_FAST_LIO_RVIZ   1 to start FAST-LIO RViz, 0 to skip it. Default: 0
  WITH_BROADCASTERS_RVIZ
                       1 to start Go2 broadcasters RViz, 0 to skip it. Default: 0
  LOG_DIR              Directory for per-terminal logs. Default: /tmp/run_mapping_terminals_logs
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-rviz)
      WITH_FAST_LIO_RVIZ=0
      WITH_BROADCASTERS_RVIZ=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

cleanup() {
  local pid

  if [[ "${CLEANING_UP}" == "1" ]]; then
    exit 130
  fi
  CLEANING_UP=1

  if [[ "${#TERMINAL_PIDS[@]}" -eq 0 ]]; then
    exit 130
  fi

  echo
  echo "Closing mapping xterm windows..."
  for pid in "${TERMINAL_PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done

  exit 130
}

open_terminal() {
  local title="$1"
  local command="$2"
  local safe_name="$3"
  local runner="${LOG_DIR}/${safe_name}.sh"
  local log_file="${LOG_DIR}/${safe_name}.log"

  cat >"${runner}" <<EOF
#!/usr/bin/env bash
set +e

echo "[$(date '+%F %T')] Starting ${title}"
echo "Log file: ${log_file}"
echo

(
${command}
) 2>&1 | tee -a "${log_file}"

status=\${PIPESTATUS[0]}
echo
echo "[$(date '+%F %T')] ${title} exited with status \${status}" | tee -a "${log_file}"
echo "Leaving this shell open for inspection. Type 'exit' or press Ctrl-D to close."
exec bash -i
EOF
  chmod +x "${runner}"

  xterm -hold -T "${title}" -e bash "${runner}" &
  TERMINAL_PIDS+=("$!")
}

running_inside_container() {
  [[ -f /.dockerenv || -f /root/legged_ws/setup.sh ]]
}

if ! running_inside_container; then
  cat >&2 <<EOF
This script currently supports running inside the mapping Docker container only.
Start and enter the container first:
  docker/run_mapping.sh
Then run:
  /root/legged_ws/src/legged_ros2/scripts/run_mapping_terminals.sh
EOF
  exit 1
fi

if [[ -z "${NET_IF:-}" ]]; then
  cat >&2 <<EOF
NET_IF is not set.

Set it to the network interface connected to the robot/MID360 network before running this script.

Example:
  export NET_IF=enp8s0
  /root/legged_ws/src/legged_ros2/scripts/run_mapping_terminals.sh

Check available interfaces with:
  ip addr
EOF
  exit 1
fi

echo "Using NET_IF=${NET_IF}"
echo "WITH_FAST_LIO_RVIZ=${WITH_FAST_LIO_RVIZ}"
echo "WITH_BROADCASTERS_RVIZ=${WITH_BROADCASTERS_RVIZ}"

mkdir -p "${LOG_DIR}"

if ! command -v xterm >/dev/null 2>&1; then
  cat >&2 <<EOF
xterm is required to run this helper script.
Install it inside the mapping container:
  apt-get update && apt-get install -y xterm
EOF
  exit 1
fi

trap cleanup INT TERM

LIVOX_CMD='
source /root/legged_ws/setup.sh
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
source /root/livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
'

FAST_LIO_CMD='
source /root/legged_ws/setup.sh
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
source /root/livox_ws/install/setup.bash
source /root/fast_lio_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=${WITH_FAST_LIO_RVIZ}
'

STATIC_TF_CMD='
source /root/legged_ws/setup.sh
ros2 launch go2_description lidar_static_tf.launch.py
'

RVIZ_CMD='
source /root/legged_ws/setup.sh
ros2 launch go2_description bringup_broadcasters.launch.py use_rviz:=${WITH_BROADCASTERS_RVIZ}
'

echo "Opening Terminal 1: Livox MID360 driver"
open_terminal "mapping: livox_ros_driver2" "${LIVOX_CMD}" "01_livox_ros_driver2"

echo "Waiting ${LIVOX_DELAY_SEC}s before starting FAST-LIO"
sleep "${LIVOX_DELAY_SEC}"

echo "Opening Terminal 2: FAST-LIO 2"
open_terminal "mapping: FAST-LIO 2" "${FAST_LIO_CMD}" "02_fast_lio"

echo "Waiting ${FAST_LIO_DELAY_SEC}s before starting static TF"
sleep "${FAST_LIO_DELAY_SEC}"

echo "Opening Terminal 3: lidar static TF"
open_terminal "mapping: lidar static TF" "${STATIC_TF_CMD}" "03_lidar_static_tf"

echo "Waiting ${STATIC_TF_DELAY_SEC}s before starting broadcasters"
sleep "${STATIC_TF_DELAY_SEC}"

echo "Opening Terminal 4: broadcasters"
open_terminal "mapping: broadcasters" "${RVIZ_CMD}" "04_broadcasters"

echo "All mapping terminals are running. Press Ctrl-C here to close them."
wait "${TERMINAL_PIDS[@]}" || true

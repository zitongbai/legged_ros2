#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_NAME="${IMAGE_NAME:-legged-ros2-mapping:humble}"
CONTAINER_NAME="${CONTAINER_NAME:-legged-ros2-mapping-humble}"
ONNX_VERSION="${ONNX_VERSION:-1.22.0}"
ONNX_ARCHIVE="onnxruntime-linux-x64-${ONNX_VERSION}.tgz"
ONNX_DIR="onnxruntime-linux-x64-${ONNX_VERSION}"
LIBMOTIONCAPTURE_REPO="${LIBMOTIONCAPTURE_REPO:-https://github.com/NOKOV-MOCAP/libmotioncapture.git}"
LIBMOTIONCAPTURE_REF="${LIBMOTIONCAPTURE_REF:-main}"

prepare_third_party() {
  for cmd in git wget tar; do
    if ! command -v "${cmd}" >/dev/null 2>&1; then
      echo "${cmd} is not installed or not in PATH" >&2
      exit 1
    fi
  done

  local third_party_dir="${REPO_ROOT}/third_party"
  mkdir -p "${third_party_dir}"

  if [[ ! -d "${third_party_dir}/${ONNX_DIR}" ]]; then
    if [[ ! -f "${third_party_dir}/${ONNX_ARCHIVE}" ]]; then
      wget -O "${third_party_dir}/${ONNX_ARCHIVE}" \
        "https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/${ONNX_ARCHIVE}"
    fi
    tar -xzf "${third_party_dir}/${ONNX_ARCHIVE}" -C "${third_party_dir}"
  fi

  if [[ ! -d "${third_party_dir}/libmotioncapture" ]]; then
    git clone --branch "${LIBMOTIONCAPTURE_REF}" "${LIBMOTIONCAPTURE_REPO}" \
      "${third_party_dir}/libmotioncapture"
  fi

  if [[ -d "${third_party_dir}/libmotioncapture/.git" ]]; then
    git -C "${third_party_dir}/libmotioncapture" submodule update --init --recursive
  fi
}

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not installed or not in PATH" >&2
  exit 1
fi

echo "Preparing third-party dependencies on host"
prepare_third_party

if docker ps -a --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "Removing existing container: ${CONTAINER_NAME}"
  docker rm -f "${CONTAINER_NAME}" >/dev/null
fi

docker_args=(
  -d
  --name "${CONTAINER_NAME}"
  --network host
  -e "TERM=${TERM:-xterm-256color}"
  -e "COLORTERM=${COLORTERM:-truecolor}"
  -v "${REPO_ROOT}:/root/legged_ws/src/legged_ros2"
  -v "${REPO_ROOT}/legged_mapping/config/MID360_config.json:/root/livox_ws/src/livox_ros_driver2/config/MID360_config.json:ro"
)

if [[ -n "${DISPLAY:-}" ]]; then
  if command -v xhost >/dev/null 2>&1; then
    xhost +local:root >/dev/null || true
  fi

  docker_args+=(
    -e "DISPLAY=${DISPLAY}"
    -e "QT_X11_NO_MITSHM=1"
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  )

  if [[ -d /dev/dri ]]; then
    docker_args+=(-v /dev/dri:/dev/dri)
  fi
else
  echo "DISPLAY is not set on the host; GUI programs inside the container will not open windows." >&2
fi

echo "Starting container ${CONTAINER_NAME} from image ${IMAGE_NAME}"
docker run "${docker_args[@]}" \
  "${IMAGE_NAME}" \
  -lc "trap : TERM INT; sleep infinity & wait"

echo "Initializing legged workspace inside ${CONTAINER_NAME}"
docker exec "${CONTAINER_NAME}" /bin/bash -lc "
set -eo pipefail

source /root/unitree_ros2/setup_local.sh
cd /root/legged_ws
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source /opt/ros/humble/setup.bash
cd /root/livox_ws/src/livox_ros_driver2
./build.sh humble
"

echo "Container is ready."
echo "Opening an interactive shell in ${CONTAINER_NAME}"
exec docker exec -it \
  -e "TERM=${TERM:-xterm-256color}" \
  -e "COLORTERM=${COLORTERM:-truecolor}" \
  "${CONTAINER_NAME}" \
  /bin/bash

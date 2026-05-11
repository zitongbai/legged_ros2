#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-legged-ros2-humble}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not installed or not in PATH" >&2
  exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "Container is not running: ${CONTAINER_NAME}" >&2
  echo "Start it first with docker/run.sh" >&2
  exit 1
fi

docker exec -it \
  -e "TERM=${TERM:-xterm-256color}" \
  -e "COLORTERM=${COLORTERM:-truecolor}" \
  "${CONTAINER_NAME}" \
  /bin/bash

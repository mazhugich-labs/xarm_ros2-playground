#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_NAME="xarm-ros2-playground"
CONTAINER_NAME="xarm-ros2-playground"

usage() {
    echo "Usage: $0 {build|run|enter|emulate|stop}"
}

case "${1:-}" in
    build)
        docker build \
            --build-arg USER_UID="$(id -u)" \
            --build-arg USER_GID="$(id -g)" \
            -t "${IMAGE_NAME}" \
            "${PROJECT_DIR}"
        ;;
    run)
        docker_args=(
            --rm -dit
            --name "${CONTAINER_NAME}"
            --network host
            --ipc host
            -v "${PROJECT_DIR}/ros2_ws:/opt/ros2_ws"
        )

        if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
            docker_args+=(
                -e DISPLAY
                -v /tmp/.X11-unix:/tmp/.X11-unix:rw
            )

            if [[ -n "${XAUTHORITY:-}" && -f "${XAUTHORITY}" ]]; then
                docker_args+=(
                    -e XAUTHORITY=/tmp/.xarm-docker.xauth
                    -v "${XAUTHORITY}:/tmp/.xarm-docker.xauth:ro"
                )
            fi
        fi

        if [[ -d /dev/dri ]]; then
            docker_args+=(--device /dev/dri)
            mapfile -t dri_group_ids < <(
                find /dev/dri -maxdepth 1 -type c -printf '%G\n' | sort -un
            )
            for dri_group_id in "${dri_group_ids[@]}"; do
                docker_args+=(--group-add "${dri_group_id}")
            done
        fi

        docker run "${docker_args[@]}" "${IMAGE_NAME}"
        ;;
    enter)
        docker exec -it "${CONTAINER_NAME}" /usr/local/bin/xarm-entrypoint bash
        ;;
    emulate)
        # Only the TCP emulator needs root to bind port 502 on the host network.
        # It writes no workspace files; ROS nodes still run as the host-mapped user.
        docker exec -it --user root -e PYTHONDONTWRITEBYTECODE=1 \
            "${CONTAINER_NAME}" /usr/local/bin/xarm-entrypoint \
            ros2 run xarm_network_emulator controller
        ;;
    stop)
        docker stop "${CONTAINER_NAME}"
        ;;
    *)
        usage
        exit 1
        ;;
esac

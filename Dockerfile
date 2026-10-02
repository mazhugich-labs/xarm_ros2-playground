FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR /opt/xarm_ros2_ws
COPY xarm_ros2 /opt/xarm_ros2_ws/src/xarm_ros2

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep update \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -y \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build

ARG GAZEBO_MODELS_COMMIT=8163eb4b5e7e21985c6591d1c0bfb56468c0093f
RUN install -d /usr/share/gazebo-11/models/table \
    && for model_file in model.config model.sdf model-1_2.sdf model-1_3.sdf model-1_4.sdf; do \
        curl -fsSL \
            "https://raw.githubusercontent.com/osrf/gazebo_models/${GAZEBO_MODELS_COMMIT}/table/${model_file}" \
            -o "/usr/share/gazebo-11/models/table/${model_file}"; \
    done

COPY docker/entrypoint.sh /usr/local/bin/xarm-entrypoint
RUN chmod 0755 /usr/local/bin/xarm-entrypoint

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source /opt/xarm_ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /opt/ros2_ws

ENTRYPOINT ["/usr/local/bin/xarm-entrypoint"]
CMD ["bash"]

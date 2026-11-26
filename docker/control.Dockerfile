FROM ros:humble-ros-base

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=franka

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    RCUTILS_COLORIZED_OUTPUT=1

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      bash-completion \
      build-essential \
      curl \
      git \
      iputils-ping \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      sudo \
      ros-humble-ros-gz \
      ros-humble-sdformat-urdf \
      ros-humble-ros2controlcli \
      ros-humble-controller-manager \
      ros-humble-control-msgs \
      ros-humble-hardware-interface \
      ros-humble-joint-state-broadcaster \
      ros-humble-joint-trajectory-controller \
      ros-humble-moveit-ros-move-group \
      ros-humble-moveit-kinematics \
      ros-humble-moveit-planners-ompl \
      ros-humble-moveit-ros-visualization \
      ros-humble-moveit-simple-controller-manager \
      ros-humble-realtime-tools \
      ros-humble-rviz2 \
      ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/franka

WORKDIR /ros2_ws
COPY --chown=${USER_UID}:${USER_GID} robot_ws/src /ros2_ws/src

RUN rosdep init 2>/dev/null || true \
    && rosdep update \
    && apt-get update \
    && rosdep install --from-paths /ros2_ws/src --ignore-src --rosdistro humble -r -y \
       --skip-keys="franka_selfcollision gz_sim_vendor parallel_gripper_controller gz_plugin_vendor" \
    && rm -rf /var/lib/apt/lists/*

RUN chown -R "${USER_UID}:${USER_GID}" /ros2_ws

USER ${USERNAME}
RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install \
       --packages-up-to franka_bringup franka_gripper franka_fr3_moveit_config franka_gazebo_bringup \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_TESTING=OFF

COPY --chown=${USER_UID}:${USER_GID} docker/control-entrypoint.sh /usr/local/bin/control-entrypoint
USER root
RUN chmod 0755 /usr/local/bin/control-entrypoint
USER ${USERNAME}

ENTRYPOINT ["/usr/local/bin/control-entrypoint"]
CMD ["bash"]

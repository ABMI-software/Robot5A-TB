FROM ros:jazzy-ros-base

RUN apt-get update && \
    apt-get install -y meshlab \
                       ros-jazzy-cv-bridge \
                       ros-jazzy-gz-ros2-control \
                       ros-jazzy-joint-state-broadcaster \
                       ros-jazzy-joint-state-publisher \
                       ros-jazzy-joint-trajectory-controller \
                       ros-jazzy-moveit \
                       ros-jazzy-robot-state-publisher \
                       ros-jazzy-ros2-control \
                       ros-jazzy-ros2-controllers \
                       ros-jazzy-rqt-graph

RUN apt-get update && \
    apt-get install -y bash-completion \
                       sudo \
                       vim && \
    echo "%ubuntu ALL=(ALL) NOPASSWD: ALL" >>"/etc/sudoers.d/ubuntu"

RUN apt-get update && \
    apt-get install -y pip \
                       python3-venv

# RUN rosdep update && \
#     rosdep install --from-paths src --ignore-src --rosdistro jazzy -y


# GamesmanROS 2 development image
#
# Base is ros:humble (NOT osrf/ros:humble-desktop, which is amd64-only and would
# run under emulation on Apple Silicon). ros:humble ships native arm64 + amd64,
# which also matches the Raspberry Pi target architecture.
FROM ros:humble

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-moveit \
      ros-humble-moveit-py \
      ros-humble-rviz2 \
      ros-humble-tf-transformations \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-joint-state-publisher-gui \
      ros-humble-xacro \
      python3-colcon-common-extensions \
      python3-transforms3d \
      python3-pip \
      libgl1 libglib2.0-0 \
      vim less \
 && rm -rf /var/lib/apt/lists/*

# OpenCV via pip, NOT apt. Ubuntu 22.04 ships python3-opencv 4.5.4, which does
# not have cv2.aruco.ArucoDetector — that API landed in 4.7 and vision_node.py
# depends on it.
# numpy is pinned <2 because tf_transformations/transforms3d still call
# np.maximum_sctype, which NumPy 2.0 removed. Without this pin arm_controller
# fails to import.
RUN pip3 install --no-cache-dir \
      "numpy<2" \
      "opencv-contrib-python>=4.7,<6.1" \
      requests

WORKDIR /ws

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

CMD ["bash"]

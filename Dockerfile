FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    ros-noetic-openni2-camera \
    ros-noetic-openni2-launch \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    python3-venv python3-pip \
    libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# venv + ultralytics（psutil衝突回避）
RUN python3 -m venv /opt/venv && \
    /opt/venv/bin/python -m pip install -U pip && \
    /opt/venv/bin/python -m pip install -U ultralytics opencv-python

ENV PATH="/opt/venv/bin:${PATH}"

# catkin workspace
WORKDIR /catkin_ws
COPY catkin_ws/src /catkin_ws/src

RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && \
  export PATH=/opt/ros/noetic/bin:/usr/bin:/bin:/usr/sbin:/sbin && \
  catkin_make"

RUN tee -a /etc/bash.bashrc >/dev/null <<'EOF'

# Auto setup for ROS + catkin + venv
source /opt/ros/noetic/setup.bash
if [ -f /catkin_ws/devel/setup.bash ]; then
  source /catkin_ws/devel/setup.bash
fi
if [ -f /opt/venv/bin/activate ]; then
  source /opt/venv/bin/activate
fi
EOF


CMD ["/bin/bash", "-lc", "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch yolo26_ros1 xtion_yolo26.launch"]


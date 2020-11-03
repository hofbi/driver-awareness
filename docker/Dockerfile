FROM ros:noetic-robot

ENV DEBIAN_FRONTEND='noninteractive'
ENV QT_X11_NO_MITSHM=1

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    python3-catkin-tools \
    python3-osrf-pycommon \
    x11-apps \
    socat \
    ubuntu-server \
    ubuntu-desktop \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*

COPY . /home/catkin_ws/src/driver-awareness
WORKDIR /home/catkin_ws/src/driver-awareness
RUN ./install.sh

RUN pip3 install $(grep -ivE "pygame" requirements.txt)

WORKDIR /home/catkin_ws
#RUN bash -c "source /opt/ros/noetic/setup.bash && source /opt/carla-ros-bridge/noetic/setup.bash && catkin init && catkin build"
RUN bash -c "source /opt/ros/noetic/setup.bash && catkin init && catkin build"

ENTRYPOINT [ "src/driver-awareness/docker/entrypoint.sh" ]

FROM ghcr.io/automotiveaichallenge/aichallenge2022-sim/autoware-universe-cuda:3.1

ARG NVIDIA_VISIBLE_DEVICES=all
ARG NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display
ARG DEBIAN_FRONTEND=noninteractive

# Copy map
COPY nishishinjuku_autoware_map /aichallenge/nishishinjuku_autoware_map

# Copy common ros2 packages
COPY aichallenge_ws /aichallenge/aichallenge_ws

# Copy user files
COPY aichallenge_submit.tar.gz /aichallenge_submit.tar.gz
RUN cd / \
 && mkdir /aichallenge/aichallenge_ws/src/aichallenge_submit \
 && tar zxf aichallenge_submit.tar.gz -C /aichallenge/aichallenge_ws/src/aichallenge_submit \
 && rm /aichallenge_submit.tar.gz

# build
RUN apt-get update
RUN bash -c ' \
  source /autoware/install/setup.bash; \
  cd /aichallenge/aichallenge_ws; \
  rosdep update; \
  rosdep install -y -r -i --from-paths src --ignore-src --rosdistro $ROS_DISTRO; \
  colcon build'

COPY main.bash /main.bash

ENTRYPOINT []
CMD ["bash", "/main.bash"]

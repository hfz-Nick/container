FROM ros:eloquent-ros-core

ADD sources.list /etc/apt/sources.list

# install necessary tools and dependencies
RUN apt-get update && \
    apt-get install -y libi2c-dev vim ros-eloquent-teleop-twist-keyboard gcc g++ curl python3-pip 

RUN sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y python3-colcon-common-extensions

RUN rm -rf /var/lib/apt/lists

# create home directory under user ros
RUN mkdir -p /home/ros

# add libraries and install them
RUN mkdir -p /home/ros/libraries/cpp/build
ADD /core_libs /home/ros/libraries
RUN cd /home/ros/libraries/cpp/build && cmake .. && make && make install

# move to ros2_libs and colcon build
RUN /bin/bash -c "source /opt/ros/eloquent/setup.bash && cd /home/ros/libraries/ros2_libs \
                  && colcon build"

# run pip install envbash
RUN python3 -m pip install envbash pyserial

# source library path for ROS2
ENV LD_LIBRARY_PATH=/usr/local/lib:/usr/local/lib/navatics_libs:$LD_LIBRARY_PATH

# set work dir to /home/ros
ENV HOME=/home/ros
WORKDIR $HOME

ADD navatics_core_entrypoint.sh /navatics_core_entrypoint.sh
CMD ["/bin/bash"]
ENTRYPOINT ["/navatics_core_entrypoint.sh"]

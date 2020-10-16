FROM ros:eloquent-ros-core

ADD sources.list /etc/apt/sources.list

# install necessary tools and dependencies
RUN apt-get update && \
    apt-get install -y libi2c-dev vim ros-eloquent-teleop-twist-keyboard gcc g++ curl python3-pip && \
    apt-get install -y libevent-dev

RUN sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y python3-colcon-common-extensions

RUN rm -rf /var/lib/apt/lists

# create home directory under user ros
RUN mkdir -p /home/ros

# source library path for ROS2
ENV LD_LIBRARY_PATH=/usr/local/lib:/usr/local/lib/navatics_libs:$LD_LIBRARY_PATH

# add libraries and install them
# currently no libraries present so these are commented
# RUN mkdir -p /home/ros/libraries/cpp/build
# ADD /endpoints_libs /home/ros/libraries
# RUN cd /home/ros/libraries/cpp/build && cmake .. && make && make install

# add endpoints workspace
ADD /endpoints_ws/src /home/ros/endpoints_ws/src
RUN /bin/bash -c "source /opt/ros/eloquent/setup.bash && \
                  cd /home/ros/endpoints_ws && \
                  colcon build"

# set work dir to /home/ros
ENV HOME=/home/ros
WORKDIR $HOME

ADD navatics_endpoints_entrypoint.sh /navatics_endpoints_entrypoint.sh
CMD ["/bin/bash", "-c", "ros2 launch navatics_server navatics_endpoints.launch.py"]
ENTRYPOINT ["/navatics_endpoints_entrypoint.sh"]

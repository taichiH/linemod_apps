FROM ros:indigo

MAINTAINER iory ab.ioryz@gmail.com

ENV ROS_DISTRO indigo

RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y \
    python-catkin-tools \
    ros-${ROS_DISTRO}-jsk-tools

RUN mkdir -p /ros_ws/src && \
    cd /ros_ws/src && \
    git clone https://github.com/iory/linemod_apps

RUN cd ros_ws; rosdep install -r -y --from-paths src --ignore-src
RUN mv /bin/sh /bin/sh_tmp && ln -s /bin/bash /bin/sh
RUN source /opt/ros/${ROS_DISTRO}/setup.bash; cd ros_ws; catkin build linemod_recognition
RUN rm /bin/sh && mv /bin/sh_tmp /bin/sh

RUN touch /root/.bashrc && \
    echo "source /opt/ros/indigo/setup.bash\n" >> /root/.bashrc && \
    echo "rossetip\n" >> /root/.bashrc && \
    echo "rossetmaster localhost\n"

COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

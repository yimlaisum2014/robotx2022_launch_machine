FROM osrf/ros:melodic-desktop-full

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO melodic


###################################### user #####################################

ENV SHELL=/bin/bash \
    USER=sam \
    UID=1000 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8

ENV HOME=/home/${USER}

RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${UID} \
    ${USER} 

RUN echo "root:root" | chpasswd
RUN echo "${USER}:111111" | chpasswd

###################################### basic tools #####################################
# Fix the time synchronization problem
RUN echo "Acquire::Check-Valid-Until \"false\";\nAcquire::Check-Date \"false\";" | cat > /etc/apt/apt.conf.d/10no--check-valid-until

Run apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install -yq --no-install-recommends \
    curl \
    lsb-release \
    wget \
    sudo \
    software-properties-common \
    git \
    cmake \
    make \
    vim \
    build-essential \
    libblkid-dev \
    e2fslibs-dev \
    libboost-all-dev \
    libaudit-dev \
    net-tools \
    iputils-ping 

###################################### ROS ######################################
Run apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-rosbridge-server \
    ros-$ROS_DISTRO-smach \
    ros-$ROS_DISTRO-smach-msgs \
    ros-$ROS_DISTRO-smach-ros \
    ros-$ROS_DISTRO-executive-smach \
    ros-$ROS_DISTRO-smach-viewer

#################################### GTSAM ####################################

RUN cd ${HOME}/ \
    && git clone https://github.com/borglab/gtsam.git \
    && cd gtsam/ \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install

##################################### setting #####################################
RUN chown -R ${USER}:${USER} ${HOME}/
RUN echo "root ALL=(ALL) ALL" > /etc/sudoers


# setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

WORKDIR ${HOME}

############################# LoCoBot and PyRobot ############################

RUN cd ${HOME}/

RUN curl 'https://raw.githubusercontent.com/yimlaisum2014/pyrobot/master/robots/LoCoBot/install/locobot_install_all_for_docker.sh' > locobot_install_all_for_docker.sh && chmod +x locobot_install_all_for_docker.sh

RUN ./locobot_install_all_for_docker.sh -t full -p 3 -l interbotix

# RUN rm locobot_install_all.sh



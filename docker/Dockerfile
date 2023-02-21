ARG from

FROM ${from}

LABEL Dmitrii Dobriborsci "dobriy06@gmail.com"

# Timezone Configuration
ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ENV DEBIAN_FRONTEND = noninteractive

# Utilities
RUN apt-get update && apt-get install -y apt-utils \
                                         lsb-release \
                                         mesa-utils \
                                         gnupg2 \
                                         net-tools \
                                         build-essential \
                                         wget \
                                         unzip \
                                         curl \
                                         git \
                                         mc \
                                         vim \
                                         ffmpeg

RUN apt-get update && apt-get install -y python3-pip
RUN pip3 install pathlib statistics scipy
RUN python3 -m pip install --upgrade pip

## GUI tunnel from docker
RUN dpkg --add-architecture i386 && \
    apt-get update && apt-get install -y --no-install-recommends \
        libxau6 libxau6:i386 \
        libxdmcp6 libxdmcp6:i386 \
        libxcb1 libxcb1:i386 \
        libxext6 libxext6:i386 \
        libx11-6 libx11-6:i386 && \
    rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
        ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
        ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

RUN apt-get update && apt-get install -y --no-install-recommends \
        libglvnd0 libglvnd0:i386 \
        libgl1 libgl1:i386 \
        libglx0 libglx0:i386 \
        libegl1 libegl1:i386 \
        libgles2 libgles2:i386 && \
    rm -rf /var/lib/apt/lists/*

COPY docker2/10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json
# Required for non-glvnd setups.
ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64

# very important libraries :)
RUN apt update && apt install -y python3-opengl
RUN pip3 install pyglet
# RUN pip3 install tensorboard
# RUN pip install optuna
RUN pip install plotly

# Install ROS desktop full

RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get -qq install -y ros-noetic-desktop-full \
    # ros-noetic-joy ros-noetic-teleop-twist-joy \
    # ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
    # ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
    # ros-noetic-rosserial-python ros-noetic-rosserial-client \
    # ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
    # ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
    # ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
    # ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers \
    ros-noetic-behaviortree-cpp-v3 \
    ros-noetic-moveit \
    ros-noetic-soem \
    ros-noetic-vision-msgs \
    ros-noetic-socketcan-interface \
    ros-noetic-industrial-robot-status-interface \
    ros-noetic-scaled-joint-trajectory-controller \
    ros-noetic-speed-scaling-interface \
    ros-noetic-speed-scaling-state-controller \
    ros-noetic-ur-msgs \
    ros-noetic-pass-through-controllers \
    ros-noetic-ur-client-library \ 
    ros-noetic-rosparam-shortcuts \
    ros-noetic-velocity-controllers \
    ros-noetic-trac-ik-kinematics-plugin \
    ros-noetic-mir-robot   

# Install additional libraries
RUN apt-get install -y libqt5svg5-dev \
    libgoogle-glog-dev \
    python3-catkin-tools \
    libuvc-dev

RUN apt-get update

# RUN pip install tabulate
# RUN pip install mpldatacursor
# RUN pip install svgpath2mpl
# RUN pip install shapely

RUN echo "source /misis/mir_ws/devel/setup.bash" >> ~/.bashrc
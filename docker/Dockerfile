FROM nvidia/opengl:1.2-glvnd-devel-ubuntu18.04

LABEL version="0.1"
LABEL description="Docker image for dense visual odometry: https://github.com/tannerliu347/dvo_contact"

# optional, if the default user is not "root", you might need to switch to root here and at the end of the script to the original user again.
# e.g.
# USER root
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y locales \
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
    && dpkg-reconfigure --frontend=noninteractive locales \
    && update-locale LANG=en_US.UTF-8


RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo \
        dialog \
        apt-utils \
        libeigen3-dev \
        git \
        mercurial \
        pkg-config \
        mesa-utils \
        libxau-dev \
        libxdmcp-dev \
        libxcb1-dev \
        libxext-dev \
        libx11-dev \ 
        git \
        tmux \
        wget \
        curl \
        cmake \
        vim \ 
        build-essential \
        unzip \
        autoconf \
        autogen \
        automake \
        libtool \
        mlocate \
        zlib1g-dev \
        libopenblas-dev \
        libqt4-dev \
        libflann-dev \
        libopencv-dev \
        libpcl-dev \
        libboost-all-dev \
        libsuitesparse-dev \
        libtbb-dev \
        libusb-1.0-0-dev \
        libgtest-dev \
        pkg-config \
        cmake-curses-gui \
        gdb \
        libyaml-cpp-dev \
        libgoogle-glog-dev \ 
        libgflags-dev \
        clang && \
    apt clean  && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 



######## Install Eigen 3.3.7
RUN  mkdir -p /home/root/tmp/ \
&&   wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.bz2 --no-check-certificate \
&&   tar -xf eigen-3.3.7.tar.bz2  && rm eigen-3.3.7.tar.bz2 \
&&   mv eigen-3.3.7 /home/root/tmp/eigen-3.3.7 \
&&   cd /home/root/tmp/eigen-3.3.7  \
&&   mkdir build && cd build && cmake .. && make -j4 && make install

######## Install sophus
RUN  mkdir -p /home/root/tmp/ \
&&   wget https://github.com/strasdat/Sophus/archive/v1.0.0.tar.gz --no-check-certificate \
&&   tar -xf v1.0.0.tar.gz  \
&&   mv Sophus-1.0.0 /home/root/tmp/Sophus-1.0.0 \
&&   cd /home/root/tmp/Sophus-1.0.0 \
&&   mkdir build && cd build && cmake .. && make -j4 && make install

########### Install gtsam
RUN cd /home/root/tmp/ \
&& git clone -c http.sslVerify=false https://github.com/borglab/gtsam.git \
&& cd gtsam \
&& mkdir build && cd build && cmake .. && make -j4 && make install

########### Install ceres
RUN cd /home/root/tmp \
&& wget http://ceres-solver.org/ceres-solver-2.0.0.tar.gz --no-check-certificate \
&& tar zxf ceres-solver-2.0.0.tar.gz && mkdir ceres-bin && cd ceres-bin && cmake ../ceres-solver-2.0.0 \
&&  make -j4 && make install 


# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compute,utility,display

# USER original_user

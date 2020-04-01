FROM ros:melodic-perception

ENV CERES_VERSION="1.12.0"
ENV CATKIN_WS=/root/catkin_ws
ENV CVVERSION="master"

RUN   mkdir installation && \
      mkdir installation/OpenCV-"$CVVERSION"
ENV cwd=/root

      # set up thread number for building
RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi && \
      apt-get update && apt-get install -y \
      cmake \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      python-catkin-tools \
      build-essential checkinstall cmake pkg-config yasm \
      git gfortran \
      libjpeg8-dev libpng-dev \
      software-properties-common && \
      rm -rf /var/lib/apt/lists/* && \
      add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main" && \
      apt-get update && apt-get install -y \
      libjasper1 libtiff-dev \
      libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev \
      libxine2-dev libv4l-dev && \
      rm -rf /var/lib/apt/lists/* && \
      cd /usr/include/linux && \
      sudo ln -s -f ../libv4l1-videodev.h videodev.h && \
      cd ${cwd} && \
      apt-get update && apt-get install -y \
      libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      libgtk2.0-dev libtbb-dev qt5-default libatlas-base-dev \
      libfaac-dev libmp3lame-dev libtheora-dev libvorbis-dev libxvidcore-dev \
      libopencore-amrnb-dev libopencore-amrwb-dev libavresample-dev x264 v4l-utils \
      libprotobuf-dev protobuf-compiler libgflags-dev libgphoto2-dev libhdf5-dev doxygen \
      python3-dev python3-pip python3-numpy && \
      rm -rf /var/lib/apt/lists/* && \
      apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-tf && \
      rm -rf /var/lib/apt/lists/* && \
      mkdir -p $CATKIN_WS/src/map_generator/

RUN git clone https://github.com/opencv/opencv.git && \
    cd opencv && \
    git checkout $CVVERSION && \
    cd .. && \
    git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv_contrib && \
    git checkout $CVVERSION && \
    cd .. && \
    cd opencv && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
            -D CMAKE_INSTALL_PREFIX=/usr/local \
            -D INSTALL_C_EXAMPLES=ON \
            -D INSTALL_PYTHON_EXAMPLES=ON \
            -D WITH_TBB=ON \
            -D WITH_V4L=ON \
        -D WITH_QT=ON \
        -D WITH_OPENGL=ON \
        -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
        -D ENABLE_CXX11=1 \
        -D BUILD_EXAMPLES=ON .. && \
    make -j4 && \
    make install

# Copy VINS-Fusion
COPY ./ $CATKIN_WS/src/map_generator/

# Build VINS-Fusion
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN /bin/bash -c 'cd $CATKIN_WS; . /opt/ros/melodic/setup.bash; cd $CATKIN_WS; catkin_make'

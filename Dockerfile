FROM ros:jazzy-ros-base

RUN apt-get update && \
    apt-get install -y meshlab \
                       ros-jazzy-cv-bridge \
                       ros-jazzy-gz-ros2-control \
                       ros-jazzy-joint-state-broadcaster \
                       ros-jazzy-joint-state-publisher \
                       ros-jazzy-joint-trajectory-controller \
                       ros-jazzy-moveit \
                       ros-jazzy-robot-state-publisher \
                       ros-jazzy-ros2-control \
                       ros-jazzy-ros2-controllers \
                       ros-jazzy-rqt-graph 

RUN apt-get update && \
    apt-get install -y bash-completion \
                       sudo \
                       vim && \
    echo "%ubuntu ALL=(ALL) NOPASSWD: ALL" >>"/etc/sudoers.d/ubuntu"

RUN apt-get update && \
    apt-get install -y pip \
                       python3-venv


RUN apt-get update && \
    apt-get remove --purge -y libopencv-dev python3-opencv && \
    apt-get autoremove -y && \
    sudo rm -rf /usr/local/lib/cmake/opencv4 && \
    sudo rm -rf /usr/local/lib/pkgconfig/opencv4.pc && \
    sudo rm -rf /usr/local/bin/opencv* && \
    cd ~ && \
    git clone https://github.com/opencv/opencv.git && \
    git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv && \
    git checkout 4.9.0 && \
    cd ../opencv_contrib && \
    git checkout 4.9.0 && \
    cd ../opencv && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
        -D WITH_CUDA=ON \
        -D ENABLE_FAST_MATH=1 \
        -D CUDA_FAST_MATH=1 \
        -D WITH_CUBLAS=1 \
        -D WITH_QT=ON \
        -D WITH_OPENGL=ON \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D BUILD_EXAMPLES=ON .. && \
    make -j$(nproc) && \
    sudo make install && \
    sudo ldconfig 



    




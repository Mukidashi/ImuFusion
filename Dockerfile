FROM --platform=linux/x86-64 ubuntu

ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN echo "Asia/Tokyo" > /etc/timezone

RUN apt update -y && apt upgrade -y && apt install -y build-essential cmake
RUN apt install -y libeigen3-dev libpython2.7-dev libboost-dev libboost-serialization-dev \
     libgl1-mesa-dev libglew-dev 
RUN apt-get update && apt install -y git libcurl4-openssl-dev libssl-dev zlib1g-dev  libevent-dev wget pkg-config libgtk2.0-dev


#install ffmpeg (Ref-->https://www.kkaneko.jp/tools/ubuntu/ffmpeglinux.html)
WORKDIR /opt
RUN apt -y install nasm yasm && wget https://ffmpeg.org/releases/ffmpeg-4.3.tar.gz && tar -xvzof ffmpeg-4.3.tar.gz
WORKDIR /opt/ffmpeg-4.3
RUN ./configure --enable-small --enable-shared --prefix=/usr/local && make && make install


#build & install dependencies
RUN mkdir /opt/Pangolin && git clone https://github.com/stevenlovegrove/Pangolin.git /opt/Pangolin \
    && mkdir /opt/Pangolin/build
WORKDIR /opt/Pangolin
RUN git checkout dd801d244db3a8e27b7fe8020cd751404aa818fd
WORKDIR /opt/Pangolin/build
RUN cmake -DCMAKE_BUILD_TYPE=Release  .. && make -j && make install 


RUN mkdir /opt/opencv && git clone -b 3.4 https://github.com/opencv/opencv.git /opt/opencv
RUN mkdir /opt/opencv_contrib && git clone -b 3.4 https://github.com/opencv/opencv_contrib.git /opt/opencv_contrib
WORKDIR /opt/opencv/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DWITH_GSTREAMER=OFF -DWITH_FFMPEG=ON \
    -DOPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules .. && make && make install 



#####
## build ORB_SLAM2
#####

RUN mkdir /work && mkdir /work/ORB_SLAM2
COPY src/ORB_SLAM2 /work/ORB_SLAM2

RUN apt install -y g++-10 gcc-10
RUN mkdir /work/ORB_SLAM2/Thirdparty/DBoW2-build && mkdir /work/ORB_SLAM2/Thirdparty/g2o-build
WORKDIR /work/ORB_SLAM2/Thirdparty/DBoW2-build
RUN  export CC=/usr/bin/gcc-10 && export CXX=/usr/bin/g++-10 && cmake -DCMAKE_BUILD_TYPE=Release ../DBoW2 && make -j 2

WORKDIR /work/ORB_SLAM2/Thirdparty/g2o-build
RUN export CC=/usr/bin/gcc-10 && export CXX=/usr/bin/g++-10 && cmake -DCMAKE_BUILD_TYPE=Release ../g2o && make -j 2

WORKDIR /work
RUN sh ORB_SLAM2/build.sh

WORKDIR /work/ORB_SLAM2/Vocabulary
RUN tar -xf ORBvoc.txt.tar.gz && ./bin_vocabulary


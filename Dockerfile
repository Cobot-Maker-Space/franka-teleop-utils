FROM ubuntu:24.04

ARG libfranka_version=0.21.2

RUN apt update -y \
  && apt install -y \
  build-essential \
  capnproto \
  cmake \
  git \
  pybind11-dev \
  wget \
  libasio-dev \
  libcapnp-dev \
  libcxxopts-dev \
  libeigen3-dev \
  libfmt-dev \
  libpoco-dev \
  libyaml-cpp-dev

WORKDIR /build

RUN git clone --depth 1 --recurse-submodules --shallow-submodules \
  --branch boost-1.77.0 https://github.com/boostorg/boost.git \
  && cd boost \
  && ./bootstrap.sh --prefix=/usr/local \
  && ./b2 install -j$(nproc) \
  && cd .. && rm -rf boost

RUN git clone --depth 1 --branch 10.0.0 https://github.com/leethomason/tinyxml2.git \
  && cd tinyxml2 \
  && mkdir build && cd build \
  && cmake .. \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DBUILD_SHARED_LIBS=OFF \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  && make -j$(nproc) \
  && make install \
  && cd ../.. && rm -rf tinyxml2

RUN git clone --depth 1 --branch 1.0.2 https://github.com/ros/console_bridge.git \
  && cd console_bridge \
  && mkdir build && cd build \
  && cmake .. \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DBUILD_SHARED_LIBS=OFF \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  && make -j$(nproc) \
  && make install \
  && cd ../.. \
  && rm -rf console_bridge

RUN git clone --depth 1 --branch 1.0.5 https://github.com/ros/urdfdom_headers.git \
  && cd urdfdom_headers \
  && mkdir build && cd build \
  && cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  && make -j$(nproc) \
  && make install \
  && cd ../.. \
  && rm -rf urdfdom_headers

RUN wget https://raw.githubusercontent.com/frankarobotics/libfranka/main/.ci/urdfdom.patch \
  && git clone --depth 1 --branch 4.0.0 https://github.com/ros/urdfdom.git \
  && cd urdfdom \
  && git apply ../urdfdom.patch \
  && mkdir build && cd build \
  && cmake .. \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DBUILD_SHARED_LIBS=OFF \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  && make -j$(nproc) \
  && make install \
  && cd ../.. \
  && rm -rf urdfdom

RUN git clone --depth 1 --recurse-submodules --shallow-submodules \
  --branch v5.4.3 https://github.com/assimp/assimp.git \
  && cd assimp && mkdir build && cd build \
  && cmake .. -DBoost_USE_STATIC_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DBUILD_SHARED_LIBS=OFF -DASSIMP_BUILD_TESTS=OFF \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
  && make -j$(nproc) && make install \
  && cd ../.. && rm -rf assimp

RUN wget https://raw.githubusercontent.com/frankarobotics/libfranka/main/.ci/pinocchio.patch \
  && git clone --depth 1 --recurse-submodules --shallow-submodules \
  --branch v3.4.0 https://github.com/stack-of-tasks/pinocchio.git \
  && cd pinocchio && git apply ../pinocchio.patch \
  && mkdir build && cd build \
  && cmake .. -DBoost_USE_STATIC_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DBUILD_SHARED_LIBS=OFF -DBUILD_PYTHON_INTERFACE=OFF \
  -DBUILD_DOCUMENTATION=OFF -DBUILD_TESTING=OFF \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
  && make -j 1 && make install \
  && cd ../.. && rm -rf pinocchio

RUN git clone --recurse-submodules https://github.com/frankarobotics/libfranka.git \
  && cd libfranka \
  && git checkout ${libfranka_version} \
  && git submodule update --init --recursive \
  && mkdir build && cd build \
  && cmake -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local .. \
  && cmake --build . -- -j$(nproc) \
  && cpack -G DEB \
  && dpkg -i libfranka*.deb

COPY . /build/franka-teleop-utils
WORKDIR /build/franka-teleop-utils

RUN mkdir build \
  && cd build \
  && cmake .. \
  && make -j`nproc`

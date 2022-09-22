# export environment variable if not
source export_env_variables.bash

# set dependencies dir, all 3rdparty dependencies will be downloaded and compiled here
mkdir -p ${GF_ORB_SLAM2_SDK}
cd ${GF_ORB_SLAM2_SDK}

# set dependencies install dir, all 3rdparty libs will be installed here
mkdir ${GF_ORB_SLAM2_ROOT}

# build openblas with single-thread
cd ${GF_ORB_SLAM2_SDK}
wget https://sourceforge.net/projects/openblas/files/v0.3.5/OpenBLAS%200.3.5%20version.zip
unzip OpenBLAS\ 0.3.5\ version.zip
cd ${GF_ORB_SLAM2_SDK}/xianyi-OpenBLAS-eebc189/
make USE_THREAD=0 TARGET=HASWELL
make PREFIX=${GF_ORB_SLAM2_ROOT}/OpenBLAS install

# build armadillo
cd ${GF_ORB_SLAM2_SDK}
wget https://sourceforge.net/projects/arma/files/armadillo-11.2.4.tar.xz
tar xf armadillo-11.2.4.tar.xz
cd ${GF_ORB_SLAM2_SDK}/armadillo-11.2.4/
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH="${GF_ORB_SLAM2_ROOT}/armadillo" -DCMAKE_INSTALL_LIBDIR:PATH="lib" -DCMAKE_BUILD_TYPE:STRING="Release" -Dopenblas_LIBRARY:FILEPATH="${GF_ORB_SLAM2_ROOT}/OpenBLAS/lib/libopenblas.so" -Dopenblasp_LIBRARY:FILEPATH="${GF_ORB_SLAM2_ROOT}/OpenBLAS/lib/libopenblas.so" -Dopenblaso_LIBRARY:FILEPATH="${GF_ORB_SLAM2_ROOT}/OpenBLAS/lib/libopenblas.so" -DLAPACK_LIBRARY:FILEPATH="${GF_ORB_SLAM2_ROOT}/OpenBLAS/lib/libopenblas.so" 
make -j4
make install

# build eigen
cd ${GF_ORB_SLAM2_SDK}
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.3/eigen-3.3.3.tar.bz2
tar xf eigen-3.3.3.tar.bz2
cd ${GF_ORB_SLAM2_SDK}/eigen-3.3.3/
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH="${GF_ORB_SLAM2_ROOT}/eigen33" -DCMAKE_BUILD_TYPE:STRING="Release" # -DEIGEN_TEST_CXX11:BOOL="1" 
make -j4
make install

# use opencv 4.x provided by ROS noetic

# build Pangolin
sudo apt-get install libglew-dev

cd ${GF_ORB_SLAM2_SDK}
wget https://github.com/stevenlovegrove/Pangolin/archive/v0.6.tar.gz
tar xf v0.6.tar.gz
cd ${GF_ORB_SLAM2_SDK}/Pangolin-0.6/
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH="${GF_ORB_SLAM2_ROOT}/Pangolin" -DCMAKE_BUILD_TYPE:STRING="Release" -DEIGEN3_INCLUDE_DIR:PATH="${GF_ORB_SLAM2_ROOT}/eigen33/include/eigen3" -DLIBREALSENSE_INCLUDE_DIR:PATH="" -DLIBREALSENSE_LIBRARY:FILEPATH="" 
make -j4
make install

# install google test
cd ${GF_ORB_SLAM2_SDK}
git clone https://github.com/google/googletest.git
cd googletest
# version
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX:PATH="${GF_ORB_SLAM2_ROOT}/gtest" -DCMAKE_BUILD_TYPE:STRING="Release" ..
make -j8 && make install

# install gflags
cd ${GF_ORB_SLAM2_SDK}
git clone https://github.com/gflags/gflags.git
cd gflags
git checkout v2.2.2
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX:PATH="${GF_ORB_SLAM2_ROOT}/gflags" -DCMAKE_BUILD_TYPE:STRING="Release" ..
make -j8 && make install
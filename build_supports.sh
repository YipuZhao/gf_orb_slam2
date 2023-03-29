source export_env_variables.bash

echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DDBOW2_LIB_TYPE:STRING="SHARED" # "STATIC" # 
make -j4

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DG2O_LIB_TYPE:STRING="SHARED" # "STATIC" # 
make -j4

cd ../../SLAM++

echo "Configuring and building Thirdparty/SLAM++ ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DSLAM_P_P_EIGEN33:BOOL="1" -DSLAM_P_P_USE_OPENMP:BOOL="1" \
         -DSLAM_P_P_FLAT_SYSTEM_ALIGNED_MEMORY:BOOL="0" -DSLAM_P_P_LIB_TYPE:STRING="SHARED" # "STATIC" # 
make -j2

cd ../../../

echo "Uncompress vocabulary ..."

cd ../ORB_Data
tar -xf ORBvoc.txt.tar.gz

# cd ../gf_orb_slam2
# echo "Converting vocabulary to binary"
# ./tools/bin_vocabulary

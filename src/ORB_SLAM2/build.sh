cd /work
rm -rf ORB_SLAM2_build
mkdir ORB_SLAM2_build

cd ORB_SLAM2_build
cmake -DCMAKE_BUILD_TYPE=Release ../ORB_SLAM2 && make


# echo "Configuring and building Thirdparty/DBoW2 ..."

# cd Thirdparty/DBoW2
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j

# cd ../../g2o

# echo "Configuring and building Thirdparty/g2o ..."

# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j

# cd ../../../

# echo "Uncompress vocabulary ..."

# cd Vocabulary
# tar -xf ORBvoc.txt.tar.gz
# cd ..

# echo "Configuring and building ORB_SLAM2 ..."

# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j

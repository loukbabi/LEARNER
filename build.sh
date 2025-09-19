echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW3
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build

make -j

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

# cd Vocabulary
# tar -xf ORBvoc.txt.tar.gz
# cd ..

echo "Configuring and building Rover-SLAM ..."

mkdir build
cd build
# The command is now clean. All paths are correctly handled inside CMakeLists.txt.
cmake .. -DCMAKE_BUILD_TYPE=Release

if [ $? -ne 0 ]; then
    echo "CMake configuration failed."
    exit 1
fi

echo "Building the library..."
make -j12

echo "Installing the library..."
sudo make install

echo "------------------------------------------------"
echo "Rover-SLAM core library built and installed successfully!"
echo "------------------------------------------------"

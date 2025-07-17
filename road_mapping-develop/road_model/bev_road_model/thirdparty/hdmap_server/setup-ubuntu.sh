cd ..
echo "start install protobuf-3.5.0"
sudo apt-get install -y autoconf automake libtool curl make g++ unzip zlib1g-dev
wget https://github.com/protocolbuffers/protobuf/archive/refs/tags/v3.5.0.zip
unzip v3.5.0.zip
cd protobuf-3.5.0
./autogen.sh
./configure --prefix=/usr
make
make check
sudo make install
sudo ldconfig # refresh shared library cache.
echo "finish install protobuf-3.5.0"
cd ..
rm v3.5.0.zip

echo "start install brpc"
sudo apt-get install -y git cmake libssl-dev libgflags-dev libgoogle-glog-dev libleveldb-dev libsnappy-dev
#sudo apt-get install -y libprotobuf-dev libprotoc-dev protobuf-compiler
git clone https://github.com/apache/incubator-brpc.git
cd incubator-brpc
mkdir build
cd build
cmake -DWITH_GLOG=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
make
sudo make install
echo "finish install brpc"
cd ../../

sudo apt-get install -y libproj-dev 

echo "start install cpp_redis"
# Clone the project
git clone https://github.com/Cylix/cpp_redis.git
# Go inside the project directory
cd cpp_redis
# Get tacopie submodule
git submodule init && git submodule update
# Create a build directory and move into it
mkdir build && cd build
# Generate the Makefile using CMake
cmake .. -DCMAKE_BUILD_TYPE=Release
# Build the library
make
# Install the library
sudo make install
echo "finish install cpp_redis"
cd ../../hdmap_server
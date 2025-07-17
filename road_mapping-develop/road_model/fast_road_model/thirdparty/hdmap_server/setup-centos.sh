cd ..
echo "start install protobuf-3.5.0"
sudo yum install -y epel-release autoconf automake libtool curl make gcc-c++ unzip zlib-devel
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
sudo yum install -y git cmake openssl-devel gflags-devel glog-devel leveldb-devel
#sudo yum install -y protobuf-devel protobuf-compiler
wget https://github.com/apache/incubator-brpc/archive/refs/heads/master.zip
unzip master.zip
cd incubator-brpc-master
mkdir build
cd build
cmake -DWITH_GLOG=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
make
sudo make install
echo "finish install brpc"
cd ../../
rm master.zip

sudo yum install -y proj-devel 

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
current_dir=$(pwd)
sudo mkdir 3rd_building
sudo mkdir 3rd_output
sudo mkdir 3rd_bin

sudo tar -zxf 3rdsrc/gdal-2.4.4.tar.gz -C ./3rd_building
cd ./3rd_building/gdal-2.4.4
sudo ./configure --prefix=$current_dir/3rd_output/gdal-2.4.4
sudo make
sudo make install
cd ../../
sudo cp -r ./3rd_output/gdal-2.4.4/lib/* ./3rd_bin/
sudo cp -r ./3rd_output/gdal-2.4.4/share ./3rd_bin/

sudo tar -xjf 3rdsrc/geos-3.11.4.tar.bz2 -C ./3rd_building
cd ./3rd_building/geos-3.11.4
sudo mkdir build
cd ./build
sudo cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$current_dir/3rd_output/geos-3.11.4 ..
sudo make
sudo make install
cd ../../../
sudo cp -r ./3rd_output/geos-3.11.4/lib/* ./3rd_bin

sudo tar -zxf 3rdsrc/proj-4.9.2.tar.gz -C ./3rd_building
cd ./3rd_building/proj-4.9.2
sudo ./configure --prefix=$current_dir/3rd_output/proj-4.9.2
sudo make
sudo make install
cd ../../
sudo cp -r ./3rd_output/proj-4.9.2/lib/* ./3rd_bin

sudo tar -zxf 3rdsrc/spatialindex-src-2.0.0.tar.gz -C ./3rd_building
sudo mkdir ./3rd_output/spatialindex-src-2.0.0
sudo mkdir ./3rd_output/spatialindex-src-2.0.0/lib
cd ./3rd_building/spatialindex-src-2.0.0
sudo cmake .
sudo make
sudo cp -r ./include ../../3rd_output/spatialindex-src-2.0.0
sudo cp -r ./src/libspatialindex* ../../3rd_output/spatialindex-src-2.0.0/lib
cd ../../
sudo cp -r ./3rd_output/spatialindex-src-2.0.0/lib/* ./3rd_bin

sudo tar -zxf 3rdsrc/eigen-3.3.9.tar.gz -C ./3rd_output

sudo unzip -q 3rdsrc/json-c.zip -d ./3rd_building
cd ./3rd_building/json-c-master
sudo cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$current_dir/3rd_output/json-c
sudo make
sudo make install
cd ../../
sudo cp -r ./3rd_output/json-c/lib/* ./3rd_bin



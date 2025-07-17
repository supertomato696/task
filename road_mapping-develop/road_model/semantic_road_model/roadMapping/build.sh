#!/bin/bash

set -e

# if [ -e "build" ] && [ -e "bin/DataParser" ]; then
if [ -e "build" ]; then
  echo -e "\e[1m\e[32m----------Skipped building project----------\e[0m"
  cd build && cmake .. && make -j 12
else
  if [ -e "build" ]; then
    rm -rf build
  fi
  mkdir build && cd build
  # cmake .. && make -j$(nproc)
  cmake .. && make -j 12
  cd ../bin
fi


# rm -rf bin/
# rm -rf build/
# mkdir build
# cd build
# cmake ..
# make -j8
# cd ../bin
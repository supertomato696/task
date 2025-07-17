#!/bin/bash

echo "cur file: $0"
echo "xx.zip: $1"
echo "save_dir: $2"

pushd $2 
unzip $1
popd
#!/bin/bash

rm -rf ./cpp_code && mkdir ./cpp_code
if [ $? -ne 0 ];then
    echo "error: code1"
    exit -1
fi
rm -rf ./python_out && mkdir ./python_out
if [ $? -ne 0 ];then
    echo "error : code2"
    exit -1
fi

protoc --proto_path=. --cpp_out=./cpp_code --python_out=./python_out $(find . -name *.proto)

echo "done"
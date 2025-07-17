#!/bin/bash

echo "cur file: $0"
echo "input_file: $1"
echo "version: $2"


# 更新 
sed -i "s/version: .*/version: \"$2\"/" $1
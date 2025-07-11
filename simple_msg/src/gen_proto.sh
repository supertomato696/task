SCRIPT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
/Users/laoning/work/simple_msg/build/third_party/protobuf/Debug/protoc -I=. --cpp_out=.  example.proto 
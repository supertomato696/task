time=$(date "+%Y%m%d%H%M%S")
mv -f data_debug_log data_debug_log.${time}
mkdir data_debug_log
gdb ./bin/bev_road_model -x break.bp
#gdb ./build/accuracy_update -x break.bp

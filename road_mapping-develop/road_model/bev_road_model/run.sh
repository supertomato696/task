time=$(date "+%Y%m%d%H%M%S")
mv -f data_debug_log data_debug_log.${time}
mkdir data_debug_log
mv -f export_to_shp export_to_shp.${time}
mkdir export_to_shp
# ./bin/bev_road_model --flagfile=./conf/road_model.ini
./bin/bev_road_model --flagfile=./conf/road_model.ini
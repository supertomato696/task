#rm bev_road_model
mkdir build
cd build 
(cmake .. && make -j20) 2>&1 &>../x
ret=$?
if [ ${ret} != 0 ];then
   cd ..
   vim -c "/error:" x
fi

#chmod 777 bev_road_model
cd ..
#cp -f build/bev_road_model .
echo "build succ"



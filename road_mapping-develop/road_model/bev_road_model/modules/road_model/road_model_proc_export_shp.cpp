
#include "road_model_proc_export_shp.h"
#include <gdal_priv.h>
#include <ogrsf_frmts.h>
namespace fsdmap
{
  namespace road_model
  {

    process_frame::PROC_STATUS RoadModelExportSHP::proc(
        RoadModelSessionData *session)
    {
      // LOG_INFO("export shp type:");
      if (export_type_ == 0) {
        LOG_INFO("export shp type:{}", export_type_);
        CHECK_FATAL_PROC(export_to_shape_file_with_vectorize0(session), "export_to_shape_file_with_vectorize0");
      } else if (export_type_ == 1) {
        LOG_INFO("export shp type:{}", export_type_);
        // CHECK_FATAL_PROC(export_to_shape_file_with_vectorize1(session), "export_to_shape_file_with_vectorize1");
      } else if (export_type_ == 2) {
        LOG_INFO("export shp type:{}", export_type_);
        CHECK_FATAL_PROC(export_to_shape_file_with_topo(session), "export_to_shape_file_with_topo");
      } else {
        LOG_WARN("invalid export_type_:{}", export_type_);
      }
      return fsdmap::process_frame::PROC_STATUS_SUCC;
    }


    /******************************************************************************************/
    /********************************* 0: 矢量化 ***********************************************/
    /******************************************************************************************/
    int RoadModelExportSHP::export_to_shape_file_with_vectorize0(RoadModelSessionData* session) {
        //
        std::string cmd = "rm -r " + FLAGS_origin_shp_file_dir + "/*.dbf";
        LOG_WARN("will run:{}", cmd);
        system(cmd.c_str());
        cmd = "rm -r " + FLAGS_origin_shp_file_dir + "/*.prj";
        LOG_WARN("will run:{}", cmd)
        system(cmd.c_str());
        cmd = "rm -r " + FLAGS_origin_shp_file_dir + "/*.shp";
        LOG_WARN("will run:{}", cmd)
        system(cmd.c_str());
        cmd = "rm -r " + FLAGS_origin_shp_file_dir + "/*.shx";
        LOG_WARN("will run:{}", cmd)
        system(cmd.c_str());
        //

        export_lane_boundary_to_shp_vectorize0(session);

        export_lane_center_to_shp_vectorize0(session);

        export_road_boundary_to_shp_vectorize0(session);

        export_stop_line_to_shp_vectorize0(session);

        export_cross_walk_to_shp_vectorize0(session);

        export_road_mark_to_shp_vectorize0(session);

        export_intersection_to_shp_vectorize0(session);

        return fsdmap::SUCC;
    }

    int RoadModelExportSHP::export_lane_boundary_to_shp_vectorize0(RoadModelSessionData* session) {
        // 注册所有的驱动
        GDALAllRegister();

        // 创建数据集
        GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        GDALDataset* dataset = driver->Create(FLAGS_origin_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        if (!dataset) {
            std::cerr << "lane boundary 创建数据集失败，路径:" << FLAGS_origin_shp_file_dir << std::endl;
            return fsdmap::FAIL;;
        }

        // 创建图层
        OGRSpatialReference poSpatialRef;
        poSpatialRef.importFromEPSG(4326);
        OGRLayer* layer = dataset->CreateLayer("lane_boundary", &poSpatialRef, wkbLineString, nullptr);
        if (!layer) {
            std::cerr << "lane boundary 创建图层失败" << std::endl;
            GDALClose(dataset);
            return fsdmap::FAIL;
        }

        // 添加字段
        OGRFieldDefn fieldDefn("ID", OFTInteger);
        if (layer->CreateField(&fieldDefn) != OGRERR_NONE) {
            std::cerr << "lane boundary 创建字段失败" << std::endl;
            GDALClose(dataset);
            return fsdmap::FAIL;
        }

        // 写入数据
        int featureId = 0;
        std::string temp_id="";
        OGRLineString lineString;
        int lane_size = session->raw_lane_feature.size();
        for (int i = 0; i < lane_size; i++) {
            const auto& lane_data = session->raw_lane_feature[i];

            //如果当前line_id与temp_id相同，继续将点加入lineString
            if(lane_data->line_id == temp_id){
                Eigen::Vector3d wgs;
                session->data_processer->local2wgs(lane_data->pos, wgs);
                // double lng_gcj2;
                // double lat_gcj2;
                // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);
                double lng_gcj2 = wgs.x();
                double lat_gcj2 = wgs.y();
                // TODO:qzc gcj01
                // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
                // TODO:qzc gcj01
                // Eigen::Vector3d lng_lat_02;
                // lng_lat_02.x() = lng_gcj2;
                // lng_lat_02.y() = lat_gcj2;
                // lng_lat_02.z() = 0;
                // Eigen::Vector3d utm_02;
                // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
                // lineString.addPoint(utm_02.x(), utm_02.y());
                lineString.addPoint(lng_gcj2, lat_gcj2);
            }else{
                if(!lineString.IsEmpty() && lineString.getNumPoints() > 2){
                    OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
                    feature->SetField("ID", featureId++);
                    feature->SetGeometry(&lineString);
                    if (layer->CreateFeature(feature) != OGRERR_NONE) {
                        std::cerr << "lane boundary 创建要素失败" << std::endl;
                    }
                    OGRFeature::DestroyFeature(feature);
                }
        
                // 更新temp_id并开始新的一段线
                temp_id = lane_data->line_id;
                
                // 转换坐标
                Eigen::Vector3d wgs;
                session->data_processer->local2wgs(lane_data->pos, wgs);
                double lng_gcj2 = wgs.x();
                double lat_gcj2 = wgs.y();
                // TODO:qzc gcj01
                // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
                // TODO:qzc gcj01

                // 初始化新的lineString并加入当前点
                OGRLineString lineString_tmp;
                lineString = lineString_tmp;
                lineString.addPoint(lng_gcj2, lat_gcj2);
            }
        }
        GDALClose(dataset);

        return fsdmap::SUCC;
    }

    int RoadModelExportSHP::export_lane_center_to_shp_vectorize0(RoadModelSessionData* session) {
        // 注册所有的驱动
        GDALAllRegister();

        // 创建数据集
        GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        GDALDataset* dataset = driver->Create(FLAGS_origin_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        if (!dataset) {
            std::cerr << "lane centers 创建数据集失败，路径:" << FLAGS_origin_shp_file_dir << std::endl;
            return fsdmap::FAIL;;
        }

        // 创建图层
        OGRSpatialReference poSpatialRef;
        poSpatialRef.importFromEPSG(4326);
        OGRLayer* layer = dataset->CreateLayer("lane_center", &poSpatialRef, wkbLineString, nullptr);
        if (!layer) {
            std::cerr << "lane centers 创建图层失败" << std::endl;
            GDALClose(dataset);
            return fsdmap::FAIL;
        }

        // 添加字段
        OGRFieldDefn fieldDefn("ID", OFTInteger);
        if (layer->CreateField(&fieldDefn) != OGRERR_NONE) {
            std::cerr << "lane centers 创建字段失败" << std::endl;
            GDALClose(dataset);
            return fsdmap::FAIL;
        }  


          // 写入数据
        int featureId = 0;
        std::string temp_id="";
        OGRLineString lineString;
        int lane_size = session->raw_features.size();
        for (int i = 0; i < lane_size; i++) {
            const auto& lane_data = session->raw_features[i];

            //如果当前line_id与temp_id相同，继续将点加入lineString
            if(lane_data->line_id == temp_id){
                Eigen::Vector3d wgs;
                session->data_processer->local2wgs(lane_data->pos, wgs);
                // double lng_gcj2;
                // double lat_gcj2;
                // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);
                double lng_gcj2 = wgs.x();
                double lat_gcj2 = wgs.y();
                // TODO:qzc gcj01
                // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
                // TODO:qzc gcj01
                // Eigen::Vector3d lng_lat_02;
                // lng_lat_02.x() = lng_gcj2;
                // lng_lat_02.y() = lat_gcj2;
                // lng_lat_02.z() = 0;
                // Eigen::Vector3d utm_02;
                // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
                // lineString.addPoint(utm_02.x(), utm_02.y());
                lineString.addPoint(lng_gcj2, lat_gcj2);
            }else{
                if(!lineString.IsEmpty() && lineString.getNumPoints() > 2){
                    OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
                    feature->SetField("ID", featureId++);
                    feature->SetGeometry(&lineString);
                    if (layer->CreateFeature(feature) != OGRERR_NONE) {
                        std::cerr << "lane center 创建要素失败" << std::endl;
                    }
                    OGRFeature::DestroyFeature(feature);
                }
        
                // 更新temp_id并开始新的一段线
                temp_id = lane_data->line_id;
                
                // 转换坐标
                Eigen::Vector3d wgs;
                session->data_processer->local2wgs(lane_data->pos, wgs);
                double lng_gcj2 = wgs.x();
                double lat_gcj2 = wgs.y();
                // TODO:qzc gcj01
                // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
                // TODO:qzc gcj01

                // 初始化新的lineString并加入当前点
                OGRLineString lineString_tmp;
                lineString = lineString_tmp;
                lineString.addPoint(lng_gcj2, lat_gcj2);
            }
        }
        GDALClose(dataset);


        return fsdmap::SUCC;
    }


    int RoadModelExportSHP::export_road_boundary_to_shp_vectorize0(RoadModelSessionData* session) {
        // 注册所有的驱动
        GDALAllRegister();

        // 创建数据集
        GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        GDALDataset* dataset = driver->Create(FLAGS_origin_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        if (!dataset) {
            std::cerr << "road boundaries 创建数据集失败，路径:" << FLAGS_origin_shp_file_dir << std::endl;
            return fsdmap::FAIL;;
        }

        // 创建图层
        OGRSpatialReference poSpatialRef;
        poSpatialRef.importFromEPSG(4326);
        OGRLayer* layer = dataset->CreateLayer("road_boundary", &poSpatialRef, wkbLineString, nullptr);
        if (!layer) {
            std::cerr << "road boundaries 创建图层失败" << std::endl;
            GDALClose(dataset);
            return fsdmap::FAIL;
        }

        // 添加字段
        OGRFieldDefn fieldDefn("ID", OFTInteger);
        if (layer->CreateField(&fieldDefn) != OGRERR_NONE) {
            std::cerr << "road boundaries 创建字段失败" << std::endl;
            GDALClose(dataset);
            return fsdmap::FAIL;
        }
        

        // 写入数据
        int featureId = 0;
        std::string temp_id="";
        OGRLineString lineString;
        int lane_size = session->raw_boundary_feature.size();
        for (int i = 0; i < lane_size; i++) {
            const auto& lane_data = session->raw_boundary_feature[i];

            //如果当前line_id与temp_id相同，继续将点加入lineString
            if(lane_data->line_id == temp_id){
                Eigen::Vector3d wgs;
                session->data_processer->local2wgs(lane_data->pos, wgs);
                // double lng_gcj2;
                // double lat_gcj2;
                // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);
                double lng_gcj2 = wgs.x();
                double lat_gcj2 = wgs.y();
                // TODO:qzc gcj01
                // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
                // TODO:qzc gcj01

                // Eigen::Vector3d lng_lat_02;
                // lng_lat_02.x() = lng_gcj2;
                // lng_lat_02.y() = lat_gcj2;
                // lng_lat_02.z() = 0;
                // Eigen::Vector3d utm_02;
                // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
                // lineString.addPoint(utm_02.x(), utm_02.y());
                lineString.addPoint(lng_gcj2, lat_gcj2);
            }else{
                if(!lineString.IsEmpty() && lineString.getNumPoints() > 2){
                    OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
                    feature->SetField("ID", featureId++);
                    feature->SetGeometry(&lineString);
                    if (layer->CreateFeature(feature) != OGRERR_NONE) {
                        std::cerr << "road boundaries 创建要素失败" << std::endl;
                    }
                    OGRFeature::DestroyFeature(feature);
                }
        
                // 更新temp_id并开始新的一段线
                temp_id = lane_data->line_id;
                
                // 转换坐标
                Eigen::Vector3d wgs;
                session->data_processer->local2wgs(lane_data->pos, wgs);
                double lng_gcj2 = wgs.x();
                double lat_gcj2 = wgs.y();
                // TODO:qzc gcj01
                // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
                // TODO:qzc gcj01

                // 初始化新的lineString并加入当前点
                OGRLineString lineString_tmp;
                lineString = lineString_tmp;
                lineString.addPoint(lng_gcj2, lat_gcj2);
            }
        }
        GDALClose(dataset);


        return fsdmap::SUCC;
    }

    int RoadModelExportSHP::export_stop_line_to_shp_vectorize0(RoadModelSessionData* session) {
        // 注册所有的驱动
        GDALAllRegister();

        // 创建数据集
        GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        GDALDataset* dataset = driver->Create(FLAGS_origin_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        if (!dataset) {
            std::cerr << "stop lines 创建数据集失败，路径:" << FLAGS_origin_shp_file_dir << std::endl;
            return fsdmap::FAIL;;
        }

      // 创建图层
        OGRSpatialReference poSpatialRef;
        poSpatialRef.importFromEPSG(4326);
        OGRLayer *layer = dataset->CreateLayer("stop_line", &poSpatialRef, wkbLineString, nullptr);
        if (!layer)
        {
          std::cerr << "stop lines 创建图层失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加ID字段
        OGRFieldDefn fieldDefn("ID", OFTInteger);
        if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
        {
          std::cerr << "stop lines 创建字段失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 停止线TYPE 字段
        OGRFieldDefn lineTypeFieldDefn("TYPE", OFTInteger);
        if (layer->CreateField(&lineTypeFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "stop lines 创建字段 TYPE 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 color 字段
        OGRFieldDefn colorFieldDefn("COLOR", OFTInteger);
        if (layer->CreateField(&colorFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "stop lines 创建字段 COLOR 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }
        // 添加 SHAPE_TYPE 字段
        OGRFieldDefn shapeTypeFieldDefn("SHAPE_TYPE", OFTInteger);
        if (layer->CreateField(&shapeTypeFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "stop lines 创建字段 SHAPE 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 LANE_ID 字段
        OGRFieldDefn laneIdFieldDefn("LANE_ID", OFTString);
        if (layer->CreateField(&laneIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "stop lines 创建字段 LANE_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 FRAME_ID 字段
        OGRFieldDefn frameIdFieldDefn("FRAME_ID", OFTString);
        if (layer->CreateField(&frameIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "stop lines 创建字段 FRAME_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 VERSION 字段
        OGRFieldDefn versionFieldDefn("VERSION", OFTString);
        if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "stop lines 创建字段 VERSION 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        for (const auto& object : session->raw_object_ret_list) {
          if (object->ele_type == 6) {
            OGRLineString lineString;
            for (const auto& point : object->list) {
                Eigen::Vector3d wgs;
                session->data_processer->local2wgs(point->pos, wgs);
                // double lng_gcj2;
                // double lat_gcj2;
                // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

                double lng_gcj2 = wgs.x();
                double lat_gcj2 = wgs.y();
                // TODO:qzc gcj01
                // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
                // TODO:qzc gcj01

                // Eigen::Vector3d lng_lat_02;
                // lng_lat_02.x() = lng_gcj2;
                // lng_lat_02.y() = lat_gcj2;
                // lng_lat_02.z() = 0;
                // Eigen::Vector3d utm_02;
                // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
                // lineString.addPoint(utm_02.x(), utm_02.y());

                lineString.addPoint(lng_gcj2, lat_gcj2);

                // lineString.addPoint(point->pos.x(), point->pos.y());
            }

            OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
            feature->SetField("ID", object->obj_id);
            feature->SetField("TYPE", std::stoi(object->type));
            // feature->SetField("LANE_ID", object->lane_id);  
            // feature->SetField("COLOR", object->color);  
            // feature->SetField("SHAPE", object->shape);  
            feature->SetGeometry(&lineString);

            if (layer->CreateFeature(feature) != OGRERR_NONE) {
                std::cerr << "stop lines 创建要素失败" << std::endl;
            }

            OGRFeature::DestroyFeature(feature);
          }
        }

        // 关闭数据集
        GDALClose(dataset);

        return fsdmap::SUCC;
    }

    int RoadModelExportSHP::export_cross_walk_to_shp_vectorize0(RoadModelSessionData* session) {
        // 注册所有的驱动
        GDALAllRegister();

        // 创建数据集
        GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        GDALDataset* dataset = driver->Create(FLAGS_origin_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        if (!dataset) {
            std::cerr << "cross walks 创建数据集失败，路径:" << FLAGS_origin_shp_file_dir << std::endl;
            return fsdmap::FAIL;;
        }

        // 创建图层
        OGRSpatialReference poSpatialRef;
        poSpatialRef.importFromEPSG(4326);
        OGRLayer *layer = dataset->CreateLayer("cross_walk", &poSpatialRef, wkbPolygon, nullptr);
        if (!layer)
        {
          std::cerr << "cross walks 创建图层失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加ID字段
        OGRFieldDefn fieldDefn("ID", OFTInteger);
        if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
        {
          std::cerr << "cross walks 创建ID字段失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 LANE_ID 字段
        OGRFieldDefn laneIdFieldDefn("LANE_ID", OFTString);
        if (layer->CreateField(&laneIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "cross walk创建字段 LANE_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }
        // 添加 JUNC_ID 字段
        OGRFieldDefn juncIdFieldDefn("JUNC_ID", OFTString);
        if (layer->CreateField(&juncIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "cross walk创建字段 JUNC_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 FRAME_ID 字段
        OGRFieldDefn frameIdFieldDefn("FRAME_ID", OFTString);
        if (layer->CreateField(&frameIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "cross walk创建字段 FRAME_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 VERSION 字段
        OGRFieldDefn versionFieldDefn("VERSION", OFTString);
        if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "cross walk创建字段 VERSION 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }


        // 写入数据
        int featureId = 0;
        for (const auto& object : session->raw_object_ret_list) {
            if (object->ele_type == 5) {
                // 创建一个多边形对象来表示块状区域
                OGRPolygon polygon;
                // 创建一个线性环，用于定义多边形的边界
                OGRLinearRing* ring = new OGRLinearRing();
                for (const auto& point : object->list) {
                    Eigen::Vector3d wgs;
                    session->data_processer->local2wgs(point->pos, wgs);

                    double lng_gcj2 = wgs.x();
                    double lat_gcj2 = wgs.y();
                    // TODO:qzc gcj01
                    // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
                    // TODO:qzc gcj01
                    ring->addPoint(lng_gcj2, lat_gcj2);
                }

                // 确保线性环闭合
                ring->closeRings();
                // 将线性环添加到多边形中
                polygon.addRing(ring);

                // 创建要素并设置字段值
                OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
                feature->SetField("ID", object->obj_id);
                // feature->SetField("LANE_ID", object->lane_id);  // 

                // 设置要素的几何对象为多边形
                feature->SetGeometry(&polygon);

                // 将要素添加到图层中
                if (layer->CreateFeature(feature) != OGRERR_NONE) {
                    std::cerr << "cross walks 创建要素失败" << std::endl;
                }

                OGRFeature::DestroyFeature(feature);
                delete ring;  
            }
        }

        // 关闭数据集
        GDALClose(dataset);

        return fsdmap::SUCC;
    }


    int RoadModelExportSHP::export_road_mark_to_shp_vectorize0(RoadModelSessionData* session) {
        // 注册所有的驱动
        GDALAllRegister();

        // 创建数据集
        GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        GDALDataset* dataset = driver->Create(FLAGS_origin_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        if (!dataset) {
            std::cerr << "arrow创建数据集失败，路径:" << FLAGS_origin_shp_file_dir << std::endl;
            return fsdmap::FAIL;;
        }

        // 设置字符编码为 UTF-8，生成 .cpg 文件
        dataset->SetMetadataItem("CHARSET", "UTF-8");
        
        // 创建arrow图层
        OGRSpatialReference poSpatialRef;
        poSpatialRef.importFromEPSG(4326);
        OGRLayer *layer = dataset->CreateLayer("arrow", &poSpatialRef, wkbPolygon, nullptr);
        if (!layer)
        {
          std::cerr << "arrow创建图层失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加字段
        OGRFieldDefn fieldDefn("ID", OFTInteger);
        if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
        {
          std::cerr << "arrow创建字段失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 箭头TYPE 字段
        OGRFieldDefn typeFieldDefn("TYPE", OFTString);
        if (layer->CreateField(&typeFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "arrow 创建字段 TYPE 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 LANE_ID 字段
        OGRFieldDefn laneIdFieldDefn("LANE_ID", OFTString);
        if (layer->CreateField(&laneIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "arrow 创建字段 LANE_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 FRAME_ID 字段
        OGRFieldDefn frameIdFieldDefn("FRAME_ID", OFTString);
        if (layer->CreateField(&frameIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "arrow 创建字段 FRAME_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 VERSION 字段
        OGRFieldDefn versionFieldDefn("VERSION", OFTString);
        if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "arrow 创建字段 VERSION 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }


        // 写入数据
        for (const auto& object : session->raw_object_ret_list) {
          if (object->ele_type == 3) {
            // 创建一个多边形对象来表示块状区域
            OGRPolygon polygon;
            // 创建一个线性环，用于定义多边形的边界
            OGRLinearRing* ring = new OGRLinearRing();
            for (const auto& point : object->list) {
                Eigen::Vector3d wgs;
                session->data_processer->local2wgs(point->pos, wgs);
                // double lng_gcj2;
                // double lat_gcj2;
                // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

                double lng_gcj2 = wgs.x();
                double lat_gcj2 = wgs.y();
                // TODO:qzc gcj01
                // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
                // TODO:qzc gcj01

                // Eigen::Vector3d lng_lat_02;
                // lng_lat_02.x() = lng_gcj2;
                // lng_lat_02.y() = lat_gcj2;
                // lng_lat_02.z() = 0;
                // Eigen::Vector3d utm_02;
                // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
                // lineString.addPoint(utm_02.x(), utm_02.y());

                ring->addPoint(lng_gcj2, lat_gcj2);

                // lineString.addPoint(point->pos.x(), point->pos.y());
            }
            // 确保线性环闭合
            ring->closeRings();
            // 将线性环添加到多边形中
            polygon.addRing(ring);

            OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
            feature->SetField("ID", object->obj_id);
            feature->SetField("TYPE", object->type.c_str());
            // feature->SetField("LANE_ID", object->lane_id);

            // 设置要素的几何对象为多边形
            feature->SetGeometry(&polygon);

            if (layer->CreateFeature(feature) != OGRERR_NONE) {
                std::cerr << "arrow创建要素失败" << std::endl;
            }

            OGRFeature::DestroyFeature(feature);
            delete ring; 
          }
        }

        // 关闭数据集
        GDALClose(dataset);

        return fsdmap::SUCC;
    }

    int RoadModelExportSHP::export_intersection_to_shp_vectorize0(RoadModelSessionData *session){
        // 注册所有的驱动
        GDALAllRegister();

        // 创建数据集
        GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        GDALDataset *dataset = driver->Create(FLAGS_origin_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        if (!dataset)
        {
          std::cerr << "junction 创建数据集失败，路径:" << FLAGS_origin_shp_file_dir << std::endl;
          return fsdmap::FAIL;
          ;
        }

      // 创建图层
        OGRSpatialReference poSpatialRef;
        poSpatialRef.importFromEPSG(4326);
        OGRLayer *layer = dataset->CreateLayer("junction", &poSpatialRef, wkbPolygon, nullptr);
        if (!layer)
        {
          std::cerr << "junction 创建图层失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }
        // 添加ID字段
        OGRFieldDefn fieldDefn("ID", OFTInteger);
        if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加TYPE 字段
        OGRFieldDefn typeFieldDefn("TYPE", OFTInteger);
        if (layer->CreateField(&typeFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段 TYPE 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 IS_EFFECT  字段
        OGRFieldDefn iseffectFieldDefn("IS_EFFECT", OFTInteger);
        if (layer->CreateField(&iseffectFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段 IS_EFFECT 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 lane road  字段
        OGRFieldDefn laneIdFieldDefn("LANE_ID", OFTString);
        if (layer->CreateField(&laneIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段 LANE_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        OGRFieldDefn inRoadFieldDefn("IN_ROAD", OFTString);
        if (layer->CreateField(&inRoadFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段 IN_ROAD 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }
        OGRFieldDefn outRoadFieldDefn("OUT_ROAD", OFTString);
        if (layer->CreateField(&outRoadFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段 IN_ROAD 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        OGRFieldDefn inLaneFieldDefn("IN_LANE", OFTString);
        if (layer->CreateField(&inLaneFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段 IN_LANE 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        OGRFieldDefn outLaneFieldDefn("OUT_LANE", OFTString);
        if (layer->CreateField(&outLaneFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段 OUT_LANE 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 FRAME_ID 字段
        OGRFieldDefn frameIdFieldDefn("FRAME_ID", OFTInteger);
        if (layer->CreateField(&frameIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction创建字段 FRAME_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 VERSION 字段
        OGRFieldDefn versionFieldDefn("VERSION", OFTString);
        if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段 VERSION 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }



        // 写入数据
        for (const auto &inter : session->raw_intersections)
        {
          // 创建一个多边形对象来表示块状区域
          OGRPolygon polygon;
          // 创建一个线性环，用于定义多边形的边界
          OGRLinearRing *ring = new OGRLinearRing();
          for (auto &pt : inter->point_info)
          {
            Eigen::Vector3d wgs;
            session->data_processer->local2wgs(pt->pos, wgs);
            // double lng_gcj2;
            // double lat_gcj2;
            // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

            double lng_gcj2 = wgs.x();
            double lat_gcj2 = wgs.y();
            // TODO:qzc gcj01
            // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
            // TODO:qzc gcj01

            // Eigen::Vector3d lng_lat_02;
            // lng_lat_02.x() = lng_gcj2;
            // lng_lat_02.y() = lat_gcj2;
            // lng_lat_02.z() = 0;
            // Eigen::Vector3d utm_02;
            // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
            // lineString.addPoint(utm_02.x(), utm_02.y());

            ring->addPoint(lng_gcj2, lat_gcj2);

            // lineString.addPoint(pt->pos.x(), pt->pos.y());
          }
          // 确保线性环闭合
          ring->closeRings();
          polygon.addRing(ring);

          OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
          feature->SetField("ID", inter->id);
          // feature->SetField("LANE_ID", object->lane_id);  //
          // feature->SetField("IN_LANE", object->lane_id);  //
          // feature->SetField("OUT_LANE", object->lane_id);  //
          // feature->SetField("IN_ROAD", object->lane_id);  //
          // feature->SetField("OUT_ROAD", object->lane_id);  //
          feature->SetGeometry(&polygon);

          if (layer->CreateFeature(feature) != OGRERR_NONE)
          {
            std::cerr << "junction 创建要素失败" << std::endl;
          }

          OGRFeature::DestroyFeature(feature);
        }
        // 关闭数据集
        GDALClose(dataset);
        return fsdmap::SUCC;
    }

    /******************************************************************************************/
    /********************************* 1：车道线去重后的矢量化 **********************************/
    /******************************************************************************************/
    int RoadModelExportSHP::export_to_shape_file_with_vectorize1(RoadModelSessionData* session) {
        //
        std::string cmd = "rm -r " + FLAGS_mid_shp_file_dir + "/*.dbf";
        LOG_WARN("will run:{}", cmd);
        system(cmd.c_str());
        cmd = "rm -r " + FLAGS_mid_shp_file_dir + "/*.prj";
        LOG_WARN("will run:{}", cmd)
        system(cmd.c_str());
        cmd = "rm -r " + FLAGS_mid_shp_file_dir + "/*.shp";
        LOG_WARN("will run:{}", cmd)
        system(cmd.c_str());
        cmd = "rm -r " + FLAGS_mid_shp_file_dir + "/*.shx";
        LOG_WARN("will run:{}", cmd)
        system(cmd.c_str());
        //

        export_lane_boundary_to_shp_vectorize1(session);

        export_lane_center_to_shp_vectorize1(session);

        export_road_boundary_to_shp_vectorize1(session);

        export_stop_line_to_shp_vectorize1(session);

        export_cross_walk_to_shp_vectorize1(session);

        export_road_mark_to_shp_vectorize1(session);

        export_intersection_to_shp_vectorize1(session);

        return fsdmap::SUCC;
    }

    int RoadModelExportSHP::export_lane_boundary_to_shp_vectorize1(RoadModelSessionData* session, int flag)
    {
      // 注册所有的驱动
      GDALAllRegister();

      std::string path_str = flag == 0 ? FLAGS_mid_shp_file_dir : FLAGS_shp_file_dir;
      int offfset = flag == 0 ? 0 : 3 * 1e8;
      // 创建数据集
      GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
      // GDALDataset *dataset = driver->Create(FLAGS_mid_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
      GDALDataset *dataset = driver->Create(path_str.c_str(), 0, 0, 0, GDT_Unknown, nullptr);

      if (!dataset) {
          std::cerr << "lane boundary 创建数据集失败，路径:" << FLAGS_mid_shp_file_dir << std::endl;
          return fsdmap::FAIL;;
      }

      // 创建图层
      OGRSpatialReference poSpatialRef;
      poSpatialRef.importFromEPSG(4326);
      OGRLayer* layer = dataset->CreateLayer("lane_boundary_vectorize", &poSpatialRef, wkbLineString, nullptr);
      if (!layer) {
          std::cerr << "lane boundary 创建图层失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
      }

      // 添加 ID 字段
      OGRFieldDefn fieldDefn("ID", OFTInteger);
      if (layer->CreateField(&fieldDefn) != OGRERR_NONE) {
          std::cerr << "lane boundary 创建字段失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
      }

      // 添加 color 字段
      OGRFieldDefn colorFieldDefn("COLOR", OFTInteger);
      if (layer->CreateField(&colorFieldDefn) != OGRERR_NONE)
      {
        std::cerr << "lane boundary  创建字段 COLOR 失败" << std::endl;
        GDALClose(dataset);
        return fsdmap::FAIL;
      }
      // 添加 shape 字段
      OGRFieldDefn shapetypeFieldDefn("SHAPE_TYPE", OFTInteger);
      if (layer->CreateField(&shapetypeFieldDefn) != OGRERR_NONE)
      {
        std::cerr << "lane boundary  创建字段 SHAPE_TYPE 失败" << std::endl;
        GDALClose(dataset);
        return fsdmap::FAIL;
      }

      // 添加 lane_id 字段
      OGRFieldDefn laneidFieldDefn("LANE_ID", OFTString);
      // safetyFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
      if (layer->CreateField(&laneidFieldDefn) != OGRERR_NONE)
      {
        std::cerr << "lane boundary  创建字段 LANE_ID 失败" << std::endl;
        GDALClose(dataset);
        return fsdmap::FAIL;
      }

      // 添加 pre_id 字段
      OGRFieldDefn preidFieldDefn("PRE_ID", OFTString);
      // preidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
      if (layer->CreateField(&preidFieldDefn) != OGRERR_NONE)
      {
        std::cerr << "lane boundary  创建字段 PRE_ID 失败" << std::endl;
        GDALClose(dataset);
        return fsdmap::FAIL;
      }

      // 添加 next_id 字段
      OGRFieldDefn nextidFieldDefn("NEXT_ID", OFTString);
      // nextidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
      if (layer->CreateField(&nextidFieldDefn) != OGRERR_NONE)
      {
        std::cerr << "lane boundary  创建字段 NEXT_ID 失败" << std::endl;
        GDALClose(dataset);
        return fsdmap::FAIL;
      }

      // 添加 frame_id 字段
      OGRFieldDefn frameidFieldDefn("FRAME_ID", OFTInteger);
      // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
      if (layer->CreateField(&frameidFieldDefn) != OGRERR_NONE)
      {
        std::cerr << "lane boundary  创建字段 FRAME_ID 失败" << std::endl;
        GDALClose(dataset);
        return fsdmap::FAIL;
      }

      // 添加 version 字段
      OGRFieldDefn versionFieldDefn("VERSION", OFTString);
      // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
      if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
      {
        std::cerr << "lane boundary  创建字段 VERSION 失败" << std::endl;
        GDALClose(dataset);
        return fsdmap::FAIL;
      }

        auto lit1 = session->key_pose_map.begin();
        if (lit1 != session->key_pose_map.end()) {
            auto trail_ptr = &lit1->second;

            // 写入数据
            int featureId = 0;
            for (const auto &object : trail_ptr->lane_line_group_list)
            {
              {
                //任务框边界点
                if(object->list.size() > 1){
                  for (int i = 1; i < object->list.size(); ++i) {
                      auto &current_point = object->list[i]; // 当前点A
                      auto &previous_point = object->list[i - 1]; // 上一个点B

                      bool current_flag = alg::point_in_polygon(current_point->pos, session->task_polygon_vec3d);
                      bool previous_flag = alg::point_in_polygon(previous_point->pos, session->task_polygon_vec3d);

                      // 如果当前点和前一个点的 flag 标志不同，则需要计算交点
                      if (current_flag != previous_flag) {
                          std::vector<Eigen::Vector3d> cross_points;
                          bool is_intersect = alg::get_cross_point_with_polygon(previous_point->pos, current_point->pos, session->task_polygon_vec3d, cross_points);
                        
                          if (is_intersect) {
                              // 将交点存入两个点之间
                              // std::shared_ptr<LaneLineSample> new_point = std::make_shared<LaneLineSample>();
                              auto new_point = session->add_ptr(session->lane_line_sample_ptr);
                              new_point->init(current_point.get());

                              new_point->pos = cross_points[0]; 
                              new_point->is_intersection_point = true;

                              object->list.insert(object->list.begin() + i, {new_point});
                              ++i; // 跳过插入的交点，避免重复处理
                          }
                      }
                  }
                }
                //剔除
                OGRLineString lineString;
                for (const auto &point : object->list)
                {
                  if (point->is_intersection_point == false) {
                    if(alg::point_in_polygon(point->pos, session->task_polygon_vec3d) == false){
                      continue;
                    }
                  }

                  Eigen::Vector3d wgs;
                  session->data_processer->local2wgs(point->pos, wgs);
                  double lng_gcj2 = wgs.x();
                  double lat_gcj2 = wgs.y();

                  lineString.addPoint(lng_gcj2, lat_gcj2);

                }

                OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
                feature->SetField("ID", offfset + featureId++);
                // feature->SetField("TYPE", std::stoi(object->type));
                // // feature->SetField("LANE_ID", object->lane_id);
                // feature->SetField("COLOR", object->color);
                // // feature->SetField("SHAPE", object->shape);
                // // feature->SetField("LENGTH", object->length);
                // feature->SetField("FRAME_ID", );  // 暂时没有
                feature->SetField("VERSION", session->version.c_str());
                feature->SetGeometry(&lineString);

                if (layer->CreateFeature(feature) != OGRERR_NONE)
                {
                  std::cerr << "stop line 创建要素失败" << std::endl;
                }

                OGRFeature::DestroyFeature(feature);
              }
            }
        }

        GDALClose(dataset);

        return fsdmap::SUCC;
    }

    int RoadModelExportSHP::export_lane_center_to_shp_vectorize1(RoadModelSessionData* session) {
        // 注册所有的驱动
        GDALAllRegister();

        // 创建数据集
        GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        GDALDataset* dataset = driver->Create(FLAGS_mid_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        if (!dataset) {
            std::cerr << "lane centers 创建数据集失败，路径:" << FLAGS_mid_shp_file_dir << std::endl;
            return fsdmap::FAIL;;
        }

        // 创建图层
        OGRSpatialReference poSpatialRef;
        poSpatialRef.importFromEPSG(4326);
        OGRLayer* layer = dataset->CreateLayer("lane_center", &poSpatialRef, wkbLineString, nullptr);
        if (!layer) {
            std::cerr << "lane centers 创建图层失败" << std::endl;
            GDALClose(dataset);
            return fsdmap::FAIL;
        }

        // 添加字段
        OGRFieldDefn fieldDefn("ID", OFTInteger);
        if (layer->CreateField(&fieldDefn) != OGRERR_NONE) {
            std::cerr << "lane centers 创建字段失败" << std::endl;
            GDALClose(dataset);
            return fsdmap::FAIL;
        }  


        auto lit1 = session->key_pose_map.begin();
        if (lit1 != session->key_pose_map.end()) {
            auto trail_ptr = &lit1->second;
            // 写入数据
            int featureId = 0;
            for (const auto &object : trail_ptr->lane_center_line_group_list)
            {
              {
                OGRLineString lineString;
                for (const auto &point : object->list)
                {
                  Eigen::Vector3d wgs;
                  session->data_processer->local2wgs(point->pos, wgs);
                  // double lng_gcj2;
                  // double lat_gcj2;
                  // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

                  double lng_gcj2 = wgs.x();
                  double lat_gcj2 = wgs.y();
                  // TODO:qzc gcj01
                  // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
                  // TODO:qzc gcj01

                  // Eigen::Vector3d lng_lat_02;
                  // lng_lat_02.x() = lng_gcj2;
                  // lng_lat_02.y() = lat_gcj2;
                  // lng_lat_02.z() = 0;
                  // Eigen::Vector3d utm_02;
                  // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
                  // lineString.addPoint(utm_02.x(), utm_02.y());

                  lineString.addPoint(lng_gcj2, lat_gcj2);

                  // lineString.addPoint(point->pos.x(), point->pos.y());
                }

                OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
                feature->SetField("ID", featureId++);
                // feature->SetField("ID", object->obj_id);
                // feature->SetField("TYPE", std::stoi(object->type));
                // // feature->SetField("LANE_ID", object->lane_id);
                // // feature->SetField("COLOR", object->color);
                // // feature->SetField("SHAPE", object->shape);
                // // feature->SetField("LENGTH", object->length);
                // feature->SetField("VERSION", session->version.c_str());
                feature->SetGeometry(&lineString);

                if (layer->CreateFeature(feature) != OGRERR_NONE)
                {
                  std::cerr << "stop line 创建要素失败" << std::endl;
                }

                OGRFeature::DestroyFeature(feature);
              }
            }
        }

        GDALClose(dataset);


        return fsdmap::SUCC;
    }


    int RoadModelExportSHP::export_road_boundary_to_shp_vectorize1(RoadModelSessionData* session, int flag) {
        // 注册所有的驱动
        GDALAllRegister();

        std::string path_str = flag == 0 ? FLAGS_mid_shp_file_dir : FLAGS_shp_file_dir;
        int offset = flag == 0 ? 0 : 2 * 1e8;
        // 创建数据集
        GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        // GDALDataset *dataset = driver->Create(FLAGS_mid_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        GDALDataset *dataset = driver->Create(path_str.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        if (!dataset) {
            std::cerr << "road boundaries 创建数据集失败，路径:" << FLAGS_mid_shp_file_dir << std::endl;
            return fsdmap::FAIL;;
        }

        // 创建图层
        OGRSpatialReference poSpatialRef;
        poSpatialRef.importFromEPSG(4326);
        OGRLayer* layer = dataset->CreateLayer("road_boundary_vectorize", &poSpatialRef, wkbLineString, nullptr);
        if (!layer) {
            std::cerr << "road boundaries 创建图层失败" << std::endl;
            GDALClose(dataset);
            return fsdmap::FAIL;
        }

        // 添加 ID 字段
        OGRFieldDefn fieldDefn("ID", OFTInteger);
        if (layer->CreateField(&fieldDefn) != OGRERR_NONE) {
            std::cerr << "road boundaries 创建字段失败" << std::endl;
            GDALClose(dataset);
            return fsdmap::FAIL;
        }

        // 添加 color 字段
        OGRFieldDefn colorFieldDefn("COLOR", OFTInteger);
        if (layer->CreateField(&colorFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "road boundary  创建字段 COLOR 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加type字段
        OGRFieldDefn typefieldDefn("TYPE", OFTInteger);
        if (layer->CreateField(&typefieldDefn) != OGRERR_NONE)
        {
          std::cerr << "road boundary 创建字段 TYPE 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 is_safety 字段
        OGRFieldDefn safetyFieldDefn("IS_SAFETY", OFTInteger);
        // safetyFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
        if (layer->CreateField(&safetyFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "road boundary  创建字段 IS_SAFETY 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 road_id 字段
        OGRFieldDefn roadidFieldDefn("ROAD_ID", OFTInteger);
        // safetyFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
        if (layer->CreateField(&roadidFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "road boundary  创建字段 ROAD_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 pre_id 字段
        OGRFieldDefn preidFieldDefn("PRE_ID", OFTString);
        // preidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
        if (layer->CreateField(&preidFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "road boundary  创建字段 PRE_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 next_id 字段
        OGRFieldDefn nextidFieldDefn("NEXT_ID", OFTString);
        // nextidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
        if (layer->CreateField(&nextidFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "road boundary  创建字段 NEXT_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 frame_id 字段
        OGRFieldDefn frameidFieldDefn("FRAME_ID", OFTInteger);
        // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
        if (layer->CreateField(&frameidFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "road boundary  创建字段 FRAME_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 version 字段
        OGRFieldDefn versionFieldDefn("VERSION", OFTString);
        // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
        if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "road boundary  创建字段 VERSION 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        auto lit1 = session->key_pose_map.begin();
        if (lit1 != session->key_pose_map.end()) {
            auto trail_ptr = &lit1->second;
            // 写入数据
            int featureId = 0;
            for (const auto &object : trail_ptr->boundary_line_group_list)
            {
              {
                //任务框边界点
                if(object->list.size() > 1){
                  for (int i = 1; i < object->list.size(); ++i) {
                      auto &current_point = object->list[i]; // 当前点A
                      auto &previous_point = object->list[i - 1]; // 上一个点B

                      bool current_flag = alg::point_in_polygon(current_point->pos, session->task_polygon_vec3d);
                      bool previous_flag = alg::point_in_polygon(previous_point->pos, session->task_polygon_vec3d);

                      // 如果当前点和前一个点的 flag 标志不同，则需要计算交点
                      if (current_flag != previous_flag) {
                          std::vector<Eigen::Vector3d> cross_points;
                          bool is_intersect = alg::get_cross_point_with_polygon(previous_point->pos, current_point->pos, session->task_polygon_vec3d, cross_points);
                        
                          if (is_intersect) {
                              // 将交点存入两个点之间
                              // std::shared_ptr<BoundaryFeature> new_point = std::make_shared<BoundaryFeature>();
                              auto new_point = session->add_ptr(session->boundary_feature_ptr);
                              new_point->init(current_point.get());
                              
                              new_point->pos = cross_points[0]; 
                              new_point->is_intersection_point = true;

                              object->list.insert(object->list.begin() + i, {new_point});
                              ++i; // 跳过插入的交点，避免重复处理
                          }
                      }
                  }
                }
                //剔除

                OGRLineString lineString;
                for (const auto &point : object->list)
                {
                  if (point->is_intersection_point == false) {
                    if(alg::point_in_polygon(point->pos, session->task_polygon_vec3d) == false){
                      continue;
                    }
                  }

                  Eigen::Vector3d wgs;
                  session->data_processer->local2wgs(point->pos, wgs);
                  // double lng_gcj2;
                  // double lat_gcj2;
                  // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

                  double lng_gcj2 = wgs.x();
                  double lat_gcj2 = wgs.y();
                  // TODO:qzc gcj01
                  // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
                  // TODO:qzc gcj01

                  // Eigen::Vector3d lng_lat_02;
                  // lng_lat_02.x() = lng_gcj2;
                  // lng_lat_02.y() = lat_gcj2;
                  // lng_lat_02.z() = 0;
                  // Eigen::Vector3d utm_02;
                  // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
                  // lineString.addPoint(utm_02.x(), utm_02.y());

                  lineString.addPoint(lng_gcj2, lat_gcj2);

                  // lineString.addPoint(point->pos.x(), point->pos.y());
                }

                OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
                feature->SetField("ID", offset + featureId++);
                // feature->SetField("ID", object->obj_id);
                // feature->SetField("TYPE", std::stoi(object->type));
                // // feature->SetField("LANE_ID", object->lane_id);
                // // feature->SetField("COLOR", object->color);
                // // feature->SetField("SHAPE", object->shape);
                // // feature->SetField("LENGTH", object->length);
                // feature->SetField("FRAME_ID", );  // 暂时没有
                feature->SetField("VERSION", session->version.c_str());
                feature->SetGeometry(&lineString);

                if (layer->CreateFeature(feature) != OGRERR_NONE)
                {
                  std::cerr << "stop line 创建要素失败" << std::endl;
                }

                OGRFeature::DestroyFeature(feature);
              }
            }
        }
        

        GDALClose(dataset);


        return fsdmap::SUCC;
    }

    int RoadModelExportSHP::export_stop_line_to_shp_vectorize1(RoadModelSessionData* session) {
        // 注册所有的驱动
        GDALAllRegister();

        // 创建数据集
        GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        GDALDataset* dataset = driver->Create(FLAGS_mid_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        if (!dataset) {
            std::cerr << "stop lines 创建数据集失败，路径:" << FLAGS_mid_shp_file_dir << std::endl;
            return fsdmap::FAIL;;
        }

      // 创建图层
        OGRSpatialReference poSpatialRef;
        poSpatialRef.importFromEPSG(4326);
        OGRLayer *layer = dataset->CreateLayer("stop_line", &poSpatialRef, wkbLineString, nullptr);
        if (!layer)
        {
          std::cerr << "stop lines 创建图层失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加ID字段
        OGRFieldDefn fieldDefn("ID", OFTInteger);
        if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
        {
          std::cerr << "stop lines 创建字段失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 停止线TYPE 字段
        OGRFieldDefn lineTypeFieldDefn("TYPE", OFTInteger);
        if (layer->CreateField(&lineTypeFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "stop lines 创建字段 TYPE 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 color 字段
        OGRFieldDefn colorFieldDefn("COLOR", OFTInteger);
        if (layer->CreateField(&colorFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "stop lines 创建字段 COLOR 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }
        // 添加 SHAPE_TYPE 字段
        OGRFieldDefn shapeTypeFieldDefn("SHAPE_TYPE", OFTInteger);
        if (layer->CreateField(&shapeTypeFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "stop lines 创建字段 SHAPE 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 LANE_ID 字段
        OGRFieldDefn laneIdFieldDefn("LANE_ID", OFTString);
        if (layer->CreateField(&laneIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "stop lines 创建字段 LANE_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 FRAME_ID 字段
        OGRFieldDefn frameIdFieldDefn("FRAME_ID", OFTString);
        if (layer->CreateField(&frameIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "stop lines 创建字段 FRAME_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 VERSION 字段
        OGRFieldDefn versionFieldDefn("VERSION", OFTString);
        if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "stop lines 创建字段 VERSION 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        for (const auto& object : session->raw_object_ret_list) {
          if (object->ele_type == 6) {
            OGRLineString lineString;
            for (const auto& point : object->list) {
                Eigen::Vector3d wgs;
                session->data_processer->local2wgs(point->pos, wgs);
                // double lng_gcj2;
                // double lat_gcj2;
                // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

                double lng_gcj2 = wgs.x();
                double lat_gcj2 = wgs.y();
                // TODO:qzc gcj01
                // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
                // TODO:qzc gcj01

                // Eigen::Vector3d lng_lat_02;
                // lng_lat_02.x() = lng_gcj2;
                // lng_lat_02.y() = lat_gcj2;
                // lng_lat_02.z() = 0;
                // Eigen::Vector3d utm_02;
                // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
                // lineString.addPoint(utm_02.x(), utm_02.y());

                lineString.addPoint(lng_gcj2, lat_gcj2);

                // lineString.addPoint(point->pos.x(), point->pos.y());
            }

            OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
            feature->SetField("ID", object->obj_id);
            feature->SetField("TYPE", std::stoi(object->type));
            // feature->SetField("LANE_ID", object->lane_id);  
            // feature->SetField("COLOR", object->color);  
            // feature->SetField("SHAPE", object->shape);  
            feature->SetGeometry(&lineString);

            if (layer->CreateFeature(feature) != OGRERR_NONE) {
                std::cerr << "stop lines 创建要素失败" << std::endl;
            }

            OGRFeature::DestroyFeature(feature);
          }
        }

        // 关闭数据集
        GDALClose(dataset);

        return fsdmap::SUCC;
    }

    int RoadModelExportSHP::export_cross_walk_to_shp_vectorize1(RoadModelSessionData* session) {
        // 注册所有的驱动
        GDALAllRegister();

        // 创建数据集
        GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        GDALDataset* dataset = driver->Create(FLAGS_mid_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        if (!dataset) {
            std::cerr << "cross walks 创建数据集失败，路径:" << FLAGS_mid_shp_file_dir << std::endl;
            return fsdmap::FAIL;;
        }

        // 创建图层
        OGRSpatialReference poSpatialRef;
        poSpatialRef.importFromEPSG(4326);
        OGRLayer *layer = dataset->CreateLayer("cross_walk", &poSpatialRef, wkbPolygon, nullptr);
        if (!layer)
        {
          std::cerr << "cross walks 创建图层失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加ID字段
        OGRFieldDefn fieldDefn("ID", OFTInteger);
        if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
        {
          std::cerr << "cross walks 创建ID字段失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 LANE_ID 字段
        OGRFieldDefn laneIdFieldDefn("LANE_ID", OFTString);
        if (layer->CreateField(&laneIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "cross walk创建字段 LANE_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }
        // 添加 JUNC_ID 字段
        OGRFieldDefn juncIdFieldDefn("JUNC_ID", OFTString);
        if (layer->CreateField(&juncIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "cross walk创建字段 JUNC_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 FRAME_ID 字段
        OGRFieldDefn frameIdFieldDefn("FRAME_ID", OFTString);
        if (layer->CreateField(&frameIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "cross walk创建字段 FRAME_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 VERSION 字段
        OGRFieldDefn versionFieldDefn("VERSION", OFTString);
        if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "cross walk创建字段 VERSION 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }


        // 写入数据
        int featureId = 0;
        for (const auto& object : session->raw_object_ret_list) {
            if (object->ele_type == 5) {
                // 创建一个多边形对象来表示块状区域
                OGRPolygon polygon;
                // 创建一个线性环，用于定义多边形的边界
                OGRLinearRing* ring = new OGRLinearRing();
                for (const auto& point : object->list) {
                    Eigen::Vector3d wgs;
                    session->data_processer->local2wgs(point->pos, wgs);

                    double lng_gcj2 = wgs.x();
                    double lat_gcj2 = wgs.y();
                    // TODO:qzc gcj01
                    // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
                    // TODO:qzc gcj01
                    ring->addPoint(lng_gcj2, lat_gcj2);
                }

                // 确保线性环闭合
                ring->closeRings();
                // 将线性环添加到多边形中
                polygon.addRing(ring);

                // 创建要素并设置字段值
                OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
                feature->SetField("ID", object->obj_id);
                // feature->SetField("LANE_ID", object->lane_id);  // 

                // 设置要素的几何对象为多边形
                feature->SetGeometry(&polygon);

                // 将要素添加到图层中
                if (layer->CreateFeature(feature) != OGRERR_NONE) {
                    std::cerr << "cross walks 创建要素失败" << std::endl;
                }

                OGRFeature::DestroyFeature(feature);
                delete ring;  
            }
        }

        // 关闭数据集
        GDALClose(dataset);

        return fsdmap::SUCC;
    }


    int RoadModelExportSHP::export_road_mark_to_shp_vectorize1(RoadModelSessionData* session) {
        // 注册所有的驱动
        GDALAllRegister();

        // 创建数据集
        GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        GDALDataset* dataset = driver->Create(FLAGS_mid_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        if (!dataset) {
            std::cerr << "arrow创建数据集失败，路径:" << FLAGS_mid_shp_file_dir << std::endl;
            return fsdmap::FAIL;;
        }

        // 设置字符编码为 UTF-8，生成 .cpg 文件
        dataset->SetMetadataItem("CHARSET", "UTF-8");
        
        // 创建arrow图层
        OGRSpatialReference poSpatialRef;
        poSpatialRef.importFromEPSG(4326);
        OGRLayer *layer = dataset->CreateLayer("arrow", &poSpatialRef, wkbPolygon, nullptr);
        if (!layer)
        {
          std::cerr << "arrow创建图层失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加字段
        OGRFieldDefn fieldDefn("ID", OFTInteger);
        if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
        {
          std::cerr << "arrow创建字段失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 箭头TYPE 字段
        OGRFieldDefn typeFieldDefn("TYPE", OFTString);
        if (layer->CreateField(&typeFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "arrow 创建字段 TYPE 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 LANE_ID 字段
        OGRFieldDefn laneIdFieldDefn("LANE_ID", OFTString);
        if (layer->CreateField(&laneIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "arrow 创建字段 LANE_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 FRAME_ID 字段
        OGRFieldDefn frameIdFieldDefn("FRAME_ID", OFTString);
        if (layer->CreateField(&frameIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "arrow 创建字段 FRAME_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 VERSION 字段
        OGRFieldDefn versionFieldDefn("VERSION", OFTString);
        if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "arrow 创建字段 VERSION 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }


        // 写入数据
        for (const auto& object : session->raw_object_ret_list) {
          if (object->ele_type == 3) {
            // 创建一个多边形对象来表示块状区域
            OGRPolygon polygon;
            // 创建一个线性环，用于定义多边形的边界
            OGRLinearRing* ring = new OGRLinearRing();
            for (const auto& point : object->list) {
                Eigen::Vector3d wgs;
                session->data_processer->local2wgs(point->pos, wgs);
                // double lng_gcj2;
                // double lat_gcj2;
                // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

                double lng_gcj2 = wgs.x();
                double lat_gcj2 = wgs.y();
                // TODO:qzc gcj01
                // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
                // TODO:qzc gcj01

                // Eigen::Vector3d lng_lat_02;
                // lng_lat_02.x() = lng_gcj2;
                // lng_lat_02.y() = lat_gcj2;
                // lng_lat_02.z() = 0;
                // Eigen::Vector3d utm_02;
                // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
                // lineString.addPoint(utm_02.x(), utm_02.y());

                ring->addPoint(lng_gcj2, lat_gcj2);

                // lineString.addPoint(point->pos.x(), point->pos.y());
            }
            // 确保线性环闭合
            ring->closeRings();
            // 将线性环添加到多边形中
            polygon.addRing(ring);

            OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
            feature->SetField("ID", object->obj_id);
            feature->SetField("TYPE", object->type.c_str());
            // feature->SetField("LANE_ID", object->lane_id);

            // 设置要素的几何对象为多边形
            feature->SetGeometry(&polygon);

            if (layer->CreateFeature(feature) != OGRERR_NONE) {
                std::cerr << "arrow创建要素失败" << std::endl;
            }

            OGRFeature::DestroyFeature(feature);
            delete ring; 
          }
        }

        // 关闭数据集
        GDALClose(dataset);

        return fsdmap::SUCC;
    }

    int RoadModelExportSHP::export_intersection_to_shp_vectorize1(RoadModelSessionData *session){
        // 注册所有的驱动
        GDALAllRegister();

        // 创建数据集
        GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        GDALDataset *dataset = driver->Create(FLAGS_mid_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        if (!dataset)
        {
          std::cerr << "junction 创建数据集失败，路径:" << FLAGS_mid_shp_file_dir << std::endl;
          return fsdmap::FAIL;
          ;
        }

      // 创建图层
        OGRSpatialReference poSpatialRef;
        poSpatialRef.importFromEPSG(4326);
        OGRLayer *layer = dataset->CreateLayer("junction", &poSpatialRef, wkbPolygon, nullptr);
        if (!layer)
        {
          std::cerr << "junction 创建图层失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }
        // 添加ID字段
        OGRFieldDefn fieldDefn("ID", OFTInteger);
        if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加TYPE 字段
        OGRFieldDefn typeFieldDefn("TYPE", OFTInteger);
        if (layer->CreateField(&typeFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段 TYPE 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 IS_EFFECT  字段
        OGRFieldDefn iseffectFieldDefn("IS_EFFECT", OFTInteger);
        if (layer->CreateField(&iseffectFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段 IS_EFFECT 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 lane road  字段
        OGRFieldDefn laneIdFieldDefn("LANE_ID", OFTString);
        if (layer->CreateField(&laneIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段 LANE_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        OGRFieldDefn inRoadFieldDefn("IN_ROAD", OFTString);
        if (layer->CreateField(&inRoadFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段 IN_ROAD 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }
        OGRFieldDefn outRoadFieldDefn("OUT_ROAD", OFTString);
        if (layer->CreateField(&outRoadFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段 IN_ROAD 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        OGRFieldDefn inLaneFieldDefn("IN_LANE", OFTString);
        if (layer->CreateField(&inLaneFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段 IN_LANE 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        OGRFieldDefn outLaneFieldDefn("OUT_LANE", OFTString);
        if (layer->CreateField(&outLaneFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段 OUT_LANE 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 FRAME_ID 字段
        OGRFieldDefn frameIdFieldDefn("FRAME_ID", OFTInteger);
        if (layer->CreateField(&frameIdFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction创建字段 FRAME_ID 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }

        // 添加 VERSION 字段
        OGRFieldDefn versionFieldDefn("VERSION", OFTString);
        if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
        {
          std::cerr << "junction 创建字段 VERSION 失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
        }



        // 写入数据
        for (const auto &inter : session->raw_intersections)
        {
          // 创建一个多边形对象来表示块状区域
          OGRPolygon polygon;
          // 创建一个线性环，用于定义多边形的边界
          OGRLinearRing *ring = new OGRLinearRing();
          for (auto &pt : inter->point_info)
          {
            Eigen::Vector3d wgs;
            session->data_processer->local2wgs(pt->pos, wgs);
            // double lng_gcj2;
            // double lat_gcj2;
            // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

            double lng_gcj2 = wgs.x();
            double lat_gcj2 = wgs.y();
            // TODO:qzc gcj01
            // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
            // TODO:qzc gcj01

            // Eigen::Vector3d lng_lat_02;
            // lng_lat_02.x() = lng_gcj2;
            // lng_lat_02.y() = lat_gcj2;
            // lng_lat_02.z() = 0;
            // Eigen::Vector3d utm_02;
            // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
            // lineString.addPoint(utm_02.x(), utm_02.y());

            ring->addPoint(lng_gcj2, lat_gcj2);

            // lineString.addPoint(pt->pos.x(), pt->pos.y());
          }
          // 确保线性环闭合
          ring->closeRings();
          polygon.addRing(ring);

          OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
          feature->SetField("ID", inter->id);
          // feature->SetField("LANE_ID", object->lane_id);  //
          // feature->SetField("IN_LANE", object->lane_id);  //
          // feature->SetField("OUT_LANE", object->lane_id);  //
          // feature->SetField("IN_ROAD", object->lane_id);  //
          // feature->SetField("OUT_ROAD", object->lane_id);  //
          feature->SetGeometry(&polygon);

          if (layer->CreateFeature(feature) != OGRERR_NONE)
          {
            std::cerr << "junction 创建要素失败" << std::endl;
          }

          OGRFeature::DestroyFeature(feature);
        }
        // 关闭数据集
        GDALClose(dataset);
        return fsdmap::SUCC;
    }


    /******************************************************************************************/
    /********************************* 2 矢量化+拓扑 *******************************************/
    /******************************************************************************************/

    int RoadModelExportSHP::export_to_shape_file_with_topo(RoadModelSessionData *session)
    {
      //
      std::string cmd = "rm -r " + FLAGS_shp_file_dir + "/*.dbf";
      LOG_WARN("will run:{}", cmd);
      system(cmd.c_str());
      cmd = "rm -r " + FLAGS_shp_file_dir + "/*.prj";
      LOG_WARN("will run:{}", cmd)
      system(cmd.c_str());
      cmd = "rm -r " + FLAGS_shp_file_dir + "/*.shp";
      LOG_WARN("will run:{}", cmd)
      system(cmd.c_str());
      cmd = "rm -r " + FLAGS_shp_file_dir + "/*.shx";
      LOG_WARN("will run:{}", cmd)
      system(cmd.c_str());
      //
      export_lane_boundary_to_shp(session);

      export_lane_center_to_shp(session);

      export_road_boundary_to_shp(session);

      export_road_to_shp(session);

      export_lane_to_shp(session);

      // export_lane_adas_to_shp(session);

      // export_road_centers_to_shp(session);

      export_stop_line_to_shp(session);

      export_cross_walk_to_shp(session);

      export_road_mark_to_shp(session);

      export_intersection_to_shp(session);

      export_lane_boundary_to_shp_vectorize1(session, 1);

      export_road_boundary_to_shp_vectorize1(session, 1);

      return fsdmap::SUCC;
    }

  int RoadModelExportSHP::export_lane_boundary_to_shp(RoadModelSessionData *session)
  {
    // 注册所有的驱动
    GDALAllRegister();

    // 创建数据集
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!dataset)
    {
      std::cerr << "lane boundary 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
      return fsdmap::FAIL;
      ;
    }

    // 创建图层
    OGRSpatialReference poSpatialRef;
    poSpatialRef.importFromEPSG(4326);
    OGRLayer *layer = dataset->CreateLayer("lane_boundary", &poSpatialRef, wkbLineString, nullptr);
    if (!layer)
    {
      std::cerr << "lane boundary 创建图层失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 ID 字段
    OGRFieldDefn fieldDefn("ID", OFTInteger);
    if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane boundary 创建字段 ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 color 字段
    OGRFieldDefn colorFieldDefn("COLOR", OFTInteger);
    if (layer->CreateField(&colorFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane boundary  创建字段 COLOR 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }
    // 添加 shape 字段
    OGRFieldDefn shapetypeFieldDefn("SHAPE_TYPE", OFTInteger);
    if (layer->CreateField(&shapetypeFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane boundary  创建字段 SHAPE_TYPE 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // // 添加 is_safety 字段
    // OGRFieldDefn safetyFieldDefn("IS_SAFETY", OFTInteger);
    // // safetyFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    // if (layer->CreateField(&safetyFieldDefn) != OGRERR_NONE)
    // {
    //   std::cerr << "lane boundary  创建字段 IS_SAFETY 失败" << std::endl;
    //   GDALClose(dataset);
    //   return fsdmap::FAIL;
    // }

    // 添加 lane_id 字段
    OGRFieldDefn laneidFieldDefn("LANE_ID", OFTString);
    // safetyFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&laneidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane boundary  创建字段 LANE_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 pre_id 字段
    OGRFieldDefn preidFieldDefn("PRE_ID", OFTString);
    // preidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&preidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane boundary  创建字段 PRE_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 next_id 字段
    OGRFieldDefn nextidFieldDefn("NEXT_ID", OFTString);
    // nextidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&nextidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane boundary  创建字段 NEXT_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 frame_id 字段
    OGRFieldDefn frameidFieldDefn("FRAME_ID", OFTInteger);
    // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&frameidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane boundary  创建字段 FRAME_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 version 字段
    OGRFieldDefn versionFieldDefn("VERSION", OFTString);
    // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane boundary  创建字段 VERSION 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 写入数据
    int featureId = 0;
    bool one_line = false;
    std::ostringstream prev_oss, next_oss, prev_right_oss, next_right_oss;
    for (const auto &lane_group : session->new_lane_groups)
    {
      for(int i = 0; i < lane_group->lane_line_info.size(); i ++)
      {
        for (int lane_side = 0; lane_side < 2; ++lane_side)
        {

          const auto &lane_info = lane_group->lane_line_info[i];
          const auto &lane_data =
              lane_side == 0 ? lane_info->left_lane_boundary_info : lane_info->right_lane_boundary_info;

          OGRLineString lineString;

          //添加边界点
          if(lane_data->line_point_info.size() > 1){
            for (int i = 1; i < lane_data->line_point_info.size(); ++i) {
                auto &current_point = lane_data->line_point_info[i]; // 当前点A
                auto &previous_point = lane_data->line_point_info[i - 1]; // 上一个点B

                bool current_flag = alg::point_in_polygon(current_point->pos, session->task_polygon_vec3d);
                bool previous_flag = alg::point_in_polygon(previous_point->pos, session->task_polygon_vec3d);

                // 如果当前点和前一个点的 flag 标志不同，则需要计算交点
                if (current_flag != previous_flag) {
                    std::vector<Eigen::Vector3d> cross_points;
                    bool is_intersect = alg::get_cross_point_with_polygon(previous_point->pos, current_point->pos, session->task_polygon_vec3d, cross_points);
                  
                    if (is_intersect) {
                        // 将交点存入两个点之间
                        RoadLinePointInfo* new_point = new RoadLinePointInfo();
                        
                        new_point->pos = cross_points[0]; 
                        new_point->is_intersection_point = true;

                        lane_data->line_point_info.insert(lane_data->line_point_info.begin() + i, {new_point});
                        ++i; // 跳过插入的交点，避免重复处理
                    }
                }
            }
          }
          
          //剔除
          for (const auto &lane_point : lane_data->line_point_info)
          {
            if (lane_point->is_intersection_point == false) {
              if(alg::point_in_polygon(lane_point->pos, session->task_polygon_vec3d) == false){
                continue;
              }
            }
            
            Eigen::Vector3d wgs;
            session->data_processer->local2wgs(lane_point->pos, wgs);
            // double lng_gcj2;
            // double lat_gcj2;
            // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

            double lng_gcj2 = wgs.x();
            double lat_gcj2 = wgs.y();
            // TODO:qzc gcj01
            // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
            // TODO:qzc gcj01

            // Eigen::Vector3d lng_lat_02;
            // lng_lat_02.x() = lng_gcj2;
            // lng_lat_02.y() = lat_gcj2;
            // lng_lat_02.z() = 0;
            // Eigen::Vector3d utm_02;
            // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
            // lineString.addPoint(utm_02.x(), utm_02.y());

            lineString.addPoint(lng_gcj2, lat_gcj2);
          }

          prev_oss.str(""), next_oss.str("");
          prev_oss.clear(), next_oss.clear();
          for (size_t i = 0; i < lane_info->context.all_prev.size(); i++)
          {
            if (i > 0)
              prev_oss << ",";
            const auto &lane_prev_data =
                lane_side == 0 ? lane_info->context.all_prev[i].src->left_lane_boundary_info : lane_info->context.all_prev[i].src->right_lane_boundary_info;
            prev_oss << lane_prev_data->lane_id;
          }

          for (size_t i = 0; i < lane_info->context.all_next.size(); i++)
          {
            if (i > 0)
              next_oss << ",";
            const auto &lane_next_data =
                lane_side == 0 ? lane_info->context.all_next[i].src->left_lane_boundary_info : lane_info->context.all_next[i].src->right_lane_boundary_info;
            next_oss << lane_next_data->lane_id;
          }

          OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
          if(one_line){

            if(prev_oss.str().empty())
            {
              prev_oss << prev_right_oss.str();
            }
            if(next_oss.str().empty())
            {
              next_oss << next_right_oss.str();
            }

            std::ostringstream lane_id_oss;
            lane_id_oss << lane_group->lane_line_info[i - 1]->road_lane_id;
            lane_id_oss << ",";
            lane_id_oss << lane_info->road_lane_id;
            feature->SetField("LANE_ID", lane_id_oss.str().c_str());
            one_line = false;
          }else{
            std::string laneIdStr = std::to_string(lane_info->road_lane_id);
            feature->SetField("LANE_ID", laneIdStr.c_str()); // 与车道线同ID，后续排查
          }

          feature->SetField("ID", lane_data->lane_id);
          int color = lane_data->color;
          int type = lane_data->type;
          color = color <= 1 ? 1 : color;
          type = type <= 1 ? 1 : type;
          feature->SetField("COLOR", color );
          feature->SetField("SHAPE_TYPE", type ); // 暂时没有
          // feature->SetField("IS_SAFETY", lane_data->geo);  // 暂时没有
          feature->SetField("PRE_ID", prev_oss.str().c_str());
          feature->SetField("NEXT_ID", next_oss.str().c_str());
          // feature->SetField("FRAME_ID", );  // 暂时没有
          feature->SetField("VERSION", session->version.c_str());

          feature->SetGeometry(&lineString);

          if (layer->CreateFeature(feature) != OGRERR_NONE)
          {
            std::cerr << "lane boundary 创建要素失败" << std::endl;
          }

          OGRFeature::DestroyFeature(feature);

          if(i < (lane_group->lane_line_info.size() - 1) &&(lane_info->right_lane_boundary_info->lane_id == lane_group->lane_line_info[i + 1]->left_lane_boundary_info->lane_id)){
            prev_right_oss.str(""), next_right_oss.str("");
            prev_right_oss.clear(), next_right_oss.clear();
            for (size_t i = 0; i < lane_info->context.all_prev.size(); i++)
            {
              if (i > 0)
                prev_right_oss << ",";
              prev_right_oss << lane_info->context.all_prev[i].src->right_lane_boundary_info->lane_id;
            }

            for (size_t i = 0; i < lane_info->context.all_next.size(); i++)
            {
              if (i > 0)
                next_right_oss << ",";
              next_right_oss << lane_info->context.all_next[i].src->right_lane_boundary_info->lane_id;
            }
              
            one_line = true;
            break;
          }

        }
      }
    }

    // 关闭数据集
    GDALClose(dataset);

    return fsdmap::SUCC;
  }

  int RoadModelExportSHP::export_lane_center_to_shp(RoadModelSessionData *session)
  {
    // 注册所有的驱动
    GDALAllRegister();

    // 创建数据集
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!dataset)
    {
      std::cerr << "lane centers 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
      return fsdmap::FAIL;
      ;
    }

    // 创建图层
    OGRSpatialReference poSpatialRef;
    poSpatialRef.importFromEPSG(4326);
    OGRLayer *layer = dataset->CreateLayer("lane_center", &poSpatialRef, wkbLineString, nullptr);
    if (!layer)
    {
      std::cerr << "lane centers 创建图层失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加字段
    OGRFieldDefn fieldDefn("ID", OFTInteger);
    if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane centers 创建字段 ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 lane_id 字段
    OGRFieldDefn laneidfieldDefn("LANE_ID", OFTInteger);
    if (layer->CreateField(&laneidfieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane centers 创建字段 LANE_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 type 字段
    OGRFieldDefn typefieldDefn("TYPE", OFTInteger);
    if (layer->CreateField(&typefieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane centers 创建字段 TYPE 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 frame_id 字段
    OGRFieldDefn frameidFieldDefn("FRAME_ID", OFTInteger);
    // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&frameidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane centers  创建字段 FRAME_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 version 字段
    OGRFieldDefn versionFieldDefn("VERSION", OFTString);
    // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane centers  创建字段 VERSION 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 写入数据
    int featureId = 0;
    for (const auto &lane_group : session->new_lane_groups)
    {
      for (const auto &lane_info : lane_group->lane_line_info)
      {
        const auto &lane_central_data = lane_info->center_lane_boundary_info;
        OGRLineString lineString;
        //插值边界点
        if(lane_central_data->line_point_info.size() > 1){
          for (int i = 1; i < lane_central_data->line_point_info.size(); ++i) {
              auto &current_point = lane_central_data->line_point_info[i]; // 当前点A
              auto &previous_point = lane_central_data->line_point_info[i - 1]; // 上一个点B

              bool current_flag = alg::point_in_polygon(current_point->pos, session->task_polygon_vec3d);
              bool previous_flag = alg::point_in_polygon(previous_point->pos, session->task_polygon_vec3d);

              // 如果当前点和前一个点的 flag 标志不同，则需要计算交点
              if (current_flag != previous_flag) {
                  std::vector<Eigen::Vector3d> cross_points;
                  bool is_intersect = alg::get_cross_point_with_polygon(previous_point->pos, current_point->pos, session->task_polygon_vec3d, cross_points);
                
                  if (is_intersect) {
                      // 将交点存入两个点之间
                      RoadLinePointInfo* new_point = new RoadLinePointInfo();
                      new_point->pos = cross_points[0]; 
                      new_point->is_intersection_point = true;

                      lane_central_data->line_point_info.insert(lane_central_data->line_point_info.begin() + i, {new_point});
                      ++i; // 跳过插入的交点，避免重复处理
                  }
              }
          }
        }


        for (const auto &lane_point : lane_central_data->line_point_info)
        {
          if (lane_point->is_intersection_point == false) {
            if(alg::point_in_polygon(lane_point->pos, session->task_polygon_vec3d) == false){
              continue;
            }
          }

          Eigen::Vector3d wgs;
          session->data_processer->local2wgs(lane_point->pos, wgs);
          // double lng_gcj2;
          // double lat_gcj2;
          // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

          double lng_gcj2 = wgs.x();
          double lat_gcj2 = wgs.y();
          // TODO:qzc gcj01
          // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
          // TODO:qzc gcj01

          // Eigen::Vector3d lng_lat_02;
          // lng_lat_02.x() = lng_gcj2;
          // lng_lat_02.y() = lat_gcj2;
          // lng_lat_02.z() = 0;
          // Eigen::Vector3d utm_02;
          // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
          // lineString.addPoint(utm_02.x(), utm_02.y());

          lineString.addPoint(lng_gcj2, lat_gcj2);

          // lineString.addPoint(lane_point->pos.x(), lane_point->pos.y());
        }

        OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
        feature->SetField("ID", lane_central_data->lane_id);
        feature->SetField("LANE_ID", lane_central_data->lane_id);
        // feature->SetField("TYPE", lane_central_data->color); // 暂时没有
        // feature->SetField("SHAPE", lane_central_data->); // 暂时没有
        // feature->SetField("FRAME_ID", );  // 暂时没有
        feature->SetField("VERSION", session->version.c_str());
        feature->SetGeometry(&lineString);

        if (layer->CreateFeature(feature) != OGRERR_NONE)
        {
          std::cerr << "lane centers 创建要素失败" << std::endl;
        }

        OGRFeature::DestroyFeature(feature);
      }
    }

    // 关闭数据集
    GDALClose(dataset);

    return fsdmap::SUCC;
  }

  int RoadModelExportSHP::export_road_boundary_to_shp(RoadModelSessionData *session)
  {
    // 注册所有的驱动
    GDALAllRegister();

    // 创建数据集
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!dataset)
    {
      std::cerr << "road boundary 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
      return fsdmap::FAIL;
      ;
    }

    // 创建图层
    OGRSpatialReference poSpatialRef;
    poSpatialRef.importFromEPSG(4326);
    OGRLayer *layer = dataset->CreateLayer("road_boundary", &poSpatialRef, wkbLineString, nullptr);
    if (!layer)
    {
      std::cerr << "road boundary 创建图层失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加ID字段
    OGRFieldDefn fieldDefn("ID", OFTInteger);
    if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road boundary 创建字段失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 color 字段
    OGRFieldDefn colorFieldDefn("COLOR", OFTInteger);
    if (layer->CreateField(&colorFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road boundary  创建字段 COLOR 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加type字段
    OGRFieldDefn typefieldDefn("TYPE", OFTInteger);
    if (layer->CreateField(&typefieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road boundary 创建字段 TYPE 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 is_safety 字段
    OGRFieldDefn safetyFieldDefn("IS_SAFETY", OFTInteger);
    // safetyFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&safetyFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road boundary  创建字段 IS_SAFETY 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 road_id 字段
    OGRFieldDefn roadidFieldDefn("ROAD_ID", OFTInteger);
    // safetyFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&roadidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road boundary  创建字段 ROAD_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 pre_id 字段
    OGRFieldDefn preidFieldDefn("PRE_ID", OFTString);
    // preidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&preidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road boundary  创建字段 PRE_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 next_id 字段
    OGRFieldDefn nextidFieldDefn("NEXT_ID", OFTString);
    // nextidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&nextidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road boundary  创建字段 NEXT_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 frame_id 字段
    OGRFieldDefn frameidFieldDefn("FRAME_ID", OFTInteger);
    // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&frameidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road boundary  创建字段 FRAME_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 version 字段
    OGRFieldDefn versionFieldDefn("VERSION", OFTString);
    // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road boundary  创建字段 VERSION 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 写入数据
    int featureId = 0;
    for (const auto &lane_group : session->new_lane_groups)
    {
      for (int lane_side = 0; lane_side < 2; ++lane_side)
      {
        const auto &barrier_info =
            lane_side == 0 ? lane_group->left_barrier_segment_info : lane_group->right_barrier_segment_info;
        for (auto &barrier_data : barrier_info)
        {

          //添加边界点
          if(barrier_data->point_info.size() > 1){

            for (int i = 1; i < barrier_data->point_info.size(); ++i) {
                auto &current_point = barrier_data->point_info[i]; // 当前点A
                auto &previous_point = barrier_data->point_info[i - 1]; // 上一个点B

                bool current_flag = alg::point_in_polygon(current_point->pos, session->task_polygon_vec3d);
                bool previous_flag = alg::point_in_polygon(previous_point->pos, session->task_polygon_vec3d);

                // 如果当前点和前一个点的 flag 标志不同，则需要计算交点
                if (current_flag != previous_flag) {
                    std::vector<Eigen::Vector3d> cross_points;
                    bool is_intersect = alg::get_cross_point_with_polygon(previous_point->pos, current_point->pos, session->task_polygon_vec3d, cross_points);
                  
                    if (is_intersect) {
                        // 将交点存入两个点之间
                        RoadBoundaryPointInfo* new_point = new RoadBoundaryPointInfo();
                        new_point->pos = cross_points[0]; 
                        new_point->is_intersection_point = true;
                        barrier_data->point_info.insert(barrier_data->point_info.begin() + i, {new_point});
                        ++i; // 跳过插入的交点，避免重复处理
                    }
                }
            }
          }

          OGRLineString lineString;

          for (const auto &lane_point : barrier_data->point_info)
          {
            if (lane_point->is_intersection_point == false) {
              if(alg::point_in_polygon(lane_point->pos, session->task_polygon_vec3d) == false){
                continue;
              }
            }
              
            Eigen::Vector3d wgs;
            session->data_processer->local2wgs(lane_point->pos, wgs);
            // double lng_gcj2;
            // double lat_gcj2;
            // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

            double lng_gcj2 = wgs.x();
            double lat_gcj2 = wgs.y();
            // TODO:qzc gcj01
            // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
            // TODO:qzc gcj01

            // Eigen::Vector3d lng_lat_02;
            // lng_lat_02.x() = lng_gcj2;
            // lng_lat_02.y() = lat_gcj2;
            // lng_lat_02.z() = 0;
            // Eigen::Vector3d utm_02;
            // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
            // lineString.addPoint(utm_02.x(), utm_02.y());

            lineString.addPoint(lng_gcj2, lat_gcj2);

            // lineString.addPoint(lane_point->pos.x(), lane_point->pos.y());
          }

          std::ostringstream prev_oss, next_oss;
          for (size_t i = 0; i < barrier_data->context.all_prev.size(); i++)
          {
            if (i > 0)
              prev_oss << ",";
            prev_oss << barrier_data->context.all_prev[i].src->boundary_id;
          }

          for (size_t i = 0; i < barrier_data->context.all_next.size(); i++)
          {
            if (i > 0)
              next_oss << ",";
            next_oss << barrier_data->context.all_next[i].src->boundary_id;
          }

          OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
          feature->SetField("ID", barrier_data->boundary_id);
          // feature->SetField("COLOR", barrier_data->color); // 暂时没有
          // feature->SetField("TYPE", lane_data->); // 暂时没有
          // feature->SetField("IS_SAFETY", lane_data->geo);  // 暂时没有
          feature->SetField("ROAD_ID", lane_group->group_index);
          feature->SetField("PRE_ID", prev_oss.str().c_str());
          feature->SetField("NEXT_ID", next_oss.str().c_str());

          // feature->SetField("FRAME_ID", );  // 暂时没有
          feature->SetField("VERSION", session->version.c_str());

          feature->SetGeometry(&lineString);

          if (layer->CreateFeature(feature) != OGRERR_NONE)
          {
            std::cerr << "road boundary 创建要素失败" << std::endl;
          }

          OGRFeature::DestroyFeature(feature);
        }
      }
    }

    // 关闭数据集
    GDALClose(dataset);

    return fsdmap::SUCC;
  }

 int RoadModelExportSHP::export_road_to_shp(RoadModelSessionData *session)
  {
    // 注册所有的驱动
    GDALAllRegister();

    // 创建数据集
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!dataset)
    {
      std::cerr << "road 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
      return fsdmap::FAIL;
      ;
    }

    // 创建图层
    OGRSpatialReference poSpatialRef;
    poSpatialRef.importFromEPSG(4326);
    OGRLayer *layer = dataset->CreateLayer("road", &poSpatialRef, wkbPolygon, nullptr);
    if (!layer)
    {
      std::cerr << "road 创建图层失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 ID 字段
    OGRFieldDefn fieldDefn("ID", OFTInteger);
    if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road 创建字段 ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 KIND 字段
    OGRFieldDefn roadtypefieldDefn("KIND", OFTInteger);
    if (layer->CreateField(&roadtypefieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road 创建字段 KIND 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 IS_BIDIR 字段
    OGRFieldDefn isbidirfieldDefn("IS_BIDIR", OFTInteger);
    if (layer->CreateField(&isbidirfieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road 创建字段 IS_BIDIR 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 SCEN_TYPE 字段
    OGRFieldDefn scentypefieldDefn("SCEN_TYPE", OFTInteger);
    if (layer->CreateField(&scentypefieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road 创建字段 SCEN_TYPE 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 TYPE 字段
    OGRFieldDefn fowfieldDefn("TYPE", OFTInteger);
    if (layer->CreateField(&fowfieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road 创建字段 TYPE 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 SPEED 字段
    OGRFieldDefn speedfieldDefn("SPEED", OFTInteger);
    if (layer->CreateField(&speedfieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road 创建字段 SPEED 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // // 添加 road_line 字段
    // OGRFieldDefn roadlineFieldDefn("ROAD_LINE", OFTString);
    // if (layer->CreateField(&roadlineFieldDefn) != OGRERR_NONE)
    // {
    //   std::cerr << "road  创建字段 ROAD_LINE 失败" << std::endl;
    //   GDALClose(dataset);
    //   return fsdmap::FAIL;
    // }

    // 添加 lane_id 字段
    OGRFieldDefn laneidFieldDefn("LANE_ID", OFTString);
    if (layer->CreateField(&laneidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road  创建字段 LANE_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 left_bid 字段
    OGRFieldDefn leftbidFieldDefn("LEFT_BID", OFTString);
    if (layer->CreateField(&leftbidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road  创建字段 LEFT_BID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 right_bid 字段
    OGRFieldDefn rightbidFieldDefn("RIGHT_BID", OFTString);
    if (layer->CreateField(&rightbidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road  创建字段 RIGHT_BID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 in_id 字段
    OGRFieldDefn inidFieldDefn("IN_ID", OFTString);
    if (layer->CreateField(&inidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road  创建字段 IN_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 cross_id 字段
    OGRFieldDefn crossidFieldDefn("CROSS_ID", OFTString);
    if (layer->CreateField(&crossidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road  创建字段 CROSS_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 out_id 字段
    OGRFieldDefn outidFieldDefn("OUT_ID", OFTString);
    if (layer->CreateField(&outidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road  创建字段 OUT_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 pre_id 字段
    OGRFieldDefn preidFieldDefn("PRE_ID", OFTString); // 工艺暂时不需要
    // preidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&preidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road  创建字段 PRE_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 next_id 字段
    OGRFieldDefn nextidFieldDefn("NEXT_ID", OFTString); // 工艺暂时不需要
    // nextidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&nextidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road  创建字段 NEXT_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 frame_id 字段
    OGRFieldDefn frameidFieldDefn("FRAME_ID", OFTInteger);
    // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&frameidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road  创建字段 FRAME_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 version 字段
    OGRFieldDefn versionFieldDefn("VERSION", OFTString);
    // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road  创建字段 VERSION 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 写入数据
    int featureId = 0;

    for (const auto &lg : session->new_lane_groups)
    {
      // 创建一个新的多边形对象
      OGRPolygon *polygon = new OGRPolygon();

      // 创建一个线性环（外环）
      OGRLinearRing *outerRing = new OGRLinearRing();
      Eigen::Vector3d wgs;

      double first_point_x(0), first_point_y(0);

      auto line = lg->lane_line_info[0]->left_lane_boundary_info;
      for (size_t tt = 0; tt < line->line_point_info.size(); tt++)
      {
        session->data_processer->local2wgs(line->line_point_info[tt]->pos, wgs);

        double lng_gcj2 = wgs.x();
        double lat_gcj2 = wgs.y();
        // TODO:qzc gcj01
        // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
        // TODO:qzc gcj01

        outerRing->addPoint(lng_gcj2, lat_gcj2);

        if (tt == 0)
        {
          first_point_x = lng_gcj2;
          first_point_y = lat_gcj2;
        }
      }

      line = lg->lane_line_info[lg->lane_line_info.size() - 1]->right_lane_boundary_info;
      for (int t = line->line_point_info.size() - 1; t >= 0; t--)
      {

        session->data_processer->local2wgs(line->line_point_info[t]->pos, wgs);

        double lng_gcj2 = wgs.x();
        double lat_gcj2 = wgs.y();
        // TODO:qzc gcj01
        // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
        // TODO:qzc gcj01

        outerRing->addPoint(lng_gcj2, lat_gcj2);
      }
      outerRing->addPoint(first_point_x, first_point_y); // 闭合多边形
      
      //判断是否在任务框内
      std::vector<Eigen::Vector3d> obj_points;
      std::vector<int> indexs;
      for (int i = 0; i < outerRing->getNumPoints(); ++i) {
        double x = outerRing->getX(i);  // 获取第 i 个点的 x 坐标
        double y = outerRing->getY(i);  // 获取第 i 个点的 y 坐标
        Eigen::Vector3d wgs = {x, y, 0};
        Eigen::Vector3d tar = wgs;
        session->data_processer->wgs2local(wgs, tar); 
        obj_points.push_back(tar);
      }

      if(alg::points_in_polygon(session->task_polygon_vec3d, obj_points, indexs) == false){
        delete polygon;
        delete outerRing;
        continue;
      }
    
      // 确保线性环闭合
      outerRing->closeRings();

      // 将线性环添加到多边形中
      polygon->addRing(outerRing);

      std::ostringstream lane_id_oss, left_bid_oss, right_bid_oss, all_prev_oss, all_next_oss;
      for (size_t i = 0; i < lg->lane_line_info.size(); i++)
      {
        if (i > 0)
        {
          lane_id_oss << ",";
        }
        lane_id_oss << lg->lane_line_info[i]->road_lane_id;
      }
      for (size_t i = 0; i < lg->left_barrier_segment_info.size(); i++) // 可能有问题
      {
        if (i > 0)
        {
          left_bid_oss << ",";
        }
        left_bid_oss << lg->left_barrier_segment_info[i]->boundary_id;
      }
      for (size_t i = 0; i < lg->right_barrier_segment_info.size(); i++)
      {
        if (i > 0)
        {
          right_bid_oss << ",";
        }
        right_bid_oss << lg->right_barrier_segment_info[i]->boundary_id;
      }

      for (size_t i = 0; i < lg->context.all_prev.size(); i++) // 工艺暂时不需要
      {
        if (i > 0)
        {
          all_prev_oss << ",";
        }
        all_prev_oss << lg->context.all_prev[i].src->group_index;
      }

      for (size_t i = 0; i < lg->context.all_next.size(); i++)
      {
        if (i > 0)
        {
          all_next_oss << ",";
        }
        all_next_oss << lg->context.all_next[i].src->group_index;
      }

      OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
      feature->SetField("ID", lg->group_index);
      // feature->SetField("KIND", lg->group_index); // 道路类型，暂时没有
      // feature->SetField("IS_BIDIR", lg->group_index); // 是否双向道路，暂时没有
      // feature->SetField("SCEN_TYPE", lg->group_index); // 规划等级，暂时没有
      // feature->SetField("TYPE", lg->group_index); // 道路构成，暂时没有
      // feature->SetField("SPEED", lg->group_index); // 道路限速，暂时没有
      // // feature->SetField("ROAD_LINE", left_bid_oss.to_str()); //关联道路中心线
      feature->SetField("LANE_ID", lane_id_oss.str().c_str());     // 关联车道ID
      feature->SetField("LEFT_BID", left_bid_oss.str().c_str());   // 关联左侧道路边界
      feature->SetField("RIGHT_BID", right_bid_oss.str().c_str()); // 关联右侧道路边界
      // feature->SetField("IN_ID", lane_data->); // 进入路口编号，暂时没有
      // feature->SetField("CROSS_ID", lane_data->color); // 通过路口编号，暂时没有
      // feature->SetField("OUT_ID", lane_data->); // 退出路口编号，暂时没有
      feature->SetField("PRE_ID", all_prev_oss.str().c_str());  // 前序道路
      feature->SetField("NEXT_ID", all_next_oss.str().c_str()); // 后续道路

      // feature->SetField("FRAME_ID", );  // 暂时没有
      feature->SetField("VERSION", session->version.c_str());


      feature->SetGeometry(polygon);

      if (layer->CreateFeature(feature) != OGRERR_NONE)
      {
        std::cerr << "road 创建要素失败" << std::endl;
      }

      OGRFeature::DestroyFeature(feature);
    }

    // 关闭数据集
    GDALClose(dataset);

    return fsdmap::SUCC;
  }

  int RoadModelExportSHP::export_lane_to_shp(RoadModelSessionData *session)
  {
    // 注册所有的驱动
    GDALAllRegister();

    // 创建数据集
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!dataset)
    {
      std::cerr << "lane 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
      return fsdmap::FAIL;
      ;
    }

    // 创建图层
    OGRSpatialReference poSpatialRef;
    poSpatialRef.importFromEPSG(4326);
    OGRLayer *layer = dataset->CreateLayer("lane", &poSpatialRef, wkbPolygon, nullptr);
    if (!layer)
    {
      std::cerr << "lane 创建图层失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 id 字段
    OGRFieldDefn idFieldDefn("ID", OFTInteger);
    if (layer->CreateField(&idFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane  创建字段 ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 lane_type 字段
    OGRFieldDefn lanetypeFieldDefn("LANE_TYPE", OFTInteger);
    if (layer->CreateField(&lanetypeFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane  创建字段 LANE_TYPE 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 turn_type 字段
    OGRFieldDefn turntypeFieldDefn("TURN_TYPE", OFTString);
    if (layer->CreateField(&turntypeFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane  创建字段 TURN_TYPE 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 road_id 字段
    OGRFieldDefn roadidFieldDefn("ROAD_ID", OFTInteger);
    if (layer->CreateField(&roadidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane  创建字段 ROAD_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 left_ln 字段
    OGRFieldDefn leftlnFieldDefn("LEFT_LN", OFTString);
    if (layer->CreateField(&leftlnFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane  创建字段 LEFT_LN 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 right_ln 字段
    OGRFieldDefn rightlnFieldDefn("RIGHT_LN", OFTString);
    if (layer->CreateField(&rightlnFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane  创建字段 RIGHT_LN 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 left_r_ln 字段
    OGRFieldDefn leftrlnFieldDefn("LEFT_R_LN", OFTString);
    if (layer->CreateField(&leftrlnFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane  创建字段 LEFT_R_LN 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 right_r_ln 字段
    OGRFieldDefn rightrlnFieldDefn("RIGHT_R_LN", OFTString);
    if (layer->CreateField(&rightrlnFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane  创建字段 RIGHT_R_LN 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 left_bid 字段
    OGRFieldDefn leftbidFieldDefn("LEFT_BID", OFTInteger);
    if (layer->CreateField(&leftbidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane  创建字段 LEFT_BID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 right_bid 字段
    OGRFieldDefn rightbidFieldDefn("RIGHT_BID", OFTInteger);
    if (layer->CreateField(&rightbidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane  创建字段 RIGHT_BID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // // 添加 obj_id 字段
    // OGRFieldDefn objidFieldDefn("OBJ_ID", OFTString);
    // if (layer->CreateField(&objidFieldDefn) != OGRERR_NONE)
    // {
    //   std::cerr << "lane  创建字段 OBJ_ID 失败" << std::endl;
    //   GDALClose(dataset);
    //   return fsdmap::FAIL;
    // }

    // // 添加 obj_type 字段
    // OGRFieldDefn objtypeFieldDefn("OBJ_TYPE", OFTString);
    // if (layer->CreateField(&objtypeFieldDefn) != OGRERR_NONE)
    // {
    //   std::cerr << "lane  创建字段 OBJ_TYPE 失败" << std::endl;
    //   GDALClose(dataset);
    //   return fsdmap::FAIL;
    // }

    // 添加 inter_id 字段
    OGRFieldDefn interidFieldDefn("INTER_ID", OFTInteger);
    if (layer->CreateField(&interidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane  创建字段 INTER_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 pre_id 字段
    OGRFieldDefn preidFieldDefn("PRE_ID", OFTString);
    // preidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&preidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane  创建字段 PRE_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 next_id 字段
    OGRFieldDefn nextidFieldDefn("NEXT_ID", OFTString);
    // nextidFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&nextidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane  创建字段 NEXT_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 frame_id 字段
    OGRFieldDefn frameidFieldDefn("FRAME_ID", OFTInteger);
    // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&frameidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane  创建字段 FRAME_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 version 字段
    OGRFieldDefn versionFieldDefn("VERSION", OFTString);
    // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane  创建字段 VERSION 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 写入数据
    int featureId = 0;
    for (const auto &lane_group : session->new_lane_groups)
    {
      for (size_t i = 0; i < lane_group->lane_line_info.size(); i++)
      {
        const auto &lane_info = lane_group->lane_line_info[i];

        // 创建一个新的多边形对象
        OGRPolygon *polygon = new OGRPolygon();

        // 创建一个线性环（外环）
        OGRLinearRing *outerRing = new OGRLinearRing();
        Eigen::Vector3d wgs;

        double first_point_x(0), first_point_y(0);

        for (size_t tt = 0; tt < lane_info->left_lane_boundary_info->line_point_info.size(); tt++)
        {
          session->data_processer->local2wgs(lane_info->left_lane_boundary_info->line_point_info[tt]->pos, wgs);

          double lng_gcj2 = wgs.x();
          double lat_gcj2 = wgs.y();
          // TODO:qzc gcj01
          // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
          // TODO:qzc gcj01

          outerRing->addPoint(lng_gcj2, lat_gcj2);

          if (tt == 0)
          {
            first_point_x = lng_gcj2;
            first_point_y = lat_gcj2;
          }
        }

        for (int t = lane_info->right_lane_boundary_info->line_point_info.size() - 1; t >= 0; t--)
        {

          session->data_processer->local2wgs(lane_info->right_lane_boundary_info->line_point_info[t]->pos, wgs);

          double lng_gcj2 = wgs.x();
          double lat_gcj2 = wgs.y();
          // TODO:qzc gcj01
          // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
          // TODO:qzc gcj01

          outerRing->addPoint(lng_gcj2, lat_gcj2);
        }
        outerRing->addPoint(first_point_x, first_point_y); // 闭合多边形

        //判断是否在任务框内
        std::vector<Eigen::Vector3d> obj_points;
        std::vector<int> indexs;
        for (int i = 0; i < outerRing->getNumPoints(); ++i) {
          double x = outerRing->getX(i);  // 获取第 i 个点的 x 坐标
          double y = outerRing->getY(i);  // 获取第 i 个点的 y 坐标
          Eigen::Vector3d wgs = {x, y, 0};
          Eigen::Vector3d tar = wgs;
          session->data_processer->wgs2local(wgs, tar); 
          obj_points.push_back(tar);
        }

        if(alg::points_in_polygon(session->task_polygon_vec3d, obj_points, indexs) == false){
          delete polygon;
          delete outerRing;
          continue;
        }

        // 确保线性环闭合
        outerRing->closeRings();

        // 将线性环添加到多边形中
        polygon->addRing(outerRing);

        OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());

        // LOG_INFO("lane center id is : {}, lane center feature list size :{}, lane center bind arrow  list size :{}", lane_info->center_lane_boundary_info->lane_id, lane_info->lane_center_feature_list.size(), lane_info->bind_arrow_list.size());

        if (lane_info->lane_center_feature_list.size() > 0)
        {
          std::ostringstream turn_type_oss;
          for (size_t i = 0; i < lane_info->bind_arrow_list.size(); i++)
          {
            if (i > 0)
            {
              turn_type_oss << ",";
            }
            turn_type_oss << lane_info->bind_arrow_list[i]->type;
          }
          feature->SetField("TURN_TYPE", turn_type_oss.str().c_str()); // 车道转向类型
        }

        // feature->SetField("ID", lane_info->center_lane_boundary_info->lane_id); // 关联车道中心线ID
        feature->SetField("ID", lane_info->road_lane_id); // 车道ID
        feature->SetField("ROAD_ID", lane_info->lane_group_info->group_index);  // 归属车道组ID
        if (i > 0)
        {
          std::string left_in_str = std::to_string(lane_group->lane_line_info[i - 1]->road_lane_id);
          feature->SetField("LEFT_LN", left_in_str.c_str()); // 左侧同向车道ID
        }
        if (i < lane_group->lane_line_info.size() - 1)
        {
          std::string right_in_str = std::to_string(lane_group->lane_line_info[i + 1]->road_lane_id);
          feature->SetField("RIGHT_LN", right_in_str.c_str()); // 右侧同向车道ID
        }

        std::ostringstream prev_oss, next_oss;
        for (size_t i = 0; i < lane_info->context.all_prev.size(); i++)
        {
          if (i > 0)
            prev_oss << ",";
          prev_oss << lane_info->context.all_prev[i].src->road_lane_id;
        }

        for (size_t i = 0; i < lane_info->context.all_next.size(); i++)
        {
          if (i > 0)
            next_oss << ",";
          next_oss << lane_info->context.all_next[i].src->road_lane_id;
        }

        // feature->SetField("LANE_TYPE", lane_data->); // 车道类型，暂时没有
        // feature->SetField("LEFT_R_LN", lane_data->); // 左侧反向车道ID，暂时没有
        // feature->SetField("RIGHT_R_LN", prev_oss.str().c_str());// 右侧反向车道ID，暂时没有
        feature->SetField("LEFT_BID", lane_info->left_lane_boundary_info->lane_id);   // 左侧车道边界线ID
        feature->SetField("RIGHT_BID", lane_info->right_lane_boundary_info->lane_id); // 右侧车道边界线ID
        // feature->SetField("OBJ_ID", next_oss.str().c_str()); //关联定位要素ID，暂时没有
        // feature->SetField("OBJ_TYPE", next_oss.str().c_str()); //关联定位要素ID，暂时没有
        // feature->SetField("INTER_ID", next_oss); //所在路口编号，暂时没有
        feature->SetField("PRE_ID", prev_oss.str().c_str());  // 前序车道
        feature->SetField("NEXT_ID", next_oss.str().c_str()); // 后序车道

        // feature->SetField("FRAME_ID", );  // 暂时没有
        feature->SetField("VERSION", session->version.c_str());

        feature->SetGeometry(polygon);

        if (layer->CreateFeature(feature) != OGRERR_NONE)
        {
          std::cerr << "lane 创建要素失败" << std::endl;
        }

        OGRFeature::DestroyFeature(feature);
      }
    }

    // 关闭数据集
    GDALClose(dataset);

    return fsdmap::SUCC;
  }

  int RoadModelExportSHP::export_lane_adas_to_shp(RoadModelSessionData *session)
  {
    // 注册所有的驱动
    GDALAllRegister();

    // 创建数据集
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!dataset)
    {
      std::cerr << "lane adas 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
      return fsdmap::FAIL;
    }

    // 创建图层
    OGRSpatialReference poSpatialRef;
    poSpatialRef.importFromEPSG(4326);
    OGRLayer *layer = dataset->CreateLayer("lane_adas", &poSpatialRef, wkbLineString, nullptr);
    if (!layer)
    {
      std::cerr << "lane adas 创建图层失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 cl_id 字段
    OGRFieldDefn clidFieldDefn("CL_ID", OFTInteger);
    if (layer->CreateField(&clidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane adas  创建字段 CL_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 index 字段
    OGRFieldDefn indexFieldDefn("INDEX", OFTInteger);
    if (layer->CreateField(&indexFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane adas  创建字段 INDEX 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 curvature 字段
    OGRFieldDefn curvaturefieldDefn("CURVATURE", OFTInteger);
    if (layer->CreateField(&curvaturefieldDefn) != OGRERR_NONE)
    {
      std::cerr << "lane adas 创建字段 CURVATURE 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 写入数据
    int featureId = 0;
    for (const auto &lane_group : session->new_lane_groups)
    {
      for (const auto &lane_info : lane_group->lane_line_info)
      {
        for (int i = 0; i < lane_info->lane_center_feature_list.size(); i++)
        {
          double cura = lane_info->lane_center_feature_list[i]->curvature;
          cura = 1 / cura * 1E5;
          int K = cura > 0 ? static_cast<int>(cura) : static_cast<int>(std::floor(cura));

          OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());

          feature->SetField("CL_ID", lane_info->center_lane_boundary_info->lane_id);
          // feature->SetField("INDEX", static_cast<int>(lane_info->lane_center_feature_list[i]->line_index)); // 有问题
          feature->SetField("INDEX", i);
          feature->SetField("CURVATURE", K);
          // feature->SetField("VERSION", session->version.c_str());
          // feature->SetGeometry(&lineString);

          if (layer->CreateFeature(feature) != OGRERR_NONE)
          {
            std::cerr << "lane adas 创建要素失败" << std::endl;
          }

          OGRFeature::DestroyFeature(feature);
        }
      }
    }

    // 关闭数据集
    GDALClose(dataset);

    return fsdmap::SUCC;
  }

  int RoadModelExportSHP::export_road_centers_to_shp(RoadModelSessionData *session)
  {
    // 注册所有的驱动
    GDALAllRegister();

    // 创建数据集
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!dataset)
    {
      std::cerr << "road centers 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
      return fsdmap::FAIL;
      ;
    }

    // 创建图层
    OGRSpatialReference poSpatialRef;
    poSpatialRef.importFromEPSG(4326);
    OGRLayer *layer = dataset->CreateLayer("road_centers", &poSpatialRef, wkbLineString, nullptr);
    if (!layer)
    {
      std::cerr << "road centers 创建图层失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加字段
    OGRFieldDefn fieldDefn("ID", OFTInteger);
    if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road centers 创建字段 ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 road_id 字段
    OGRFieldDefn roadidfieldDefn("ROAD_ID", OFTInteger);
    if (layer->CreateField(&roadidfieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road centers 创建字段 road_id 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 frame_id 字段
    OGRFieldDefn frameidFieldDefn("FRAME_ID", OFTInteger);
    // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&frameidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road centers  创建字段 FRAME_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 version 字段
    OGRFieldDefn versionFieldDefn("VERSION", OFTString);
    // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road centers  创建字段 VERSION 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 写入数据
    int featureId = 0;
    for (const auto &lane_group : session->new_lane_groups)
    {
      auto road_center = lane_group->road_center_line_info;
#if 0
      if (road_center == NULL)
      {
        LOG_INFO("road_center is not exist");
      }
      else
      {
        LOG_INFO("road_center is exist");
      }
#endif
      if (road_center == NULL)
      {
        continue;
      }

      OGRLineString lineString;
      for (const auto &lane_point : road_center->line_point_info)
      {

        Eigen::Vector3d wgs;
        session->data_processer->local2wgs(lane_point->pos, wgs);

        double lng_gcj2 = wgs.x();
        double lat_gcj2 = wgs.y();
        // TODO:qzc gcj01
        // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
        // TODO:qzc gcj01

        lineString.addPoint(lng_gcj2, lat_gcj2);
      }

      OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
      feature->SetField("ID", road_center->lane_id);
      feature->SetField("ROAD_ID", lane_group->group_index); // 暂时没有
      // feature->SetField("FRAME_ID", );  // 暂时没有
      // feature->SetField("VERSION", );  // 暂时没有
      feature->SetGeometry(&lineString);

      if (layer->CreateFeature(feature) != OGRERR_NONE)
      {
        std::cerr << "road centers 创建要素失败" << std::endl;
      }

      OGRFeature::DestroyFeature(feature);
    }

    // 关闭数据集
    GDALClose(dataset);

    return fsdmap::SUCC;
  }

  int RoadModelExportSHP::export_stop_line_to_shp(RoadModelSessionData *session)
  {
    // 注册所有的驱动
    GDALAllRegister();

    // 创建数据集
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!dataset)
    {
      std::cerr << "stop line 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
      return fsdmap::FAIL;
      ;
    }

    // 创建图层
    OGRSpatialReference poSpatialRef;
    poSpatialRef.importFromEPSG(4326);
    OGRLayer *layer = dataset->CreateLayer("stop_line", &poSpatialRef, wkbLineString, nullptr);
    if (!layer)
    {
      std::cerr << "stop line 创建图层失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加ID字段
    OGRFieldDefn fieldDefn("ID", OFTInteger);
    if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
    {
      std::cerr << "stop line 创建字段失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 停止线TYPE 字段
    OGRFieldDefn lineTypeFieldDefn("TYPE", OFTInteger);
    if (layer->CreateField(&lineTypeFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "stop line 创建字段 TYPE 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 color 字段
    OGRFieldDefn colorFieldDefn("COLOR", OFTInteger);
    if (layer->CreateField(&colorFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "stop line 创建字段 COLOR 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }
    // 添加 SHAPE_TYPE 字段
    OGRFieldDefn shapeTypeFieldDefn("SHAPE_TYPE", OFTInteger);
    if (layer->CreateField(&shapeTypeFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "stop line 创建字段 SHAPE 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 LANE_ID 字段
    OGRFieldDefn laneIdFieldDefn("LANE_ID", OFTString);
    if (layer->CreateField(&laneIdFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "stop line 创建字段 LANE_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 FRAME_ID 字段
    OGRFieldDefn frameIdFieldDefn("FRAME_ID", OFTInteger);
    if (layer->CreateField(&frameIdFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "stop line 创建字段 FRAME_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 VERSION 字段
    OGRFieldDefn versionFieldDefn("VERSION", OFTString);
    if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "stop line 创建字段 VERSION 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }


    // for (const auto& polygon : session->task_polygon) {
    //     // 获取外环的坐标
    //     const auto& outer_ring = polygon.outer();  // 获取外环坐标
    //     std::cout << "Exterior Ring (Outer Ring):" << std::endl;
    //     for (const auto& pt : outer_ring) {
    //         std::cout << "(" << boost::geometry::get<0>(pt) << ", " << boost::geometry::get<1>(pt) << ")" << std::endl;
    //     }
    // }

    // 写入数据
    for (const auto &object : session->raw_object_ret_list)
    {
      if (object->ele_type == 6)
      {
        //1 剔除
        std::vector<Eigen::Vector3d> obj_points;
        std::vector<int> indexs;
        for(const auto &point : object->list){
          obj_points.push_back(point->pos);
        }

        if(alg::points_in_polygon(session->task_polygon_vec3d, obj_points, indexs) == false){
            continue;
        }

        //2 写入数据
        OGRLineString lineString;
        for (const auto &point : object->list)
        {
          Eigen::Vector3d wgs;
          session->data_processer->local2wgs(point->pos, wgs);
          // double lng_gcj2;
          // double lat_gcj2;
          // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

          double lng_gcj2 = wgs.x();
          double lat_gcj2 = wgs.y();
          // TODO:qzc gcj01
          // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
          // TODO:qzc gcj01

          // Eigen::Vector3d lng_lat_02;
          // lng_lat_02.x() = lng_gcj2;
          // lng_lat_02.y() = lat_gcj2;
          // lng_lat_02.z() = 0;
          // Eigen::Vector3d utm_02;
          // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
          // lineString.addPoint(utm_02.x(), utm_02.y());
          lineString.addPoint(lng_gcj2, lat_gcj2);

          // lineString.addPoint(point->pos.x(), point->pos.y());
        }

        std::ostringstream lane_id_oss;
        for(int i = 0; i< object->obj_bind_lanes.size(); i++)
        {
          if(i >0){
            lane_id_oss << ",";
          }
          lane_id_oss << object->obj_bind_lanes[i]->road_lane_id;
        }

        OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
        feature->SetField("ID", object->obj_id);
        feature->SetField("TYPE", std::stoi(object->type));
        feature->SetField("LANE_ID", lane_id_oss.str().c_str());
        // feature->SetField("COLOR", object->color);
        // feature->SetField("SHAPE", object->shape);
        // feature->SetField("LENGTH", object->length);
        feature->SetField("VERSION", session->version.c_str());
        feature->SetGeometry(&lineString);

        if (layer->CreateFeature(feature) != OGRERR_NONE)
        {
          std::cerr << "stop line 创建要素失败" << std::endl;
        }

        OGRFeature::DestroyFeature(feature);
      }
    }

    // 关闭数据集
    GDALClose(dataset);

    return fsdmap::SUCC;
  }

  int RoadModelExportSHP::export_cross_walk_to_shp(RoadModelSessionData *session)
  {
    // 注册所有的驱动
    GDALAllRegister();

    // 创建数据集
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!dataset)
    {
      std::cerr << "cross walk 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
      return fsdmap::FAIL;
      ;
    }

    // 创建图层
    OGRSpatialReference poSpatialRef;
    poSpatialRef.importFromEPSG(4326);
    OGRLayer *layer = dataset->CreateLayer("cross_walk", &poSpatialRef, wkbPolygon, nullptr);
    if (!layer)
    {
      std::cerr << "cross walk 创建图层失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加ID字段
    OGRFieldDefn fieldDefn("ID", OFTInteger);
    if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
    {
      std::cerr << "cross walk 创建ID字段失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 LANE_ID 字段
    OGRFieldDefn laneIdFieldDefn("LANE_ID", OFTString);
    if (layer->CreateField(&laneIdFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "cross walk创建字段 LANE_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }
    // 添加 JUNC_ID 字段
    OGRFieldDefn juncIdFieldDefn("JUNC_ID", OFTString);
    if (layer->CreateField(&juncIdFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "cross walk创建字段 JUNC_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 FRAME_ID 字段
    OGRFieldDefn frameIdFieldDefn("FRAME_ID", OFTInteger);
    if (layer->CreateField(&frameIdFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "cross walk创建字段 FRAME_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 VERSION 字段
    OGRFieldDefn versionFieldDefn("VERSION", OFTString);
    if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "cross walk创建字段 VERSION 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 写入数据
    for (const auto &object : session->raw_object_ret_list)
    {
      if (object->ele_type == 5)
      {
        //1 剔除
        std::vector<Eigen::Vector3d> obj_points;
        std::vector<int> indexs;
        for(const auto &point : object->list){
          obj_points.push_back(point->pos);
        }

        if(alg::points_in_polygon(session->task_polygon_vec3d, obj_points, indexs) == false){
            continue;
        }

        
        //2. 创建一个多边形对象来表示块状区域
        OGRPolygon polygon;
        // 创建一个线性环，用于定义多边形的边界
        OGRLinearRing *ring = new OGRLinearRing();

        for (const auto &point : object->list){
          Eigen::Vector3d wgs;
          session->data_processer->local2wgs(point->pos, wgs);

          double lng_gcj2 = wgs.x();
          double lat_gcj2 = wgs.y();
          // TODO:qzc gcj01
          // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
          // TODO:qzc gcj01
          ring->addPoint(lng_gcj2, lat_gcj2);
        }

        std::ostringstream lane_id_oss,inter_id_oss;
        for(int i = 0; i< object->obj_bind_lanes.size(); i++)
        {
          if(i >0){
            lane_id_oss << ",";
          }
          lane_id_oss << object->obj_bind_lanes[i]->road_lane_id;
        }

        for(int i =0; i<object->obj_bind_intersections.size(); i++)
        {
          if(i >0){
            inter_id_oss << ",";
          }
          inter_id_oss << object->obj_bind_intersections[i]->id;
        }

        // 确保线性环闭合
        ring->closeRings();

        // 将线性环添加到多边形中
        polygon.addRing(ring);

        // 创建要素并设置字段值
        OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
        feature->SetField("ID", object->obj_id);
        feature->SetField("LANE_ID", lane_id_oss.str().c_str());  //
        feature->SetField("JUNC_ID", inter_id_oss.str().c_str());
        feature->SetField("VERSION", session->version.c_str());

        // 设置要素的几何对象为多边形
        feature->SetGeometry(&polygon);

        // 将要素添加到图层中
        if (layer->CreateFeature(feature) != OGRERR_NONE)
        {
          std::cerr << "cross walk 创建要素失败" << std::endl;
        }

        OGRFeature::DestroyFeature(feature);
        delete ring;
      }
    }

    // 关闭数据集
    GDALClose(dataset);

    return fsdmap::SUCC;
  }

  int RoadModelExportSHP::export_road_mark_to_shp(RoadModelSessionData *session)
  {
    // 注册所有的驱动
    GDALAllRegister();

    // 创建数据集
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!dataset)
    {
      std::cerr << "arrow创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
      return fsdmap::FAIL;
      ;
    }

    // 创建arrow图层
    OGRSpatialReference poSpatialRef;
    poSpatialRef.importFromEPSG(4326);
    OGRLayer *layer = dataset->CreateLayer("arrow", &poSpatialRef, wkbPolygon, nullptr);
    if (!layer)
    {
      std::cerr << "arrow创建图层失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加字段
    OGRFieldDefn fieldDefn("ID", OFTInteger);
    if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
    {
      std::cerr << "arrow创建字段失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 箭头TYPE 字段
    OGRFieldDefn typeFieldDefn("TYPE", OFTString);
    if (layer->CreateField(&typeFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "arrow 创建字段 TYPE 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 LANE_ID 字段
    OGRFieldDefn laneIdFieldDefn("LANE_ID", OFTString);
    if (layer->CreateField(&laneIdFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "arrow 创建字段 LANE_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 FRAME_ID 字段
    OGRFieldDefn frameIdFieldDefn("FRAME_ID", OFTInteger);
    if (layer->CreateField(&frameIdFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "arrow 创建字段 FRAME_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 VERSION 字段
    OGRFieldDefn versionFieldDefn("VERSION", OFTString);
    if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "arrow 创建字段 VERSION 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 写入数据
    for (const auto &object : session->raw_object_ret_list)
    {
      if (object->ele_type == 3)
      {

        //1 剔除
        std::vector<Eigen::Vector3d> obj_points;
        std::vector<int> indexs;
        for(const auto &point : object->list){
          obj_points.push_back(point->pos);
        }

        if(alg::points_in_polygon(session->task_polygon_vec3d, obj_points, indexs) == false){
            continue;
        }

        // 创建一个多边形对象来表示块状区域
        OGRPolygon polygon;
        // 创建一个线性环，用于定义多边形的边界
        OGRLinearRing *ring = new OGRLinearRing();
        for (const auto &point : object->list)
        {
          Eigen::Vector3d wgs;
          session->data_processer->local2wgs(point->pos, wgs);
          // double lng_gcj2;
          // double lat_gcj2;
          // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

          double lng_gcj2 = wgs.x();
          double lat_gcj2 = wgs.y();
          // TODO:qzc gcj01
          // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
          // TODO:qzc gcj01

          // Eigen::Vector3d lng_lat_02;
          // lng_lat_02.x() = lng_gcj2;
          // lng_lat_02.y() = lat_gcj2;
          // lng_lat_02.z() = 0;
          // Eigen::Vector3d utm_02;
          // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
          // lineString.addPoint(utm_02.x(), utm_02.y());

          ring->addPoint(lng_gcj2, lat_gcj2);

          // lineString.addPoint(point->pos.x(), point->pos.y());
        }
        std::ostringstream lane_id_oss;
        for(int i = 0; i< object->obj_bind_lanes.size(); i++)
        {
          if(i >0){
            lane_id_oss << ",";
          }
          lane_id_oss << object->obj_bind_lanes[i]->road_lane_id;
        }

        
        // 确保线性环闭合
        ring->closeRings();
        // 将线性环添加到多边形中
        polygon.addRing(ring);

        OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
        feature->SetField("ID", object->obj_id);
        feature->SetField("TYPE", object->type.c_str());
        feature->SetField("LANE_ID", lane_id_oss.str().c_str());
        feature->SetField("VERSION", session->version.c_str());
        feature->SetGeometry(&polygon);

        if (layer->CreateFeature(feature) != OGRERR_NONE)
        {
          std::cerr << "arrow创建要素失败" << std::endl;
        }

        OGRFeature::DestroyFeature(feature);
      }
    }

    // 关闭数据集
    GDALClose(dataset);

    return fsdmap::SUCC;
  }

  int RoadModelExportSHP::export_intersection_to_shp(RoadModelSessionData *session)
  {
    // 注册所有的驱动
    GDALAllRegister();

    // 创建数据集
    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!dataset)
    {
      std::cerr << "junction 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
      return fsdmap::FAIL;
      ;
    }

    // 创建图层
    OGRSpatialReference poSpatialRef;
    poSpatialRef.importFromEPSG(4326);
    OGRLayer *layer = dataset->CreateLayer("junction", &poSpatialRef, wkbPolygon, nullptr);
    if (!layer)
    {
      std::cerr << "junction 创建图层失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加ID字段
    OGRFieldDefn fieldDefn("ID", OFTInteger);
    if (layer->CreateField(&fieldDefn) != OGRERR_NONE)
    {
      std::cerr << "junction 创建字段失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加TYPE 字段
    OGRFieldDefn typeFieldDefn("TYPE", OFTInteger);
    if (layer->CreateField(&typeFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "junction 创建字段 TYPE 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 IS_EFFECT  字段
    OGRFieldDefn iseffectFieldDefn("IS_EFFECT", OFTInteger);
    if (layer->CreateField(&iseffectFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "junction 创建字段 IS_EFFECT 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 lane road  字段
    OGRFieldDefn laneIdFieldDefn("LANE_ID", OFTString);
    if (layer->CreateField(&laneIdFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "junction 创建字段 LANE_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    OGRFieldDefn inRoadFieldDefn("IN_ROAD", OFTString);
    if (layer->CreateField(&inRoadFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "junction 创建字段 IN_ROAD 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }
    OGRFieldDefn outRoadFieldDefn("OUT_ROAD", OFTString);
    if (layer->CreateField(&outRoadFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "junction 创建字段 IN_ROAD 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    OGRFieldDefn inLaneFieldDefn("IN_LANE", OFTString);
    if (layer->CreateField(&inLaneFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "junction 创建字段 IN_LANE 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    OGRFieldDefn outLaneFieldDefn("OUT_LANE", OFTString);
    if (layer->CreateField(&outLaneFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "junction 创建字段 OUT_LANE 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 FRAME_ID 字段
    OGRFieldDefn frameIdFieldDefn("FRAME_ID", OFTInteger);
    if (layer->CreateField(&frameIdFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "junction创建字段 FRAME_ID 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 添加 VERSION 字段
    OGRFieldDefn versionFieldDefn("VERSION", OFTString);
    if (layer->CreateField(&versionFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "junction 创建字段 VERSION 失败" << std::endl;
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    // 写入数据
    for (const auto &inter : session->raw_intersections)
    {
      //1 剔除
      std::vector<Eigen::Vector3d> obj_points;
      std::vector<int> indexs;
      for(const auto &point:inter->point_info){
        obj_points.push_back(point->pos);
      }

      if(alg::points_in_polygon(session->task_polygon_vec3d, obj_points, indexs) == false){
          continue;
      }
      
      // 创建一个多边形对象来表示块状区域
      OGRPolygon polygon;
      // 创建一个线性环，用于定义多边形的边界
      OGRLinearRing *ring = new OGRLinearRing();
      for (auto &pt : inter->point_info)
      {
        Eigen::Vector3d wgs;
        session->data_processer->local2wgs(pt->pos, wgs);
        // double lng_gcj2;
        // double lat_gcj2;
        // alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);

        double lng_gcj2 = wgs.x();
        double lat_gcj2 = wgs.y();
        // TODO:qzc gcj01
        // alg::wgs84_to_gcj02(lng_gcj2, lat_gcj2);
        // TODO:qzc gcj01

        // Eigen::Vector3d lng_lat_02;
        // lng_lat_02.x() = lng_gcj2;
        // lng_lat_02.y() = lat_gcj2;
        // lng_lat_02.z() = 0;
        // Eigen::Vector3d utm_02;
        // session->data_processer->wgs2local(lng_lat_02, utm_02, false);
        // lineString.addPoint(utm_02.x(), utm_02.y());
        ring->addPoint(lng_gcj2, lat_gcj2);

        // lineString.addPoint(pt->pos.x(), pt->pos.y());
      }

      std::ostringstream lane_id_oss, in_road_oss, out_road_oss, in_lane_oss, out_lane_oss;
      //车道
      for(int i = 0; i< inter->all_lanes.size(); i++)
      {
        if(i >0){
          lane_id_oss << ",";
        }
        lane_id_oss << inter->all_lanes[i]->road_lane_id;
      }

      for(int i = 0; i< inter->in_lanes.size(); i++)
      {
        if(i >0){
          in_lane_oss << ",";
        }
        in_lane_oss << inter->all_lanes[i]->road_lane_id;
      }

      for(int i = 0; i< inter->out_lanes.size(); i++)
      {
        if(i >0){
          out_lane_oss << ",";
        }
        out_lane_oss << inter->all_lanes[i]->road_lane_id;
      }
      // //进入退出道路
      std::vector<RoadLaneGroupInfo*> in_loads_vector(inter->in_loads.begin(), inter->in_loads.end());
      std::vector<RoadLaneGroupInfo*> out_loads_vector(inter->out_loads.begin(), inter->out_loads.end());
      for(int i = 0; i< in_loads_vector.size(); i++)
      {
        if(i >0){
          in_road_oss << ",";
        }
        in_road_oss << in_loads_vector[i]->group_index;
      }

      for(int i = 0; i< out_loads_vector.size(); i++)
      {
        if(i >0){
          out_road_oss << ",";
        }
        out_road_oss << out_loads_vector[i]->group_index;
      }

      // 确保线性环闭合
      ring->closeRings();
      polygon.addRing(ring);

      OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
      feature->SetField("ID", inter->id);
      feature->SetField("LANE_ID", lane_id_oss.str().c_str());  
      feature->SetField("IN_LANE", in_lane_oss.str().c_str());  
      feature->SetField("OUT_LANE", out_lane_oss.str().c_str()); 
      feature->SetField("IN_ROAD", in_road_oss.str().c_str());  
      feature->SetField("OUT_ROAD", out_road_oss.str().c_str()); 
      feature->SetField("VERSION", session->version.c_str());
      for (auto &in_lane : inter->in_lanes)
      {
      }
      feature->SetGeometry(&polygon);

      if (layer->CreateFeature(feature) != OGRERR_NONE)
      {
        std::cerr << "junction 创建要素失败" << std::endl;
      }

      OGRFeature::DestroyFeature(feature);
    }
    // 关闭数据集
    GDALClose(dataset);
    return fsdmap::SUCC;
  }

}
}

#include "road_model_proc_export_shp.h"
#include "utils/algorithm_util.h"

namespace fsdmap
{
  namespace road_model
  {

    process_frame::PROC_STATUS RoadModelExportSHP::proc(
        RoadModelSessionData *session)
    {
      export_type_=2;
      if (export_type_ == 2) {
        LOG_INFO("export shp type:{}", export_type_);
        CHECK_FATAL_PROC(export_to_shape_file_with_topo(session), "export_to_shape_file_with_topo");
      } else {
        LOG_WARN("invalid export_type_:{}", export_type_);
      }
      return fsdmap::process_frame::PROC_STATUS_SUCC;
    }
    
    /********************************* 2 矢量化+拓扑 *******************************************/
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

      export_stop_line_to_shp(session);
      
      export_cross_walk_to_shp(session);

      export_road_mark_to_shp(session);

      export_intersection_to_shp(session);

      export_lane_boundary_to_shp_vectorize1(session, 1);

      export_road_boundary_to_shp_vectorize1(session, 1);

      return fsdmap::SUCC;
    }
    
    bool RoadModelExportSHP::create_field_defn(OGRLayer *layer, const std::string &key, OGRFieldType type, const std::string &element)
    {
      // 添加 ID 字段
      OGRFieldDefn field_defn(key.c_str(), type);
      if (layer->CreateField(&field_defn) != OGRERR_NONE)
      {
        LOG_WARN("create:{} key:{} type:{} failed", element, key, static_cast<int>(type));
        return false;
      }
      return true;
    }
    

    int RoadModelExportSHP::export_lane_boundary_to_shp_vectorize1(RoadModelSessionData* session, int flag)
    {
      // 注册所有的驱动
      GDALAllRegister();

      std::string path_str = flag == 0 ? FLAGS_mid_shp_file_dir : FLAGS_shp_file_dir;
      int offset = flag == 0 ? 0 : 3 * 1e8;
      // 创建数据集
      GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
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

      bool create = true;
      create &= create_field_defn(layer, "ID", OFTString, "lane_boundary_v");
      create &= create_field_defn(layer, "COLOR", OFTInteger, "lane_boundary_v");
      create &= create_field_defn(layer, "SHAPE_TYPE", OFTInteger, "lane_boundary_v");
      create &= create_field_defn(layer, "LANE_ID", OFTString, "lane_boundary_v");
      create &= create_field_defn(layer, "PRE_ID", OFTString, "lane_boundary_v");
      create &= create_field_defn(layer, "NEXT_ID", OFTString, "lane_boundary_v");
      create &= create_field_defn(layer, "ALG_ID", OFTInteger, "lane_boundary_v");
      create &= create_field_defn(layer, "VERSION", OFTString, "lane_boundary_v");

      if(!create)
      {
        GDALClose(dataset);
        return fsdmap::FAIL;  
      }

      auto write_one_lane_boundary = [session, layer](const LaneLineSampleGroupLine* lane_boundary, 
        int featureId)
      {

          std::vector<Eigen::Vector3d> temp_points;
          if (!lane_boundary){
              // LOG_WARN("lane_boundary null");
              return;
          }
          
          for (auto point : lane_boundary->list){
              temp_points.push_back(point->pos);
          }

          temp_points=alg::cut_line_in_polygon(session->task_polygon_vec3d,temp_points);
          if(temp_points.size()<2){
              LOG_WARN("lane_boundary:[{}] points:{} <2",uint64_t(lane_boundary),temp_points.size());
              return ;
          }

          OGRLineString lineString; // 经纬高
          for (auto p : temp_points){
              Eigen::Vector3d wgs;
              session->data_processer->local2wgs(p, wgs);
              lineString.addPoint(wgs.x(), wgs.y());
          }
          if (lineString.IsEmpty())return;
              
          OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
          feature->SetField("ID", (session->frame_id + "_" + std::to_string(featureId)).c_str());
          feature->SetField("ALG_ID", std::stoi(session->frame_id));  
          feature->SetField("VERSION", session->version.c_str());
          feature->SetGeometry(&lineString);
          if (layer->CreateFeature(feature) != OGRERR_NONE){
              std::cerr << "lane_boundary 创建要素失败" << std::endl;
          }
          OGRFeature::DestroyFeature(feature);
      };

      //写入数据
      int featureId = 0;
      featureId += offset;
      for (const auto& object : session->merge_lane_line_list) {
        if(!object){
            LOG_WARN("object null");
            continue;
        }
        write_one_lane_boundary(object, featureId);
        featureId++;
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

        bool create = true;
        create &= create_field_defn(layer, "ID", OFTString, "road_boundary_v");
        create &= create_field_defn(layer, "COLOR", OFTInteger, "road_boundary_v");
        create &= create_field_defn(layer, "TYPE", OFTInteger, "road_boundary_v");
        create &= create_field_defn(layer, "IS_SAFETY", OFTInteger, "road_boundary_v");
        create &= create_field_defn(layer, "ROAD_ID", OFTString, "road_boundary_v");
        create &= create_field_defn(layer, "PRE_ID", OFTString, "road_boundary_v");
        create &= create_field_defn(layer, "NEXT_ID", OFTString, "road_boundary_v");
        create &= create_field_defn(layer, "ALG_ID", OFTInteger, "road_boundary_v");
        create &= create_field_defn(layer, "VERSION", OFTString, "road_boundary_v");

        if(!create)
        {
          GDALClose(dataset);
          return fsdmap::FAIL;  
        }

        auto write_one_road_boundary = [session, layer](const BoundaryGroupLine* road_boundary, 
          int featureId)
        {
            std::vector<Eigen::Vector3d> temp_points;
            if (!road_boundary){
                // LOG_WARN("road_boundary null");
                return;
            }
            
            for (auto point : road_boundary->list){
                temp_points.push_back(point->pos);
            }

            temp_points=alg::cut_line_in_polygon(session->task_polygon_vec3d,temp_points);
            if(temp_points.size()<2){
                LOG_WARN("road_boundary:[{}] points:{} <2",uint64_t(road_boundary),temp_points.size());
                return ;
            }

            OGRLineString lineString; // 经纬高
            for (auto p : temp_points){
                Eigen::Vector3d wgs;
                session->data_processer->local2wgs(p, wgs);
                lineString.addPoint(wgs.x(), wgs.y());
            }
            if (lineString.IsEmpty())return;
            
            OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
            feature->SetField("ID", (session->frame_id + "_" + std::to_string(featureId++)).c_str());
            feature->SetField("ALG_ID", std::stoi(session->frame_id));  
            feature->SetField("VERSION", session->version.c_str());
            feature->SetGeometry(&lineString);

            if (layer->CreateFeature(feature) != OGRERR_NONE){
                std::cerr << "road_boundary 创建要素失败" << std::endl;
            }

            OGRFeature::DestroyFeature(feature);
        };

        // 写入数据
        int featureId = 0;
        featureId += offset;
        for (const auto& object : session->merge_boundary_line_list) {
          if(!object){
              LOG_WARN("object null");
              continue;
          }
          write_one_road_boundary(object, featureId);
          featureId++;
        }


        GDALClose(dataset);
        
        return fsdmap::SUCC;
    }

    int RoadModelExportSHP::export_lane_boundary_to_shp(RoadModelSessionData *session)
    {
      // 注册所有的驱动
      GDALAllRegister();
      // 创建数据集
      GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
      GDALDataset *dataset = driver->Create(FLAGS_shp_file_dir.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
      if (!dataset){
          std::cerr << "lane boundary 创建数据集失败，路径:" << FLAGS_shp_file_dir << std::endl;
          return fsdmap::FAIL;
      }
      // 创建图层
      OGRSpatialReference poSpatialRef;
      poSpatialRef.importFromEPSG(4326);
      OGRLayer *layer = dataset->CreateLayer("lane_boundary", &poSpatialRef, wkbLineString, nullptr);
      if (!layer){
          std::cerr << "lane boundary 创建图层失败" << std::endl;
          GDALClose(dataset);
          return fsdmap::FAIL;
      }

      bool create=true;
      create &= create_field_defn(layer,"ID", OFTString, "lane_boundary");
      create &=create_field_defn(layer,"COLOR", OFTInteger, "lane_boundary");
      create &=create_field_defn(layer,"SHAPE_TYPE", OFTInteger, "lane_boundary");
      create &=create_field_defn(layer,"LANE_ID", OFTString, "lane_boundary");
      create &=create_field_defn(layer,"PRE_ID", OFTString, "lane_boundary");
      create &=create_field_defn(layer,"NEXT_ID", OFTString, "lane_boundary");
      create &=create_field_defn(layer,"ALG_ID", OFTInteger, "lane_boundary");
      create &= create_field_defn(layer,"VERSION", OFTString, "lane_boundary");
      if(!create){
          GDALClose(dataset);
          return fsdmap::FAIL;  
      }
      auto write_one_line=[session,layer](const std::vector<fast_road_model::LanePtr> &lane_list, int index,const  fast_road_model::LaneBoundaryPtr lane_boundary )
      {
        std::vector<Eigen::Vector3d> temp_points;
        std::vector<fast_road_model::LanePtr> neighbor_lane;
        for(auto lane:lane_list)
        {
          if(!lane)
          {
            continue;
          }
          auto lbs={lane->left,lane->right};
          for(auto lb:lbs)
          {
            if(lb==lane_boundary)
            {
              neighbor_lane.push_back(lane);
            }
          }
        }

        const int n=lane_list.size();
        auto &lane = lane_list[index];
        if (!lane)
        {
          LOG_WARN("lane null");
          return;
        }
        if (!lane_boundary)
        {
          LOG_WARN("lane_boundary null");
          return;
        }
        //
        for (auto point : lane_boundary->points)
        {
          temp_points.push_back(point->pos);
        }
        if(temp_points.size()<2)
        {
          LOG_WARN("lane_boundary points:{} <2",temp_points.size());
          return ;
        }
        // TODO cut line  
        temp_points=alg::cut_line_in_polygon(session->task_polygon_vec3d,temp_points);
   
        OGRLineString lineString; // 经纬高
        for (auto p : temp_points)
        {
          Eigen::Vector3d wgs;
          session->data_processer->local2wgs(p, wgs);
          lineString.addPoint(wgs.x(), wgs.y());
        }
        if (lineString.IsEmpty())
        {
          return;
        }
        std::ostringstream prev_oss, next_oss,lane_id;
        auto &all_prev= lane_boundary->get_prev();
        auto &all_next= lane_boundary->get_next();
        for ( auto iter=all_prev.begin();iter!=all_prev.end();iter++)
        {
          if(iter!=all_prev.begin())
          {
            prev_oss<<","<<session->frame_id + "_" + std::to_string((*iter)->id);
          }else
          {
            prev_oss << session->frame_id + "_" + std::to_string((*iter)->id);
          }
        }
        for ( auto iter=all_next.begin();iter!=all_next.end();iter++)
        {
          if(iter!=all_next.begin())
          {
            next_oss<<","<<session->frame_id + "_" + std::to_string((*iter)->id);
          }else
          {
            next_oss<<session->frame_id + "_" + std::to_string((*iter)->id);
          }
        }
 
        for(int i=0;i<neighbor_lane.size();i++)
        {
          if(i==0)
          {
            lane_id<< session->frame_id + "_" +  std::to_string(neighbor_lane[i]->id);
          }else
          {
            lane_id<<","<< session->frame_id + "_" +  std::to_string(neighbor_lane[i]->id);
          }
        }
        OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
        feature->SetField("LANE_ID", lane_id.str().c_str());   // TODO 关联
        feature->SetField("ID",(session->frame_id + "_" + std::to_string(lane_boundary->id)).c_str());
        // int color=lane_data->color;
        // int type=lane_data->type;
        // color=color<=1?1:color;
        // type=type<=1?1:type;
        // feature->SetField("COLOR",color );
        // feature->SetField("SHAPE_TYPE",type ); // 暂时没有
        // feature->SetField("IS_SAFETY", lane_data->geo);  // 暂时没有
        int type=lane_boundary->type;
        // type=type==1?0:type;
        feature->SetField("SHAPE_TYPE", type);
        int color=lane_boundary->color;
        // color=color==1?0:color;
        feature->SetField("COLOR",color);
        // LOG_WARN( "lane boundary 输出属性：{}", lane_boundary->type);
        feature->SetField("PRE_ID", prev_oss.str().c_str());
        feature->SetField("NEXT_ID", next_oss.str().c_str());
        feature->SetField("ALG_ID", std::stoi(session->frame_id));  
        feature->SetField("VERSION", session->version.c_str());
        feature->SetGeometry(&lineString);
        if (layer->CreateFeature(feature) != OGRERR_NONE)
        {
         LOG_WARN( "lane boundary 创建要素失败" );
        }
        OGRFeature::DestroyFeature(feature);
      };

      auto write_one_lane = [layer,write_one_line](const std::vector<fast_road_model::LanePtr> &lane_list, int index,
        std::set<fast_road_model::LaneBoundaryPtr>&hash_table)
      {
        auto &lane=lane_list[index];
        auto lane_boundarys={lane->left,lane->right};
        for(auto lb:lane_boundarys)
        {
          if(hash_table.count(lb))
          {
            continue;
          }
          hash_table.insert(lb);
          write_one_line(lane_list,index,lb);
        }
      };

      // 写入数据
      for (auto rs : session->road_segments)
      {
        for (auto lane_group : rs->lane_group_list)
        {
          // lane boundary && lane center
          std::set<fast_road_model::LaneBoundaryPtr> hash_table;
          int n = lane_group->lane_list.size();
          for (int i = 0; i < n; i++)
          {
            write_one_lane(lane_group->lane_list, i, hash_table);
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

    bool create=true;
    create &= create_field_defn(layer,"ID", OFTString, "lane_center");
    create &=create_field_defn(layer,"TYPE", OFTInteger, "lane_center");
    create &=create_field_defn(layer,"LANE_ID", OFTString, "lane_center");
    create &=create_field_defn(layer,"PRE_ID", OFTString, "lane_center");
    create &=create_field_defn(layer,"NEXT_ID", OFTString, "lane_center");
    create &=create_field_defn(layer,"ALG_ID", OFTInteger, "lane_boundary");
    create &= create_field_defn(layer,"VERSION", OFTString, "lane_boundary");
    if(!create)
    {
      GDALClose(dataset);
      return fsdmap::FAIL;  
    }

    auto write_one_lane=[session,layer](const std::vector<fast_road_model::LanePtr> &lane_list, int index)
    {
      std::vector<Eigen::Vector3d> temp_points;
      const int n=lane_list.size();
      auto &lane = lane_list[index];
      if (!lane)
      {
        LOG_WARN("lane null");
        return;
      }
      auto center= lane->center;
      if (!center)
      {
        LOG_WARN("center null");
        return;
      }
      //
      for (auto point : center->points)
      {
        temp_points.push_back(point->pos);
      }

      temp_points=alg::cut_line_in_polygon(session->task_polygon_vec3d,temp_points);
      
      if(temp_points.size()<2)
      {
        LOG_WARN("lane_center points:{} <2",temp_points.size());
        return ;
      }
      //
      OGRLineString lineString; // 经纬高
      for (auto p : temp_points)
      {
        Eigen::Vector3d wgs;
        session->data_processer->local2wgs(p, wgs);
        lineString.addPoint(wgs.x(), wgs.y());
      }
      if (lineString.IsEmpty())
      {
        return;
      }
      std::ostringstream prev_oss, next_oss;
      auto &all_prev= center->get_prev();
      auto &all_next= center->get_next();
      for ( auto iter=all_prev.begin();iter!=all_prev.end();iter++)
      {
        if(iter!=all_prev.begin())
        {
          prev_oss<<","<< session->frame_id + "_" + std::to_string((*iter)->id);
        }else
        {
          prev_oss<< session->frame_id + "_" + std::to_string((*iter)->id);
        }
      }
      for ( auto iter=all_next.begin();iter!=all_next.end();iter++)
      {
        if(iter!=all_next.begin())
        {
          next_oss<<","<< session->frame_id + "_" + std::to_string((*iter)->id);
        }else
        {
          next_oss<< session->frame_id + "_" + std::to_string((*iter)->id);
        }
      }
      OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
      feature->SetField("ID", (session->frame_id + "_" + std::to_string(center->id)).c_str());   //TODO 关联
      feature->SetField("LANE_ID", (session->frame_id + "_" + std::to_string(lane->id)).c_str());   //TODO 关联
      feature->SetField("PRE_ID", prev_oss.str().c_str());
      feature->SetField("NEXT_ID", next_oss.str().c_str());
      feature->SetField("ALG_ID", std::stoi(session->frame_id)); 
      feature->SetField("VERSION", session->version.c_str());
      feature->SetGeometry(&lineString);
      if (layer->CreateFeature(feature) != OGRERR_NONE)
      {
        LOG_WARN( "lane center 创建要素失败" );
      }
      OGRFeature::DestroyFeature(feature);
    };
    // 写入数据
    for (auto rs : session->road_segments)
    {
      for (auto lane_group : rs->lane_group_list)
      {
        // lane boundary && lane center
        int n = lane_group->lane_list.size();
        for (int i = 0; i < n; i++)
        {
          write_one_lane(lane_group->lane_list, i);
        }
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

    bool create = true;
    create &= create_field_defn(layer, "ID", OFTString, "road_boundary");
    // create &=create_field_defn(layer,"COLOR", OFTInteger, "road_boundary");
    create &= create_field_defn(layer, "TYPE", OFTInteger, "road_boundary");
    create &= create_field_defn(layer, "IS_SAFETY", OFTInteger, "road_boundary");
    create &= create_field_defn(layer, "ROAD_ID", OFTString, "road_boundary");
    create &= create_field_defn(layer, "PRE_ID", OFTString, "road_boundary");
    create &= create_field_defn(layer, "NEXT_ID", OFTString, "road_boundary");
    create &= create_field_defn(layer, "ALG_ID", OFTInteger, "road_boundary");
    create &= create_field_defn(layer, "VERSION", OFTString, "road_boundary");
    if (!create)
    {
      GDALClose(dataset);
      return fsdmap::FAIL;
    }
    auto write_one_road_boundary = [session, layer](const fast_road_model::RoadBoundaryPtr &road_boundary, int group_id)
    {
      std::vector<Eigen::Vector3d> temp_points;
      if (!road_boundary)
      {
        // LOG_WARN("road_boundary null");
        return;
      }
      //
      for (auto point : road_boundary->points)
      {
        temp_points.push_back(point->pos);
      }
 
      temp_points=alg::cut_line_in_polygon(session->task_polygon_vec3d,temp_points);
      if(temp_points.size()<2)
      {
        LOG_WARN("road_boundary:[{}] points:{} <2",uint64_t(road_boundary.get()),temp_points.size());
        return ;
      }
      //
      OGRLineString lineString; // 经纬高
      for (auto p : temp_points)
      {
        Eigen::Vector3d wgs;
        session->data_processer->local2wgs(p, wgs);
        lineString.addPoint(wgs.x(), wgs.y());
      }
      if (lineString.IsEmpty())
      {
        return;
      }
      std::ostringstream prev_oss, next_oss, lane_id;
      auto &all_prev = road_boundary->get_prev();
      auto &all_next = road_boundary->get_next();
      for (auto iter = all_prev.begin(); iter != all_prev.end(); iter++)
      {
        if (iter != all_prev.begin())
        {
          prev_oss << "," << session->frame_id + "_" + std::to_string((*iter)->id);
        }
        else
        {
          prev_oss << session->frame_id + "_" + std::to_string((*iter)->id);
        }
      }
      for (auto iter = all_next.begin(); iter != all_next.end(); iter++)
      {
        if (iter != all_next.begin())
        {
          next_oss << "," << session->frame_id + "_" + std::to_string((*iter)->id);
        }
        else
        {
          next_oss << session->frame_id + "_" + std::to_string((*iter)->id);
        }
      }
      OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
      // feature->SetField("LANE_ID", lane_id.c_str());   //TODO 关联
      feature->SetField("ROAD_ID", (session->frame_id + "_" + std::to_string(group_id)).c_str());
      feature->SetField("ID", (session->frame_id + "_" + std::to_string(road_boundary->id)).c_str());
      feature->SetField("TYPE", road_boundary->type);
      feature->SetField("PRE_ID", prev_oss.str().c_str());
      feature->SetField("NEXT_ID", next_oss.str().c_str());
      feature->SetField("ALG_ID", std::stoi(session->frame_id));  
      feature->SetField("VERSION", session->version.c_str());
      feature->SetGeometry(&lineString);
      if (layer->CreateFeature(feature) != OGRERR_NONE)
      {
        LOG_WARN("lane center 创建要素失败");
      }
      OGRFeature::DestroyFeature(feature);
    };
    // 写入数据
    for (auto rs : session->road_segments)
    {
      for (auto lane_group : rs->lane_group_list)
      {
        if(!lane_group)
        {
          LOG_WARN("lane_group null");
          continue;
        }
        const  int group_id=lane_group->id;
        const auto &rbs = {lane_group->left_road_boundary, lane_group->right_road_boundary};
        for (auto rb : rbs)
        {
          write_one_road_boundary(rb,group_id);
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

    bool create = true;
    create &= create_field_defn(layer, "ID", OFTString, "road");
    create &= create_field_defn(layer, "KIND", OFTInteger, "road");
    create &= create_field_defn(layer, "IS_BIDIR", OFTInteger, "road");
    create &= create_field_defn(layer, "SCEN_TYPE", OFTInteger, "road");
    create &= create_field_defn(layer, "TYPE", OFTInteger, "road");
    create &= create_field_defn(layer, "SPEED", OFTInteger, "road");
    create &= create_field_defn(layer, "LANE_ID", OFTString, "road");
    create &= create_field_defn(layer, "LEFT_BID", OFTString, "road");
    create &= create_field_defn(layer, "RIGHT_BID", OFTString, "road");
    create &= create_field_defn(layer, "IN_ID", OFTString, "road");
    create &= create_field_defn(layer, "CROSS_ID", OFTString, "road");
    create &= create_field_defn(layer, "OUT_ID", OFTString, "road");
    create &= create_field_defn(layer, "NEXT_ID", OFTString, "road");
    create &= create_field_defn(layer, "PRE_ID", OFTString, "road");
    create &= create_field_defn(layer, "ALG_ID", OFTInteger, "road");
    create &= create_field_defn(layer, "VERSION", OFTString, "road");
    if (!create)
    {
      GDALClose(dataset);
      return fsdmap::FAIL;
    }
    auto write_one_lane_group = [session, layer](const fast_road_model::LaneGroupPtr &lane_group)
    {
      std::vector<Eigen::Vector3d> temp_points;
      if (!lane_group)
      {
        LOG_WARN("lane_group null");
        return;
      }
      auto &lane_list=lane_group->lane_list;
      if(lane_list.empty())
      {
        return;
      }
      //  TODO 
      OGRPolygon *polygon = new OGRPolygon();
      // 创建一个线性环（外环）
      OGRLinearRing *outerRing = new OGRLinearRing();
      auto &left_lane=lane_list.front();
      auto &right_lane=lane_list.back();
      // TODO cut line
    if(!left_lane||!right_lane)
    {
      return;
    }
    auto &left_line=left_lane->left;
    auto &right_line=right_lane->right;
    if(!left_line||!right_line)
    {
      return;
    }
    const int n=left_line->points.size();
    const int m=right_line->points.size();
    for(int i=0;i<n;i++)
    {
      temp_points.push_back(left_line->points[i]->pos);
    }
    for(int i=m-1;i>=0;i--)
    {
      temp_points.push_back(right_line->points[i]->pos);
    }
    auto front=temp_points.front();
    temp_points.push_back(front);
// 
if(temp_points.size()<4)
{
  LOG_WARN("road points:{} <2",temp_points.size());
  return ;
}
// 

      // std::vector<Eigen::Vector3d> obj_points;
      std::vector<int> indexs;
      if(alg::points_in_polygon(session->task_polygon_vec3d, temp_points, indexs) == false){
        delete polygon;
        delete outerRing;
        return ;
      }

      for (auto p : temp_points)
      {
        Eigen::Vector3d wgs;
        session->data_processer->local2wgs(p, wgs);
        outerRing->addPoint(wgs.x(), wgs.y());
      }



      if (outerRing->IsEmpty())
      {
        return;
      }
           // 确保线性环闭合
      outerRing->closeRings();
              // 将线性环添加到多边形中
      polygon->addRing(outerRing);
      std::ostringstream lane_id_oss, left_bid_oss, right_bid_oss,prev_oss, next_oss, lane_id;
      auto &all_prev = lane_group->get_prev();
      auto &all_next = lane_group->get_next();

      for(auto iter=lane_list.begin();iter!=lane_list.end();iter++)
      {
        if (iter != lane_list.begin())
        {
          lane_id_oss << "," << session->frame_id + "_" + std::to_string((*iter)->id);
        }
        else
        {
          lane_id_oss << session->frame_id + "_" + std::to_string((*iter)->id);
        }
      }

      for (auto iter = all_prev.begin(); iter != all_prev.end(); iter++)
      {
        if (iter != all_prev.begin())
        {
          prev_oss << "," << session->frame_id + "_" + std::to_string((*iter)->id);
        }
        else
        {
          prev_oss << session->frame_id + "_" + std::to_string((*iter)->id);
        }
      }
      for (auto iter = all_next.begin(); iter != all_next.end(); iter++)
      {
        if (iter != all_next.begin())
        {
          next_oss << "," << session->frame_id + "_" + std::to_string((*iter)->id);
        }
        else
        {
          next_oss << session->frame_id + "_" + std::to_string((*iter)->id);
        }
      }
      // 
      auto &left_boundary=lane_group->left_road_boundary;
      auto &right_boundary=lane_group->right_road_boundary;
    if(left_boundary)
    {
      left_bid_oss<<session->frame_id + "_" + std::to_string(left_boundary->id);
      
    }
    if(right_boundary)
    {
      right_bid_oss<<session->frame_id + "_" + std::to_string(right_boundary->id);
    }
      OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
      feature->SetField("ID", (session->frame_id + "_" + std::to_string(lane_group->id)).c_str());
      feature->SetField("LANE_ID", lane_id_oss.str().c_str());
      feature->SetField("LEFT_BID", left_bid_oss.str().c_str());   // 关联左侧道路边界
      feature->SetField("RIGHT_BID", right_bid_oss.str().c_str()); // 关联右侧道路边界
      feature->SetField("PRE_ID", prev_oss.str().c_str());  // 前序道路
      feature->SetField("NEXT_ID", next_oss.str().c_str()); // 后续道路
      feature->SetField("VERSION", session->version.c_str());
      feature->SetGeometry(polygon);
      if (layer->CreateFeature(feature) != OGRERR_NONE)
      {
        LOG_WARN("lane center 创建要素失败");
      }
      OGRFeature::DestroyFeature(feature);
    };
    for (auto rs : session->road_segments)
    {
      for (auto lane_group : rs->lane_group_list)
      {
        write_one_lane_group(lane_group);
      }
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

    bool create = true;
    create &= create_field_defn(layer, "ID", OFTString, "lane");
    create &= create_field_defn(layer, "LANE_TYPE", OFTInteger, "lane");
    create &= create_field_defn(layer, "TURN_TYPE", OFTInteger, "lane");
    create &= create_field_defn(layer, "ROAD_ID", OFTString, "lane");
    create &= create_field_defn(layer, "LEFT_LN", OFTString, "lane");
    create &= create_field_defn(layer, "RIGHT_LN", OFTString, "lane");
    create &= create_field_defn(layer, "LEFT_R_LN", OFTString, "lane");
    create &= create_field_defn(layer, "RIGHT_R_LN", OFTString, "lane");
    create &= create_field_defn(layer, "LEFT_BID", OFTString, "lane");
    create &= create_field_defn(layer, "RIGHT_BID", OFTString, "lane");
    create &= create_field_defn(layer, "INTER_ID", OFTString, "lane");
    create &= create_field_defn(layer, "PRE_ID", OFTString, "lane");
    create &= create_field_defn(layer, "NEXT_ID", OFTString, "lane");
    create &= create_field_defn(layer, "ALG_ID", OFTInteger, "lane");
    create &= create_field_defn(layer, "VERSION", OFTString, "lane");
    if (!create)
    {
      GDALClose(dataset);
      return fsdmap::FAIL;
    }

    auto write_one_lane = [session, layer](const fast_road_model::LaneGroupPtr &lane_group, int index)
    {
      OGRPolygon *polygon = new OGRPolygon();
      // 创建一个线性环（外环）
      OGRLinearRing *outerRing = new OGRLinearRing();
      int group_id = lane_group->id;
      auto lane = lane_group->lane_list[index];
      auto &left_line = lane->left;
      auto &right_line = lane->right;
      if (!left_line || !right_line)
      {
        return;
      }
      std::vector<Eigen::Vector3d> temp_points;
      const int n = left_line->points.size();
      const int m = right_line->points.size();
      for (int i = 0; i < n; i++)
      {
        temp_points.push_back(left_line->points[i]->pos);
      }
      for (int i = m - 1; i >= 0; i--)
      {
        temp_points.push_back(right_line->points[i]->pos);
      }
      auto front = temp_points.front();
      temp_points.push_back(front);
      //
      if(temp_points.size()<4)
      {
        LOG_WARN("lane points:{} <2",temp_points.size());
        return ;
      }
      // 

      
      std::vector<int> indexs;
      if(alg::points_in_polygon(session->task_polygon_vec3d, temp_points, indexs) == false){
        delete polygon;
        delete outerRing;
        return ;
      }


      for (auto p : temp_points)
      {
        Eigen::Vector3d wgs;
        session->data_processer->local2wgs(p, wgs);
        outerRing->addPoint(wgs.x(), wgs.y());
      }
      
    

      if (outerRing->IsEmpty())
      {
        return;
      }
      // 确保线性环闭合
      outerRing->closeRings();
      // 将线性环添加到多边形中
      polygon->addRing(outerRing);

      std::ostringstream prev_oss, next_oss, lane_id, turn_type_oss;
      auto &all_prev = lane->get_prev();
      auto &all_next = lane->get_next();
      for (auto iter = all_prev.begin(); iter != all_prev.end(); iter++)
      {
        if (iter != all_prev.begin())
        {
          prev_oss << "," << session->frame_id + "_" + std::to_string((*iter)->id);
        }
        else
        {
          prev_oss << session->frame_id + "_" + std::to_string((*iter)->id);
        }
      }
      for (auto iter = all_next.begin(); iter != all_next.end(); iter++)
      {
        if (iter != all_next.begin())
        {
          next_oss << "," << session->frame_id + "_" + std::to_string((*iter)->id);
        }
        else
        {
          next_oss << session->frame_id + "_" + std::to_string((*iter)->id);
        }
      }

      OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
      // feature->SetField("LANE_ID", lane_id.c_str());   //TODO 关联
      feature->SetField("ROAD_ID", (session->frame_id + "_" + std::to_string(group_id)).c_str());
      feature->SetField("ID", (session->frame_id + "_" + std::to_string(lane->id)).c_str());
      feature->SetField("TURN_TYPE", turn_type_oss.str().c_str()); // 车道转向类型
      feature->SetField("PRE_ID", prev_oss.str().c_str());
      feature->SetField("NEXT_ID", next_oss.str().c_str());
      // TODO
      int group_size=lane_group->lane_list.size();
      int left_lane_index=index-1;
      int right_lane_index=index+1;
      if (left_lane_index >= 0)
      {
        std::ostringstream left_oss;
        for (int i = 0; i <= left_lane_index; i++)
        {
          auto cur_lane = lane_group->lane_list[i];
          int lane_id = cur_lane->id;
          if (i == 0)
          {
            left_oss << session->frame_id + "_" + std::to_string(lane_id);
          }
          else
          {
            left_oss << "," << session->frame_id + "_" + std::to_string(lane_id);
          }
        }
        feature->SetField("LEFT_LN", left_oss.str().c_str());
      }
      if (right_lane_index >= 0)
      {
        std::ostringstream right_oss;
        for (int i = right_lane_index; i < group_size; i++)
        {
          auto cur_lane = lane_group->lane_list[i];
          int lane_id = cur_lane->id;
          if (i == right_lane_index)
          {
            right_oss << session->frame_id + "_" + std::to_string(lane_id);
          }
          else
          {
            right_oss << "," << session->frame_id + "_" + std::to_string(lane_id);
          }
        }
        feature->SetField("RIGHT_LN", right_oss.str().c_str());
      }

      if(lane->left)
      {
        
        feature->SetField("LEFT_BID", (session->frame_id + "_" + std::to_string(lane->left->id)).c_str());   // 左侧车道边界线ID
      }
      if(lane->right)
      {
        feature->SetField("RIGHT_BID", (session->frame_id + "_" + std::to_string(lane->right->id)).c_str()); // 右侧车道边界线ID
      }
      feature->SetField("ALG_ID", std::stoi(session->frame_id));  
      feature->SetField("VERSION", session->version.c_str());
      feature->SetGeometry(polygon);
      if (layer->CreateFeature(feature) != OGRERR_NONE)
      {
        LOG_WARN("lane center 创建要素失败");
      }
      OGRFeature::DestroyFeature(feature);
    };
    auto write_one_lane_group = [session, write_one_lane](const fast_road_model::LaneGroupPtr &lane_group)
    {
      std::vector<Eigen::Vector3d> temp_points;
      if (!lane_group)
      {
        LOG_WARN("lane_group null");
        return;
      }
      const int n = lane_group->lane_list.size();
      for (int i = 0; i < n; i++)
      {
        write_one_lane(lane_group, i);
      }
    };
    // 写入数据
    for (auto rs : session->road_segments)
    {
      for (auto lane_group : rs->lane_group_list)
      {
        if (!lane_group)
        {
          LOG_WARN("lane_group null");
          continue;
        }
        write_one_lane_group(lane_group);
      }
    }
    // 关闭数据集
    GDALClose(dataset);
    return fsdmap::SUCC;
  }

  //不维护
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
 //不维护
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
    OGRFieldDefn frameidFieldDefn("ALG_ID", OFTInteger);
    // copyedFieldDefn.SetWidth(256); // 设置一个足够大的宽度来容纳整数列表的字符串表示
    if (layer->CreateField(&frameidFieldDefn) != OGRERR_NONE)
    {
      std::cerr << "road centers  创建字段 ALG_ID 失败" << std::endl;
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
      // feature->SetField("ALG_ID", );  // 暂时没有
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
    
    std::vector<std::shared_ptr<OGRFieldDefn>> filed_buff;
    auto create_field_defn = [&filed_buff](OGRLayer *layer, const std::string &key, OGRFieldType type, const std::string &element) -> bool
    {
      // 添加 ID 字段
      std::shared_ptr<OGRFieldDefn> field_defn = std::make_shared<OGRFieldDefn>(key.c_str(), type);
      filed_buff.push_back(field_defn);
      if (layer->CreateField(field_defn.get()) != OGRERR_NONE)
      {
        LOG_WARN("create:{} key:{} type:{} failed ", element, key, (int)type)
        return false;
      }
      return true;
    };
    bool create = true;
    create &= create_field_defn(layer, "ID", OFTString, "stop_line");
    create &= create_field_defn(layer, "TYPE", OFTInteger, "stop_line");
    create &= create_field_defn(layer, "COLOR", OFTInteger, "stop_line");
    create &= create_field_defn(layer, "SHAPE_TYPE", OFTInteger, "stop_line");
    create &= create_field_defn(layer, "LANE_ID", OFTString, "stop_line");
    create &= create_field_defn(layer, "ALG_ID", OFTInteger, "stop_line");
    create &= create_field_defn(layer, "VERSION", OFTString, "stop_line");
    if (!create)
    {
      GDALClose(dataset);
      return fsdmap::FAIL;
    }


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

        // std::ostringstream lane_id_oss;
        // for(int i = 0; i< object->obj_bind_lanes.size(); i++)
        // {
        //   if(i >0){
        //     lane_id_oss << ",";
        //   }
        //   lane_id_oss << object->obj_bind_lanes[i]->road_lane_id;
        // }

        OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
        feature->SetField("ID", (session->frame_id + "_" + std::to_string(object->obj_id)).c_str());
        feature->SetField("TYPE", std::stoi(object->type));
        // feature->SetField("LANE_ID", lane_id_oss.str().c_str());
        // feature->SetField("COLOR", object->color);
        // feature->SetField("SHAPE", object->shape);
        // feature->SetField("LENGTH", object->length);
        feature->SetField("ALG_ID", std::stoi(session->frame_id));  
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
    

    bool create = true;
    create &= create_field_defn(layer, "ID", OFTString, "cross_walk");
    create &= create_field_defn(layer, "LANE_ID", OFTString, "cross_walk");
    create &= create_field_defn(layer, "JUNC_ID", OFTString, "cross_walk");
    create &= create_field_defn(layer, "ALG_ID", OFTInteger, "cross_walk");
    create &= create_field_defn(layer, "VERSION", OFTString, "cross_walk");

    if (!create)
    {
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
        
        //车道线绑定Id 先注释
        // std::ostringstream lane_id_oss,inter_id_oss;
        // for(int i = 0; i< object->obj_bind_lanes.size(); i++)
        // {
        //   if(i >0){
        //     lane_id_oss << ",";
        //   }
        //   lane_id_oss << object->obj_bind_lanes[i]->road_lane_id;
        // }

        // for(int i =0; i<object->obj_bind_intersections.size(); i++)
        // {
        //   if(i >0){
        //     inter_id_oss << ",";
        //   }
        //   inter_id_oss << object->obj_bind_intersections[i]->id;
        // }

        // 确保线性环闭合
        ring->closeRings();

        // 将线性环添加到多边形中
        polygon.addRing(ring);

        // 创建要素并设置字段值
        OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
        feature->SetField("ID", (session->frame_id + "_" + std::to_string(object->obj_id)).c_str());
        // feature->SetField("LANE_ID", lane_id_oss.str().c_str());  //
        // feature->SetField("JUNC_ID", inter_id_oss.str().c_str());
        feature->SetField("ALG_ID", std::stoi(session->frame_id));
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
    

    bool create = true;
    create &= create_field_defn(layer, "ID", OFTString, "arrow");
    create &= create_field_defn(layer, "TYPE", OFTString, "arrow");
    create &= create_field_defn(layer, "LANE_ID", OFTString, "arrow");
    create &= create_field_defn(layer, "ALG_ID", OFTInteger, "arrow");
    create &= create_field_defn(layer, "VERSION", OFTString, "arrow");


    if (!create)
    {
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
      
        //车道线绑定id
        // std::ostringstream lane_id_oss;
        // for(int i = 0; i< object->obj_bind_lanes.size(); i++)
        // {
        //   if(i >0){
        //     lane_id_oss << ",";
        //   }
        //   lane_id_oss << object->obj_bind_lanes[i]->road_lane_id;
        // }

        
        // 确保线性环闭合
        ring->closeRings();
        // 将线性环添加到多边形中
        polygon.addRing(ring);

        OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
        feature->SetField("ID", (session->frame_id + "_" + std::to_string(object->obj_id)).c_str());
        feature->SetField("TYPE", object->type.c_str());
        // feature->SetField("LANE_ID", lane_id_oss.str().c_str());
        feature->SetField("ALG_ID", std::stoi(session->frame_id));  
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
    
    bool create = true;
    create &= create_field_defn(layer, "ID", OFTString, "junction");
    create &= create_field_defn(layer, "TYPE", OFTInteger, "junction");
    create &= create_field_defn(layer, "IS_EFFECT", OFTInteger, "junction");
    create &= create_field_defn(layer, "LANE_ID", OFTString, "junction");
    create &= create_field_defn(layer, "IN_ROAD", OFTString, "junction");
    create &= create_field_defn(layer, "OUT_ROAD", OFTString, "junction");
    create &= create_field_defn(layer, "IN_LANE", OFTString, "junction");
    create &= create_field_defn(layer, "OUT_LANE", OFTString, "junction");
    create &= create_field_defn(layer, "ALG_ID", OFTInteger, "junction");
    create &= create_field_defn(layer, "VERSION", OFTString, "junction");
    

    if (!create)
    {
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

      // std::ostringstream lane_id_oss, in_road_oss, out_road_oss, in_lane_oss, out_lane_oss;
      // //车道
      // for(int i = 0; i< inter->all_lanes.size(); i++)
      // {
      //   if(i >0){
      //     lane_id_oss << ",";
      //   }
      //   lane_id_oss << inter->all_lanes[i]->road_lane_id;
      // }

      // for(int i = 0; i< inter->in_lanes.size(); i++)
      // {
      //   if(i >0){
      //     in_lane_oss << ",";
      //   }
      //   in_lane_oss << inter->all_lanes[i]->road_lane_id;
      // }

      // for(int i = 0; i< inter->out_lanes.size(); i++)
      // {
      //   if(i >0){
      //     out_lane_oss << ",";
      //   }
      //   out_lane_oss << inter->all_lanes[i]->road_lane_id;
      // }
      // // //进入退出道路
      // std::vector<RoadLaneGroupInfo*> in_loads_vector(inter->in_loads.begin(), inter->in_loads.end());
      // std::vector<RoadLaneGroupInfo*> out_loads_vector(inter->out_loads.begin(), inter->out_loads.end());
      // for(int i = 0; i< in_loads_vector.size(); i++)
      // {
      //   if(i >0){
      //     in_road_oss << ",";
      //   }
      //   in_road_oss << in_loads_vector[i]->group_index;
      // }

      // for(int i = 0; i< out_loads_vector.size(); i++)
      // {
      //   if(i >0){
      //     out_road_oss << ",";
      //   }
      //   out_road_oss << out_loads_vector[i]->group_index;
      // }

      // 确保线性环闭合
      ring->closeRings();
      polygon.addRing(ring);

      OGRFeature *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
      feature->SetField("ID", (session->frame_id + "_" + std::to_string(inter->id)).c_str());
      // feature->SetField("LANE_ID", lane_id_oss.str().c_str());  
      // feature->SetField("IN_LANE", in_lane_oss.str().c_str());  
      // feature->SetField("OUT_LANE", out_lane_oss.str().c_str()); 
      // feature->SetField("IN_ROAD", in_road_oss.str().c_str());  
      // feature->SetField("OUT_ROAD", out_road_oss.str().c_str()); 
      feature->SetField("ALG_ID", std::stoi(session->frame_id));  
      feature->SetField("VERSION", session->version.c_str());
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
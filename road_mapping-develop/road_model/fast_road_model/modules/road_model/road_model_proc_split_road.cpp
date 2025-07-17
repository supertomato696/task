


#define PCL_NO_PRECOMPILE
#include "road_model_proc_split_road.h"
#include "utils/algorithm_util.h"
#include "core/road_segment.h"
DECLARE_double(display_scope_buff);

DEFINE_bool(split_road_debug_pos_enable, true, "split_road_debug_enable");
DEFINE_bool(split_road_save_data_enable, true, "split_road_save_data_enable");
DEFINE_bool(split_road_save_data_pcd_all_enable, true, "split_road_save_data_pcd_all_enable");

DEFINE_bool(split_road_split_road_by_link_enable, true, "split_road_split_road_by_link_enable");
DEFINE_bool(split_road_gen_by_inter_enable, false, "split_road_gen_by_inter_enable");

DEFINE_bool(split_road_load_pcd_enable, true, "split_road_load_pcd_enable");
DEFINE_double(split_road_extend_road_segment_scope, 20, "split_road_extend_road_segment_scope");
DEFINE_double(split_road_make_line_sample_gap, 2, "split_road_make_line_sample_gap");
DEFINE_double(split_road_intersection_match_radius, 35, "split_road_intersection_match_radius");
DEFINE_double(split_road_max_inter_gap, 10, "split_road_max_inter_gap");
DEFINE_double(split_road_mark_inter_poss_radius, 40, "split_road_mark_inter_poss_radius");

namespace fsdmap {
namespace road_model {

fsdmap::process_frame::PROC_STATUS RoadModelProcSplitRoad::proc(
        RoadModelSessionData* session) {
     // 
     CHECK_FATAL_PROC(make_road_segment(session), "make_road_segment");
    //  
    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    // 
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

int RoadModelProcSplitRoad::make_road_segment(RoadModelSessionData* session)
{
    auto gen_road_segment=[]( std::vector<KeyPose*> &poss_buff,
        std::vector<std::shared_ptr<fast_road_model::RoadSegment>> &temp,
        RoadModelSessionData* session)
    {
        auto new_segment=std::make_shared<fast_road_model::RoadSegment>();
        // new_segment->id=
        std::swap(new_segment->pos_sample_list,poss_buff);
        session->road_segments.push_back(new_segment.get());
        session->road_segments_ptr.push_back(new_segment);
        temp.push_back(new_segment);
    };
    for (auto link : session->link_sample_list)
    {
        std::vector<std::shared_ptr<fast_road_model::RoadSegment>> temp;
        std::vector<KeyPose*> poss_buff;
        for (auto poss : link->list)
        {
            poss_buff.push_back(poss);
            if (poss->road_break&&poss_buff.size()>1) // road_break : 不一定是真實的斷電位置，目前找的是最近的key pos
            {
                // 
                gen_road_segment(poss_buff,temp,session);
                // 
                poss_buff.clear();
                //
                poss_buff.push_back(poss);
            }

        }
        if (poss_buff.size()>1)
        {
            gen_road_segment(poss_buff,temp,session);
        }
        // 
        if(!temp.empty())
        {  
            auto &prev=temp.front();
            const int n=temp.size();
            for(int i=1;i<n;i++)
            {   //check all right
                auto &cur=temp[i];
                cur->add_prev(prev.get());
                prev->add_next(cur.get());
                // LOG_INFO("rs[{}-->{}] ",uint64_t(prev.get()),uint64_t(cur.get()));
                prev=cur;
            }
        }
    }

    return fsdmap::SUCC;
}

int RoadModelProcSplitRoad::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_split_road_save_data_enable) {
        return fsdmap::SUCC;
    }
    int count=0;
    session->set_display_name("split_road");
    for(auto rs:session->road_segments)
    {
       srand48(count++);
       int label=drand48()*1e3;
        auto    log = session->add_debug_log(utils::DisplayInfo::POINT, "poss");
       for(auto poss:rs->pos_sample_list)
       {
        auto &ele=log->add(poss->pos);
        ele.label.label=label;
        ele.color={255,255,255};
       }
    }

    for(auto rs:session->road_segments)
    {
       for(auto poss:rs->pos_sample_list)
       {
        if (poss->road_break)
        {
            auto log = session->add_debug_log(utils::DisplayInfo::LINE, "line_{}",count++);
            log->color = {255, 255, 255};
            auto p1 = alg::get_hori_pos(poss->pos, poss->road_vertical_dir, +3.75*2);
            auto p2 = alg::get_hori_pos(poss->pos, poss->road_vertical_dir, -3.75*2);
            log->add(p1);
            log->add(p2);
        }
       }
    }
    // for (auto link : session->link_sample_list)
    // {
    //     for (auto poss : link->list)
    //     {
    //         if (poss->road_break)
    //         {
    //             auto log = session->add_debug_log(utils::DisplayInfo::LINE, "line_{}",count++);
    //             log->color = {255, 255, 255};
    //             auto p1 = alg::get_hori_pos(poss->pos, poss->road_vertical_dir, +3.75*2);
    //             auto p2 = alg::get_hori_pos(poss->pos, poss->road_vertical_dir, -3.75*2);
    //             log->add(p1);
    //             log->add(p2);
    //         }
    //     }
    // }
    session->save_debug_info("split_road");
    return fsdmap::SUCC;
}

}
}

// Licensed to the Apache Software Foundation (ASF) under one
// or more contributor license agreements.  See the NOTICE file
// distributed with this work for additional information
// regarding copyright ownership.  The ASF licenses this file
// to you under the Apache License, Version 2.0 (the
// "License"); you may not use this file except in compliance
// with the License.  You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing,
// software distributed under the License is distributed on an
// "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied.  See the License for the
// specific language governing permissions and limitations
// under the License.

#include <atomic>
#include <fstream>
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <thread>
#include <experimental/filesystem>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "json/json.h"
#include "brpc/channel.h"
#include <brpc/policy/gzip_compress.h>

#include "data_access_engine.h"
#include "manager/road_geometry_mgr.h"
#include "proxy/link_proxy.h"
#include "proxy/position_proxy.h"
#include "proxy/traffic_proxy.h"
#include "proxy/lane_proxy.h"
#include "proxy/tile_proxy.h"
#include "ndm.pb.h"
#include "lane/lane.pb.h"
#include "utility.h"

DEFINE_string(branch, "test_beijing_ring6", "upload branch");
DEFINE_string(down_branch, "test_link", "download branch");
DEFINE_string(file, "/mnt/d/code/path_shsz", "input file");
DEFINE_string(server, "172.21.207.124:8081", "IP Address of server");
DEFINE_int32(timeout_ms, 600000, "RPC timeout in milliseconds");
DEFINE_int32(max_retry, 10, "Max retries(not including the first RPC)");
DEFINE_int32(interval_ms, 1000, "Milliseconds between consecutive requests");
DEFINE_string(range, //"(120.211 30.236,122.086 31.658)",
                    "(116.0225 39.6501,116.9467 40.495)",
                    "Range for bind trajectory to link");

using namespace data_access_engine;
using namespace Json;

bool load_vids(double minx, double miny, double maxx, double maxy, std::unordered_set<std::string>& vids) {
    char buf[1024];
    snprintf(buf, sizeof(buf), "http://poseidon-cmdb-prod.biyadi.com/api/v1/prevideo/getvidbygps?sc_type=2&tl_latlng=%.13f,%.13f&br_latlng=%.13f,%.13f&page=", maxy, minx, miny, maxx);
    int page = 1;
    int total = 0;
    do {
        std::string url = buf;
        url += std::to_string(page);
        LOG(INFO) << "Load " << url;
        brpc::Channel channel;
        brpc::ChannelOptions options;
        options.connect_timeout_ms = 600 * 1000;
        options.timeout_ms = 600 * 1000;
        options.protocol = brpc::PROTOCOL_HTTP;  // or brpc::PROTOCOL_H2
        if (channel.Init(url.c_str(), &options) != 0) {
            LOG(ERROR) << "Fail to initialize channel to " << url;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        brpc::Controller cntl;
        cntl.http_request().uri() = url;
        channel.CallMethod(NULL, &cntl, NULL, NULL, NULL/*done*/);
            
        auto resp = cntl.response_attachment().to_string();
        JSONCPP_STRING err;
        Json::Value root;
        Json::CharReaderBuilder builder;
        const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
        if (!reader->parse(resp.c_str(), resp.c_str() + resp.length(), &root, &err))  {
            LOG(ERROR) << "Fail to parse response from " << url << " : " << err;
            continue;
        }
        page++;
        total = root["data"]["total_page"].asInt();
        auto ls = root["data"]["list"];
        LOG(INFO) << "Finish load " << url << " : " << page << " / " << total << " vids:" << ls.size();
        for (int i = 0; i < ls.size(); ++i) {
            auto it = ls[i];
            auto vid = it["gps_location"].asCString();
            vids.insert(vid);
        }
    } while (page < total);
    return true;
}

bool download_gps(const std::string& url, std::vector<Tuple<5, double>>& dps) {
    brpc::Channel channel;
    brpc::ChannelOptions options;
    options.connect_timeout_ms = 600 * 1000;
    options.timeout_ms = 600 * 1000;
    options.protocol = brpc::PROTOCOL_HTTP; // or brpc::PROTOCOL_H2
    if (channel.Init(url.c_str(), &options) != 0) {
        LOG(ERROR) << "Fail to initialize channel to " << url;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return false;
    }
    brpc::Controller cntl;
    cntl.http_request().uri() = url;
    channel.CallMethod(NULL, &cntl, NULL, NULL, NULL /*done*/);
    LOG(INFO) << "Load " << url;
    auto resp = cntl.response_attachment().to_string();
    JSONCPP_STRING err;
    Json::Value root;
    Json::CharReaderBuilder builder;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(resp.c_str(), resp.c_str() + resp.length(), &root, &err)) {
        LOG(ERROR) << "Fail to parse response from " << url << " : " << err;
        return false;
    }
    try {
    std::map<int64_t, Tuple<4, double>> time2pts;
    for (int i = 0; i < root.size(); ++i) {
        auto type = root[i]["signalName"];
        int ind = -1;
        if (type == "Location_lon") {
            ind = 0;
        }
        else if (type == "Location_lat") {
            ind = 1;
        }
        else if (type == "Location_dir") {
            ind = 3;
        }
        else if (type == "Location_alt") {
            ind = 2;
        }
        else {
            continue;
        }
        auto& dps = root[i]["dps"];
        for (auto it = dps.begin(); it != dps.end(); ++it) {
            int64_t ts = atoll(it.key().asCString());
            time2pts[ts][ind] = it->asDouble() + 1e-9;
        }
    }
    if (time2pts.size() <= 10) {
        return false;
    }
    dps.reserve(time2pts.size());
    for (auto& it : time2pts) {
        Tuple<5, double> t;
        t[0] = it.first;
        t[1] = it.second[0];
        t[2] = it.second[1];
        t[3] = it.second[2];
        t[4] = it.second[3];
        dps.push_back(t);
    }
    } catch(...) {
        return false;
    }
    return true;
};

void check_roads(double minx, double miny, double maxx, double maxy, RoadGeometryManager* mgr) {
    std::set<int> tidset;
    for (double y = miny; y < maxy + 180./(1 << 16); y += 180./(1 << 16)) {
        for (double x = minx; x < maxx + 180./(1 << 16); x += 180./(1 << 16)) {
            int tid = mgr->WGS84_to_tileID(Vector3D(x, y, 0));
            tidset.insert(tid);
        }
    }
    if (tidset.empty()) {
        LOG(ERROR) << "check_road() with empty range: " << minx << " " << miny << ", "
                    << maxx << " " << maxy;
        return;
    }
    std::vector<int> tids(tidset.begin(), tidset.end());
    mgr->load_tiles_by_id(tids, nullptr, 8);
    Json::Value root;
    ID2TileMap tiles;
    mgr->get_road_tiles(tiles);
    for (auto& tit : tiles) {
        auto tile = tit.second;
        for (auto& lg : tile->lane_groups()) {
            if (!lg->is_editable()) {
                continue;
            }
            std::vector<std::shared_ptr<LaneBoundaryProxy>> lls;
            lls.reserve(lg->lanes().size() * 2);
            for (auto& lid : *lg->mutable_lanes()) {
                auto lane = lid.proxy();
                if (!lane) {
                    continue;
                }
                for (auto& ls : *lane->mutable_lanes()) {
                    if (!ls->left_boundary() || !ls->left_boundary()->bound_id().proxy()) {
                        LOG(ERROR) << "No left boundary for lane " << lane->id()->to_string();
                    }
                    else {
                        auto lb = ls->mutable_left_boundary()->mutable_bound_id()->proxy();
                        if (!lls.empty() && lls.back() == lb) {
                            continue;
                        }
                        else {
                            lls.push_back(lb);
                        }
                    }
                    if (!ls->right_boundary() || !ls->right_boundary()->bound_id().proxy()) {
                        LOG(ERROR) << "No right boundary for lane " << lane->id()->to_string();
                    }
                    else {
                        auto lb = ls->mutable_right_boundary()->mutable_bound_id()->proxy();
                        if (!lls.empty() && lls.back() == lb) {
                            continue;
                        }
                        else {
                            lls.push_back(lb);
                        }
                    }
                    Optional<float> maxs;
                    Optional<float> mins;
                    std::shared_ptr<FixedSpeedLimitProxy> max_proxy;
                    for (auto& sp : *ls->mutable_speed_limits()) {
                        if (sp->max_speed()) {
                            maxs = sp->max_speed();
                            max_proxy = sp;
                        }
                        if (sp->min_speed()) {
                            mins = sp->min_speed();
                        }
                    }
                    if (maxs && mins && (maxs.get() < mins.get() || (maxs.get() == mins.get() && mins.get() > 60))) {
                        if (mins.get() >= 100) {
                            max_proxy->set_max_speed(120);
                        }
                        else if (mins.get() >= 80) {
                            max_proxy->set_max_speed(100);
                        }
                        else {
                            max_proxy->set_max_speed(80);
                        }
                        Json::Value ep;
                        std::stringstream ss;
                        ss << "Speed error: min " << mins.get() << " max " << maxs.get() << ", change max to " << max_proxy->max_speed().get();
                        ep["description"] = ss.str();
                        LOG(INFO) << ss.str() << " @ " << lane->id()->to_string();
		                ep["errortileid"] = lane->id()->tileid();
		                ep["id"] = lane->id()->id();
		                ep["memo"] = "";
		                ep["opState"] = 0;
		                ep["qcState"] = 0;
		                ep["tileid"] = lane->id()->tileid();
		                ep["type"] = 4;
		                ep["version"] = lane->id()->version();
                        auto pos = lane->wgs84_pos();
		                ep["x"] = pos.X();
		                ep["y"] = pos.Y();
		                ep["z"] = pos.Z();
                        root.append(ep);
                    }
                    else if (maxs) {
                        max_proxy->set_max_speed(maxs.get() + 5);
                    }
                }
            }

            if (lls.size() > 2) {
                size_t left = lls.size();
                size_t right = 0;
                for (size_t i = 0; i < lls.size(); i++) {
                    if (lls[i]->marking() && lls[i]->marking().get() == RoadPB::LaneBoundary::LineMarking_SolidLine) {
                        left = i + 1;
                        break;
                    }
                }
                for (size_t i = lls.size() - 1; i > 0; i--) {
                    if (lls[i]->marking() && lls[i]->marking().get() == RoadPB::LaneBoundary::LineMarking_SolidLine) {
                        right = i;
                        break;                        
                    }
                }
                for (size_t i = left; i < right; i++) {
                    if (lls[i]->marking() && lls[i]->marking().get() == RoadPB::LaneBoundary::LineMarking_SolidLine) {
                        lls[i]->set_marking(RoadPB::LaneBoundary::LineMarking_DashedLine);
                        Json::Value ep;
                        std::stringstream ss;
                        ss << "Lane type change to dashed @ " << lls[i]->id()->to_string();
                        ep["description"] = ss.str();
                        LOG(INFO) << ss.str() << " @ " << lg->id()->to_string();
		                ep["errortileid"] = lls[i]->id()->tileid();
		                ep["id"] = lls[i]->id()->id();
		                ep["memo"] = "";
		                ep["opState"] = 0;
		                ep["qcState"] = 0;
		                ep["tileid"] = lls[i]->id()->tileid();
		                ep["type"] = 4;
		                ep["version"] = lls[i]->id()->version();
                        auto pos = lls[i]->wgs84_pos();
		                ep["x"] = pos.X();
		                ep["y"] = pos.Y();
		                ep["z"] = pos.Z();
                        root.append(ep);
                    }
                }
            }
        }
    }
    std::string path("mark_");
    path.append(FLAGS_branch);
    path.append(".json");
    std::ofstream ofs(path, std::ios::out);
    if (!ofs.good()) {
        return;
    }
    ofs << root;
    ofs.close();
    std::cout << root;
    TileInfoList ts;
    mgr->get_changed_tile_data(ts);
    mgr->upload_tiles("speed_limit_fix", ts, -4, 4);
}

int bind_link(double minx, double miny, double maxx, double maxy, RoadGeometryManager* mgr) {
    std::set<int> tidset;
    for (double y = miny; y < maxy + 180./(1 << 16); y += 180./(1 << 16)) {
        for (double x = minx; x < maxx + 180./(1 << 16); x += 180./(1 << 16)) {
            int tid = mgr->WGS84_to_tileID(Vector3D(x, y, 0));
            tidset.insert(tid);
        }
    }
    if (tidset.empty()) {
        LOG(ERROR) << "bind_link() with empty range: " << minx << " " << miny << ", "
                    << maxx << " " << maxy;
        return 0;
    }
    std::vector<int> tids(tidset.begin(), tidset.end());
    mgr->load_tiles_by_id(tids, nullptr, 8);
    
    std::unordered_set<std::string> vids;
    if (!load_vids(minx, miny, maxx, maxy, vids)) {
        return 0;
    }
    data_access_engine::TileInfoPtr tmp_tile(data_access_engine::FeatureProxyBase::create_proxy<data_access_engine::TileInfoProxy>()); 
    tmp_tile->relink_parent();
    std::vector<std::string> vs(vids.begin(), vids.end());
    std::vector<std::thread> ths;
    std::atomic<int> ind(0);
    std::mutex mut;
    for (int ti = 0; ti < 8; ti++) {
        ths.push_back(std::thread([&vs, &ind, mgr, &mut]() {
            while (true) {
                int index = ind.fetch_add(1);
                if (index >= vs.size()) {
                    break;
                }
                std::string url = "http://poseidon-cmdb-prod.biyadi.com/api/v1/m01/data/show?m=get_vid_detail&vid=";
                url += vs[index];
                url += "&t=4";
                brpc::Channel channel;
                brpc::ChannelOptions options;
                options.connect_timeout_ms = 600 * 1000;
                options.timeout_ms = 600 * 1000;
                options.protocol = brpc::PROTOCOL_HTTP;  // or brpc::PROTOCOL_H2
                if (channel.Init(url.c_str(), &options) != 0) {
                    LOG(ERROR) << "Fail to initialize channel to " << url;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }
                brpc::Controller cntl;
                cntl.http_request().uri() = url;
                channel.CallMethod(NULL, &cntl, NULL, NULL, NULL/*done*/);
                    
                auto resp = cntl.response_attachment().to_string();
                JSONCPP_STRING err;
                Json::Value root;
                Json::CharReaderBuilder builder;
                const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
                if (!reader->parse(resp.c_str(), resp.c_str() + resp.length(), &root, &err))  {
                    LOG(ERROR) << "Fail to parse response from " << url << " : " << err;
                    continue;
                }
                if (root["code"].asInt() != 200) {
                    continue;
                }
                auto ls = root["data"];
                for (int i = 0; i < ls.size(); i++) {
                    auto& item = ls[i];
                    if (item["type"].asInt() == 3) {
                        auto down_url = item["url"].asCString();
                        std::vector<Tuple<5, double>> dps;
                        if (download_gps(down_url, dps)) {
                            std::shared_ptr<LinkProxy> link(FeatureProxyBase::create_proxy<LinkProxy>());
                            auto name = link->mutable_names()->add();
                            name->set_name(vs[index]);
                            for (auto& dp : dps) {
                                auto pt = link->mutable_geom()->mutable_pts()->add();
                                nuts::dpoint_t pp = { dp[1], dp[2] };
                                nuts::dpoint_t op;
                                nuts::wgs2gcj(pp, &op);
                                pt->set_x(op.x);
                                pt->set_y(op.y);
                                pt->set_z(dp[3]);                        
                            }
                            Vector3D pos = *link->geom()->pts()[0];
                            TileInfoPtr tile;
                            std::lock_guard<std::mutex> lck(mut);
                            mgr->make_new_id(pos, link, tile, true);
                            tile->links().push_back(link);
                        }
                    }
                }
            }
        }));
    }
    for (int ti = 0; ti < 8; ti++) {
        ths[ti].join();
    }
    return ind;
};

int load_pb(const std::string& path, RoadGeometryManager* mgr) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.good()) {
        std::cout << "Fail to open pb file " << path;
        return 0;
    }
    int cnt = 0;
    ndm_proto::MapEnvMsg env_msg;
    env_msg.ParseFromIstream(&ifs);
    LOG(INFO) << "Read pb file: " << path;
    std::unordered_map<std::string, std::shared_ptr<PositionObjectProxy> > id2obj(10240);
    std::unordered_map<std::string, std::shared_ptr<TrafficInfoProxy> > id2info(10240);
    TileInfoPtr tmptile(FeatureProxyBase::create_proxy<TileInfoProxy>());
    tmptile->relink_parent();
    int64_t timestamp = mgr->current_version();
    for (auto& lm : env_msg.physical_layer().lanemarkings()) {        
        auto obj = tmptile->mutable_position_objects()->add();
        auto border = obj->mutable_border();
        for (auto& pt : lm.border().points()) {
            auto pts = border->mutable_pts()->add();
            pts->set_x(pt.x());
            pts->set_y(pt.y());
            pts->set_z(pt.z());            
        }
        if (border->pts().empty()) {
            continue;
        }
        //LOG(INFO) << "Construct lanemark " << lm.id() << " with " << border->pts().size() << " pts";
        Vector3D pos = *(border->pts()[0]);
        obj->set_type(RoadPB::PositionObject::LANE_MARKING);
        obj->set_content(lm.str().str() + std::to_string(lm.type().type()));
        if (id2obj.count(lm.id()) > 0) {
            LOG(ERROR) << "Duplicated id found " << lm.id();
        }
        id2obj[lm.id()] = obj;
        TileInfoPtr tile;
        if (mgr->make_new_id(pos, obj, tile, true)) {
            if (!obj->is_valid()) {
                LOG(ERROR) << "Construct lanemark " << lm.id() << " object invalid " << obj->id()->to_string();
            }
            cnt++;
            tile->position_objects().push_back(obj);
            tile->position_object_refs().push_back(obj->id());
            auto info = tile->traffic_infos().add();
            info->mutable_objs()->push_back(obj->id());
            info->set_type(RoadPB::TrafficInfo::LANE_MARK);
            auto mark = info->mutable_marking();
            mark->set_type(lm.type().type());
            if (lm.has_str()) {
                mark->set_content(lm.str().str());
            }
            if (lm.has_number()) {
                mark->set_value(lm.number().value());
            }
            info->mutable_id()->set_value(obj->id()->tileid(), TrafficInfoProxy::ELEM_ID_TYPE, 0, timestamp);
            info->make_id_index();
            if (!info->is_valid()) {
                LOG(ERROR) << "Construct lanemark " << lm.id() << " info invalid " << info->id()->to_string();
            }
            cnt++;
            tile->traffic_info_refs().push_back(info->id());
            id2info[lm.id()] = info;
        }
    }
    for (auto& pole : env_msg.physical_layer().poles()) {
        auto obj = tmptile->position_objects().add();
        auto body = obj->mutable_pole();
        for (auto& pt : pole.body().points()) {
            auto pts = body->mutable_pts()->add();
            pts->set_x(pt.x());
            pts->set_y(pt.y());
            pts->set_z(pt.z());            
        }        
        if (body->pts().empty()) {
            continue;
        }
        //LOG(INFO) << "Construct poles " << pole.id() << " with " << body->points().size() << " pts";
        //if (pole.body().has_radius()) {
        //    body->set_radius(pole.body().radius().value());
        //}
        Vector3D pos = *(body->pts()[0]);
        obj->set_type(RoadPB::PositionObject::POLE);
        obj->set_content(std::to_string(pole.type().type()));
        if (id2obj.count(pole.id()) > 0) {
            LOG(ERROR) << "Duplicated id found " << pole.id();
        }
        id2obj[pole.id()] = obj;
        TileInfoPtr tile;
        if (mgr->make_new_id(pos, obj, tile, true)) {
            if (!obj->is_valid()) {
                LOG(ERROR) << "Construct pole " << pole.id() << " object invalid " << obj->id()->to_string();
            }
            cnt++;
            tile->position_objects().push_back(obj);
            tile->mutable_position_object_refs()->push_back(obj->id());
        }
    }
    /*for (auto& board : env_msg.physical_layer().boards()) {
        auto obj = tmptile->position_objects().add();
        auto bd = obj->mutable_border();
        for (auto& pt : board.border().points()) {
            auto pts = bd->mutable_pts()->add();
            pts->set_x(pt.x());
            pts->set_y(pt.y());
            pts->set_z(pt.z());            
        }
        if (bd->pts().size()) {
            continue;
        }        
        //LOG(INFO) << "Construct board " << board.id() << " with " << bd->pts().size() << " pts";
        Vector3D pos = *(bd->pts()[0]);
        obj->set_type(RoadPB::PositionObject::BOARD);
        obj->set_content(board.str().str());
        id2obj[board.id()] = obj;
        TileInfoPtr tile;
        if (mgr->make_new_id(pos, obj, tile, true)) {
            if (!obj->is_valid()) {
                LOG(ERROR) << "Construct board " << board.id() << " object invalid " << obj->id()->to_string();
            }
            cnt++;
            tile->position_objects().push_back(obj);
            tile->mutable_position_object_refs()->push_back(obj->id());
        }
    }*/
    for (auto& crs : env_msg.physical_layer().crosswalks()) {
        auto obj = tmptile->position_objects().add();
        auto bd = obj->mutable_border();
        for (auto& pt : crs.border().points()) {
            auto pts = bd->mutable_pts()->add();
            pts->set_x(pt.x());
            pts->set_y(pt.y());
            pts->set_z(pt.z());            
        }
        if (bd->pts().size()) {
            continue;
        }        
        //LOG(INFO) << "Construct crosswalk " << crs.id() << " with " << bd->pts().size() << " pts";
        Vector3D pos = *(bd->pts()[0]);
        obj->set_type(RoadPB::PositionObject::CROSS_WALK);
        if (id2obj.count(crs.id()) > 0) {
            LOG(ERROR) << "Duplicated id found " << crs.id();
        }
        id2obj[crs.id()] = obj;
        TileInfoPtr tile;
        if (mgr->make_new_id(pos, obj, tile, true)) {
            if (!obj->is_valid()) {
                LOG(ERROR) << "Construct crosswalk " << crs.id() << " object invalid " << obj->id()->to_string();
            }
            cnt++;
            tile->position_objects().push_back(obj);
            tile->mutable_position_object_refs()->push_back(obj->id());
        }
    }
    for (auto& sl : env_msg.physical_layer().stoplines()) {
        auto obj = tmptile->position_objects().add();
        auto bd = obj->mutable_border();
        for (auto& pt : sl.border().points()) {
            auto pts = bd->mutable_pts()->add();
            pts->set_x(pt.x());
            pts->set_y(pt.y());
            pts->set_z(pt.z());            
        }
        if (bd->pts().size()) {
            continue;
        }        
        //LOG(INFO) << "Construct stopline " << sl.id() << " with " << bd->pts().size() << " pts";        
        Vector3D pos = *(bd->pts()[0]);
        obj->set_type(RoadPB::PositionObject::STOP_LINE);
        if (id2obj.count(sl.id()) > 0) {
            LOG(ERROR) << "Duplicated id found " << sl.id();
        }
        id2obj[sl.id()] = obj;
        TileInfoPtr tile;
        if (mgr->make_new_id(pos, obj, tile, true)) {
            if (!obj->is_valid()) {
                LOG(ERROR) << "Construct stopline " << sl.id() << " object invalid " << obj->id()->to_string();
            }
            cnt++;
            tile->position_objects().push_back(obj);
            tile->mutable_position_object_refs()->push_back(obj->id());
        }
    }
    /*for (auto& park : env_msg.physical_layer().parkingslots()) {
        auto obj = tmptile->position_objects().add();
        auto bd = obj->mutable_border();
        for (auto& pt : park.border().points()) {
            auto pts = bd->mutable_pts()->add();
            pts->set_x(pt.x());
            pts->set_y(pt.y());
            pts->set_z(pt.z());            
        }
        if (bd->pts().size()) {
            continue;
        }        
        //LOG(INFO) << "Construct parkingslot " << park.id() << " with " << bd->pts().size() << " pts";
        Vector3D pos = *(bd->pts()[0]);
        obj->set_type(RoadPB::PositionObject::PARKING_SLOT);
        obj->set_content(park.str().str());
        id2obj[park.id()] = obj;
        TileInfoPtr tile;
        if (mgr->make_new_id(pos, obj, tile, true)) {
            if (!obj->is_valid()) {
                LOG(ERROR) << "Construct parkingslot " << park.id() << " object invalid " << obj->id()->to_string();
            }
            cnt++;
            tile->position_objects().push_back(obj);
            tile->mutable_position_object_refs()->push_back(obj->id());
        }
    }*/
    for (auto& sign : env_msg.physical_layer().trafficsigns()) {
        auto obj = tmptile->position_objects().add();
        auto border = obj->mutable_border();
        for (auto& pt : sign.border().points()) {
            auto pts = border->mutable_pts()->add();
            pts->set_x(pt.x());
            pts->set_y(pt.y());
            pts->set_z(pt.z());            
        }
        if (border->pts().empty()) {
            continue;
        }
        //LOG(INFO) << "Construct trafficsign " << sign.id() << " with " << border->pts().size() << " pts";        
        Vector3D pos = *(border->pts()[0]);
        obj->set_type(RoadPB::PositionObject::SIGN);
        obj->set_content(sign.str().str() + std::to_string(sign.type().type()));
        if (id2obj.count(sign.id()) > 0) {
            LOG(ERROR) << "Duplicated id found " << sign.id();
        }
        id2obj[sign.id()] = obj;
        TileInfoPtr tile;
        if (mgr->make_new_id(pos, obj, tile, true)) {
            if (!obj->is_valid()) {
                LOG(ERROR) << "Construct trafficsign " << sign.id() << " object invalid " << obj->id()->to_string();
            }
            cnt++;
            tile->position_objects().push_back(obj);
            tile->mutable_position_object_refs()->push_back(obj->id());
            auto info = tile->traffic_infos().add();
            info->mutable_objs()->push_back(obj->id());
            info->set_type(RoadPB::TrafficInfo::SIGN);
            auto sn = info->mutable_sign();
            if (sign.has_type()) {
                sn->set_type(sign.type().type());
            }
            if (sign.has_shape()) {
                sn->set_shape(sign.shape());
            }
            if (sign.has_number()) {
                sn->set_value(sign.number().value());
            }
            if (sign.has_str()) {
                sn->set_content(sign.str().str());
            }
            if (sign.has_vms() && sign.vms() != ndm_proto::YesNoUnknown_Unknown) {
                sn->set_variable(sign.vms() == ndm_proto::YesNoUnknown_Yes);
            }
            for (auto& p : sign.panels()) {
                auto pn = sn->mutable_panels()->add();
                if (p.has_type()) {
                    pn->set_type(p.type());
                }
                if (p.has_condition_numeric()) {
                    pn->set_value(p.condition_numeric().value());
                }
                if (p.has_condition_vehicletype()) {
                    pn->set_vehicle_type_mask(p.condition_vehicletype().vehicletypemask());
                }
                if (p.has_condition_load()) {
                    pn->set_load_type(p.condition_load().value());
                }
                if (p.has_condition_timeofday()) {
                    if (p.condition_timeofday().has_startminutes()) {
                        pn->set_start_minues(p.condition_timeofday().startminutes());
                    }
                    if (p.condition_timeofday().has_endminutes()) {
                        pn->set_end_minues(p.condition_timeofday().endminutes());
                    }
                }
                if (p.has_condition_weather()) {
                    pn->set_weather(p.condition_weather().weather());
                }
                if (p.has_condition_fuzzytime()) {
                    pn->set_fuzzy_time(p.condition_fuzzytime().fuzzytime());
                }
                if (p.has_condition_turndirection()) {
                    pn->set_direction(p.condition_turndirection().direction());
                }
            }
            info->mutable_id()->set_value(obj->id()->tileid(), TrafficInfoProxy::ELEM_ID_TYPE, 0, timestamp);
            info->make_id_index();
            if (!info->is_valid()) {
                LOG(ERROR) << "Construct trafficsign " << sign.id() << " info invalid " << info->id()->to_string();
            }
            cnt++;
            tile->traffic_info_refs().push_back(info->id());
            id2info[sign.id()] = info;
        }
    }
    for (auto& li : env_msg.physical_layer().trafficlights()) {        
        auto obj = tmptile->mutable_position_objects()->add();
        auto border = obj->mutable_border();
        for (auto& pt : li.border().points()) {
            auto pts = border->mutable_pts()->add();
            pts->set_x(pt.x());
            pts->set_y(pt.y());
            pts->set_z(pt.z());            
        }
        if (border->pts().empty()) {
            continue;
        }
        //LOG(INFO) << "Construct trafficlight " << li.id() << " with " << border->pts().size() << " pts";
        Vector3D pos = *(border->pts()[0]);
        obj->set_type(RoadPB::PositionObject::TRAFFIC_LIGHT);
        obj->set_content(std::to_string(li.state().state()) + " : " + std::to_string(li.type().type()));
        if (id2obj.count(li.id()) > 0) {
            LOG(ERROR) << "Duplicated id found " << li.id();
        }
        id2obj[li.id()] = obj;
        TileInfoPtr tile;
        if (mgr->make_new_id(pos, obj, tile, true)) {
            if (!obj->is_valid()) {
                LOG(ERROR) << "Construct trafficlight " << li.id() << " object invalid " << obj->id()->to_string();
            }
            cnt++;
            tile->position_objects().push_back(obj);
            tile->position_object_refs().push_back(obj->id());
            auto info = tile->traffic_infos().add();
            info->mutable_objs()->push_back(obj->id());
            info->set_type(RoadPB::TrafficInfo::LIGHT);
            auto light = info->mutable_light();
            if (li.has_type()) {
                light->set_type(li.type().type());
            }
            light->set_bulbs_num(li.bulbs_size());
            /*for (auto& bb : li.bulbs()) {
                auto b = light->mutable_bulbs()->add();
                if (bb.has_shape()) {
                    b->set_shape(bb.shape().type());
                }
                if (bb.has_cborder()) {
                    auto c = b->mutable_border();
                    c->set_radius(bb.cborder().radius().value());
                    auto center = c->mutable_center();
                    center->set_x(bb.cborder().center().x());
                    center->set_y(bb.cborder().center().y());
                    center->set_z(bb.cborder().center().z());
                }
            }*/
            
            info->mutable_id()->set_value(obj->id()->tileid(), TrafficInfoProxy::ELEM_ID_TYPE, 0, timestamp);
            info->make_id_index();
            if (!info->is_valid()) {
                LOG(ERROR) << "Construct trafficlight " << li.id() << " info invalid " << info->id()->to_string();
            }
            cnt++;
            tile->traffic_info_refs().push_back(info->id());
            id2info[li.id()] = info;
        }
    }
    for (auto& sp : env_msg.physical_layer().speedbumps()) {        
        auto obj = tmptile->mutable_position_objects()->add();
        auto border = obj->mutable_border();
        for (auto& pt : sp.border().points()) {
            auto pts = border->mutable_pts()->add();
            pts->set_x(pt.x());
            pts->set_y(pt.y());
            pts->set_z(pt.z());            
        }
        if (border->pts().empty()) {
            continue;
        }
        //LOG(INFO) << "Construct speedbump " << sp.id() << " with " << border->pts().size() << " pts";
        Vector3D pos = *(border->pts()[0]);
        obj->set_type(RoadPB::PositionObject::SPEED_BUMP);
        if (id2obj.count(sp.id()) > 0) {
            LOG(ERROR) << "Duplicated id found " << sp.id();
        }
        id2obj[sp.id()] = obj;
        TileInfoPtr tile;
        if (mgr->make_new_id(pos, obj, tile, true)) {
            if (!obj->is_valid()) {
                LOG(ERROR) << "Construct speedbump " << sp.id() << " object invalid " << obj->id()->to_string();
            }
            cnt++;
            tile->position_objects().push_back(obj);
            tile->position_object_refs().push_back(obj->id());
            auto info = tile->traffic_infos().add();
            info->mutable_objs()->push_back(obj->id());
            info->set_type(RoadPB::TrafficInfo::SPEED_BUMP);
            auto bump = info->mutable_bump();
            bump->set_height(sp.height().value());
            //bump->set_speed_limit(sp.speedlimit().value());
            info->mutable_id()->set_value(obj->id()->tileid(), TrafficInfoProxy::ELEM_ID_TYPE, 0, timestamp);
            info->make_id_index();
            if (!info->is_valid()) {
                LOG(ERROR) << "Construct trafficlight " << sp.id() << " info invalid " << info->id()->to_string();
            }
            cnt++;
            tile->traffic_info_refs().push_back(info->id());
            id2info[sp.id()] = info;
        }
    }
    std::unordered_map<std::string, std::shared_ptr<LaneBoundaryProxy>> id2lb;
    for (auto& line : env_msg.physical_layer().lanelines()) {
        auto lb = tmptile->lane_boundarys().add();
        auto pts = lb->mutable_geom()->mutable_pts();
        for (auto& l : line.lines()) {
            for (auto& p : l.points()) {
                auto pt = pts->add();
                pt->set_x(p.x());
                pt->set_y(p.y());
                pt->set_z(p.z());
            }
        }
        if (pts->size() < 2) {
            continue;
        }
        //LOG(INFO) << "Construct laneline " << line.id() << " with " << pts->size() << " pts";
        if (id2lb.count(line.id()) > 0) {
            LOG(ERROR) << "Duplicated id found " << line.id();
        }
        id2lb[line.id()] = lb;
        Vector3D pos = *(pts->at(0));
        lb->set_types({line.type().type()});
        if (line.has_color()) {
            lb->set_color(line.color().color());
        }
        //if (line.has_confidence()) {
            //lb->set_confidence(line.confidence());
        //}
        if (line.has_marking()) {
            lb->set_marking(line.marking());
        }
        if (line.has_ldm()) {
            lb->set_ldm(line.ldm());
        }
        TileInfoPtr tile;
        if (mgr->make_new_id(pos, lb, tile, true)) {
            if (!lb->is_valid()) {
                LOG(ERROR) << "Construct laneline " << line.id() << " laneboundary invalid " << lb->id()->to_string();
            }
            cnt++;
            tile->lane_boundarys().push_back(lb);
            tile->lane_boundary_refs().push_back(lb->id());
        }
    }
    for (auto& line : env_msg.logical_layer().virtuallines()) {
        auto lb = tmptile->lane_boundarys().add();
        auto pts = lb->mutable_geom()->mutable_pts();
        for (auto& l : line.lines()) {
            for (auto& p : l.points()) {
                auto pt = pts->add();
                pt->set_x(p.x());
                pt->set_y(p.y());
                pt->set_z(p.z());
            }
        }
        if (pts->size() < 2) {
            continue;
        }
        //LOG(INFO) << "Construct virtualline " << line.id() << " with " << pts->size() << " pts";
        if (id2lb.count(line.id()) > 0) {
            LOG(ERROR) << "Duplicated id found " << line.id();
        }
        id2lb[line.id()] = lb;
        Vector3D pos = *(pts->at(0));
        lb->set_types({line.type().type()});
        if (line.has_color()) {
            lb->set_color(line.color().color());
        }
        //if (line.has_confidence()) {
            //lb->set_confidence(line.confidence());
        //}
        if (line.has_marking()) {
            lb->set_marking(line.marking());
        }
        if (line.has_ldm()) {
            lb->set_ldm(line.ldm());
        }
        TileInfoPtr tile;
        if (mgr->make_new_id(pos, lb, tile, true)) {
            if (!lb->is_valid()) {
                LOG(ERROR) << "Construct virtualline " << line.id() << " laneboundary invalid " << lb->id()->to_string();
            }
            cnt++;
            tile->lane_boundarys().push_back(lb);
            tile->lane_boundary_refs().push_back(lb->id());
        }
    }
    std::unordered_map<std::string, std::shared_ptr<LaneExtProxy>> id2lane(1024);
    std::unordered_multimap<std::string, std::string> id2pred;
    std::unordered_multimap<std::string, std::string> id2succ;
    for (auto& lane : env_msg.logical_layer().lanes()) {
        auto ln = std::dynamic_pointer_cast<LaneExtProxy>(tmptile->lanes().add());
        for (auto& d : lane.driveline_ids()) {
            if (id2lb.find(d) != id2lb.end()) {
                ln->mutable_drivelines()->push_back(id2lb[d]->id());
            }
            else {
                LOG(ERROR) << "Found unknown driveline " << d;
            }
        }
        
        //LOG(INFO) << "Construct lane " << lane.id();
        auto ls = ln->mutable_lanes()->add();
        for (auto& l : lane.l_laneline_ids()) {
            if (id2lb.find(l) != id2lb.end()) {
                auto lb = id2lb[l];
                ls->mutable_left_boundary()->mutable_bound_id()->set(lb->id());
                ls->mutable_left_boundary()->mutable_start_pt()->set(*(lb->geom()->pts().front()));
                ls->mutable_left_boundary()->mutable_end_pt()->set(*(lb->geom()->pts().back()));
                break;
            }
        }
        Vector3D pos;
        for (auto& l : lane.r_laneline_ids()) {
            if (id2lb.find(l) != id2lb.end()) {
                auto lb = id2lb[l];
                ls->mutable_right_boundary()->mutable_bound_id()->set(lb->id());
                ls->mutable_right_boundary()->mutable_start_pt()->set(*(lb->geom()->pts().front()));
                ls->mutable_right_boundary()->mutable_end_pt()->set(*(lb->geom()->pts().back()));
                pos = *(lb->geom()->pts().front());
                break;
            }
        }
        if (!ls->left_boundary() || !ls->right_boundary()) {
            LOG(ERROR) << "Invalid lane " << lane.id() << " without boundary";
            continue;
        }
        id2lane[lane.id()] = ln;
        if (lane.has_direction()) {
            auto dir = ln->mutable_directions()->add();
            dir->set_direction(lane.direction().direction());
        }
        if (lane.has_function()) {
            ln->set_function(lane.function().function());
        }
        if (lane.has_lane_length()) {
            ln->set_length(lane.lane_length());
        }
        if (lane.has_priority()) {
            ln->set_priority(lane.priority().priority());
        }
        if (lane.has_transition()) {
            ln->set_transition(lane.transition());
        }
        if (lane.has_type()) {
            ln->set_type(lane.type());
        }
        std::vector<int> lrs;
        for (int i = 0; i < lane.l_restrictions_size(); i++) {
            lrs.push_back(lane.l_restrictions(i).type());
        }
        ln->set_l_restrictions(lrs);
        std::vector<int> rrs;
        for (int i = 0; i < lane.r_restrictions_size(); i++) {
            rrs.push_back(lane.r_restrictions(i).type());
        }
        ln->set_r_restrictions(rrs);

        const ::ndm_proto::SpeedLimit* maxlimit = nullptr;
        const ::ndm_proto::SpeedLimit* minlimit = nullptr;
        for (auto& r : lane.restrictions()) {
            if (r.has_height_limit()) {
                ls->set_height_limit(r.height_limit().value());
            }
            if (r.has_weight_limit()) {
                ls->set_width_limit(r.weight_limit().value());
            }
            for (auto& sl : r.speed_limits()) {
                if (sl.limit_type().type() == ndm_proto::LaneMarkingType::SPEED_LIMIT_LOW) {
                    if (minlimit == nullptr || minlimit->end_offset() - minlimit->offset() < sl.end_offset() - sl.offset()) {
                        minlimit = &sl;
                    }
                }
                else if (sl.limit_type().type() == ndm_proto::LaneMarkingType::SPEED_LIMIT_HIGH) {
                    if (maxlimit == nullptr || maxlimit->end_offset() - maxlimit->offset() < sl.end_offset() - sl.offset()) {
                        maxlimit = &sl;
                    }
                }                
            }
        }
        if (minlimit != nullptr) {
            auto s = ls->mutable_speed_limits()->add();
            s->set_min_speed(minlimit->speed_value().value());
        }
        if (maxlimit != nullptr) {   
            auto s = ls->mutable_speed_limits()->add();
            s->set_max_speed(maxlimit->speed_value().value());
        }
        
        for (auto& pred : lane.pred_ids()) {
            id2pred.insert(std::make_pair(lane.id(), pred));
        }
        for (auto& succ : lane.succ_ids()) {
            id2succ.insert(std::make_pair(lane.id(), succ));
        }        
        TileInfoPtr tile;
        if (mgr->make_new_id(pos, ln, tile, true)) {
            if (!ln->is_valid()) {
                LOG(ERROR) << "Construct lane " << lane.id() << " lane invalid " << ln->id()->to_string();
            }
            cnt++;
            tile->lanes().push_back(ln);
            tile->lane_refs().push_back(ln->id());
        }
        for (auto& link : lane.objs()) {
            if (id2info.find(link.id()) != id2info.end()) {
                id2info[link.id()]->mutable_lanes()->set(ln->id());
            }
        }                
    }
    for (auto& lane : env_msg.logical_layer().virtual_lanes()) {
        auto ln = std::dynamic_pointer_cast<LaneExtProxy>(tmptile->lanes().add());
        for (auto& d : lane.driveline_ids()) {
            if (id2lb.find(d) != id2lb.end()) {
                ln->mutable_drivelines()->push_back(id2lb[d]->id());
            }
            else {
                LOG(ERROR) << "Found unknown driveline " << d;
            }
        }
        //LOG(INFO) << "Construct virtual lane " << lane.id();
        auto ls = ln->mutable_lanes()->add();
        for (auto& l : lane.left_ids()) {
            if (id2lb.find(l) != id2lb.end()) {
                auto lb = id2lb[l];
                ls->mutable_left_boundary()->mutable_bound_id()->set(lb->id());
                ls->mutable_left_boundary()->mutable_start_pt()->set(*(lb->geom()->pts().front()));
                ls->mutable_left_boundary()->mutable_end_pt()->set(*(lb->geom()->pts().back()));
                break;
            }
        }
        Vector3D pos;
        for (auto& l : lane.right_ids()) {
            if (id2lb.find(l) != id2lb.end()) {
                auto lb = id2lb[l];
                ls->mutable_right_boundary()->mutable_bound_id()->set(lb->id());
                ls->mutable_right_boundary()->mutable_start_pt()->set(*(lb->geom()->pts().front()));
                ls->mutable_right_boundary()->mutable_end_pt()->set(*(lb->geom()->pts().back()));
                pos = *(lb->geom()->pts().front());
                break;
            }
        }
        if (!ls->left_boundary() || !ls->right_boundary()) {
            LOG(ERROR) << "Invalid virtual lane " << lane.id() << " without boundary";
            continue;
        }
        id2lane[lane.id()] = ln;
        if (lane.has_direction()) {
            auto dir = ln->mutable_directions()->add();
            dir->set_direction(lane.direction().direction());
        }
        if (lane.has_function()) {
            ln->set_function(lane.function().function());
        }
        if (lane.has_lane_length()) {
            ln->set_length(lane.lane_length());
        }
        if (lane.has_priority()) {
            ln->set_priority(lane.priority().priority());
        }
        if (lane.has_transition()) {
            ln->set_transition(lane.transition());
        }
        if (lane.has_type()) {
            ln->set_type(lane.type());
        }
        std::vector<int> lrs;
        for (int i = 0; i < lane.l_restrictions_size(); i++) {
            lrs.push_back(lane.l_restrictions(i).type());
        }
        ln->set_l_restrictions(lrs);
        std::vector<int> rrs;
        for (int i = 0; i < lane.r_restrictions_size(); i++) {
            rrs.push_back(lane.r_restrictions(i).type());
        }
        ln->set_r_restrictions(rrs);
        for (auto& r : lane.restrictions()) {
            if (r.has_height_limit()) {
                ls->set_height_limit(r.height_limit().value());
            }
            if (r.has_weight_limit()) {
                ls->set_width_limit(r.weight_limit().value());
            }
            for (auto& sl : r.speed_limits()) {
                if (sl.limit_type().type() == ndm_proto::LaneMarkingType::SPEED_LIMIT_LOW) {
                    auto s = ls->mutable_speed_limits()->add();
                    s->set_max_speed(sl.speed_value().value());                     
                }
                else if (sl.limit_type().type() == ndm_proto::LaneMarkingType::SPEED_LIMIT_HIGH) {
                    auto s = ls->mutable_speed_limits()->add();
                    s->set_max_speed(sl.speed_value().value());
                }
            }
        }
        for (auto& pred : lane.pred_ids()) {
            id2pred.insert(std::make_pair(lane.id(), pred));
        }
        for (auto& succ : lane.succ_ids()) {
            id2succ.insert(std::make_pair(lane.id(), succ));
        }
        TileInfoPtr tile;
        if (mgr->make_new_id(pos, ln, tile, true)) {
            if (!ln->is_valid()) {
                LOG(ERROR) << "Construct virtual lane " << lane.id() << " lane invalid " << ln->id()->to_string();
            }
            cnt++;
            tile->lanes().push_back(ln);
            tile->lane_refs().push_back(ln->id());
        }
    }
    for (auto& sit : id2pred) {
        if (id2lane.count(sit.first) <= 0 || id2lane.count(sit.second) <= 0) {
            continue;
        }
        auto l = id2lane[sit.first];
        auto p = id2lane[sit.second];
        l->mutable_preds()->push_back(p->id());
    }
    for (auto& sit : id2succ) {
        if (id2lane.count(sit.first) <= 0 || id2lane.count(sit.second) <= 0) {
            continue;
        }
        auto l = id2lane[sit.first];
        auto s = id2lane[sit.second];
        l->mutable_succs()->push_back(s->id());
    }
    std::unordered_map<std::string, std::shared_ptr<LaneGroupProxy>> id2lanegroup;
    std::unordered_multimap<std::string, std::string> id2lgsucc;
    std::unordered_multimap<std::string, std::string> id2lgpred;
    for (auto& section : env_msg.logical_layer().sections()) {
        auto lg = tmptile->lane_groups().add();
        //LOG(INFO) << "Construct section " << section.id();
        if (id2lanegroup.count(section.id()) > 0) {
            LOG(ERROR) << "Duplicated id found " << section.id();
        }
        id2lanegroup[section.id()] = lg;
        int seqno = -1;
        for (auto& lid : section.lane_ids()) {
            if (id2lane.find(lid) == id2lane.end()) {
                continue;
            }
            auto ln = id2lane[lid];
            ln->set_seq_no(seqno);
            --seqno;
            lg->mutable_lanes()->push_back(ln->id());
        }
        for (auto& oid : section.objs()) {            
            if (id2obj.find(oid.id()) == id2obj.end()) {
                continue;
            }
            auto obj = id2obj[oid.id()];
            auto lp = lg->mutable_lanes()->front().proxy();
            lp->mutable_lanes()->front()->mutable_objects()->push_back(obj->id());
        }
        for (auto& id : section.pred_ids()) {
            id2lgpred.insert(std::make_pair(section.id(), id));
        }
        for (auto& id : section.succ_ids()) {
            id2lgsucc.insert(std::make_pair(section.id(), id));
        }
        TileInfoPtr tile;
        Vector3D pos = lg->wgs84_pos();
        if (mgr->make_new_id(pos, lg, tile, true)) {
            if (!lg->is_valid()) {
                LOG(ERROR) << "Construct section " << section.id() << " lanegroup invalid " << lg->id()->to_string();
            }
            cnt++;
            tile->lane_groups().push_back(lg);
            tile->lane_group_refs().push_back(lg->id());
        }
        for (auto& link : section.objs()) {
            if (id2info.find(link.id()) != id2info.end()) {
                id2info[link.id()]->mutable_lanegroups()->set(lg->id());
            }
            else if (id2obj.find(link.id()) != id2obj.end()) {
                id2obj[link.id()]->mutable_lane_groups()->push_back(lg->id());
            }
        }  
    }
    for (auto& sit : id2lgpred) {
        if (id2lanegroup.count(sit.first) <= 0 || id2lanegroup.count(sit.second) <= 0) {
            continue;
        }
        auto l = id2lanegroup[sit.first];
        auto p = id2lanegroup[sit.second];
        l->mutable_preds()->push_back(p->id());
    }
    for (auto& sit : id2lgsucc) {
        if (id2lanegroup.count(sit.first) <= 0 || id2lanegroup.count(sit.second) <= 0) {
            continue;
        }
        auto l = id2lanegroup[sit.first];
        auto s = id2lanegroup[sit.second];
        l->mutable_succs()->push_back(s->id());
    }
    tmptile.reset();
    return cnt;
};

void upload_tiles(data_access_engine::RoadGeometryManager* mgr, bool copy_link = false) {
    data_access_engine::TileInfoList tiles;
    data_access_engine::ID2TileMap id2tiles;
    mgr->get_road_tiles(id2tiles);
    std::set<int> tids;
    for (auto& t : id2tiles) {
        tids.insert(t.second->tile_id());        
    }
    auto conf = ConfigAddress::get_instance();
    conf->set_road_server_download_branch(FLAGS_branch);
    std::vector<int> ts(tids.begin(), tids.end());
    mgr->clear_all();
    mgr->load_tiles_by_id(ts, nullptr, 4);
    mgr->merge_features(id2tiles);
    mgr->correct_tiles();
    mgr->correct_tile_refs();
    id2tiles.clear();
    mgr->get_road_tiles(id2tiles);
    for (auto& t : id2tiles) {
        tiles.push_back(t.second);
        t.second->set_version(0);
    }
    
    LOG(INFO) << "Upload " << tiles.size() << " tiles";
    mgr->upload_tiles("ndm_converter", tiles, -2, 4);
    if (copy_link && FLAGS_branch != FLAGS_down_branch) {
        std::vector<int> req_types = { RoadPB::FeatureID::LINK, RoadPB::FeatureID::NODE };
        conf->set_road_server_download_branch(FLAGS_down_branch);
        int64_t ver = 0;
        mgr->merge_tiles(ts, req_types, "ndm_converter", 0, 4, ver);
        mgr->clear_all();
    }
};


class LaneExtProxyCreator : public data_access_engine::FeatureProxyCreator<data_access_engine::LaneProxy> {
public:
    virtual LaneProxy* create() {
        return new LaneExtProxy();
    }
    virtual LaneProxy* create(FeatureProxyBase* p) {
        return new LaneExtProxy(p);
    }
};

int main(int argc, char *argv[]) {
    FLAGS_alsologtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    google::InitGoogleLogging(argv[0]);
    GFLAGS_NS::ParseCommandLineFlags(&argc, &argv, true);
    LOG(INFO) << "ndm_converter start for " << FLAGS_file << " @ " << FLAGS_branch << " on " << FLAGS_server;
    auto conf = ConfigAddress::get_instance();
    conf->set_road_server_address(FLAGS_server);
    conf->set_road_server_download_branch(FLAGS_down_branch);
    conf->set_road_server_upload_branch(FLAGS_branch);
    auto mgr = DAEInterface::get_instance()->get_road_geometry_manager();
    int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    mgr->set_current_version(timestamp);    
    delete FeatureProxyBase::register_creator<LaneProxy>(new LaneExtProxyCreator());
    ProjectionHelper<11> tp;    
    if (!FLAGS_range.empty()) {
        double minx = 1000;
        double miny = 1000;
        double maxx = 0;
        double maxy = 0;
        char* ptr = (char*)FLAGS_range.c_str();
        char* stp = strchr(ptr, '(');
        char* edp = strchr(ptr, ')');
        ptr = stp + 1;
        std::unordered_set<int> tids;
        while (ptr && ptr < edp) {
            double x = strtod(ptr, &ptr);
            ptr++;
            double y = strtod(ptr, &ptr);
            ptr++;
            minx = std::min(minx, x);
            miny = std::min(miny, y);
            maxx = std::max(maxx, x);
            maxy = std::max(maxy, y);
        }
        //bind_link(minx, miny, maxx, maxy, mgr);
        //upload_tiles(mgr);
        conf->set_road_server_download_branch(FLAGS_branch);
        check_roads(minx, miny, maxx, maxy, mgr);
        return 0;
    }
    if (!FLAGS_file.empty()) {
        std::experimental::filesystem::recursive_directory_iterator fit(FLAGS_file.c_str());
        std::experimental::filesystem::recursive_directory_iterator fed;
        int cnt = 0;
        int total = 0;
        for (; fit != fed; ++fit) {
            auto& p = fit->path();
            if (p.has_extension()) {
                int c = load_pb(p.c_str(), mgr);
                cnt += c;
                total += c;
                LOG(INFO) << "Construct " << c << " / " << total << " elems from " << p.c_str();
                if (cnt > 1000000) {
                    upload_tiles(mgr, true);
                    cnt = 0;
                }
            }
        }
        upload_tiles(mgr, true);
    }
    LOG(INFO) << "ndm_converter is going to quit";
    return 0;
}

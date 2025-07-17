#include "dao/tile_server_dao.h"
#include <unordered_set>
#include <thread>
#include "flatbuffers/util.h"
#include "public/sha1.hpp"

#ifdef _MSC_VER
#include <sofa/pbrpc/pbrpc.h>
#include "log.h"
#else
#include <baidu/rpc/server.h>
#include <baidu/rpc/channel.h>

#define HRESULT int
#define S_OK 0
#define S_FALSE 1
#define E_FAIL -1
#define __cdecl

#ifndef _countof
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif
#define sprintf_s sprintf
#endif

#ifdef _MSC_VER
#define INITIALIZE_RPC(SERVICE_TYPE, SERVER_ADDRESS)\
    sofa::pbrpc::RpcClientOptions client_options; \
    client_options.keep_alive_time = -1; \
    sofa::pbrpc::RpcClient rpc_client(client_options); \
    sofa::pbrpc::RpcChannelOptions channel_options; \
    channel_options.connect_timeout = 600000; \
    sofa::pbrpc::RpcChannel channel(&rpc_client, SERVER_ADDRESS, channel_options); \
    services::SERVICE_TYPE##Service_Stub stub(&channel); \
    sofa::pbrpc::RpcController crtl; \
    crtl.SetTimeout(600000); \
    //crtl.SetRequestCompressType(sofa::pbrpc::CompressTypeZlib);
#else
#define INITIALIZE_RPC(SERVICE_TYPE, SERVER_ADDRESS)\
    brpc::Channel channel; \
    brpc::ChannelOptions options; \
    options.connect_timeout_ms = 600000; \
    options.timeout_ms = 600000; \
    channel.Init(SERVER_ADDRESS.c_str(), &options); \
    services::SERVICE_TYPE##Service_Stub stub(&channel); \
    brpc::Controller crtl; \
    crtl.set_timeout_ms(600000); \
    //crtl.set_request_compress_type(brpc::COMPRESS_TYPE_GZIP);
#endif

#ifdef _MSC_VER
#define RPC_REQUEST(RPC_FUNC, REQUEST_VARIABLE, RESPONSE_VARIABLE) \
    crtl.Reset(); \
    crtl.SetTimeout(600000); \
    /*crtl.SetRequestCompressType(sofa::pbrpc::CompressTypeZlib);*/ \
    stub.RPC_FUNC(&crtl, &REQUEST_VARIABLE, &RESPONSE_VARIABLE, NULL); 
#else
#define RPC_REQUEST(RPC_FUNC, REQUEST_VARIABLE, RESPONSE_VARIABLE) \
    crtl.set_timeout_ms(600000); \
    /*crtl.set_request_compress_type(brpc::COMPRESS_TYPE_GZIP);*/ \
    stub.RPC_FUNC(&crtl, &REQUEST_VARIABLE, &RESPONSE_VARIABLE, NULL);
#endif

#ifdef _MSC_VER
#define DO_RPC(SERVICE_TYPE, SERVER_ADDRESS, RPC_FUNC, REQUEST_VARIABLE, RESPONSE_VARIABLE) \
{ \
    INITIALIZE_RPC(SERVICE_TYPE, SERVER_ADDRESS); \
    RPC_REQUEST(RPC_FUNC, REQUEST_VARIABLE, RESPONSE_VARIABLE); \
    if (crtl.Failed()) { \
        XLogError(XTAG(L"tile"), L"%S() to %S failed: %S", #RPC_FUNC, SERVER_ADDRESS.c_str(), crtl.ErrorText().c_str()); \
        if (crtl.ErrorText().find("too big") < crtl.ErrorText().size()) { \
            RESPONSE_VARIABLE.mutable_ret()->set_code(137); \
        } \
    } \
}
#else    
#define DO_RPC(SERVICE_TYPE, SERVER_ADDRESS, RPC_FUNC, REQUEST_VARIABLE, RESPONSE_VARIABLE) \
{ \
    INITIALIZE_RPC(SERVICE_TYPE, SERVER_ADDRESS); \
    RPC_REQUEST(RPC_FUNC, REQUEST_VARIABLE, RESPONSE_VARIABLE); \
    if (crtl.Failed()) { \
        LOG(ERROR) << #RPC_FUNC << "() to " << SERVER_ADDRESS << " failed: " << crtl.ErrorText(); \
        if (crtl.ErrorText().find("too big") < crtl.ErrorText().size()) { \
            RESPONSE_VARIABLE.mutable_ret()->set_code(137); \
        } \
    } \
}
#endif

#define DO_RESOURCE_RPC(RPC_FUNC, REQUEST_VARIABLE, RESPONSE_VARIABLE) \
    DO_RPC(Resource, _resource_server, RPC_FUNC, REQUEST_VARIABLE, RESPONSE_VARIABLE)

#define DO_SIMPLE_RESOURCE_RPC(RPC_FUNC) \
    DO_RPC(Resource, _resource_server, RPC_FUNC, request, response)

#define DO_TILE_RPC(RPC_FUNC, REQUEST_VARIABLE, RESPONSE_VARIABLE) \
    DO_RPC(Tile, _tile_server, RPC_FUNC, REQUEST_VARIABLE, RESPONSE_VARIABLE)

#define DO_SIMPLE_TILE_RPC(RPC_FUNC) \
    DO_RPC(Tile, _tile_server, RPC_FUNC, request, response)

void TilePosInfo::world2local(double& x, double& y, double& z) const
{
    Eigen::Vector3d p = affine_w_2_l * Eigen::Vector3d(x, y, z);
    x = p.x();
    y = p.y();
    z = p.z();
}

void TilePosInfo::local2world(double& x, double& y, double& z) const
{
    Eigen::Vector3d p = affine_l_2_w * Eigen::Vector3d(x, y, z);
    x = p.x();
    y = p.y();
    z = p.z();
}

Eigen::Vector3d TilePosInfo::get_heading() const
{
    Eigen::Vector3d dir(0, 1, 0);
    dir = affine_l_2_w * dir;
    dir = dir - pos;
    dir.normalize();
    return dir;
}

double TilePosInfo::get_head() const
{
    return std::atan2(2.0 * (quat.w() * quat.x() + quat.y() * quat.z()), 1 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y()));
}

double TilePosInfo::get_pitch() const
{
    return std::asin(2.0 * (quat.w() * quat.y() - quat.z() * quat.x()));
}

double TilePosInfo::get_roll() const
{
    return std::atan2(2.0 * (quat.w() * quat.z() + quat.x() * quat.y()), 1 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
}

bool TileImagePos::project_3d_2_2d(double x, double y, double z, int& px, int& py) const
{
    /*double cam_x = mat[0] * x + mat[1] * y + mat[2] * z + mat[3];
    double cam_y = mat[4] * x + mat[5] * y + mat[6] * z + mat[7];
    double cam_z = mat[8] * x + mat[9] * y + mat[10] * z + mat[11];

    px = std::round(cam_x / cam_z);
    py = std::round(cam_y / cam_z);

    if (cam_z > 0.0 && px >= 0 && px < 2448 && py >= 0 && py < 2048)
    {
        return true;
    }*/
    return false;
}

const std::string CTileServerDao::get_server_info() const
{
    return _branch + "|" + std::to_string(_tile_id);
}

//    bool CTileServerDao::download_clouds(const std::string& path, std::vector<TileCloudInfo>& infos) {
//        bool bsucc = true;
//        for (auto& info : infos) {
//            if (info.path.empty()) {
//                info.path = path + "/" + info.pos.to_str();
//            }
//            std::string data;
//            if (download_cloud_data(info.pos, data)) {
//                FILE* file = fopen(info.path.c_str(), "wb");
//                if (!file) {
//                    bsucc = false;
//                    continue;
//                }
//                if (!fwrite(data.c_str(), data.size(), 1, file)) {
//                    bsucc = false;
//                    fclose(file);
//                    continue;
//                }
//                fclose(file);
//            }
//            else {
//                bsucc = false;
//            }
//        }
//        return bsucc;
//    };
//
//    bool CTileServerDao::download_cloud_data(const TilePosInfo& pos, std::string& data) {
//        hadmap::ResourceDownloadRequest request;
//        hadmap::ResourceDownloadResponse response;
//        auto rrg = request.add_res();
//        auto res = rrg->mutable_res();
//        res->mutable_id()->CopyFrom(pos.res_id);
//        if (pos.res_id.tile_id() != pos.tile_id) {
//            int64_t version = pos.res_id.version();
//        }
//        request.set_branch(_branch);
//        DO_SIMPLE_TILE_RPC(DownloadData);
//        if (!response.has_response_msg() || response.response_msg().ret_code() != 0 || response.res_size() <= 0 || response.res(0).res().slots_size() <= 0
//            || !response.res(0).res().slots(0).has_data() || response.res(0).res().slots(0).data().size() <= 0) {
//#ifdef WIN32
//        XLogError(XTAG(L"CTileServerDao"), L"Failed to download pcd index:%ld tile_id:%d trail_id:%ld version:%lld branch:%S",
//            pos.index, pos.tile_id, pos.trail_id, pos.version, _branch.c_str());
//#else
//            LOG(ERROR) << "Failed to download pcd index:" << pos.index << " tile_id:" << pos.tile_id << " trail_id:" << pos.trail_id << " version:" << pos.version << " branch:" << _branch;
//#endif
//            return false;
//        }
//        data.swap(*response.mutable_res(0)->mutable_res()->mutable_slots(0)->mutable_data());
//        return true;
//    };
//
//    pcl::PointCloud<PointXYZIRGB>::Ptr CTileServerDao::get_pointcloud(const TileCloudInfo& id) {
//        std::string data;
//        if (!download_cloud_data(id.pos, data)) {
//            return nullptr;
//        }
//        flatbuffers::Verifier v((const uint8_t*)data.c_str(), data.size());
//        if (!HadmapFBS::VerifyPointCloudBuffer(v)) {
//            pcl::PointCloud<::PointXYZIRGB>::Ptr cloud(new pcl::PointCloud<::PointXYZIRGB>);
//            return cloud;
//        }
//
//        using namespace HadmapFBS;
//        auto pcd = GetPointCloud(data.c_str());
//        //     if (pcd->point_type() != PointType_XYZIRGB_Frame) {
//        //         return nullptr;
//        //     }
//        if (!pcd->points_xyzirgb() || !pcd->points_mask()) {
//            pcl::PointCloud<::PointXYZIRGB>::Ptr cloud(new pcl::PointCloud<::PointXYZIRGB>);
//            return cloud;
//        }
//        auto& ps = *pcd->points_xyzirgb();
//        auto& masks = *pcd->points_mask();
//        pcl::PointCloud<::PointXYZIRGB>::Ptr cloud(new pcl::PointCloud<::PointXYZIRGB>(pcd->width(), masks.size()));
//        if (pcd->header()) {
//            cloud->header.seq = pcd->header()->seq();
//            cloud->header.frame_id = pcd->header()->frame_id()->str();
//            cloud->header.stamp = pcd->header()->stamp();
//            cloud->is_dense = pcd->is_dense();
//        }
//        if (pcd->sensor_origin()) {
//            cloud->sensor_origin_.x() = pcd->sensor_origin()->x();
//            cloud->sensor_origin_.y() = pcd->sensor_origin()->y();
//            cloud->sensor_origin_.z() = pcd->sensor_origin()->z();
//        }
//        if (pcd->sensor_orientation()) {
//            cloud->sensor_orientation_.x() = pcd->sensor_orientation()->x();
//            cloud->sensor_orientation_.y() = pcd->sensor_orientation()->y();
//            cloud->sensor_orientation_.z() = pcd->sensor_orientation()->z();
//            cloud->sensor_orientation_.w() = pcd->sensor_orientation()->w();
//        }
//
//        int cnt = 1;
//        for (int i = 0; i < masks.size(); ++i) {
//            uint32_t mask = masks[i];
//            for (int j = 0; j < 32; ++j) {
//                auto po = ps[0];
//                auto& p = cloud->at(i * 32 + j);
//                uint32_t m = 1;
//                if (mask & (m << j)) {
//                    po = ps[cnt];
//                    ++cnt;
//                    if (ps.size() < cnt) {
//                        break;
//                    }
//                }
//                p.x = po->x();
//                p.y = po->y();
//                p.z = po->z();
//                p.intensity = po->intensity();
//                p.r = po->r();
//                p.g = po->g();
//                p.b = po->b();
//            }
//        }
//        return cloud;
//    };

bool CTileServerDao::download_images(const std::string& path, std::vector<TileImagePos>& infos) {
    bool bsucc = true;
    for (auto& info : infos) {
        if (info.path.empty()) {
            info.path = path + "/" + info.pos.to_str();
        }
        std::map<std::string, std::string> data;
        data["file"];
        if (download_data(info.res_id, data)) {
            FILE* file = fopen(info.path.c_str(), "wb");
            if (!file) {
                bsucc = false;
                continue;
            }
            if (!fwrite(data["file"].c_str(), data["file"].size(), 1, file)) {
                bsucc = false;
                fclose(file);
                continue;
            }
            fclose(file);
        }
        else {
            bsucc = false;
        }
    }
    return bsucc;
};

cv::Mat CTileServerDao::get_image(const TileImagePos& iid) {
    std::vector<TileImagePos> cis;
    cis.push_back(iid);
    if (!download_images(_path, cis)) {
        return cv::Mat();
    }
    return cv::imread(iid.path.c_str());
};

bool CTileServerDao::query_resources(const services::TileResourceID& id, std::vector<services::TileResource>& descs) {
    services::ResourceQueryRequest request;
    services::ResourceQueryResponse response;
    if (id.has_tile_id()) {
        request.set_tile_id(id.tile_id());
    }
    else {
        return false;
    }
    if (id.has_tile_level()) {
        request.set_tile_level(id.tile_level());
    }
    if (id.has_type()) {
        request.add_type(id.type());
    }
    else {
        return false;
    }
    if (id.has_index()) {
        request.set_index(id.index());
    }
    if (id.has_trail_id()) {
        request.set_trail_id(id.trail_id());
    }
    if (id.has_version()) {
        request.set_version(id.version());
    }
    request.set_branch(_branch);
    DO_SIMPLE_TILE_RPC(QueryResource);
    if (!response.has_ret() || response.ret().code() != 0) {
        return false;
    }
    descs.reserve(response.res_size());
    for (int i = 0; i < response.res_size(); ++i) {
        auto& res = response.res(i);
        descs.emplace_back();
        descs.back().CopyFrom(res);
    }
    return true;
};

bool CTileServerDao::download_data(const services::TileResourceID& id, std::map<std::string, std::string>& slots) {
    services::ResourceDownloadRequest request;
    services::ResourceDownloadResponse response;
    auto res = request.add_res();
    res->mutable_id()->CopyFrom(id);
    request.set_branch(_branch);
    for (auto& sit : slots) {
        res->add_slots()->set_name(sit.first);
    }
        
    DO_SIMPLE_TILE_RPC(DownloadData);
    if (!response.has_ret() || response.ret().code() != 0 || response.res_size() <= 0) {
        return false;
    }
    auto rss = response.mutable_res(0);
    for (int ind = 0; ind < rss->slots_size(); ++ind) {
        auto slot = rss->mutable_slots(ind);
        auto& data = slots[slot->name()];
        if (slot->has_data()) {
            data.swap(*slot->mutable_data());
        }                
    }
    return true;
};

//#include "json/json.h"
//DEFINE_string(extmap, ".log:101;.timestamps:102;.h264:103;.json:201;.jpg:202;.gps:203;", "file extension to type");
//DEFINE_string(filemap, "HD_LANE_EDGE.json:301;HD_TOPO.json:302;", "file name mapping to type");

bool CTileServerDao::query_pos_rect(int trail_id, std::vector<TilePosInfo>& infos) {
    if (_branch.empty()) {
        return true;
    }
    for (auto tid : _tids) {
        services::ResourceQueryRequest request;
        services::ResourceQueryResponse response;
        request.set_tile_id(tid);
        if (trail_id > 0) {
            request.set_trail_id(trail_id);
        }
        request.add_type(203);
        request.set_branch(_branch);
        int64_t req_ver = 0;
        if (req_ver > 0) {
            request.set_version(req_ver);
        }
        double cpx = 0;
        double cpy = 0;
        _proj.GetTileCenter(_tile_id, cpx, cpy);            
        DO_SIMPLE_TILE_RPC(QueryResource);
        XLogInfo(XTAG(L"tile"), L"query_pos_rect() for tile:%d branch:%S type:203 res:%d", tid, _branch.c_str(), response.res_size());
        for (int i = 0; i < response.res_size(); ++i) {
            auto& res = response.res(i);
            services::ResourceDownloadRequest req;
            services::ResourceDownloadResponse resp;
            req.set_branch(_branch);
            req.add_res()->CopyFrom(res);
            DO_TILE_RPC(DownloadData, req, resp);
            if (resp.res_size() == 1 && resp.res(0).slots_size() == 1) {
                auto& data = resp.res(0).slots(0).data();
                std::stringstream ss(data);
                char buf[4096] = { 0 };
                while (ss.good()) {
                    ss.getline(buf, _countof(buf));
                    if (buf[0] == '#' || buf[0] == '0') {
                        continue;
                    }
                    double time_stamp = 0;
                    double lat = 0;
                    double lon = 0;
                    double alt = 0;
                    double R00 = 0;
                    double R01 = 0;
                    double R02 = 0;
                    double R10 = 0;
                    double R11 = 0;
                    double R12 = 0;
                    double R20 = 0;
                    double R21 = 0;
                    double R22 = 0;
                    char name[256] = { 0 };
                    if (sscanf(buf, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %s", 
                            &time_stamp, &lat, &lon, &alt, &R00, &R01, &R02,
                            &R10, &R11, &R12, &R20, &R21, &R22, name) < 14) {
                        continue;
                    }
                    XLogInfo(XTAG(L"tile"), L"Make TilePosInfo for %S", buf);
                    TilePosInfo pos;
                    pos.res_id = res.id();
                    pos.index = (res.id().tile_id() * 10000000000L + res.id().trail_id() * 100000000L + res.id().index()) * 1000L + atoi(name);
                    pos.tile_id = res.id().tile_id();
                    pos.trail_id = res.id().trail_id();
                    pos.timestamp = time_stamp;
                    nuts::dpoint_t dp = { lon, lat };
                    nuts::dpoint_t op;
                    nuts::wgs2gcj(dp, &op);
                    pos.pos = { op.x, op.y, alt };
                    _proj.Wgs2TileLocal(_tile_id, pos.pos.x(), pos.pos.y(), pos.pos.z());
                    
                    Eigen::Matrix3d mat;
                    mat << R00, R01, R02, R10, R11, R12, R20, R21, R22;
                    pos.quat = mat;
                    pos.affine_l_2_w = Eigen::Affine3d(pos.quat).pretranslate(pos.pos);
                    pos.affine_w_2_l = pos.affine_l_2_w.inverse();
                    infos.push_back(pos);
                }
            }
        }
    }
    
    return true;
};


bool CTileServerDao::query_image_rect(int trail_id, std::vector<TileImagePos>& infos) {
    if (_branch.empty()) {
        return true;
    }
    for (auto tid : _tids) {
        services::ResourceQueryRequest request;
        services::ResourceQueryResponse response;
        request.set_tile_id(tid);
        if (trail_id > 0) {
            request.set_trail_id(trail_id);
        }
        request.add_type(202);
        request.set_branch(_branch);
        int64_t req_ver = 0;
        if (req_ver > 0) {
            request.set_version(req_ver);
        }
        double cpx = 0;
        double cpy = 0;
        _proj.GetTileCenter(_tile_id, cpx, cpy);
        DO_SIMPLE_TILE_RPC(QueryResource);
        XLogInfo(XTAG(L"tile"), L"query_image_rect() for tile:%d branch:%S type:202 res:%d", tid, _branch.c_str(), response.res_size());
        std::map<int64_t, services::TileResourceID> ind2id;
        for (int i = 0; i < response.res_size(); ++i) {
            auto& res = response.res(i);
            ind2id[res.id().index()] = res.id();
        }

        response.Clear();
        request.clear_type();
        request.add_type(203);
        DO_SIMPLE_TILE_RPC(QueryResource);
        XLogInfo(XTAG(L"tile"), L"query_image_rect() for tile:%d branch:%S type:203 res:%d", tid, _branch.c_str(), response.res_size());
        for (int i = 0; i < response.res_size(); ++i) {
            auto& res = response.res(i);
            services::ResourceDownloadRequest req;
            services::ResourceDownloadResponse resp;
            req.set_branch(_branch);
            req.add_res()->CopyFrom(res);
            DO_TILE_RPC(DownloadData, req, resp);
            if (resp.res_size() == 1 && resp.res(0).slots_size() == 1) {
                auto& data = resp.res(0).slots(0).data();
                std::stringstream ss(data);
                char buf[4096] = { 0 };
                while (ss.good()) {
                    ss.getline(buf, _countof(buf));
                    if (buf[0] == '#' || buf[0] == '0') {
                        continue;
                    }
                    double time_stamp = 0;
                    double lat = 0;
                    double lon = 0;
                    double alt = 0;
                    double R00 = 0;
                    double R01 = 0;
                    double R02 = 0;
                    double R10 = 0;
                    double R11 = 0;
                    double R12 = 0;
                    double R20 = 0;
                    double R21 = 0;
                    double R22 = 0;
                    char name[256] = { 0 };
                    if (sscanf(buf, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %s",
                        &time_stamp, &lat, &lon, &alt, &R00, &R01, &R02,
                        &R10, &R11, &R12, &R20, &R21, &R22, name) < 14) {
                        continue;
                    }
                    XLogInfo(XTAG(L"tile"), L"Make TilePosInfo for %S", buf);
                    TileImagePos img;
                    img.pos.res_id = res.id();
                    img.pos.index = (res.id().tile_id() * 10000000000L + res.id().trail_id() * 100000000L + res.id().index()) * 1000L + atoi(name);
                    img.pos.tile_id = res.id().tile_id();
                    img.pos.trail_id = res.id().trail_id();
                    img.pos.timestamp = time_stamp;
                    nuts::dpoint_t dp = { lon, lat };
                    nuts::dpoint_t op;
                    nuts::wgs2gcj(dp, &op);
                    img.pos.pos = { op.x, op.y, alt };
                    _proj.Wgs2TileLocal(_tile_id, img.pos.pos.x(), img.pos.pos.y(), img.pos.pos.z());

                    Eigen::Matrix3d mat;
                    mat << R00, R01, R02, R10, R11, R12, R20, R21, R22;
                    img.pos.quat = mat;
                    img.pos.affine_l_2_w = Eigen::Affine3d(img.pos.quat).pretranslate(img.pos.pos);
                    img.pos.affine_w_2_l = img.pos.affine_l_2_w.inverse();
                    img.res_id = res.id();
                    int64_t indimg = res.id().index() * 1000L + atoi(name);
                    if (ind2id.find(indimg) != ind2id.end()) {
                        img.res_id = ind2id[indimg];
                    }
                    else {
                        continue;
                    }
                    img.path = res.data().substr(res.data().find_last_of('/'));
                    img.path.resize(img.path.size() - 4);
                    img.path.push_back('/');
                    img.path.append(name);
                    img.index = img.pos.index;
                    infos.push_back(img);
                }
            }
        }
    }
    return true;
};

bool CTileServerDao::query_feature_rect(int trail_id, std::deque<Feature3DInfo>& infos) 
{
    for (auto tid : _tids) {
        services::ResourceQueryRequest request;
        services::ResourceQueryResponse response;
        request.set_tile_id(tid);
        if (trail_id > 0) {
            request.set_trail_id(trail_id);
        }
        request.add_type(301);
        request.set_branch(_branch);
        int64_t req_ver = 0;
        if (req_ver > 0) {
            request.set_version(req_ver);
        }
        double cpx = 0;
        double cpy = 0;
        _proj.GetTileCenter(_tile_id, cpx, cpy);
        DO_SIMPLE_TILE_RPC(QueryResource);
        XLogInfo(XTAG(L"tile"), L"query_image_rect() for tile:%d branch:%S type:301 res:%d", tid, _branch.c_str(), response.res_size());
        for (int i = 0; i < response.res_size(); ++i) {
            auto& res = response.res(i);
            services::ResourceDownloadRequest req;
            services::ResourceDownloadResponse resp;
            req.set_branch(_branch);
            req.add_res()->CopyFrom(res);
            DO_TILE_RPC(DownloadData, req, resp);
            if (resp.res_size() == 1 && resp.res(0).slots_size() == 1) {
                auto& data = resp.res(0).slots(0).data();
                Json::Reader reader;
                Json::Value root;
                reader.parse(data, root);
                auto fts = root["features"];
                for (int i = 0; i < fts.size(); ++i) {
                    auto ft = fts[i];
                    if (ft["type"].asString() != "feature") {
                        continue;
                    }
                    auto cds = ft["geometry"]["coordinates"];
                    if (cds.size() <= 0) {
                        continue;
                    }
                    Feature3DInfo feat;
                    feat.width = ft["properties"]["WIDTH"].asFloat();
                    auto type = ft["properties"]["LANE_EDGE_TYPE"].asString();
                    if (type == "SOLID") {
                        feat.type = 1;
                    }
                    else if (type == "DASHED") {
                        feat.type = 2;
                    }
                    for (int p = 0; p < cds.size(); ++p) {
                        auto cd = cds[p];
                        nuts::dpoint_t dp = { cd[0].asDouble(), cd[1].asDouble() };
                        nuts::dpoint_t op;
                        nuts::wgs2gcj(dp, &op);
                        feat.pts.push_back({ op.x, op.y, cd[2].asDouble() });
                    }
                    feat.pos = feat.pts[0];
                    infos.push_back(feat);
                    XLogInfo(XTAG(L"tile"), L"Make feature @ (%f, %f, %f)", feat.pos.x(), feat.pos.y(), feat.pos.z());
                }
            }
        }
    }
    return true;
}

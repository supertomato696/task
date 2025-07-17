#include "road_tile_dao.h"
#include <atomic>
#include <chrono>
#include <thread>
#include <fstream>

#ifdef WIN32
#include <windows.h>
#include <direct.h>
#undef min
#undef max
#include <sofa/pbrpc/pbrpc.h>
#else
#include <brpc/server.h>
#include <brpc/channel.h>
#include <brpc/controller.h>
#endif

#include "public/platform.h"
// #include "utils/log_util.h"

namespace data_access_engine
{
bool RoadTileDao::download_tiles_one_by_one(RoadTileDownloadParam& download_param,
                        int64_t& version, std::vector<services::TileInfo*>& tiles) {
    auto str_download_branch = _config->road_server_download_branch();
    // LOG_BEGIN(INFO) << "download_tiles_one_by_one() tile size = " << download_param.tile_15_ids.size()
    //     << " req_ver = " << download_param.req_ver << " version = " << version << " download branch = "
    //     << str_download_branch; LOG_END;
    auto tid2branch = _config->get_all_tileidbranches();
    int tile_index = 0;
    for (auto& tids : download_param.tile_15_ids) {
        if (tids.empty()) {
            continue;
        }
        DownLoadParam param;
        param.param = download_param;
        param.branch = str_download_branch;
        param.tile_index = tile_index;
        param.tiles = &tiles;
        int tile_15_id = tids.front();
        auto itr = tid2branch.find(tile_15_id);
        if (itr != tid2branch.end()) {
            if (!itr->second.road_server_branch.empty()) {
                param.branch = itr->second.road_server_branch;
            }
            if (!itr->second.road_server_version.empty()) {
                param.param.req_ver = atoll(itr->second.road_server_version.c_str());
            }
        }
        else if (str_download_branch.empty()) {
            bool bf = false;
            for (auto& mm : tid2branch) {
                if (mm.first / 16 == tile_15_id / 16) {
                    param.branch = mm.second.road_server_branch;
                    bf = true;
                    break;
                }
            }
            if (!bf && !tid2branch.empty()) {
                param.branch = tid2branch.begin()->second.road_server_branch;
            }
        }
        std::stringstream ss;
        for (size_t i = 0; i < tids.size(); i++) {
            ss << tids[i] << ',';
            if (i > 8) {
                ss << "...(" << tids.size() << ") ";
                break;
            }
        }
        int retry_num = 0;
        bool ret = false;
        while (retry_num < 3 && !ret) {
            ret = download_tiles_15(tids, param);
            ++retry_num;
            if (retry_num < 3 && !ret) {                
                // LOG_BEGIN(ERROR) << "download_tiles_one_by_one() failed to download tiles " << ss.str()
                //     << " sleep and retry " << retry_num; LOG_END;
                std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            }
        }
        if (retry_num >= 3 && !ret) {
            // LOG_BEGIN(ERROR) << "download_tiles_15() failed to download tiles " << ss.str()
            //     << " exit!"; LOG_END;
            return false;
        }
        download_param.pb_size += param.param.pb_size;
        // LOG_BEGIN(INFO) << "download_tiles_15() success to download tiles " << ss.str()
        //     << " with version " << param.version << " (" << tile_index << '/' 
        //     << download_param.tile_15_ids.size() << ") size:" << download_param.pb_size; LOG_END;
        version = param.version;
        ++tile_index;
    }
    return true;
}

bool RoadTileDao::download_tiles_parallel(RoadTileDownloadParam& download_param, int thread_num, 
                                          int64_t& ver, std::vector<services::TileInfo*>& tiles) {
    thread_num = std::min<int>(thread_num, (int)download_param.tile_15_ids.size());
    auto str_download_branch = _config->road_server_download_branch();
    // LOG_BEGIN(INFO) << "download_tiles_parallel() tile size = " << download_param.tile_15_ids.size()
    //     << " req_ver = " << download_param.req_ver << " thread_num = " << thread_num 
    //     << " download branch = " << str_download_branch; LOG_END;
    if (thread_num <= 1) {
        return download_tiles_one_by_one(download_param, ver, tiles);
    }
    std::vector<std::thread> threads;
    threads.reserve(thread_num);
    std::atomic<int> tile_ind(0);
    bool bsucc = true;
    for (int ti = 0; ti < thread_num; ++ti) {
        threads.push_back(std::thread([this, &tile_ind, &tiles, &ver, &bsucc, &download_param]() {
            while (true) {
                int ind = tile_ind.fetch_add(1);
                if (ind < (int)download_param.tile_15_ids.size()) {
                    DownLoadParam param;
                    param.param = download_param;
                    param.branch = _config->road_server_download_branch();
                    param.tile_index = ind;
                    param.tiles = &tiles;
                    auto& tids = download_param.tile_15_ids[ind];
                    std::stringstream ss;
                    for (size_t i = 0; i < tids.size(); i++) {
                        ss << tids[i] << ',';
                        if (i > 8) {
                            ss << "...(" << tids.size() << ") ";
                            break;
                        }
                    }
                    // LOG_BEGIN(INFO) << "download_tiles_parallel() begin download " << ind << "/"
                    //     << download_param.tile_15_ids.size() << " tiles:" << ss.str(); LOG_END;
                    int retry_num = 0;
                    bool ret = false;
                    while (retry_num < 3 && !ret){
                        ret = download_tiles_15(tids, param);
                        ++retry_num;
                        if (retry_num < 3 && !ret){
                            // LOG_BEGIN(ERROR) << "download_tiles_parallel() failed to download tiles " 
                            //     << ss.str() << " sleep and retry " << retry_num; LOG_END;
                            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                        }
                    }
                    if (retry_num >= 3 && !ret) {
                        bsucc = false;
                        // LOG_BEGIN(ERROR) << "download_tiles_parallel() failed to download tiles " 
                        //     << ss.str() << " exit thread!"; LOG_END;
                        return;
                    }
                    _road_tile_dao_mutex.lock();
                    ver = std::max(ver, param.version);
                    download_param.pb_size += param.param.pb_size;
                    _road_tile_dao_mutex.unlock(); 
                    // LOG_BEGIN(INFO) << "download_tiles_parallel() success to download tiles "
                    //     << ss.str() << " with version " << param.version << " (" << ind << '/'
                    //     << download_param.tile_15_ids.size() << ") size:" << download_param.pb_size; LOG_END;                   
                }
                else {
                    break;
                }
            }
        }));
    }
    for (int ti = 0; ti < thread_num; ++ti) {
        threads[ti].join();
    }
    // LOG_BEGIN(INFO) << "download_tiles_parallel() " << (bsucc ? "succeed" : "failed") << " with "
    //     << tiles.size() << " tiles. size:" << download_param.pb_size; LOG_END;
    return bsucc;
}

bool RoadTileDao::download_tiles_15(const std::vector<int>& tids, DownLoadParam& param) {
    services::TileDownloadRequest request;
    std::stringstream ss;
    for (int tid : tids) {        
        request.add_tile_id(tid);
    }
    for (size_t i = 0; i < tids.size(); i++) {
        ss << tids[i] << ',';
        if (i > 8) {
            ss << "...(" << tids.size() << ") ";
            break;
        }
    }
    if (!param.branch.empty()) {
        request.set_branch(param.branch);
    }
    if (param.param.req_ver > 0) {
        request.set_request_version(param.param.req_ver);
    }
    if (!param.param.editor_name.empty()) {
        request.set_editor(param.param.editor_name);
    }
    for (int type : param.param.req_types) {
        request.add_need_type(type);
    }
    request.set_load_ref(param.param.load_ref);

    services::TileDownloadResponse response;
    std::string str_road_server_address = _config->road_server_address();
    // LOG_BEGIN(INFO) << "download_tiles_15() begin download tiles " << ss.str()
    //     << " req_ver = " << param.param.req_ver << " branch = " << param.branch
    //     << " address = " << str_road_server_address; LOG_END;
#ifdef WIN32
    sofa::pbrpc::RpcClientOptions client_options;
    client_options.keep_alive_time = -1;
    sofa::pbrpc::RpcClient rpc_client(client_options);
    sofa::pbrpc::RpcChannelOptions channel_options;
    channel_options.connect_timeout = 600 * 1000;
    sofa::pbrpc::RpcChannel channel(&rpc_client, str_road_server_address, channel_options);
    services::RoadService_Stub stub(&channel);
    sofa::pbrpc::RpcController ctrl;
    ctrl.SetTimeout(600 * 1000);
    //ctrl.SetRequestCompressType(sofa::pbrpc::CompressTypeZlib);
    stub.DownloadTile(&ctrl, &request, &response, NULL);
#else
    brpc::Channel channel;
    brpc::ChannelOptions options;
    options.connect_timeout_ms = 600 * 1000;
    options.timeout_ms = 600 * 1000;
    channel.Init(str_road_server_address.c_str(), &options);
    services::RoadService_Stub stub(&channel);
    brpc::Controller ctrl;
    ctrl.set_timeout_ms(600 * 1000);
    //ctrl.set_request_compress_type(brpc::COMPRESS_TYPE_GZIP);
    stub.DownloadTile(&ctrl, &request, &response, NULL);
#endif

    if (ctrl.Failed()) {
        // LOG_BEGIN(ERROR) << "download_tiles_15() download tiles " << ss.str() << " failed :" 
        //     << ctrl.ErrorText().c_str(); LOG_END;
        return false;
    }
    if (response.has_ret() && response.ret().code() != 0) {
        // LOG_BEGIN(ERROR) << "download_tiles_15() download tiles " << ss.str() << " response "
        //     << response.ret().code() << ":" << response.ret().message(); LOG_END;
        return false;
    }
    _road_tile_dao_mutex.lock();
    (*param.tiles).reserve((*param.tiles).size() + response.tile_info_size() + 1);
    _road_tile_dao_mutex.unlock();
    param.version = response.editor_version();
    // LOG_BEGIN(INFO) << "download_tiles_15() download tiles " << ss.str() << " succeed"; LOG_END;
    while (response.tile_info_size() > 0) {
        auto tile = response.mutable_tile_info()->ReleaseLast();
        _road_tile_dao_mutex.lock();
        param.tiles->push_back(tile);
        _road_tile_dao_mutex.unlock();
    }
    // LOG_BEGIN(INFO) << "download_tiles_15() finish download tiles " << ss.str() 
    //     << " index:" << param.tile_index << " size:" << param.param.pb_size; LOG_END;
    return true;
}

bool RoadTileDao::merge_tiles(const std::vector<int>& tids, const std::vector<int>& types, const std::string& editor, int64_t req_ver, int64_t& ver) {
    // LOG_BEGIN(INFO) << "merge_tiles() tids_size " << tids.size() << "editor = " << editor 
    //     << "req_ver=" << req_ver << "src_branch = " << _config->road_server_download_branch() 
    //     << "dest_branch = " << _config->road_server_upload_branch() << "ver = " << ver; LOG_END;
    if (_config->road_server_download_branch() == _config->road_server_upload_branch()) {
        return false;
    }
    services::TileMergeRequest request;
    if (!editor.empty()) {
        request.set_editor(editor);
    }
    for (int t : types) {
        request.add_need_type(t);
    }
    std::stringstream ss;
    for (int tid : tids) {        
        request.add_tile_id(tid);
    }
    for (size_t i = 0; i < tids.size(); i++) {
        ss << tids[i] << ',';
        if (i > 8) {
            ss << "...(" << tids.size() << ") ";
            break;
        }
    }
    request.set_src_branch(_config->road_server_download_branch());
    request.set_dst_branch(_config->road_server_upload_branch());

    if (req_ver > 0) {
        request.set_request_version(req_ver);
    }

    services::TileMergeResponse response;
    std::string str_road_server_address = _config->road_server_address();
#ifdef WIN32
    sofa::pbrpc::RpcClientOptions client_options;
    client_options.keep_alive_time = -1;
    sofa::pbrpc::RpcClient rpc_client(client_options);
    sofa::pbrpc::RpcChannelOptions channel_options;
    channel_options.connect_timeout = 60 * 1000;
    sofa::pbrpc::RpcChannel channel(&rpc_client, str_road_server_address, channel_options);
    services::RoadService_Stub stub(&channel);
    sofa::pbrpc::RpcController ctrl;
    ctrl.SetTimeout(600 * 1000);
    //ctrl.SetRequestCompressType(sofa::pbrpc::CompressTypeZlib);
    stub.MergeTile(&ctrl, &request, &response, NULL);
#else
    brpc::Channel channel;
    brpc::ChannelOptions options;
    options.connect_timeout_ms = 600 * 1000;
    options.timeout_ms = 600 * 1000;
    channel.Init(str_road_server_address.c_str(), &options);
    services::RoadService_Stub stub(&channel);
    brpc::Controller ctrl;
    ctrl.set_timeout_ms(600 * 1000);
    //ctrl.set_request_compress_type(baidu::rpc::COMPRESS_TYPE_GZIP);
    stub.MergeTile(&ctrl, &request, &response, NULL);
#endif

    if (ctrl.Failed()) {
        // LOG_BEGIN(ERROR) << "merge_tiles() merge tiles " << ss.str() << "failed :"
        //     << ctrl.ErrorText().c_str(); LOG_END;
        return false;
    }
    if (response.has_ret() && response.ret().code() != 0) {
        // LOG_BEGIN(ERROR) << "merge_tiles() merge tiles " << ss.str() << " response "
        //     << response.ret().code() << ":" << response.ret().message(); LOG_END;
        return false;
    }
    ver = response.editor_version();

    // LOG_BEGIN(INFO) << "merge_tiles() succeed for tiles " << ss.str() 
    //     << "src=" << _config->road_server_download_branch() << ", dst="
    //     << _config->road_server_upload_branch(); LOG_END;
    return true;
}

bool RoadTileDao::revert_tiles(const std::vector<int>& tids, const std::vector<int>& types,
                                const std::string& editor, int64_t req_ver, int64_t& ver) {
    // LOG_BEGIN(INFO) << "revert_tiles() tids_size " << tids.size() << "editor = " << editor 
    //     <<  "req_ver=" << req_ver << "branch = " << _config->road_server_upload_branch()
    //     << "ver = " << ver; LOG_END;
    services::TileRevertRequest request;
    if (!editor.empty()) {
        request.set_editor(editor);
    }
    std::stringstream ss;
    for (int tid : tids) {
        request.add_tile_id(tid);
    }
    for (size_t i = 0; i < tids.size(); i++) {
        ss << tids[i] << ',';
        if (i > 8) {
            ss << "...(" << tids.size() << ") ";
            break;
        }
    }
    request.set_branch(_config->road_server_upload_branch());

    if (req_ver > 0) {
        request.set_request_version(req_ver);
    }

    services::TileRevertResponse response;
    std::string str_road_server_address = _config->road_server_address();
#ifdef WIN32
    sofa::pbrpc::RpcClientOptions client_options;
    client_options.keep_alive_time = -1;
    sofa::pbrpc::RpcClient rpc_client(client_options);
    sofa::pbrpc::RpcChannelOptions channel_options;
    channel_options.connect_timeout = 60 * 1000;
    sofa::pbrpc::RpcChannel channel(&rpc_client, str_road_server_address, channel_options);
    services::RoadService_Stub stub(&channel);
    sofa::pbrpc::RpcController ctrl;
    ctrl.SetTimeout(600 * 1000);
    //ctrl.SetRequestCompressType(sofa::pbrpc::CompressTypeZlib);
    stub.RevertTile(&ctrl, &request, &response, NULL);
#else
    brpc::Channel channel;
    brpc::ChannelOptions options;
    options.connect_timeout_ms = 600 * 1000;
    options.timeout_ms = 600 * 1000;
    channel.Init(str_road_server_address.c_str(), &options);
    services::RoadService_Stub stub(&channel);
    brpc::Controller ctrl;
    ctrl.set_timeout_ms(600 * 1000);
    //ctrl.set_request_compress_type(baidu::rpc::COMPRESS_TYPE_GZIP);
    stub.RevertTile(&ctrl, &request, &response, NULL);
#endif

    if (ctrl.Failed()) {
        // LOG_BEGIN(ERROR) << "revert_tiles() revert tiles " << ss.str() << "failed :"
        //     << ctrl.ErrorText().c_str(); LOG_END;
        return false;
    }
    if (response.has_ret() && response.ret().code() != 0) {
        // LOG_BEGIN(ERROR) << "revert_tiles() revert tiles " << ss.str() << " response "
        //     << response.ret().code() << ":" << response.ret().message(); LOG_END;
        return false;
    }
    ver = response.editor_version();

    // LOG_BEGIN(INFO) << "revert_tiles() succeed for tiles " << ss.str() 
    //     << "branch=" << _config->road_server_upload_branch(); LOG_END;
    return true;
}

bool RoadTileDao::upload_tile(const std::string& cur_editor, int64_t& version, 
                              const TileInfoList& tiles) {
    // CLOG_INFO("upload_tile() tiles size : %d , cur_editor : %s ,version: %d ",tiles.size(), cur_editor, version );
    // LOG_BEGIN(INFO) << "upload_tile() tiles.size() = " << tiles.size() << " cur_editor = " 
    //     << cur_editor << " version=" << version; LOG_END;
    services::TileUploadRequest request;
    if (!_config->road_server_upload_branch().empty()) {
        request.set_branch(_config->road_server_upload_branch());
    }
    if (!cur_editor.empty()) {
        request.set_editor(cur_editor);
    }
    
    for (auto& tile : tiles) {
        auto pt = request.add_tile();
        tile->to_message(pt);
        //LOG_BEGIN(INFO) << "upload_tile() convert TileInfoProxy " << tile->tile_id()
        //    << '@' << tile->version() << " to message, " << pt->ByteSize() << "Bytes"; LOG_END;
    }

    {
#ifdef WIN32
        std::string str_task_dir;
        {
            char* buffer = _getcwd(NULL, 0);
            str_task_dir = buffer;
        }
        struct tm stm;
        tm* p_tm = &stm;
        time_t timep;
        time(&timep);
        localtime_s(p_tm, &timep);
        char curtaskdir[1024] = { 0 }, dir[1024] = { 0 };
        sprintf_s(curtaskdir, "%s/upload_%d-%02d-%02d_%02d-%02d-%02d.pb", str_task_dir.c_str(), (int)p_tm->tm_year + 1900, (int)p_tm->tm_mon + 1, (int)p_tm->tm_mday,
            (int)p_tm->tm_hour, (int)p_tm->tm_min, (int)p_tm->tm_sec);
        _fullpath(dir, curtaskdir, 1024);
        std::ofstream stf(dir, std::ios::binary);
        request.SerializeToOstream(&stf);
        stf.close();
        /*std::ifstream fis(dir, std::ios::binary);
        services::TileUploadRequest rq;
        if (!rq.ParseFromIstream(&fis)) {
            rq.Clear();
            fis.seekg(std::ios::beg);
            rq.ParseFromIstream(&fis);
        }*/
#endif // WIN32
    }

    services::TileUploadResponse response;
    std::string str_road_server_address = _config->road_server_address();

#ifdef WIN32
    sofa::pbrpc::RpcClientOptions client_options;
    client_options.keep_alive_time = -1;
    sofa::pbrpc::RpcClient rpc_client(client_options);
    sofa::pbrpc::RpcChannelOptions channel_options;
    channel_options.connect_timeout = 600 * 1000;
    sofa::pbrpc::RpcChannel channel(&rpc_client, str_road_server_address, channel_options);
    services::RoadService_Stub stub(&channel);
    sofa::pbrpc::RpcController ctrl;
    ctrl.SetTimeout(600 * 1000);
    //ctrl.SetRequestCompressType(sofa::pbrpc::CompressTypeZlib);
    stub.UploadTile(&ctrl, &request, &response, NULL);
#else
    brpc::Channel channel;
    brpc::ChannelOptions options;
    options.connect_timeout_ms = 600 * 1000;
    options.timeout_ms = 600 * 1000;
    channel.Init(str_road_server_address.c_str(), &options);
    services::RoadService_Stub stub(&channel);
    brpc::Controller ctrl;
    ctrl.set_timeout_ms(600 * 1000);
    //ctrl.set_request_compress_type(brpc::COMPRESS_TYPE_GZIP);
    stub.UploadTile(&ctrl, &request, &response, NULL);
#endif

    if (ctrl.Failed()) {
        // LOG_BEGIN(ERROR) << "UploadTile() upload for tiles " << tiles.size() << " rpc failed:"
        //     << ctrl.ErrorText().c_str(); LOG_END;
        return false;
    }
    if (response.has_ret() && response.ret().code() != 0) {
        // LOG_BEGIN(ERROR) << "UploadTile() upload tiles " << tiles.size() << " response "
        //     << response.ret().code() << ":" << response.ret().message(); LOG_END;
        return false;
    }
    std::unordered_map<int, int64_t> id2ver(response.tile_size() * 2);
    for (int i = 0; i < response.tile_size(); ++i) {
        id2ver[response.tile(i).tile_id()] = response.tile(i).version();
    }

    int64_t ver = version;
    if (response.has_read_ver()) {
        version = response.read_ver();

        if (version <= ver) {
            version = ver + 1;
        }
    }
    else {
        ++version;
    }
    // CLOG_INFO("UploadTile() for tiles %d success with version %d ",tiles.size(), version);
    // LOG_BEGIN(INFO) << "UploadTile() for tiles " << tiles.size() << " success with version "
    //     << version; LOG_END;
    return true;
}

bool RoadTileDao::save_tiles(const TileInfoList& tiles, const std::string& editor, const int64_t version, const char* path)
{
    services::TileUploadRequest request;
    if (!editor.empty()) {
        request.set_editor(editor);
    }
    for (auto& tile : tiles) {        
        tile->to_message(request.add_tile());
    }
    // LOG_BEGIN(INFO) << "save_tiles() for " << tiles.size() << " tiles editor = "
    //     << editor << " version=" << version << " path=" << path; LOG_END;
    std::ofstream ost(path, std::ios::binary | std::ios::out);
    return request.SerializeToOstream(&ost) ? true : false;
}

bool RoadTileDao::load_tiles(const char* path, std::string& editor, TileInfoList& tiles)
{
    services::TileUploadRequest request;
    std::ifstream ist(path, std::ios::binary | std::ios::in);
    if (!request.ParseFromIstream(&ist)) {
        return false;
    }

    if (request.has_editor()) {
        editor = request.editor();
    }
    if (!_config->road_server_download_branch().empty()) {
        request.set_branch(_config->road_server_download_branch());
    }
    // LOG_BEGIN(INFO) << "load_tiles() from " << path; LOG_END;
    for (int i = 0; i < request.tile_size(); ++i) {
        auto tile = request.mutable_tile(i);
        std::shared_ptr<TileInfoProxy> pt(FeatureProxyBase::create_proxy<TileInfoProxy>());
        pt->relink_parent();
        pt->from_message(tile);
        tiles.push_back(pt);
        // LOG_BEGIN(INFO) << "load_tiles() construct TileInfoProxy " << pt->tile_id() << '@'
        //     << pt->version(); LOG_END;
    }
    return true;
}

bool RoadTileDao::fetch_tile_versions(const std::vector<int32_t>& tids, const std::string& editor,
        int64_t& read_ver, std::map<int32_t, int64_t>& tile_vers)
{
    services::TileDownloadRequest request;
    if (!_config->road_server_download_branch().empty()) {
        request.set_branch(_config->road_server_download_branch());
    }
    std::stringstream ss;
    for (int tid : tids) {
        request.add_tile_id(tid);
    }
    for (size_t i = 0; i < tids.size(); i++) {
        ss << tids[i] << ',';
        if (i > 8) {
            ss << "...(" << tids.size() << ") ";
            break;
        }
    }
    // LOG_BEGIN(INFO) << "fetch_tile_versions() for tiles " << ss.str() << "editor = " 
    //     << editor << "read_ver = " << read_ver; LOG_END;

    if (!editor.empty()) {
        request.set_editor(editor);
    }
    request.add_need_type(0);
    services::TileDownloadResponse response;
    std::string str_road_server_address = _config->road_server_address();

#ifdef WIN32
    sofa::pbrpc::RpcClientOptions client_options;
    client_options.keep_alive_time = -1;
    sofa::pbrpc::RpcClient rpc_client(client_options);
    sofa::pbrpc::RpcChannelOptions channel_options;
    channel_options.connect_timeout = 600;
    sofa::pbrpc::RpcChannel channel(&rpc_client, str_road_server_address, channel_options);
    services::RoadService_Stub stub(&channel);
    sofa::pbrpc::RpcController ctrl;
    ctrl.SetTimeout(600 * 1000);
    //ctrl.SetRequestCompressType(sofa::pbrpc::CompressTypeZlib);
    stub.DownloadTile(&ctrl, &request, &response, NULL);
#else
    brpc::Channel channel;
    brpc::ChannelOptions options;
    options.connect_timeout_ms = 600;
    options.timeout_ms = 60 * 1000;
    channel.Init(str_road_server_address.c_str(), &options);
    services::RoadService_Stub stub(&channel);
    brpc::Controller ctrl;
    ctrl.set_timeout_ms(60 * 1000);
    //ctrl.set_request_compress_type(brpc::COMPRESS_TYPE_GZIP);
    stub.DownloadTile(&ctrl, &request, &response, NULL);
#endif

    if (ctrl.Failed()) {
        // LOG_BEGIN(ERROR) << "fetch_tile_versions() failed :" << ctrl.ErrorText().c_str(); LOG_END;
        return false;
    }

    read_ver = response.editor_version();
    for (int i = 0; i < response.tile_info_size(); ++i) {
        auto ti = response.tile_info(i);
        tile_vers[ti.tile_id()] = ti.version();
    }
    // LOG_BEGIN(INFO) << "fetch_tile_versions() succeed with version " << read_ver; LOG_END;
    return true;
}

bool RoadTileDao::fetch_tile_versions(const std::map<int, std::vector<int32_t>>& map_15_tids, const std::string& editor,
        int64_t& read_ver, std::map<int32_t, int64_t> tile_vers)
{
    std::string str_road_server_address = _config->road_server_address();
    auto tid2branch = _config->get_all_tileidbranches();
    for (auto& m : map_15_tids){
        // LOG_BEGIN(INFO) << "fetch_tile_versions() for tile11 " << m.first << "editor = "
        //     << editor << "read_ver = " << read_ver; LOG_END;

        services::TileDownloadRequest request;
        services::TileDownloadResponse response;
        if (tid2branch.find(m.first) != tid2branch.end()){
            auto str = tid2branch[m.first];
            if (!str.road_server_branch.empty()){
                request.set_branch(str.road_server_branch);
            }
        }
        if (!_config->road_server_download_branch().empty()) {
            request.set_branch(_config->road_server_download_branch());
        }
        else{
            for (auto& mm : tid2branch){
                if (mm.first / 16 == m.first / 16){
                    request.set_branch(mm.second.road_server_branch);
                }
            }

            bool bf = false;
            for (auto& mm : tid2branch){
                if (mm.first / 16 == m.first / 16){
                    request.set_branch(mm.second.road_server_branch);
                    bf = true;
                    break;
                }
            }
            if (!bf && !tid2branch.empty()){
                request.set_branch(tid2branch.begin()->second.road_server_branch);
            }
        }

        for (auto tid : m.second) {
            request.add_tile_id(tid);
        }
        if (!editor.empty()) {
            request.set_editor(editor);
        }
        request.add_need_type(0);
#ifdef WIN32
        sofa::pbrpc::RpcClientOptions client_options;
        client_options.keep_alive_time = -1;
        sofa::pbrpc::RpcClient rpc_client(client_options);
        sofa::pbrpc::RpcChannelOptions channel_options;
        channel_options.connect_timeout = 600;
        sofa::pbrpc::RpcChannel channel(&rpc_client, str_road_server_address, channel_options);
        services::RoadService_Stub stub(&channel);
        sofa::pbrpc::RpcController ctrl;
        ctrl.SetTimeout(600 * 1000);
        //ctrl.SetRequestCompressType(sofa::pbrpc::CompressTypeZlib);
        stub.DownloadTile(&ctrl, &request, &response, NULL);
#else
        brpc::Channel channel;
        brpc::ChannelOptions options;
        options.connect_timeout_ms = 600;
        options.timeout_ms = 60 * 1000;
        channel.Init(str_road_server_address.c_str(), &options);
        services::RoadService_Stub stub(&channel);
        brpc::Controller ctrl;
        ctrl.set_timeout_ms(600 * 1000);
        //ctrl.set_request_compress_type(brpc::COMPRESS_TYPE_GZIP);
        stub.DownloadTile(&ctrl, &request, &response, NULL);
#endif

        if (ctrl.Failed()) {
            // LOG_BEGIN(ERROR) << "fetch_tile_versions() failed " << ctrl.ErrorCode() << ": " << ctrl.ErrorText(); LOG_END;
            return false;
        }

        read_ver = response.editor_version();
        // LOG_BEGIN(INFO) << "fetch_tile_versions() for tile11 " << m.first << " succeed, read_ver=" << read_ver; LOG_END;
        for (int i = 0; i < response.tile_info_size(); ++i) {
            auto ti = response.tile_info(i);
            tile_vers[ti.tile_id()] = ti.version();
        }
    }

    return true;
}

bool RoadTileDao::commit_change(TileInfoList& tiles, std::string& editor, int64_t& cur_ver)
{
    /*LOG_BEGIN(INFO) << "commit_change() for tiles_size = " << tiles.size() << " editor = " 
        << editor << "cur_ver = " << cur_ver; LOG_END;
    hadmap::TileCommitRequest request;
    if (!_config->road_server_upload_branch().empty()) {
        request.set_branch(_config->road_server_upload_branch());
    }
    if (!editor.empty()) {
        request.set_editor(editor);
    }
    request.set_version(cur_ver);
    for (auto& tile : tiles) {
        auto tp = request.add_tile();
        tile->to_message(tp);
    }

    hadmap::TileCommitResponse response;
    std::string str_road_server_address = _config->road_server_address();

#ifdef WIN32
    sofa::pbrpc::RpcClientOptions client_options;
    client_options.keep_alive_time = -1;
    sofa::pbrpc::RpcClient rpc_client(client_options);
    sofa::pbrpc::RpcChannelOptions channel_options;
    channel_options.connect_timeout = 600;
    sofa::pbrpc::RpcChannel channel(&rpc_client, str_road_server_address, channel_options);
    hadmap::RoadService_Stub stub(&channel);
    sofa::pbrpc::RpcController ctrl;
    ctrl.SetTimeout(600 * 1000);
    //ctrl.SetRequestCompressType(sofa::pbrpc::CompressTypeZlib);
    stub.CommitTileChange(&ctrl, &request, &response, NULL);
#else
    baidu::rpc::Channel channel;
    baidu::rpc::ChannelOptions options;
    options.connect_timeout_ms = 600;
    options.timeout_ms = 60 * 1000;
    channel.Init(str_road_server_address.c_str(), &options);
    hadmap::RoadService_Stub stub(&channel);
    baidu::rpc::Controller ctrl;
    ctrl.set_timeout_ms(600 * 1000);
    //ctrl.set_request_compress_type(baidu::rpc::COMPRESS_TYPE_GZIP);
    stub.CommitTileChange(&ctrl, &request, &response, NULL);
#endif

    if (ctrl.Failed()) {
        LOG_BEGIN(ERROR) << "commit_change() failed :" << ctrl.ErrorText().c_str(); LOG_END;
        return false;
    }
    cur_ver = response.version();
    LOG_BEGIN(INFO) << "commit_change() succeed with version " << cur_ver; LOG_END;*/
    return true;
}

}; // data_access_engine ;

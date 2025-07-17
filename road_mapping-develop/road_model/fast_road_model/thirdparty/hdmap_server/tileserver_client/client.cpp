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
#include <thread>
#include <experimental/filesystem>
#include <gflags/gflags.h>
#include <butil/logging.h>
#include <butil/time.h>
#include <brpc/channel.h>
#include "tileserver.pb.h"

DEFINE_string(branch, "test_pp", "branch");
DEFINE_string(dst_branch, "test_dst", "merge branch");
DEFINE_int32(tileid, 1, "tileid");
DEFINE_int32(type, 0, "type");
DEFINE_string(extmap, ".log:101;.timestamps:102;.h264:103;.json:201;.jpg:202;.gps:203;", "file extension to type");
DEFINE_string(filemap, "HD_LANE_EDGE.json:301;HD_TOPO.json:302;points3D.las:204;", "file name mapping to type");
DEFINE_int32(method, 1, "1 for upload, 2 for query, 3 for download, 4 for merge, 5 for revert");
DEFINE_string(server, /*"127.0.0.1:8082"*/"172.21.207.124:8082", "IP Address of server");
DEFINE_int32(timeout_ms, 100000, "RPC timeout in milliseconds");
DEFINE_int32(max_retry, 10, "Max retries(not including the first RPC)");
DEFINE_int32(interval_ms, 1000, "Milliseconds between consecutive requests");
DEFINE_int64(reqver, 0, "max version retrieved");
DEFINE_string(basedir, "/mnt/d/Temp/20kmneg/", "base dir for download/upload files");
DEFINE_int32(thread, 1, "concurrent thread");

namespace brpc {
DECLARE_uint64(max_body_size);
}

int main(int argc, char *argv[]) {
    GFLAGS_NS::ParseCommandLineFlags(&argc, &argv, true);

    brpc::FLAGS_max_body_size = 256 * 1024 * 1024;
    
    if (FLAGS_method > 1) {
        services::ResourceQueryRequest request;
        if (FLAGS_type > 0) {
            request.add_type(FLAGS_type);
        }
        else {
            const char* ptr = FLAGS_extmap.c_str();            
            while (ptr && *ptr) {
                const char* sep = strchr(ptr, ':');
                const char* edp = strchr(ptr, ';');
                if (sep && edp && sep + 1 < edp) {
                    int type = atoi(sep + 1);
                    if (type > 0) {
                        request.add_type(type);
                    }
                }
                else {
                    break;
                }
                ptr = edp + 1;
            }
            ptr = FLAGS_filemap.c_str();
            while (ptr && *ptr) {
                const char* sep = strchr(ptr, ':');
                const char* edp = strchr(ptr, ';');
                if (sep && edp && sep + 1 < edp) {
                    int type = atoi(sep + 1);
                    if (type > 0) {
                        request.add_type(type);
                    }
                }
                else {
                    break;
                }
                ptr = edp + 1;
            }
        }
        request.set_tile_id(FLAGS_tileid);
        request.set_branch(FLAGS_branch.c_str());
        if (FLAGS_reqver > 0) {
            request.set_version(FLAGS_reqver);
        }
        if (FLAGS_method == 4) {
            services::ResourceMergeRequest req;
            req.set_tile_id(request.tile_id());
            req.mutable_type()->CopyFrom(request.type());
            if (request.has_index()) {
                req.set_index(request.index());
            }
            if (request.has_trail_id()) {
                req.set_trail_id(request.trail_id());
            }
            if (request.has_version()) {
                req.set_version(request.version());
            }
            req.set_src_branch(FLAGS_branch);
            req.set_dst_branch(FLAGS_dst_branch);
            LOG(INFO) << "MergeResource() request: " << req.ShortDebugString();
            services::ResourceMergeResponse response;
            brpc::Channel channel;
            brpc::ChannelOptions options;
            options.protocol = "baidu_std";
            options.connection_type = "";
            options.timeout_ms = FLAGS_timeout_ms /*milliseconds*/;
            options.connect_timeout_ms = FLAGS_timeout_ms;
            options.max_retry = FLAGS_max_retry;
            if (channel.Init(FLAGS_server.c_str(), "", &options) != 0) {
                LOG(ERROR) << "Fail to initialize channel";
                return -1;
            }

            services::TileService_Stub stub(&channel);
            brpc::Controller cntl;
            stub.MergeResource(&cntl, &req, &response, nullptr);
            if (cntl.Failed()) {
                LOG(INFO) << "MergeResource() response from " << cntl.remote_side()
                        << " to " << cntl.local_side()
                        << ": " << response.ret().message() << " (code="
                        << response.ret().code() << ")"
                        << " latency=" << cntl.latency_us() << "us";
            }
            else {
                LOG(INFO) << "MergeResource() response " << response.DebugString();
            }
            return 0;
        }
        else if (FLAGS_method == 5) {
            services::ResourceRevertRequest req;
            req.set_tile_id(request.tile_id());
            req.mutable_type()->CopyFrom(request.type());
            if (request.has_index()) {
                req.set_index(request.index());
            }
            if (request.has_trail_id()) {
                req.set_trail_id(request.trail_id());
            }
            if (request.has_version()) {
                req.set_version(request.version());
            }
            req.set_branch(FLAGS_branch);
            LOG(INFO) << "RevertResource() request: " << req.ShortDebugString();
            services::ResourceRevertResponse response;
            brpc::Channel channel;
            brpc::ChannelOptions options;
            options.protocol = "baidu_std";
            options.connection_type = "";
            options.timeout_ms = FLAGS_timeout_ms /*milliseconds*/;
            options.connect_timeout_ms = FLAGS_timeout_ms;
            options.max_retry = FLAGS_max_retry;
            if (channel.Init(FLAGS_server.c_str(), "", &options) != 0) {
                LOG(ERROR) << "Fail to initialize channel";
                return -1;
            }

            services::TileService_Stub stub(&channel);
            brpc::Controller cntl;
            stub.RevertResource(&cntl, &req, &response, nullptr);
            if (cntl.Failed()) {
                LOG(INFO) << "RevertResource() response from " << cntl.remote_side()
                        << " to " << cntl.local_side()
                        << ": " << response.ret().message() << " (code="
                        << response.ret().code() << ")"
                        << " latency=" << cntl.latency_us() << "us";
            }
            else {
                LOG(INFO) << "RevertResource() response " << response.DebugString();
            }
            return 0;
        }
        services::ResourceQueryResponse response;
        LOG(INFO) << "QueryResource() request: " << request.ShortDebugString();
        brpc::Channel channel;
        brpc::ChannelOptions options;
        options.protocol = "baidu_std";
        options.connection_type = "";
        options.timeout_ms = FLAGS_timeout_ms /*milliseconds*/;
        options.connect_timeout_ms = FLAGS_timeout_ms;
        options.max_retry = FLAGS_max_retry;
        if (channel.Init(FLAGS_server.c_str(), "", &options) != 0) {
            LOG(ERROR) << "Fail to initialize channel";
            return -1;
        }

        services::TileService_Stub stub(&channel);
        brpc::Controller cntl;
        stub.QueryResource(&cntl, &request, &response, nullptr);
        if (!cntl.Failed()) {
            LOG(INFO) << "QueryResource() response from " << cntl.remote_side()
                      << " to " << cntl.local_side()
                      << ": " << response.ret().message() << " (code="
                      << response.ret().code() << ")"
                      << " latency=" << cntl.latency_us() << "us";
            for (int i = 0 ; i < response.res_size(); ++i) {
                auto& res = response.res(i);
                LOG(INFO) << "Found res " << res.id().ShortDebugString() << " : " << res.data();
            }
            if (FLAGS_method > 2) {
                std::vector<std::thread> workers(FLAGS_thread);
                std::atomic<int> index(0);
                for (int i = 0; i < FLAGS_thread; i++) {
                    std::thread([&index, &response, &options]() {
                        while (true) {
                            int i = index.fetch_add(1);
                            if (i >= response.res_size()) {
                                break;
                            }
                            auto& res = response.res(i);
                            if (res.slots_size() != 1) {
                                continue;
                            }                            
                            std::experimental::filesystem::path p(FLAGS_basedir + res.data());
                            std::experimental::filesystem::create_directories(p.parent_path());
                            services::ResourceDownloadRequest req;
                            req.add_res()->CopyFrom(res);
                            req.set_branch(FLAGS_branch.c_str());
                            services::ResourceDownloadResponse rp;
                            for (int retry_cnt = 0; retry_cnt < FLAGS_max_retry * 10; retry_cnt++) {
                                brpc::Channel channel;
                                if (channel.Init(FLAGS_server.c_str(), "", &options) != 0) {
                                    LOG(ERROR) << "Fail to initialize channel";
                                    std::this_thread::sleep_for(std::chrono::seconds(retry_cnt));
                                    continue;
                                }
                                services::TileService_Stub stub(&channel);
                                brpc::Controller cntl;
                                LOG(INFO) << "DownloadData() for " << res.id().ShortDebugString() << " with " << res.slots(0).name();                            
                                stub.DownloadData(&cntl, &req, &rp, nullptr);
                                if (!cntl.Failed()) {
                                    if (rp.res_size() == 1 && rp.res(0).slots_size() == 1) {
                                        std::ofstream ofs(FLAGS_basedir + res.data(), std::ios::binary);
                                        auto& dat  = rp.res(0).slots(0).data();
                                        ofs.write(dat.c_str(), dat.size());
                                        ofs.close();
                                        LOG(INFO) << "Write file " << FLAGS_basedir + res.data() << " with " << dat.size() << " Bytes";
                                    }
                                    else {
                                        LOG(WARNING) << "DownloadData() ret: " << rp.ret().message();
                                    }
                                    break;
                                }
                                else {
                                    LOG(WARNING) << retry_cnt << ". Failed to DownloadData() " << cntl.ErrorText();
                                    std::this_thread::sleep_for(std::chrono::seconds(retry_cnt));
                                }
                            }
                        }
                    }).swap(workers[i]);
                }
                for (int i = 0; i < FLAGS_thread; i++) {
                    workers[i].join();
                }
            }
        }
        else {
            LOG(WARNING) << "Failed to QueryResource() " << cntl.ErrorText();
        }
    }
    else {
        std::map<std::string, int> ext2type;
        {
            const char* ptr = FLAGS_extmap.c_str();            
            while (ptr && *ptr) {
                const char* sep = strchr(ptr, ':');
                const char* edp = strchr(ptr, ';');
                if (sep && edp && sep + 1 < edp) {
                    int type = atoi(sep + 1);
                    if (type > 0) {
                        ext2type[std::string(ptr, sep)] = type;
                    }
                }
                else {
                    break;
                }
                ptr = edp + 1;
            }
        }
        std::map<std::string, int> file2type;
        {
            const char* ptr = FLAGS_filemap.c_str();            
            while (ptr && *ptr) {
                const char* sep = strchr(ptr, ':');
                const char* edp = strchr(ptr, ';');
                if (sep && edp && sep + 1 < edp) {
                    int type = atoi(sep + 1);
                    if (type > 0) {
                        file2type[std::string(ptr, sep)] = type;
                    }
                }
                else {
                    break;
                }
                ptr = edp + 1;
            }
        }
        std::experimental::filesystem::recursive_directory_iterator fit(FLAGS_basedir.c_str());
        std::experimental::filesystem::recursive_directory_iterator fed;
        std::map<int, std::vector<std::experimental::filesystem::path>> tid2file;
        std::experimental::filesystem::path bp(FLAGS_basedir);
        for (; fit != fed; ++fit) {
            auto& p = fit->path();
            if (file2type.find(p.filename()) != file2type.end() || 
                    (p.has_extension() && ext2type.find(p.extension()) != ext2type.end())) {
                std::string ns = p.parent_path().c_str();
                std::string ps = bp.parent_path().c_str();
                auto chr = ns.c_str() + ps.size() + 1;
                while (*chr == '0') {
                    chr++;
                }
                int tid = atoi(chr);
                if (tid > 0) {
                    tid2file[tid].push_back(p);
                    auto pt = strchr(chr, '/');
                    LOG(INFO) << "Found file " << p.filename() << " @ tile " << tid << " from " << pt;
                }
            }
        }
        std::vector<int> tids;
        tids.reserve(tid2file.size());
        for (auto it = tid2file.begin(); it != tid2file.end(); ++it) {
            tids.push_back(it->first);
        }
        std::atomic<int> index(0);
        std::vector<std::thread> workers(FLAGS_thread);
        for (int i = 0; i < FLAGS_thread; ++i) {
            std::thread([&tid2file, &tids, &index, &ext2type, &file2type]() {
                brpc::ChannelOptions options;
                options.protocol = "baidu_std";
                options.connection_type = "";
                options.timeout_ms = FLAGS_timeout_ms /*milliseconds*/;
                options.connect_timeout_ms = FLAGS_timeout_ms;
                options.max_retry = FLAGS_max_retry;
                
                int fileind = 1;
                std::experimental::filesystem::path bp(FLAGS_basedir);                
                while (true) {
                    int i = index.fetch_add(1);
                    if (i >= (int)tids.size()) {
                        break;
                    }
                    for (auto& p : tid2file[tids[i]]) {
                        std::ifstream ifs(p, std::ios::binary);
                        services::ResourceUploadRequest req;
                        services::ResourceUploadResponse res;
                        req.set_branch(FLAGS_branch);
                        auto r = req.add_res();
                        auto id = r->mutable_id();
                        id->set_tile_id(tids[i]);
                        std::string fn = p.filename();
                        int index = 0;
                        int date = 0;
                        int pos = 0;
                        char sep[256] = {0};
                        if (file2type.find(fn) != file2type.end()) {
                            id->set_index(fileind++);
                            id->set_type(file2type[fn]);
                        }
                        else if (sscanf(fn.c_str(), "%d_%d_%[^_]_%d", &date, &index, sep, &pos) >= 3) {
                            id->set_index(date * 100 + index);
                            id->set_trail_id(pos);                        
                            id->set_type(ext2type[p.extension()]);
                        }
                        else {
                            int ind = atoi(fn.c_str());
                            std::string dir = p.parent_path().c_str();
                            auto str = strrchr(dir.c_str(), '/');
                            if (sscanf(str + 1, "%d_%d_%[^_]_%d", &date, &index, sep, &pos) < 3) {
                                continue;
                            }
                            id->set_index((date * 100L + index) * 1000L + ind);
                            id->set_trail_id(pos);                        
                            id->set_type(ext2type[p.extension()]);
                        }
                        
                        id->set_version(0);                        
                        std::string ns = p.c_str();
                        std::string ps = bp.parent_path().c_str();
                        auto chr = ns.c_str() + ps.size();
                        r->set_data(chr);
                        auto slot = r->add_slots();
                        slot->set_name("file");
                        slot->set_data({std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>()});
                        
                        for (int retry_cnt = 0; retry_cnt < FLAGS_max_retry * 10; retry_cnt++) {
                            brpc::Channel channel;
                            if (channel.Init(FLAGS_server.c_str(), "", &options) != 0) {
                                LOG(ERROR) << "Fail to initialize channel";
                                std::this_thread::sleep_for(std::chrono::seconds(retry_cnt));
                                continue;
                            }

                            services::TileService_Stub stub(&channel);
                            brpc::Controller cntl;
                            LOG(INFO) << "UploadResource() " << id->ShortDebugString() << " @ " << r->data();stub.UploadResource(&cntl, &req, &res, nullptr);
                            if (!cntl.Failed()) {
                                if (res.ret().code() == 0 && res.id_size() == 1) {
                                    LOG(INFO) << "UploadResource() " << res.id(0).ShortDebugString() << " @ " << r->data() << " for " << slot->data().size() << " Bytes";
                                }
                                else {
                                    LOG(INFO) << "UploadResource() " << id->ShortDebugString() << " @ " << r->data() << " failed: " << res.ret().message();
                                }
                                break;
                            }
                            else {
                                LOG(WARNING) << retry_cnt << ". Failed to UploadResource() " << cntl.ErrorText();
                                std::this_thread::sleep_for(std::chrono::seconds(retry_cnt));
                            }
                        }
                    }
                }
            }).swap(workers[i]);
        }
        for (int i = 0; i < FLAGS_thread; i++) {
            workers[i].join();
        }
    }

    LOG(INFO) << "tileserver_client is going to quit";
    return 0;
}

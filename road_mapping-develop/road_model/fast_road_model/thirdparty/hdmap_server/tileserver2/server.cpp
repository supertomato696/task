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

// A server to receive EchoRequest and send back EchoResponse.

#include <gflags/gflags.h>
#include <butil/logging.h>
#include <brpc/server.h>
#include <butil/time.h>
#include <brpc/channel.h>
#include <string>
#include <fstream>
#include <experimental/filesystem>

#include "services/tileserver.pb.h"

DEFINE_int32(timeout_ms, 100000, "RPC timeout in milliseconds");
DEFINE_int32(max_retry, 10, "Max retries(not including the first RPC)");
DEFINE_int32(interval_ms, 1000, "Milliseconds between consecutive requests");
DEFINE_int64(reqver, 0, "max version retrieved");
DEFINE_int64(bucket_num, 256, "divide data to those bucket by it's tileid");
DEFINE_string(bucket_path, "/biyadi/pfs/hdmap/tiledata/", "comma divided storage path suffix for buckets");

DEFINE_int32(port, 8082, "TCP Port of this server");
DEFINE_int32(idle_timeout_s, -1, "Connection will be closed if there is no "
             "read/write operations during the last `idle_timeout_s'");
DEFINE_int32(logoff_ms, 2000, "Maximum duration of server's LOGOFF state "
             "(waiting for client to close connection before server stops)");

namespace brpc {
DECLARE_uint64(max_body_size);
DECLARE_int64(socket_max_unwritten_bytes);
}

namespace services {

namespace fs = std::experimental::filesystem;

class TileServiceImpl : public TileService {
    std::vector<std::string> _bucket_paths;

    const std::string& get_bucket_by_tileid(int tid) {
        int bucket = tid % FLAGS_bucket_num;
        return _bucket_paths[bucket % _bucket_paths.size()];
    }
    std::string id_to_path(const std::string& branch, const services::TileResourceID& id, int64_t ts) {
        if (id.version() > 0 && id.version() <= ts) {
            ts = id.version();
        }
        char buf[4096];
        if (id.has_trail_id()) {
            snprintf(buf, sizeof(buf), "%s/%d/%d/@%ld/%016lx/%ld.@", branch.c_str(), id.tile_id(), id.type(), id.trail_id(), id.index(), ts);
        }
        else {
            snprintf(buf, sizeof(buf), "%s/%d/%d/%016lx/%ld.@", branch.c_str(), id.tile_id(), id.type(), id.index(), ts);
        }
        return buf;        
    }
    std::string id_to_slot_path(const std::string& branch, const std::string& slot, const services::TileResourceID& id, int64_t ts) {
        if (id.version() > 0 && id.version() <= ts) {
            ts = id.version();
        }
        char buf[4096];
        if (id.has_trail_id()) {
            snprintf(buf, sizeof(buf), "%s/%d/%d/@%ld/%016lx/%ld.%s", branch.c_str(), id.tile_id(), id.type(), id.trail_id(), id.index(), ts, slot.c_str());
        }
        else {
            snprintf(buf, sizeof(buf), "%s/%d/%d/%016lx/%ld.%s", branch.c_str(), id.tile_id(), id.type(), id.index(), ts, slot.c_str());
        }
        return buf;
    }

public:
    TileServiceImpl() {
        _bucket_paths.reserve(16);
        const char* ptr = FLAGS_bucket_path.c_str();
        while (ptr && *ptr) {
            const char* edp = strchr(ptr, ',');
            if (edp && ptr + 1 < edp) {
                std::string p(ptr, edp);
                fs::path path(p);
                if (!fs::exists(path)) {
                    if (!fs::create_directories(path)) {
                        LOG(FATAL) << "Fail to create bucket path " << path;
                        exit(-1);
                    }
                }
                _bucket_paths.push_back(p);
            }
            else if (ptr && *ptr) {
                fs::path path(ptr);
                if (!fs::exists(path)) {
                    if (!fs::create_directories(path)) {
                        LOG(FATAL) << "Fail to create bucket path " << path;
                        exit(-1);
                    }
                }
                _bucket_paths.push_back(ptr);
                break;
            }
            ptr = edp + 1;
        }
        if (_bucket_paths.empty()) {
            LOG(FATAL) << "No bucket path has been setted";
            exit(-1);
        }
    }
    virtual ~TileServiceImpl() {}
    virtual void UploadResource(::google::protobuf::RpcController* controller,
                       const ::services::ResourceUploadRequest* request,
                       ::services::ResourceUploadResponse* response,
                       ::google::protobuf::Closure* done) {
        brpc::ClosureGuard done_guard(done);
        brpc::Controller* cntl = static_cast<brpc::Controller*>(controller);
        LOG(INFO) << "Received UploadResource() request[log_id=" << cntl->log_id() 
                  << "] @ " << request->branch() << " with " << request->res_size() 
                  << " res from " << cntl->remote_side() << " to " << cntl->local_side();
        butil::Timer tm;
        tm.start();
        int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>
                            (std::chrono::system_clock::now().time_since_epoch()).count();
        int ret_code = 0;
        std::stringstream ret_msg;            
        for (auto& res : *(((::services::ResourceDownloadRequest*)request)->mutable_res())) {
            std::string data_str = get_bucket_by_tileid(res.id().tile_id()) + id_to_path(request->branch(), res.id(), timestamp);
            fs::path p(data_str);
            std::error_code err_code;
            fs::create_directories(p.parent_path(), err_code);
            if (err_code.value() != 0) {
                LOG(ERROR) << "Fail to create slot path " << p.c_str() << " : " << err_code.message();
                ret_msg << "Fail to create slot path " << p.c_str() << " : " << err_code.message() << std::endl;
                ret_code++;
                continue;
            }
            for (auto& slot : *res.mutable_slots()) {
                std::string slot_str = get_bucket_by_tileid(res.id().tile_id()) + id_to_slot_path(request->branch(), slot.name(), res.id(), timestamp);
                std::ofstream ofs(slot_str, std::ios::binary);
                if (!ofs.write(slot.data().c_str(), slot.data().size()).good()) {
                    LOG(ERROR) << "Fail to write slot file " << slot_str << " with " << slot.data().size() << " Bytes";
                    ret_msg << "Fail to write slot file " << slot_str << " with " << slot.data().size() << " Bytes" << std::endl;
                    ret_code++;                    
                }
                slot.clear_data();
            }
            if (res.id().version() <= 0 || res.id().version() > timestamp) {
                res.mutable_id()->set_version(timestamp);
            }
            auto data = res.SerializeAsString();
            std::ofstream ofs(data_str, std::ios::binary);
            if (!ofs.write(data.c_str(), data.size()).good()) {
                LOG(ERROR) << "Fail to write res file " << data_str << " with " << data.size() << " Bytes";
                ret_msg << "Fail to write slot file " << data_str << " with " << data.size() << " Bytes" << std::endl;
                ret_code++;                    
                continue;
            }
            response->add_id()->CopyFrom(res.id());
        }
        response->mutable_ret()->set_code(ret_code);
        response->mutable_ret()->set_message(ret_msg.str());
        response->set_version(timestamp);
        tm.stop();
        LOG(INFO) << "Finish UploadResource() request[log_id=" << cntl->log_id() 
                  << "] @ " << request->branch() << " with " << response->id_size() 
                  << " res from " << cntl->remote_side() << " to " << cntl->local_side()
                  << ", " << tm.m_elapsed() << " ms";
    }

    virtual void QueryResource(::google::protobuf::RpcController* controller,
                       const ::services::ResourceQueryRequest* request,
                       ::services::ResourceQueryResponse* response,
                       ::google::protobuf::Closure* done) {
        brpc::ClosureGuard done_guard(done);
        brpc::Controller* cntl = static_cast<brpc::Controller*>(controller);
        LOG(INFO) << "Received QueryResource() request[log_id=" << cntl->log_id() 
                  << "] " << request->tile_id() << " @ " << request->branch() 
                  << "from " << cntl->remote_side() << " to " << cntl->local_side();
        butil::Timer tm;
        tm.start();
        std::stringstream ret_msg;
        int ret_code = 0;
        std::map<std::string, int64_t> elem2time;
        for (auto type : request->type()) {
            std::string base = get_bucket_by_tileid(request->tile_id());
            base.reserve(4096);
            base.append(request->branch());
            base.push_back('/');
            base.append(std::to_string(request->tile_id()));
            base.push_back('/');
            base.append(std::to_string(type));
            base.push_back('/');
            if (request->has_trail_id()) {
                base.push_back('@');
                base.append(std::to_string(request->trail_id()));
                base.push_back('/');
            }
            if (request->has_index()) {
                char buf[32];
                snprintf(buf, sizeof(buf), "%016lx", request->index());
                base.append(buf);
                base.push_back('/');                
            }
            if (!fs::exists(base)) {
                continue;
            }
            LOG(INFO) << "QueryResource() iterated on path " << base;
            for (auto& p : fs::recursive_directory_iterator(base)) {
                if (p.path().extension() != ".@") {
                    continue;
                }
                int64_t ver = atoll(p.path().filename().c_str());
                if (request->has_version() && ver > request->version()) {
                    continue;
                }
                auto ps = p.path().parent_path().generic_string();
                auto it = elem2time.find(ps);
                if (it == elem2time.end()) {
                    elem2time[ps] = ver;
                }
                else if (it->second < ver) {
                    it->second = ver;
                }
            }
        }
        for (auto& eit : elem2time) {
            std::string path = eit.first + '/' + std::to_string(eit.second) + ".@";
            std::ifstream ifs(path, std::ios::binary);
            auto res = response->add_res();
            if (!res->ParseFromIstream(&ifs)) {
                LOG(ERROR) << "Fail to load res " << path;
                ret_msg << "Fail to load res " << path << std::endl;
                ret_code++;
                response->mutable_res()->ReleaseLast();
                continue;
            }
            if (res->id().has_is_deleted() && res->id().is_deleted()) {
                response->mutable_res()->ReleaseLast();
                LOG(INFO) << "Skip deleted res " << path;
            }
        }
        response->mutable_ret()->set_code(ret_code);
        response->mutable_ret()->set_message(ret_msg.str());
        tm.stop();
        LOG(INFO) << "Finish QueryResource() request[log_id=" << cntl->log_id() 
                  << "] " << request->tile_id() << " @ " << request->branch() 
                  << " get " << response->res_size() << " res "
                  << "from " << cntl->remote_side() << " to " << cntl->local_side()
                  << ", " << tm.m_elapsed() << " ms";
    }

    virtual void DownloadData(::google::protobuf::RpcController* controller,
                       const ::services::ResourceDownloadRequest* request,
                       ::services::ResourceDownloadResponse* response,
                       ::google::protobuf::Closure* done) {
        brpc::ClosureGuard done_guard(done);
        brpc::Controller* cntl = static_cast<brpc::Controller*>(controller);
        LOG(INFO) << "Received DownloadData() request[log_id=" << cntl->log_id() 
                  << "] " << request->res_size() << " res @ " << request->branch() 
                  << "from " << cntl->remote_side() << " to " << cntl->local_side();
        butil::Timer tm;
        tm.start();
        std::stringstream ret_msg;
        int ret_code = 0;
        for (auto& res : request->res()) {
            if (!request->has_branch() || !res.id().has_tile_id() || !res.id().has_type() || !res.id().has_index()) {
                continue;
            }
            auto dst = response->add_res();
            dst->CopyFrom(res);
            dst->clear_slots();
            dst->mutable_id()->set_version(0);
            for (auto& slot : res.slots()) {
                std::string slot_str = get_bucket_by_tileid(res.id().tile_id()) + id_to_slot_path(request->branch(), slot.name(), res.id(), 0);
                fs::path path(slot_str);
                if (!fs::exists(path)) {                    
                    int64_t best_ver = -1;
                    if (!fs::exists(path.parent_path())) {
                        LOG(INFO) << "Can't find path for res " << res.ShortDebugString();
                        continue;
                    }
                    for (auto& p : fs::recursive_directory_iterator(path.parent_path())) {
                        if (slot.name() != (p.path().extension().c_str() + 1)) {
                            continue;
                        }
                        int64_t ver = atoll(p.path().filename().c_str());
                        if (ver > res.id().version()) {
                            continue;
                        }
                        if (ver > best_ver) {
                            best_ver = ver;
                        }
                    }
                    if (best_ver < 0) {
                        LOG(ERROR) << "Can't find slot for res " << slot_str;
                        ret_msg << "Can't find slot for res " << slot_str;
                        ret_code++;
                        continue;
                    }
                    slot_str = path.parent_path().generic_string() + '/' + std::to_string(best_ver) + '.' + slot.name();
                    LOG(INFO) << "Iterate slot " << slot_str << " for " << res.id().version();
                }
                else {
                    LOG(INFO) << "Load slot " << slot_str;
                }
                auto sb = dst->add_slots();
                sb->CopyFrom(slot);
                sb->set_version(res.id().version());
                std::ifstream ifs(slot_str, std::ios::binary);
                if (ifs.good()) {
                    sb->set_data(std::string((std::istreambuf_iterator<char>(ifs)),
                                            std::istreambuf_iterator<char>()));
                }
                else {
                    LOG(ERROR) << "Fail to load slot " << slot_str;
                    ret_msg << "Fail to load slot " << slot_str << std::endl;
                    ret_code++;
                    continue;
                }
            }
        }
        response->mutable_ret()->set_code(ret_code);
        response->mutable_ret()->set_message(ret_msg.str());
        tm.stop();
        LOG(INFO) << "Finish DownloadData() request[log_id=" << cntl->log_id() 
                  << "] @ " << request->branch() << " get " << response->res_size()
                  << " res from " << cntl->remote_side() << " to " << cntl->local_side()
                  << ", " << tm.m_elapsed() << " ms";
    }

    virtual void MergeResource(::google::protobuf::RpcController* controller,
                       const ::services::ResourceMergeRequest* request,
                       ::services::ResourceMergeResponse* response,
                       ::google::protobuf::Closure* done) {
        brpc::ClosureGuard done_guard(done);
        brpc::Controller* cntl =
            static_cast<brpc::Controller*>(controller);
        int64_t request_ver = request->has_version() ? request->version() : 0;
        LOG(INFO) << "Received MergeResource() request[log_id=" << cntl->log_id() 
                  << "] " << request->tile_id() << " @ " << request->src_branch() 
                  << " -> " << request->dst_branch() << " : " <<  request_ver
                  << " from " << cntl->remote_side() << " to " << cntl->local_side();
        butil::Timer tm;
        tm.start();
        std::stringstream ret_msg;
        int ret_code = 0;
        std::map<std::string, std::map<std::string, int64_t>> elem2type_time;
        auto prefix = get_bucket_by_tileid(request->tile_id());
        for (auto type : request->type()) {
            std::string base; 
            base.reserve(4096);
            base.append(prefix);
            base.append(request->src_branch());
            base.push_back('/');
            base.append(std::to_string(request->tile_id()));
            base.push_back('/');
            base.append(std::to_string(type));
            base.push_back('/');
            if (request->has_trail_id()) {
                base.push_back('@');
                base.append(std::to_string(request->trail_id()));
                base.push_back('/');
            }
            if (request->has_index()) {
                char buf[32];
                snprintf(buf, sizeof(buf), "%016lx", request->index());
                base.append(buf);
                base.push_back('/');                
            }
            if (!fs::exists(base)) {
                continue;
            }
            LOG(INFO) << "MergeResource() iterated on path " << base;
            for (auto& p : fs::recursive_directory_iterator(base)) {
                if (!p.path().has_extension()) {
                    continue;
                }
                int64_t ver = atoll(p.path().filename().c_str());
                if (request->has_version() && ver > request->version()) {
                    continue;
                }
                auto ps = p.path().parent_path().generic_string();
                auto it = elem2type_time.find(ps);
                if (it == elem2type_time.end()) {
                    elem2type_time[ps][p.path().extension()] = ver;
                }
                else {
                    auto vit = it->second.find(p.path().extension());
                    if (vit == it->second.end()) {
                        it->second[p.path().extension()] = ver;
                    }
                    else if (vit->second < ver) {
                        vit->second = ver;
                    }
                }
            }
        }
        for (auto& eit : elem2type_time) {
            std::string dst = eit.first;
            dst.replace(prefix.length(), request->src_branch().length(), request->dst_branch());
            std::error_code err_code;
            fs::create_directories(fs::path(dst), err_code);
            if (err_code.value() != 0) {
                LOG(ERROR) << "Fail to create dst path " << dst << " : " << err_code.message();
                ret_msg << "Fail to create dst path " << dst << " : " << err_code.message() << std::endl;
                ret_code++;
                continue;
            }
            for (auto& fit : eit.second) {
                fs::path src_path(eit.first + '/' + std::to_string(fit.second) + fit.first);
                fs::path dst_path(dst + '/' + std::to_string(fit.second) + fit.first);
                fs::create_hard_link(src_path, dst_path, err_code);
                if (err_code.value() != 0) {
                    LOG(ERROR) << "Fail to create link to " << dst_path << " : " << err_code.message();
                    ret_msg << "Fail to create link to " << dst_path << " : " << err_code.message() << std::endl;
                    ret_code++;
                    continue;
                }
                if (fit.first == ".@") {
                    std::ifstream ifs(src_path, std::ios::binary);
                    auto res = response->add_res();
                    if (!res->ParseFromIstream(&ifs)) {
                        LOG(ERROR) << "Fail to load res " << src_path;
                        ret_msg << "Fail to load res " << src_path << std::endl;
                        ret_code++;
                        response->mutable_res()->ReleaseLast();
                    }
                }
            }
        }
        response->mutable_ret()->set_code(ret_code);
        response->mutable_ret()->set_message(ret_msg.str());
        tm.stop();
        LOG(INFO) << "Finish MergeResource() request[log_id=" << cntl->log_id() 
                  << "] " << request->tile_id() << " @ " << request->src_branch() 
                  << " -> " << request->dst_branch() << " with " << response->res_size()
                  << " res from " << cntl->remote_side() << " to " << cntl->local_side()
                  << ", " << tm.m_elapsed() << " ms";
    }

    virtual void RevertResource(::google::protobuf::RpcController* controller,
                       const ::services::ResourceRevertRequest* request,
                       ::services::ResourceRevertResponse* response,
                       ::google::protobuf::Closure* done) {
        brpc::ClosureGuard done_guard(done);
        brpc::Controller* cntl = static_cast<brpc::Controller*>(controller);
        int64_t request_ver = request->has_version() ? request->version() : 0;
        LOG(INFO) << "Received RevertResource() request[log_id=" << cntl->log_id() 
                  << "] " << request->tile_id() << " @ " << request->branch() << " to " << request_ver
                  << "from " << cntl->remote_side() << " to " << cntl->local_side();
        butil::Timer tm;
        tm.start();
        std::stringstream ret_msg;
        int ret_code = 0;
        size_t remove_cnt = 0;
        for (auto type : request->type()) {
            std::string base = get_bucket_by_tileid(request->tile_id());
            base.reserve(4096);
            base.append(request->branch());
            base.push_back('/');
            base.append(std::to_string(request->tile_id()));
            base.push_back('/');
            base.append(std::to_string(type));
            base.push_back('/');
            if (request->has_trail_id()) {
                base.push_back('@');
                base.append(std::to_string(request->trail_id()));
                base.push_back('/');
            }
            if (request->has_index()) {
                char buf[32];
                snprintf(buf, sizeof(buf), "%016lx", request->index());
                base.append(buf);
                base.push_back('/');                
            }
            if (!fs::exists(base)) {
                continue;
            }
            LOG(INFO) << "RevertResource() to " << request_ver << "iterated on path " << base;
            std::error_code err_code;
            if (request_ver <= 0) {
                auto cnt = fs::remove_all(fs::path(base), err_code);
                if (err_code.value() != 0) {
                    LOG(ERROR) << "Fail to remove path " << base << " : " << err_code.message();
                    ret_msg << "Fail to remove path " << base << " : " << err_code.message() << std::endl;
                    ret_code++;
                }
                remove_cnt += cnt;
                continue;
            }
            for (auto& p : fs::recursive_directory_iterator(base)) {
                if (!p.path().has_extension()) {
                    continue;
                }
                int64_t ver = atoll(p.path().filename().c_str());
                if (ver <= request_ver) {
                    continue;
                }
                if (!fs::remove(p.path(), err_code)) {
                    LOG(ERROR) << "Fail to remove file " << p << " : " << err_code.message();
                    ret_msg << "Fail to remove file " << p << " : " << err_code.message() << std::endl;
                    ret_code++;
                }
                else {
                    remove_cnt++;
                }                
            }
        }
        
        response->mutable_ret()->set_code(ret_code);
        response->mutable_ret()->set_message(ret_msg.str());
        tm.stop();
        LOG(INFO) << "Finish RevertResource() request[log_id=" << cntl->log_id() 
                  << "] " << request->tile_id() << " @ " << request->branch() 
                  << " to " << request_ver << " remove " << remove_cnt << " files "
                  << "from " << cntl->remote_side() << " to " << cntl->local_side()
                  << ", " << tm.m_elapsed() << " ms";
    }
};
}  // namespace example

int main(int argc, char* argv[]) {
    // Parse gflags. We recommend you to use gflags as well.
    GFLAGS_NS::ParseCommandLineFlags(&argc, &argv, true);

    brpc::FLAGS_max_body_size = 256 * 1024 * 1024;
    brpc::FLAGS_socket_max_unwritten_bytes = 256 * 1024 * 1024;

    // Generally you only need one Server.
    brpc::Server server;

    // Instance of your service.
    services::TileServiceImpl tile_service_impl;

    // Add the service into server. Notice the second parameter, because the
    // service is put on stack, we don't want server to delete it, otherwise
    // use brpc::SERVER_OWNS_SERVICE.
    if (server.AddService(&tile_service_impl, 
                          brpc::SERVER_DOESNT_OWN_SERVICE) != 0) {
        LOG(ERROR) << "Fail to add service";
        return -1;
    }

    // Start the server.
    brpc::ServerOptions options;
    options.idle_timeout_sec = FLAGS_idle_timeout_s;
    options.max_concurrency = 1024;
    options.method_max_concurrency = 1024;
    if (server.Start(FLAGS_port, &options) != 0) {
        LOG(ERROR) << "Fail to start RoadServer";
        return -1;
    }

    // Wait until Ctrl-C is pressed, then Stop() and Join() the server.
    server.RunUntilAskedToQuit();
    return 0;
}

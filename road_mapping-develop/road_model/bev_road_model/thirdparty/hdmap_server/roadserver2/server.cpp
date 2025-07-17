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
#include <unordered_map>
#include <thread>

#include <gflags/gflags.h>
#include <butil/logging.h>
#include <brpc/server.h>
#include <butil/time.h>
#include <brpc/channel.h>
#include <brpc/redis.h>
#include <cpp_redis/cpp_redis>

#include "metadata/metadata.pb.h"
#include "services/roadserver.pb.h"

#include "data_access_engine.h"
#include "manager/road_geometry_mgr.h"
#include "proxy/tile_proxy.h"
#include "proxy/common_proxy.h"
#include "proxy/confidence_proxy.h"
#include "proxy/dynamics_proxy.h"
#include "proxy/lane_proxy.h"
#include "proxy/link_proxy.h"
#include "proxy/position_proxy.h"
#include "proxy/traffic_proxy.h"

DEFINE_string(store_host, "172.21.24.85", "Host of locker redis");
DEFINE_int32(store_port, 11213, "Port on host locker redis");
DEFINE_string(store_pass, "9XQmdzPdvJgbo3WP", "Auth password of storage redis");
DEFINE_string(locker_host, "172.21.24.85", "Host of locker redis");
DEFINE_int32(locker_port, 11213, "Port on host locker redis");
DEFINE_string(locker_pass, "9XQmdzPdvJgbo3WP", "Auth password of locker reids");
DEFINE_string(watch, "watchlist", "watch command for redis server");
DEFINE_int32(port, 8081, "Server bind port");
/*DEFINE_string(store_host, "127.0.0.1", "Host of locker redis");
DEFINE_int32(store_port, 6379, "Port on host locker redis");
DEFINE_string(store_pass, "", "Auth password of storage redis");
DEFINE_string(locker_host, "127.0.0.1", "Host of locker redis");
DEFINE_int32(locker_port, 6379, "Port on host locker redis");
DEFINE_string(locker_pass, "", "Auth password of locker reids");
DEFINE_string(watch, "watch", "watch command for redis server");
DEFINE_int32(port, 8081, "Server bind port");*/
DEFINE_int32(timeout_ms, 600000, "RPC timeout in milliseconds");
DEFINE_int32(max_retry, 10, "Max retries(not including the first RPC)");
DEFINE_int32(interval_ms, 1000, "Milliseconds between consecutive requests");
DEFINE_int64(reqver, 0, "max version retrieved");
DEFINE_int32(idle_timeout_s, -1, "Connection will be closed if there is no "
             "read/write operations during the last `idle_timeout_s'");
DEFINE_int32(logoff_ms, 2000, "Maximum duration of server's LOGOFF state "
             "(waiting for client to close connection before server stops)");
DEFINE_int32(max_conn_num, 256, "Concurrent connection to storage redis");
DEFINE_int32(max_pipeline_cmd, 1000, "Max redis commands in pipeline before commit()");

namespace brpc {
DECLARE_uint64(max_body_size);
DECLARE_int64(socket_max_unwritten_bytes);
}

namespace services {

class RedisChannel;
class RedisReply {
public:
    RedisReply() : _res(nullptr), _ind(-1) {};
    RedisReply(RedisChannel* res, int ind) : _res(res), _ind(ind) {};
    ~RedisReply() { _res = nullptr; _ind = -1; };
    const brpc::RedisReply* operator->() const;
    explicit operator bool() const;
    int index() { return _ind; };
    std::string cmds() const;

private:
    RedisChannel* _res;
    int _ind;
};

class RedisChannel {
public:
    enum ChannelState {
        INVALID = -1,
        CONNECTED = 0,
        PIPED = 1,
        CALLING = 2,
        FINISHED = 3
    };
    RedisChannel(const std::string& server, const std::string& pass) : _state(0) {
        //LOG(INFO) << "RedisChannel() to " << server << " @ " << (void*)this;
        brpc::ChannelOptions options;
        options.protocol = brpc::PROTOCOL_REDIS;
        options.connection_type = brpc::CONNECTION_TYPE_SHORT;
        options.connect_timeout_ms = FLAGS_timeout_ms / 50;
        options.timeout_ms = FLAGS_timeout_ms / 10;
        options.max_retry = std::max(1, FLAGS_max_retry / 3);
        if (_channel.Init(server.c_str(), &options) != 0) {
            LOG(ERROR) << "Fail to init channel to " << server;
            _state = ChannelState::INVALID;
            throw cpp_redis::redis_error("fail to connect to server");
        }
        if (!pass.empty()) {
            _request.AddCommand("AUTH %s", pass.c_str());
            _cmds_bufs.emplace_back();
        }        
    };
    ~RedisChannel() {
        if (_state == ChannelState::CALLING) {
            //LOG(INFO) << "Waiting for " << _request.command_size() << " cmds finish";
            brpc::Join(_cntl.call_id());
        }
        _state = -1;
        _request.Clear();
        _response.Clear();
        _cntl.Reset();
        _cmds_bufs.clear();
        //LOG(INFO) << "~RedisChannel() finished @ " << (void*)this;
    };

    void on_done() {
        _state = ChannelState::FINISHED;
        s_conn_cnt--;
        if (_cntl.Failed()) {
            LOG(ERROR) << "Fail to commit() " << _request.command_size() << " cmds @ " << (void*)this 
                       << " : " << _cntl.ErrorText();
            _state = ChannelState::INVALID;
        }
        else {
            //LOG(INFO) << "commit() " << _request.command_size() << " cmds @ " << (void*)this << ", "
            //          << s_conn_cnt.load() << " conns";
        }
    };
    bool commit() {
        if (_state < ChannelState::PIPED) {
            return false;
        }
        if (_state != ChannelState::PIPED) {
            LOG(ERROR) << "RedisChannel::commit() failed for state " << _state << " @ " << (void*)this;
            return false;
        }
        _state = ChannelState::CALLING;
        s_conn_cnt++;
        _channel.CallMethod(nullptr, &_cntl, &_request, &_response, brpc::NewCallback(this, &RedisChannel::on_done));
        //LOG(INFO) << "RedisChannel::commit() called with " << _request.command_size() << " cmds @ " 
        //          << (void*)this << ", " << s_conn_cnt.load() << " conns";
        return true;
    };
    bool sync() {
        if (_state == ChannelState::INVALID) {
            throw cpp_redis::redis_error("commit to server failed");
        }
        if (_state != ChannelState::CALLING) {
            //LOG(ERROR) << "RedisChannel::sync() failed for state " << _state << " @ " << (void*)this;
            return false;
        }
        brpc::Join(_cntl.call_id());
        if (_state == ChannelState::INVALID) {
            throw cpp_redis::redis_error("commit to server failed");
        }
        return true;
    };
    bool sync_commit() {
        if (_state == ChannelState::PIPED) {
            commit();
        }
        return sync();
    };

    RedisReply send(std::vector<std::string>& cmds) {
        if (cmds.empty()) {
            return RedisReply();
        }
        if (_state != ChannelState::PIPED && _state != ChannelState::CONNECTED) {
            return RedisReply();
        }
        int cmd_ind = _request.command_size();
        std::vector<butil::StringPiece> cs(cmds.size());
        for (size_t i = 0; i < cmds.size(); i++) {
            cs[i] = cmds[i];
        }
        _request.AddCommandByComponents(cs.data(), cs.size());
        _cmds_bufs.emplace_back();
        _cmds_bufs.back().swap(cmds);
        _state = ChannelState::PIPED;
        return RedisReply(this, cmd_ind);
    };
    
    bool is_connected() const {
        return _state >= ChannelState::CONNECTED && _state < ChannelState::FINISHED;
    };
    bool need_sync() const {
        return _state >= ChannelState::PIPED && _state < ChannelState::FINISHED;
    };
    bool is_finished() const { return _state == ChannelState::FINISHED; };
    int cmd_cnt() { return _request.command_size(); };
    static int conn_cnt() { return s_conn_cnt.load(); };
private:
    friend class RedisReply;
    brpc::Channel _channel;
    brpc::RedisRequest _request;
    brpc::RedisResponse _response;
    brpc::Controller _cntl;
    std::atomic<int> _state;
    std::deque<std::vector<std::string>> _cmds_bufs;
    static std::atomic<int> s_conn_cnt;
};

const brpc::RedisReply* RedisReply::operator->() const {
    if (_res == nullptr || _ind < 0) {
        return nullptr;
    }
    _res->sync();
    if (!_res->is_finished()) {
        return nullptr;
    }
    if (_res->_response.reply_size() <= _ind) { 
        return nullptr;
    }
    return &_res->_response.reply(_ind);
};

std::string RedisReply::cmds() const {
    std::stringstream tmp;
    if (_res == nullptr || _ind < 0) {
        return tmp.str();
    }
    if ((int)_res->_cmds_bufs.size() <= _ind) {
        return tmp.str();
    }
    for (auto& s : _res->_cmds_bufs[_ind]) {
        tmp << s << ' ';
    }
    return tmp.str();
};

RedisReply::operator bool() const {
    if (_res == nullptr || _ind < 0) {
        return false;
    }
    _res->sync();
    if (!_res->is_finished()) {
        return false;
    }
    if (_res->_response.reply_size() <= _ind) { 
        return false;
    }
    return true;
};

class MultiConnClient {
public:
    MultiConnClient(int mc) : _port(0), _max_cmds(mc) {};
    ~MultiConnClient() {
        int i = 0;
        for (auto& cli : _clients) {
            if (cli) {
                //LOG(INFO) << "Client " << i << " disconnect(), " << s_conn_cnt.load() << " remains";
                cli.reset();             
            }                
            i++;
        }
        _clients.clear();
        LOG(INFO) << "~MultiConnClient() finish, " << RedisChannel::conn_cnt() << " remains";
    };

    bool connect(const std::string &host = "127.0.0.1",
					int port = 6379,
                    const std::string &pass = "",
					std::uint32_t timeout_ms = 0,
					std::int32_t max_reconnects = 0,
					std::uint32_t reconnect_interval_ms = 0) {
        _host = host;
        _port = port;
        _password = pass;
        _timeout_ms = timeout_ms;
        _max_reconnects = max_reconnects;
        _reconnect_interval_ms = _reconnect_interval_ms;
        std::unique_ptr<RedisChannel> cli(new RedisChannel(host+':'+std::to_string(port), pass));
        _clients.push_back(std::move(cli));
        //LOG(INFO) << "Client construct: " << RedisChannel::conn_cnt() << " remains";
        return _clients.back()->is_connected();
    };

    bool commit() {
        if (_clients.empty() || !_clients.back()) {
            return false;
        }
        //LOG(INFO) << "Client " << _clients.size() - 1 << " commit()";
        _clients.back()->commit();
        std::unique_ptr<RedisChannel> c(new RedisChannel(_host+':'+std::to_string(_port), _password));
        _clients.push_back(std::move(c));
        return true;
    };
    bool sync_commit() {
        if (_clients.empty()) {
            return false;
        }
        int i = 0;
        for (auto& cli : _clients) {
            if (cli && cli->need_sync()) {
                //LOG(INFO) << "Client " << i << " sync_commit()";
                cli->sync_commit();
            }
            i++;            
        }
        std::unique_ptr<RedisChannel> c(new RedisChannel(_host+':'+std::to_string(_port), _password));
        _clients.push_back(std::move(c));
        return true;
    };

    RedisReply send(std::vector<std::string>& cmds) {
        if (_clients.empty() || cmds.empty()) {
            return RedisReply();
        }
        auto& cli = _clients.back();
        if (cmds.size() > 1) {
            //LOG(INFO) << "Client" << _clients.size() - 1 << " send " << cmds[0] << " " << cmds[1];
        }
        else {
            //LOG(INFO) << "Client" << _clients.size() - 1 << " send " << cmds[0];
        }
        auto rep = cli->send(cmds);
        if (cli->cmd_cnt() >= _max_cmds) {
            //LOG(INFO) << "Client " << _clients.size() - 1 << " commit & pushback " << s_conn_cnt.load();
            cli->commit();
            for (int retry = 0; retry < 10; retry++) {
                std::this_thread::sleep_for(std::chrono::seconds(retry));
                if (RedisChannel::conn_cnt() < FLAGS_max_conn_num) {
                    break;
                }
                LOG(INFO) << "Sleep " << retry+1 << " to retry commit & pushback " << RedisChannel::conn_cnt(); 
            }
            std::unique_ptr<RedisChannel> c(new RedisChannel(_host+':'+std::to_string(_port), _password));
            _clients.push_back(std::move(c));                        
        }
        return rep;
    };

    RedisReply get(const std::string& key) {
        std::vector<std::string> cmds = {"GET", key};
        return send(cmds);
    };
    RedisReply set(const std::string& key, const std::string& val) {
        std::vector<std::string> cmds = {"SET", key, val};
        return send(cmds);
    };

private:
    std::deque<std::unique_ptr<RedisChannel>> _clients;
    std::string _host;
    int _port;
    std::string _password;
    uint32_t _timeout_ms;
    int32_t _max_reconnects;
    uint32_t _reconnect_interval_ms;
    int _max_cmds;    
};

std::atomic<int> RedisChannel::s_conn_cnt(0);

class RoadServiceImpl : public RoadService {
public:
    RoadServiceImpl() {};
    virtual ~RoadServiceImpl() {};

    std::string IDtoKey(const RoadPB::FeatureID& id, int64_t ts) {
        if (id.version() <= 0 || id.version() > ts) {
            char buf[128];
            snprintf(buf, sizeof(buf), "%08x|%x:%016lx@%ld", id.tileid(), id.type(), id.id(), ts);
            return buf;
        }
        else {
            char buf[128];
            snprintf(buf, sizeof(buf), "%08x|%x:%016lx@%ld", id.tileid(), id.type(), id.id(), id.version());
            return buf;
        }
    };
    std::string IDtoKey(const data_access_engine::FeatureIDProxy& id) {
        char buf[128];
        snprintf(buf, sizeof(buf), "%08x|%x:%016lx@0", id.tileid(), id.type(), id.id());
        return buf;
    };

    std::string IDtoTileKey(const std::string& branch, int tileid, int type, int64_t ts) {
        if (ts > 0) {
            char buf[512];
            snprintf(buf, sizeof(buf), "%s/%08x|%x:%ld", branch.c_str(), tileid, type, ts);
            return buf;
        }
        else {
            char buf[512];
            snprintf(buf, sizeof(buf), "%s/%08x|%x", branch.c_str(), tileid, type);
            return buf;
        }
    };
    std::string IDtoTileEnd(const std::string& branch, int tileid, int type) {
        char buf[512];
        snprintf(buf, sizeof(buf), "%s/%08x|%x;", branch.c_str(), tileid, type);
        return buf;
    };

    virtual void UploadTile(::google::protobuf::RpcController* controller,
                            const ::services::TileUploadRequest* request,
                            ::services::TileUploadResponse* response,
                            ::google::protobuf::Closure* done) {
        brpc::ClosureGuard done_guard(done);
        butil::Timer tm;
        tm.start();
        brpc::Controller* cntl = static_cast<brpc::Controller*>(controller);

        LOG(INFO) << "Received UploadTile() request[log_id=" << cntl->log_id() 
                  << "] from " << cntl->remote_side() 
                  << " to " << cntl->local_side();
        try {
        cpp_redis::client locker;
        std::future<cpp_redis::reply> lock_rep;
        locker.connect(FLAGS_locker_host, FLAGS_locker_port);//, nullptr, 
                      //FLAGS_timeout_ms / 10, FLAGS_max_retry / 2, FLAGS_interval_ms);
        if (!FLAGS_locker_pass.empty()) {
            lock_rep = locker.auth(FLAGS_locker_pass);
            locker.commit();
        }     
        MultiConnClient store(FLAGS_max_pipeline_cmd);
        if ((!FLAGS_locker_pass.empty() && !lock_rep.get().is_simple_string())
                || !store.connect(FLAGS_store_host, FLAGS_store_port, FLAGS_store_pass/*,
                                  FLAGS_timeout_ms / 10, FLAGS_max_retry / 2, FLAGS_interval_ms*/)) {
            cntl->SetFailed("Fail to connect to redis-server");
            LOG(INFO) << "Failed UploadTile() request[log_id=" << cntl->log_id() 
                      << "] from " << cntl->remote_side() 
                      << " to " << cntl->local_side();
            return;
        }
        std::stringstream ret_msg;
        response->set_branch(request->branch());
        
        for (int retry_cnt = 0; retry_cnt < 10; ++retry_cnt) {
            int64_t timestamp = store_tiles(locker, store, request->branch(), true, false, ret_msg, 
                                        request->tile(), response->mutable_tile());
            if (timestamp == 0) {
                response->mutable_ret()->set_message("Upload empty request");
                response->mutable_ret()->set_code(0);
                LOG(INFO) << "Finish UploadTile() empty request[log_id=" << cntl->log_id() 
                          << "] from " << cntl->remote_side() 
                          << " to " << cntl->local_side();
                return;
            }
            if (timestamp > 0) {
                response->mutable_ret()->set_message(ret_msg.str());
                response->mutable_ret()->set_code(0);
                response->set_read_ver(timestamp);
                tm.stop();
                LOG(INFO) << "Finish UploadTile() to " << request->branch() << " @ " << timestamp 
                        << "[log_id=" << cntl->log_id() 
                        << "] from " << cntl->remote_side() 
                        << " to " << cntl->local_side()
                        << ", " << tm.m_elapsed() << "ms";
                return;
            }
        }
        ret_msg << "Commit confliction for 10 times, failed!";
        response->mutable_ret()->set_message(ret_msg.str());
        response->mutable_ret()->set_code(-1);
        cntl->SetFailed("Commit confliction for 10 times, failed!");
        } catch (cpp_redis::redis_error& err) {
            LOG(ERROR) << "redis_error: " << err.what();
            cntl->SetFailed("Fail to connect to redis-server");
            std::this_thread::sleep_for(std::chrono::seconds(10));
            return;
        }
        tm.stop();
        LOG(INFO) << "Conflicted UploadTile() to " << request->branch()
                  << " request[log_id=" << cntl->log_id() 
                  << "] from " << cntl->remote_side() 
                  << " to " << cntl->local_side()
                  << ", " << tm.m_elapsed() << "ms";
    };

    virtual void DownloadTile(::google::protobuf::RpcController* controller,
                              const ::services::TileDownloadRequest* request,
                              ::services::TileDownloadResponse* response,
                              ::google::protobuf::Closure* done) {
        brpc::ClosureGuard done_guard(done);

        brpc::Controller* cntl = static_cast<brpc::Controller*>(controller);
        butil::Timer tm;
        tm.start();
        LOG(INFO) << "Received DownloadTile() request[log_id=" << cntl->log_id() 
                  << "] from " << cntl->remote_side() 
                  << " to " << cntl->local_side();
        try {
            cpp_redis::client locker;
            std::future<cpp_redis::reply> lock_rep;
            locker.connect(FLAGS_locker_host, FLAGS_locker_port/*, nullptr, 
                        FLAGS_timeout_ms / 10, FLAGS_max_retry / 2, FLAGS_interval_ms*/);
            if (!FLAGS_locker_pass.empty()) {
                lock_rep = locker.auth(FLAGS_locker_pass);        
            }
            locker.commit();
            MultiConnClient store(FLAGS_max_pipeline_cmd);
            if ((!FLAGS_locker_pass.empty() && !lock_rep.get().is_simple_string())
                    || !store.connect(FLAGS_store_host, FLAGS_store_port, FLAGS_store_pass/*,
                                    FLAGS_timeout_ms / 10, FLAGS_max_retry / 2, FLAGS_interval_ms*/)) {
                cntl->SetFailed("Fail to connect to redis-server");
                LOG(INFO) << "Failed DownloadTile() request[log_id=" << cntl->log_id() 
                        << "] from " << cntl->remote_side() 
                        << " to " << cntl->local_side();
                return;
            }
            std::stringstream ret_msg;
            response->set_branch(request->branch());
            int64_t max_ver = 0;        
            if (request->has_request_version()) {
                max_ver = request->request_version();            
            }
            else {
                std::vector<std::string> cmds = { "ZSCORE", "TimestampBranches", request->branch() };
                auto sc = locker.send(cmds);
                locker.sync_commit();
                auto rep = sc.get();
                if (rep.is_string()) {
                    max_ver = atoll(rep.as_string().c_str());
                }
            }
            if (max_ver <= 0 || request->tile_id_size() <= 0 || request->need_type_size() <= 0) {
                response->mutable_ret()->set_message("Retrive empty branch!");
                response->mutable_ret()->set_code(0);
                int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>
                                    (std::chrono::system_clock::now().time_since_epoch()).count();
                response->set_editor_version(timestamp);
                LOG(INFO) << "Finish DownloadTile() empty request[log_id=" << cntl->log_id() 
                        << "] from " << cntl->remote_side() 
                        << " to " << cntl->local_side();
                return;
            }
            
            if (request->has_load_ref() && request->load_ref()) {
                load_tiles_with_ref(locker, store, request->branch(), request->tile_id(), request->need_type(),
                                    max_ver, false, ret_msg, response->mutable_tile_info());
            }
            else {
                load_tiles(locker, store, request->branch(), request->tile_id(), request->need_type(),
                            max_ver, true, ret_msg, nullptr, response->mutable_tile_info());
            }
            response->mutable_ret()->set_message(ret_msg.str());
            response->mutable_ret()->set_code(0);
        } catch (cpp_redis::redis_error& err) {
            LOG(ERROR) << "redis_error: " << err.what(); 
            cntl->SetFailed("Fail to connect to redis-server");
            std::this_thread::sleep_for(std::chrono::seconds(10));
            return;
        }
        tm.stop();
        LOG(INFO) << "Finish DownloadTile() to " << request->branch() 
                  << " request[log_id=" << cntl->log_id() 
                  << "] from " << cntl->remote_side() 
                  << " to " << cntl->local_side()
                  << ", " << tm.m_elapsed() << " ms";

        int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>
                                (std::chrono::system_clock::now().time_since_epoch()).count();
        response->set_editor_version(timestamp);
    };

    virtual void MergeTile(::google::protobuf::RpcController* controller,
                              const ::services::TileMergeRequest* request,
                              ::services::TileMergeResponse* response,
                              ::google::protobuf::Closure* done) {
        brpc::ClosureGuard done_guard(done);

        brpc::Controller* cntl = static_cast<brpc::Controller*>(controller);
        butil::Timer tm;
        tm.start();
        LOG(INFO) << "Received MergeTile() request[log_id=" << cntl->log_id() 
                  << "] from " << cntl->remote_side() 
                  << " to " << cntl->local_side();
        try {
            cpp_redis::client locker;
            std::future<cpp_redis::reply> lock_rep;
            locker.connect(FLAGS_locker_host, FLAGS_locker_port/*, nullptr, 
                        FLAGS_timeout_ms / 10, FLAGS_max_retry / 2, FLAGS_interval_ms*/);
            if (!FLAGS_locker_pass.empty()) {
                lock_rep = locker.auth(FLAGS_locker_pass);        
            }
            locker.commit();
            MultiConnClient store(FLAGS_max_pipeline_cmd);
            if ((!FLAGS_locker_pass.empty() && !lock_rep.get().is_simple_string())
                    || !store.connect(FLAGS_store_host, FLAGS_store_port, FLAGS_store_pass/*,
                                    FLAGS_timeout_ms / 10, FLAGS_max_retry / 2, FLAGS_interval_ms*/)) {
                cntl->SetFailed("Fail to connect to redis-server");
                LOG(INFO) << "Failed MergeTile() request[log_id=" << cntl->log_id() 
                        << "] from " << cntl->remote_side() 
                        << " to " << cntl->local_side();
                return;
            }
            std::stringstream ret_msg;
            response->set_dst_branch(request->dst_branch());
            int64_t max_ver = 0;        
            if (request->has_request_version()) {
                max_ver = request->request_version();            
            }
            else {
                std::vector<std::string> cmds = { "ZSCORE", "TimestampBranches", request->src_branch() };
                auto sc = locker.send(cmds);
                locker.sync_commit();
                auto rep = sc.get();
                if (rep.is_string()) {
                    max_ver = atoll(rep.as_string().c_str());
                }
            }
            if (max_ver <= 0 || request->tile_id_size() <= 0 || request->need_type_size() <= 0) {
                response->mutable_ret()->set_message("Retrive empty branch!");
                response->mutable_ret()->set_code(0);
                int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>
                                    (std::chrono::system_clock::now().time_since_epoch()).count();
                response->set_editor_version(timestamp);
                LOG(INFO) << "Finish MergeTile() empty request[log_id=" << cntl->log_id() 
                        << "] from " << cntl->remote_side() 
                        << " to " << cntl->local_side();
                return;
            }
            ::google::protobuf::RepeatedPtrField<::services::TileInfo> tile_infos;
            load_tiles_with_ref(locker, store, request->src_branch(), request->tile_id(), request->need_type(),
                                max_ver, false, ret_msg, &tile_infos);            
        
            for (int retry_cnt = 0; retry_cnt < 10; ++retry_cnt) {
                int64_t timestamp = store_tiles(locker, store, request->dst_branch(), false, false, ret_msg, 
                                            tile_infos, response->mutable_tile_info());
                if (timestamp == 0) {
                    response->mutable_ret()->set_message("Merge empty request");
                    response->mutable_ret()->set_code(0);
                    LOG(INFO) << "Finish MergeTile() empty request[log_id=" << cntl->log_id() 
                            << "] from " << cntl->remote_side() 
                            << " to " << cntl->local_side();
                    return;
                }
                if (timestamp > 0) {
                    response->mutable_ret()->set_message(ret_msg.str());
                    response->mutable_ret()->set_code(0);
                    tm.stop();
                    LOG(INFO) << "Finish MergeTile() " << request->src_branch()
                                << " -> " << request->dst_branch() 
                                << " request[log_id=" << cntl->log_id() 
                                << "] from " << cntl->remote_side() 
                                << " to " << cntl->local_side()
                                << ", " << tm.m_elapsed() << " ms";
                    int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>
                                    (std::chrono::system_clock::now().time_since_epoch()).count();
                    response->set_editor_version(timestamp);
                    return;
                }
            }
            ret_msg << "Commit confliction for 10 times, failed!";
            response->mutable_ret()->set_message(ret_msg.str());
            response->mutable_ret()->set_code(-1);
            cntl->SetFailed("Commit confliction for 10 times, failed!");
        } catch (cpp_redis::redis_error& err) {
            LOG(ERROR) << "redis_error: " << err.what(); 
            cntl->SetFailed("Fail to connect to redis-server");
            std::this_thread::sleep_for(std::chrono::seconds(10));
            return;
        }
        tm.stop();
        LOG(INFO) << "Conflict MergeTile() " << request->src_branch()
                  << " -> " << request->dst_branch() 
                  << " request[log_id=" << cntl->log_id() 
                  << "] from " << cntl->remote_side() 
                  << " to " << cntl->local_side()
                  << ", " << tm.m_elapsed() << " ms";

        int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>
                                (std::chrono::system_clock::now().time_since_epoch()).count();
        response->set_editor_version(timestamp);
    };

    virtual void RevertTile(::google::protobuf::RpcController* controller,
                              const ::services::TileRevertRequest* request,
                              ::services::TileRevertResponse* response,
                              ::google::protobuf::Closure* done) {
        brpc::ClosureGuard done_guard(done);

        brpc::Controller* cntl = static_cast<brpc::Controller*>(controller);
        butil::Timer tm;
        tm.start();
        LOG(INFO) << "Received RevertTile() request[log_id=" << cntl->log_id() 
                  << "] from " << cntl->remote_side() 
                  << " to " << cntl->local_side();
        try {
            cpp_redis::client locker;
            std::future<cpp_redis::reply> lock_rep;
            locker.connect(FLAGS_locker_host, FLAGS_locker_port/*, nullptr, 
                        FLAGS_timeout_ms / 10, FLAGS_max_retry / 2, FLAGS_interval_ms*/);
            if (!FLAGS_locker_pass.empty()) {
                lock_rep = locker.auth(FLAGS_locker_pass);        
            }
            locker.commit();
            MultiConnClient store(FLAGS_max_pipeline_cmd);
            if ((!FLAGS_locker_pass.empty() && !lock_rep.get().is_simple_string())
                    || !store.connect(FLAGS_store_host, FLAGS_store_port, FLAGS_store_pass/*,
                                    FLAGS_timeout_ms / 10, FLAGS_max_retry / 2, FLAGS_interval_ms*/)) {
                cntl->SetFailed("Fail to connect to redis-server");
                LOG(INFO) << "Failed MergeTile() request[log_id=" << cntl->log_id() 
                        << "] from " << cntl->remote_side() 
                        << " to " << cntl->local_side();
                return;
            }
            std::stringstream ret_msg;
            response->set_branch(request->branch());
            int64_t max_ver = 0;
            {
                std::vector<std::string> cmds = { "ZSCORE", "TimestampBranches", request->branch() };
                auto sc = locker.send(cmds);
                locker.sync_commit();
                auto rep = sc.get();
                if (rep.is_string()) {
                    max_ver = atoll(rep.as_string().c_str());
                }
            }
            if (max_ver <= 0 || request->tile_id_size() <= 0 || request->need_type_size() <= 0) {
                response->mutable_ret()->set_message("Revert empty branch!");
                response->mutable_ret()->set_code(0);
                int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>
                                    (std::chrono::system_clock::now().time_since_epoch()).count();
                response->set_editor_version(timestamp);
                LOG(INFO) << "Finish RevertTile() empty request[log_id=" << cntl->log_id() 
                        << "] from " << cntl->remote_side() 
                        << " to " << cntl->local_side();
                return;
            }
            ::google::protobuf::RepeatedPtrField<::services::TileInfo> cur_tiles;
            load_tiles_with_ref(locker, store, request->branch(), request->tile_id(), request->need_type(), 
                                max_ver, true, ret_msg, &cur_tiles);
            if (!request->has_request_version() || request->request_version() <= 0) {
                for (auto& t : cur_tiles) {
                    for (auto& fl : *t.mutable_feat_list()) {
                        for (auto& f : *fl.mutable_feats()) {
                            f.mutable_id()->set_is_deleted(true);
                        }
                        for (auto& r : *fl.mutable_refs()) {
                            r.set_is_deleted(true);
                        }
                    }
                }
            }
            else {
                max_ver = request->request_version();
                ::google::protobuf::RepeatedPtrField<::services::TileInfo> old_tiles;
                load_tiles_with_ref(locker, store, request->branch(), request->tile_id(), request->need_type(),
                                    max_ver, true, ret_msg, &old_tiles);
                std::map<int, int> cur_tileind;
                std::map<int64_t, int> cur_featind;
                std::map<int, std::map<int, std::map<uint64_t, ::services::RoadFeature*>>> cur_featmap;
                std::map<int, std::map<int, std::map<std::string, ::RoadPB::FeatureID*>>> cur_refmap;
                for (int ti = 0; ti < cur_tiles.size(); ti++) {
                    auto t = cur_tiles.Mutable(ti);
                    cur_tileind[t->tile_id()] = ti;
                    for (int fi = 0; fi < t->feat_list_size(); fi++) {
                        auto fl = t->mutable_feat_list(fi);
                        int64_t ind = ti;
                        ind <<= 8;
                        ind += fl->type();
                        cur_featind[ind] = fi;
                        for (auto& feat : *fl->mutable_feats()) {
                            feat.mutable_id()->set_is_deleted(true);
                        }
                        for (auto& ref : *fl->mutable_refs()) {
                            ref.set_is_deleted(true);
                        }
                    }
                }
                for (auto& ot : old_tiles) {
                    if (cur_tileind.count(ot.tile_id()) <= 0) {
                        cur_tileind[ot.tile_id()] = cur_tiles.size();
                        cur_tiles.Add()->CopyFrom(ot);
                        continue;
                    }
                    auto ct = cur_tiles.Mutable(cur_tileind[ot.tile_id()]);
                    for (auto& fl : *ot.mutable_feat_list()) {
                        int64_t ind = ot.tile_id();
                        ind <<= 8;
                        ind += fl.type();
                        if (cur_featind.count(ind) <= 0) {
                            cur_featind[ind] = ct->feat_list_size();
                            ct->add_feat_list()->CopyFrom(fl);
                            continue;
                        }
                        auto cl = ct->mutable_feat_list(cur_featind[ind]);
                        std::unordered_map<uint64_t, ::services::RoadFeature*> featmap(cl->feats_size() * 2);
                        std::unordered_map<std::string, ::RoadPB::FeatureID*> refmap(cl->refs_size() * 2);
                        for (auto& f : *cl->mutable_feats()) {
                            featmap[f.id().id()] = &f;
                        }
                        for (auto& r : *cl->mutable_refs()) {
                            refmap[IDtoKey(r, 0)] = &r;
                        }
                        for (auto& f : fl.feats()) {
                            if (featmap.count(f.id().id()) <= 0) {
                                auto cf = cl->add_feats();
                                cf->CopyFrom(f);
                                featmap[f.id().id()] = cf;
                                continue;
                            }
                            featmap[f.id().id()]->CopyFrom(f);
                        }
                        for (auto& r : fl.refs()) {
                            auto key = IDtoKey(r, 0);
                            if (refmap.count(key) <= 0) {
                                auto cr = cl->add_refs();
                                cr->CopyFrom(r);
                                refmap[key] = cr;
                                continue;
                            }
                            refmap[key]->CopyFrom(r);
                        }
                    }
                }
            }
            for (int retry_cnt = 0; retry_cnt < 10; ++retry_cnt) {
                int64_t timestamp = store_tiles(locker, store, request->branch(), false, true, ret_msg,
                                                cur_tiles, response->mutable_tile_info());
                if (timestamp == 0) {
                    response->mutable_ret()->set_message("Revert empty request");
                    response->mutable_ret()->set_code(0);
                    LOG(INFO) << "Finish RevertTile() empty request[log_id=" << cntl->log_id() 
                            << "] from " << cntl->remote_side() 
                            << " to " << cntl->local_side();
                    return;
                }
                if (timestamp > 0) {
                    response->mutable_ret()->set_message(ret_msg.str());
                    response->mutable_ret()->set_code(0);
                    tm.stop();
                    LOG(INFO) << "Finish RevertTile() " << request->branch()
                                << " request[log_id=" << cntl->log_id() 
                                << "] from " << cntl->remote_side() 
                                << " to " << cntl->local_side()
                                << ", " << tm.m_elapsed() << " ms";
                    int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>
                                    (std::chrono::system_clock::now().time_since_epoch()).count();
                    response->set_editor_version(timestamp);
                    return;
                }
            }
            response->mutable_ret()->set_message(ret_msg.str());
            response->mutable_ret()->set_code(-1);
            cntl->SetFailed("Commit confliction for 10 times, failed!");
        } catch (cpp_redis::redis_error& err) {
            LOG(ERROR) << "redis_error: " << err.what(); 
            cntl->SetFailed("Fail to connect to redis-server");
            std::this_thread::sleep_for(std::chrono::seconds(10));
            return;
        }
        tm.stop();
        LOG(INFO) << "Finish RevertTile() to " << request->branch() 
                  << " request[log_id=" << cntl->log_id() 
                  << "] from " << cntl->remote_side() 
                  << " to " << cntl->local_side()
                  << ", " << tm.m_elapsed() << " ms";

        int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>
                                (std::chrono::system_clock::now().time_since_epoch()).count();
        response->set_editor_version(timestamp);
    };

private:
    int64_t store_tiles(cpp_redis::client& locker, MultiConnClient& store, const std::string& branch,
                        bool write_data, bool force_commit, std::stringstream& ret_msg,
                        const ::google::protobuf::RepeatedPtrField<::services::TileInfo>& tiles,
                        ::google::protobuf::RepeatedPtrField<::services::TileInfo>* resp_tiles) {
        int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>
                                    (std::chrono::system_clock::now().time_since_epoch()).count();
        while (true) {
            std::string ts("ts");
            ts += std::to_string(timestamp);
            auto rep = locker.setnx(ts, branch);
            locker.sync_commit();
            if (rep.get().is_integer()) {
                break;                    
            }
            timestamp++;
        }
        auto ts = std::to_string(timestamp);
            
        resp_tiles->Clear();
        LOG(INFO) << "store_tiles() to " << branch << " @ " << timestamp;
        std::vector<std::string> btkeys;
        btkeys.reserve(tiles.size() * 16 + 1);
        btkeys.push_back(FLAGS_watch);
        for (int i = 0; i < tiles.size(); i++) {
            auto& tile = tiles.Get(i);
            for (int j = 0; j < tile.feat_list_size(); j++) {
                auto& fl = tile.feat_list(j);
                if (fl.feats_size() > 0 || fl.refs_size() > 0) {
                    auto key = IDtoTileEnd(branch, tile.tile_id(), fl.type());
                    btkeys.push_back(key);
                }
            }
        }
        if (btkeys.size() <= 1) {
            return 0;
        }
            
        LOG(INFO) << "store_tiles() begin watch " << btkeys.size() - 1 << " keys";
        locker.send(btkeys);
        locker.sync_commit();            
        LOG(INFO) << "store_tile() finish watch " << btkeys.size() - 1 << " keys";            
            
        std::vector<std::map<int, std::future<cpp_redis::reply>>> linds(tiles.size());
        for (int i = 0; i < tiles.size(); i++) {
            auto& tile = tiles.Get(i);
            for (int j = 0; j < tile.feat_list_size(); j++) {
                auto& fl = tile.feat_list(j);
                if (fl.feats_size() > 0 || fl.refs_size() > 0) {
                    auto tile_key = IDtoTileEnd(branch, tile.tile_id(), fl.type());
                    linds[i][j] = locker.lindex(tile_key, 0);                    
                }
            }
        }
        locker.commit();
        LOG(INFO) << "store_tile() finish lindex " << btkeys.size() - 1 << " keys";
            
        std::vector<std::map<int, RedisReply>> flists(tiles.size());
        for (size_t i = 0; i < linds.size(); i++) {
            auto& tile = tiles.Get(i);
            auto tile_bld = resp_tiles->Add();
            tile_bld->set_version(timestamp);
            for (auto& tit : linds[i]) {
                auto& fl = tile.feat_list(tit.first);                
                if (tit.second.valid()) {
                    auto rep = tit.second.get();
                    if (rep.is_string()) {
                        int64_t ver = atoll(rep.as_string().c_str());
                        auto list_key = IDtoTileKey(branch, tile.tile_id(), fl.type(), ver);
                        flists[i][tit.first] = store.get(list_key);
                        continue;
                    }   
                } 
                {
                    auto new_key = IDtoTileKey(branch, tile.tile_id(), fl.type(), timestamp);
                    auto fl_bld = tile_bld->add_feat_list();
                    fl_bld->set_type(fl.type());
                    for (int k = 0; k < fl.feats_size(); k++) {
                        auto& rf = fl.feats(k);
                        if (write_data && !rf.data().empty()) {
                            auto key = IDtoKey(rf.id(), timestamp);
                            store.set(key, rf.data());
                        }
                        if (rf.id().version() != 1) {
                            auto fd = fl_bld->add_feats();
                            auto id = fd->mutable_id();
                            id->CopyFrom(rf.id());
                            if (id->version() <= 0 || id->version() > timestamp) {
                                id->set_version(timestamp);
                            }
                        }
                    }
                    for (int k = 0; k < fl.refs_size(); k++) {
                        if (fl.refs(k).version() != 1) {
                            auto id = fl_bld->add_refs();
                            id->CopyFrom(fl.refs(k));
                            if (id->version() <= 0 || id->version() > timestamp) {
                                id->set_version(timestamp);
                            }
                        }
                    }
                    //LOG(INFO) << "Create list " << new_key << " @ " << ind;
                    if (fl_bld->feats_size() > 0 || fl_bld->refs_size() > 0) {
                        store.set(new_key, fl_bld->SerializeAsString());
                    }                    
                }                                
            }            
        }
        store.sync_commit();

        locker.multi();
        for (size_t i = 1; i < btkeys.size(); ++i) {
            locker.lpush(btkeys[i], { ts });
        }
        auto exec_rep = locker.exec();
        locker.sync_commit();
            
        LOG(INFO) << "store_tiles() finish get feature list";
        for (size_t i = 0; i < flists.size(); i++) {
            auto tile = tiles.Get(i);
            auto tile_bld = resp_tiles->Mutable(i);
            for (auto& tit : flists[i]) {
                auto& fl = tile.feat_list(tit.first);
                auto& reply = tit.second;
                auto new_key = IDtoTileKey(branch, tile.tile_id(), fl.type(), timestamp);
                services::TileFeatureList pb;
                if (!reply || !reply->is_string()) {
                    LOG(ERROR) << "Failed to " << reply.cmds() << " returns " << (!reply ? -1 : reply->type());
                }
                else if (!pb.ParseFromArray(reply->c_str(), reply->size())) {
                    LOG(ERROR) << "Failed to parse " << new_key;
                }
                std::unordered_map<uint64_t, const RoadPB::FeatureID*> feat_map(pb.feats_size() * 2);
                for (int k = 0; k < pb.feats_size(); k++) {
                    auto& id = pb.feats(k).id();
                    auto it = feat_map.find(id.id());
                    if (it == feat_map.end()) {
                        feat_map[id.id()] = &id;
                    }
                    else if (it->second->version() < id.version()) {
                        it->second = &id;
                    }
                }
                std::unordered_map<std::string, const RoadPB::FeatureID*> ref_map(pb.refs_size() * 2);
                for (int k = 0; k < pb.refs_size(); k++) {
                    auto& id = pb.refs(k);
                    auto key = std::to_string(id.tileid()) + '|' + std::to_string(id.id());
                    auto it = ref_map.find(key);
                    if (it == ref_map.end()) {
                        ref_map[key] = &id;
                    }
                    else if (it->second->version() < id.version()) {
                        it->second = &id;
                    }
                }
                auto fl_bld = tile_bld->add_feat_list();
                fl_bld->set_type(fl.type());
                for (int k = 0; k < fl.feats_size(); k++) {
                    auto& rf = fl.feats(k);
                    if (rf.id().version() == 1) {
                        continue;
                    }
                    auto it = feat_map.find(rf.id().id());
                    if (it == feat_map.end() || it->second->version() < rf.id().version() || force_commit) {
                        if (!rf.id().has_is_deleted() || !rf.id().is_deleted()) {
                            if (write_data) {
                                auto key = IDtoKey(rf.id(), timestamp);
                                store.set(key, rf.data());
                            }
                            //LOG(INFO) << "load feature " << rf.id().ShortDebugString();
                        }
                        else {
                            //LOG(INFO) << "skip feature " << rf.id().ShortDebugString();
                        }
                        feat_map[rf.id().id()] = &rf.id();
                    }
                }
                for (auto& it : feat_map) {
                    auto id = fl_bld->add_feats()->mutable_id();
                    id->CopyFrom(*(it.second));
                    if (id->version() <= 0 || id->version() > timestamp) {
                        id->set_version(timestamp);
                    }
                }
                for (int k = 0; k < fl.refs_size(); k++) {
                    auto& id = fl.refs(k);
                    if (id.version() == 1) {
                        continue;
                    }
                    auto key = std::to_string(id.tileid()) + '|' + std::to_string(id.id());
                    auto it = ref_map.find(key);
                    if (it == ref_map.end() || it->second->version() < id.version() || force_commit) {
                        ref_map[key] = &id;
                    }
                }
                for (auto& it : ref_map) {
                    auto id = fl_bld->add_refs();
                    id->CopyFrom(*(it.second));
                    if (id->version() <= 0 || id->version() > timestamp) {
                        id->set_version(timestamp);
                    }
                }
                //LOG(INFO) << "Update list " << new_key << " @ " << ind;
                if (fl_bld->feats_size() > 0 || fl_bld->refs_size() > 0) {
                    store.set(new_key, fl_bld->SerializeAsString());
                }
            }
        }            
        store.commit();
        auto reply = exec_rep.get();
        LOG(INFO) << "store_tiles() finish multi for " << btkeys.size() - 1 << " keys";
        if (reply.is_null()) {
            store.sync_commit();                
            ret_msg << "Commit conflicted for " << ts << ", retry!" << std::endl;    
            LOG(ERROR) << "Commit conflicted for " << ts << ", retry!";
            return -1;               
        }
        else {
            std::vector<std::string> cmds = { "ZADD", "TimestampBranches", ts, branch };
            locker.send(cmds);

            cmds = { "ZADD" };
            std::string bk = "Tids_";
            bk.append(branch);
            cmds.push_back(bk);
            cmds.reserve(2 + tiles.size() * 2);
            for (int i = 0; i < tiles.size(); i++) {
                auto& tile = tiles.Get(i);
                cmds.push_back(ts);
                cmds.push_back(std::to_string(tile.tile_id()));
            }
            locker.send(cmds);

            locker.sync_commit();
            LOG(INFO) << "store_tiles() finish zadd to " << ts;
            store.sync_commit(); 
            return timestamp;
        }
    }

    void load_tiles(cpp_redis::client& locker, MultiConnClient& store, const std::string& branch,
                    const ::google::protobuf::RepeatedField<::google::protobuf::int32>& tile_ids,
                    const ::google::protobuf::RepeatedField<::google::protobuf::int32>& need_types,
                    int64_t max_ver, bool load_data, std::stringstream& ret_msg, 
                    const std::set<std::string>* elem_filter,
                    ::google::protobuf::RepeatedPtrField<::services::TileInfo>* tile_infos) {
        std::map<int, std::map<int, std::shared_ptr<services::TileFeatureList>>> tid2fls;
        std::map<int, std::map<int, int64_t>> tid2ver;
        std::vector<std::pair<int, int>> flids;
        flids.reserve(tile_ids.size() * need_types.size());
        std::vector<std::future<cpp_redis::reply>> lrange_reps;
        lrange_reps.reserve(tile_ids.size() * need_types.size());
        for (int i = 0; i < tile_ids.size(); i++) {
            int tileid = tile_ids[i];
            for (int j = 0; j < need_types[j]; j++) {
                auto tilekey = IDtoTileEnd(branch, tileid, need_types[j]);
                lrange_reps.push_back(locker.lrange(tilekey, 0, -1));
                flids.push_back(std::make_pair(tileid, need_types[j]));
            }
        }
        locker.sync_commit();
        LOG(INFO) << "load_tiles() finish lrange for " <<  lrange_reps.size() << " keys";
        std::vector<std::shared_ptr<services::TileFeatureList>> fl_ptrs;
        fl_ptrs.reserve(lrange_reps.size());
        std::vector<RedisReply> fl_reps;
        fl_reps.reserve(lrange_reps.size());
        std::vector<std::pair<int, int>> fl_keys;
        fl_keys.reserve(lrange_reps.size());
        for (size_t i = 0; i < lrange_reps.size(); ++i) {
            auto reply = lrange_reps[i].get();
            if (!reply.is_array()) {
                continue;
            }
            auto pr = flids[i];
            bool bf = false;
            std::string tilekey;
            auto& reps = reply.as_array();
            for (size_t k = 0; k < reps.size(); k++) {
                auto& rp = reps[k];
                if (rp.is_string()) {
                    int64_t v = atoll(rp.as_string().c_str());
                    if (v <= max_ver) {
                        tid2ver[pr.first][pr.second] = v;
                        tilekey = IDtoTileKey(branch, pr.first, pr.second, v);
                        bf = true;
                        break;
                    }
                }
            }
            if (!bf) {
                //LOG(ERROR) << "Failed to LRANGE ts for " << tilekey << " with " << max_ver;
                continue;
            }
            std::shared_ptr<services::TileFeatureList> flb(new services::TileFeatureList);
            flb->set_type(pr.second);
            fl_ptrs.push_back(flb);
            tid2fls[pr.first][pr.second] = flb;
            fl_reps.push_back(store.get(tilekey));
            fl_keys.push_back(pr);
        }
        store.commit();

        LOG(INFO) << "load_tiles() commit get feature_list for " << fl_reps.size() << " keys"; 
        std::vector<std::vector<RedisReply>> ft_reps(fl_reps.size());
        int feat_cnt = 0;
        for (size_t i = 0; i < fl_reps.size(); i++) {
            auto flb = fl_ptrs[i];
            auto& reply = fl_reps[i];
            auto pr = fl_keys[i];
            if (!reply || !reply->is_string()) {
                ret_msg << "Failed to " << reply.cmds() << std::endl;
                LOG(ERROR) << "Failed to " << reply.cmds();
                continue;
            }
            
            if (!flb->ParseFromArray(reply->c_str(), reply->size())) {
                LOG(ERROR) << "Fail to parse " << pr.second << " @ " << pr.first << " with "
                           << reply->size() << " Bytes";
                flb->Clear();
                continue;
            }
            if (flb->feats_size() <= 0 && flb->refs_size() <= 0) {
                LOG(ERROR) << "Skip empty feature_list " << pr.second << " @ " << pr.first;
                flb->Clear();
                continue;
            }
            if (!load_data) {
                continue;
            }
            ft_reps[i].reserve(flb->feats_size());
            for (int k = 0; k < flb->feats_size(); k++) {
                if (elem_filter != nullptr) {
                    auto ids = IDtoKey(flb->feats(k).id(), 0);
                    if (elem_filter->find(ids) == elem_filter->end()) {
                        ft_reps[i].emplace_back();
                        flb->mutable_feats(k)->mutable_id()->set_version(1);
                        continue;
                    }
                }
                if (!flb->feats(k).id().has_is_deleted() || !flb->feats(k).id().is_deleted()) {
                    auto key = IDtoKey(flb->feats(k).id(), tid2ver[pr.first][pr.second]);
                    ft_reps[i].push_back(store.get(key));
                }
                else {
                    ft_reps[i].emplace_back();
                    //LOG(INFO) << "skip deleted feature " << flb->feats(k).id().ShortDebugString();
                }
            }
            if (elem_filter != nullptr) {
                for (int k = 0; k < flb->refs_size(); k++) {
                    auto ids = IDtoKey(flb->refs(k), 0);
                    if (elem_filter->find(ids) == elem_filter->end()) {
                        flb->mutable_refs(k)->set_version(1);
                    }
                }
            }
            feat_cnt += flb->feats_size();
        }
        if (load_data) {
            store.commit();
        }
        LOG(INFO) << "load_tiles() commit get feature for " << feat_cnt << " features";
        for (size_t i = 0; i < ft_reps.size(); i++) {
            auto flb = fl_ptrs[i];
            for (size_t k = 0; k < ft_reps[i].size(); k++) {
                if (!flb->feats(k).id().has_is_deleted() || !flb->feats(k).id().is_deleted()
                        || flb->feats(k).id().version() > 1) {
                    auto& rep = ft_reps[i][k];
                    if (rep) {
                        if (rep->is_string()) {
                            flb->mutable_feats(k)->set_data(rep->c_str(), rep->size());
                        }
                        else {
                            LOG(ERROR) << "Failed to " << rep.cmds();
                        }
                    }
                }
            }
            //LOG(INFO) << "Load " << tilekey << " with " << flb->feats_size() << " feats, "
            //          << flb->refs_size() << " refs";
        }
        LOG(INFO) << "load_tiles() finish get feature for " << feat_cnt << " features"; 
        for (auto& tit : tid2fls) {
            int64_t ver = -1;
            for (auto& fit : tit.second) {
                auto& flb = fit.second;
                if (flb->feats_size() <= 0 && flb->refs_size() <= 0) {
                    flb.reset();
                }
                else {
                    ver = std::max(ver, tid2ver[tit.first][fit.first]);
                }
            }
            if (ver <= 0) {
                continue;
            }
            auto nt = tile_infos->Add();
            nt->set_tile_id(tit.first);
            nt->set_version(ver);
            for (auto& fit : tit.second) {
                auto& flb = fit.second;
                if (flb) {
                    nt->mutable_feat_list()->Add()->CopyFrom(*flb);
                }
            }
        }
        tid2fls.clear();
        tid2ver.clear();
    }

    void filter_refs(data_access_engine::ID2TileMap& tiles, std::set<int>& down_tids,
                     std::set<int>& ref_tids, std::set<std::string>& ref_fids, bool load_ref) {
        for (auto& tit : tiles) {
            auto tile = tit.second;
            if (!tile) {
                continue;
            }
            for (auto& l : tile->junctions()) {
                if (!load_ref) {
                    auto id = IDtoKey(*l->id());
                    if (ref_fids.count(id) <= 0) {
                        continue;
                    }
                }
                auto ms = l->contained_members();
                for (auto m : ms) {
                    if (!m || !m->id() || down_tids.count(m->id()->tileid()) > 0) {
                        continue;
                    }
                    ref_fids.insert(IDtoKey(*m->id()));
                    ref_tids.insert(m->id()->tileid());
                }
                /*auto rs = l->contained_references();
                for (auto r : rs) {
                    if (!r || !r->id() || down_tids.count(r->id()->tileid()) > 0) {
                        continue;
                    }
                    ref_fids.insert(IDtoKey(*r->id()));
                    ref_tids.insert(r->id()->tileid());
                }*/                
            }
        }
        for (auto& tit : tiles) {
            auto tile = tit.second;
            if (!tile) {
                continue;
            }
            for (auto& l : tile->lane_groups()) {
                if (!load_ref) {
                    auto id = IDtoKey(*l->id());
                    if (ref_fids.count(id) <= 0) {
                        continue;
                    }
                }
                auto ms = l->contained_members();
                for (auto m : ms) {
                    if (!m || !m->id() || down_tids.count(m->id()->tileid()) > 0) {
                        continue;
                    }
                    ref_fids.insert(IDtoKey(*m->id()));
                    ref_tids.insert(m->id()->tileid());
                }
                /*auto rs = l->contained_references();
                for (auto r : rs) {
                    if (!r || !r->id() || down_tids.count(r->id()->tileid()) > 0) {
                        continue;
                    }
                    ref_fids.insert(IDtoKey(*r->id()));
                    ref_tids.insert(r->id()->tileid());
                }*/
            }
        }
        for (auto& tit : tiles) {
            auto tile = tit.second;
            if (!tile) {
                continue;
            }
            for (auto& l : tile->lanes()) {
                if (!load_ref) {
                    auto id = IDtoKey(*l->id());
                    if (ref_fids.count(id) <= 0) {
                        continue;
                    }
                }
                auto ms = l->contained_members();
                for (auto m : ms) {
                    if (!m || !m->id() || down_tids.count(m->id()->tileid()) > 0) {
                        continue;
                    }
                    ref_fids.insert(IDtoKey(*m->id()));
                    ref_tids.insert(m->id()->tileid());
                }
                auto rs = l->contained_references();
                for (auto r : rs) {
                    if (!r || !r->id() || r->id()->type() == data_access_engine::LaneProxy::ELEM_ID_TYPE
                            || down_tids.count(r->id()->tileid()) > 0) {
                        continue;
                    }
                    ref_fids.insert(IDtoKey(*r->id()));
                    ref_tids.insert(r->id()->tileid());
                }
            }
        }        
        for (auto& tit : tiles) {
            auto tile = tit.second;
            if (!tile) {
                continue;
            }
            for (auto& l : tile->traffic_infos()) {
                if (!load_ref) {
                    auto id = IDtoKey(*l->id());
                    if (ref_fids.count(id) <= 0) {
                        continue;
                    }
                }
                auto rs = l->contained_references();
                for (auto r : rs) {
                    if (!r || !r->id() || r->id()->type() != data_access_engine::PositionObjectProxy::ELEM_ID_TYPE
                            || down_tids.count(r->id()->tileid()) > 0) {
                        continue;
                    }
                    ref_fids.insert(IDtoKey(*r->id()));
                    ref_tids.insert(r->id()->tileid());
                }
            }
        }

        if (!load_ref) {
            return;
        }
        for (auto& tit : tiles) {
            auto tile = tit.second;
            if (!tile) {
                continue;
            }
            for (auto& id : tile->link_refs()) {
                if (!id || down_tids.count(id->tileid()) > 0) {
                    continue;
                }
                ref_tids.insert(id->tileid());
                ref_fids.insert(IDtoKey(*id.id()));
            }
            for (auto& id : tile->node_refs()) {
                if (!id || down_tids.count(id->tileid())) {
                    continue;
                }
                ref_tids.insert(id->tileid());
                ref_fids.insert(IDtoKey(*id.id()));
            }
            for (auto& id : tile->lane_refs()) {
                if (!id || down_tids.count(id->tileid())) {
                    continue;
                }
                ref_tids.insert(id->tileid());
                ref_fids.insert(IDtoKey(*id.id()));
            }
            for (auto& id : tile->lane_boundary_refs()) {
                if (!id || down_tids.count(id->tileid())) {
                    continue;
                }
                ref_tids.insert(id->tileid());
                ref_fids.insert(IDtoKey(*id.id()));
            }
            for (auto& id : tile->lane_group_refs()) {
                if (!id || down_tids.count(id->tileid())) {
                    continue;
                }
                ref_tids.insert(id->tileid());
                ref_fids.insert(IDtoKey(*id.id()));
            }
            for (auto& id : tile->junction_refs()) {
                if (!id || down_tids.count(id->tileid())) {
                    continue;
                }
                ref_tids.insert(id->tileid());
                ref_fids.insert(IDtoKey(*id.id()));
            }
            for (auto& id : tile->traffic_info_refs()) {
                if (!id || down_tids.count(id->tileid())) {
                    continue;
                }
                ref_tids.insert(id->tileid());
                ref_fids.insert(IDtoKey(*id.id()));
            }
            for (auto& id : tile->position_object_refs()) {
                if (!id || down_tids.count(id->tileid())) {
                    continue;
                }
                ref_tids.insert(id->tileid());
                ref_fids.insert(IDtoKey(*id.id()));
            }
            for (auto& id : tile->road_boundary_refs()) {
                if (!id || down_tids.count(id->tileid())) {
                    continue;
                }
                ref_tids.insert(id->tileid());
                ref_fids.insert(IDtoKey(*id.id()));
            }
            for (auto& id : tile->data_quality_refs()) {
                if (!id || down_tids.count(id->tileid())) {
                    continue;
                }
                ref_tids.insert(id->tileid());
                ref_fids.insert(IDtoKey(*id.id()));
            }
            for (auto& id : tile->dynamic_refs()) {
                if (!id || down_tids.count(id->tileid())) {
                    continue;
                }
                ref_tids.insert(id->tileid());
                ref_fids.insert(IDtoKey(*id.id()));
            }            
        }
    }

    void load_tiles_with_ref(cpp_redis::client& locker, MultiConnClient& store, const std::string& branch,
                    const ::google::protobuf::RepeatedField<::google::protobuf::int32>& tile_ids,
                    const ::google::protobuf::RepeatedField<::google::protobuf::int32>& need_types,
                    int64_t max_ver, bool judge_editable, std::stringstream& ret_msg, 
                    ::google::protobuf::RepeatedPtrField<::services::TileInfo>* tile_infos) {
        load_tiles(locker, store, branch, tile_ids, need_types,
                            max_ver, true, ret_msg, nullptr, tile_infos);
        data_access_engine::ConfigAddress adr;
        data_access_engine::DAEInterface itf(&adr);
        itf.get_projection_helper()->set_destination_Proj4_string("+proj=geocent +datum=WGS84");
        data_access_engine::RoadGeometryManager mgr(&adr, &itf);
        std::set<std::string> ref_fids;
        std::set<int> down_tids;
        std::set<int> ref_tids;
        for (int i = 0; i < tile_infos->size(); i++) {
            auto& tile = tile_infos->Get(i);
            down_tids.insert(tile.tile_id());
            auto tp = mgr.get_road_tile(tile.tile_id());
            tp->from_message(&tile);                
        }
        mgr.resolve_tiles();
        std::set<int> init_tids = down_tids;
        data_access_engine::ID2TileMap tiles;
        mgr.get_road_tiles(tiles);
        filter_refs(tiles, down_tids, ref_tids, ref_fids, true);
        if (!ref_tids.empty()) {
            ::google::protobuf::RepeatedField<::google::protobuf::int32> tids;
            tids.Reserve(ref_tids.size());
            for (int id : ref_tids) {
                tids.Add(id);
            }
            int tile_base = tile_infos->size();
            load_tiles(locker, store, branch, tids, need_types,
                           max_ver, true, ret_msg, nullptr, tile_infos);
            tiles.clear();
            for (int ti = tile_base; ti < tile_infos->size(); ti++) {
                auto& tile = tile_infos->Get(ti);                    
                auto tp = mgr.get_road_tile(tile.tile_id());
                tp->from_message(&tile);
                tiles[tile.tile_id()] = tp;
            }
            mgr.resolve_tiles();
            ref_tids.clear();
            filter_refs(tiles, down_tids, ref_tids, ref_fids, false);
            for (int ti = tile_base; ti < tile_infos->size(); ti++) {
                auto tile = tile_infos->Mutable(ti);
                down_tids.insert(tile->tile_id());
                ref_tids.erase(tile->tile_id());
                for (auto& rf : *tile->mutable_feat_list()) {
                    for (auto& f : *rf.mutable_feats()) {
                        if (!f.has_data() || f.data().empty()) {
                            continue;
                        }
                        auto str = IDtoKey(f.id(), 0);
                        if (ref_fids.count(str) <= 0) {
                            f.clear_data();
                            f.mutable_id()->set_version(1);
                        }
                    }
                    for (auto& r : *rf.mutable_refs()) {
                        auto str = IDtoKey(r, 0);
                        if (ref_fids.count(str) <= 0) {
                            r.set_version(1);
                        }
                    }
                }
            }
            if (!ref_tids.empty()) {
                tids.Clear();
                for (int id : ref_tids) {
                    tids.Add(id);
                }
                tile_base = tile_infos->size();
                load_tiles(locker, store, branch, tids, need_types,
                           max_ver, true, ret_msg, &ref_fids, tile_infos);
                for (int ti = tile_base; ti < tile_infos->size(); ti++) {
                    auto& tile = tile_infos->Get(ti);
                    down_tids.insert(tile.tile_id());
                    auto tp = mgr.get_road_tile(tile.tile_id());
                    tp->from_message(&tile);
                    tiles[tile.tile_id()] = tp;                    
                }
                mgr.resolve_tiles();
            }
        }
        
        mgr.correct_tile_refs();
        tiles.clear();
        mgr.get_road_tiles(tiles);
        if (judge_editable) {
            tile_infos->Clear();
            mgr.judge_editable(init_tids);
            data_access_engine::TileInfoList ts;
            mgr.get_editable_tile_data(ts);
            for (auto tp : ts) {
                auto t = tile_infos->Add();
                tp->to_message(t);
            }
        }
        else {
            for (auto& tit : tiles) {
                if (down_tids.count(tit.first) > 0) {
                    continue;
                }
                auto t = tile_infos->Add();
                tit.second->to_message(t);
            }
        }
    }
};
};

class LinkExtProxyCreator : public data_access_engine::FeatureProxyCreator<data_access_engine::LinkProxy> {
public:
    virtual data_access_engine::LinkProxy* create() {
        return new data_access_engine::LinkExtProxy();
    }
    virtual data_access_engine::LinkProxy* create(data_access_engine::FeatureProxyBase* p) {
        return new data_access_engine::LinkExtProxy(p);
    }
};
class LaneExtProxyCreator : public data_access_engine::FeatureProxyCreator<data_access_engine::LaneProxy> {
public:
    virtual data_access_engine::LaneProxy* create() {
        return new data_access_engine::LaneExtProxy();
    }
    virtual data_access_engine::LaneProxy* create(data_access_engine::FeatureProxyBase* p) {
        return new data_access_engine::LaneExtProxy(p);
    }
};

int main(int argc, char* argv[]) {
    // Parse gflags. We recommend you to use gflags as well.
    GFLAGS_NS::ParseCommandLineFlags(&argc, &argv, true);

    delete data_access_engine::FeatureProxyBase::register_creator<data_access_engine::LinkProxy>(new LinkExtProxyCreator());
    delete data_access_engine::FeatureProxyBase::register_creator<data_access_engine::LaneProxy>(new LaneExtProxyCreator());

    brpc::FLAGS_max_body_size = 256 * 1024 * 1024;
    brpc::FLAGS_socket_max_unwritten_bytes = 256 * 1024 * 1024;
    //cpp_redis::active_logger = std::unique_ptr<cpp_redis::logger>(new cpp_redis::logger);
    
    // Generally you only need one Server.
    brpc::Server server;

    // Instance of your service.
    services::RoadServiceImpl road_service_impl;

    // Add the service into server. Notice the second parameter, because the
    // service is put on stack, we don't want server to delete it, otherwise
    // use brpc::SERVER_OWNS_SERVICE.
    if (server.AddService(&road_service_impl, 
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

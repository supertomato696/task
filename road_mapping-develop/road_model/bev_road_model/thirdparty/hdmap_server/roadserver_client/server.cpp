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

#include "services/roadserver.pb.h"

DEFINE_string(server, "localhost:8085", "IP Address of server");
DEFINE_int32(timeout_ms, 100000, "RPC timeout in milliseconds");
DEFINE_int32(max_retry, 10, "Max retries(not including the first RPC)");
DEFINE_int32(interval_ms, 1000, "Milliseconds between consecutive requests");
DEFINE_int64(reqver, 0, "max version retrieved");
DEFINE_string(basedir, "./", "base dir for download/upload files");

DEFINE_int32(port, 8081, "TCP Port of this server");
DEFINE_int32(idle_timeout_s, -1, "Connection will be closed if there is no "
             "read/write operations during the last `idle_timeout_s'");
DEFINE_int32(logoff_ms, 2000, "Maximum duration of server's LOGOFF state "
             "(waiting for client to close connection before server stops)");

namespace brpc {
DECLARE_uint64(max_body_size);
}

namespace services {
class RoadServiceImpl : public RoadService {
public:
    RoadServiceImpl() {};
    virtual ~RoadServiceImpl() {};
    virtual void UploadTile(::google::protobuf::RpcController* controller,
                       const ::services::TileUploadRequest* request,
                       ::services::TileUploadResponse* response,
                       ::google::protobuf::Closure* done) {
        // This object helps you to call done->Run() in RAII style. If you need
        // to process the request asynchronously, pass done_guard.release().
        brpc::ClosureGuard done_guard(done);

        brpc::Controller* cntl =
            static_cast<brpc::Controller*>(controller);

        // The purpose of following logs is to help you to understand
        // how clients interact with servers more intuitively. You should 
        // remove these logs in performance-sensitive servers.
        LOG(INFO) << "Received request[log_id=" << cntl->log_id() 
                  << "] from " << cntl->remote_side() 
                  << " to " << cntl->local_side();

        brpc::Channel channel;
        brpc::ChannelOptions options;
        options.protocol = "baidu_std";
        options.connection_type = "";
        options.timeout_ms = FLAGS_timeout_ms /*milliseconds*/;
        options.connect_timeout_ms = FLAGS_timeout_ms;
        options.max_retry = FLAGS_max_retry;
        if (channel.Init(FLAGS_server.c_str(), "", &options) != 0) {
            LOG(ERROR) << "Fail to initialize channel";
            cntl->SetFailed("Fail to initialize channel");
            return;
        }

        services::RoadService_Stub stub(&channel);
        brpc::Controller ctl;
        stub.UploadTile(&ctl, request, response, nullptr);
        if (!ctl.Failed()) {
            LOG(INFO) << "UploadTile() response from " << ctl.remote_side()
                      << " to " << ctl.local_side()
                      << ": " << response->ret().message() << " (code="
                      << response->ret().code() << ")"
                      << " latency=" << ctl.latency_us() << "us";
        }
        else {
            LOG(WARNING) << "Failed to UploadTile() " << ctl.ErrorText();
            cntl->SetFailed(ctl.ErrorText());
        }
    }

    virtual void DownloadTile(::google::protobuf::RpcController* controller,
                       const ::services::TileDownloadRequest* request,
                       ::services::TileDownloadResponse* response,
                       ::google::protobuf::Closure* done) {
        // This object helps you to call done->Run() in RAII style. If you need
        // to process the request asynchronously, pass done_guard.release().
        brpc::ClosureGuard done_guard(done);

        brpc::Controller* cntl =
            static_cast<brpc::Controller*>(controller);

        // The purpose of following logs is to help you to understand
        // how clients interact with servers more intuitively. You should 
        // remove these logs in performance-sensitive servers.
        LOG(INFO) << "Received request[log_id=" << cntl->log_id() 
                  << "] from " << cntl->remote_side() 
                  << " to " << cntl->local_side();

        brpc::Channel channel;
        brpc::ChannelOptions options;
        options.protocol = "baidu_std";
        options.connection_type = "";
        options.timeout_ms = FLAGS_timeout_ms /*milliseconds*/;
        options.connect_timeout_ms = FLAGS_timeout_ms;
        options.max_retry = FLAGS_max_retry;
        if (channel.Init(FLAGS_server.c_str(), "", &options) != 0) {
            LOG(ERROR) << "Fail to initialize channel";
            cntl->SetFailed("Fail to initialize channel");
            return;
        }

        services::RoadService_Stub stub(&channel);
        brpc::Controller ctl;
        stub.DownloadTile(&ctl, request, response, nullptr);
        if (!ctl.Failed()) {
            LOG(INFO) << "DownloadTile() response from " << ctl.remote_side()
                      << " to " << ctl.local_side()
                      << ": " << response->ret().message() << " (code="
                      << response->ret().code() << ")"
                      << " latency=" << ctl.latency_us() << "us";
        }
        else {
            LOG(WARNING) << "Failed to DownloadTile() " << ctl.ErrorText();
            cntl->SetFailed(ctl.ErrorText());
        }
    }
};
}  // namespace example

int main(int argc, char* argv[]) {
    // Parse gflags. We recommend you to use gflags as well.
    GFLAGS_NS::ParseCommandLineFlags(&argc, &argv, true);

    brpc::FLAGS_max_body_size = 256 * 1024 * 1024;
    
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
    if (server.Start(FLAGS_port, &options) != 0) {
        LOG(ERROR) << "Fail to start RoadServer";
        return -1;
    }

    // Wait until Ctrl-C is pressed, then Stop() and Join() the server.
    server.RunUntilAskedToQuit();
    return 0;
}

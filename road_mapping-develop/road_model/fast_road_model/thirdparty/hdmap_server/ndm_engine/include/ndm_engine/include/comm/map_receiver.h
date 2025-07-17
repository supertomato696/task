/**
 * @file comm.h
 * @author Zeng Siyu(siyu.zeng@horizon.ai)
 * @brief  communication Interface
 * @version 3.2
 * @date 2020-07-30
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef COMM_MAP_RECEIVER_H_
#define COMM_MAP_RECEIVER_H_

#include <locale.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <zmq.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <chrono>

#include "../../proto/location.pb.h"
#include "../../proto/ndm.pb.h"
#include "../interface/data_type.h"
#include "../interface/io.h"
#include "../interface/util.h"
#include "../interface/map_engine.h"
#include "../comm/reliable_transfer.h"

//最大缓存信息数
#define MAX_MSG_QUEUE_LEN 3

namespace map_interface {

class ReliableReceiver;

class MapReceiver {
 public:
  MapReceiver();
  ~MapReceiver();

  static MapReceiver& GetInstance() {
    static MapReceiver instance_;
    return instance_;
  }

  bool Start(std::string ip);
  void Run();
  // 初始化ndm消息接收socket
  bool InitSocket();
  void FeedEngine(map_interface::MapEngine* engine);

  bool PopBackReceiveMap(std::shared_ptr<ndm_proto::MapEnvMsg> &msg);
  bool PopBackReceiveNav(std::shared_ptr<ndm_proto::NavigationMsg> &msg);
  bool PopBackSendMap(std::shared_ptr<ndm_proto::MapEnvMsg> &msg);
  bool PopBackSendNav(std::shared_ptr<ndm_proto::NavigationMsg> &msg);

private:
  bool ProcessNavData(std::shared_ptr<ndm_proto::NavigationMsg> nav_msg);
  bool ProcessDestination(std::shared_ptr<ndm_proto::NavigationMsg> nav_msg,
                          std::string &lane_dest);
  void ProcessData();
  bool TimeSynchronization();

  void PushFrontReceiveMap(std::shared_ptr<ndm_proto::MapEnvMsg> msg);
  void PushFrontReceiveNav(std::shared_ptr<ndm_proto::NavigationMsg> msg);
  void PushFrontSendMap(std::shared_ptr<ndm_proto::MapEnvMsg> msg);
  void PushFrontSendNav(std::shared_ptr<ndm_proto::NavigationMsg> msg);

 private:
  map_interface::MapEngine* engine_;

  // front 最新, end最旧
  std::list<std::shared_ptr<ndm_proto::MapEnvMsg> > receive_map_;
  std::list<std::shared_ptr<ndm_proto::NavigationMsg> > receive_nav_;

  std::list<std::shared_ptr<ndm_proto::MapEnvMsg> > send_map_;
  std::list<std::shared_ptr<ndm_proto::NavigationMsg> > send_nav_;

  bool state_recived_map_;
  bool state_run_thread_;

  // ndm消息接收socket
  std::string ip_;
  void *ndm_context_;
  void *ndm_requester_;
  // 接收失败次数, 大于200次后, 尝试重连
  int recv_fail_count_;

  std::thread mthread_;

  // int database_idx_;
  // std::vector<std::shared_ptr<ndm_proto::Map>> map_databse_;

  std::mutex map_lock_;
  std::mutex index_lock_;
  std::mutex route_lock_;

  std::mutex receive_map_lock_;
  std::mutex receive_nav_lock_;
  std::mutex send_map_lock_;
  std::mutex send_nav_lock_;
};
}  // namespace map_interface

#endif  // COMM_MAP_RECEIVER_H_

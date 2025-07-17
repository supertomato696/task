/*
 *   Copyright (C) Horizon Robotics 2019 All rights reserved.
 */

#ifndef COMM_NAV_RECEIVER_H_
#define COMM_NAV_RECEIVER_H_

#include "../interface/util.h"
#include "../interface/io.h"
#include "../interface/map_engine.h"
#include "../../proto/ndm.pb.h"
#include <zmq.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

//最大缓存信息数
#define IMG_QUEUE_LEN 3
#define LOC_CACHE_NUM 50

namespace map_interface {
class NavReceiver {
 public:
  NavReceiver();
  ~NavReceiver();

  bool init(const char *endpoint);

  void recvZmqMsg();
  void recvImgData(ndm_proto::Image *img);

  void processNavData(std::shared_ptr<ndm_proto::NavigationMsg> &ndm_proto);
  void FeedEngine(map_interface::MapEngine* engine);

  void stop();
  bool initSocket();
  void loop();

  bool PopBackSendNav(std::shared_ptr<ndm_proto::NavigationMsg> &msg);

private:
  void PushFrontSendNav(std::shared_ptr<ndm_proto::NavigationMsg> msg);

  void *context_;
  void *requester_;
  char m_endpoint_[100];

  int recv_fail_count_;

  std::thread recv_thread_;
  std::mutex msg_queue_mutex_;

  std::mutex send_nav_lock_;

  bool run_recv_thread_;  //控制线程关闭

  std::list<std::shared_ptr<ndm_proto::NavigationMsg> > send_nav_;

  map_interface::MapEngine* engine_;
};
}  // namespace map_interface

#endif  // COMM_NAV_RECEIVER_H_


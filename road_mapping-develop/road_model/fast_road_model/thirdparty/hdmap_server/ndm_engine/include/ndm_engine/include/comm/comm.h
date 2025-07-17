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
#ifndef INTERFACE_COMM_H_
#define INTERFACE_COMM_H_

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

//最大缓存信息数
#define COMM_MAX_MSG_QUEUE_LEN 5

namespace map_interface {

struct Msg {
  /* data */
  std::string type_;
  std::vector<std::string> vec_data_;
};

class Comm {
 public:
  Comm();
  ~Comm();

  static Comm& GetInstance() {
    static Comm comm_instance;
    return comm_instance;
  }

  bool Start();
  void Stop();

  void PublishMsg(const std::string &type, const std::string &data);
  void PublishMsg(const std::string& type, std::vector<std::string>& vec_data);
  void UploadThreadProc();

 private:
  int ZmqSend(void* socket, const char* string, const bool &sendmore);

 private:
  std::list<Msg> msg_queue_;  // front 最新, end最旧

  std::thread comm_thread_;
  std::mutex queue_lock_;

  bool run_comm_thread_;  //控制线程关闭

  void* context_;
  void* publisher_;
};
}  // namespace map_interface

#endif  // INTERFACE_COMM_H_

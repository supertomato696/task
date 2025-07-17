/*
 * @Date: 2020-12-07 16:05:27
 * @Copyright: 2020, Horizon Robotics
 * @Author: (gang01.xu) gang01.xu@horizon.ai
 * @LastEditTime: 2020-12-09 15:13:40
 * @LastEditors: (gang01.xu) gang01.xu@horizon.ai
 */
#pragma once
#include <string.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>
#include <condition_variable>
#include <glog/logging.h>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "assert.h"
#include "zmq.h"

namespace map_interface {

class ReliableSender {
 private:
  int recv_timeout_;
  int max_retry_cnts_;
  void* m_context_;
  void* m_requester_;
  char m_endpoint_[100];

 public:
  explicit ReliableSender(int recv_timeout, int max_retry_cnts,
                          const char* endpoint);
  ~ReliableSender();
  bool SendData(const void* data, int length) const;
  ReliableSender(const ReliableSender&) = delete;
  ReliableSender(ReliableSender&&) = delete;
  ReliableSender& operator=(const ReliableSender&) = delete;
  ReliableSender& operator=(ReliableSender&&) = delete;
};

class ReliableReceiver {
 private:
  int recv_timeout_;
  void* m_context_;
  void* m_requester_;
  char m_endpoint_[100];

 public:
  explicit ReliableReceiver(const char* endpoint);
  ~ReliableReceiver();
  std::string ReceiveData() const;
  ReliableReceiver(const ReliableReceiver&) = delete;
  ReliableReceiver(ReliableReceiver&&) = delete;
  ReliableReceiver& operator=(const ReliableReceiver&) = delete;
  ReliableReceiver& operator=(ReliableReceiver&&) = delete;
};

class ReliableRequester {
 private:
  int request_timeout_;
  void* m_context_;
  void* m_requester_;
  char m_endpoint_[100];

 public:
  ReliableRequester(int recv_timeout, const char* endpoint);
  ~ReliableRequester();
  std::string RequestData() const;
  ReliableRequester(const ReliableRequester&) = delete;
  ReliableRequester& operator=(const ReliableRequester&) = delete;
  ReliableRequester(ReliableRequester&&) = delete;
  ReliableRequester& operator=(ReliableRequester&&) = delete;
};

class ReliableResponser {
 private:
  void* m_context_;
  void* m_requester_;
  char m_endpoint_[100];
  std::string resp_;
  mutable std::mutex resp_mutex_;
  std::condition_variable resp_cond_;
  void RequestReceiver();
  // int64_t request_timestamp_;

  // struct Response {
  //   int64_t timestamp_;
  //   std::string data_;
  // };

 public:
  explicit ReliableResponser(const char* endpoint);
  ~ReliableResponser();
  void SetData(std::string s);

  ReliableResponser(const ReliableResponser&) = delete;
  ReliableResponser& operator=(const ReliableResponser&) = delete;
  ReliableResponser(ReliableResponser&&) = delete;
  ReliableResponser& operator=(ReliableResponser&&) = delete;
};
}  // namespace map_interface

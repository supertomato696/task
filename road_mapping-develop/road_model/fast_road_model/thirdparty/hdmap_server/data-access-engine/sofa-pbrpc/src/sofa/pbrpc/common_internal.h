// Copyright (c) 2014 Baidu.com, Inc. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef _SOFA_PBRPC_COMMON_INTERNAL_H_
#define _SOFA_PBRPC_COMMON_INTERNAL_H_

#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <sofa/pbrpc/common.h>
#include <sofa/pbrpc/atomic.h>
#include <sofa/pbrpc/counter.h>
#include <sofa/pbrpc/func_tracer.h>
#include <sofa/pbrpc/io_service.h>
#include <sofa/pbrpc/locks.h>
#include <sofa/pbrpc/ptime.h>

namespace sofa {
namespace pbrpc {

class RpcControllerImpl;
typedef std::shared_ptr<RpcControllerImpl> RpcControllerImplPtr;
typedef std::weak_ptr<RpcControllerImpl> RpcControllerImplWPtr;

class RpcChannelImpl;
typedef std::shared_ptr<RpcChannelImpl> RpcChannelImplPtr;

class RpcClientImpl;
typedef std::shared_ptr<RpcClientImpl> RpcClientImplPtr;

class RpcServerImpl;
typedef std::shared_ptr<RpcServerImpl> RpcServerImplPtr;
typedef std::weak_ptr<RpcServerImpl> RpcServerImplWPtr;

class RpcClientStream;
typedef std::shared_ptr<RpcClientStream> RpcClientStreamPtr;
typedef std::weak_ptr<RpcClientStream> RpcClientStreamWPtr;

class RpcServerStream;
typedef std::shared_ptr<RpcServerStream> RpcServerStreamPtr;
typedef std::weak_ptr<RpcServerStream> RpcServerStreamWPtr;

class RpcListener;
typedef std::shared_ptr<RpcListener> RpcListenerPtr;

class TimerWorker;
typedef std::shared_ptr<TimerWorker> TimerWorkerPtr;

class RpcTimeoutManager;
typedef std::shared_ptr<RpcTimeoutManager> RpcTimeoutManagerPtr;

class ThreadGroup;
typedef std::shared_ptr<ThreadGroup> ThreadGroupPtr;

class ServicePool;
typedef std::shared_ptr<ServicePool> ServicePoolPtr;
typedef std::weak_ptr<ServicePool> ServicePoolWPtr;

class FlowController;
typedef std::shared_ptr<FlowController> FlowControllerPtr;

class WaitEvent;
typedef std::shared_ptr<WaitEvent> WaitEventPtr;

class IOServicePool;
typedef std::shared_ptr<IOServicePool> IOServicePoolPtr;

class WebService;
typedef std::shared_ptr<WebService> WebServicePtr;

#define SOFA_PBRPC_DECLARE_RESOURCE_COUNTER(name_) \
    extern sofa::pbrpc::AtomicCounter g_sofa_counter_##name_
#define SOFA_PBRPC_DEFINE_RESOURCE_COUNTER(name_) \
    sofa::pbrpc::AtomicCounter g_sofa_counter_##name_(0)
#define SOFA_PBRPC_INC_RESOURCE_COUNTER(name_) \
    ++g_sofa_counter_##name_
#define SOFA_PBRPC_DEC_RESOURCE_COUNTER(name_) \
    --g_sofa_counter_##name_
#define SOFA_PBRPC_GET_RESOURCE_COUNTER(name_) \
    static_cast<int>(g_sofa_counter_##name_)

SOFA_PBRPC_DECLARE_RESOURCE_COUNTER(RpcByteStream);
SOFA_PBRPC_DECLARE_RESOURCE_COUNTER(RpcListener);

// Use for affecting global/static variables' construct/destruct order.
inline void touch_boost_error_category()
{
    (void)boost::system::system_category();
    (void)boost::system::generic_category();
    (void)boost::asio::error::get_addrinfo_category();
    (void)boost::asio::error::get_misc_category();
    (void)boost::asio::error::get_netdb_category();
}

} // namespace pbrpc
} // namespace sofa

#endif // _SOFA_PBRPC_COMMON_INTERNAL_H_

/* vim: set ts=4 sw=4 sts=4 tw=100 */

// Copyright (c) 2014 Baidu.com, Inc. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// Author: qinzuoyan01@baidu.com (Qin Zuoyan)

#ifndef _SOFA_PBRPC_CONDITION_VARIABLE_H_
#define _SOFA_PBRPC_CONDITION_VARIABLE_H_

#include <errno.h>
#include <condition_variable>
#include <sofa/pbrpc/mutex_lock.h>

namespace sofa {
namespace pbrpc {

class ConditionVariable {
public:
    ConditionVariable() {
    }
    ~ConditionVariable() {
    }
    void wait(MutexLock& mutex) {
        std::unique_lock<std::mutex> lock(mutex._lock);
        _cond.wait(lock);
    }
    bool wait(MutexLock& mutex, int64 timeout_in_ms) {
        if (timeout_in_ms < 0) {
            wait(mutex);
            return true;
        }

        std::unique_lock<std::mutex> lock(mutex._lock);
        _cond.wait_for(lock, std::chrono::microseconds(timeout_in_ms));
    }
    void signal() {
        _cond.notify_one();
    }
    void broadcast() {
        _cond.notify_all();
    }
private:
    std::condition_variable _cond;
};

} // namespace pbrpc
} // namespace sofa

#endif // _SOFA_PBRPC_CONDITION_VARIABLE_H_

/* vim: set ts=4 sw=4 sts=4 tw=100 */

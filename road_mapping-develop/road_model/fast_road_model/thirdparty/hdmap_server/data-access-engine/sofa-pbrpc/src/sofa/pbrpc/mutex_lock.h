// Copyright (c) 2014 Baidu.com, Inc. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// Author: qinzuoyan01@baidu.com (Qin Zuoyan)

#ifndef _SOFA_PBRPC_MUTEX_LOCK_H_
#define _SOFA_PBRPC_MUTEX_LOCK_H_

#include <mutex>

namespace sofa {
namespace pbrpc {
class ConditionVariable;

class MutexLock {
public:
    MutexLock() {
    }
    ~MutexLock() {
    }
    void lock() {
        _lock.lock();
    }
    void unlock() {
        _lock.unlock();
    }
private:
    friend class ConditionVariable;
    std::mutex _lock;
};

} // namespace pbrpc
} // namespace sofa

#endif // _SOFA_PBRPC_MUTEX_LOCK_H_

/* vim: set ts=4 sw=4 sts=4 tw=100 */

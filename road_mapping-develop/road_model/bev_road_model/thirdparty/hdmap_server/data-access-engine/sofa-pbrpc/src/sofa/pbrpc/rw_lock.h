// Copyright (c) 2014 Baidu.com, Inc. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// Author: qinzuoyan01@baidu.com (Qin Zuoyan)

#ifndef _SOFA_PBRPC_RW_LOCK_H_
#define _SOFA_PBRPC_RW_LOCK_H_

#include <boost/thread/shared_mutex.hpp>

namespace sofa {
namespace pbrpc {

class RWLock {
public:
    RWLock() {
    }
    ~RWLock() {
    }
    void lock() {
        _lock.lock();
    }
    void lock_shared() {
        _lock.lock_shared();
    }
    void unlock_unique() {
        _lock.unlock();
    }
    void unlock_shared() {
        _lock.unlock_shared();
    }
private:
    boost::shared_mutex _lock;
};

class ReadLocker {
public:
    explicit ReadLocker(RWLock* lock) : _lock(lock) {
        _lock->lock_shared();
    }
    ~ReadLocker() {
        _lock->unlock_shared();
    }
private:
    RWLock* _lock;
};
class WriteLocker {
public:
    explicit WriteLocker(RWLock* lock) : _lock(lock) {
        _lock->lock();
    }
    ~WriteLocker() {
        _lock->unlock_unique();
    }
private:
    RWLock* _lock;
};

} // namespace pbrpc
} // namespace sofa

#endif // _SOFA_PBRPC_RW_LOCK_H_

/* vim: set ts=4 sw=4 sts=4 tw=100 */

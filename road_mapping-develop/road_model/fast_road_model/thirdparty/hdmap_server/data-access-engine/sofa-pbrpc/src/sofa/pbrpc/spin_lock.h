// Copyright (c) 2014 Baidu.com, Inc. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef _SOFA_PBRPC_SPIN_LOCK_H_
#define _SOFA_PBRPC_SPIN_LOCK_H_

#ifdef __APPLE__
#include <libkern/OSAtomic.h>

namespace sofa {
namespace pbrpc {

class SpinLock
{
public:
    SpinLock(): _lock(0) { }
    ~SpinLock() { }
    void lock() { OSSpinLockLock(&_lock); }
    bool try_lock() { return OSSpinLockTry(&_lock); }
    void unlock() { OSSpinLockUnlock(&_lock); }

private:
    OSSpinLock _lock;
}; // class SpinLock

}
}

#else

#include <atomic>
#include <thread>

namespace sofa {
namespace pbrpc {

class SpinLock {
public:
    SpinLock() {}
    ~SpinLock() {
        while (_lock.load()) {
            std::this_thread::yield();
        }
    }
    void lock() {
        int u = 0;

        do {
            u = 0;
            _lock.compare_exchange_strong(u, 1);
        } while (u);
    }
    bool try_lock() {
        int u = 0;
        _lock.compare_exchange_strong(u, 1);
        return !u;
    }
    void unlock() {
        _lock.store(0);
    }

private:
    //pthread_spinlock_t _lock;
    std::atomic<int> _lock;
}; // class SpinLock

} // namespace pbrpc
} // namespace sofa
#endif

#endif // _SOFA_PBRPC_SPIN_LOCK_H_

/* vim: set ts=4 sw=4 sts=4 tw=100 */

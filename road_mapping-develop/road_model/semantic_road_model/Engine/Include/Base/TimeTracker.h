/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:TimeTracker.h
简要描述: 时间追踪类，主要用于测试某一个方法或者过程到底花费了多少时间
主要用法：
    1. Start();
    2. Elapsed();
    3. Stop();
******************************************************************/

#ifndef ENGINE_BASE_TIMETRACKER_H_
#define ENGINE_BASE_TIMETRACKER_H_

#include "Base.h"
#include "Base/Types.h"

namespace Engine
{
    namespace Base
    {
        class Base_API TimeTracker
        {
        public:
            TimeTracker();
            ~TimeTracker();

        public:
            // 开始计时
            // 返回值是毫秒数
            Base::UInt64 Start();

            // 结束计时
            // 返回值是毫秒数
            Base::UInt64 Stop();

            // 统计从计时开始到现在耗费的时间
            // 返回值是毫秒数
            Base::UInt64 Elapsed();

            // 返回当前毫秒数
            Base::UInt64 Now();

        protected:
            Base::UInt64 m_u64Now;
        };
    }
}

#endif // ENGINE_BASE_TIMETRACKER_H_
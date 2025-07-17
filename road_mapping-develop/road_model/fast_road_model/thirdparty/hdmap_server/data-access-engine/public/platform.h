#pragma once

#if defined(USE_GLOG)
#include <glog/logging.h>
#define LOG_BEGIN(LEVEL) LOG( LEVEL )
#define COMPACT_GOOGLE_LOG_DEBUG COMPACT_GOOGLE_LOG_INFO
#define LOG_END
#elif defined(USE_XLOG)
#include <sstream>
#include "log.h"
#undef min
#undef max
#define LOG_CLASS_FATAL LOG_CLASS_CRITICAL
#define LOG_BEGIN(LEVEL) do { LogClass xlog_log_class = LOG_CLASS_##LEVEL; std::stringstream xlog_tmp_sstm; xlog_tmp_sstm.precision(10); xlog_tmp_sstm
#define LOG_END XLog(xlog_log_class, XTAG(L"DataAccessEngine"), L"%S", xlog_tmp_sstm.str().c_str()); } while (0)
#else
#include <sstream>
#define LOG_BEGIN(LEVEL) do { std::stringstream null_tmp_sstm; null_tmp_sstm
#define LOG_END } while (0)
#endif


#pragma once

#include <string.h>

namespace fsdmap {

extern int SUCC;
extern int FAIL;

}

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define MAP_FIND(map, key) \
    (map.find(key) != map.end())

#define MAP_NOT_FIND(map, key) \
    (map.find(key) == map.end())

#define VEC_FIND(vec, key) \
    (std::find(vec.begin(), vec.end(), key) != vec.end())

#define VEC_PUSH_ALL(vec1, vec2) \
    (vec1.insert(vec1.end(), vec2.begin(), vec2.end()));

#define VEC_INSERT(vec1, ele) \
    (vec1.insert(vec1.begin(), ele));

#define VEC_INSERT_INDEX(vec1, index, ele) \
    (vec1.insert(vec1.begin() + index, ele));

#define VEC_INSERT_ALL(vec1, vec2) \
    (vec1.insert(vec1.begin(), vec2.begin(), vec2.end()));

#define VEC_ERASE(vec, index) \
    (vec.erase(vec.begin() + index)); \
    --index;

#define VEC_ERASE_ALL(vec, index) \
    (vec.erase(vec.begin() + index, vec.end()))

#define SORT(vec, fun) \
    (std::sort(vec.begin(), vec.end(), fun))

#define SUM(vec, start) \
    (std::accumulate(vec.begin(), vec.end(), 0))

#define LOG_POINT(LOG_LEVEL, point, msg, arg...) do { \
    std::string log_msg = utils::fmt(msg, ##arg); \
    LOG_##LOG_LEVEL("{}[xy={:.2f},{:.2f}]", log_msg, point.x(), point.y()); \
} while (0)

#define LOG_POINT2(LOG_LEVEL, point1, point2, msg, arg...) do { \
    std::string log_msg = utils::fmt(msg, ##arg); \
    LOG_##LOG_LEVEL("{}[xy1={:.2f},{:.2f},xy2={:.2f},{:.2f}]", \
            log_msg, point1.x(), point1.y(), point2.x(), point2.y()); \
} while (0)

#define DLOG_POINT(point, msg, arg...) do { \
    if (session->enable_debug_pos) { \
        LOG_POINT(DEBUG, point, msg, ##arg); \
    } \
} while (0)

#define DLOG_POINT2(point1, point2, msg, arg...) do { \
    if (session->enable_debug_pos) { \
        LOG_POINT2(DEBUG, point1, point2, msg, ##arg); \
    } \
} while (0)

#define FLOG_POINT(point, msg, arg...) do { \
    LOG_POINT(ERROR, point, msg, ##arg); \
} while (0)

#define FLOG_POINT2(point1, point2, msg, arg...) do { \
    LOG_POINT2(ERROR, point1, point2, msg, ##arg); \
} while (0)





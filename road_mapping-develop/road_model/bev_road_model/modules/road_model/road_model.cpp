


// #include <memory>
// #include <fstream>
// 
// #include <fcntl.h>
// #include <stdio.h>
#include "road_model.h"

namespace fsdmap {
namespace road_model {

int RoadModelProcess::registe_all_session() {
    if (registe_session(&_road_model_session_mgr) != fsdmap::SUCC) {
        return fsdmap::FAIL;
    }
    return fsdmap::SUCC;
}

int RoadModelProcess::road_model() {
    auto session = _road_model_session_mgr.get_instance();
    session->_session_mgr = &_road_model_session_mgr;
    if (session->init() != fsdmap::SUCC) {
        CLOG_ERROR("failed to init road_model session");
    }
    // session->thread_pool = _road_model_session_mgr.thread_pool.get();
    // _data_processer->_thread_pool = _road_model_session_mgr.thread_pool.get();
    session->thread_pool = _data_processer->_thread_pool.get();
    session->_logger = _logger;
    session->data_processer = _data_processer;
    session->err_code = fsdmap::FAIL;
    // session->set_debug_issue_vec(NULL);
    if (session->proc() != fsdmap::SUCC) {
        if (session->err_code == fsdmap::FAIL) {
            return 1;
        }
        return session->err_code;
    }
    return fsdmap::SUCC;
}

}
}
/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */

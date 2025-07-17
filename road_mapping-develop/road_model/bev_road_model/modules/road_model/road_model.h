

#pragma once

#include "utils/process_frame.h"
#include "road_model_session_data.h"
#include "dao/data_processer.h"

namespace fsdmap {
namespace road_model {

class RoadModelProcess :
    public fsdmap::process_frame::ProcessFrame {

public:
    RoadModelProcess() {};
    virtual ~RoadModelProcess() {}

    virtual int road_model();

    virtual void set_data_processer(dao::DataProcessorBase* dp) {
        this->_data_processer = dp;
    }

    virtual void set_logger(fsdmap::utils::Logger* logger) {
        this->_logger = logger;
    }

public: // ��ܽӿ�
    virtual const char * servername() {
        return "road_model";
    }

    virtual int registe_all_session();

private:
    RoadModelSessionManager _road_model_session_mgr;
    dao::DataProcessorBase* _data_processer;
    fsdmap::utils::Logger* _logger;
};

}
}

#include "data_access_engine.h"
#include "dao/config_address.h"
#include "public/platform.h"
#include "manager/road_geometry_mgr.h"
#include "utils/log_util.h"
namespace data_access_engine
{
std::unique_ptr<DAEInterface> DAEInterface::s_dae_interface;    
std::recursive_mutex DAEInterface::s_interface_mutex;

DAEInterface::DAEInterface() { _config = ConfigAddress::get_instance(); };
DAEInterface::DAEInterface(ConfigAddress* config) {
    if (config == nullptr) {
        LOG_BEGIN(INFO) << "Construct DAEInterface with default instance of ConfigAddress"; LOG_END;
        _config = ConfigAddress::get_instance();
    }
    else {
        LOG_BEGIN(INFO) << "Construct DAEInterface with ConfigAddress " << config; LOG_END;
        _config = config;
    }
}
DAEInterface::~DAEInterface() {
    if (_road_geometry_manager) {
        _road_geometry_manager->clear_all();
        _road_geometry_manager.reset();
        //_road_geometry_manager.release();
    }
    if (_projection_helper) {
        _projection_helper.reset();
        //_projection_helper.release();
    }    
};

DAEInterface* DAEInterface::get_instance() {
    std::lock_guard<std::recursive_mutex> locker(s_interface_mutex);
    if (!s_dae_interface) {
        LOG_INFO("Construct default instance of DAEInterface");
        // LOG_BEGIN(INFO) << "Construct default instance of DAEInterface"; LOG_END;
        s_dae_interface.reset(new DAEInterface);
    }
    return s_dae_interface.get();
};

RoadGeometryManager* DAEInterface::get_road_geometry_manager() {
    std::lock_guard<std::recursive_mutex> locker(s_interface_mutex);
    if (!_road_geometry_manager) {
        LOG_INFO("Construct RoadGeometrymanager for DAEInterface get_road_geometry_manager ");
        // LOG_BEGIN(INFO) << "Construct RoadGeometrymanager for DAEInterface " << this; LOG_END;
        _road_geometry_manager.reset(new RoadGeometryManager(_config, this));
    }
    return _road_geometry_manager.get();
};

ProjectionHelper<15>* DAEInterface::get_projection_helper() {
    std::lock_guard<std::recursive_mutex> locker(s_interface_mutex);
    if (!_projection_helper) {
        LOG_INFO("Construct RoadGeometrymanager for DAEInterface get_projection_helper");
        // LOG_BEGIN(INFO) << "Construct ProjectionHelper for DAEInterface " << this; LOG_END;
        _projection_helper.reset(new ProjectionHelper<15>);
    }
    return _projection_helper.get();
};

void DAEInterface::set_destination_tile_id(int tile_id)
{
    std::lock_guard<std::recursive_mutex> locker(s_interface_mutex);
    _destination_tile_id = tile_id;
}

int DAEInterface::get_destination_tile_id()
{
    return _destination_tile_id;
}

void DAEInterface::set_all_tileids(std::vector<int>& all_tileids) {
    std::lock_guard<std::recursive_mutex> locker(s_interface_mutex);
    _all_tileids = all_tileids;
};
std::vector<int> DAEInterface::get_all_tileids() {
    std::lock_guard<std::recursive_mutex> locker(s_interface_mutex);
    return _all_tileids;
};
void DAEInterface::set_current_download_tiles(std::vector<int>& cur_tileids) {
    std::lock_guard<std::recursive_mutex> locker(s_interface_mutex);
    _current_download_tileids = cur_tileids;
};
std::vector<int> DAEInterface::get_current_download_tileids() {
    std::lock_guard<std::recursive_mutex> locker(s_interface_mutex);
    return _current_download_tileids;
};
void DAEInterface::set_downloaded_tileids(std::vector<int>& downloaded_tileids) {
    std::lock_guard<std::recursive_mutex> locker(s_interface_mutex);
    _downloaded_tileids = downloaded_tileids;
};
std::vector<int> DAEInterface::get_downloaded_tileids() {
    std::lock_guard<std::recursive_mutex> locker(s_interface_mutex);
    return _downloaded_tileids;
};

};

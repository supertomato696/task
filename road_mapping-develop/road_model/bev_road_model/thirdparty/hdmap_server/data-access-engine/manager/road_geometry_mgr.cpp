#include "road_geometry_mgr.h"
#include <atomic>
#include <chrono>
#include <thread>
#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif
#include "proxy/feature_proxy_base.h"
#include "proxy/feature_id_proxy.h"
#include "proxy/feature_with_id_proxy_base.h"

#include "proxy/tile_proxy.h"
#include "proxy/common_proxy.h"
#include "proxy/confidence_proxy.h"
#include "proxy/dynamics_proxy.h"
#include "proxy/lane_proxy.h"
#include "proxy/link_proxy.h"
#include "proxy/position_proxy.h"
#include "proxy/traffic_proxy.h"
#include "proxy/feature_id_proxy.h"

#include "data_access_engine.h"
#include "dao/config_address.h"
#include "public/platform.h"
#include "dao/road_tile_dao.h"
// #include "utils/log_util.h"

namespace data_access_engine
{

int64_t get_tick_count(void) {
    int64_t currentTime = 0;
#ifdef WIN32
    currentTime = GetTickCount64();
#else
    struct timeval current;
    gettimeofday(&current, NULL);
    currentTime = current.tv_sec * 1000 + current.tv_usec / 1000;
#endif
    return currentTime;
};

RoadGeometryManager::RoadGeometryManager(ConfigAddress* c, DAEInterface* d) : _tile_inited(false),
                    _interface(d), _current_ver(0), _prev_tick(0), _last_tick(0), _sync_ver(0) {
    if (d == nullptr) {
        _interface = DAEInterface::get_instance();            
    }
    _road_tile_dao.reset(new RoadTileDao(c));
    _download_param.reset(new RoadTileDownloadParam);
};
RoadGeometryManager::RoadGeometryManager() : RoadGeometryManager(nullptr, nullptr) {};
RoadGeometryManager::~RoadGeometryManager() { clear_all(); };

bool RoadGeometryManager::tile_local_project(int tile_id, Vector3D& pos) const {
    if (!_tile_inited) {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        if (!_tile_inited) {
            int tid = tile_id;
            ProjectionHelper<15> ph;
            auto proj_str = ph.get_tile_Proj4_string(tid);
            _interface->get_projection_helper()->set_destination_Proj4_string(proj_str);
            _tile_inited = true;
        }
    }
    auto proj = _interface->get_projection_helper();
    if (proj) {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        return proj->tile_local_project(tile_id, pos.X(), pos.Y(), pos.Z());
    }
    return false;
};
bool RoadGeometryManager::tile_local_unproject(int tile_id, Vector3D& pos) const {
    if (!_tile_inited) {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        if (!_tile_inited) {
            int tid = tile_id;
            ProjectionHelper<15> ph;
            auto proj_str = ph.get_tile_Proj4_string(tid);
            _interface->get_projection_helper()->set_destination_Proj4_string(proj_str);
            _tile_inited = true;
        }
    }
    auto proj = _interface->get_projection_helper();
    if (proj) {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        return proj->tile_local_unproject(tile_id, pos.X(), pos.Y(), pos.Z());
    }
    return false;
};

int RoadGeometryManager::global_point_to_tileID(const Vector3D& pos) const {
    auto proj = _interface->get_projection_helper();
    if (proj) {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        return proj->point_to_tile_ID(pos.X(), pos.Y(), pos.Z());
    }
    return 0;
}

int RoadGeometryManager::WGS84_to_tileID(const Vector3D& pos) const {
    auto proj = _interface->get_projection_helper();
    if (proj) {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        return proj->WGS_to_tile_ID(pos.X(), pos.Y(), pos.Z());
    }
    return 0;
};

bool RoadGeometryManager::global_point_to_WGS84(Vector3D& pos) const {
    if (pos.SquaredLength() < FLOAT_TOLERANCE) {
        return false;
    }
    auto proj = _interface->get_projection_helper();
    if (proj) {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        return proj->Gauss_Kruger_unprojection(pos.X(), pos.Y(), pos.Z());
    }
    return false;
};

bool RoadGeometryManager::global_point_from_WGS84(Vector3D& pos) const {
    auto proj = _interface->get_projection_helper();
    if (proj) {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        /*auto tmp = pos;
        proj->Gauss_Kruger_projection(pos.X(), pos.Y(), pos.Z());
        proj->Gauss_Kruger_unprojection(pos.X(), pos.Y(), pos.Z());
        if ((tmp - pos).Length() > 0.00001) {
            LOG(ERROR) << "projection error";
            pos = tmp;
            proj->Gauss_Kruger_projection(pos.X(), pos.Y(), pos.Z());
            proj->Gauss_Kruger_unprojection(pos.X(), pos.Y(), pos.Z());
        }*/
        return proj->Gauss_Kruger_projection(pos.X(), pos.Y(), pos.Z());
    }
    return false;
};

bool RoadGeometryManager::get_tile_box_WGS(int tid, double& minx, double& miny, double& maxx, double& maxy) {
    return _interface->get_projection_helper()->get_tile_box_WGS(tid, minx, miny, maxx, maxy);
};

template<typename ID2ElemMap>
bool GetNextID(ID2ElemMap& idm, std::shared_ptr<FeatureIDProxy>& id) {
    while (idm.find(id->to_binary_string()) != idm.end()) {
        id->set_id(id->id() - 1);
    }
    return true;
}

bool RoadGeometryManager::make_new_id(const Vector3D& pos, std::shared_ptr<FeatureWithIDProxyBase> elem,
                                        TileInfoPtr& tile, bool bwgs) {
    if (!elem->mutable_id()) {
        return false;
    }
    auto id = elem->mutable_id();
    int32_t tile_id = 0;
    {
        auto proj_helper = _interface->get_projection_helper();
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        if (bwgs) {
            tile_id = proj_helper->WGS_to_tile_ID(pos.X(), pos.Y(), pos.Z());
        }
        else {
            tile_id = proj_helper->point_to_tile_ID(pos.X(), pos.Y(), pos.Z());
        }
        if (!_tile_inited && tile_id > 0) {
            auto proj_str = proj_helper->get_tile_Proj4_string(tile_id);
            _interface->set_destination_tile_id(tile_id);
            _interface->get_projection_helper()->set_destination_Proj4_string(proj_str);
            _tile_inited = true;
            auto time_now = std::chrono::system_clock::now();
            _sync_ver = std::chrono::duration_cast<std::chrono::milliseconds>(time_now.time_since_epoch()).count();
            _current_ver = _sync_ver + 1;
            _prev_tick = get_tick_count();
            // LOG_DEBUG("make_new_id() init center tile: %d sync_ver: %d prev_tick: %s",tile_id, _sync_ver, _prev_tick);
            // LOG_BEGIN(INFO) << "make_new_id() init center tile: " << tile_id << " sync_ver: "
            //     << _sync_ver << " prev_tick: " << _prev_tick; LOG_END;
        }
    }
    if (tile_id == 0) {
        tile.reset();
        // LOG_BEGIN(ERROR) << "make_new_id() for pos(" << pos << ") failed"; LOG_END;
        return false;
    }
    tile = get_road_tile(tile_id);
    id->set_tileid(tile_id);

    int64_t current_ver = _current_ver;
    uint64_t tick = get_tick_count();
    if (int64_t(tick - _prev_tick) > 0) {
        if (int64_t(tick - _last_tick) <= 0) {
            tick = _last_tick + 1;
        }
        current_ver += tick - _prev_tick;
        _last_tick = tick;
    }
    else {
        current_ver++;
    }

    id->set_version(current_ver);
    id->set_id(0);
    id->set_type(elem->feature_id_type());
    elem->make_id_index();
    id->set_version(current_version());
    //LOG_BEGIN(DEBUG) << "make_new_id() for pos(" << pos << ") get "
    //    << id->to_string(); LOG_END;
    switch (id->type()) {
    case LinkProxy::ELEM_ID_TYPE: {
        GetNextID(tile->_link_map, id);
        auto p = std::dynamic_pointer_cast<const LinkProxy>(elem);
        tile->_link_map[p->id()->to_binary_string()] = p;
        break;
    }
    case NodeProxy::ELEM_ID_TYPE: {
        GetNextID(tile->_node_map, id);
        auto p = std::dynamic_pointer_cast<const NodeProxy>(elem);
        tile->_node_map[p->id()->to_binary_string()] = p;
        break;
    }
    case LaneProxy::ELEM_ID_TYPE: {
        GetNextID(tile->_lane_map, id);
        auto p = std::dynamic_pointer_cast<const LaneProxy>(elem);
        tile->_lane_map[p->id()->to_binary_string()] = p;
        break;
    }
    case LaneBoundaryProxy::ELEM_ID_TYPE: {
        GetNextID(tile->_lane_boundary_map, id);
        auto p = std::dynamic_pointer_cast<const LaneBoundaryProxy>(elem);
        tile->_lane_boundary_map[p->id()->to_binary_string()] = p;
        break;
    }
    case LaneGroupProxy::ELEM_ID_TYPE: {
        GetNextID(tile->_lane_group_map, id);
        auto p = std::dynamic_pointer_cast<const LaneGroupProxy>(elem);
        tile->_lane_group_map[p->id()->to_binary_string()] = p;
        break;
    }
    case JunctionProxy::ELEM_ID_TYPE: {
        GetNextID(tile->_junction_map, id);
        auto p = std::dynamic_pointer_cast<const JunctionProxy>(elem);
        tile->_junction_map[p->id()->to_binary_string()] = p; 
        break;
    }
    case TrafficInfoProxy::ELEM_ID_TYPE: {
        GetNextID(tile->_traffic_info_map, id);
        auto p = std::dynamic_pointer_cast<const TrafficInfoProxy>(elem);
        tile->_traffic_info_map[p->id()->to_binary_string()] = p;
        break;
    }
    case PositionObjectProxy::ELEM_ID_TYPE: {
        GetNextID(tile->_position_object_map, id);
        auto p = std::dynamic_pointer_cast<const PositionObjectProxy>(elem);
        tile->_position_object_map[p->id()->to_binary_string()] = p;
        break;
    }
    case RoadBoundaryProxy::ELEM_ID_TYPE: {
        GetNextID(tile->_road_boundary_map, id);
        auto p = std::dynamic_pointer_cast<const RoadBoundaryProxy>(elem);
        tile->_road_boundary_map[p->id()->to_binary_string()] = p;
        break;
    }
    case DataQualityProxy::ELEM_ID_TYPE: {
        GetNextID(tile->_data_quality_map, id);
        auto p = std::dynamic_pointer_cast<const DataQualityProxy>(elem);
        tile->_data_quality_map[p->id()->to_binary_string()] = p;
        break;
    }
    case DynamicProxy::ELEM_ID_TYPE: {
        GetNextID(tile->_dynamic_map, id);
        auto p = std::dynamic_pointer_cast<const DynamicProxy>(elem);
        tile->_dynamic_map[p->id()->to_binary_string()] = p;
        break;
    }
    default:
        return false;
    }
    return true;
}

template <class Proxy>
bool RoadGeometryManager::remake_id(std::shared_ptr<Proxy>& ptr, const Vector3D& pos, TileInfoPtr& tile, bool bwgs) {
    if (pos.SquaredLength() < 0.00001 || !ptr) {
        tile.reset();
        return false;
    }
    if (ptr->id() && ptr->id()->is_del()) {
        if (ptr->id()->version() <= 0) {
            ptr->mutable_id()->set_version(_current_ver);
        }
        return false;
    }
    if (!ptr->id()) {
        auto id = ptr->mutable_id();
        if (!id) {
            return false;
        }
        ptr->mutable_id()->set_type(ptr->feature_id_type());
        ptr->mutable_id()->set_version(_current_ver);
    }
    if (ptr->id()->version() <= 0) {
        ptr->mutable_id()->set_version(_current_ver);
    }

    int tile_id = bwgs ? WGS84_to_tileID(pos) : global_point_to_tileID(pos);
    if (tile_id <= 0) {
        tile.reset();
        return false;
    }
    else if (tile_id != ptr->id()->tileid()) {
        auto newptr = ptr->template dump<Proxy>();
        newptr->mutable_id()->set_is_deleted(true);
        newptr->mutable_id()->set_version(_current_ver);
        newptr->set_parent(tile.get());
        if (!make_new_id(pos, ptr, tile, bwgs)) {
            LOG_BEGIN(ERROR) << "remake_id() for pos(" << pos << ") failed for "
                << ptr->id()->to_string(); LOG_END;
            return false;
        }
        ptr->set_parent(tile.get());
        ptr->mutable_id()->set_version(newptr->id()->version());
        if (ptr->id()->version() <= _sync_ver){
            ptr->mutable_id()->set_version(_current_ver);
        }
        if (newptr->id()->version() <= _sync_ver){
            newptr->mutable_id()->set_version(_current_ver);
        }
        LOG_BEGIN(DEBUG) << "remake_id() for pos(" << pos << ") change "
            << newptr->id()->to_string() << " to " << ptr->id()->to_string(); LOG_END;
        ptr = newptr;
        return true;
    }
    return false;
};

bool RoadGeometryManager::remake_elem_id(std::shared_ptr<FeatureWithIDProxyBase>& ptr, const Vector3D& pos, TileInfoPtr& tile, bool bwgs) {
    bool bf = false;
    switch (ptr->id()->type()) {
    case LinkProxy::ELEM_ID_TYPE: {
        auto p = std::dynamic_pointer_cast<LinkProxy>(ptr);
        bf = remake_id(p, pos, tile, bwgs);
        ptr = p;
        break;
    }
    case NodeProxy::ELEM_ID_TYPE: {
        auto p = std::dynamic_pointer_cast<NodeProxy>(ptr);
        bf = remake_id(p, pos, tile, bwgs);
        ptr = p;
        break;
    }
    case LaneProxy::ELEM_ID_TYPE: {
        auto p = std::dynamic_pointer_cast<LaneProxy>(ptr);
        bf = remake_id(p, pos, tile, bwgs);
        ptr = p;
        break;
    }
    case LaneBoundaryProxy::ELEM_ID_TYPE: {
        auto p = std::dynamic_pointer_cast<LaneBoundaryProxy>(ptr);
        bf = remake_id(p, pos, tile, bwgs);
        ptr = p;
        break;
    }
    case LaneGroupProxy::ELEM_ID_TYPE: {
        auto p = std::dynamic_pointer_cast<LaneGroupProxy>(ptr);
        bf = remake_id(p, pos, tile, bwgs);
        ptr = p;
        break;
    }
    case JunctionProxy::ELEM_ID_TYPE: {
        auto p = std::dynamic_pointer_cast<JunctionProxy>(ptr);
        bf = remake_id(p, pos, tile, bwgs);
        ptr = p;
        break;
    }
    case TrafficInfoProxy::ELEM_ID_TYPE: {
        auto p = std::dynamic_pointer_cast<TrafficInfoProxy>(ptr);
        bf = remake_id(p, pos, tile, bwgs);
        ptr = p;
        break;
    }
    case PositionObjectProxy::ELEM_ID_TYPE: {
        auto p = std::dynamic_pointer_cast<PositionObjectProxy>(ptr);
        bf = remake_id(p, pos, tile, bwgs);
        ptr = p;
        break;
    }
    case RoadBoundaryProxy::ELEM_ID_TYPE: {
        auto p = std::dynamic_pointer_cast<RoadBoundaryProxy>(ptr);
        bf = remake_id(p, pos, tile, bwgs);
        ptr = p;
        break;
    }
    case DataQualityProxy::ELEM_ID_TYPE: {
        auto p = std::dynamic_pointer_cast<DataQualityProxy>(ptr);
        bf = remake_id(p, pos, tile, bwgs);
        ptr = p;
        break;
    }
    case DynamicProxy::ELEM_ID_TYPE: {
        auto p = std::dynamic_pointer_cast<DynamicProxy>(ptr);
        bf = remake_id(p, pos, tile, bwgs);
        ptr = p;
        break;
    }
    default:
        return false;
    }
    return bf;
}

std::shared_ptr<const FeatureWithIDProxyBase> RoadGeometryManager::get_feature_by_id(const FeatureIDProxy& id) const {
    switch (id.type()) {
    case LinkProxy::ELEM_ID_TYPE:
        return get_link(id);
    case NodeProxy::ELEM_ID_TYPE:
        return get_node(id);
    case LaneProxy::ELEM_ID_TYPE:
        return get_lane(id);
    case LaneBoundaryProxy::ELEM_ID_TYPE:
        return get_lane_boundary(id);
    case LaneGroupProxy::ELEM_ID_TYPE:
        return get_lane_group(id);
    case JunctionProxy::ELEM_ID_TYPE:
        return get_junction(id);
    case TrafficInfoProxy::ELEM_ID_TYPE:
        return get_traffic_info(id);
    case PositionObjectProxy::ELEM_ID_TYPE:
        return get_position_object(id);
    case RoadBoundaryProxy::ELEM_ID_TYPE:
        return get_road_boundary(id);
    case DataQualityProxy::ELEM_ID_TYPE:
        return get_data_quality(id);
    case DynamicProxy::ELEM_ID_TYPE:
        return get_dynamic(id);
    default:
        return nullptr;
    }
}

std::shared_ptr<const LinkProxy> RoadGeometryManager::get_link(const FeatureIDProxy& eid) const {
    auto it = _tiles.find(eid.tileid());
    if (it == _tiles.end()) {
        return nullptr;
    }
    auto eit = it->second->_link_map.find(eid.to_binary_string());
    if (eit != it->second->_link_map.end()) {
        auto elem = eit->second.lock();
        if (elem) {
            return elem;
        }
    }
    for (auto& elem : it->second->links()) {
        if (elem && elem->id() && elem->id()->is_equal(eid)) {
            it->second->_link_map[eid.to_binary_string()] = elem;
            return elem;
        }
    }
    return nullptr;
}

std::shared_ptr<const NodeProxy> RoadGeometryManager::get_node(const FeatureIDProxy& eid) const {
    auto it = _tiles.find(eid.tileid());
    if (it == _tiles.end()) {
        return nullptr;
    }
    auto eit = it->second->_node_map.find(eid.to_binary_string());
    if (eit != it->second->_node_map.end()) {
        auto elem = eit->second.lock();
        if (elem) {
            return elem;
        }
    }
    for (auto& elem : it->second->nodes()) {
        if (elem && elem->id() && elem->id()->is_equal(eid)) {
            it->second->_node_map[eid.to_binary_string()] = elem;
            return elem;
        }
    }
    return nullptr;
}

std::shared_ptr<const LaneProxy> RoadGeometryManager::get_lane(const FeatureIDProxy& eid) const {
    auto it = _tiles.find(eid.tileid());
    if (it == _tiles.end()) {
        return nullptr;
    }
    auto eit = it->second->_lane_map.find(eid.to_binary_string());
    if (eit != it->second->_lane_map.end()) {
        auto elem = eit->second.lock();
        if (elem) {
            return elem;
        }
    }
    for (auto& elem : it->second->lanes()) {
        if (elem && elem->id() && elem->id()->is_equal(eid)) {
            it->second->_lane_map[eid.to_binary_string()] = elem;
            return elem;
        }
    }
    return nullptr;
}

std::shared_ptr<const LaneBoundaryProxy> RoadGeometryManager::get_lane_boundary(const FeatureIDProxy& eid) const {
    auto it = _tiles.find(eid.tileid());
    if (it == _tiles.end()) {
        return nullptr;
    }
    auto eit = it->second->_lane_boundary_map.find(eid.to_binary_string());
    if (eit != it->second->_lane_boundary_map.end()) {
        auto elem = eit->second.lock();
        if (elem) {
            return elem;
        }
    }
    for (auto& elem : it->second->lane_boundarys()) {
        if (elem && elem->id() && elem->id()->is_equal(eid)) {
            it->second->_lane_boundary_map[eid.to_binary_string()] = elem;
            return elem;
        }
    }
    return nullptr;
}

std::shared_ptr<const LaneGroupProxy> RoadGeometryManager::get_lane_group(const FeatureIDProxy& eid) const {
    auto it = _tiles.find(eid.tileid());
    if (it == _tiles.end()) {
        return nullptr;
    }
    auto eit = it->second->_lane_group_map.find(eid.to_binary_string());
    if (eit != it->second->_lane_group_map.end()) {
        auto elem = eit->second.lock();
        if (elem) {
            return elem;
        }
    }
    for (auto& elem : it->second->lane_groups()) {
        if (elem && elem->id() && elem->id()->is_equal(eid)) {
            it->second->_lane_group_map[eid.to_binary_string()] = elem;
            return elem;
        }
    }
    return nullptr;
}

std::shared_ptr<const JunctionProxy> RoadGeometryManager::get_junction(const FeatureIDProxy& eid) const {
    auto it = _tiles.find(eid.tileid());
    if (it == _tiles.end()) {
        return nullptr;
    }
    auto eit = it->second->_junction_map.find(eid.to_binary_string());
    if (eit != it->second->_junction_map.end()) {
        auto elem = eit->second.lock();
        if (elem) {
            return elem;
        }
    }
    for (auto& elem : it->second->junctions()) {
        if (elem && elem->id() && elem->id()->is_equal(eid)) {
            it->second->_junction_map[eid.to_binary_string()] = elem;
            return elem;
        }
    }
    return nullptr;
}

std::shared_ptr<const TrafficInfoProxy> RoadGeometryManager::get_traffic_info(const FeatureIDProxy& eid) const {
    auto it = _tiles.find(eid.tileid());
    if (it == _tiles.end()) {
        return nullptr;
    }
    auto eit = it->second->_traffic_info_map.find(eid.to_binary_string());
    if (eit != it->second->_traffic_info_map.end()) {
        auto elem = eit->second.lock();
        if (elem) {
            return elem;
        }
    }
    for (auto& elem : it->second->traffic_infos()) {
        if (elem && elem->id() && elem->id()->is_equal(eid)) {
            it->second->_traffic_info_map[eid.to_binary_string()] = elem;
            return elem;
        }
    }
    return nullptr;
}

std::shared_ptr<const PositionObjectProxy> RoadGeometryManager::get_position_object(const FeatureIDProxy& eid) const {
    auto it = _tiles.find(eid.tileid());
    if (it == _tiles.end()) {
        return nullptr;
    }
    auto eit = it->second->_position_object_map.find(eid.to_binary_string());
    if (eit != it->second->_position_object_map.end()) {
        auto elem = eit->second.lock();
        if (elem) {
            return elem;
        }
    }
    for (auto& elem : it->second->position_objects()) {
        if (elem && elem->id() && elem->id()->is_equal(eid)) {
            it->second->_position_object_map[eid.to_binary_string()] = elem;
            return elem;
        }
    }
    return nullptr;
}

std::shared_ptr<const RoadBoundaryProxy> RoadGeometryManager::get_road_boundary(const FeatureIDProxy& eid) const {
    auto it = _tiles.find(eid.tileid());
    if (it == _tiles.end()) {
        return nullptr;
    }
    auto eit = it->second->_road_boundary_map.find(eid.to_binary_string());
    if (eit != it->second->_road_boundary_map.end()) {
        auto elem = eit->second.lock();
        if (elem) {
            return elem;
        }
    }
    for (auto& elem : it->second->road_boundarys()) {
        if (elem && elem->id() && elem->id()->is_equal(eid)) {
            it->second->_road_boundary_map[eid.to_binary_string()] = elem;
            return elem;
        }
    }
    return nullptr;
}

std::shared_ptr<const DataQualityProxy> RoadGeometryManager::get_data_quality(const FeatureIDProxy& eid) const {
    auto it = _tiles.find(eid.tileid());
    if (it == _tiles.end()) {
        return nullptr;
    }
    auto eit = it->second->_data_quality_map.find(eid.to_binary_string());
    if (eit != it->second->_data_quality_map.end()) {
        auto elem = eit->second.lock();
        if (elem) {
            return elem;
        }
    }
    for (auto& elem : it->second->data_qualitys()) {
        if (elem && elem->id() && elem->id()->is_equal(eid)) {
            it->second->_data_quality_map[eid.to_binary_string()] = elem;
            return elem;
        }
    }
    return nullptr;
}

std::shared_ptr<const DynamicProxy> RoadGeometryManager::get_dynamic(const FeatureIDProxy& eid) const {
    auto it = _tiles.find(eid.tileid());
    if (it == _tiles.end()) {
        return nullptr;
    }
    auto eit = it->second->_dynamic_map.find(eid.to_binary_string());
    if (eit != it->second->_dynamic_map.end()) {
        auto elem = eit->second.lock();
        if (elem) {
            return elem;
        }
    }
    for (auto& elem : it->second->dynamics()) {
        if (elem && elem->id() && elem->id()->is_equal(eid)) {
            it->second->_dynamic_map[eid.to_binary_string()] = elem;
            return elem;
        }
    }
    return nullptr;
}

template <class T>
inline void append_elem(SharedProxyVector<T>& vec_elems, SharedProxyVector<T>& vec_new_elems) {
    for (auto& p : vec_new_elems) {
        if (!p || !p->id()) {
            continue;
        }
        bool exist = false;
        for (size_t i = 0; i < vec_elems.size(); ++i) {
            auto& e = vec_elems[i];
            if (!e || !e->id()) {
                continue;
            }
            if (e->id()->is_equal(*(p->id()))) {
                if (e->id()->version() < p->id()->version()) {
                    vec_elems.erase(i);
                }
                else {
                    exist = true;
                }
                break;
            }
        }
        if (!exist){
            vec_elems.push_back(p);
        }
    }
}

template <class T>
inline void append_elem(SharedProxyVector<T>& vec_elems,
                        std::unordered_map<std::string, std::weak_ptr<const T>>& map_id2elem,
                        SharedProxyVector<T>& vec_new_elems) {
    for (auto& p : vec_new_elems) {
        if (!p || !p->id()) {
            continue;
        }
        auto sid = p->id()->to_binary_string();
        auto it = map_id2elem.find(sid);
        if (it == map_id2elem.end()) {
            vec_elems.push_back(p);
            map_id2elem[sid] = p;
        }
        else {
            auto org = std::const_pointer_cast<T>(it->second.lock());
            if (org && org->id()->version() >= p->id()->version()) {
                continue;
            }
            vec_elems.replace(org, p);
            it->second = p;
        }
    }
}

template <class T>
inline void append_elem_id(FeatureReferenceVector<T>& vec_elems, FeatureReferenceVector<T>& vec_new_elems) {
    for (auto& p : vec_new_elems) {
        if (!p) {
            continue;
        }
        bool exist = false;
        for (size_t i = 0; i < vec_elems.size(); ++i){
            auto& e = vec_elems[i];
            if (!e) {
                continue;
            }
            if (e->is_equal(*p)) {
                if (e->version() < p->version()){
                    vec_elems.erase(i);
                }
                else {
                    exist = true;
                }
                break;
            }
        }
        if (!exist){
            vec_elems.push_back(p);
        }
    }
}

void RoadGeometryManager::merge_tiles(std::vector<TileInfoPtr>& tiles, 
                                        std::vector<services::TileInfo*>& pb_tiles,
                                        std::map<int, std::pair<TileInfoPtr, int>>& tile_infos,
                                        std::set<int>& download_tiles, int thread_num) {
    thread_num = std::max(1, thread_num);
    std::map<int, std::vector<services::TileInfo*>> tid2infos;
    for (auto t : pb_tiles) {
        int lc = 0;
        for (int i = 0; i < t->feat_list_size(); ++i) {
            lc += t->feat_list(i).feats_size() + t->feat_list(i).refs_size();
        }
        if (lc > 0) {
            tid2infos[t->tile_id()].push_back(t);
        }
    }
    std::vector<int> tids;
    tids.reserve(tid2infos.size());
    for (auto& tit : tid2infos) {
        tids.push_back(tit.first);
        std::sort(tit.second.begin(), tit.second.end(), [](services::TileInfo* lt, services::TileInfo* rt) {
            if (lt->version() > rt->version()) {
                return true;
            }
            else if (lt->version() < rt->version()) {
                return false;
            }
            int lc = 0;
            for (int i = 0; i < lt->feat_list_size(); ++i) {
                lc += lt->feat_list(i).feats_size();
            }
            int rc = 0;
            for (int i = 0; i < rt->feat_list_size(); ++i) {
                rc += rt->feat_list(i).feats_size();
            }            
            return lt > rt;
        });
    }
    thread_num = std::min<int>(thread_num, (int)tids.size());
    std::vector<std::thread> threads;
    threads.reserve(thread_num);
    std::atomic<int> tile_ind(0);
    std::mutex mutex;
    for (int ti = 0; ti < thread_num; ++ti) {
        threads.push_back(std::thread([this, &tile_ind, &tids, &tid2infos, &tile_infos, &tiles, &mutex]() {
            while (true) {
                int ind = tile_ind.fetch_add(1);
                if (ind >= (int)tids.size()) {
                    break;
                }
                auto& tis = tid2infos[tids[ind]];
                for (size_t i = 0; i < tis.size(); ++i) {
                    mutex.lock();
                    auto itr = tile_infos.find(tis[i]->tile_id());
                    if (itr == tile_infos.end()) {
                        LOG_BEGIN(DEBUG) << "merge_tiles() insert tile " << tis[i]->tile_id() << '@'
                            << tis[i]->version() << " ind " << ind << '/' << tids.size() << " : "
                            << i << '/' << tis.size();
                        LOG_END;
                        std::shared_ptr<TileInfoProxy> pt(FeatureProxyBase::create_proxy<TileInfoProxy>());
                        pt->relink_parent();
                        tiles.push_back(pt);
                        tile_infos[tis[i]->tile_id()] = std::make_pair(pt, (int)tiles.size() - 1);
                        mutex.unlock();
                        pt->from_message(tis[i]);
                        pt->load_maps();
                    }
                    else {
                        auto org_tile = itr->second.first;
                        mutex.unlock();
                        std::shared_ptr<TileInfoProxy> tile(FeatureProxyBase::create_proxy<TileInfoProxy>());
                        tile->from_message(tis[i]);
                        tile_infos.insert(std::make_pair(tile->tile_id(), std::make_pair(tile, i)));
                        LOG_BEGIN(DEBUG) << "merge_tiles() merge tile " << tile->tile_id() << '@'
                            << tile->version() << " with " << org_tile->version() << " ind " << ind << '/'
                            << tids.size() << " : " << i << '/' << tis.size();
                        LOG_END;
                        org_tile->set_version((std::max)(org_tile->version(), tile->version()));
                        append_elem(org_tile->links(), org_tile->_link_map, tile->links());
                        append_elem_id(org_tile->link_refs(), tile->link_refs());
                        append_elem(org_tile->nodes(), org_tile->_node_map, tile->nodes());
                        append_elem_id(org_tile->node_refs(), tile->node_refs());
                        append_elem(org_tile->lanes(), org_tile->_lane_map, tile->lanes());
                        append_elem_id(org_tile->lane_refs(), tile->lane_refs());
                        append_elem(org_tile->lane_boundarys(), org_tile->_lane_boundary_map, tile->lane_boundarys());
                        append_elem_id(org_tile->lane_boundary_refs(), tile->lane_boundary_refs());
                        append_elem(org_tile->lane_groups(), org_tile->_lane_group_map, tile->lane_groups());
                        append_elem_id(org_tile->lane_group_refs(), tile->lane_group_refs());
                        append_elem(org_tile->junctions(), org_tile->_junction_map, tile->junctions());
                        append_elem_id(org_tile->junction_refs(), tile->junction_refs());
                        append_elem(org_tile->traffic_infos(), org_tile->_traffic_info_map, tile->traffic_infos());
                        append_elem_id(org_tile->traffic_info_refs(), tile->traffic_info_refs());
                        append_elem(org_tile->position_objects(), org_tile->_position_object_map, tile->position_objects());
                        append_elem_id(org_tile->position_object_refs(), tile->position_object_refs());
                        append_elem(org_tile->road_boundarys(), org_tile->_road_boundary_map, tile->road_boundarys());
                        append_elem_id(org_tile->road_boundary_refs(), tile->road_boundary_refs());
                        append_elem(org_tile->data_qualitys(), org_tile->_data_quality_map, tile->data_qualitys());
                        append_elem_id(org_tile->data_quality_refs(), tile->data_quality_refs());
                        append_elem(org_tile->dynamics(), org_tile->_dynamic_map, tile->dynamics());
                        append_elem_id(org_tile->dynamic_refs(), tile->dynamic_refs());
                        org_tile->clear_changed();
                    }
                    delete tis[i];
                    tis[i] = nullptr;
                }
            }
        }));
    }
    download_tiles.insert(tids.begin(), tids.end());
    for (int ti = 0; ti < thread_num; ++ti) {
        threads[ti].join();
    }
}

bool RoadGeometryManager::remake_tile_proxy(std::vector<TileInfoPtr>& tiles, int thread_num) {
    thread_num = std::max(1, thread_num);
    thread_num = std::min<int>(thread_num, (int)tiles.size());
    bool bsucc = true;
    LOG_BEGIN(INFO) << "remake_tile_proxy() start: "; LOG_END;
    std::vector<std::thread> threads;
    threads.reserve(thread_num);
    std::atomic<int> tile_ind(0);
    for (int ti = 0; ti < thread_num; ++ti) {
        threads.push_back(std::thread([&tiles, &tile_ind, &bsucc, this]() {
            while (true) {
                int ind = tile_ind.fetch_add(1);
                if (ind >= (int)tiles.size()) {
                    break;
                }
                auto& tile = tiles[ind];
                LOG_BEGIN(INFO) << "remake_tile_proxy() for points @ " << tile->tile_id(); LOG_END;
                bsucc &= tile->links().remake_proxy(this);
                bsucc &= tile->nodes().remake_proxy(this);
                bsucc &= tile->lanes().remake_proxy(this);
                bsucc &= tile->lane_boundarys().remake_proxy(this);
                bsucc &= tile->lane_groups().remake_proxy(this);
                bsucc &= tile->junctions().remake_proxy(this);
                bsucc &= tile->traffic_infos().remake_proxy(this);
                bsucc &= tile->position_objects().remake_proxy(this);
                bsucc &= tile->road_boundarys().remake_proxy(this);
                bsucc &= tile->data_qualitys().remake_proxy(this);
                bsucc &= tile->dynamics().remake_proxy(this);
            }
        }));
    }
    for (int ti = 0; ti < thread_num; ++ti) {
        threads[ti].join();
    }
    threads.clear();
    tile_ind = 0;
    for (int ti = 0; ti < thread_num; ++ti) {
        threads.push_back(std::thread([&tiles, &tile_ind, &bsucc, this]() {
            while (true) {
                int ind = tile_ind.fetch_add(1);
                if (ind >= (int)tiles.size()) {
                    break;
                }
                auto& tile = tiles[ind];
                LOG_BEGIN(INFO) << "remake_tile_proxy() for tile' refs @ " << tile->tile_id();
                LOG_END;
                bsucc &= tile->link_refs().resolve_proxy(this);
                bsucc &= tile->node_refs().resolve_proxy(this);
                bsucc &= tile->lane_refs().resolve_proxy(this);
                bsucc &= tile->lane_boundary_refs().resolve_proxy(this);
                bsucc &= tile->lane_group_refs().resolve_proxy(this);
                bsucc &= tile->junction_refs().resolve_proxy(this);
                bsucc &= tile->traffic_info_refs().resolve_proxy(this);
                bsucc &= tile->position_object_refs().resolve_proxy(this);
                bsucc &= tile->road_boundary_refs().resolve_proxy(this);
                bsucc &= tile->data_quality_refs().resolve_proxy(this);
                bsucc &= tile->dynamic_refs().resolve_proxy(this);
            }
        }));
    }
    for (int ti = 0; ti < thread_num; ++ti) {
        threads[ti].join();
    }    
    threads.clear();
    LOG_BEGIN(INFO) << "remake_tile_proxy() finished: " << bsucc; LOG_END;
    return bsucc;
}

bool RoadGeometryManager::resolve_tiles() {
    TileInfoList tiles;
    {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        for (auto& tit : _tiles) {
            tit.second->load_maps();
            tiles.push_back(tit.second);
        }
    }
    return remake_tile_proxy(tiles);
}

void RoadGeometryManager::judge_editable(std::set<int>& tids) {
    TileInfoList tiles;
    {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        _tile15_ids.swap(tids);
        tiles.reserve(_tiles.size());
        for (auto& tit : _tiles) {
            tiles.push_back(tit.second);
        }
    }
    for (auto& tit : _tiles) {
        auto tile = tit.second;
        LOG_BEGIN(INFO) << "remake_tile_proxy() for tile' refs @ " << tile->tile_id(); LOG_END;
        tile->links().judge_editable(this);
        tile->nodes().judge_editable(this);
        tile->lanes().judge_editable(this);
        tile->lane_boundarys().judge_editable(this);
        tile->lane_groups().judge_editable(this);
        tile->junctions().judge_editable(this);
        tile->traffic_infos().judge_editable(this);
        tile->position_objects().judge_editable(this);
        tile->road_boundarys().judge_editable(this);
        tile->data_qualitys().judge_editable(this);
        tile->dynamics().judge_editable(this);
    }
    {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        _tile15_ids.swap(tids);
    }
}

bool RoadGeometryManager::correct_tile_refs() {
    TileInfoList tiles;
    {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        tiles.reserve(_tiles.size());
        for (auto& tit : _tiles) {
            tiles.push_back(tit.second);
        }
    }
    bool bc = false;
    for (auto& tile : tiles) {
        bc |= tile->correct_tile_refs(this);
    }
    return bc;
}


template <class T>
inline void merge_elems_by_value(SharedProxyVector<T>& es, SharedProxyVector<T>& res,
                       std::unordered_map<std::string, std::weak_ptr<const T>>& ms,
                       std::unordered_map<std::string, std::weak_ptr<const T>>& rms) {
    for (auto& r : res) {
        if (!r || !r->id()) {
            continue;
        }
        auto str = r->id()->to_binary_string();
        bool  bf = false;
        for (size_t i = 0; i < es.size(); i++) {
            std::shared_ptr<T> o = es[i];
            r->template set_remote_proxy<T>(o);
            if (!r->merge(MERGE_COMPARE_ONLY, nullptr)) {
                bf = true;
                LOG_BEGIN(INFO) << "Duplicate elem " << r->id()->to_string() << " with " << o->id()->to_string(); LOG_END;
                r->mutable_id()->set_value(o->id()->tileid(), o->id()->type(), o->id()->id(), o->id()->version());
                break;
            }
            r->reset_merge();
        }
        if (bf) {
            continue;
        }
        LOG_BEGIN(INFO) << "Append new elem " << r->id()->to_string(); LOG_END;
        es.push_back(r);
        ms[str] = r;
    }
};

void RoadGeometryManager::merge_features(ID2TileMap& tiles_map) {
    LOG_BEGIN(INFO) << "merge_features() start for " << tiles_map.size() << " tiles"; LOG_END;
    LOG_BEGIN(INFO) << "merge_features() for boundarys"; LOG_END;
    for (auto& t : tiles_map) {
        auto tile = get_road_tile(t.first);
        merge_elems_by_value(tile->lane_boundarys(), t.second->lane_boundarys(), tile->_lane_boundary_map, t.second->_lane_boundary_map);
        merge_elems_by_value(tile->road_boundarys(), t.second->road_boundarys(), tile->_road_boundary_map, t.second->_road_boundary_map);
        merge_elems_by_value(tile->data_qualitys(), t.second->data_qualitys(), tile->_data_quality_map, t.second->_data_quality_map);        
    }
    LOG_BEGIN(INFO) << "merge_features() for lanes"; LOG_END;
    for (auto& t : tiles_map) {   
        auto tile = get_road_tile(t.first);
        merge_elems_by_value(tile->lanes(), t.second->lanes(), tile->_lane_map, t.second->_lane_map);
    }
    LOG_BEGIN(INFO) << "merge_features() for lanegroups"; LOG_END;
    for (auto& t : tiles_map) {
        auto tile = get_road_tile(t.first);
        merge_elems_by_value(tile->lane_groups(), t.second->lane_groups(), tile->_lane_group_map, t.second->_lane_group_map);
    }
    LOG_BEGIN(INFO) << "merge_features() for objects"; LOG_END;
    for (auto& t : tiles_map) {        
        auto tile = get_road_tile(t.first);
        merge_elems_by_value(tile->position_objects(), t.second->position_objects(), tile->_position_object_map, t.second->_position_object_map);
    }
    LOG_BEGIN(INFO) << "merge_features() for traffics"; LOG_END;
    for (auto& t : tiles_map) {
        auto tile = get_road_tile(t.first);
        merge_elems_by_value(tile->traffic_infos(), t.second->traffic_infos(), tile->_traffic_info_map, t.second->_traffic_info_map);
    }
    LOG_BEGIN(INFO) << "merge_features() for junction"; LOG_END;
    for (auto& t : tiles_map) {
        auto tile = get_road_tile(t.first);
        merge_elems_by_value(tile->junctions(), t.second->junctions(), tile->_junction_map, t.second->_junction_map);
    }
    LOG_BEGIN(INFO) << "merge_features() for link"; LOG_END;
    for (auto& t : tiles_map) {
        auto tile = get_road_tile(t.first);
        merge_elems_by_value(tile->links(), t.second->links(), tile->_link_map, t.second->_link_map);
    }
    LOG_BEGIN(INFO) << "merge_features() for node"; LOG_END;
    for (auto& t : tiles_map) {
        auto tile = get_road_tile(t.first);
        merge_elems_by_value(tile->nodes(), t.second->nodes(), tile->_node_map, t.second->_node_map);
    }
    LOG_BEGIN(INFO) << "merge_features() for dynamic"; LOG_END;
    for (auto& t : tiles_map) {
        auto tile = get_road_tile(t.first);
        merge_elems_by_value(tile->dynamics(), t.second->dynamics(), tile->_dynamic_map, t.second->_dynamic_map);
    }    
    LOG_BEGIN(INFO) << "merge_features() finish"; LOG_END;
}

bool RoadGeometryManager::correct_tiles() {
    // LOG_INFO("correct_tiles() start");
    // LOG_BEGIN(INFO) << "correct_tiles() start"; LOG_END;
    make_new_version();
    TileInfoList tiles;
    {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        tiles.reserve(_tiles.size());
        for (auto& tit : _tiles) {
            tiles.push_back(tit.second);
        }
    }
    bool bc = false;

    // LOG_BEGIN(INFO) << "correct_tiles() for boundarys"; LOG_END;
    for (auto& tile : tiles) {
        bc |= tile->lane_boundarys().correct_content(this);
        bc |= tile->road_boundarys().correct_content(this);
        bc |= tile->data_qualitys().correct_content(this);
    }
    // LOG_BEGIN(INFO) << "correct_tiles() for lanes"; LOG_END;
    for (auto& tile : tiles) {        
        bc |= tile->lanes().correct_content(this);
    }
    for (auto& tile : tiles) {
        // LOG_DEBUG("correct_tiles() for lanegroup @ %u ",tile->tile_id());
        // LOG_BEGIN(INFO) << "correct_tiles() for lanegroup @ " << tile->tile_id();
        // LOG_END;
        bc |= tile->lane_groups().correct_content(this);
    }
    for (auto& tile : tiles) {
        // LOG_DEBUG("correct_tiles() for object @ %u ",tile->tile_id());
        // LOG_BEGIN(INFO) << "correct_tiles() for object @ " << tile->tile_id();
        // LOG_END;
        bc |= tile->position_objects().correct_content(this);
    }
    for (auto& tile : tiles) {
        // LOG_DEBUG("correct_tiles() for traffic @ %u ",tile->tile_id());
        // LOG_BEGIN(INFO) << "correct_tiles() for traffic @ " << tile->tile_id();
        // LOG_END;
        bc |= tile->traffic_infos().correct_content(this);
    }
    for (auto& tile : tiles) {
        // LOG_DEBUG("correct_tiles() for junction @ %u ",tile->tile_id());
        // LOG_BEGIN(INFO) << "correct_tiles() for junction @ " << tile->tile_id();
        // LOG_END;
        bc |= tile->junctions().correct_content(this);
    }
    for (auto& tile : tiles) {
        // LOG_DEBUG("correct_tiles() for link @ %u ",tile->tile_id());
        // LOG_BEGIN(INFO) << "correct_tiles() for link @ " << tile->tile_id();
        // LOG_END;
        bc |= tile->links().correct_content(this);
    }
    for (auto& tile : tiles) {
        // LOG_DEBUG("correct_tiles() for node @ %u ",tile->tile_id());
        // LOG_BEGIN(INFO) << "correct_tiles() for node @ " << tile->tile_id();
        // LOG_END;
        bc |= tile->nodes().correct_content(this);
    }
    for (auto& tile : tiles) {
        // LOG_DEBUG("correct_tiles() for odd @ %u ",tile->tile_id());
        // LOG_BEGIN(INFO) << "correct_tiles() for odd @ " << tile->tile_id();
        // LOG_END;
        bc |= tile->dynamics().correct_content(this);
    }
    for (auto& tile : tiles) {
        tile->correct_content(nullptr);
    }
    // CLOG_INFO("correct_tiles() finish");
    // LOG_BEGIN(INFO) << "correct_tiles() finish"; LOG_END;
    return bc;
}

bool RoadGeometryManager::filter_invalids() {
    LOG_BEGIN(INFO) << "filter_invalids() start"; LOG_END;
    TileInfoList tiles;
    {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        tiles.reserve(_tiles.size());
        for (auto& tit : _tiles) {
            tiles.push_back(tit.second);
        }
    }
    bool bc = false;
    for (auto& tile : tiles) {
        bc |= tile->lane_boundarys().remove_invalid();
        bc |= tile->road_boundarys().remove_invalid();
        bc |= tile->data_qualitys().remove_invalid();
    }
    for (auto& tile : tiles) {
        bc |= tile->lanes().remove_invalid();
    }
    for (auto& tile : tiles) {
        bc |= tile->lane_groups().remove_invalid();
    }
    for (auto& tile : tiles) {
        bc |= tile->position_objects().remove_invalid();
    }
    for (auto& tile : tiles) {
        bc |= tile->traffic_infos().remove_invalid();
    }
    for (auto& tile : tiles) {
        bc |= tile->junctions().remove_invalid();
    }
    for (auto& tile : tiles) {
        bc |= tile->links().remove_invalid();
    }
    for (auto& tile : tiles) {
        bc |= tile->nodes().remove_invalid();
    }
    for (auto& tile : tiles) {
        bc |= tile->dynamics().remove_invalid();
    }
    LOG_BEGIN(INFO) << "filter_invalids() finish"; LOG_END;
    return bc;
}

bool RoadGeometryManager::init_tiles_by_id(const std::vector<int>& tile_15_ids) {
    if (tile_15_ids.empty()) {
        LOG_BEGIN(ERROR) << "init_tiles_by_id() failed: empty tiles!"; LOG_END;
        return false;
    }
    {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        if (_tile_inited) {
            return true;
        }
    }
    clear_all();
    std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
    _tile15_ids.insert(tile_15_ids.begin(), tile_15_ids.end());
    int mask = 0x55555555;
    std::vector<int> tids = tile_15_ids;
    std::sort(tids.begin(), tids.end(), [mask](int l, int r) { 
        int lm = (l & mask);
        int rm = (r & mask);
        return (lm < rm) || (lm == rm && l < r);
    });
    int tid = tids[tids.size() / 2];
    ProjectionHelper<15> ph;
    auto proj_str = ph.get_tile_Proj4_string(tid);
    _interface->set_destination_tile_id(tid);
    _interface->get_projection_helper()->set_destination_Proj4_string(proj_str);
    _tile_inited = true;
    auto time_now = std::chrono::system_clock::now();
    _sync_ver = std::chrono::duration_cast<std::chrono::milliseconds>(time_now.time_since_epoch()).count();
    _current_ver = _sync_ver + 1;
    _prev_tick = get_tick_count();
    LOG_BEGIN(INFO) << "init_tiles_by_id() finished, center tile: " << tid << " sync_ver: "
        << _sync_ver << " prev_tick: " << _prev_tick; LOG_END;
    return true;
}

bool RoadGeometryManager::fetch_tile_versions(const std::vector<int32_t>& tids, const std::string& editor, int64_t& read_ver,
        std::map<int32_t, int64_t>& tile_vers) {
    if (!_road_tile_dao || !_road_tile_dao->fetch_tile_versions(tids, editor, read_ver, tile_vers)) {
        return false;
    }
    std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
    _sync_ver = read_ver;
    _prev_tick = get_tick_count();
    if (_current_ver < _sync_ver + 1) {
        _current_ver = _sync_ver + 1;
    }
    
    LOG_BEGIN(INFO) << "fetch_tile_versions() succeed for " << tids.size()
        << " tiles with version: " << _sync_ver << " tick: " << _prev_tick; LOG_END;
    return true;
}

bool RoadGeometryManager::merge_tiles(const std::vector<int>& tile_15_ids, const std::vector<int>& types, 
                                      const std::string& editor, int64_t req_ver, int thread_num, int64_t& read_ver) {
    std::map<int, std::vector<int>> tids_by11;
    std::stringstream ss;
    for (size_t i = 0; i < tile_15_ids.size(); ++i) {
        int tile_id = tile_15_ids[i];
        if (i < 8) {
            ss << tile_id << ", ";
        }
        else if (i == 8) {
            ss << "... ";
        }
        tids_by11[tile_id / 256].push_back(tile_id);
    }
    std::vector<int> tids11;
    tids11.reserve(tids_by11.size());
    for (auto& tit : tids_by11) {
        tids11.push_back(tit.first);
    }
    
    LOG_BEGIN(INFO) << "merge_tiles() for tiles " << tile_15_ids.size() << '/' << tids_by11.size()
        << " thread_num:" << thread_num; LOG_END;
    std::vector<std::thread> threads;
    threads.reserve(thread_num);
    std::atomic<int> tile_ind(0);
    bool bsucc = true;
    for (int ti = 0; ti < thread_num; ++ti) {
        threads.push_back(std::thread([this, &tile_ind, &tids_by11, &tids11, &bsucc, &types, &editor, req_ver, &read_ver]() {
            while (true) {
                int ind = tile_ind.fetch_add(1);
                if (ind < (int)tids11.size()) {
                    auto& tids = tids_by11[tids11[ind]];
                    std::stringstream ss;
                    for (size_t i = 0; i < tids.size(); i++) {
                        ss << tids[i] << ',';
                        if (i > 8) {
                            ss << "...(" << tids.size() << ") ";
                            break;
                        }
                    }
                    LOG_BEGIN(INFO) << "merge_tiles() begin merge " << ind << "/" << tids.size() << " tiles:" << ss.str(); LOG_END;
                    int retry_num = 0;
                    bool ret = false;
                    int64_t ver = 0;
                     while (retry_num < 3 && !ret){
                        ret = _road_tile_dao->merge_tiles(tids, types, editor, req_ver, ver);
                        ++retry_num;
                        if (retry_num < 3 && !ret){
                            LOG_BEGIN(ERROR) << "merge_tiles() failed to merge tiles " 
                                << ss.str() << " sleep and retry " << retry_num; LOG_END;
                            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                        }
                    }
                    if (retry_num >= 3 && !ret) {
                        bsucc = false;
                        LOG_BEGIN(ERROR) << "merge_tiles() failed to merge tiles " 
                            << ss.str() << " exit thread!"; LOG_END;
                        return;
                    }
                    read_ver = std::max(read_ver, ver);
                    LOG_BEGIN(INFO) << "merge_tiles() success to merge tiles "
                        << ss.str() << " with version " << ver << " (" << ind << '/'
                        << tids11.size() << ") tiles:" << tids.size(); LOG_END;                   
                }
                else {
                    break;
                }
            }
        }));
    }
    for (int ti = 0; ti < thread_num; ++ti) {
        threads[ti].join();
    }
    if (bsucc) {
        LOG_BEGIN(INFO) << "merge_tiles() finish for tiles " << tile_15_ids.size() << '/' << tids_by11.size()
                        << " read_ver:" << read_ver; LOG_END;
    }
    else {
        LOG_BEGIN(INFO) << "merge_tiles() failed for tiles " << tile_15_ids.size() << '/' << tids_by11.size()
                        << " read_ver:" << read_ver; LOG_END;
    }
    return bsucc;
}

bool RoadGeometryManager::revert_tiles(const std::vector<int>& tile_15_ids, const std::string& editor, int64_t req_ver,
                                       int thread_num, int64_t& read_ver) {
    std::map<int, std::vector<int>> tids_by11;
    std::stringstream ss;
    for (size_t i = 0; i < tile_15_ids.size(); ++i) {
        int tile_id = tile_15_ids[i];
        if (i < 8) {
            ss << tile_id << ", ";
        }
        else if (i == 8) {
            ss << "... ";
        }
        tids_by11[tile_id / 256].push_back(tile_id);
    }
    std::vector<int> tids11;
    tids11.reserve(tids_by11.size());
    for (auto& tit : tids_by11) {
        tids11.push_back(tit.first);
    }
    std::vector<int> types = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

    LOG_BEGIN(INFO) << "revert_tiles() for tiles " << tile_15_ids.size() << '/' << tids_by11.size()
        << " thread_num:" << thread_num; LOG_END;
    std::vector<std::thread> threads;
    threads.reserve(thread_num);
    std::atomic<int> tile_ind(0);
    bool bsucc = true;
    for (int ti = 0; ti < thread_num; ++ti) {
        threads.push_back(std::thread([this, &tile_ind, &tids_by11, &tids11, &types, &bsucc, &editor, req_ver, &read_ver]() {
            while (true) {
                int ind = tile_ind.fetch_add(1);
                if (ind < (int)tids11.size()) {
                    auto& tids = tids_by11[tids11[ind]];
                    std::stringstream ss;
                    for (size_t i = 0; i < tids.size(); i++) {
                        ss << tids[i] << ',';
                        if (i > 8) {
                            ss << "...(" << tids.size() << ") ";
                            break;
                        }
                    }
                    LOG_BEGIN(INFO) << "revert_tiles() begin revert " << ind << "/" << tids.size() << " tiles:" << ss.str(); LOG_END;
                    int retry_num = 0;
                    bool ret = false;
                    int64_t ver = 0;
                    while (retry_num < 3 && !ret){
                        ret = _road_tile_dao->revert_tiles(tids, types, editor, req_ver, ver);
                        ++retry_num;
                        if (retry_num < 3 && !ret){
                            LOG_BEGIN(ERROR) << "revert_tiles() failed to revert tiles " 
                                << ss.str() << " sleep and retry " << retry_num; LOG_END;
                            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                        }
                    }
                    if (retry_num >= 3 && !ret) {
                        bsucc = false;
                        LOG_BEGIN(ERROR) << "revert_tiles() failed to revert tiles " 
                            << ss.str() << " exit thread!"; LOG_END;
                        return;
                    }
                    read_ver = std::max(read_ver, ver);
                    LOG_BEGIN(INFO) << "revert_tiles() success to revert tiles "
                        << ss.str() << " with version " << ver << " (" << ind << '/'
                        << tids11.size() << ") tiles:" << tids.size(); LOG_END;                   
                }
                else {
                    break;
                }
            }
        }));
    }
    for (int ti = 0; ti < thread_num; ++ti) {
        threads[ti].join();
    }
    if (bsucc) {
        LOG_BEGIN(INFO) << "revert_tiles() finish for tiles " << tile_15_ids.size() << '/' << tids_by11.size()
                        << " read_ver:" << read_ver; LOG_END;
    }
    else {
        LOG_BEGIN(INFO) << "revert_tiles() failed for tiles " << tile_15_ids.size() << '/' << tids_by11.size()
                        << " read_ver:" << read_ver; LOG_END;
    }
    return bsucc;
}

bool RoadGeometryManager::diff_tiles(const std::vector<int>& tile_15_ids, const std::vector<int>& types, 
                                     const std::string& editor, int64_t req_ver, int thread_num, TileInfoList& diff_tiles) {
    if (tile_15_ids.empty() || types.empty() || req_ver <= 0) {
        LOG_BEGIN(INFO) << "diff_tiles() skip for tiles " << tile_15_ids.size() << " types: " << types.size()
                         << " editor: " << editor << " req_ver: " << req_ver; LOG_END;
        return true;
    }
    RoadTileDownloadParam param;
    if (!types.empty()) {
        param.req_types.clear();
        param.req_types.insert(types.begin(), types.end());
    }
    param.req_ver = req_ver;
    param.editor_name = editor;
    if (!load_tiles_by_id(tile_15_ids, &param, thread_num)) {
        LOG_BEGIN(ERROR) << "diff_tiles() failed for tiles " << tile_15_ids.size() << " types: " << types.size()
                         << " editor: " << editor << " req_ver: " << req_ver; LOG_END;
        return false;
    }
    param.req_ver = 0;
    if (!load_and_merge(MergeConflictPolicy::MERGE_ALL_REMOTE, thread_num, _merged_tiles, _conflict_tiles)) {
        LOG_BEGIN(ERROR) << "diff_tiles() failed to merge tiles " << tile_15_ids.size() << " types: " << types.size()
                         << " editor: " << editor << " req_ver: " << req_ver; LOG_END;
        return false;
    }
    diff_tiles = _merged_tiles;
    LOG_BEGIN(INFO) << "diff_tiles() finish for tiles " << tile_15_ids.size() << " types: " << types.size()
                         << " editor: " << editor << " req_ver: " << req_ver; LOG_END;
    return true;
}

bool RoadGeometryManager::load_tiles(const std::vector<int>& tile_15_ids, RoadTileDownloadParam* param, int thread_num,
                                     std::vector<TileInfoPtr>& tiles, bool update_ver) {
    std::map<int, std::vector<int>> tids_by11;
    std::stringstream ss;
    for (size_t i = 0; i < tile_15_ids.size(); ++i) {
        int tile_id = tile_15_ids[i];
        if (i < 8) {
            ss << tile_id << ", ";
        }
        else if (i == 8) {
            ss << "... ";
        }
        tids_by11[tile_id / 256].push_back(tile_id);
    }

    size_t tile_size = tile_15_ids.size();
    LOG_BEGIN(INFO) << "load_tiles_by_id() for tiles " << tile_15_ids.size() << '/' << tids_by11.size()
        << " with param " << param << " thread_num:" << thread_num; LOG_END;
    std::vector<TileInfoPtr> old_tiles;
    tiles.swap(old_tiles);
    tiles.reserve(tile_size * 3);
    std::vector<services::TileInfo*> tile_infos;
    tile_infos.reserve(tile_size * 3);
    int64_t sync_ver = 0;
    int retry_num = 0;
    bool bret = false;
    while (retry_num < 3 && !bret) {
        if (param && param != _download_param.get()) {
            *_download_param = *param;
        }
        _download_param->tile_15_ids.clear();
        _download_param->tile_15_ids.reserve(tids_by11.size());
        for (auto& tit : tids_by11) {
            _download_param->tile_15_ids.push_back(tit.second);
        }
        
        if (thread_num > 1) {
            bret = _road_tile_dao->download_tiles_parallel(*_download_param, thread_num, sync_ver, tile_infos);
        }
        else {
            bret = _road_tile_dao->download_tiles_one_by_one(*_download_param, sync_ver, tile_infos);
        }        
        retry_num++;
        if (retry_num < 3 && !bret) {
            LOG_BEGIN(ERROR) << "load_tiles_by_id() failed for tiles " << ss.str()
                << "sleep and retry " << retry_num; LOG_END;
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        }
    }    
    if (retry_num >= 3 && !bret) {
        LOG_BEGIN(ERROR) << "load_tiles_by_id() failed for tiles " << ss.str()
            << "exit!"; LOG_END;
        return false;
    }
    if (update_ver && sync_ver > 0) {
        _sync_ver = sync_ver;
        _prev_tick = get_tick_count();
        _current_ver = _sync_ver + 1;
    }
    LOG_BEGIN(INFO) << "load_tiles_by_id() succeed for tiles " << ss.str()
        << " with version: " << sync_ver << " tick: " << _prev_tick; LOG_END;
    std::set<int> current_tile_ids;
    std::map<int, std::pair<TileInfoPtr, int>> tileinfos;
    for (auto& t : old_tiles) {
        auto itr = tileinfos.find(t->tile_id());
        if (itr == tileinfos.end()) {
            tileinfos[t->tile_id()] = std::make_pair(t, -1);
        }
    }

    merge_tiles(tiles, tile_infos, tileinfos, current_tile_ids, thread_num);
    if (_download_param->load_ref) {
        return true;
    }
    std::unordered_set<int> tids(tile_size * 3);
    tids.insert(tile_15_ids.begin(), tile_15_ids.end());
    std::vector<int> ids;
    ids.reserve(tile_15_ids.size());
    for (auto& t : tiles) {
        for (auto& r : t->link_refs()) {
            if (r && tids.find(r->tileid()) == tids.end()) {
                ids.push_back(r->tileid());
                tids.insert(r->tileid());
            }
        }
        for (auto& r : t->node_refs()) {
            if (r && tids.find(r->tileid()) == tids.end()) {
                ids.push_back(r->tileid());
                tids.insert(r->tileid());
            }
        }
        for (auto& r : t->lane_refs()) {
            if (r && tids.find(r->tileid()) == tids.end()) {
                ids.push_back(r->tileid());
                tids.insert(r->tileid());
            }
        }
        for (auto& r : t->lane_boundary_refs()) {
            if (r && tids.find(r->tileid()) == tids.end()) {
                ids.push_back(r->tileid());
                tids.insert(r->tileid());
            }
        }
        for (auto& r : t->lane_group_refs()) {
            if (r && tids.find(r->tileid()) == tids.end()) {
                ids.push_back(r->tileid());
                tids.insert(r->tileid());
            }
        }
        for (auto& r : t->junction_refs()) {
            if (r && tids.find(r->tileid()) == tids.end()) {
                ids.push_back(r->tileid());
                tids.insert(r->tileid());
            }
        }
        for (auto& r : t->traffic_info_refs()) {
            if (r && tids.find(r->tileid()) == tids.end()) {
                ids.push_back(r->tileid());
                tids.insert(r->tileid());
            }
        }
        for (auto& r : t->position_object_refs()) {
            if (r && tids.find(r->tileid()) == tids.end()) {
                ids.push_back(r->tileid());
                tids.insert(r->tileid());
            }
        }
        for (auto& r : t->road_boundary_refs()) {
            if (r && tids.find(r->tileid()) == tids.end()) {
                ids.push_back(r->tileid());
                tids.insert(r->tileid());
            }
        }
        for (auto& r : t->data_quality_refs()) {
            if (r && tids.find(r->tileid()) == tids.end()) {
                ids.push_back(r->tileid());
                tids.insert(r->tileid());
            }
        }
        for (auto& r : t->dynamic_refs()) {
            if (r && tids.find(r->tileid()) == tids.end()) {
                ids.push_back(r->tileid());
                tids.insert(r->tileid());
            }
        }
    }
    tile_infos.clear();
    tids_by11.clear();
    ss.clear();
    for (size_t i = 0; i < ids.size(); ++i) {
        int tile_id = ids[i];
        ss << tile_id << ", ";
        tids_by11[tile_id / 256].push_back(tile_id);
    }
    LOG_BEGIN(INFO) << "load_tiles_by_id() for ref tiles " << ids.size()
        << " with param " << param << " thread_num:" << thread_num; LOG_END;
    retry_num = 0;
    bret = false;
    while (retry_num < 3 && !bret) {
        _download_param->tile_15_ids.clear();
        _download_param->tile_15_ids.reserve(tids_by11.size());
        for (auto& tit : tids_by11) {
            _download_param->tile_15_ids.push_back(tit.second);
        }
        if (thread_num > 1) {
            bret = _road_tile_dao->download_tiles_parallel(*_download_param, thread_num, sync_ver, tile_infos);
        }
        else {
            bret = _road_tile_dao->download_tiles_one_by_one(*_download_param, sync_ver, tile_infos);
        }
        retry_num++;
        if (retry_num < 3 && !bret) {
            LOG_BEGIN(ERROR) << "load_tiles_by_id() failed for ref tiles " << ss.str()
                << "sleep and retry " << retry_num; LOG_END;
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        }
    }
    if (retry_num >= 3 && !bret) {
        LOG_BEGIN(ERROR) << "load_tiles_by_id() failed for ref tiles " << ss.str()
            << "exit!"; LOG_END;
        return false;
    }
    merge_tiles(tiles, tile_infos, tileinfos, current_tile_ids, thread_num);
    return true;
};

bool RoadGeometryManager::load_tiles_by_id(const std::vector<int>& tile_15_ids, 
                                           RoadTileDownloadParam* param, int thread_num) {
    if (!_tile_inited) {
        if (!init_tiles_by_id(tile_15_ids)) {
            return false;
        }
    }

    std::vector<TileInfoPtr> tiles;
    {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        _current_download_tiles.clear();
        _tile15_ids.insert(tile_15_ids.begin(), tile_15_ids.end());
        tiles.reserve(_tiles.size());
        for (auto& tile : _tiles) {
            tiles.push_back(tile.second);
        }
        if (!load_tiles(tile_15_ids, param, thread_num, tiles, true)) {
            return false;
        }
        for (auto& tile : tiles) {
            _tiles[tile->_tile_id] = tile;
            _current_download_tiles[tile->_tile_id] = tile;
            //tile->load_maps();
            if (tile->_version > _sync_ver) {
                tile->_version = _sync_ver;
            }
        }
    }    
    remake_tile_proxy(tiles);
    if (!_download_param->judge_editable) {
        return true;
    }
    thread_num = std::max(1, thread_num);
    thread_num = std::min<int>(thread_num, tiles.size());
    LOG_BEGIN(INFO) << "judge_editable() start: "; LOG_END;
    std::vector<std::thread> threads;
    threads.reserve(thread_num);
    std::atomic<int> tile_ind(0);
    for (int ti = 0; ti < thread_num; ++ti) {
        threads.push_back(std::thread([&tiles, &tile_ind, this]() {
            while (true) {
                int ind = tile_ind.fetch_add(1);
                if (ind >= (int)tiles.size()) {
                    break;
                }
                auto& tile = tiles[ind];
                LOG_BEGIN(INFO) << "remake_tile_proxy() for tile' refs @ " << tile->tile_id();
                LOG_END;
                tile->links().judge_editable(this);
                tile->nodes().judge_editable(this);
                tile->lanes().judge_editable(this);
                tile->lane_boundarys().judge_editable(this);
                tile->lane_groups().judge_editable(this);
                tile->junctions().judge_editable(this);
                tile->traffic_infos().judge_editable(this);
                tile->position_objects().judge_editable(this);
                tile->road_boundarys().judge_editable(this);
                tile->data_qualitys().judge_editable(this);
                tile->dynamics().judge_editable(this);
            }
        }));
    }    
    for (int ti = 0; ti < thread_num; ++ti) {
        threads[ti].join();
    }    
    return true;
}

bool RoadGeometryManager::upload_tiles(const std::string& cur_editor, const TileInfoList& tiles,
                                        int max_tile11_num, int thread_num) {
    if (tiles.empty()) {
        LOG_BEGIN(ERROR) << "upload_tiles() failed: empty tiles!"; LOG_END;
        return true;
    }
    std::map<int, std::vector<TileInfoPtr>> tid11_map;
    for (auto& t : tiles) {
        if (!t->cached_is_valid()) {
            LOG_BEGIN(ERROR) << "upload_tiles() failed: tile " << t->tile_id()
                << '@' << t->version() << " is invalid."; LOG_END;
            return false;
        }
        tid11_map[t->tile_id() / 256].push_back(t);
    }
    
    /*if (load_and_merge(MergeConflictPolicy::MERGE_BEST, 5, _merged_tiles, _conflict_tiles)) {
        std::stringstream ss;
        for (auto& t : _conflict_tiles) {
            if (t) {
                ss << "tile" << t->tile_id();
            }
        }
        for (auto& t : _conflict_tiles) {
            if (!t) {
                continue;
            }            
        }
        LOG_BEGIN(ERROR) << "upload_tiles() failed: conflict with remote " << ss.str(); LOG_END;
        return false;
    }*/
    // CLOG_INFO("upload_tiles() for %d tiles with %d tiles", tiles.size(), tid11_map.size());
    // LOG_BEGIN(INFO) << "upload_tiles() for " << tiles.size() << " tiles with "
    //     << tid11_map.size() << "  tiles"; LOG_END;
    if (max_tile11_num == 0 || std::abs(max_tile11_num) >= (int)tid11_map.size()) {
        int64_t ver = _current_ver;
        if (_road_tile_dao->upload_tile(cur_editor, ver, tiles)) {
            set_current_version(ver);
            for (auto& t : tiles) {
                t->clear_changed();
            }
            for (auto& t : _merged_tiles) {
                t->reset_merge();
            }
            return true;
        }
        else {
            return false;
        }
    }
    else {
        std::vector<TileInfoList> sub_tiles;
        if (!split_tile_list_by_tile11(tiles, max_tile11_num, sub_tiles)) {
            return false;
        }
        bool bfailed = false;
        if (thread_num <= 1) {
            int64_t ver = _current_ver;
            for (size_t i = 0; i < sub_tiles.size(); i++) {
                auto& ts = sub_tiles[i];
                
                // LOG_BEGIN(INFO) << "upload_tiles() for subset " << i << " / " << sub_tiles.size() 
                //         << " with " << ts.size() << " tiles"; LOG_END;                
                bool bf = true;
                for (int retry = 0; retry < 10; retry++) {
                    if (_road_tile_dao->upload_tile(cur_editor, ver, ts)) {
                        set_current_version(ver);
                        LOG_BEGIN(INFO) << "upload_tiles() succeed with version " << ver; LOG_END;
                        for (auto& t : ts) {
                            t->clear_changed();                    
                        }
                        bf = false;
                        break;
                    }
                    else {
                        // CLOG_ERROR("upload_tiles() for %d tiles failed, retry %d ",ts.size(),retry);
                        // LOG_BEGIN(ERROR) << "upload_tiles() for " << ts.size()
                        //     << " tiles failed, retry " << retry; LOG_END;
                    }
                }
                if (bf) {
                    bfailed = true;
                }
            }
        }
        else {
            thread_num = std::min(thread_num, (int)sub_tiles.size());
            std::vector<std::thread> threads;
            threads.reserve(thread_num);
            std::atomic<int> ind(0);
            for (int ti = 0; ti < thread_num; ++ti) {
                threads.push_back(std::thread([this, &ind, &sub_tiles, &bfailed, &cur_editor]() {
                    while (true) {
                        int i = ind.fetch_add(1);
                        if (i >= (int)sub_tiles.size()) {
                            break;
                        }
                        auto& ts = sub_tiles[i];
                        int64_t ver = 0;
                        bool bf = true;
                        for (int retry = 0; retry < 10; retry++) {
                            LOG_BEGIN(INFO) << "upload_tiles() parallel for subset " << i << " / " << sub_tiles.size() 
                                            << " with " << ts.size() << " tiles"; LOG_END;
                            if (_road_tile_dao->upload_tile(cur_editor, ver, ts)) {
                                set_current_version(ver);
                                LOG_BEGIN(INFO) << "upload_tiles() parallel succeed with version " << ver; LOG_END;
                                for (auto& t : ts) {
                                    t->clear_changed();                    
                                }
                                bf = false;
                                break;
                            }
                            else {
                                LOG_BEGIN(ERROR) << "upload_tiles() parallel for tiles " << ts.size() << " failed, retry "
                                    << retry; LOG_END;
                                std::this_thread::sleep_for(std::chrono::seconds(retry));
                            }
                        }
                        if (bf) {
                            bfailed = true;
                        }
                    }
                }));
            }
            for (int ti = 0; ti < thread_num; ++ti) {
                threads[ti].join();                
            }
        }
        return !bfailed;
    }
}

bool RoadGeometryManager::load_and_merge(MergeConflictPolicy p, int thread_num, TileInfoList& merged_tiles, TileInfoList& conflict_tiles) {
    if (_tile15_ids.empty() || !_download_param) {
        return false;
    }
    merged_tiles.clear();
    conflict_tiles.clear();
    _merge_issues.clear();
    std::vector<TileInfoPtr> tiles;
    std::vector<int> tids(_tile15_ids.begin(), _tile15_ids.end());
    auto param = *_download_param;
    param.req_ver = 0;
    if (!load_tiles(tids, &param, thread_num, tiles, false)) {
        return false;
    }
    for (auto t : tiles) {
        auto tile = get_road_tile(t->tile_id());
        tile->restart_merge();
        tile->merge_tile(t.get());
        tile->merge(p, this);
        if (tile->changed_tile()) {
            merged_tiles.push_back(tile->mutable_changed_tile());
        }
        if (tile->conflicted_tile()) {
            conflict_tiles.push_back(tile->mutable_conflict_tile());
        }
    }
    resolve_tiles();
    correct_tiles();
    for (auto t : conflict_tiles) {
        t->append_to_issue(this);
    }
    return true;
}

int tile_id_distance(int tid1, int tid2) {
    int lx1 = 0;
    int ly1 = 0;
    int lx2 = 0;
    int ly2 = 0;
    for (int i = 0; i <= 15; ++i) {
        lx1 |= (tid1 & (1 << (i * 2))) >> i;
        ly1 |= (tid1 & (1 << (i * 2 + 1))) >> (i + 1);
        lx2 |= (tid2 & (1 << (i * 2))) >> i;
        ly2 |= (tid2 & (1 << (i * 2 + 1))) >> (i + 1);
    }
    return std::abs(lx1 - lx2) + std::abs(ly1 - ly2);
}

TileInfoPtr RoadGeometryManager::get_tile_in_map(int tile_id, bool link_parent, ID2TileMap& id2tiles) {
    auto tile = id2tiles[tile_id];
    if (!tile) {
        tile.reset(FeatureProxyBase::create_proxy<TileInfoProxy>());
        if (link_parent) {
            tile->relink_parent();
        }
        tile->set_tile_id(tile_id);
        tile->set_version(0);
        id2tiles[tile_id] = tile;
    }
    return tile;
}

bool RoadGeometryManager::split_tile_list_by_tile11(const TileInfoList& tiles, int max_tile11_num,
                                                    std::vector<TileInfoList>& sub_tiles) {
    std::map<int, std::vector<TileInfoPtr>> tid11_map;
    std::map<int, TileInfoPtr> id2tiles;
    if (max_tile11_num < 0) {
        for (auto& t : tiles) {
            tid11_map[t->tile_id() / 256].push_back(t);
        }
        LOG_BEGIN(INFO) << "split_tile_list_by_tiles11() for " << tid11_map.size()
            << " tile11 with max_tile11_num:" << max_tile11_num; LOG_END;
        max_tile11_num = -max_tile11_num;
        int cnt = 0;
        sub_tiles.emplace_back();
        for (auto& tit : tid11_map) {
            sub_tiles.back().insert(sub_tiles.back().end(), tit.second.begin(), tit.second.end());
            cnt++;
            if (cnt >= max_tile11_num) {
                cnt = 0;
                sub_tiles.emplace_back();
            }
        }
        if (sub_tiles.back().empty()) {
            sub_tiles.resize(sub_tiles.size() - 1);
        }
        return true;
    }
    for (auto& t : tiles) {
        /*if (!t->cached_is_valid()) {
            return false;
        }*/
        t->load_maps();
        id2tiles[t->tile_id()] = t;
        tid11_map[t->tile_id() / 256].push_back(t);
    }
    std::stringstream ss;
    std::multimap<int, int> cnt2tid;
    for (auto& tit : tid11_map) {
        auto& ts = tit.second;
        int cnt = 0;
        for (auto& t : ts) {
            cnt += (int)t->lane_boundarys().size();
            cnt += (int)t->road_boundarys().size();
            cnt += (int)t->lanes().size();
            cnt += (int)t->lane_groups().size();
            cnt += (int)t->links().size();
            cnt += (int)t->junctions().size();
            cnt += (int)t->nodes().size();
        }
        cnt2tid.insert(std::make_pair(-cnt, tit.first));
        ss << tit.first << ':' << cnt << ", ";
    }
    LOG_BEGIN(INFO) << "split_tile_list_by_tiles11() for tile11 " << ss.str()
        << " with max_tile11_num:" << max_tile11_num; LOG_END;
    std::set<int> used_tids;
    for (auto& tit : cnt2tid) {
        int tid = tit.second;
        if (used_tids.count(tid) > 0) {
            continue;
        }
        used_tids.insert(tid);
        std::set<int> cur_tids;
        ss.clear();
        cur_tids.insert(tid);
        for (int ti = 1; ti < max_tile11_num; ++ti) {
            int mindis = (1 << 25);
            int bestid = 0;
            for (auto& cit : cnt2tid) {
                int id = cit.second;
                if (used_tids.count(id) > 0) {
                    continue;
                }
                int dis = tile_id_distance(tid, id);
                if (mindis > dis) {
                    mindis = dis;
                    bestid = id;
                }
            }
            if (bestid > 0) {
                cur_tids.insert(bestid);
                used_tids.insert(bestid);
                ss << bestid << ", ";
            }
        }
        LOG_BEGIN(INFO) << "split_tile_list_by_tile11() copy element for tile11 "
            << ss.str(); LOG_END;
        ID2TileMap tmp_tile_map;
        std::set<std::string> lanes;
        for (int tid : cur_tids) {
            auto& ts = tid11_map[tid];
            for (auto& tile : ts) {
                auto new_tile = get_tile_in_map(tile->tile_id(), false, tmp_tile_map);
                new_tile->shallow_copy(tile.get());
                for (size_t i = 0; i < new_tile->lanes().size();) {
                    auto p = new_tile->lanes()[i];
                    if (!p) {
                        ++i;
                        continue;
                    }
                    auto es = p->owner_elems();
                    if (es.size() == 1) {
                        auto seg = *es.begin();
                        if (seg && cur_tids.count(seg->id()->tileid() / 256) <= 0
                                && id2tiles.count(seg->id()->tileid()) > 0) {
                            auto t = id2tiles[seg->id()->tileid()];
                            if (t && t->_lane_map.count(seg->id()->to_binary_string()) > 0) {
                                LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() erase "
                                    << p->id()->to_string() << " of " << seg->id()->to_string();
                                LOG_END;
                                new_tile->lanes().erase(i);
                                auto& rs = new_tile->lane_refs();
                                auto it = rs.find(p->id());
                                if (it != rs.end()) {
                                    rs.erase(it);
                                }
                                continue;
                            }
                        }
                    }
                    auto tiles = p->referenced_tiles();
                    for (auto& t : tiles) {
                        if (cur_tids.count(t->tile_id() / 256) > 0) {
                            continue;
                        }
                        auto e = t->lane_refs().find(*(p->id()));
                        if (e) {
                            auto tt = get_tile_in_map(t->tile_id(), false, tmp_tile_map);
                            tt->mutable_lane_refs()->push_back(e);
                            LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() append ref "
                                << e->to_string() << " to tile " << t->tile_id(); LOG_END;
                        }
                    }
                    ++i;
                }
                for (size_t i = 0; i < new_tile->lane_refs().size();) {
                    auto& id = new_tile->lane_refs()[i];
                    if (cur_tids.count(id->tileid() / 256) > 0) {
                        ++i;
                        continue;
                    }
                    if (id2tiles.count(id->tileid()) > 0
                        && id2tiles[id->tileid()]->_lane_map
                        .count(id->to_binary_string()) > 0) {
                        LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() erase ref "
                            << id->to_string() << " to tile " << new_tile->tile_id(); LOG_END;
                        new_tile->mutable_lane_refs()->erase(i);
                    }
                    else {
                        ++i;
                    }
                }
                for (size_t i = 0; i < new_tile->road_boundarys().size();) {
                    auto p = new_tile->road_boundarys()[i];
                    if (!p) {
                        ++i;
                        continue;
                    }
                    auto es = p->owner_elems();
                    if (es.size() == 1) {
                        auto seg = *es.begin();
                        if (seg && cur_tids.count(seg->id()->tileid() / 256) <= 0
                                && id2tiles.count(seg->id()->tileid()) > 0) {
                            auto t = id2tiles[seg->id()->tileid()];
                            if (t && t->_lane_group_map.count(seg->id()->to_binary_string()) > 0) {
                                LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() erase "
                                    << p->id()->to_string() << " of " << seg->id()->to_string();
                                LOG_END;
                                new_tile->road_boundarys().erase(i);
                                auto& rs = new_tile->road_boundary_refs();
                                auto it = rs.find(p->id());
                                if (it != rs.end()) {
                                    rs.erase(it);
                                }
                                continue;
                            }
                        }                        
                    }
                    auto tiles = p->referenced_tiles();
                    for (auto& t : tiles) {
                        if (cur_tids.count(t->tile_id() / 256) > 0) {
                            continue;
                        }
                        auto e = t->road_boundary_refs().find(*(p->id()));
                        if (e) {
                            auto tt = get_tile_in_map(t->tile_id(), false, tmp_tile_map);
                            tt->mutable_road_boundary_refs()->push_back(e);
                            LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() append ref "
                                << e->to_string() << " to tile " << t->tile_id(); LOG_END;
                        }
                    }
                    ++i;
                }
                for (size_t i = 0; i < new_tile->road_boundary_refs().size();) {
                    auto& id = new_tile->road_boundary_refs()[i];
                    if (cur_tids.count(id->tileid() / 256) > 0) {
                        ++i;
                        continue;
                    }
                    if (id2tiles.count(id->tileid()) > 0
                            && id2tiles[id->tileid()]->_road_boundary_map
                                .count(id->to_binary_string()) > 0) {
                        LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() erase ref "
                            << id->to_string() << " to tile " << new_tile->tile_id(); LOG_END;
                        new_tile->mutable_road_boundary_refs()->erase(i);
                    }
                    else {
                        ++i;
                    }
                }
                for (auto& seg : new_tile->lane_groups()) {
                    for (auto& id : seg->lanes()) {
                        if (cur_tids.count(id->tileid() / 256) > 0) {
                            continue;
                        }
                        if (id2tiles.count(id->tileid()) <= 0) {
                            continue;
                        }
                        auto org_tile = id2tiles[id->tileid()];
                        auto it = org_tile->_lane_map.find(id->to_binary_string());
                        if (it == org_tile->_lane_map.end()) {
                            continue;
                        }
                        auto p = std::const_pointer_cast<LaneProxy>(it->second.lock());
                        auto tmp_tile = get_tile_in_map(p->id()->tileid(), false, tmp_tile_map);
                        tmp_tile->mutable_lanes()->push_back(p);
                        LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() append "
                            << p->id()->to_string() << " of " << seg->id()->to_string();
                        LOG_END;
                    }
                    for (auto& id : seg->left_boundarys()) {
                        if (cur_tids.count(id->tileid() / 256) > 0) {
                            continue;
                        }
                        if (id2tiles.count(id->tileid()) <= 0) {
                            continue;
                        }
                        auto org_tile = id2tiles[id->tileid()];
                        auto it = org_tile->_road_boundary_map.find(id->to_binary_string());
                        if (it == org_tile->_road_boundary_map.end()) {
                            continue;
                        }
                        auto p = std::const_pointer_cast<RoadBoundaryProxy>(it->second.lock());
                        auto tmp_tile = get_tile_in_map(p->id()->tileid(), false, tmp_tile_map);
                        tmp_tile->mutable_road_boundarys()->push_back(p);
                        LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() append "
                            << p->id()->to_string() << " of " << seg->id()->to_string();
                        LOG_END;
                    }
                    for (auto& id : seg->right_boundarys()) {
                        if (cur_tids.count(id->tileid() / 256) > 0) {
                            continue;
                        }
                        if (id2tiles.count(id->tileid()) <= 0) {
                            continue;
                        }
                        auto org_tile = id2tiles[id->tileid()];
                        auto it = org_tile->_road_boundary_map.find(id->to_binary_string());
                        if (it == org_tile->_road_boundary_map.end()) {
                            continue;
                        }
                        auto p = std::const_pointer_cast<RoadBoundaryProxy>(it->second.lock());
                        auto tmp_tile = get_tile_in_map(p->id()->tileid(), false, tmp_tile_map);
                        tmp_tile->mutable_road_boundarys()->push_back(p);
                        LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() append "
                            << p->id()->to_string() << " of " << seg->id()->to_string();
                        LOG_END;
                    }
                    auto tiles = seg->referenced_tiles();
                    for (auto& t : tiles) {
                        if (cur_tids.count(t->tile_id() / 256) > 0) {
                            continue;
                        }
                        auto e = t->lane_group_refs().find(*(seg->id()));
                        if (e) {
                            auto tt = get_tile_in_map(t->tile_id(), false, tmp_tile_map);
                            tt->mutable_lane_group_refs()->push_back(e);
                            LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() append ref "
                                << e->to_string() << " to tile " << t->tile_id(); LOG_END;
                        }
                    }
                }
                for (size_t i = 0; i < new_tile->lane_group_refs().size();) {
                    auto& id = new_tile->lane_group_refs()[i];
                    if (cur_tids.count(id->tileid() / 256) > 0) {
                        ++i;
                        continue;
                    }
                    if (id2tiles.count(id->tileid()) > 0
                            && id2tiles[id->tileid()]->_lane_group_map
                                .count(id->to_binary_string()) > 0) {
                        LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() erase ref "
                            << id->to_string() << " to tile " << new_tile->tile_id(); LOG_END;
                        new_tile->mutable_lane_group_refs()->erase(i);
                    }
                    else {
                        ++i;
                    }
                }                
                for (auto& seg : new_tile->links()) {
                    auto tiles = seg->referenced_tiles();
                    for (auto& t : tiles) {
                        if (cur_tids.count(t->tile_id() / 256) > 0) {
                            continue;
                        }
                        auto e = t->link_refs().find(*(seg->id()));
                        if (e) {
                            auto tt = get_tile_in_map(t->tile_id(), false, tmp_tile_map);
                            tt->mutable_link_refs()->push_back(e);
                            LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() append ref "
                                << e->to_string() << " to tile " << t->tile_id(); LOG_END;
                        }
                    }
                }
                for (size_t i = 0; i < new_tile->link_refs().size();) {
                    auto& id = new_tile->link_refs()[i];
                    if (cur_tids.count(id->tileid() / 256) > 0) {
                        ++i;
                        continue;
                    }
                    if (id2tiles.count(id->tileid()) > 0
                            && id2tiles[id->tileid()]->_link_map
                            .count(id->to_binary_string()) > 0) {
                        LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() erase ref "
                            << id->to_string() << " to tile " << new_tile->tile_id(); LOG_END;
                        new_tile->mutable_link_refs()->erase(i);
                    }
                    else {
                        ++i;
                    }
                }
                for (auto& seg : new_tile->nodes()) {
                    auto tiles = seg->referenced_tiles();
                    for (auto& t : tiles) {
                        if (cur_tids.count(t->tile_id() / 256) > 0) {
                            continue;
                        }
                        auto e = t->node_refs().find(*(seg->id()));
                        if (e) {
                            auto tt = get_tile_in_map(t->tile_id(), false, tmp_tile_map);
                            tt->mutable_node_refs()->push_back(e);
                            LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() append ref "
                                << e->to_string() << " to tile " << t->tile_id(); LOG_END;
                        }
                    }
                }
                for (size_t i = 0; i < new_tile->node_refs().size();) {
                    auto& id = new_tile->node_refs()[i];
                    if (cur_tids.count(id->tileid() / 256) > 0) {
                        ++i;
                        continue;
                    }
                    if (id2tiles.count(id->tileid()) > 0
                            && id2tiles[id->tileid()]->_node_map
                            .count(id->to_binary_string()) > 0) {
                        LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() erase ref "
                            << id->to_string() << " to tile " << new_tile->tile_id(); LOG_END;
                        new_tile->mutable_node_refs()->erase(i);
                    }
                    else {
                        ++i;
                    }
                }
                for (auto& seg : new_tile->junctions()) {
                    auto tiles = seg->referenced_tiles();
                    for (auto& t : tiles) {
                        if (cur_tids.count(t->tile_id() / 256) > 0) {
                            continue;
                        }
                        auto e = t->junction_refs().find(*(seg->id()));
                        if (e) {
                            auto tt = get_tile_in_map(t->tile_id(), false, tmp_tile_map);
                            tt->mutable_junction_refs()->push_back(e);
                            LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() append ref "
                                << e->to_string() << " to tile " << t->tile_id(); LOG_END;
                        }
                    }
                }
                for (size_t i = 0; i < new_tile->junction_refs().size();) {
                    auto& id = new_tile->junction_refs()[i];
                    if (cur_tids.count(id->tileid() / 256) > 0) {
                        ++i;
                        continue;
                    }
                    if (id2tiles.count(id->tileid()) > 0
                            && id2tiles[id->tileid()]->_junction_map
                            .count(id->to_binary_string()) > 0) {
                        LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() erase ref "
                            << id->to_string() << " to tile " << new_tile->tile_id(); LOG_END;
                        new_tile->mutable_junction_refs()->erase(i);
                    }
                    else {
                        ++i;
                    }
                }
                for (auto& seg : new_tile->traffic_infos()) {
                    auto tiles = seg->referenced_tiles();
                    for (auto& t : tiles) {
                        if (cur_tids.count(t->tile_id() / 256) > 0) {
                            continue;
                        }
                        auto e = t->traffic_info_refs().find(*(seg->id()));
                        if (e) {
                            auto tt = get_tile_in_map(t->tile_id(), false, tmp_tile_map);
                            tt->mutable_traffic_info_refs()->push_back(e);
                            LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() append ref "
                                << e->to_string() << " to tile " << t->tile_id(); LOG_END;
                        }
                    }
                }
                for (size_t i = 0; i < new_tile->traffic_info_refs().size();) {
                    auto& id = new_tile->traffic_info_refs()[i];
                    if (cur_tids.count(id->tileid() / 256) > 0) {
                        ++i;
                        continue;
                    }
                    if (id2tiles.count(id->tileid()) > 0
                            && id2tiles[id->tileid()]->_traffic_info_map
                            .count(id->to_binary_string()) > 0) {
                        LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() erase ref "
                            << id->to_string() << " to tile " << new_tile->tile_id(); LOG_END;
                        new_tile->mutable_traffic_info_refs()->erase(i);
                    }
                    else {
                        ++i;
                    }
                }
                for (auto& seg : new_tile->position_objects()) {
                    auto tiles = seg->referenced_tiles();
                    for (auto& t : tiles) {
                        if (cur_tids.count(t->tile_id() / 256) > 0) {
                            continue;
                        }
                        auto e = t->position_object_refs().find(*(seg->id()));
                        if (e) {
                            auto tt = get_tile_in_map(t->tile_id(), false, tmp_tile_map);
                            tt->mutable_position_object_refs()->push_back(e);
                            LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() append ref "
                                << e->to_string() << " to tile " << t->tile_id(); LOG_END;
                        }
                    }
                }
                for (size_t i = 0; i < new_tile->position_object_refs().size();) {
                    auto& id = new_tile->position_object_refs()[i];
                    if (cur_tids.count(id->tileid() / 256) > 0) {
                        ++i;
                        continue;
                    }
                    if (id2tiles.count(id->tileid()) > 0
                            && id2tiles[id->tileid()]->_position_object_map
                            .count(id->to_binary_string()) > 0) {
                        LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() erase ref "
                            << id->to_string() << " to tile " << new_tile->tile_id(); LOG_END;
                        new_tile->mutable_position_object_refs()->erase(i);
                    }
                    else {
                        ++i;
                    }
                }
                for (auto& seg : new_tile->data_qualitys()) {
                    auto tiles = seg->referenced_tiles();
                    for (auto& t : tiles) {
                        if (cur_tids.count(t->tile_id() / 256) > 0) {
                            continue;
                        }
                        auto e = t->data_quality_refs().find(*(seg->id()));
                        if (e) {
                            auto tt = get_tile_in_map(t->tile_id(), false, tmp_tile_map);
                            tt->mutable_data_quality_refs()->push_back(e);
                            LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() append ref "
                                << e->to_string() << " to tile " << t->tile_id(); LOG_END;
                        }
                    }
                }
                for (size_t i = 0; i < new_tile->data_quality_refs().size();) {
                    auto& id = new_tile->data_quality_refs()[i];
                    if (cur_tids.count(id->tileid() / 256) > 0) {
                        ++i;
                        continue;
                    }
                    if (id2tiles.count(id->tileid()) > 0
                            && id2tiles[id->tileid()]->_data_quality_map
                            .count(id->to_binary_string()) > 0) {
                        LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() erase ref "
                            << id->to_string() << " to tile " << new_tile->tile_id(); LOG_END;
                        new_tile->mutable_data_quality_refs()->erase(i);
                    }
                    else {
                        ++i;
                    }
                }
                for (auto& seg : new_tile->dynamics()) {
                    auto tiles = seg->referenced_tiles();
                    for (auto& t : tiles) {
                        if (cur_tids.count(t->tile_id() / 256) > 0) {
                            continue;
                        }
                        auto e = t->dynamic_refs().find(*(seg->id()));
                        if (e) {
                            auto tt = get_tile_in_map(t->tile_id(), false, tmp_tile_map);
                            tt->mutable_dynamic_refs()->push_back(e);
                            LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() append ref "
                                << e->to_string() << " to tile " << t->tile_id(); LOG_END;
                        }
                    }
                }
                for (size_t i = 0; i < new_tile->dynamic_refs().size();) {
                    auto& id = new_tile->dynamic_refs()[i];
                    if (cur_tids.count(id->tileid() / 256) > 0) {
                        ++i;
                        continue;
                    }
                    if (id2tiles.count(id->tileid()) > 0
                            && id2tiles[id->tileid()]->_dynamic_map
                            .count(id->to_binary_string()) > 0) {
                        LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() erase ref "
                            << id->to_string() << " to tile " << new_tile->tile_id(); LOG_END;
                        new_tile->mutable_dynamic_refs()->erase(i);
                    }
                    else {
                        ++i;
                    }
                }
            }
        }
        for (auto& tit : tmp_tile_map) {
            tit.second->load_maps();
        }
        for (auto& new_tile : tiles) {
            for (size_t i = 0; i < new_tile->lane_boundarys().size();) {
                auto p = new_tile->lane_boundarys()[i];
                if (!p) {
                    ++i;
                    continue;
                }
                auto es = p->owner_elems();
                for (auto seg : es) {
                    if (seg && cur_tids.count(seg->id()->tileid() / 256) <= 0
                        && tmp_tile_map.count(seg->id()->tileid()) > 0) {
                        auto t = tmp_tile_map[seg->id()->tileid()];
                        if (t && t->_lane_map.count(seg->id()->to_binary_string()) > 0) {
                            LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() erase "
                                << p->id()->to_string() << " of " << seg->id()->to_string();
                            LOG_END;
                            new_tile->lane_boundarys().erase(i);
                            auto& rs = new_tile->lane_boundary_refs();
                            auto it = rs.find(p->id());
                            if (it != rs.end()) {
                                rs.erase(it);
                            }
                            continue;
                        }
                    }
                }
                auto tiles = p->referenced_tiles();
                for (auto& t : tiles) {
                    if (cur_tids.count(t->tile_id() / 256) > 0) {
                        continue;
                    }
                    auto e = t->lane_boundary_refs().find(*(p->id()));
                    if (e) {
                        auto tt = get_tile_in_map(t->tile_id(), false, tmp_tile_map);
                        tt->mutable_lane_boundary_refs()->push_back(e);
                        LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() append ref "
                            << e->to_string() << " to tile " << t->tile_id(); LOG_END;
                    }
                }
                ++i;
            }
            for (size_t i = 0; i < new_tile->lane_boundary_refs().size();) {
                auto& id = new_tile->lane_boundary_refs()[i];
                if (cur_tids.count(id->tileid() / 256) > 0) {
                    ++i;
                    continue;
                }
                if (id2tiles.count(id->tileid()) > 0
                        && id2tiles[id->tileid()]->_lane_boundary_map
                            .count(id->to_binary_string()) > 0) {
                    LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() erase ref "
                        << id->to_string() << " to tile " << new_tile->tile_id(); LOG_END;
                    new_tile->mutable_lane_boundary_refs()->erase(i);
                }
                else {
                    ++i;
                }
            }
            for (auto& seg : new_tile->lanes()) {
                for (auto& sec : seg->lanes()) {
                    for (auto& mem : sec->contained_members()) {
                        if (cur_tids.count(mem->id()->tileid() / 256) > 0) {
                            continue;
                        }
                        if (id2tiles.count(mem->id()->tileid()) <= 0) {
                            continue;
                        }
                        auto org_tile = id2tiles[mem->id()->tileid()];
                        auto it = org_tile->_lane_boundary_map.find(mem->id()->to_binary_string());
                        if (it == org_tile->_lane_boundary_map.end()) {
                            continue;
                        }
                        auto p = std::const_pointer_cast<LaneBoundaryProxy>(it->second.lock());
                        auto tmp_tile = get_tile_in_map(p->id()->tileid(), false, tmp_tile_map);
                        tmp_tile->mutable_lane_boundarys()->push_back(p);
                        LOG_BEGIN(DEBUG) << "split_tile_list_by_tile11() append "
                            << p->id()->to_string() << " of " << seg->id()->to_string();
                        LOG_END;
                    }
                }
            }
        }
        TileInfoList tiles;
        tiles.reserve(tmp_tile_map.size());
        for (auto& tit : tmp_tile_map) {
            tiles.push_back(tit.second);
        }
        sub_tiles.push_back(tiles);
    }
    return true;
}

bool RoadGeometryManager::set_current_version(int64_t ver) {
    std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
    _sync_ver = ver - 1;
    _current_ver = ver;
    _prev_tick = get_tick_count();
    // LOG_DEBUG("set_current_version() for version: %d tick: %d", ver , _prev_tick);
    // LOG_BEGIN(INFO) << "set_current_version() for version:" << ver
    //     << " tick:" << _prev_tick; LOG_END;
    return true;
}
int64_t RoadGeometryManager::current_version() const {
    std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
    return _current_ver;
}
int64_t RoadGeometryManager::download_version() const {
    std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
    return _sync_ver;
}

bool RoadGeometryManager::get_road_tiles(ID2TileMap& tiles) {
    std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
    tiles = _tiles;
    return true;    
}
bool RoadGeometryManager::get_current_download_tiles(ID2TileMap& tiles) {
    std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
    tiles = _current_download_tiles;
    return true;
}
TileInfoPtr RoadGeometryManager::get_road_tile(int tile_id) {
    std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
    return get_tile_in_map(tile_id, true, _tiles);
}
bool RoadGeometryManager::set_road_tile(int tile_id, TileInfoPtr& tile) {
    std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
    _tiles[tile_id] = tile;
    return true;
}

bool RoadGeometryManager::is_tile_15_downloaded(int tile_id) const {
    std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
    return _tile15_ids.count(tile_id) > 0;
}
bool RoadGeometryManager::is_tile_15_current_downloaded(int tile_id) const {
    std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
    return _current_download_tiles.count(tile_id) > 0;
}

bool RoadGeometryManager::is_editable_in_tiles(const std::map<int, double>& tid2lens) {
    if (tid2lens.empty()) {
        return true;
    }
    std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
    /*double total = 0;
    double val = 0;
    for (auto& tit : tid2lens) {
        total += tit.second;
        if (_tile15_ids.count(tit.first) > 0) {
            val += tit.second;
        }
    }
    if (std::abs(total) > 0.01) {
        return (val * 2 > total);
    }
    return (val > 0);*/
    double max_val = 0;
    int max_tid = 0;
    for (auto& tit : tid2lens) {
        if (max_val < tit.second) {
            max_val = tit.second;
            max_tid = tit.first;
        }
    }
    if (max_tid > 0) {
        return _tile15_ids.count(max_tid) > 0;
    }
    return true;
}

bool RoadGeometryManager::get_editable_tile_data(TileInfoList& tiles) {
    std::vector<TileInfoPtr> ts;
    {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        ts.reserve(_tiles.size());
        for (auto& tile : _tiles) {
            ts.push_back(tile.second);
        }
    }
    for (auto& tile : ts) {
        TileInfoPtr new_tile(FeatureProxyBase::create_proxy<TileInfoProxy>());
        new_tile->set_version(tile->version());
        new_tile->set_tile_id(tile->tile_id());
        bool bempty = true; 
        for (auto& p : tile->links()) {
            if (!p) {
                continue;
            }
            if (p->is_editable() || p->is_deleted()) {
                p->correct_version(this);
                new_tile->links().push_back(p);
                bempty = false;
                //LinkProxy::message_type msg;
                //p->to_message(&msg);
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found elem "
                    << p->id()->to_string()/* << " with " << msg.ByteSize() << " Bytes"*/; LOG_END;
            }
        }            
        for (auto& p : tile->link_refs()) {
            if (!p || (!p->is_del() && !p->owner())) {
                continue;
            }
            if (p->is_del() || p->owner()->is_editable()) {
                new_tile->link_refs().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found ref %s",p->to_string());
                // LOG_BEGIN(DEBUG) << "get_changed_tile_data() found ref "
                //     << p->to_string(); LOG_END;
            }
        }            
        for (auto& p : tile->nodes()) {
            if (!p) {
                continue;
            }
            if (p->is_editable() || p->is_deleted()) {
                p->correct_version(this);
                new_tile->nodes().push_back(p);
                bempty = false;
                //NodeProxy::message_type msg;
                //p->to_message(&msg);
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found elem "
                    << p->id()->to_string()/* << " with " << msg.ByteSize() << " Bytes"*/; LOG_END;
            }
        }
        for (auto& p : tile->node_refs()) {
            if (!p || (!p->is_del() && !p->owner())) {
                continue;
            }
            if (p->is_del() || p->owner()->is_editable()) {
                new_tile->node_refs().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found ref "
                    << p->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->lanes()) {
            if (!p) {
                continue;
            }
            if (p->is_editable() || p->is_deleted()) {
                p->correct_version(this);
                new_tile->lanes().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found elem "
                    << p->id()->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->lane_refs()) {
            if (!p || (!p->is_del() && !p->owner())) {
                continue;
            }
            if (p->is_del() || p->owner()->is_editable()) {
                new_tile->lane_refs().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found ref "
                    << p->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->lane_boundarys()) {
            if (!p) {
                continue;
            }
            if (p->is_editable() || p->is_deleted()) {
                p->correct_version(this);
                new_tile->lane_boundarys().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found elem "
                    << p->id()->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->lane_boundary_refs()) {
            if (!p || (!p->is_del() && !p->owner())) {
                continue;
            }
            if (p->is_del() || p->owner()->is_editable()) {
                new_tile->lane_boundary_refs().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found ref "
                    << p->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->lane_groups()) {
            if (!p) {
                continue;
            }
            if (p->is_editable() || p->is_deleted()) {
                p->correct_version(this);
                new_tile->lane_groups().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found elem "
                    << p->id()->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->lane_group_refs()) {
            if (!p || (!p->is_del() && !p->owner())) {
                continue;
            }
            if (p->is_del() || p->owner()->is_editable()) {
                new_tile->lane_group_refs().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found ref "
                    << p->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->junctions()) {
            if (!p) {
                continue;
            }
            if (p->is_editable() || p->is_deleted()) {
                p->correct_version(this);
                new_tile->junctions().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found elem "
                    << p->id()->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->junction_refs()) {
            if (!p || (!p->is_del() && !p->owner())) {
                continue;
            }
            if (p->is_del() || p->owner()->is_editable()) {
                new_tile->junction_refs().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found ref "
                    << p->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->traffic_infos()) {
            if (!p) {
                continue;
            }
            if (p->is_editable() || p->is_deleted()) {
                p->correct_version(this);
                new_tile->traffic_infos().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found elem "
                    << p->id()->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->traffic_info_refs()) {
            if (!p || (!p->is_del() && !p->owner())) {
                continue;
            }
            if (p->is_del() || p->owner()->is_editable()) {
                new_tile->traffic_info_refs().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found ref "
                    << p->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->position_objects()) {
            if (!p) {
                continue;
            }
            if (p->is_editable() || p->is_deleted()) {
                p->correct_version(this);
                new_tile->position_objects().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found elem "
                    << p->id()->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->position_object_refs()) {
            if (!p || (!p->is_del() && !p->owner())) {
                continue;
            }
            if (p->is_del() || p->owner()->is_editable()) {
                new_tile->position_object_refs().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found ref "
                    << p->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->road_boundarys()) {
            if (!p) {
                continue;
            }
            if (p->is_editable() || p->is_deleted()) {
                p->correct_version(this);
                new_tile->road_boundarys().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found elem "
                    << p->id()->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->road_boundary_refs()) {
            if (!p || (!p->is_del() && !p->owner())) {
                continue;
            }
            if (p->is_del() || p->owner()->is_editable()) {
                new_tile->road_boundary_refs().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found ref "
                    << p->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->data_qualitys()) {
            if (!p) {
                continue;
            }
            if (p->is_editable() || p->is_deleted()) {
                p->correct_version(this);
                new_tile->data_qualitys().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found elem "
                    << p->id()->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->data_quality_refs()) {
            if (!p || (!p->is_del() && !p->owner())) {
                continue;
            }
            if (p->is_del() || p->owner()->is_editable()) {
                new_tile->data_quality_refs().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found ref "
                    << p->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->dynamics()) {
            if (!p) {
                continue;
            }
            if (p->is_editable() || p->is_deleted()) {
                p->correct_version(this);
                new_tile->dynamics().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found elem "
                    << p->id()->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->dynamic_refs()) {
            if (!p || (!p->is_del() && !p->owner())) {
                continue;
            }
            if (p->is_del() || p->owner()->is_editable()) {
                new_tile->dynamic_refs().push_back(p);
                bempty = false;
                LOG_BEGIN(DEBUG) << "get_editable_tile_data() found ref "
                    << p->to_string(); LOG_END;
            }
        }
        if (!bempty) {
            tiles.push_back(new_tile);
            LOG_BEGIN(DEBUG) << "get_editable_tile_data() add tile "
                << new_tile->tile_id() << '@' << new_tile->version(); LOG_END;
        }
    }
    return true;
}

bool RoadGeometryManager::get_changed_tile_data(TileInfoList& tiles) {
    std::vector<TileInfoPtr> ts;
    {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        ts.reserve(_tiles.size());
        for (auto& tile : _tiles) {
            ts.push_back(tile.second);
        }
    }
    for (auto& tile : ts) {
        TileInfoPtr new_tile(FeatureProxyBase::create_proxy<TileInfoProxy>());
        new_tile->set_version(tile->version());
        new_tile->set_tile_id(tile->tile_id());
        bool bempty = true; 
        for (auto& p : tile->links()) {
            if (!p) {
                continue;
            }
            if (p->is_changed()) {
                p->correct_version(this);
                new_tile->links().push_back(p);
                bempty = false;
                //LinkProxy::message_type msg;
                //p->to_message(&msg);
                // CLOG_DEBUG("get_changed_tile_data() found elem %s",p->id()->to_string());
                // LOG_BEGIN(DEBUG) << "get_changed_tile_data() found elem "
                //     << p->id()->to_string()/* << " with " << msg.ByteSize() << " Bytes"*/; LOG_END;
            }
        }            
        for (auto& p : tile->link_refs()) {
            if (!p || !p->owner()) {
                continue;
            }
            if (p->owner()->is_changed() || (p->is_changed() && p->owner()->is_editable())) {
                new_tile->link_refs().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found ref %s",p->to_string());
                // LOG_BEGIN(DEBUG) << "get_changed_tile_data() found ref "
                //     << p->to_string(); LOG_END;
            }
        }            
        for (auto& p : tile->nodes()) {
            if (!p) {
                continue;
            }
            if (p->is_changed()) {
                p->correct_version(this);
                new_tile->nodes().push_back(p);
                bempty = false;
                //NodeProxy::message_type msg;
                //p->to_message(&msg);
                // CLOG_DEBUG("get_changed_tile_data() found elem %s",p->id()->to_string());
                // LOG_BEGIN(DEBUG) << "get_changed_tile_data() found elem "
                //     << p->id()->to_string()/* << " with " << msg.ByteSize() << " Bytes"*/; LOG_END;
            }
        }
        for (auto& p : tile->node_refs()) {
            if (!p || !p->owner()) {
                continue;
            }
            if (p->owner()->is_changed() || (p->is_changed() && p->owner()->is_editable())) {
                new_tile->node_refs().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found ref %s",p->to_string());
                // LOG_BEGIN(DEBUG) << "get_changed_tile_data() found ref "
                //     << p->to_string(); LOG_END;
            }
        }
        for (auto& p : tile->lanes()) {
            if (!p) {
                continue;
            }
            if (p->is_changed()) {
                p->correct_version(this);
                new_tile->lanes().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found elem %s",p->id()->to_string());
            }
        }
        for (auto& p : tile->lane_refs()) {
            if (!p || !p->owner()) {
                continue;
            }
            if (p->owner()->is_changed() || (p->is_changed() && p->owner()->is_editable())) {
                new_tile->lane_refs().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found ref %s",p->to_string());
            }
        }
        for (auto& p : tile->lane_boundarys()) {
            if (!p) {
                continue;
            }
            if (p->is_changed()) {
                p->correct_version(this);
                new_tile->lane_boundarys().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found elem %s",p->id()->to_string());
            }
        }
        for (auto& p : tile->lane_boundary_refs()) {
            if (!p || !p->owner()) {
                continue;
            }
            if (p->owner()->is_changed() || (p->is_changed() && p->owner()->is_editable())) {
                new_tile->lane_boundary_refs().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found ref %s",p->to_string());
            }
        }
        for (auto& p : tile->lane_groups()) {
            if (!p) {
                continue;
            }
            if (p->is_changed()) {
                p->correct_version(this);
                new_tile->lane_groups().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found elem %s",p->id()->to_string());
            }
        }
        for (auto& p : tile->lane_group_refs()) {
            if (!p || !p->owner()) {
                continue;
            }
            if (p->owner()->is_changed() || (p->is_changed() && p->owner()->is_editable())) {
                new_tile->lane_group_refs().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found ref %s",p->to_string());
            }
        }
        for (auto& p : tile->junctions()) {
            if (!p) {
                continue;
            }
            if (p->is_changed()) {
                p->correct_version(this);
                new_tile->junctions().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found elem %s",p->id()->to_string());
            }
        }
        for (auto& p : tile->junction_refs()) {
            if (!p || !p->owner()) {
                continue;
            }
            if (p->owner()->is_changed() || (p->is_changed() && p->owner()->is_editable())) {
                new_tile->junction_refs().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found ref %s",p->to_string());
            }
        }
        for (auto& p : tile->traffic_infos()) {
            if (!p) {
                continue;
            }
            if (p->is_changed()) {
                p->correct_version(this);
                new_tile->traffic_infos().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found elem %s",p->id()->to_string());
            }
        }
        for (auto& p : tile->traffic_info_refs()) {
            if (!p || !p->owner()) {
                continue;
            }
            if (p->owner()->is_changed() || (p->is_changed() && p->owner()->is_editable())) {
                new_tile->traffic_info_refs().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found ref %s",p->to_string());
            }
        }
        for (auto& p : tile->position_objects()) {
            if (!p) {
                continue;
            }
            if (p->is_changed()) {
                p->correct_version(this);
                new_tile->position_objects().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found elem %s",p->id()->to_string());
            }
        }
        for (auto& p : tile->position_object_refs()) {
            if (!p || !p->owner()) {
                continue;
            }
            if (p->owner()->is_changed() || (p->is_changed() && p->owner()->is_editable())) {
                new_tile->position_object_refs().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found ref %s",p->to_string());
            }
        }
        for (auto& p : tile->road_boundarys()) {
            if (!p) {
                continue;
            }
            if (p->is_changed()) {
                p->correct_version(this);
                new_tile->road_boundarys().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found elem %s",p->id()->to_string());
            }
        }
        for (auto& p : tile->road_boundary_refs()) {
            if (!p || !p->owner()) {
                continue;
            }
            if (p->owner()->is_changed() || (p->is_changed() && p->owner()->is_editable())) {
                new_tile->road_boundary_refs().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found ref %s",p->to_string());
            }
        }
        for (auto& p : tile->data_qualitys()) {
            if (!p) {
                continue;
            }
            if (p->is_changed()) {
                p->correct_version(this);
                new_tile->data_qualitys().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found elem %s",p->id()->to_string());
            }
        }
        for (auto& p : tile->data_quality_refs()) {
            if (!p || !p->owner()) {
                continue;
            }
            if (p->owner()->is_changed() || (p->is_changed() && p->owner()->is_editable())) {
                new_tile->data_quality_refs().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found ref %s",p->to_string());
            }
        }
        for (auto& p : tile->dynamics()) {
            if (!p) {
                continue;
            }
            if (p->is_changed()) {
                p->correct_version(this);
                new_tile->dynamics().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found elem %s",p->id()->to_string());
            }
        }
        for (auto& p : tile->dynamic_refs()) {
            if (!p || !p->owner()) {
                continue;
            }
            if (p->owner()->is_changed() || (p->is_changed() && p->owner()->is_editable())) {
                new_tile->dynamic_refs().push_back(p);
                bempty = false;
                // CLOG_DEBUG("get_changed_tile_data() found ref %s",p->to_string());
            }
        }
        if (!bempty) {
            tiles.push_back(new_tile);
            // LOG_DEBUG("get_changed_tile_data() add tile %d  version %d",new_tile->tile_id(), new_tile->version());
            // LOG_BEGIN(DEBUG) << "get_changed_tile_data() add tile "
            //     << new_tile->tile_id() << '@' << new_tile->version(); LOG_END;
        }
    }
    return true;
}

bool RoadGeometryManager::make_new_version()
{
    std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
    uint64_t tick = get_tick_count();
    if (tick - _prev_tick > 0) {
        _current_ver += tick - _prev_tick;
        _prev_tick = tick;
        // LOG_DEBUG("make_new_version() finish, current_ver: %d  prev_tick:  %d", _current_ver, _prev_tick);
        // LOG_BEGIN(INFO) << "make_new_version() finish, current_ver: " << _current_ver
        //     << " prev_tick: " << _prev_tick; LOG_END;
    }
    else {
        _current_ver++;
        // LOG_DEBUG("make_new_version() duplicated, current_ver: %d  prev_tick:  %d", _current_ver, _prev_tick);
        // LOG_BEGIN(INFO) << "make_new_version() duplicated, current_ver: " << _current_ver
        //     << " prev_tick: " << _prev_tick; LOG_END;
    }    
    return true;
}

void RoadGeometryManager::clear_all() {
    std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
    _tiles.clear();
    _current_download_tiles.clear();
    _tile15_ids.clear();
    _tile_inited = false;
    LOG_BEGIN(INFO) << "clear_all() finished"; LOG_END;
}

void RoadGeometryManager::clear_tiles() {
    std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
    _tiles.clear();
    _tile15_ids.clear();
    LOG_BEGIN(INFO) << "clear_tiles() finished"; LOG_END;
}

void RoadGeometryManager::clear_current_download_tiles() {
    std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
    _current_download_tiles.clear();
    LOG_BEGIN(INFO) << "clear_current_download_tiles() finished"; LOG_END;
}

bool RoadGeometryManager::load_road_data_from_pbfile(const std::string& strPbFile)
{
    TileInfoList tiles;
    std::string editor("editor");
    if (!_road_tile_dao->load_tiles(strPbFile.c_str(), editor, tiles))
        return false;
    clear_tiles();
    int64_t ver = INT_MAX;
    std::vector<int> tile_id_list;
    for (auto tile : tiles) {
        tile_id_list.push_back(tile->tile_id());
    }
    init_tiles_by_id(tile_id_list);
    for (auto tile : tiles) {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        _tiles[tile->tile_id()] = tile;
        if (ver > tile->version() && tile->version() > 0)
            ver = tile->version();
        tile->load_maps();
    }
    set_current_version(ver);
    remake_tile_proxy(tiles);
    return true;
}

bool RoadGeometryManager::save_road_data_to_pbfile(const std::string& strPbFile)
{
    data_access_engine::TileInfoList ts;
    ts.reserve(_tiles.size());

    for (auto& tit : _tiles) {
        ts.push_back(tit.second);
    }

    if (!_road_tile_dao->save_tiles(ts, "", _current_ver, strPbFile.c_str())) {
        return false;
    }
    return true;
}
};


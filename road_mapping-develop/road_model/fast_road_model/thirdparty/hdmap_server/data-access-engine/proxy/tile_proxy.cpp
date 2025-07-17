#include "tile_proxy.h"
#include <thread>
#include "common_proxy.h"
#include "confidence_proxy.h"
#include "dynamics_proxy.h"
#include "lane_proxy.h"
#include "link_proxy.h"
#include "position_proxy.h"
#include "traffic_proxy.h"
#include "feature_id_proxy.h"
#include "manager/road_geometry_mgr.h"

namespace data_access_engine {

TileInfoProxy::TileInfoProxy() : _tile_id(0), _version(0),
                                    _links(nullptr), _link_refs(nullptr),
                                    _nodes(nullptr), _node_refs(nullptr),
                                    _lanes(nullptr), _lane_refs(nullptr),
                                    _lane_boundarys(nullptr), _lane_boundary_refs(nullptr),
                                    _lane_groups(nullptr), _lane_group_refs(nullptr),
                                    _junctions(nullptr), _junction_refs(nullptr),
                                    _traffic_infos(nullptr), _traffic_info_refs(nullptr),
                                    _position_objects(nullptr), _position_object_refs(nullptr),
                                    _road_boundarys(nullptr), _road_boundary_refs(nullptr),
                                    _data_qualitys(nullptr), _data_quality_refs(nullptr),
                                    _dynamics(nullptr), _dynamic_refs(nullptr) {
    make_ref_record();
    mutable_id()->set_type(-1);
};
TileInfoProxy::~TileInfoProxy() {
    _links.clear();
    _link_refs.clear();
    _nodes.clear();
    _node_refs.clear();
    _lanes.clear();
    _lane_refs.clear();
    _lane_boundarys.clear();
    _lane_boundary_refs.clear();
    _lane_groups.clear();
    _lane_group_refs.clear();
    _junctions.clear();
    _junction_refs.clear();
    _traffic_infos.clear();
    _traffic_info_refs.clear();
    _position_objects.clear();
    _position_object_refs.clear();
    _road_boundarys.clear();
    _road_boundary_refs.clear();
    _data_qualitys.clear();
    _data_quality_refs.clear();
    _dynamics.clear();
    _dynamic_refs.clear();
};

void TileInfoProxy::relink_parent() {
    SharedProxyVector<LinkProxy> links(this);
    _links.swap(links);
    FeatureReferenceVector<LinkProxy> link_refs(this);
    _link_refs.swap(link_refs);
    SharedProxyVector<NodeProxy> nodes(this);
    _nodes.swap(nodes);
    FeatureReferenceVector<NodeProxy> node_refs(this);
    _node_refs.swap(node_refs);
    SharedProxyVector<LaneProxy> lanes(this);
    _lanes.swap(lanes);
    FeatureReferenceVector<LaneProxy> lane_refs(this);
    _lane_refs.swap(lane_refs);
    SharedProxyVector<LaneBoundaryProxy> lane_boundarys(this);
    _lane_boundarys.swap(lane_boundarys);
    FeatureReferenceVector<LaneBoundaryProxy> lane_boundary_refs(this);
    _lane_boundary_refs.swap(lane_boundary_refs);
    SharedProxyVector<LaneGroupProxy> lane_groups(this);
    _lane_groups.swap(lane_groups);
    FeatureReferenceVector<LaneGroupProxy> lane_group_refs(this);
    _lane_group_refs.swap(lane_group_refs);
    SharedProxyVector<JunctionProxy> junctions(this);
    _junctions.swap(junctions);
    FeatureReferenceVector<JunctionProxy> junction_refs(this);
    _junction_refs.swap(junction_refs);
    SharedProxyVector<TrafficInfoProxy> traffic_infos(this);
    _traffic_infos.swap(traffic_infos);
    FeatureReferenceVector<TrafficInfoProxy> traffic_info_refs(this);
    _traffic_info_refs.swap(traffic_info_refs);
    SharedProxyVector<PositionObjectProxy> position_objects(this);
    _position_objects.swap(position_objects);
    FeatureReferenceVector<PositionObjectProxy> position_object_refs(this);
    _position_object_refs.swap(position_object_refs);
    SharedProxyVector<RoadBoundaryProxy> road_boundarys(this);
    _road_boundarys.swap(road_boundarys);
    FeatureReferenceVector<RoadBoundaryProxy> road_boundary_refs(this);
    _road_boundary_refs.swap(road_boundary_refs);
    SharedProxyVector<DataQualityProxy> data_qualitys(this);
    _data_qualitys.swap(data_qualitys);
    FeatureReferenceVector<DataQualityProxy> data_quality_refs(this);
    _data_quality_refs.swap(data_quality_refs);
    SharedProxyVector<DynamicProxy> dynamics(this);
    _dynamics.swap(dynamics);
    FeatureReferenceVector<DynamicProxy> dynamic_refs(this);
    _dynamic_refs.swap(dynamic_refs);
};

void TileInfoProxy::clear_parent() {
    SharedProxyVector<LinkProxy> links(nullptr);
    _links.swap(links);
    FeatureReferenceVector<LinkProxy> link_refs(nullptr);
    _link_refs.swap(link_refs);
    SharedProxyVector<NodeProxy> nodes(nullptr);
    _nodes.swap(nodes);
    FeatureReferenceVector<NodeProxy> node_refs(nullptr);
    _node_refs.swap(node_refs);
    SharedProxyVector<LaneProxy> lanes(nullptr);
    _lanes.swap(lanes);
    FeatureReferenceVector<LaneProxy> lane_refs(nullptr);
    _lane_refs.swap(lane_refs);
    SharedProxyVector<LaneBoundaryProxy> lane_boundarys(nullptr);
    _lane_boundarys.swap(lane_boundarys);
    FeatureReferenceVector<LaneBoundaryProxy> lane_boundary_refs(nullptr);
    _lane_boundary_refs.swap(lane_boundary_refs);
    SharedProxyVector<LaneGroupProxy> lane_groups(nullptr);
    _lane_groups.swap(lane_groups);
    FeatureReferenceVector<LaneGroupProxy> lane_group_refs(nullptr);
    _lane_group_refs.swap(lane_group_refs);
    SharedProxyVector<JunctionProxy> junctions(nullptr);
    _junctions.swap(junctions);
    FeatureReferenceVector<JunctionProxy> junction_refs(nullptr);
    _junction_refs.swap(junction_refs);
    SharedProxyVector<TrafficInfoProxy> traffic_infos(nullptr);
    _traffic_infos.swap(traffic_infos);
    FeatureReferenceVector<TrafficInfoProxy> traffic_info_refs(nullptr);
    _traffic_info_refs.swap(traffic_info_refs);
    SharedProxyVector<PositionObjectProxy> position_objects(nullptr);
    _position_objects.swap(position_objects);
    FeatureReferenceVector<PositionObjectProxy> position_object_refs(nullptr);
    _position_object_refs.swap(position_object_refs);
    SharedProxyVector<RoadBoundaryProxy> road_boundarys(nullptr);
    _road_boundarys.swap(road_boundarys);
    FeatureReferenceVector<RoadBoundaryProxy> road_boundary_refs(nullptr);
    _road_boundary_refs.swap(road_boundary_refs);
    SharedProxyVector<DataQualityProxy> data_qualitys(nullptr);
    _data_qualitys.swap(data_qualitys);
    FeatureReferenceVector<DataQualityProxy> data_quality_refs(nullptr);
    _data_quality_refs.swap(data_quality_refs);
    SharedProxyVector<DynamicProxy> dynamics(nullptr);
    _dynamics.swap(dynamics);
    FeatureReferenceVector<DynamicProxy> dynamic_refs(nullptr);
    _dynamic_refs.swap(dynamic_refs);
}

template <class T>
void insert_elem_to_map(const std::shared_ptr<const T>& p, const std::unordered_map<std::string,
        std::weak_ptr<const T>>& id2es) {
    auto& id2elems = const_cast<std::unordered_map<std::string, std::weak_ptr<const T>>&>(id2es);
    if (p && p->id()) {
        auto itr = id2elems.find(p->id()->to_binary_string());
        if (itr != id2elems.end()) {
            if (itr->second.lock() && itr->second.lock()->id()->less_version(p->id())){
                id2elems[p->id()->to_binary_string()] = p;
            }
        }
        else {
            id2elems[p->id()->to_binary_string()] = p;
        }
    }
};
template <class T>
void generate_elem_map(const SharedProxyVector<T>& elems, std::unordered_map<std::string, 
        std::weak_ptr<const T>>& id2elems) {
    id2elems.clear();
    id2elems.reserve(elems.size() * 2);
    for (size_t i = 0; i < elems.size(); ++i) {
        auto p = elems[i];
        insert_elem_to_map(p, id2elems);
    }
};
    
void TileInfoProxy::load_maps() {
    generate_elem_map(_links, _link_map);
    generate_elem_map(_nodes, _node_map);
    generate_elem_map(_lanes, _lane_map);
    generate_elem_map(_lane_boundarys, _lane_boundary_map);
    generate_elem_map(_lane_groups, _lane_group_map);
    generate_elem_map(_junctions, _junction_map);
    generate_elem_map(_traffic_infos, _traffic_info_map);
    generate_elem_map(_position_objects, _position_object_map);
    generate_elem_map(_road_boundarys, _road_boundary_map);
    generate_elem_map(_data_qualitys, _data_quality_map);
    generate_elem_map(_dynamics, _dynamic_map);
}

template<class Proxy>
void to_featurelist(const SharedProxyVector<Proxy>& fs, const FeatureReferenceVector<Proxy>& rs, 
                    services::TileInfo* msg) {
    auto fl = msg->add_feat_list();
    fl->set_type(Proxy::ELEM_ID_TYPE);
    for (size_t i = 0; i < fs.size(); ++i) {
        auto& ft = fs[i];
        auto f = fl->add_feats();
        ft->id()->to_message(f->mutable_id());
        typename Proxy::message_type m;
        ft->to_message(&m);
        f->set_data(m.SerializeAsString());
    }
    rs.to_message(fl->mutable_refs());
}

bool TileInfoProxy::to_message(google::protobuf::Message* msg_base) const {
    auto msg = static_cast<services::TileInfo*>(msg_base);
    msg->Clear();
    msg->set_tile_id(_tile_id);
    msg->set_version(_version);
    if (!_links.empty() || !_link_refs.empty()) {
        to_featurelist(_links, _link_refs, msg);
    }
    if (!_nodes.empty() || !_node_refs.empty()) {
        to_featurelist(_nodes, _node_refs, msg);
    }
    if (!_lanes.empty() || !_lane_refs.empty()) {
        to_featurelist(_lanes, _lane_refs, msg);
    }
    if (!_lane_boundarys.empty() || !_lane_boundary_refs.empty()) {
        to_featurelist(_lane_boundarys, _lane_boundary_refs, msg);
    }
    if (!_lane_groups.empty() || !_lane_group_refs.empty()) {
        to_featurelist(_lane_groups, _lane_group_refs, msg);
    }
    if (!_junctions.empty() || !_junction_refs.empty()) {
        to_featurelist(_junctions, _junction_refs, msg);
    }
    if (!_traffic_infos.empty() || !_traffic_info_refs.empty()) {
        to_featurelist(_traffic_infos, _traffic_info_refs, msg);
    }
    if (!_position_objects.empty() || !_position_object_refs.empty()) {
        to_featurelist(_position_objects, _position_object_refs, msg);
    }
    if (!_road_boundarys.empty() || !_road_boundary_refs.empty()) {
        to_featurelist(_road_boundarys, _road_boundary_refs, msg);
    }
    if (!_data_qualitys.empty() || !_data_quality_refs.empty()) {
        to_featurelist(_data_qualitys, _data_quality_refs, msg);
    }
    if (!_dynamics.empty() || !_dynamic_refs.empty()) {
        to_featurelist(_dynamics, _dynamic_refs, msg);
    }
    if (_last_editor) {
        msg->set_last_editor(*_last_editor);
    }
    return true;
};

template<class Proxy>
void from_featurelist(const services::TileFeatureList& fl, SharedProxyVector<Proxy>& fs,
                        FeatureReferenceVector<Proxy>& rs) {
    if (fl.type() != Proxy::ELEM_ID_TYPE) {
        return;
    }
    fs.clear();
    fs.reserve(fl.feats_size());
    for (int i = 0; i < fl.feats_size(); ++i) {
        auto& ft = fl.feats(i);
        if ((ft.id().has_is_deleted() && ft.id().is_deleted()) || ft.id().version() <= 1) {
            continue;
        }
        typename Proxy::message_type m;
        if (ft.data().empty() || !m.ParseFromString(ft.data())) {
            LOG_BEGIN(ERROR) << "Fail to parse feature " << ft.id().ShortDebugString() << " : " << ft.data().size() << " Bytes"; LOG_END;
            continue;
        }
        if (ft.id().tileid() != m.id().tileid() || ft.id().id() != m.id().id() || ft.id().type() != m.id().type()) {
            LOG_BEGIN(ERROR) << "Inconsist feature " << fs.back()->id()->to_string() << " with " << ft.id().ShortDebugString() << " : " << ft.data().size() << " Bytes"; LOG_END;
            continue;
        }
        fs.add()->from_message(&m);
        /*if (!fs.back()->is_valid()) {
            LOG_BEGIN(ERROR) << "Invalid feature " << fs.back()->id()->to_string() << " : " << ft.data().size() << " Bytes"; LOG_END;
            continue;
        }*/
    }
    rs.from_message(fl.refs());
}
bool TileInfoProxy::from_message(const google::protobuf::Message* msg_base) {
    auto msg = static_cast<const services::TileInfo*>(msg_base);
    _tile_id = msg->tile_id();
    _version = msg->version();
    for (int i = 0; i < msg->feat_list_size(); ++i) {
        auto& fl = msg->feat_list(i);
        if (fl.type() == LinkProxy::ELEM_ID_TYPE) {
            from_featurelist(fl, _links, _link_refs);
        }
        else if (fl.type() == NodeProxy::ELEM_ID_TYPE) {
            from_featurelist(fl, _nodes, _node_refs);
        }
        else if (fl.type() == LaneProxy::ELEM_ID_TYPE) {
            from_featurelist(fl, _lanes, _lane_refs);
        }
        else if (fl.type() == LaneBoundaryProxy::ELEM_ID_TYPE) {
            from_featurelist(fl, _lane_boundarys, _lane_boundary_refs);
        }
        else if (fl.type() == LaneGroupProxy::ELEM_ID_TYPE) {
            from_featurelist(fl, _lane_groups, _lane_group_refs);
        }
        else if (fl.type() == JunctionProxy::ELEM_ID_TYPE) {
            from_featurelist(fl, _junctions, _junction_refs);
        }
        else if (fl.type() == TrafficInfoProxy::ELEM_ID_TYPE) {
            from_featurelist(fl, _traffic_infos, _traffic_info_refs);
        }
        else if (fl.type() == PositionObjectProxy::ELEM_ID_TYPE) {
            from_featurelist(fl, _position_objects, _position_object_refs);
        }
        else if (fl.type() == RoadBoundaryProxy::ELEM_ID_TYPE) {
            from_featurelist(fl, _road_boundarys, _road_boundary_refs);
        }
        else if (fl.type() == DataQualityProxy::ELEM_ID_TYPE) {
            from_featurelist(fl, _data_qualitys, _data_quality_refs);
        }
        else if (fl.type() == DynamicProxy::ELEM_ID_TYPE) {
            from_featurelist(fl, _dynamics, _dynamic_refs);
        }
    }
    _last_editor.reset();
    if (msg->has_last_editor()) {
        _last_editor = msg->last_editor();
    }
    //clear_changed();
    return true;
};

template <class Proxy>
bool has_duplicated_elements(const SharedProxyVector<Proxy>& proxys, int tile_id) {
    std::unordered_set<uint64_t> ids(proxys.size() * 2);
    for (size_t i = 0; i < proxys.size(); ++i) {
        auto p = proxys[i];
        if (!p || !p->id()) {
            LOG_BEGIN(ERROR) << "has_duplicated_elements() found empty proxy "
                << i << "/" << proxys.size() << " in tile " << tile_id; LOG_END;
            continue;
        }
        if (p->id()->tileid() != tile_id) {
            LOG_BEGIN(ERROR) << "has_duplicated_elements() found inconsistent proxy "
                << p->id()->to_string() << " @ " << i << "/" << proxys.size()
                << " in tile " << tile_id; LOG_END;
            return true;
        }
        if (ids.count(p->id()->id()) > 0) {
            LOG_BEGIN(ERROR) << "has_duplicated_elements() found duplicated proxy "
                << p->id()->to_string() << " @ " << i << "/" << proxys.size()
                << " in tile " << tile_id; LOG_END;
            return true;
        }
        ids.insert(p->id()->id());
    }
    return false;
};

bool TileInfoProxy::is_valid() const {
    if (_tile_id <= 0 || _version < 0) {
        return false;
    }
    if (!_links.is_valid() || has_duplicated_elements(_links, _tile_id) 
            || !_link_refs.is_valid(false) || !_nodes.is_valid()
            || has_duplicated_elements(_nodes, _tile_id) || !_node_refs.is_valid(false)
            || !_lanes.is_valid() || has_duplicated_elements(_lanes, _tile_id)
            || !_lane_refs.is_valid(false) || !_lane_boundarys.is_valid()
            || has_duplicated_elements(_lane_boundarys, _tile_id)
            || !_lane_boundary_refs.is_valid(false) || !_lane_groups.is_valid()
            || has_duplicated_elements(_lane_groups, _tile_id) 
            || !_lane_group_refs.is_valid(false) || !_junctions.is_valid()
            || has_duplicated_elements(_junctions, _tile_id)
            || !_junction_refs.is_valid(false) || !_traffic_infos.is_valid()
            || has_duplicated_elements(_traffic_infos, _tile_id)
            || !_traffic_info_refs.is_valid(false) || !_position_objects.is_valid()
            || has_duplicated_elements(_position_objects, _tile_id)
            || !_position_object_refs.is_valid(false) || !_road_boundarys.is_valid()
            || has_duplicated_elements(_road_boundarys, _tile_id)
            || !_road_boundary_refs.is_valid(false) || !_data_qualitys.is_valid()
            || has_duplicated_elements(_data_qualitys, _tile_id)
            || !_data_quality_refs.is_valid(false) || !_dynamics.is_valid()
            || has_duplicated_elements(_dynamics, _tile_id) 
            || !_dynamic_refs.is_valid(false)) {
        return false;
    }
    return true;
};

bool TileInfoProxy::is_empty() const {
    return (_links.empty() && _link_refs.empty() && _nodes.empty()
            && _node_refs.empty() && _lanes.empty() && _lane_refs.empty()
            && _lane_boundarys.empty() && _lane_boundary_refs.empty()
            && _lane_groups.empty() && _lane_group_refs.empty()
            && _junctions.empty() && _junction_refs.empty()
            && _traffic_infos.empty() && _traffic_info_refs.empty()
            && _position_objects.empty() && _position_object_refs.empty()
            && _road_boundarys.empty() && _road_boundary_refs.empty()
            && _data_qualitys.empty() && _data_quality_refs.empty()
            && _dynamics.empty() && _dynamic_refs.empty());
};

template <class Proxy>
bool remove_duplicated_elements(SharedProxyVector<Proxy>& proxys, int tile_id) {
    bool bf = false;
    std::unordered_set<uint64_t> ids(proxys.size() * 2);
    for (size_t i = 0; i < proxys.size(); ++i) {
        auto& p = proxys[i];
        if (!p || !p->id()) {
            bf = true;
            LOG_BEGIN(ERROR) << "Remove empty element " << i << '/' << proxys.size()
                << " in SharedProxyVector at tile " << tile_id; LOG_END; 
            proxys.erase(i);
            --i;            
            continue;
        }
        if (p->id()->tileid() != tile_id) {
            bf = true;
            LOG_BEGIN(ERROR) << "Remove error element " << i << '/' << proxys.size()
                << " in SharedProxyVector at tile " << tile_id; LOG_END;
            proxys.erase(i);
            --i;
            continue;
        }
        if (ids.count(p->id()->id()) > 0) {
            bf = true;
            LOG_BEGIN(ERROR) << "Remove duplicated element " << p->id()->to_string()
                << " in SharedProxyVector at tile " << tile_id; LOG_END;
            proxys.erase(i);
            --i;
            continue;
        }
        ids.insert(p->id()->id());
    }
    return bf;
};

template <class Proxy>
bool remove_invalid_ids(FeatureReferenceVector<Proxy>& proxys) {
    bool bf = false;
    for (size_t i = 0; i < proxys.size(); ++i) {
        auto& p = proxys[i];
        if (!p || !p->cached_is_valid()) {
            std::string sid("!no_id!");
            if (p) {
                sid = p->to_string();
            }
            LOG_BEGIN(INFO) << "Remove invalid id " << sid
                << " at " << i << '/' << proxys.size(); LOG_END;
            proxys.erase(i);
            --i;
            bf = true;
            continue;
        }
    }
    for (size_t i = 0; i < proxys.size(); ++i) {
        auto& p = proxys[i];
        for (size_t j = i + 1; j < proxys.size(); ++j) {
            if (p->is_equal(*proxys[j].id())) {
                LOG_BEGIN(INFO) << "Remove duplicated id " << proxys[j]->to_string()
                    << " at " << j << " with " << proxys[i]->to_string() << " at " << i
                    << " / " << proxys.size(); LOG_END;
                proxys.erase(j);
                --j;
                bf = true;
            }
        }
    }
    return bf;
};

bool TileInfoProxy::correct_content(RoadGeometryManager* pmgr) {
    bool bc = false;
    if (pmgr) {
        bc |= _links.correct_content(pmgr);
    }
    bc |= remove_duplicated_elements(_links, _tile_id);
    bc |= remove_invalid_ids(_link_refs);
    if (pmgr) {
        bc |= _nodes.correct_content(pmgr);
    }
    bc |= remove_duplicated_elements(_nodes, _tile_id);
    bc |= remove_invalid_ids(_node_refs);
    if (pmgr) {
        bc |= _lane_boundarys.correct_content(pmgr);
    }
    bc |= remove_duplicated_elements(_lane_boundarys, _tile_id);
    bc |= remove_invalid_ids(_lane_boundary_refs);
    if (pmgr) {
        bc |= _lanes.correct_content(pmgr);
    }
    bc |= remove_duplicated_elements(_lanes, _tile_id);
    bc |= remove_invalid_ids(_lane_refs);
    if (pmgr) {
        bc |= _lane_groups.correct_content(pmgr);
    }
    bc |= remove_duplicated_elements(_lane_groups, _tile_id);
    bc |= remove_invalid_ids(_lane_group_refs);
    if (pmgr) {
        bc |= _junctions.correct_content(pmgr);
    }
    bc |= remove_duplicated_elements(_junctions, _tile_id);
    bc |= remove_invalid_ids(_junction_refs);
    if (pmgr) {
        bc |= _traffic_infos.correct_content(pmgr);
    }
    bc |= remove_duplicated_elements(_traffic_infos, _tile_id);
    bc |= remove_invalid_ids(_traffic_info_refs);
    if (pmgr) {
        bc |= _position_objects.correct_content(pmgr);
    }
    bc |= remove_duplicated_elements(_position_objects, _tile_id);
    bc |= remove_invalid_ids(_position_object_refs);
    if (pmgr) {
        bc |= _road_boundarys.correct_content(pmgr);
    }
    bc |= remove_duplicated_elements(_road_boundarys, _tile_id);
    bc |= remove_invalid_ids(_road_boundary_refs);
    if (pmgr) {
        bc |= _data_qualitys.correct_content(pmgr);
    }
    bc |= remove_duplicated_elements(_data_qualitys, _tile_id);
    bc |= remove_invalid_ids(_data_quality_refs);
    if (pmgr) {
        bc |= _dynamics.correct_content(pmgr);
    }
    bc |= remove_duplicated_elements(_dynamics, _tile_id);
    bc |= remove_invalid_ids(_dynamic_refs);
    return bc;
};

bool TileInfoProxy::correct_tile_refs(RoadGeometryManager* pmgr) {
    bool bc = false;
    bc |= _links.correct_tile_refs(pmgr);
    bc |= _nodes.correct_tile_refs(pmgr);
    bc |= _lanes.correct_tile_refs(pmgr);
    bc |= _lane_boundarys.correct_tile_refs(pmgr);
    bc |= _lane_groups.correct_tile_refs(pmgr);
    bc |= _junctions.correct_tile_refs(pmgr);
    bc |= _traffic_infos.correct_tile_refs(pmgr);
    bc |= _position_objects.correct_tile_refs(pmgr);
    bc |= _road_boundarys.correct_tile_refs(pmgr);
    bc |= _data_qualitys.correct_tile_refs(pmgr);
    bc |= _dynamics.correct_tile_refs(pmgr);
    return bc;
};

bool TileInfoProxy::remake_proxy(RoadGeometryManager* mgr) {
    bool bf = true;
    bf &= _links.remake_proxy(mgr);
    bf &= _link_refs.resolve_proxy(mgr);
    bf &= _nodes.remake_proxy(mgr);
    bf &= _node_refs.resolve_proxy(mgr);
    bf &= _lanes.remake_proxy(mgr);
    bf &= _lane_refs.resolve_proxy(mgr);
    bf &= _lane_boundarys.remake_proxy(mgr);
    bf &= _lane_boundary_refs.resolve_proxy(mgr);
    bf &= _lane_groups.remake_proxy(mgr);
    bf &= _lane_group_refs.resolve_proxy(mgr);
    bf &= _junctions.remake_proxy(mgr);
    bf &= _junction_refs.resolve_proxy(mgr);
    bf &= _traffic_infos.remake_proxy(mgr);
    bf &= _traffic_info_refs.resolve_proxy(mgr);
    bf &= _position_objects.remake_proxy(mgr);
    bf &= _position_object_refs.resolve_proxy(mgr);
    bf &= _road_boundarys.remake_proxy(mgr);
    bf &= _road_boundary_refs.resolve_proxy(mgr);
    bf &= _data_qualitys.remake_proxy(mgr);
    bf &= _data_quality_refs.resolve_proxy(mgr);
    bf &= _dynamics.remake_proxy(mgr);
    bf &= _dynamic_refs.resolve_proxy(mgr);
    return bf;
};

template <class VectorType>
void vector_shallow_copy(VectorType& dst, VectorType& src) {
    dst.clear();
    dst.reserve(src.size());
    for (size_t i = 0; i < src.size(); ++i) {
        auto s = src[i];
        dst.push_back(s);
    }
};

void TileInfoProxy::shallow_copy(TileInfoProxy* tile) {
    _tile_id = tile->_tile_id;
    _version = tile->_version;
    _last_editor = tile->_last_editor;
    vector_shallow_copy(_links, tile->_links);
    vector_shallow_copy(_link_refs, tile->_link_refs);
    vector_shallow_copy(_nodes, tile->_nodes);
    vector_shallow_copy(_node_refs, tile->_node_refs);
    vector_shallow_copy(_lanes, tile->_lanes);
    vector_shallow_copy(_lane_refs, tile->_lane_refs);
    vector_shallow_copy(_lane_boundarys, tile->_lane_boundarys);
    vector_shallow_copy(_lane_boundary_refs, tile->_lane_boundary_refs);
    vector_shallow_copy(_lane_groups, tile->_lane_groups);
    vector_shallow_copy(_lane_group_refs, tile->_lane_group_refs);
    vector_shallow_copy(_junctions, tile->_junctions);
    vector_shallow_copy(_junction_refs, tile->_junction_refs);
    vector_shallow_copy(_traffic_infos, tile->_traffic_infos);
    vector_shallow_copy(_traffic_info_refs, tile->_traffic_info_refs);
    vector_shallow_copy(_position_objects, tile->_position_objects);
    vector_shallow_copy(_position_object_refs, tile->_position_object_refs);
    vector_shallow_copy(_road_boundarys, tile->_road_boundarys);
    vector_shallow_copy(_road_boundary_refs, tile->_road_boundary_refs);
    vector_shallow_copy(_data_qualitys, tile->_data_qualitys);
    vector_shallow_copy(_data_quality_refs, tile->_data_quality_refs);
    vector_shallow_copy(_dynamics, tile->_dynamics);
    vector_shallow_copy(_dynamic_refs, tile->_dynamic_refs);
};

void TileInfoProxy::clear_changed() {
    _links.clear_changed();
    _nodes.clear_changed();
    _lanes.clear_changed();
    _lane_boundarys.clear_changed();
    _lane_groups.clear_changed();
    _junctions.clear_changed();
    _traffic_infos.clear_changed();
    _position_objects.clear_changed();
    _road_boundarys.clear_changed();
    _data_qualitys.clear_changed();
    _dynamics.clear_changed();
};

bool TileInfoProxy::add_reference_to_tile(FeatureReferenceBase* ref) {
    switch ((*ref)->type()) {
    case LinkProxy::ELEM_ID_TYPE:
        _link_refs.push_back(ref->id());
        break;
    case NodeProxy::ELEM_ID_TYPE:
        _node_refs.push_back(ref->id());
        break;
    case LaneProxy::ELEM_ID_TYPE:
        _lane_refs.push_back(ref->id());
        break;
    case LaneBoundaryProxy::ELEM_ID_TYPE:
        _lane_boundary_refs.push_back(ref->id());
        break;
    case LaneGroupProxy::ELEM_ID_TYPE:
        _lane_group_refs.push_back(ref->id());
        break;
    case JunctionProxy::ELEM_ID_TYPE:
        _junction_refs.push_back(ref->id());
        break;
    case TrafficInfoProxy::ELEM_ID_TYPE:
        _traffic_info_refs.push_back(ref->id());
        break;
    case PositionObjectProxy::ELEM_ID_TYPE:
        _position_object_refs.push_back(ref->id());
        break;
    case RoadBoundaryProxy::ELEM_ID_TYPE:
        _road_boundary_refs.push_back(ref->id());
        break;
    case DataQualityProxy::ELEM_ID_TYPE:
        _data_quality_refs.push_back(ref->id());
        break;
    case DynamicProxy::ELEM_ID_TYPE:
        _dynamic_refs.push_back(ref->id());
        break;
    default:
        return false;
    }
    return true;
};

bool TileInfoProxy::add_element_to_tile(std::shared_ptr<FeatureWithIDProxyBase> elem) {
    switch (elem->feature_id_type()) {
    case LinkProxy::ELEM_ID_TYPE:
        _links.push_back(std::dynamic_pointer_cast<LinkProxy>(elem));
        break;
    case NodeProxy::ELEM_ID_TYPE:
        _nodes.push_back(std::dynamic_pointer_cast<NodeProxy>(elem));
        break;
    case LaneProxy::ELEM_ID_TYPE:
        _lanes.push_back(std::dynamic_pointer_cast<LaneProxy>(elem));
        break;
    case LaneBoundaryProxy::ELEM_ID_TYPE:
        _lane_boundarys.push_back(std::dynamic_pointer_cast<LaneBoundaryProxy>(elem));
        break;
    case LaneGroupProxy::ELEM_ID_TYPE:
        _lane_groups.push_back(std::dynamic_pointer_cast<LaneGroupProxy>(elem));
        break;
    case JunctionProxy::ELEM_ID_TYPE:
        _junctions.push_back(std::dynamic_pointer_cast<JunctionProxy>(elem));
        break;
    case TrafficInfoProxy::ELEM_ID_TYPE:
        _traffic_infos.push_back(std::dynamic_pointer_cast<TrafficInfoProxy>(elem));
        break;
    case PositionObjectProxy::ELEM_ID_TYPE:
        _position_objects.push_back(std::dynamic_pointer_cast<PositionObjectProxy>(elem));
        break;
    case RoadBoundaryProxy::ELEM_ID_TYPE:
        _road_boundarys.push_back(std::dynamic_pointer_cast<RoadBoundaryProxy>(elem));
        break;
    case DataQualityProxy::ELEM_ID_TYPE:
        _data_qualitys.push_back(std::dynamic_pointer_cast<DataQualityProxy>(elem));
        break;
    case DynamicProxy::ELEM_ID_TYPE:
        _dynamics.push_back(std::dynamic_pointer_cast<DynamicProxy>(elem));
        break;
    default:
        return false;
    }
    return true;
};

bool TileInfoProxy::revert() {
    bool b = true;
    for (auto& e : _links) {
        b &= e->revert();
    }
    for (auto& e : _nodes) {
        b &= e->revert();
    }
    for (auto& e : _lanes) {
        b &= e->revert();
    }
    for (auto& e : _lane_boundarys) {
        b &= e->revert();
    }
    for (auto& e : _lane_groups) {
        b &= e->revert();
    }
    for (auto& e : _junctions) {
        b &= e->revert();
    }
    for (auto& e : _traffic_infos) {
        b &= e->revert();
    }
    for (auto& e : _position_objects) {
        b &= e->revert();
    }
    for (auto& e : _road_boundarys) {
        b &= e->revert();
    }
    for (auto& e : _data_qualitys) {
        b &= e->revert();
    }
    for (auto& e : _dynamics) {
        b &= e->revert();
    }
    return b;
};

bool TileInfoProxy::merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) {
    bool b = false;
    for (auto& e : _links) {
        b &= e->merge(policy, mgr);
    }
    for (auto& e : _nodes) {
        b &= e->merge(policy, mgr);
    }
    for (auto& e : _lanes) {
        b &= e->merge(policy, mgr);
    }
    for (auto& e : _lane_boundarys) {
        b &= e->merge(policy, mgr);
    }
    for (auto& e : _lane_groups) {
        b &= e->merge(policy, mgr);
    }
    for (auto& e : _junctions) {
        b &= e->merge(policy, mgr);
    }
    for (auto& e : _traffic_infos) {
        b &= e->merge(policy, mgr);
    }
    for (auto& e : _position_objects) {
        b &= e->merge(policy, mgr);
    }
    for (auto& e : _road_boundarys) {
        b &= e->merge(policy, mgr);
    }
    for (auto& e : _data_qualitys) {
        b &= e->merge(policy, mgr);
    }
    for (auto& e : _dynamics) {
        b &= e->merge(policy, mgr);
    }
    return b;
};

template <class T>
inline void merge_elems(SharedProxyVector<T>& es, SharedProxyVector<T>& res,
                       std::unordered_map<std::string, std::weak_ptr<const T>>& ms,
                       std::unordered_map<std::string, std::weak_ptr<const T>>& rms) {
    if (es.empty()) {
        if (res.empty()) {
            return;
        }
        es.swap(res);
        ms.swap(rms);
        return;
    }
    for (auto r : res) {
        if (!r || !r->id()) {
            continue;
        }
        auto str = r->id()->to_binary_string();
        auto it = ms.find(str);
        if (it != ms.end()) {
            auto e = std::const_pointer_cast<T>(it->second.lock());
            if (e) {
                if (e->origin()->id()->version() != r->id()->version()) {
                    if (r->id()->version() != e->id()->version()) {
                        e->set_remote_proxy(r);
                    }
                    else {
                        e->set_remote_proxy(r);
                    }
                    LOG_BEGIN(DEBUG) << "Set remote elem " << r->id()->to_string() << " with " 
                        << e->id()->version() << " / " << e->origin()->id()->version();
                    LOG_END;
                }
                continue;
            }
        }
        LOG_BEGIN(INFO) << "Append remote elem " << r->id()->to_string();
        LOG_END;
        es.push_back(r);
        ms[str] = r;
    }
    for (auto e : es) {
        if (!e || !e->id() || e->origin() == e
                || e->origin()->id()->version() >= e->id()->version()
                || e->origin()->id()->version() <= 0) {
            continue;
        }
        auto str = e->id()->to_binary_string();
        auto it = rms.find(str);
        if (it == rms.end()) {
            LOG_BEGIN(INFO) << "Set deleted remote elem " << e->id()->to_string();
            LOG_END;
            e->template set_remote_elem<T>(e.get());
            auto r = std::const_pointer_cast<T>(e->remote());
            r->set_deleted(true);
            r->mutable_id()->set_version(e->origin()->id()->version() + 1);
        }
    }
};

void TileInfoProxy::merge_tile(TileInfoProxy* tile) {
    merge_elems(_links, tile->_links,
                _link_map, tile->_link_map);
    merge_elems(_nodes, tile->_nodes,
                _node_map, tile->_node_map);
    merge_elems(_lanes, tile->_lanes,
                _lane_map, tile->_lane_map);
    merge_elems(_lane_boundarys, tile->_lane_boundarys,
                _lane_boundary_map, tile->_lane_boundary_map);
    merge_elems(_lane_groups, tile->_lane_groups,
                _lane_group_map, tile->_lane_group_map);
    merge_elems(_junctions, tile->_junctions,
                _junction_map, tile->_junction_map);
    merge_elems(_traffic_infos, tile->_traffic_infos,
                _traffic_info_map, tile->_traffic_info_map);
    merge_elems(_position_objects, tile->_position_objects,
                _position_object_map, tile->_position_object_map);
    merge_elems(_road_boundarys, tile->_road_boundarys,
                _road_boundary_map, tile->_road_boundary_map);
    merge_elems(_data_qualitys, tile->_data_qualitys,
                _data_quality_map, tile->_data_quality_map);
    merge_elems(_dynamics, tile->_dynamics,
                _dynamic_map, tile->_dynamic_map);
}

void TileInfoProxy::reset_merge() {
    for (auto& e : _links) {
        e->reset_merge();
    }
    for (auto& e : _nodes) {
        e->reset_merge();
    }
    for (auto& e : _lanes) {
        e->reset_merge();
    }
    for (auto& e : _lane_boundarys) {
        e->reset_merge();
    }
    for (auto& e : _lane_groups) {
        e->reset_merge();
    }
    for (auto& e : _junctions) {
        e->reset_merge();
    }
    for (auto& e : _traffic_infos) {
        e->reset_merge();
    }
    for (auto& e : _position_objects) {
        e->reset_merge();
    }
    for (auto& e : _road_boundarys) {
        e->reset_merge();
    }
    for (auto& e : _data_qualitys) {
        e->reset_merge();
    }
    for (auto& e : _dynamics) {
        e->reset_merge();
    }
};

bool TileInfoProxy::append_to_issue(RoadGeometryManager* mgr) {
    bool bc = false;
    return bc;
};

}; // data_access_engine ;

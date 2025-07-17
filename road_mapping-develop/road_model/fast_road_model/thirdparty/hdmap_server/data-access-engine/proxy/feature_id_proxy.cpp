#include "feature_id_proxy.h"
#include "feature_with_id_proxy_base.h"
#include "manager/road_geometry_mgr.h"

namespace data_access_engine {

bool FeatureIDProxy::set_tileid(int tileid) {
    if (!is_editable()) {
        return false;
    }
    if (_tileid != tileid) {
        mark_changed();
    }
    _tileid = tileid;
    return true;
};

bool FeatureIDProxy::set_type(int type) {
    if (!is_editable()) {
        return false;
    }
    if (_type != type) {
        mark_changed();
    }
    _type = type;
    return true;
};

bool FeatureIDProxy::set_id(uint64_t id) {
    if (!is_editable()) {
        return false;
    }
    if (_id != id) {
        mark_changed();
    }
    _id = id;
    return true;
};

bool FeatureIDProxy::set_version(int64_t ver) {
    if (!is_editable()) {
        return false;
    }
    if (_version != ver) {
        mark_changed();
    }
    _version = ver;
    return true;
};

bool FeatureIDProxy::set_is_deleted(Optional<bool> is_deleted) {
    if (!is_editable()) {
        return false;
    }
    if (_is_deleted != is_deleted) {
        mark_changed();
    }
    _is_deleted = is_deleted;
    return true;
};

void FeatureIDProxy::set_owner(FeatureWithIDProxyBase* owner) {
    _owner = owner;
};

bool FeatureIDProxy::to_message(google::protobuf::Message* msg_base) const {
    /*if (!is_valid()) {
        return false;
    }*/
    auto msg = static_cast<RoadPB::FeatureID*>(msg_base);
    msg->set_tileid(_tileid);
    msg->set_type(_type);
    msg->set_id(_id);
    msg->set_version(_version);
    if (_is_deleted) {
        msg->set_is_deleted(*_is_deleted);
    }
    else {
        msg->clear_is_deleted();
    }
    return true;
};

bool FeatureIDProxy::from_message(const google::protobuf::Message* msg_base) {
    auto msg = static_cast<const RoadPB::FeatureID*>(msg_base);
    _tileid = msg->tileid();
    _type = msg->type();
    _id = msg->id();
    _version = msg->version();
    if (msg->has_is_deleted()) {
        _is_deleted = msg->is_deleted();
    }
    else {
        _is_deleted.reset();
    }
    clear_changed();
    return true;
};

bool FeatureIDProxy::is_valid() const {
    if (_tileid <= 0) {
        return false;
    }
    /*if (_version <= 0) {
        return false;
    }*/
    int type = _type;
    if (type != RoadPB::FeatureID::UNKNOWN
            && type != RoadPB::FeatureID::LINK
            && type != RoadPB::FeatureID::NODE
            && type != RoadPB::FeatureID::LANE
            && type != RoadPB::FeatureID::LANE_BOUNDARY
            && type != RoadPB::FeatureID::LANE_GROUP
            && type != RoadPB::FeatureID::JUNCTION
            && type != RoadPB::FeatureID::TRAFFIC_INFO
            && type != RoadPB::FeatureID::POSITION_OBJ
            && type != RoadPB::FeatureID::ROAD_BOUNDARY
            && type != RoadPB::FeatureID::CONFIDENCE
            && type != RoadPB::FeatureID::ODD) {
        return false;
    }
    return true;
};

bool FeatureIDProxy::correct_content(RoadGeometryManager* pmgr) {
    if (_version <= 0) {
        LOG_BEGIN(INFO) << "ElementIDProxy::correct_content() change version:"
            << to_string() << " to " << pmgr->current_version(); LOG_END;
        _version = pmgr->current_version();
        return true;
    }
    return false;
};

std::shared_ptr<const FeatureWithIDProxyBase> FeatureIDProxy::resolve_id(RoadGeometryManager* mgr) const {
    return mgr->get_feature_by_id(*this);
};

bool FeatureIDProxy::is_weak_reference_valid(RoadGeometryManager* mgr) const {
    if (_owner && _owner->id().get() == this) {
        return true;
    }
    if (_tileid <= 0 || mgr->is_tile_15_downloaded(_tileid)) {
        return false;
    }
    return true;
};

}; // data_access_engine
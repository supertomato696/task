#include "lane_proxy.h"
#include "data_access_engine.h"
#include "manager/road_geometry_mgr.h"
#include "position_proxy.h"
#include "traffic_proxy.h"
#include "link_proxy.h"
#include "tile_proxy.h"
#include "feature_merger.h"
#include "utils/log_util.h"
namespace data_access_engine {

bool LaneDirectionProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::LaneDirection*>(msg_base);
	if (msg->has_direction()) {
		_direction = msg->direction();
	}
	else {
		_direction.reset();
	}
	if (msg->has_valid_period()) {
		_valid_period = msg->valid_period();
	}
	else {
		_valid_period.reset();
	}
	_allowed_vehicle_types.clear();
	_allowed_vehicle_types.reserve(msg->allowed_vehicle_types_size());
	for (int i = 0; i < msg->allowed_vehicle_types_size(); ++i) {
		_allowed_vehicle_types.push_back(msg->allowed_vehicle_types(i));
	}
	clear_changed();
	return true;
};

bool LaneDirectionProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::LaneDirection*>(msg_base);
	msg->Clear();
	if (_direction) {
		msg->set_direction(*_direction);
	}
	if (_valid_period) {
		msg->set_valid_period(*_valid_period);
	}
	for (size_t i = 0; i < _allowed_vehicle_types.size(); ++i) {
		msg->add_allowed_vehicle_types(_allowed_vehicle_types[i]);
	}
	return true;
};

bool LaneDirectionProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
	const std::shared_ptr<const FeatureProxyBase>& base,
	const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const LaneDirectionProxy>(remote);
	auto o = std::dynamic_pointer_cast<const LaneDirectionProxy>(base);
	return ProxyMerger<LaneDirectionProxy>(policy, mgr, this, "LaneDirectionProxy", r, o)
		.merge(_direction, "direction")
		.merge(_valid_period, "valid_period")
		.merge(_allowed_vehicle_types, "allowed_vehicle_types")
		.is_conflict();
}

bool LaneDirectionProxy::is_valid() const {
	if (!_direction) {
		return false;
	}
	return true;
};

int LaneDirectionProxy::compare(const LaneDirectionProxy& rhs) const {
	int ret = value_compare(_direction, rhs._direction);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_valid_period, rhs._valid_period);
	if (ret != 0) {
		return ret;
	}
	return vector_compare(_allowed_vehicle_types, rhs._allowed_vehicle_types);
};

bool LaneDirectionProxy::set_direction(Optional<int> d) {
	if (!is_editable()) {
		return false;
	}
	if (_direction != d) {
		mark_changed();
	}
	_direction = d;
	return true;
};

bool LaneDirectionProxy::set_valid_period(const Optional<std::string>& v) {
	if (!is_editable()) {
		return false;
	}
	if (_valid_period != v) {
		mark_changed();
	}
	_valid_period = v;
	return true;
};

bool LaneDirectionProxy::set_allowed_vehicle_types(const std::vector<int>& vs) {
	if (!is_editable()) {
		return false;
	}
	if (vector_compare(_allowed_vehicle_types, vs) != 0) {
		mark_changed();
	}
	_allowed_vehicle_types = vs;
	return true;
};

PolylineProxy* LaneBoundaryProxy::mutable_geom() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_geom) {
		_geom.reset(FeatureProxyBase::create_proxy<PolylineProxy>(this));
	}
	return _geom.get();
};
bool LaneBoundaryProxy::clear_geom() {
	if (!is_editable()) {
		return false;
	}
	if (_geom) {
		mark_changed();
	}
	_geom.reset();
	return true;
};

bool LaneBoundaryProxy::set_color(Optional<int> c) {
	if (!is_editable()) {
		return false;
	}
	if (_color != c) {
		mark_changed();
	}
	_color = c;
	return true;
}

bool LaneBoundaryProxy::set_types(const std::vector<int>& ts) {
	if (!is_editable()) {
		return false;
	}
	if (vector_compare(_types, ts) != 0) {
		mark_changed();
	}
	_types = ts;
	return true;
};

bool LaneBoundaryProxy::set_marking(Optional<int> m) {
	if (!is_editable()) {
		return false;
	}
	if (_marking != m) {
		mark_changed();
	}
	_marking = m;
	return true;
};

/*bool LaneBoundaryProxy::set_confidence(Optional<int> c) {
	if (!is_editable()) {
		return false;
	}
	if (_confidence != c) {
		mark_changed();
	}
	_confidence = c;
	return true;
};*/

bool LaneBoundaryProxy::set_ldm(Optional<bool> l) {
	if (!is_editable()) {
		return false;
	}
	if (_ldm != l) {
		mark_changed();
	}
	_ldm = l;
	return true;
};

bool LaneBoundaryProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::LaneBoundary*>(msg_base);
	msg->Clear();
	id_to_message(msg);
	if (_geom) {
		_geom->to_message(msg->mutable_geom());
	}
	if (_color) {
		msg->set_color(*_color);
	}
	for (size_t i = 0; i < _types.size(); ++i) {
		msg->add_types(_types[i]);
	}
	if (_marking) {
		msg->set_marking(*_marking);
	}
	/*if (_confidence) {
		msg->set_confidence(*_confidence);
	}*/
	if (_ldm) {
		msg->set_ldm(*_ldm);
	}
	return true;
};

bool LaneBoundaryProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::LaneBoundary*>(msg_base);
	id_from_message(msg);
	if (msg->has_geom()) {
		_geom.reset(FeatureProxyBase::create_proxy<PolylineProxy>(this));
		_geom->from_message(&msg->geom());
	}
	else {
		_geom.reset();
	}
	if (msg->has_color()) {
		_color = msg->color();
	}
	else {
		_color.reset();
	}
	_types.clear();
	_types.reserve(msg->types_size());
	for (int i = 0; i < msg->types_size(); ++i) {
		_types.push_back(msg->types(i));
	}
	if (msg->has_marking()) {
		_marking = msg->marking();
	}
	else {
		_marking.reset();
	}
	/*if (msg->has_confidence()) {
		_confidence = msg->confidence();
	}
	else {
		_confidence.reset();
	}*/
	if (msg->has_ldm()) {
		_ldm = msg->ldm();
	}
	else {
		_ldm.reset();
	}
	clear_changed();
	return true;
};

bool LaneBoundaryProxy::is_ref_valid() const {
    if (!_ref_record || !_id || !_id->cached_is_valid() || _id->type() != feature_id_type()) {
        return false;
    }
    if (owner_count() > 2) {
		return false;
	}
    return true;
};

bool LaneBoundaryProxy::is_valid() const {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	if (!is_message_valid<LaneBoundaryProxy>()) {
		LOG_BEGIN(ERROR) << "LaneBoundaryProxy::is_valid() failed for " << sid
			<< " : is_message_valid() failed!"; LOG_END;
		return false;
	}
	if (!is_editable() || is_deleted()) {
		//LOG_BEGIN(DEBUG) << "LaneBoundaryProxy::is_valid() skip for " << sid; LOG_END;
		return true;
	}
	if (!is_ref_valid()) {
		LOG_BEGIN(ERROR) << "LaneBoundaryProxy::is_valid() failed for " << sid
			<< " : has " << owner_count() << " owners"; LOG_END;
		return false;
	}
	if (_geom && _geom->pts().size() < 2) {
		LOG_BEGIN(ERROR) << "LaneBoundaryProxy::is_valid() failed: " << _geom->pts().size()
			<< " points too few"; LOG_END;
		return false;
	}
	return true;
};
bool LaneBoundaryProxy::judge_editable(RoadGeometryManager* mgr) {
	_editable = true;
	if (!_geom || _geom->pts().empty()) {
		return true;
	}
	std::map<int, double> tid2lens;
	if (!_geom->calc_length_in_tiles(mgr, tid2lens)) {
		return true;
	}
	_editable = mgr->is_editable_in_tiles(tid2lens);
	return _editable;
};
bool LaneBoundaryProxy::correct_tile_refs(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted() || !_geom || !_id) {
		return false;
	}
	bool bc = false;
	std::set<int> ref_tiles;
	for (auto p : _geom->pts()) {
		int tid = mgr->WGS84_to_tileID(*p);
		if (tid <= 0) {
			LOG_DEBUG("LaneBoundaryProxy::correct_tile_refs() for %s found error point @ %d , %d, %d"
			,_id->to_string(), p->x(), p->y(), p->z());
			// LOG_BEGIN(INFO) << "LaneBoundaryProxy::correct_tile_refs() for " << _id->to_string()
			// 	<< " found error point @ " << p->x() << ", " << p->y() << ", " << p->z(); LOG_END;
		}
		if (ref_tiles.count(tid) > 0) {
			continue;
		}
		ref_tiles.insert(tid);
		auto tile = mgr->get_road_tile(tid);
		if (add_missed_ref_tiles(tile.get())) {
			FeatureReference<LaneBoundaryProxy> e;
			e = mutable_changed_id(mgr->current_version());
			tile->lane_boundary_refs().push_back(e);
			LOG_DEBUG("LaneBoundaryProxy::correct_tile_refs(): add tile %d 's ref %s",tile->tile_id(), e->to_string());
			// LOG_BEGIN(INFO) << "LaneBoundaryProxy::correct_tile_refs(): add tile "
			// 	<< tile->tile_id() << " 's ref " << e->to_string(); LOG_END;
			bc = true;
		}
	}
	
	auto tiles = referenced_tiles();
	for (auto& tw : tiles) {
		auto tile = const_cast<TileInfoProxy*>(tw);
		if (tile && ref_tiles.count(tile->tile_id()) <= 0) {
			auto ref = tile->lane_boundary_refs().find(*id());
			if (ref && ref->is_del()) {
				continue;
			}
			auto did = mutable_deleted_id(mgr->current_version());
			tile->lane_boundary_refs().replace(id(), did);
			LOG_DEBUG("LaneBoundaryProxy::correct_tile_refs(): remove tile %d 's ref %s",tile->tile_id() ,did->to_string());
			// LOG_BEGIN(INFO) << "LaneBoundaryProxy::correct_tile_refs(): remove tile "
			// 	<< tile->tile_id() << " 's ref " << did->to_string(); LOG_END;
			bc = true;
		}
	}
	return bc;
};

bool LaneBoundaryProxy::correct_content(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted()) {
		return false;
	}
	std::string sid("!no_id!");
	bool bc = false;
	if (_id) {
		sid = _id->to_string();
		if (!_id->is_type(feature_id_type())) {
			LOG_BEGIN(INFO) << "LaneBoundaryProxy::correct_content() correct: "
				<< sid << " to " << feature_id_type(); LOG_END;
			_id->set_type(feature_id_type());
		}
		bc |= _id->correct_content(mgr);
	}
	else {
		LOG_BEGIN(ERROR) << "LaneBoundaryProxy::correct_content(): no id in element"; LOG_END;
		return false;
	}
	if (!_geom) {
		LOG_BEGIN(INFO) << "LaneBoundaryProxy::correct_content() failed for " << sid
			<< " : no geom"; LOG_END;
		set_deleted();
		return true;
	}
	if (_geom->pts().size() < 2) {
		LOG_BEGIN(ERROR) << "LaneBoundarProxy::correct_content(): set deleted for "
			<< _id->to_string() << ": geom is incompleted"; LOG_END;
		set_deleted();
		return true;
	}

	auto pos = wgs84_pos();
	auto np = std::dynamic_pointer_cast<LaneBoundaryProxy>(shared_from_this());
	auto ptr = std::dynamic_pointer_cast<LaneBoundaryProxy>(shared_from_this());
	TileInfoPtr nti;
	if (mgr->remake_id(np, pos, nti, true)) {
		nti->lane_boundarys().push_back(ptr);
		auto tile = mgr->get_road_tile(np->id()->tileid());
		tile->lane_boundarys().replace(ptr, np);
		auto tiles = referenced_tiles();
		for (auto& tw : tiles) {
			auto tile = const_cast<TileInfoProxy*>(tw);
			if (tile) {
				tile->lane_boundary_refs().replace(ptr->id(), np->mutable_id());
				LOG_BEGIN(INFO) << "LaneBoundaryProxy::correct_content(): replace tile "
					<< tile->tile_id() << " 's ref " << ptr->id()->to_string() << " to "
					<< np->id()->to_string(); LOG_END;
			}
		}
		bc = true;
	}

	bc |= correct_tile_refs(mgr);
	
	bc |= correct_version(mgr);
	return bc;
};

const Vector3D& LaneBoundaryProxy::global_stp_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	sync_wgs_to_global_pos(mgr);
	return _global_stp_pos;
};

const Vector3D& LaneBoundaryProxy::global_edp_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	sync_wgs_to_global_pos(mgr);
	return _global_edp_pos;
};

void LaneBoundaryProxy::sync_wgs_to_global_pos(const RoadGeometryManager* mgr) const {
	if (_pos_sync_cnt < _change_count) {
		if (mgr == nullptr) {
			mgr = DAEInterface::get_instance()->get_road_geometry_manager();
		}
		if (mgr && _geom && _geom->pts().size() > 0) {
			_global_stp_pos = *_geom->pts().front();
			_global_edp_pos = *_geom->pts().back();
			if (mgr->global_point_from_WGS84(_global_stp_pos)
				&& mgr->global_point_from_WGS84(_global_edp_pos)) {
				_pos_sync_cnt = _change_count;
			}
			else {
				LOG_BEGIN(DEBUG) << "LaneBoundaryProxy::sync_wgs_to_global() failed ("
					<< _global_stp_pos << ", " << _global_edp_pos << ")"; LOG_END;
				_global_stp_pos = { 0, 0, 0 };
				_global_edp_pos = { 0, 0, 0 };
			}

		}
		else {
			_global_stp_pos = { 0, 0, 0 };
			_global_edp_pos = { 0, 0, 0 };
			LOG_BEGIN(ERROR) << "LaneBoundaryProxy::sync_wgs_to_global() failed for empty pos"; LOG_END;
		}
	}
	else {
		_pos_sync_cnt = _change_count;
	}
};

Vector3D LaneBoundaryProxy::wgs84_pos() const {
	if (_geom && _geom->pts().size() > 0) {
		return *_geom->pts().front();
	}
	return Vector3D(0, 0, 0);
};


bool LaneBoundaryProxy::merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) {
	return FeatureMerger<LaneBoundaryProxy>(policy, mgr, this, "LaneBoundary")
											.merge(_geom, "geom")
											.merge(_color, "color")
											.merge(_types, "type")
											.merge(_marking, "marking")
											//.merge(_confidence, "confidence")
											.merge(_ldm, "ldm")
											.is_conflict();
};

bool LaneBoundaryRangeProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::LaneBoundaryRange*>(msg_base);
	if (msg->has_bound_id()) {
		_bound_id.create(this)->from_message(&msg->bound_id());
	}
	else {
		_bound_id.reset();
	}
	if (msg->has_start_pt()) {
		_start_pt.reset(FeatureProxyBase::create_proxy<PointProxy>(this));
		_start_pt->from_message(&msg->start_pt());
	}
	else {
		_start_pt.reset();
	}
	if (msg->has_end_pt()) {
		_end_pt.reset(FeatureProxyBase::create_proxy<PointProxy>(this));
		_end_pt->from_message(&msg->end_pt());
	}
	else {
		_end_pt.reset();
	}
	clear_changed();
	return true;
};

bool LaneBoundaryRangeProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::LaneBoundaryRange*>(msg_base);
	msg->Clear();
	if (_bound_id) {
		_bound_id->to_message(msg->mutable_bound_id());
	}
	if (_start_pt) {
		_start_pt->to_message(msg->mutable_start_pt());
	}
	if (_end_pt) {
		_end_pt->to_message(msg->mutable_end_pt());
	}
	return true;
};

bool LaneBoundaryRangeProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
							const std::shared_ptr<const FeatureProxyBase>& base,
							const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const LaneBoundaryRangeProxy>(remote);
	auto o = std::dynamic_pointer_cast<const LaneBoundaryRangeProxy>(base);
	return ProxyMerger<LaneBoundaryRangeProxy>(policy, mgr, this, "LaneBoundaryRangeProxy", r, o)
												.merge(_bound_id, "bound_id")
												.merge(_start_pt, "start_pt")
												.merge(_end_pt, "end_pt")
												.is_conflict();
}

bool LaneBoundaryRangeProxy::is_valid() const {
	if (!_bound_id) {
		LOG_BEGIN(INFO) << "bound_id is empty"; LOG_END;
		return false;
	}
	return true;
};

int LaneBoundaryRangeProxy::compare(const LaneBoundaryRangeProxy& rhs) const {
	int ret = proxy_compare(_start_pt, rhs._start_pt);
	if (ret != 0) {
		return ret;
	}
	ret = proxy_compare(_end_pt, rhs._end_pt);
	if (ret != 0) {
		return ret;
	}
	return proxy_compare(_bound_id.id(), rhs._bound_id.id());
};

FeatureMember<LaneBoundaryProxy>* LaneBoundaryRangeProxy::mutable_bound_id() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_bound_id;
};
PointProxy* LaneBoundaryRangeProxy::mutable_start_pt() { 
	if (!is_editable()) {
		return nullptr;
	}
	if (!_start_pt) {
		_start_pt.reset(FeatureProxyBase::create_proxy<PointProxy>(this));
	}
	return _start_pt.get();
};
bool LaneBoundaryRangeProxy::clear_start_pt() {
	if (!is_editable()) {
		return false;
	}
	if (_start_pt) {
		mark_changed();
	}
	_start_pt.reset();
	return true;
};

PointProxy* LaneBoundaryRangeProxy::mutable_end_pt() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_end_pt) {
		_end_pt.reset(FeatureProxyBase::create_proxy<PointProxy>(this));
	}
	return _end_pt.get();
};
bool LaneBoundaryRangeProxy::clear_end_pt() {
	if (!is_editable()) {
		return false;
	}
	if (_end_pt) {
		mark_changed();
	}
	_end_pt.reset();
	return true;
};

PolylineProxy* RoadBoundaryProxy::mutable_geom() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_geom) {
		_geom.reset(FeatureProxyBase::create_proxy<PolylineProxy>(this));
	}
	return _geom.get();
};
bool RoadBoundaryProxy::clear_geom() {
	if (!is_editable()) {
		return false;
	}
	if (_geom) {
		mark_changed();
	}
	_geom.reset();
	return true;
};

bool RoadBoundaryProxy::set_type(Optional<int> t) {
	if (!is_editable()) {
		return false;
	}
	if (_type != t) {
		mark_changed();
	}
	_type = t;
	return true;
};

/*bool RoadBoundaryProxy::set_confidence(Optional<int> c) {
	if (!is_editable()) {
		return false;
	}
	if (_confidence != c) {
		mark_changed();
	}
	_confidence = c;
	return true;
};

bool RoadBoundaryProxy::set_restriction(const std::vector<int>& r) {
	if (!is_editable()) {
		return false;
	}
	if (vector_compare(_restriction, r) != 0) {
		mark_changed();
	}
	_restriction = r;
	return true;
};*/

bool RoadBoundaryProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::RoadBoundary*>(msg_base);
	msg->Clear();
	id_to_message(msg);
	if (_geom) {
		_geom->to_message(msg->mutable_geom());
	}	
	if (_type) {
		msg->set_type(*_type);
	}	
	/*if (_confidence) {
		msg->set_confidence(*_confidence);
	}
	for (size_t i = 0; i < _restriction.size(); ++i) {
		msg->add_restriction(_restriction[i]);
	}*/
	return true;
};

bool RoadBoundaryProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::RoadBoundary*>(msg_base);
	id_from_message(msg);
	if (msg->has_geom()) {
		_geom.reset(FeatureProxyBase::create_proxy<PolylineProxy>(this));
		_geom->from_message(&msg->geom());
	}
	else {
		_geom.reset();
	}
	if (msg->has_type()) {
		_type = msg->type();
	}
	else {
		_type.reset();
	}
	/*if (msg->has_confidence()) {
		_confidence = msg->confidence();
	}
	else {
		_confidence.reset();
	}
	_restriction.clear();
	_restriction.reserve(msg->restriction_size());
	for (int i = 0; i < msg->restriction_size(); ++i) {
		_restriction.push_back(msg->restriction(i));
	}*/
	clear_changed();
	return true;
};

bool RoadBoundaryProxy::is_valid() const {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	if (!is_message_valid<RoadBoundaryProxy>()) {
		LOG_BEGIN(ERROR) << "RoadBoundaryProxy::is_valid() failed for " << sid
			<< " : is_message_valid() failed!"; LOG_END;
		return false;
	}
	if (!is_editable() || is_deleted()) {
		//LOG_BEGIN(DEBUG) << "RoadBoundaryProxy::is_valid() skip for " << sid; LOG_END;
		return true;
	}
	if (!is_ref_valid()) {
		LOG_BEGIN(ERROR) << "RoadBoundaryProxy::is_valid() failed for " << sid
			<< " : has " << owner_count() << " owners"; LOG_END;
		return false;
	}
	if (_geom && _geom->pts().size() < 2) {
		LOG_BEGIN(ERROR) << "RoadBoundaryProxy::is_valid() failed: " << _geom->pts().size()
			<< " points too few"; LOG_END;
		return false;
	}
	return true;
};

bool RoadBoundaryProxy::judge_editable(RoadGeometryManager* mgr) {
	_editable = true;
	if (!_geom || _geom->pts().empty()) {
		return true;
	}
	std::map<int, double> tid2lens;
	if (!_geom->calc_length_in_tiles(mgr, tid2lens)) {
		return true;
	}
	_editable = mgr->is_editable_in_tiles(tid2lens);
	return _editable;
};

bool RoadBoundaryProxy::correct_tile_refs(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted() || !_geom || !_id) {
		return false;
	}
	bool bc = false;
	std::set<int> ref_tiles;
	for (auto p : _geom->pts()) {
		int tid = mgr->WGS84_to_tileID(*p);
		if (tid <= 0) {
			LOG_DEBUG("RoadBoundaryProxy::correct_tile_refs() for %s found error point @ %d , %d , %d",
						_id->to_string(), p->x(), p->y(), p->z());
			// LOG_BEGIN(INFO) << "RoadBoundaryProxy::correct_tile_refs() for " << _id->to_string()
			// 	<< " found error point @ " << p->x() << ", " << p->y() << ", " << p->z(); LOG_END;
		}
		if (ref_tiles.count(tid) > 0) {
			continue;
		}
		ref_tiles.insert(tid);
		auto tile = mgr->get_road_tile(tid);
		if (add_missed_ref_tiles(tile.get())) {
			FeatureReference<RoadBoundaryProxy> e;
			e = mutable_changed_id(mgr->current_version());
			tile->road_boundary_refs().push_back(e);
			LOG_DEBUG("RoadBoundaryProxy::correct_tile_refs(): add tile %d 's ref  %s",
						tile->tile_id(), e->to_string());
			// LOG_BEGIN(INFO) << "RoadBoundaryProxy::correct_tile_refs(): add tile "
			// 	<< tile->tile_id() << " 's ref " << e->to_string(); LOG_END;
			bc = true;
		}
	}

	auto tiles = referenced_tiles();
	for (auto& tw : tiles) {
		auto tile = const_cast<TileInfoProxy*>(tw);
		if (tile && ref_tiles.count(tile->tile_id()) <= 0) {
			auto ref = tile->road_boundary_refs().find(*id());
			if (ref && ref->is_del()) {
				continue;
			}
			auto did = mutable_deleted_id(mgr->current_version());
			tile->road_boundary_refs().replace(id(), did);
			LOG_DEBUG("RoadBoundaryProxy::correct_tile_refs(): remove tile %d 's ref  %s",
						tile->tile_id(), did->to_string());
			// LOG_BEGIN(INFO) << "RoadBoundaryProxy::correct_tile_refs(): remove tile "
			// 	<< tile->tile_id() << " 's ref " << did->to_string(); LOG_END;
			bc = true;
		}
	}
	return bc;
}

bool RoadBoundaryProxy::correct_content(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted()) {
		return false;
	}
	std::string sid("!no_id!");
	bool bc = false;
	if (_id) {
		sid = _id->to_string();
		if (!_id->is_type(feature_id_type())) {
			LOG_BEGIN(INFO) << "RoadBoundaryProxy::correct_content() correct: "
				<< sid << " to " << feature_id_type(); LOG_END;
			_id->set_type(feature_id_type());
		}
		bc |= _id->correct_content(mgr);
	}
	else {
		LOG_BEGIN(ERROR) << "RoadBoundaryProxy::correct_content(): no id in element"; LOG_END;
		return false;
	}
	if (!_geom) {
		LOG_BEGIN(INFO) << "RoadBoundaryProxy::correct_content() failed for " << sid
			<< " : no geom"; LOG_END;
		set_deleted();
		return true;
	}
	if (_geom->pts().size() < 2) {
		LOG_BEGIN(ERROR) << "RoadBoundarProxy::correct_content(): set deleted for "
			<< _id->to_string() << ": geom is incompleted"; LOG_END;
		set_deleted();
		return true;
	}

	auto pos = wgs84_pos();
	auto np = std::dynamic_pointer_cast<RoadBoundaryProxy>(shared_from_this());
	auto ptr = std::dynamic_pointer_cast<RoadBoundaryProxy>(shared_from_this());
	TileInfoPtr nti;
	if (mgr->remake_id(np, pos, nti, true)) {
		nti->road_boundarys().push_back(ptr);
		auto tile = mgr->get_road_tile(np->id()->tileid());
		tile->road_boundarys().replace(ptr, np);
		auto tiles = referenced_tiles();
		for (auto& tw : tiles) {
			auto tile = const_cast<TileInfoProxy*>(tw);
			if (tile) {
				tile->road_boundary_refs().replace(ptr->id(), np->mutable_id());
				LOG_BEGIN(INFO) << "RoadBoundaryProxy::correct_content(): replace tile "
					<< tile->tile_id() << " 's ref " << ptr->id()->to_string() << " to "
					<< np->id()->to_string(); LOG_END;
			}
		}
		bc = true;
	}

	bc |= correct_tile_refs(mgr);

	bc |= correct_version(mgr);
	return bc;
};

const Vector3D& RoadBoundaryProxy::global_stp_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	sync_wgs_to_global_pos(mgr);
	return _global_stp_pos;
};

const Vector3D& RoadBoundaryProxy::global_edp_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	sync_wgs_to_global_pos(mgr);
	return _global_edp_pos;
};

void RoadBoundaryProxy::sync_wgs_to_global_pos(const RoadGeometryManager* mgr) const {
	if (_pos_sync_cnt < _change_count) {
		if (mgr == nullptr) {
			mgr = DAEInterface::get_instance()->get_road_geometry_manager();
		}
		if (mgr && _geom && _geom->pts().size() > 0) {
			_global_stp_pos = *_geom->pts().front();
			_global_edp_pos = *_geom->pts().back();
			if (mgr->global_point_from_WGS84(_global_stp_pos)
				&& mgr->global_point_from_WGS84(_global_edp_pos)) {
				_pos_sync_cnt = _change_count;
			}
			else {
				LOG_BEGIN(DEBUG) << "RoadBoundaryProxy::sync_wgs_to_global() failed ("
					<< _global_stp_pos << ", " << _global_edp_pos << ")"; LOG_END;
				_global_stp_pos = { 0, 0, 0 };
				_global_edp_pos = { 0, 0, 0 };
			}

		}
		else {
			_global_stp_pos = { 0, 0, 0 };
			_global_edp_pos = { 0, 0, 0 };
			LOG_BEGIN(ERROR) << "RoadBoundaryProxy::sync_wgs_to_global() failed for empty pos"; LOG_END;
		}
	}
	else {
		_pos_sync_cnt = _change_count;
	}
};

Vector3D RoadBoundaryProxy::wgs84_pos() const {
	if (_geom && _geom->pts().size() > 0) {
		return *_geom->pts().front();
	}
	return Vector3D(0, 0, 0);
};

bool RoadBoundaryProxy::merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) {
	return FeatureMerger<RoadBoundaryProxy>(policy, mgr, this, "RoadBoundary")
											.merge(_geom, "geom")
											.merge(_type, "type")
											//.merge(_confidence, "confidence")
											//.merge(_restriction, "restriction")
											.is_conflict();
};

LaneBoundaryRangeProxy* LaneSectionProxy::mutable_left_boundary() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_left_boundary) {
		_left_boundary.reset(FeatureProxyBase::create_proxy<LaneBoundaryRangeProxy>(this));
	}
	return _left_boundary.get();
};
bool LaneSectionProxy::clear_left_boundary() {
	if (!is_editable()) {
		return false;
	}
	if (_left_boundary) {
		mark_changed();
	}
	_left_boundary.reset();
	return true;
};
LaneBoundaryRangeProxy* LaneSectionProxy::mutable_right_boundary() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_right_boundary) {
		_right_boundary.reset(FeatureProxyBase::create_proxy<LaneBoundaryRangeProxy>(this));
	}
	return _right_boundary.get();
};
bool LaneSectionProxy::clear_right_boundary() {
	if (!is_editable()) {
		return false;
	}
	if (_right_boundary) {
		mark_changed();
	}
	_right_boundary.reset();
	return true;
};

SharedProxyVector<FixedSpeedLimitProxy>* LaneSectionProxy::mutable_speed_limits() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_speed_limits;
};

FeatureReferenceVector<PositionObjectProxy>* LaneSectionProxy::mutable_objects() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_objects;
};

FeatureReferenceVector<TrafficInfoProxy>* LaneSectionProxy::mutable_traffics() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_traffics;
};

bool LaneSectionProxy::set_width(Optional<float> w) {
	if (!is_editable()) {
		return false;
	}
	if (_width != w) {
		mark_changed();
	}
	_width = w;
	return true;
};

bool LaneSectionProxy::set_height_limit(Optional<float> h) {
	if (!is_editable()) {
		return false;
	}
	if (_height_limit != h) {
		mark_changed();
	}
	_height_limit = h;
	return true;
};

bool LaneSectionProxy::set_weight_limit(Optional<float> w) {
	if (!is_editable()) {
		return false;
	}
	if (_weight_limit != w) {
		mark_changed();
	}
	_weight_limit = w;
	return true;
};

bool LaneSectionProxy::set_width_limit(Optional<float> w) {
	if (!is_editable()) {
		return false;
	}
	if (_width_limit != w) {
		mark_changed();
	}
	_width_limit = w;
	return true;
};

const Vector3D& LaneSectionProxy::global_left_start_pos(const RoadGeometryManager* mgr) const {
	if (_lstp_sync_cnt < _change_count) {
		if (mgr == nullptr) {
			mgr = DAEInterface::get_instance()->get_road_geometry_manager();
		}
		if (mgr && _left_boundary && _left_boundary->start_pt()) {
			Vector3D pos = *(_left_boundary->start_pt());
			if (mgr && mgr->global_point_from_WGS84(pos)) {
				Vector3D tp = *(_left_boundary->start_pt());
				LOG_BEGIN(DEBUG) << "LaneSectionProxy::global_left_start_pos() (" << tp
					<< ") -> (" << pos << ")"; LOG_END;
				_global_lstp_pos = pos;
				_lstp_sync_cnt = _change_count;
			}
			else {
				Vector3D tp = *(_left_boundary->start_pt());
				LOG_BEGIN(ERROR) << "LaneSectionProxy::global_left_start_pos() failed for ("
					<< tp << ")"; LOG_END;
			}
		}
		else {
			Vector3D tp;
			if (_left_boundary && _left_boundary->start_pt()) {
				tp = *(_left_boundary->start_pt());
			}
			LOG_BEGIN(ERROR) << "LaneSectionProxy::global_left_start_pos() failed for left_start_point:("
				<< tp << ") mgr:" << mgr; LOG_END;
		}
	}
	else {
		_lstp_sync_cnt = _change_count;
	}
	return _global_lstp_pos;
};
bool LaneSectionProxy::set_global_left_start_pos(const Vector3D& p, const RoadGeometryManager* mgr) {
	if (!is_editable()) {
		return false;
	}
	_global_lstp_pos = p;
	if (mgr == nullptr) {
		mgr = DAEInterface::get_instance()->get_road_geometry_manager();
	}
	if (mgr && _left_boundary) {
		Vector3D pos = _global_lstp_pos;
		if (mgr->global_point_to_WGS84(pos)) {
			auto pp = _left_boundary->mutable_start_pt();
			pp->set_x(pos.X());
			pp->set_y(pos.Y());
			pp->set_z(pos.Z());
			LOG_BEGIN(DEBUG) << "LaneSectionProxy::set_global_left_start_pos() (" << p
				<< ") -> (" << pos << ")"; LOG_END;
			_lstp_sync_cnt = _change_count;
		}
		else {
			LOG_BEGIN(ERROR) << "LaneSectionProxy::set_global_left_start_pos() failed for ("
				<< p << ")"; LOG_END;
		}
	}
	else {
		LOG_BEGIN(ERROR) << "LaneSectionProxy::set_global_left_start_pos() failed for pos:("
			<< p << ") mgr:" << mgr; LOG_END;
	}
	return true;
};

const Vector3D& LaneSectionProxy::global_left_end_pos(const RoadGeometryManager* mgr) const {
	if (_ledp_sync_cnt < _change_count) {
		if (mgr == nullptr) {
			mgr = DAEInterface::get_instance()->get_road_geometry_manager();
		}
		if (mgr && _left_boundary && _left_boundary->end_pt()) {
			Vector3D pos = *(_left_boundary->end_pt());
			if (mgr && mgr->global_point_from_WGS84(pos)) {
				Vector3D tp = *(_left_boundary->end_pt());
				LOG_BEGIN(DEBUG) << "LaneSectionProxy::global_left_end_pos() (" << tp
					<< ") -> (" << pos << ")"; LOG_END;
				_global_ledp_pos = pos;
				_ledp_sync_cnt = _change_count;
			}
			else {
				Vector3D tp = *(_left_boundary->end_pt());
				LOG_BEGIN(ERROR) << "LaneSectionProxy::global_left_end_pos() failed for ("
					<< tp << ")"; LOG_END;
			}
		}
		else {
			Vector3D tp;
			if (_left_boundary && _left_boundary->end_pt()) {
				tp = *(_left_boundary->end_pt());
			}
			LOG_BEGIN(ERROR) << "LaneSectionProxy::global_left_end_pos() failed for left_end_point:("
				<< tp << ") mgr:" << mgr; LOG_END;
		}
	}
	else {
		_ledp_sync_cnt = _change_count;
	}
	return _global_ledp_pos;
};
bool LaneSectionProxy::set_global_left_end_pos(const Vector3D& p, const RoadGeometryManager* mgr) {
	if (!is_editable()) {
		return false;
	}
	_global_ledp_pos = p;
	if (mgr == nullptr) {
		mgr = DAEInterface::get_instance()->get_road_geometry_manager();
	}
	if (mgr && _left_boundary) {
		Vector3D pos = _global_ledp_pos;
		if (mgr->global_point_to_WGS84(pos)) {
			auto pp = _left_boundary->mutable_end_pt();
			pp->set_x(pos.X());
			pp->set_y(pos.Y());
			pp->set_z(pos.Z());
			LOG_BEGIN(DEBUG) << "LaneSectionProxy::set_global_left_end_pos() (" << p
				<< ") -> (" << pos << ")"; LOG_END;
			_ledp_sync_cnt = _change_count;
		}
		else {
			LOG_BEGIN(ERROR) << "LaneSectionProxy::set_global_left_end_pos() failed for ("
				<< p << ")"; LOG_END;
		}
	}
	else {
		LOG_BEGIN(ERROR) << "LaneSectionProxy::set_global_left_end_pos() failed for pos:("
			<< p << ") mgr:" << mgr; LOG_END;
	}
	return true;
};

const Vector3D& LaneSectionProxy::global_right_start_pos(const RoadGeometryManager* mgr) const {
	if (_rstp_sync_cnt < _change_count) {
		if (mgr == nullptr) {
			mgr = DAEInterface::get_instance()->get_road_geometry_manager();
		}
		if (mgr && _right_boundary && _right_boundary->start_pt()) {
			Vector3D pos = *(_right_boundary->start_pt());
			if (mgr && mgr->global_point_from_WGS84(pos)) {
				Vector3D tp = *(_right_boundary->start_pt());
				LOG_BEGIN(DEBUG) << "LaneSectionProxy::global_right_start_pos() (" << tp
					<< ") -> (" << pos << ")"; LOG_END;
				_global_rstp_pos = pos;
				_rstp_sync_cnt = _change_count;
			}
			else {
				Vector3D tp = *(_right_boundary->start_pt());
				LOG_BEGIN(ERROR) << "LaneSectionProxy::global_right_start_pos() failed for ("
					<< tp << ")"; LOG_END;
			}
		}
		else {
			Vector3D tp;
			if (_right_boundary && _right_boundary->start_pt()) {
				tp = *(_right_boundary->start_pt());
			}
			LOG_BEGIN(ERROR) << "LaneSectionProxy::global_right_start_pos() failed for right_start_point:("
				<< tp << ") mgr:" << mgr; LOG_END;
		}
	}
	else {
		_rstp_sync_cnt = _change_count;
	}
	return _global_rstp_pos;
};
bool LaneSectionProxy::set_global_right_start_pos(const Vector3D& p, const RoadGeometryManager* mgr) {
	if (!is_editable()) {
		return false;
	}
	_global_rstp_pos = p;
	if (mgr == nullptr) {
		mgr = DAEInterface::get_instance()->get_road_geometry_manager();
	}
	if (mgr && _right_boundary) {
		Vector3D pos = _global_rstp_pos;
		if (mgr->global_point_to_WGS84(pos)) {
			auto pp = _right_boundary->mutable_start_pt();
			pp->set_x(pos.X());
			pp->set_y(pos.Y());
			pp->set_z(pos.Z());
			LOG_BEGIN(DEBUG) << "LaneSectionProxy::set_global_right_start_pos() (" << p
				<< ") -> (" << pos << ")"; LOG_END;
			_rstp_sync_cnt = _change_count;
		}
		else {
			LOG_BEGIN(ERROR) << "LaneSectionProxy::set_global_right_start_pos() failed for ("
				<< p << ")"; LOG_END;
		}
	}
	else {
		LOG_BEGIN(ERROR) << "LaneSectionProxy::set_global_right_start_pos() failed for pos:("
			<< p << ") mgr:" << mgr; LOG_END;
	}
	return true;
};

const Vector3D& LaneSectionProxy::global_right_end_pos(const RoadGeometryManager* mgr) const {
	if (_redp_sync_cnt < _change_count) {
		if (mgr == nullptr) {
			mgr = DAEInterface::get_instance()->get_road_geometry_manager();
		}
		if (mgr && _right_boundary && _right_boundary->end_pt()) {
			Vector3D pos = *(_right_boundary->end_pt());
			if (mgr && mgr->global_point_from_WGS84(pos)) {
				Vector3D tp = *(_right_boundary->end_pt());
				LOG_BEGIN(DEBUG) << "LaneSectionProxy::global_right_end_pos() (" << tp
					<< ") -> (" << pos << ")"; LOG_END;
				_global_redp_pos = pos;
				_redp_sync_cnt = _change_count;
			}
			else {
				Vector3D tp = *(_right_boundary->end_pt());
				LOG_BEGIN(ERROR) << "LaneSectionProxy::global_right_end_pos() failed for ("
					<< tp << ")"; LOG_END;
			}
		}
		else {
			Vector3D tp;
			if (_right_boundary && _right_boundary->end_pt()) {
				tp = *(_right_boundary->end_pt());
			}
			LOG_BEGIN(ERROR) << "LaneSectionProxy::global_right_end_pos() failed for right_end_point:("
				<< tp << ") mgr:" << mgr; LOG_END;
		}
	}
	else {
		_redp_sync_cnt = _change_count;
	}
	return _global_redp_pos;
};
bool LaneSectionProxy::set_global_right_end_pos(const Vector3D& p, const RoadGeometryManager* mgr) {
	if (!is_editable()) {
		return false;
	}
	_global_redp_pos = p;
	if (mgr == nullptr) {
		mgr = DAEInterface::get_instance()->get_road_geometry_manager();
	}
	if (mgr && _right_boundary) {
		Vector3D pos = _global_redp_pos;
		if (mgr->global_point_to_WGS84(pos)) {
			auto pp = _right_boundary->mutable_end_pt();
			pp->set_x(pos.X());
			pp->set_y(pos.Y());
			pp->set_z(pos.Z());
			LOG_BEGIN(DEBUG) << "LaneSectionProxy::set_global_right_end_pos() (" << p
				<< ") -> (" << pos << ")"; LOG_END;
			_redp_sync_cnt = _change_count;
		}
		else {
			LOG_BEGIN(ERROR) << "LaneSectionProxy::set_global_right_end_pos() failed for ("
				<< p << ")"; LOG_END;
		}
	}
	else {
		LOG_BEGIN(ERROR) << "LaneSectionProxy::set_global_right_end_pos() failed for pos:("
			<< p << ") mgr:" << mgr; LOG_END;
	}
	return true;
};

bool LaneSectionProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::LaneSection*>(msg_base);
	msg->Clear();
	if (_left_boundary) {
		_left_boundary->to_message(msg->mutable_left_boundary());
	}
	if (_right_boundary) {
		_right_boundary->to_message(msg->mutable_right_boundary());
	}
	_speed_limits.to_message(msg->mutable_speed_limits());
	_objects.to_message(msg->mutable_objects());
	_traffics.to_message(msg->mutable_traffics());
	if (_width) {
		msg->set_width(*_width);
	}
	if (_height_limit) {
		msg->set_height_limit(*_height_limit);
	}
	if (_weight_limit) {
		msg->set_weight_limit(*_weight_limit);
	}
	if (_width_limit) {
		msg->set_width_limit(*_width_limit);
	}
	return true;
};

bool LaneSectionProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::LaneSection*>(msg_base);
	if (msg->has_left_boundary()) {
		_left_boundary.reset(FeatureProxyBase::create_proxy<LaneBoundaryRangeProxy>(this));
		_left_boundary->from_message(&msg->left_boundary());
	}
	else {
		_left_boundary.reset();
	}
	if (msg->has_right_boundary()) {
		_right_boundary.reset(FeatureProxyBase::create_proxy<LaneBoundaryRangeProxy>(this));
		_right_boundary->from_message(&msg->right_boundary());
	}
	else {
		_right_boundary.reset();
	}
	_speed_limits.from_message(msg->speed_limits());
	_objects.from_message(msg->objects());
	_traffics.from_message(msg->traffics());
	if (msg->has_width()) {
		_width = msg->width();
	}
	else {
		_width.reset();
	}
	if (msg->has_height_limit()) {
		_height_limit = msg->height_limit();
	}
	else {
		_height_limit.reset();
	}
	if (msg->has_weight_limit()) {
		_weight_limit = msg->width_limit();
	}
	else {
		_weight_limit.reset();
	}
	if (msg->has_width_limit()) {
		_width_limit = msg->width_limit();
	}
	else {
		_width_limit.reset();
	}
	clear_changed();
	return true;
};

bool LaneSectionProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
	const std::shared_ptr<const FeatureProxyBase>& base,
	const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const LaneSectionProxy>(remote);
	auto o = std::dynamic_pointer_cast<const LaneSectionProxy>(base);
	return ProxyMerger<LaneSectionProxy>(policy, mgr, this, "LaneSectionProxy", r, o)
										.merge(_left_boundary, "left_boundary")
										.merge(_right_boundary, "right_boundary")
										.merge_elemental(_speed_limits, "speed_limits")
										.merge_unordered(_objects, "objects")
										.merge_unordered(_traffics, "traffics")
										.merge(_width, "width")
										.merge(_height_limit, "height_limit")
										.merge(_weight_limit, "weight_limit")
										.merge(_width_limit, "width_limit")
										.is_conflict();
}

bool LaneSectionProxy::is_valid() const {
	if (!_left_boundary && !_right_boundary) {
		LOG_BEGIN(INFO) << "LaneSectionProxy::is_valid() failed: no left_boundary && right_boundary"; LOG_END;
		return false;
	}
	if (_left_boundary && !_left_boundary->cached_is_valid()) {
		LOG_BEGIN(ERROR) << "LaneSectionProxy::is_valid() failed: left_boundary "
			<< global_left_start_pos() << " - " << global_left_end_pos() << " is invalid"; LOG_END;
		return false;
	}
	if (_right_boundary && !_right_boundary->cached_is_valid()) {
		LOG_BEGIN(ERROR) << "LaneSectionProxy::is_valid() failed: right_boundary "
			<< global_right_start_pos() << " - " << global_right_end_pos() << " is invalid"; LOG_END;
		return false;
	}
	return true;
};

bool LaneSectionProxy::correct_content(RoadGeometryManager* mgr) {
	return false;
};

bool LaneSectionProxy::remake_proxy(RoadGeometryManager* mgr) {
	bool bf = true;
	if (_left_boundary && _left_boundary->_bound_id) {
		auto p = _left_boundary->_bound_id->resolve_id(mgr);
		if (p) {
			_left_boundary->_bound_id.set(p->id());
		}
		else {
			bf = false;
			LOG_BEGIN(INFO) << "LaneSectionProxy::remake_proxy(): failed for "
				<< _left_boundary->_bound_id->to_string(); LOG_END;
		}
	}
	if (_right_boundary && _right_boundary->_bound_id) {
		auto p = _right_boundary->_bound_id->resolve_id(mgr);
		if (p) {
			_right_boundary->_bound_id.set(p->id());
		}
		else {
			bf = false;
			LOG_BEGIN(INFO) << "LaneSectionProxy::remake_proxy(): failed for "
				<< _right_boundary->_bound_id->to_string(); LOG_END;
		}
	}
	return bf;
};

int LaneSectionProxy::compare(const LaneSectionProxy& rhs) const {
	int ret = proxy_compare(_left_boundary, rhs._left_boundary);
	if (ret != 0) {
		return ret;
	}
	ret = proxy_compare(_right_boundary, rhs._right_boundary);
	if (ret != 0) {
		return ret;
	}
	ret = proxys_compare(_speed_limits, rhs._speed_limits);
	if (ret != 0) {
		return ret;
	}
	ret = refs_compare(_objects, rhs._objects);
	if (ret != 0) {
		return ret;
	}
	ret = refs_compare(_traffics, rhs._traffics);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_width, rhs._width);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_height_limit, rhs._height_limit);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_weight_limit, rhs._weight_limit);
	if (ret != 0) {
		return ret;
	}
	return value_compare(_width_limit, rhs._width_limit);
};

SharedProxyVector<LaneSectionProxy>* LaneProxy::mutable_lanes() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_lanes;
};

FeatureMemberVector<LaneBoundaryProxy>* LaneProxy::mutable_drivelines() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_drivelines;
};

/*bool LaneProxy::set_function(Optional<int> f) {
	if (!is_editable()) {
		return false;
	}
	if (_function != f) {
		mark_changed();
	}
	_function = f;
	return true;
};*/

bool LaneProxy::set_type(Optional<uint64_t> t) {
	if (!is_editable()) {
		return false;
	}
	if (_type != t) {
		mark_changed();
	}
	_type = t;
	return true;
};

/*bool LaneProxy::set_kind(Optional<int> k) {
	if (!is_editable()) {
		return false;
	}
	if (_kind != k) {
		mark_changed();
	}
	_kind = k;
	return true;
};*/

SharedProxyVector<LaneDirectionProxy>* LaneProxy::mutable_directions() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_directions;
};

/*bool LaneProxy::set_priority(Optional<int> p) {
	if (!is_editable()) {
		return false;
	}
	if (_priority != p) {
		mark_changed();
	}
	_priority = p;
	return true;
};*/

bool LaneProxy::set_length(Optional<float> l) {
	if (!is_editable()) {
		return false;
	}
	if (_length != l) {
		mark_changed();
	}
	_length = l;
	return true;
};

bool LaneProxy::set_seq_no(Optional<int> s) {
	if (!is_editable()) {
		return false;
	}
	if (_seq_no != s) {
		mark_changed();
	}
	_seq_no = s;
	return true;
};

FeatureReferenceVector<LaneProxy>* LaneProxy::mutable_preds() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_preds;
};

FeatureReferenceVector<LaneProxy>* LaneProxy::mutable_succs() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_succs;
};

Vector3D LaneProxy::wgs84_pos() const {
	for (size_t i = 0; i < _lanes.size(); ++i) {
		auto& l = _lanes[i];
		if (l && l->right_boundary() && l->right_boundary()->start_pt()) {
			return *(l->right_boundary()->start_pt());
		}
	}
	return Vector3D(0, 0, 0);
};
const Vector3D& LaneProxy::global_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	return global_right_start_pos(mgr);	
};

const Vector3D& LaneProxy::global_left_start_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	for (size_t i = 0; i < _lanes.size(); ++i) {
		auto& l = _lanes[i];
		if (l) {
			return l->global_left_start_pos(mgr);
		}
	}
	static Vector3D dummy(0, 0, 0);
	return dummy;
};

const Vector3D& LaneProxy::global_right_start_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	for (size_t i = 0; i < _lanes.size(); ++i) {
		auto& l = _lanes[i];
		if (l) {
			return l->global_right_start_pos(mgr);
		}
	}
	static Vector3D dummy(0, 0, 0);
	return dummy;
};

const Vector3D& LaneProxy::global_left_end_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	for (size_t i = _lanes.size(); i > 0; --i) {
		auto& l = _lanes[i - 1];
		if (l) {
			return l->global_left_end_pos(mgr);
		}
	}
	static Vector3D dummy(0, 0, 0);
	return dummy;
};

const Vector3D& LaneProxy::global_right_end_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	for (size_t i = _lanes.size(); i > 0; --i) {
		auto& l = _lanes[i - 1];
		if (l) {
			return l->global_right_end_pos(mgr);
		}
	}
	static Vector3D dummy(0, 0, 0);
	return dummy;
};

bool LaneProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::Lane*>(msg_base);
	msg->Clear();
	id_to_message(msg);
	_lanes.to_message(msg->mutable_lanes());
	_drivelines.to_message(msg->mutable_drivelines());
	/*if (_function) {
		msg->set_function(*_function);
	}*/
	if (_type) {
		msg->set_type(*_type);
	}
	/*if (_kind) {
		msg->set_kind(*_kind);
	}*/
	_directions.to_message(msg->mutable_directions());
	/*if (_priority) {
		msg->set_priority(*_priority);
	}*/
	if (_length) {
		msg->set_length(*_length);
	}
	if (_seq_no) {
		msg->set_seq_no(*_seq_no);
	}
	_preds.to_message(msg->mutable_preds());
	_succs.to_message(msg->mutable_succs());
	return true;
};

bool LaneProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::Lane*>(msg_base);
	id_from_message(msg);
	_lanes.from_message(msg->lanes());
	_drivelines.from_message(msg->drivelines());
	/*if (msg->has_function()) {
		_function = msg->function();
	}
	else {
		_function.reset();
	}*/
	if (msg->has_type()) {
		_type = msg->type();
	}
	else {
		_type.reset();
	}
	/*if (msg->has_kind()) {
		_kind = msg->kind();
	}
	else {
		_kind.reset();
	}*/
	_directions.from_message(msg->directions());
	/*if (msg->has_priority()) {
		_priority = msg->priority();
	}
	else {
		_priority.reset();
	}*/
	if (msg->has_length()) {
		_length = msg->length();
	}
	else {
		_length.reset();
	}
	if (msg->has_seq_no()) {
		_seq_no = msg->seq_no();
	}
	else {
		_seq_no.reset();
	}
	_preds.from_message(msg->preds());
	_succs.from_message(msg->succs());
	clear_changed();
	return true;
};

bool LaneProxy::is_valid() const {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	if (!is_message_valid<LaneProxy>()) {
		LOG_BEGIN(ERROR) << "LaneProxy::is_valid(): failed for " << sid
			<< " : is_message_valid() failed!"; LOG_END;
		return false;
	}
	if (!is_editable() || is_deleted()) {
		//LOG_BEGIN(DEBUG) << "LaneProxy::is_valid() skip for " << sid; LOG_END;
		return true;
	}
	if (!is_ref_valid()) {
		LOG_BEGIN(ERROR) << "LaneProxy::is_valid() failed for " << sid
			<< " : has " << owner_count() << " owners"; LOG_END;
		return false;
	}
	if (!_preds.is_valid(true) || !_succs.is_valid(true)) {
		LOG_BEGIN(ERROR) << "LaneProxy::is_valid() failed for " << sid
			<< " : has " << _preds.size() << " preds, "
			<< _succs.size() << " succs"; LOG_END;
		return false;
	}
	if (_lanes.empty()) {
		LOG_BEGIN(ERROR) << "LaneProxy::is_valid() failed for " << sid
			<< " : has empty lanes!"; LOG_END;
		return false;
	}
	
	if (!_lanes.is_valid()) {
		LOG_BEGIN(ERROR) << "LaneProxy::is_valid() failed for " << sid
			<< " : has invalid lanes " << _lanes.size(); LOG_END;
		return false;
	}
	if (!_drivelines.is_valid(true)) {
		LOG_BEGIN(ERROR) << "LaneProxy::is_valid() failed for " << sid
			<< " : has invalid drivelines " << _drivelines.size(); LOG_END;
		return false;
	}
	if (!_directions.is_valid()) {
		LOG_BEGIN(INFO) << "LaneProxy::is_valid() failed for " << sid
			<< " : has invalid directions " << _directions.size(); LOG_END;
		return false;
	}
	return true;
};

bool LaneProxy::judge_editable(RoadGeometryManager* mgr) {
	_editable = true;
	
	std::map<int, double> tid2lens;
	std::set<const LaneBoundaryProxy*> lbs;
	for (auto& sec : _lanes) {
		if (sec->left_boundary() && sec->left_boundary()->bound_id()) {
			if (!sec->left_boundary()->bound_id().proxy()) {
				if (!mgr->is_tile_15_current_downloaded(sec->left_boundary()->bound_id()->tileid())) {
					_editable = false;
					return false;
				}
				continue;
			}
			if (lbs.find(sec->left_boundary()->bound_id().proxy().get()) == lbs.end()) {
				lbs.insert(sec->left_boundary()->bound_id().proxy().get());
				if (sec->left_boundary()->bound_id().proxy()->geom()) {
					sec->left_boundary()->bound_id().proxy()->geom()->calc_length_in_tiles(mgr, tid2lens);
				}
			}
		}
		if (sec->right_boundary() && sec->right_boundary()->bound_id()) {
			if (!sec->right_boundary()->bound_id().proxy()) {
				if (!mgr->is_tile_15_current_downloaded(sec->right_boundary()->bound_id()->tileid())) {
					_editable = false;
					return false;
				}
				continue;
			}
			if (lbs.find(sec->right_boundary()->bound_id().proxy().get()) == lbs.end()) {
				lbs.insert(sec->right_boundary()->bound_id().proxy().get());
				if (sec->right_boundary()->bound_id().proxy()->geom()) {
					sec->right_boundary()->bound_id().proxy()->geom()->calc_length_in_tiles(mgr, tid2lens);
				}
			}
		}
	}
	_editable = mgr->is_editable_in_tiles(tid2lens);
	return _editable;
};

bool LaneProxy::correct_tile_refs(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted() || !_id) {
		return false;
	}
	bool bc = false;
	std::set<int> ref_tiles;
	for (auto& l : _lanes) {
		if (!l) {
			continue;
		}
		std::vector<const TileInfoProxy*> tiles;
		if (l->left_boundary() && l->left_boundary()->bound_id().proxy()) {
			tiles = l->left_boundary()->bound_id().proxy()->referenced_tiles();
		}
		for (auto t : tiles) {
			if (ref_tiles.count(t->tile_id()) > 0) {
				continue;
			}
			ref_tiles.insert(t->tile_id());
			auto tile = mgr->get_road_tile(t->tile_id());
			if (add_missed_ref_tiles(tile.get())) {
				FeatureReference<LaneProxy> e;
				e = mutable_changed_id(mgr->current_version());
				tile->lane_refs().push_back(e);
				CLOG_DEBUG("LaneProxy::correct_tile_refs(): left_boundary add tile %d s ref %s",tile->tile_id(),e->to_string());
				// LOG_BEGIN(INFO) << "LaneProxy::correct_tile_refs(): left_boundary add tile "
				// 	<< tile->tile_id() << " 's ref " << e->to_string(); LOG_END;
				bc = true;
			}
		}
		tiles.clear();
		if (l->right_boundary() && l->right_boundary()->bound_id().proxy()) {
			tiles = l->right_boundary()->bound_id().proxy()->referenced_tiles();
		}
		for (auto t : tiles) {
			if (ref_tiles.count(t->tile_id()) > 0) {
				continue;
			}
			ref_tiles.insert(t->tile_id());
			auto tile = mgr->get_road_tile(t->tile_id());
			if (add_missed_ref_tiles(tile.get())) {
				FeatureReference<LaneProxy> e;
				e = mutable_changed_id(mgr->current_version());
				tile->lane_refs().push_back(e);
				CLOG_DEBUG("LaneProxy::correct_tile_refs(): right_boundary add tile %d  's ref %s",tile->tile_id(),e->to_string());
				// LOG_BEGIN(INFO) << "LaneProxy::correct_tile_refs(): right_boundary add tile "
				// 	<< tile->tile_id() << " 's ref " << e->to_string(); LOG_END;
				bc = true;
			}
		}
	}
	for (auto l : _drivelines) {
		std::vector<const TileInfoProxy*> tiles;
		if (l && l.proxy()) {
			tiles = l.proxy()->referenced_tiles();
		}
		for (auto t : tiles) {
			if (ref_tiles.count(t->tile_id()) > 0) {
				continue;
			}
			ref_tiles.insert(t->tile_id());
			auto tile = mgr->get_road_tile(t->tile_id());
			if (add_missed_ref_tiles(tile.get())) {
				FeatureReference<LaneProxy> e;
				e = mutable_changed_id(mgr->current_version());
				tile->lane_refs().push_back(e);
				CLOG_DEBUG("LaneProxy::correct_tile_refs(): right_boundary add tile %d 's ref %s",tile->tile_id(),e->to_string());
				// LOG_BEGIN(INFO) << "LaneProxy::correct_tile_refs(): drivelines add tile "
				// 	<< tile->tile_id() << " 's ref " << e->to_string(); LOG_END;
				bc = true;
			}
		}
	}
	
	auto tiles = referenced_tiles();
	for (auto& tw : tiles) {
		auto tile = const_cast<TileInfoProxy*>(tw);
		if (tile && ref_tiles.count(tile->tile_id()) <= 0) {
			auto ref = tile->lane_refs().find(*id());
			if (ref && ref->is_deleted()) {
				continue;
			}
			auto did = mutable_deleted_id(mgr->current_version());
			tile->lane_refs().replace(id(), did);
			CLOG_DEBUG("LaneProxy::correct_tile_refs(): remove tile %d 's ref %s",tile->tile_id(),did->to_string());
			// LOG_BEGIN(INFO) << "LaneProxy::correct_tile_refs(): remove tile "
			// 	<< tile->tile_id() << " 's ref " << did->to_string(); LOG_END;
			bc = true;
		}
	}
	return bc;
}

bool LaneProxy::correct_content(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted()) {
		return false;
	}
	bool bc = false;
	if (_id) {
		if (!_id->is_type(feature_id_type())) {
			LOG_BEGIN(INFO) << "LaneProxy::correct_content() correct id->type:"
				<< _id->type() << " to " << feature_id_type(); LOG_END;
			_id->set_type(feature_id_type());
			bc = true;
		}
		bc |= _id->correct_content(mgr);
	}
	else {
		LOG_BEGIN(ERROR) << "LaneProxy::correct_content(): no id in element"; LOG_END;
		return false;
	}
	if (_lanes.correct_content(mgr)) {
		mark_changed();
		LOG_BEGIN(INFO) << "LaneProxy::correct_content(): correct lanes for "
			<< _id->to_string() << ", " << _lanes.size() << " left"; LOG_END;
		bc = true;
	}
	if (_lanes.remove_invalid()) {
		mark_changed();
		LOG_BEGIN(INFO) << "LanenProxy::correct_content(): remove invalid lanes for "
			<< _id->to_string() << ", " << _lanes.size() << " left"; LOG_END;
		bc = true;
	}
	if (_drivelines.remove_invalid(mgr)) {
		mark_changed();
		LOG_BEGIN(INFO) << "LaneProxy::correct_content(): remove invalid drivelines for "
			<< _id->to_string() << ", " << _drivelines.size() << " left"; LOG_END;
		bc = true;
	}
	if (_directions.remove_invalid()) {
		mark_changed();
		LOG_BEGIN(INFO) << "LaneProxy::correct_content(): remove invalid directions for "
			<< _id->to_string(); LOG_END;
		bc = true;
	}
	if (_preds.remove_invalid(mgr, true)) {
		mark_changed();
		LOG_BEGIN(INFO) << "LaneProxy::correct_content(): remove invalid preds for "
			<< _id->to_string() << ", " << _preds.size() << " left"; LOG_END;
		bc = true;
	}
	if (_succs.remove_invalid(mgr, true)) {
		mark_changed();
		LOG_BEGIN(INFO) << "LaneProxy::correct_content(): remove invalid succs for "
			<< _id->to_string() << ", " << _succs.size() << " left"; LOG_END;
		bc = true;
	}
	if (_lanes.empty()) {
		LOG_BEGIN(ERROR) << "LaneProxy::correct_content(): set deleted for "
			<< _id->to_string() << ": lanes is empty"; LOG_END;
		set_deleted();
		auto es = referenced_elems();
		for (auto& e : es) {
			auto ep = const_cast<FeatureWithIDProxyBase*>(e);
			if (ep && ep->id()) {
				ep->correct_content(mgr);
			}
		}
		return true;
	}
	auto pos = wgs84_pos();
	auto np = std::dynamic_pointer_cast<LaneProxy>(shared_from_this());
	auto ptr = std::dynamic_pointer_cast<LaneProxy>(shared_from_this());
	TileInfoPtr nti;
	if (mgr->remake_id(np, pos, nti, true)) {
		nti->lanes().push_back(ptr);
		auto tile = mgr->get_road_tile(np->id()->tileid());
		tile->lanes().replace(ptr, np);
		auto tiles = referenced_tiles();
		for (auto& tw : tiles) {
			auto tile = const_cast<TileInfoProxy*>(tw);
			if (tile) {
				tile->lane_refs().replace(ptr->id(), np->mutable_id());
				LOG_BEGIN(INFO) << "LaneProxy::correct_content(): replace tile "
					<< tile->tile_id() << " 's ref " << ptr->id()->to_string() << " to "
					<< np->id()->to_string(); LOG_END;
			}
		}
		bc = true;
	}

	bc |= correct_tile_refs(mgr);
	
	bc |= correct_version(mgr);
	
	if (!cached_is_valid()) {
		LOG_BEGIN(ERROR) << "LaneProxy::correct_content(): set deleted for "
			<< _id->to_string(); LOG_END;
		set_deleted();
		auto es = referenced_elems();
		for (auto& e : es) {
			auto ep = const_cast<FeatureWithIDProxyBase*>(e);
			if (ep && ep->id()) {
				ep->correct_content(mgr);
			}
		}
		return true;
	}
	return bc;
};

bool LaneProxy::remake_proxy(RoadGeometryManager* mgr) {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	bool bf = _lanes.remake_proxy(mgr);
	if (!_drivelines.resolve_proxy(mgr)) {
		LOG_BEGIN(INFO) << "LaneProxy::remake_proxy(): " << sid
			<< " has unresolved drivelines!"; LOG_END;
	}
	if (!_preds.resolve_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "LaneProxy::remake_proxy(): " << sid
			<< " has unresolved preds!"; LOG_END;
	}
	if (!_succs.resolve_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "LaneProxy::remake_proxy(): " << sid
			<< " has unresolved succs!"; LOG_END;
	}
	return bf;
};

bool LaneProxy::merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) {
	return FeatureMerger<LaneProxy>(policy, mgr, this, "Lane")
									.merge(_lanes, "lanes")
									.merge(_drivelines, "drivelines")
									//.merge(_function, "function")
									.merge(_type, "type")
									//.merge(_kind, "kind")
									.merge_elemental(_directions, "directions")
									//.merge(_priority, "priority")
									.merge(_length, "length")
									.merge(_seq_no, "seq_no")
									.merge(_preds, "preds")
									.merge(_succs, "succs")
									.is_conflict();
};

bool LaneExtProxy::set_function(Optional<int> f) {
	if (!is_editable()) {
		return false;
	}
	if (_function != f) {
		mark_changed();
	}
	_function = f;
	return true;
};
bool LaneExtProxy::set_priority(Optional<int> p) {
	if (!is_editable()) {
		return false;
	}
	if (_priority != p) {
		mark_changed();
	}
	_priority = p;
	return true;
};
bool LaneExtProxy::set_l_restrictions(const std::vector<int>& ls) {
	if (!is_editable()) {
		return false;
	}
	if (!vector_equal(ls, _l_restrictions)) {
		mark_changed();
	}
	_l_restrictions = ls;
	return true;
};
bool LaneExtProxy::set_r_restrictions(const std::vector<int>& rs) {
	if (!is_editable()) {
		return false;
	}
	if (!vector_equal(rs, _r_restrictions)) {
		mark_changed();
	}
	_r_restrictions = rs;
	return true;
};
bool LaneExtProxy::set_transition(Optional<int> t) {
	if (!is_editable()) {
		return false;
	}
	if (_transition != t) {
		mark_changed();
	}
	_transition = t;
	return true;
};

bool LaneExtProxy::to_message(google::protobuf::Message* msg_base) const {
	LaneProxy::to_message(msg_base);
	auto msg = static_cast<RoadPB::Lane*>(msg_base);
	if (_function) {
		msg->set_function(*_function);
	}
	if (_priority) {
		msg->set_priority(*_priority);
	}
	if (_transition) {
		msg->set_transition(*_transition);
	}
	for (size_t i = 0; i < _l_restrictions.size(); ++i) {
		msg->add_l_restrictions(_l_restrictions[i]);
	}
	for (size_t i = 0; i < _r_restrictions.size(); ++i) {
		msg->add_r_restrictions(_r_restrictions[i]);
	}
	return true;
};

bool LaneExtProxy::from_message(const google::protobuf::Message* msg_base) {
	LaneProxy::from_message(msg_base);
	auto msg = static_cast<const RoadPB::Lane*>(msg_base);
	if (msg->has_function()) {
		_function = msg->function();
	}
	else {
		_function.reset();
	}
	if (msg->has_priority()) {
		_priority = msg->priority();
	}
	else {
		_priority.reset();
	}
	if (msg->has_transition()) {
		_transition = msg->transition();
	}
	else {
		_transition.reset();
	}
	_l_restrictions.clear();
	_l_restrictions.reserve(msg->l_restrictions_size());
	for (int i = 0; i < msg->l_restrictions_size(); ++i) {
		_l_restrictions.push_back(msg->l_restrictions(i));
	}
	_r_restrictions.clear();
	_r_restrictions.reserve(msg->r_restrictions_size());
	for (int i = 0; i < msg->r_restrictions_size(); ++i) {
		_r_restrictions.push_back(msg->r_restrictions(i));
	}
	clear_changed();
	return true;
};

bool LaneExtProxy::merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) {
	bool bf = LaneProxy::merge(policy, mgr);
	return FeatureMerger<LaneExtProxy>(policy, mgr, this, "LaneExt")
									.merge(_function, "function")
									.merge(_priority, "priority")
									.merge(_transition, "transition")
									.merge(_l_restrictions, "l_restrictions")
									.merge(_r_restrictions, "r_restrictions")
									.is_conflict()
									|| bf;
};

FeatureReference<LinkProxy>* LaneGroupProxy::mutable_link_id() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_link_id;
};
bool LaneGroupProxy::set_types(const std::vector<int>& t) {
	if (!is_editable()) {
		return false;
	}
	if (vector_compare(_types, t) != 0) {
		mark_changed();
	}
	_types = t;
	return true;
};
FeatureMemberVector<RoadBoundaryProxy>* LaneGroupProxy::mutable_left_boundarys() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_left_boundarys;
};
FeatureMemberVector<RoadBoundaryProxy>* LaneGroupProxy::mutable_right_boundarys() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_right_boundarys;
};
FeatureMemberVector<LaneProxy>* LaneGroupProxy::mutable_lanes() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_lanes;
};
FeatureReferenceVector<LaneGroupProxy>* LaneGroupProxy::mutable_preds() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_preds;
};
FeatureReferenceVector<LaneGroupProxy>* LaneGroupProxy::mutable_succs() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_succs;
};

Vector3D LaneGroupProxy::wgs84_pos() const {
for (size_t i = 0; i < _lanes.size(); ++i) {
		auto& l = _lanes[i];
		if (l && l.proxy()) {
			return l.proxy()->wgs84_pos();
		}
	}
	return Vector3D(0, 0, 0);
};
const Vector3D& LaneGroupProxy::global_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	for (size_t i = 0; i < _lanes.size(); ++i) {
		auto& l = _lanes[i];
		if (l && l.proxy()) {
			return l.proxy()->global_pos(mgr);
		}
	}
	static Vector3D dummy(0, 0, 0);
	return dummy;
};

const Vector3D& LaneGroupProxy::global_left_start_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	for (size_t i = 0; i < _lanes.size(); ++i) {
		auto& l = _lanes[i];
		if (l && l.proxy()) {
			return l.proxy()->global_left_start_pos(mgr);
		}
	}
	static Vector3D dummy(0, 0, 0);
	return dummy;
};

const Vector3D& LaneGroupProxy::global_right_start_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	for (size_t i = _lanes.size(); i > 0; --i) {
		auto& l = _lanes[i - 1];
		if (l && l.proxy()) {
			return l.proxy()->global_right_start_pos(mgr);
		}
	}
	static Vector3D dummy(0, 0, 0);
	return dummy;
};

const Vector3D& LaneGroupProxy::global_right_end_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	for (size_t i = _lanes.size(); i > 0; --i) {
		auto& l = _lanes[i - 1];
		if (l && l.proxy()) {
			return l.proxy()->global_right_end_pos(mgr);
		}
	}
	static Vector3D dummy(0, 0, 0);
	return dummy;
};

const Vector3D& LaneGroupProxy::global_left_end_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	for (size_t i = 0; i < _lanes.size(); ++i) {
		auto& l = _lanes[i];
		if (l && l.proxy()) {
			return l.proxy()->global_left_end_pos(mgr);
		}
	}
	static Vector3D dummy(0, 0, 0);
	return dummy;
};

bool LaneGroupProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::LaneGroup*>(msg_base);
	msg->Clear();
	id_to_message(msg);
	if (_link_id) {
		_link_id->to_message(msg->mutable_link_id());
	}
	for (size_t i = 0; i < _types.size(); ++i) {
		msg->add_types(_types[i]);
	}
	_left_boundarys.to_message(msg->mutable_left_boundarys());
	_right_boundarys.to_message(msg->mutable_right_boundarys());
	_lanes.to_message(msg->mutable_lanes());
	_preds.to_message(msg->mutable_preds());
	_succs.to_message(msg->mutable_succs());
	return true;
};

bool LaneGroupProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::LaneGroup*>(msg_base);
	id_from_message(msg);
	if (msg->has_link_id()) {
		_link_id.create(this)->from_message(&msg->link_id());
	}
	else {
		_link_id.reset();
	}
	_types.clear();
	_types.reserve(msg->types_size());
	for (int i = 0; i < msg->types_size(); ++i) {
		_types.push_back(msg->types(i));
	}
	_left_boundarys.from_message(msg->left_boundarys());
	_right_boundarys.from_message(msg->right_boundarys());
	_lanes.from_message(msg->lanes());
	_preds.from_message(msg->preds());
	_succs.from_message(msg->succs());
	clear_changed();
	return true;
};

bool LaneGroupProxy::is_valid() const {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	if (!is_message_valid<LaneGroupProxy>()) {
		LOG_BEGIN(ERROR) << "LaneGroupProxy::is_valid(): failed for " << sid
			<< " : is_message_valid() failed!"; LOG_END;
		return false;
	}
	if (!is_editable() || is_deleted()) {
		//LOG_BEGIN(DEBUG) << "LaneGroupProxy::is_valid() skip for " << sid; LOG_END;
		return true;
	}
	if (!is_ref_valid()) {
		LOG_BEGIN(ERROR) << "LaneGroupProxy::is_valid() failed for " << sid
			<< " : has " << owner_count() << " owners"; LOG_END;
		return false;
	}
	if (!_preds.is_valid(true) || !_succs.is_valid(true)) {
		LOG_BEGIN(ERROR) << "LaneGroupProxy::is_valid() failed for " << sid
			<< " : has " << _preds.size() << " preds, "
			<< _succs.size() << " succs"; LOG_END;
		return false;
	}
	if (_lanes.empty()) {
		LOG_BEGIN(ERROR) << "LaneGroupProxy::is_valid() failed for " << sid
			<< " : has empty lanes!"; LOG_END;
		return false;
	}

	if (!_lanes.is_valid(true)) {
		LOG_BEGIN(ERROR) << "LaneGroupProxy::is_valid() failed for " << sid
			<< " : has invalid lanes " << _lanes.size(); LOG_END;
		return false;
	}
	if (!_left_boundarys.is_valid(true)) {
		LOG_BEGIN(ERROR) << "LaneGroupProxy::is_valid() failed for " << sid
			<< " : has invalid left_boundarys " << _left_boundarys.size(); LOG_END;
		return false;
	}
	if (!_right_boundarys.is_valid(true)) {
		LOG_BEGIN(INFO) << "LaneGroupProxy::is_valid() failed for " << sid
			<< " : has invalid right_boundarys " << _right_boundarys.size(); LOG_END;
		return false;
	}
	return true;
};

bool LaneGroupProxy::judge_editable(RoadGeometryManager* mgr) {
	_editable = true;
	
	std::map<int, double> tid2lens;
	std::set<const LaneBoundaryProxy*> lbs;
	for (auto& lane : _lanes) {
		if (!lane || !lane.proxy()) {
			continue;
		}
		for (auto& sec : lane.proxy()->lanes()) {
			if (sec->left_boundary() && sec->left_boundary()->bound_id()) {
				if (!sec->left_boundary()->bound_id().proxy()) {
					if (!mgr->is_tile_15_current_downloaded(sec->left_boundary()->bound_id()->tileid())) {
						_editable = false;
						return false;
					}
					continue;
				}
				if (lbs.find(sec->left_boundary()->bound_id().proxy().get()) == lbs.end()) {
					lbs.insert(sec->left_boundary()->bound_id().proxy().get());
					if (sec->left_boundary()->bound_id().proxy()->geom()) {
						sec->left_boundary()->bound_id().proxy()->geom()->calc_length_in_tiles(mgr, tid2lens);
					}
				}
			}
			if (sec->right_boundary() && sec->right_boundary()->bound_id()) {
				if (!sec->right_boundary()->bound_id().proxy()) {
					if (!mgr->is_tile_15_current_downloaded(sec->right_boundary()->bound_id()->tileid())) {
						_editable = false;
						return false;
					}
					continue;
				}
				if (lbs.find(sec->right_boundary()->bound_id().proxy().get()) == lbs.end()) {
					lbs.insert(sec->right_boundary()->bound_id().proxy().get());
					if (sec->right_boundary()->bound_id().proxy()->geom()) {
						sec->right_boundary()->bound_id().proxy()->geom()->calc_length_in_tiles(mgr, tid2lens);
					}
				}
			}
		}
	}
	for (auto& rb : _left_boundarys) {
		if (!rb || !rb.proxy()) {
			if (!mgr->is_tile_15_current_downloaded(rb->tileid())) {
				_editable = false;
				return false;
			}
			continue;
		}
		if (rb.proxy()->geom()) {
			rb.proxy()->geom()->calc_length_in_tiles(mgr, tid2lens);
		}
	}
	for (auto& rb : _right_boundarys) {
		if (!rb || !rb.proxy()) {
			if (!mgr->is_tile_15_current_downloaded(rb->tileid())) {
				_editable = false;
				return false;
			}
			continue;
		}
		if (rb.proxy()->geom()) {
			rb.proxy()->geom()->calc_length_in_tiles(mgr,  tid2lens);
		}
	}	
	_editable = mgr->is_editable_in_tiles(tid2lens);
	return _editable;
};

bool LaneGroupProxy::correct_tile_refs(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted() || !_id) {
		return false;
	}
	bool bc = false;
	std::set<int> ref_tiles;
	std::vector<const TileInfoProxy*> tiles;
	for (auto& l : _lanes) {
		if (!l || !l.proxy()) {
			continue;
		}
		auto ts = l.proxy()->referenced_tiles();
		tiles.insert(tiles.end(), ts.begin(), ts.end());
	}
	for (auto& l : _left_boundarys) {
		if (!l || !l.proxy()) {
			continue;
		}
		auto ts = l.proxy()->referenced_tiles();
		tiles.insert(tiles.end(), ts.begin(), ts.end());
	}
	for (auto& l : _right_boundarys) {
		if (!l || !l.proxy()) {
			continue;
		}
		auto ts = l.proxy()->referenced_tiles();
		tiles.insert(tiles.end(), ts.begin(), ts.end());
	}
	for (auto t : tiles) {
		if (ref_tiles.count(t->tile_id()) > 0) {
			continue;
		}
		ref_tiles.insert(t->tile_id());
		auto tile = mgr->get_road_tile(t->tile_id());
		if (add_missed_ref_tiles(tile.get())) {
			FeatureReference<LaneGroupProxy> e;
			e = mutable_changed_id(mgr->current_version());
			tile->lane_group_refs().push_back(e);
			LOG_DEBUG("LaneGroupProxy::correct_tile_refs(): add tile %d 's ref %s",tile->tile_id(), e->to_string()); 
			// LOG_BEGIN(INFO) << "LaneGroupProxy::correct_tile_refs(): add tile "
			// 	<< tile->tile_id() << " 's ref " << e->to_string(); LOG_END;
			bc = true;
		}
	}	
	
	tiles = referenced_tiles();
	for (auto& tw : tiles) {
		auto tile = const_cast<TileInfoProxy*>(tw);
		if (tile && ref_tiles.count(tile->tile_id()) <= 0) {
			auto ref = tile->lane_group_refs().find(*id());
			if (ref && ref->is_deleted()) {
				continue;
			}
			auto did = mutable_deleted_id(mgr->current_version());
			tile->lane_group_refs().replace(id(), did);
			LOG_DEBUG("LaneGroupProxy::correct_tile_refs(): remove tile %d 's ref %s",tile->tile_id(), did->to_string()); 
			// LOG_BEGIN(INFO) << "LaneGroupProxy::correct_tile_refs(): remove tile "
			// 	<< tile->tile_id() << " 's ref " << did->to_string(); LOG_END;
			bc = true;
		}
	}
	return bc;
}

bool LaneGroupProxy::correct_content(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted()) {
		return false;
	}
	bool bc = false;
	if (_id) {
		if (!_id->is_type(feature_id_type())) {
			LOG_BEGIN(INFO) << "LaneGroupProxy::correct_content() correct id->type:"
				<< _id->type() << " to " << feature_id_type(); LOG_END;
			_id->set_type(feature_id_type());
			bc = true;
		}
		bc |= _id->correct_content(mgr);
	}
	else {
		LOG_BEGIN(ERROR) << "LaneGroupProxy::correct_content(): no id in element"; LOG_END;
		return false;
	}
	if (_lanes.remove_invalid(true)) {
		mark_changed();
		LOG_BEGIN(INFO) << "LaneGroupProxy::correct_content(): correct lanes for "
			<< _id->to_string() << ", " << _lanes.size() << " left"; LOG_END;
		bc = true;
	}
	if (_left_boundarys.remove_invalid(true)) {
		mark_changed();
		LOG_BEGIN(INFO) << "LaneGroupProxy::correct_content(): remove invalid left_boundarys for "
			<< _id->to_string() << ", " << _left_boundarys.size() << " left"; LOG_END;
		bc = true;
	}
	if (_right_boundarys.remove_invalid(true)) {
		mark_changed();
		LOG_BEGIN(INFO) << "LaneGroupProxy::correct_content(): remove invalid right_boundarys for "
			<< _id->to_string() << ", " << _right_boundarys.size() << " left"; LOG_END;
		bc = true;
	}
	if (_preds.remove_invalid(mgr, true)) {
		mark_changed();
		LOG_BEGIN(INFO) << "LaneProxy::correct_content(): remove invalid preds for "
			<< _id->to_string() << ", " << _preds.size() << " left"; LOG_END;
		bc = true;
	}
	if (_succs.remove_invalid(mgr, true)) {
		mark_changed();
		LOG_BEGIN(INFO) << "LaneProxy::correct_content(): remove invalid succs for "
			<< _id->to_string() << ", " << _succs.size() << " left"; LOG_END;
		bc = true;
	}
	if (_lanes.empty()) {
		LOG_BEGIN(ERROR) << "LaneGroupProxy::correct_content(): set deleted for "
			<< _id->to_string() << ": lanes is empty"; LOG_END;
		set_deleted();
		auto es = referenced_elems();
		for (auto& e : es) {
			auto ep = const_cast<FeatureWithIDProxyBase*>(e);
			if (ep && ep->id()) {
				ep->correct_content(mgr);
			}
		}
		return true;
	}
	auto pos = wgs84_pos();
	auto np = std::dynamic_pointer_cast<LaneGroupProxy>(shared_from_this());
	auto ptr = std::dynamic_pointer_cast<LaneGroupProxy>(shared_from_this());
	TileInfoPtr nti;
	if (mgr->remake_id(np, pos, nti, true)) {
		nti->lane_groups().push_back(ptr);
		auto tile = mgr->get_road_tile(np->id()->tileid());
		tile->lane_groups().replace(ptr, np);
		auto tiles = referenced_tiles();
		for (auto& tw : tiles) {
			auto tile = const_cast<TileInfoProxy*>(tw);
			if (tile) {
				tile->lane_group_refs().replace(ptr->id(), np->mutable_id());
				LOG_BEGIN(INFO) << "LaneGroupProxy::correct_content(): replace tile "
					<< tile->tile_id() << " 's ref " << ptr->id()->to_string() << " to "
					<< np->id()->to_string(); LOG_END;
			}
		}
		bc = true;
	}

	bc |= correct_tile_refs(mgr);

	bc |= correct_version(mgr);

	if (!cached_is_valid()) {
		LOG_BEGIN(ERROR) << "LaneGroupProxy::correct_content(): set deleted for "
			<< _id->to_string(); LOG_END;
		set_deleted();
		auto es = referenced_elems();
		for (auto& e : es) {
			auto ep = const_cast<FeatureWithIDProxyBase*>(e);
			if (ep && ep->id()) {
				ep->correct_content(mgr);
			}
		}
		return true;
	}
	return bc;
};

bool LaneGroupProxy::remake_proxy(RoadGeometryManager* mgr) {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	bool bf = true;
	if (_link_id) {
		auto p = mgr->get_link(*_link_id.id());
		if (p) {
			_link_id.set(p->id());
		}
		else {
			LOG_BEGIN(INFO) << "LaneGroupProxy::remake_proxy(): " << sid
				<< " has unresolved link_id " << _link_id->to_string();
			LOG_END;
			bf = false;
		}
	}
	if (!_left_boundarys.resolve_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "LaneGroupProxy::remake_proxy(): " << sid
			<< " has unresolved left_boundary!"; LOG_END;
	}
	if (!_right_boundarys.resolve_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "LaneGroupProxy::remake_proxy(): " << sid
			<< " has unresolved right_boundary!"; LOG_END;
	}
	if (!_lanes.resolve_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "LaneGroupProxy::remake_proxy(): " << sid
			<< " has unresolved lanes!"; LOG_END;
	}
	if (!_preds.resolve_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "LaneGroupProxy::remake_proxy(): " << sid
			<< " has unresolved preds!"; LOG_END;
	}
	if (!_succs.resolve_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "LaneGroupProxy::remake_proxy(): " << sid
			<< " has unresolved succs!"; LOG_END;
	}
	return bf;
};

bool LaneGroupProxy::merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) {
	return FeatureMerger<LaneGroupProxy>(policy, mgr, this, "LaneGroup")		
										.merge(_link_id, "link_id")
										.merge(_types, "types")
										.merge(_left_boundarys, "left_boundarys")
										.merge(_right_boundarys, "right_boundarys")
										.merge(_lanes, "lanes")
										.merge(_preds, "preds")
										.merge(_succs, "succs")
										.is_conflict();
};

bool ImpassableAreaProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::ImpassableArea*>(msg_base);
	if (msg->has_geom()) {
		_geom.reset(FeatureProxyBase::create_proxy<PolygonProxy>(this));
		_geom->from_message(&msg->geom());
	}
	else {
		_geom.reset();
	}
	if (msg->has_type()) {
		_type = msg->type();
	}
	else {
		_type.reset();
	}
	if (msg->has_kind()) {
		_kind = msg->kind();
	}
	else {
		_kind.reset();
	}
	clear_changed();
	return true;
};

bool ImpassableAreaProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::ImpassableArea*>(msg_base);
	msg->Clear();
	if (_geom) {
		_geom->to_message(msg->mutable_geom());
	}
	if (_type) {
		msg->set_type(*_type);
	}
	if (_kind) {
		msg->set_kind(*_kind);
	}
	return true;
};

bool ImpassableAreaProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
									const std::shared_ptr<const FeatureProxyBase>& base,
									const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const ImpassableAreaProxy>(remote);
	auto o = std::dynamic_pointer_cast<const ImpassableAreaProxy>(base);
	return ProxyMerger<ImpassableAreaProxy>(policy, mgr, this, "ImpassableProxy", r, o)
											.merge(_geom, "geom")
											.merge(_type, "type")
											.merge(_kind, "kind")
											.is_conflict();
}

bool ImpassableAreaProxy::is_valid() const {
	if (!_geom || !_geom->cached_is_valid()) {
		return false;
	}
	return true;
};

int ImpassableAreaProxy::compare(const ImpassableAreaProxy& rhs) const {
	int ret = value_compare(_type, rhs._type);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_kind, rhs._kind);
	if (ret != 0) {
		return ret;
	}
	return proxy_compare(_geom, rhs._geom);
};

PolygonProxy* ImpassableAreaProxy::mutable_geom() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_geom) {
		_geom.reset(FeatureProxyBase::create_proxy<PolygonProxy>(this));
	}
	return _geom.get();
};

bool ImpassableAreaProxy::clear_geom() {
	if (!is_editable()) {
		return false;
	}
	if (_geom) {
		mark_changed();
	}
	_geom.reset();
	return true;
};

bool ImpassableAreaProxy::set_type(Optional<int> t) {
	if (!is_editable()) {
		return false;
	}
	if (_type != t) {
		mark_changed();
	}
	_type = t;
	return true;
};

bool ImpassableAreaProxy::set_kind(Optional<int> k) {
	if (!is_editable()) {
		return false;
	}
	if (_kind != k) {
		mark_changed();
	}
	_kind = k;
	return true;
};

FeatureReference<NodeProxy>* JunctionProxy::mutable_node_id() {
	if (!is_editable()) {
		return nullptr;
	}	
	return &_node_id;
};
SharedProxyVector<ImpassableAreaProxy>* JunctionProxy::mutable_areas() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_areas;
};
FeatureMemberVector<RoadBoundaryProxy>* JunctionProxy::mutable_boundarys() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_boundarys;
};
FeatureMemberVector<LaneGroupProxy>* JunctionProxy::mutable_conn_groups() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_conn_groups;
};
FeatureReferenceVector<LaneGroupProxy>* JunctionProxy::mutable_in_groups() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_in_groups;
};
FeatureReferenceVector<LaneGroupProxy>* JunctionProxy::mutable_out_groups() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_out_groups;
};

Vector3D JunctionProxy::wgs84_pos() const {
	if (_boundarys.empty() && _out_groups.empty()) {
		return Vector3D(0, 0, 0);
	}
	int cnt = 0;
	Vector3D pt(0, 0, 0);
	for (auto g : _boundarys) {
		if (g && g.proxy()) {
			auto p = g.proxy()->wgs84_pos();
			pt += p;
			cnt++;
		}
	}
	if (cnt > 0) {
		pt /= cnt;
		return pt;
	}
	for (auto g : _out_groups) {
		if (g && g.proxy()) {
			auto p = g.proxy()->wgs84_pos();
			return p;
		}
	}
	return Vector3D(0, 0, 0);
};

const Vector3D& JunctionProxy::global_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	static Vector3D dummy(0, 0, 0);
	auto pos = wgs84_pos();
	if (pos.Length() > 1e-4 && mgr->global_point_from_WGS84(pos)) {
		_global_pos = pos;
		return _global_pos;
	}
	return dummy;
};

bool JunctionProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::Junction*>(msg_base);
	msg->Clear();
	id_to_message(msg);
	if (_node_id) {
		_node_id->to_message(msg->mutable_node_id());
	}
	_areas.to_message(msg->mutable_areas());
	_boundarys.to_message(msg->mutable_boundarys());
	_conn_groups.to_message(msg->mutable_conn_groups());
	_in_groups.to_message(msg->mutable_in_groups());
	_out_groups.to_message(msg->mutable_out_groups());
	return true;
};

bool JunctionProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::Junction*>(msg_base);
	id_from_message(msg);
	if (msg->has_node_id()) {
		_node_id.create(this)->from_message(&msg->node_id());
	}
	else {
		_node_id.reset();
	}
	_areas.from_message(msg->areas());
	_boundarys.from_message(msg->boundarys());
	_conn_groups.from_message(msg->conn_groups());
	_in_groups.from_message(msg->in_groups());
	_out_groups.from_message(msg->out_groups());
	clear_changed();
	return true;
};

bool JunctionProxy::is_valid() const {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	if (!is_message_valid<JunctionProxy>()) {
		LOG_BEGIN(ERROR) << "JunctionProxy::is_valid(): failed for " << sid
			<< " : is_message_valid() failed!"; LOG_END;
		return false;
	}
	if (!is_editable() || is_deleted()) {
		//LOG_BEGIN(DEBUG) << "JunctionProxy::is_valid() skip for " << sid; LOG_END;
		return true;
	}
	if (!is_ref_valid()) {
		LOG_BEGIN(ERROR) << "JunctionProxy::is_valid() failed for " << sid
			<< " : has " << owner_count() << " owners"; LOG_END;
		return false;
	}
	if (!_in_groups.is_valid(true) || !_out_groups.is_valid(true)) {
		LOG_BEGIN(ERROR) << "JunctionProxy::is_valid() failed for " << sid
			<< " : has " << _in_groups.size() << " ingroups, "
			<< _out_groups.size() << " outgroups"; LOG_END;
		return false;
	}
	if (_boundarys.empty() && _in_groups.empty() && _out_groups.empty() && _conn_groups.empty()) {
		LOG_BEGIN(ERROR) << "JunctionProxy::is_valid() failed for " << sid
			<< " : empty junction!"; LOG_END;
		return false;
	}

	if (!_conn_groups.is_valid(true)) {
		LOG_BEGIN(ERROR) << "JunctionProxy::is_valid() failed for " << sid
			<< " : has invalid lanes " << _conn_groups.size(); LOG_END;
		return false;
	}
	if (!_areas.is_valid()) {
		LOG_BEGIN(ERROR) << "JunctionProxy::is_valid() failed for " << sid
			<< " : has invalid areas " << _areas.size(); LOG_END;
		return false;
	}
	if (!_in_groups.is_valid(true) || !_out_groups.is_valid(true)) {
		LOG_BEGIN(INFO) << "JunctionProxy::is_valid() failed for " << sid
			<< " : has invalid in/out_groups " << _in_groups.size() << '/' 
			<< _out_groups.size(); LOG_END;
		return false;
	}
	return true;
};

bool JunctionProxy::judge_editable(RoadGeometryManager* mgr) {
	_editable = true;
	if (!_boundarys.empty()) {
		return FeatureWithIDProxyBase::judge_editable(mgr);
	}
	std::map<int, double> tid2lens;
	for (auto& rb : _boundarys) {
		if (!rb || !rb.proxy()) {
			if (!mgr->is_tile_15_current_downloaded(rb->tileid())) {
				_editable = false;
				return false;
			}
			continue;
		}
		if (rb.proxy()->geom()) {
			rb.proxy()->geom()->calc_length_in_tiles(mgr, tid2lens);
		}
	}		
	_editable = mgr->is_editable_in_tiles(tid2lens);
	return _editable;
};

bool JunctionProxy::correct_tile_refs(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted() || !_id) {
		return false;
	}
	bool bc = false;
	std::set<int> ref_tiles;
	std::vector<const TileInfoProxy*> tiles;
	for (auto& l : _boundarys) {
		if (!l || !l.proxy()) {
			continue;
		}
		auto ts = l.proxy()->referenced_tiles();
		tiles.insert(tiles.end(), ts.begin(), ts.end());
	}
	for (auto& l : _conn_groups) {
		if (!l || !l.proxy()) {
			continue;
		}
		auto ts = l.proxy()->referenced_tiles();
		tiles.insert(tiles.end(), ts.begin(), ts.end());
	}
	for (auto t : tiles) {
		if (ref_tiles.count(t->tile_id()) > 0) {
			continue;
		}
		ref_tiles.insert(t->tile_id());
		auto tile = mgr->get_road_tile(t->tile_id());
		if (add_missed_ref_tiles(tile.get())) {
			FeatureReference<JunctionProxy> e;
			e = mutable_changed_id(mgr->current_version());
			tile->junction_refs().push_back(e);
			LOG_BEGIN(INFO) << "JunctionProxy::correct_content(): add tile "
				<< tile->tile_id() << " 's ref " << e->to_string(); LOG_END;
			bc = true;
		}
	}

	tiles = referenced_tiles();
	for (auto& tw : tiles) {
		auto tile = const_cast<TileInfoProxy*>(tw);
		if (tile && ref_tiles.count(tile->tile_id()) <= 0) {
			auto ref = tile->junction_refs().find(*id());
			if (ref && ref->is_deleted()) {
				continue;
			}
			auto did = mutable_deleted_id(mgr->current_version());
			tile->junction_refs().replace(id(), did);
			LOG_BEGIN(INFO) << "JunctionProxy::correct_content(): remove tile "
				<< tile->tile_id() << " 's ref " << did->to_string(); LOG_END;
			bc = true;
		}
	}
	return bc;
};

bool JunctionProxy::correct_content(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted()) {
		return false;
	}
	bool bc = false;
	if (_id) {
		if (!_id->is_type(feature_id_type())) {
			LOG_BEGIN(INFO) << "JunctionProxy::correct_content() correct id->type:"
				<< _id->type() << " to " << feature_id_type(); LOG_END;
			_id->set_type(feature_id_type());
			bc = true;
		}
		bc |= _id->correct_content(mgr);
	}
	else {
		LOG_BEGIN(ERROR) << "JunctionProxy::correct_content(): no id in element"; LOG_END;
		return false;
	}
	if (_boundarys.remove_invalid(true)) {
		mark_changed();
		LOG_BEGIN(INFO) << "JunctionProxy::correct_content(): remove invalid boundarys for "
			<< _id->to_string() << ", " << _boundarys.size() << " left"; LOG_END;
		bc = true;
	}
	if (_conn_groups.remove_invalid(true)) {
		mark_changed();
		LOG_BEGIN(INFO) << "JunctionProxy::correct_content(): remove invalid conn_groups for "
			<< _id->to_string() << ", " << _conn_groups.size() << " left"; LOG_END;
		bc = true;
	}
	if (_in_groups.remove_invalid(mgr, true)) {
		mark_changed();
		LOG_BEGIN(INFO) << "JunctionProxy::correct_content(): remove invalid in_groups for "
			<< _id->to_string() << ", " << _in_groups.size() << " left"; LOG_END;
		bc = true;
	}
	if (_out_groups.remove_invalid(mgr, true)) {
		mark_changed();
		LOG_BEGIN(INFO) << "JunctionProxy::correct_content(): remove invalid out_groups for "
			<< _id->to_string() << ", " << _out_groups.size() << " left"; LOG_END;
		bc = true;
	}
	
	auto pos = wgs84_pos();
	auto np = std::dynamic_pointer_cast<JunctionProxy>(shared_from_this());
	auto ptr = std::dynamic_pointer_cast<JunctionProxy>(shared_from_this());
	TileInfoPtr nti;
	if (mgr->remake_id(np, pos, nti, true)) {
		nti->junctions().push_back(ptr);
		auto tile = mgr->get_road_tile(np->id()->tileid());
		tile->junctions().replace(ptr, np);
		auto tiles = referenced_tiles();
		for (auto& tw : tiles) {
			auto tile = const_cast<TileInfoProxy*>(tw);
			if (tile) {
				tile->junction_refs().replace(ptr->id(), np->mutable_id());
				LOG_BEGIN(INFO) << "JunctionProxy::correct_content(): replace tile "
					<< tile->tile_id() << " 's ref " << ptr->id()->to_string() << " to "
					<< np->id()->to_string(); LOG_END;
			}
		}
		bc = true;
	}

	bc |= correct_tile_refs(mgr);

	bc |= correct_version(mgr);

	if (!cached_is_valid()) {
		LOG_BEGIN(ERROR) << "JunctionProxy::correct_content(): set deleted for "
			<< _id->to_string(); LOG_END;
		set_deleted();
		auto es = referenced_elems();
		for (auto& e : es) {
			auto ep = const_cast<FeatureWithIDProxyBase*>(e);
			if (ep && ep->id()) {
				ep->correct_content(mgr);
			}
		}
		return true;
	}
	return bc;
};

bool JunctionProxy::remake_proxy(RoadGeometryManager* mgr) {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	bool bf = true;
	if (_node_id) {
		auto p = mgr->get_node(*_node_id.id());
		if (p) {
			_node_id.set(p->id());
		}
		else {
			LOG_BEGIN(INFO) << "JunctionProxy::remake_proxy(): " << sid
				<< " has unresolved node_id " << _node_id->to_string();
			LOG_END;
			bf = false;
		}
	}
	if (!_boundarys.resolve_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "JunctionProxy::remake_proxy(): " << sid
			<< " has unresolved boundarys!"; LOG_END;
	}
	if (!_conn_groups.resolve_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "JunctionProxy::remake_proxy(): " << sid
			<< " has unresolved conn_groups!"; LOG_END;
	}
	if (!_in_groups.resolve_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "JunctionProxy::remake_proxy(): " << sid
			<< " has unresolved in_groups!"; LOG_END;
	}
	if (!_out_groups.resolve_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "JunctionProxy::remake_proxy(): " << sid
			<< " has unresolved out_groups!"; LOG_END;
	}
	return bf;
};

bool JunctionProxy::merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) {
	return FeatureMerger<JunctionProxy>(policy, mgr, this, "Junction")
										.merge(_node_id, "node_id")
										.merge_elemental(_areas, "areas")
										.merge(_boundarys, "boundarys")
										.merge(_conn_groups, "lanes")
										.merge(_in_groups, "in_groups")
										.merge(_out_groups, "out_groups")
										.is_conflict();
};

};
#include "position_proxy.h"
#include "data_access_engine.h"
#include "manager/road_geometry_mgr.h"
#include "lane_proxy.h"
#include "tile_proxy.h"
#include "feature_merger.h"

namespace data_access_engine {

PolygonProxy* PositionObjectProxy::mutable_border() {
    if (!is_editable()) {
        return nullptr;
    }
    if (!_border) {
        _border.reset(FeatureProxyBase::create_proxy<PolygonProxy>(this));
    }
    return _border.get();
};
bool PositionObjectProxy::clear_border() {
    if (!is_editable()) {
        return false;
    }
    if (_border) {
        mark_changed();
    }
    _border.reset();
    return true;
};

CircleProxy* PositionObjectProxy::mutable_circle() {
    if (!is_editable()) {
        return nullptr;
    }
    if (!_circle) {
        _circle.reset(FeatureProxyBase::create_proxy<CircleProxy>(this));
    }
    return _circle.get();
};
bool PositionObjectProxy::clear_circle() {
    if (!is_editable()) {
        return false;
    }
    if (_circle) {
        mark_changed();
    }
    _circle.reset();
    return true;
};
    
PolylineProxy* PositionObjectProxy::mutable_pole() {
    if (!is_editable()) {
        return nullptr;
    }
    if (!_pole) {
        _pole.reset(FeatureProxyBase::create_proxy<PolylineProxy>(this));
    }
    return _pole.get();
};
bool PositionObjectProxy::clear_pole() {
    if (!is_editable()) {
        return false;
    }
    if (_pole) {
        mark_changed();
    }
    _pole.reset();
    return true;
};
    
bool PositionObjectProxy::set_type(Optional<int> t) {
    if (!is_editable()) {
        return false;
    }
    if (_type != t) {
        mark_changed();
    }
	_type = t;
    return true;
};

bool PositionObjectProxy::set_sub_type(Optional<int> t)
{
	if (!is_editable()) {
		return false;
	}
	if (_sub_type != t) {
		mark_changed();
	}
	_sub_type = t;
	return true;
}
bool PositionObjectProxy::set_content(const Optional<std::string>& c) {
    if (!is_editable()) {
        return false;
    }
    if (_content != c) {
        mark_changed();
    }
    _content = c;
    return true;
};

FeatureReferenceVector<LaneGroupProxy>* PositionObjectProxy::mutable_lane_groups() {
    if (!is_editable()) {
        return nullptr;
    }
    return &_lane_groups;
};

bool PositionObjectProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::PositionObject*>(msg_base);
	msg->Clear();
	id_to_message(msg);
	if (_id->tileid() == 329569862 && _id->id() == 16265757605902367830)
		int lk = 0;
	if (_border) {
		_border->to_message(msg->mutable_border());
	}
	if (_circle) {
		_circle->to_message(msg->mutable_circle());
	}
	if (_pole) {
		_pole->to_message(msg->mutable_pole());
	}
	if (_type) {
		msg->set_type(*_type);
	}
	if (_sub_type)
	{
		msg->set_subtype(*_sub_type);
	}
	if (_content) {
		msg->set_content(*_content);
	}
	_lane_groups.to_message(msg->mutable_lane_groups());
	return true;
};

bool PositionObjectProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::PositionObject*>(msg_base);
	id_from_message(msg);
	if (msg->has_border()) {
		_border.reset(FeatureProxyBase::create_proxy<PolygonProxy>(this));
		_border->from_message(&msg->border());
	}
	else {
		_border.reset();
	}
	if (msg->has_circle()) {
		_circle.reset(FeatureProxyBase::create_proxy<CircleProxy>(this));
		_circle->from_message(&msg->circle());
	}
	else {
		_circle.reset();
	}
	if (msg->has_pole()) {
		_pole.reset(FeatureProxyBase::create_proxy<PolylineProxy>(this));
		_pole->from_message(&msg->pole());
	}
	else {
		_pole.reset();
	}
	if (msg->has_type()) {
		_type = msg->type();
	}
	else {
		_type.reset();
	}
	if (msg->has_subtype()) {
		_sub_type = msg->subtype();
	}
	else {
		_sub_type.reset();
	}
	if (msg->has_content()) {
		_content = msg->content();
	}
	else {
		_content.reset();
	}
	_lane_groups.from_message(msg->lane_groups());
	clear_changed();
	return true;
};

bool PositionObjectProxy::is_valid() const {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	if (!is_message_valid<PositionObjectProxy>()) {
		LOG_BEGIN(ERROR) << "PositionObjectProxy::is_valid() failed for " << sid
			<< " : is_message_valid() failed!"; LOG_END;
		return false;
	}
	if (!is_editable() || is_deleted()) {
		//LOG_BEGIN(DEBUG) << "PositionObjectProxy::is_valid() skip for " << sid; LOG_END;
		return true;
	}
	if (!is_ref_valid()) {
		LOG_BEGIN(ERROR) << "PositionObjectProxy::is_valid() failed for " << sid
			<< " : has " << owner_count() << " owners"; LOG_END;
		if (owner_count() >= 2)
		{
			for (auto oe : _ref_record->owner_elems()) {
				if (oe == nullptr) {
					continue;
				}
				auto owner = oe->container();
				if (owner != nullptr) {
					LOG_BEGIN(ERROR) << "owner" << owner->id()->to_string(); LOG_END;
				}
			}
		}
		return false;
	}
	if (_border && !_border->is_valid()) {
		LOG_BEGIN(ERROR) << "PositionObjectProxy::is_valid() failed for " << sid
			<< " : " << _border->pts().size() << " points too few"; LOG_END;
		return false;
	}
	if (_circle && !_circle->is_valid()) {
		LOG_BEGIN(ERROR) << "PositionObjectProxy::is_valid() failed for " << sid
			<< ": invalid circle"; LOG_END;
		return false;
	}
	if (_pole && !_pole->is_valid()) {
		LOG_BEGIN(ERROR) << "PositionObjectProxy::is_valid() failed for " << sid
			<< ": invalid pole"; LOG_END;
		return false;
	}
	if (!_lane_groups.is_valid(true)) {
		LOG_BEGIN(ERROR) << "PositionObjectProxy::is_valid() failed for " << sid
			<< ": has invalid lane_groups"; LOG_END;
		return false;
	}
	return true;
};

bool PositionObjectProxy::correct_tile_refs(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted() || !_id) {
		return false;
	}
	bool bc = false;
	std::set<int> ref_tiles;
	if (_border) {
		for (auto p : _border->pts()) {
			int tid = mgr->WGS84_to_tileID(*p);
			if (tid <= 0) {
				LOG_BEGIN(INFO) << "PositionObjectProxy::correct_tile_refs() for " << _id->to_string()
					<< " found error border point @ " << p->x() << ", " << p->y() << ", " << p->z(); LOG_END;
			}
			ref_tiles.insert(tid);
		}
	}
	if (_circle && _circle->center()) {
		int tid = mgr->WGS84_to_tileID(*(_circle->center()));
		if (tid <= 0) {
			auto p = _circle->center();
			LOG_BEGIN(INFO) << "PositionObjectProxy::correct_tile_refs() for " << _id->to_string()
				<< " found error pole point @ " << p->x() << ", " << p->y() << ", " << p->z(); LOG_END;
		}
		ref_tiles.insert(tid);
	}
	if (_pole) {
		for (auto p : _pole->pts()) {
			int tid = mgr->WGS84_to_tileID(*p);
			if (tid <= 0) {
				LOG_BEGIN(INFO) << "PositionObjectProxy::correct_tile_refs() for " << _id->to_string()
					<< " found error pole point @ " << p->x() << ", " << p->y() << ", " << p->z(); LOG_END;
			}
			ref_tiles.insert(tid);
		}
	}
	for (auto& l : _lane_groups) {
		if (l && l.proxy()) {
			auto ts = l.proxy()->referenced_tiles();
			for (auto& t : ts) {
				int tid = t->tile_id();
				ref_tiles.insert(tid);
			}
		}
	}
	for (int tid : ref_tiles) {
		auto tile = mgr->get_road_tile(tid);
		if (add_missed_ref_tiles(tile.get())) {
			FeatureReference<PositionObjectProxy> e;
			e = mutable_changed_id(mgr->current_version());
			tile->position_object_refs().push_back(e);
			// LOG_BEGIN(INFO) << "PositionObjectProxy::correct_tile_refs(): add tile "
			// 	<< tile->tile_id() << " 's ref " << e->to_string(); LOG_END;  //cxf打印了太多log  注释了
			bc = true;
		}
	}

	auto tiles = referenced_tiles();
	for (auto& tw : tiles) {
		auto tile = const_cast<TileInfoProxy*>(tw);
		if (tile && ref_tiles.count(tile->tile_id()) <= 0) {
			auto ref = tile->position_object_refs().find(*id());
			if (ref && ref->is_deleted()) {
				continue;
			}
			auto did = mutable_deleted_id(mgr->current_version());
			tile->position_object_refs().replace(id(), did);
			LOG_BEGIN(INFO) << "PositionObjectProxy::correct_tile_refs(): remove tile " 
				<< tile->tile_id() << " 's ref " << did->to_string(); LOG_END;
			bc = true;
		}
	}
	return bc;
};

bool PositionObjectProxy::correct_content(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted()) {
		return false;
	}
	std::string sid("!no_id!");
	bool bc = false;
	if (_id) {
		sid = _id->to_string();
		if (!_id->is_type(feature_id_type())) {
			LOG_BEGIN(INFO) << "PositionObjectProxy::correct_content() correct: "
				<< sid << " to " << feature_id_type(); LOG_END;
			_id->set_type(feature_id_type());
		}
		bc |= _id->correct_content(mgr);
	}
	else {
		LOG_BEGIN(ERROR) << "PositionObjectProxy::correct_content(): no id in element"; LOG_END;
		return false;
	}
	if (!_border && !_circle && !_pole) {
		LOG_BEGIN(INFO) << "PositionObjectProxy::correct_content() failed for " << sid
			<< " : no geom"; LOG_END;
		set_deleted();
		return true;
	}
	if (_border && _circle) {
		if (_type && (*_type == RoadPB::PositionObject::SIGN
				//|| *_type == RoadPB::PositionObject::BOARD
				|| *_type == RoadPB::PositionObject::TRAFFIC_LIGHT)) {
			_border.reset();
			LOG_BEGIN(ERROR) << "PositionObjectProxy::correct_content():  for "
				<< _id->to_string() << ": reset border for type " << *_type; LOG_END;
			bc = true;
		}
		else {
			_circle.reset();
			LOG_BEGIN(ERROR) << "PositionObjectProxy::correct_content():  for "
				<< _id->to_string() << ": reset circle for type " << *_type; LOG_END;
			bc = true;
		}
	}
	if (_border && _pole) {
		if (_type && *_type == RoadPB::PositionObject::POLE) {
			_border.reset();
			LOG_BEGIN(ERROR) << "PositionObjectProxy::correct_content():  for "
				<< _id->to_string() << ": reset border for type " << *_type; LOG_END;
			bc = true;
		}
		else {
			_pole.reset();
			LOG_BEGIN(ERROR) << "PositionObjectProxy::correct_content():  for "
				<< _id->to_string() << ": reset pole for type " << *_type; LOG_END;
			bc = true;
		}
	}
	if (_circle && _pole) {
		if (_type && *_type == RoadPB::PositionObject::POLE) {
			_circle.reset();
			LOG_BEGIN(ERROR) << "PositionObjectProxy::correct_content():  for "
				<< _id->to_string() << ": reset circle for type " << *_type; LOG_END;
			bc = true;
		}
		else {
			_pole.reset();
			LOG_BEGIN(ERROR) << "PositionObjectProxy::correct_content():  for "
				<< _id->to_string() << ": reset pole for type " << *_type; LOG_END;
			bc = true;
		}
	}
	for (auto itr = _lane_groups.begin(); itr != _lane_groups.end();)
	{
		if ((*itr) && (*itr)->is_deleted())
		{
			itr = _lane_groups.erase(itr);
			bc = true;
		}
		else
		{
			++itr;
		}
	}
	auto pos = wgs84_pos();
	auto np = std::dynamic_pointer_cast<PositionObjectProxy>(shared_from_this());
	auto ptr = std::dynamic_pointer_cast<PositionObjectProxy>(shared_from_this());
	TileInfoPtr nti;
	if (mgr->remake_id(np, pos, nti, true)) {
		nti->position_objects().push_back(ptr);
		auto tile = mgr->get_road_tile(np->id()->tileid());
		tile->position_objects().replace(ptr, np);
		auto tiles = referenced_tiles();
		for (auto& tw : tiles) {
			auto tile = const_cast<TileInfoProxy*>(tw);
			if (tile) {
				tile->position_object_refs().replace(ptr->id(), np->mutable_id());
				LOG_BEGIN(INFO) << "PositionObjectProxy::correct_content(): replace tile "
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

Vector3D PositionObjectProxy::wgs84_pos() const {
	if (_border && _border->pts().size() > 0) {
		return *_border->pts().front();
	}
	else if (_circle && _circle->center()) {
		return *_circle->center();
	}
	else if (_pole && _pole->pts().size() > 0) {
		return *_pole->pts().front();
	}
	else {
		return { 0, 0, 0 };
	}
};

const Vector3D& PositionObjectProxy::global_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	sync_wgs_to_global_pos(mgr);
	return _global_pos;
};

void PositionObjectProxy::sync_wgs_to_global_pos(const RoadGeometryManager* mgr) const {
	if (_pos_sync_cnt < _change_count) {
		if (mgr == nullptr) {
			mgr = DAEInterface::get_instance()->get_road_geometry_manager();
		}
		if (mgr && _border && _border->pts().size() > 0) {
			_global_pos = *_border->pts().front();
			if (mgr->global_point_from_WGS84(_global_pos)) {
				_pos_sync_cnt = _change_count;
			}
			else {
				LOG_BEGIN(INFO) << "PositionObjectProxy::sync_wgs_to_global() failed ("
					<< _global_pos << ")"; LOG_END;
				_global_pos = { 0, 0, 0 };
			}
		}
		else if (mgr && _circle && _circle->center()) {
			_global_pos = *_circle->center();
			if (mgr->global_point_from_WGS84(_global_pos)) {
				_pos_sync_cnt = _change_count;
			}
			else {
				LOG_BEGIN(INFO) << "PositionObjectProxy::sync_wgs_to_global() failed ("
					<< _global_pos << ")"; LOG_END;
				_global_pos = { 0, 0, 0 };
			}
		}
		else if (mgr && _pole && _pole->pts().size() > 0) {
			_global_pos = *_pole->pts().front();
			if (mgr->global_point_from_WGS84(_global_pos)) {
				_pos_sync_cnt = _change_count;
			}
			else {
				LOG_BEGIN(INFO) << "PositionObjectProxy::sync_wgs_to_global() failed ("
					<< _global_pos << ")"; LOG_END;
				_global_pos = { 0, 0, 0 };
			}
		}
		else {
			_global_pos = { 0, 0, 0 };
			LOG_BEGIN(ERROR) << "PositionObjectProxy::sync_wgs_to_global() failed for empty pos"; LOG_END;
		}
	}
	else {
		_pos_sync_cnt = _change_count;
	}
};

bool PositionObjectProxy::remake_proxy(RoadGeometryManager* mgr) {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	bool bf = true;
	if (!_lane_groups.resolve_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "PositionObjectProxy::remake_proxy(): " << sid
			<< " has unresolved lane_groups!"; LOG_END;
	}
	return bf;
};

bool PositionObjectProxy::merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) {
	return FeatureMerger<PositionObjectProxy>(policy, mgr, this, "PositionObject")
		.merge(_border, "border")
		.merge(_circle, "circle")
		.merge(_pole, "pole")
		.merge(_type, "type")
		.merge(_content, "content")
		.merge_unordered(_lane_groups, "lane_groups", false)
		.is_conflict();
};

};
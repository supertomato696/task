#include "link_proxy.h"
#include "data_access_engine.h"
#include "manager/road_geometry_mgr.h"
#include "lane_proxy.h"
#include "tile_proxy.h"
#include "feature_merger.h"

namespace data_access_engine {

/*PolylineProxy* LinkTollInfoProxy::mutable_geom() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_geom) {
		_geom.reset(FeatureProxyBase::create_proxy<PolylineProxy>(this));
	}
	return _geom.get();
};
bool LinkTollInfoProxy::clear_geom() {
	if (!is_editable()) {
		return false;
	}
	if (_geom) {
		mark_changed();
	}
	_geom.reset();
	return true;
};

bool LinkTollInfoProxy::set_toll_type(Optional<int> t) {
	if (!is_editable()) {
		return false;
	}
	if (_toll_type != t) {
		mark_changed();
	}
	_toll_type = t;
	return true;
};

bool LinkTollInfoProxy::set_fee_type(Optional<int> f) {
	if (!is_editable()) {
		return false;
	}
	if (_fee_type != f) {
		mark_changed();
	}
	_fee_type = f;
	return true;
};

bool LinkTollInfoProxy::set_toll_standard(Optional<double> t) {
	if (!is_editable()) {
		return false;
	}
	if (_toll_standard != t) {
		mark_changed();
	}
	_toll_standard = t;
	return true;
};

bool LinkTollInfoProxy::set_valid_period(const Optional<std::string>& v) {
	if (!is_editable()) {
		return false;
	}
	if (_valid_period != v) {
		mark_changed();
	}
	_valid_period = v;
	return true;
};

bool LinkTollInfoProxy::set_vehicle_type(const Optional<std::string>& v) {
	if (!is_editable()) {
		return false;
	}
	if (_vehicle_type != v) {
		mark_changed();
	}
	_vehicle_type = v;
	return true;
};

bool LinkTollInfoProxy::set_attribute_len(Optional<double> a) {
	if (!is_editable()) {
		return false;
	}
	if (_attribute_len != a) {
		mark_changed();
	}
	_attribute_len = a;
	return true;
}

bool LinkTollInfoProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::LinkTollInfo*>(msg_base);
	if (msg->has_geom()) {
		_geom.reset(FeatureProxyBase::create_proxy<PolylineProxy>(this));
		_geom->from_message(&msg->geom());
	}
	else {
		_geom.reset();
	}
	if (msg->has_toll_type()) {
		_toll_type = msg->toll_type();
	}
	else {
		_toll_type.reset();
	}
	if (msg->has_fee_type()) {
		_fee_type = msg->fee_type();
	}
	else {
		_fee_type.reset();
	}
	if (msg->has_toll_standard()) {
		_toll_standard = msg->toll_standard();
	}
	else {
		_toll_standard.reset();
	}
	if (msg->has_valid_period()) {
		_valid_period = msg->valid_period();
	}
	else {
		_valid_period.reset();
	}
	if (msg->has_vehicle_type()) {
		_vehicle_type = msg->vehicle_type();
	}
	else{
		_vehicle_type.reset();
	}
	if (msg->has_attribute_len()) {
		_attribute_len = msg->attribute_len();
	}
	else {
		_attribute_len.reset();
	}
	clear_changed();
	return true;
};

bool LinkTollInfoProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::LinkTollInfo*>(msg_base);
	msg->Clear();
	if (_geom) {
		_geom->to_message(msg->mutable_geom());
	}
	if (_toll_type) {
		msg->set_toll_type(*_toll_type);
	}
	if (_fee_type) {
		msg->set_fee_type(*_fee_type);
	}
	if (_toll_standard) {
		msg->set_toll_standard(*_toll_standard);
	}
	if (_valid_period) {
		msg->set_valid_period(*_valid_period);
	}
	if (_vehicle_type) {
		msg->set_vehicle_type(*_vehicle_type);
	}
	if (_attribute_len) {
		msg->set_attribute_len(*_attribute_len);
	}
	return true;
};

bool LinkTollInfoProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
									const std::shared_ptr<const FeatureProxyBase>& base,
									const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const LinkTollInfoProxy>(remote);
	auto o = std::dynamic_pointer_cast<const LinkTollInfoProxy>(base);
	return ProxyMerger<LinkTollInfoProxy>(policy, mgr, this, "LinkTollInfoProxy", r, o)
										.merge(_geom, "geom")
										.merge(_toll_type, "toll_type")
										.merge(_fee_type, "fee_type")
										.merge(_toll_standard, "toll_standard")
										.merge(_valid_period, "valid_period")
										.merge(_vehicle_type, "vehicle_type")
										.merge(_attribute_len, "attribute_len")
										.is_conflict();
}

bool LinkTollInfoProxy::is_valid() const {
	return true;
};

int LinkTollInfoProxy::compare(const LinkTollInfoProxy& rhs) const {
	int ret = value_compare(_toll_type, rhs._toll_type);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_fee_type, rhs._fee_type);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_toll_standard, rhs._toll_standard);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_valid_period, rhs._valid_period);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_vehicle_type, rhs._vehicle_type);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_attribute_len, rhs._attribute_len);
	if (ret != 0) {
		return ret;
	}
	return proxy_compare(_geom, rhs._geom);
};

FeatureReference<NodeProxy>* TurnRestrictProxy::mutable_node_id() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_node_id;
};

FeatureReference<LinkProxy>* TurnRestrictProxy::mutable_tolink_id() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_tolink_id;
};

bool TurnRestrictProxy::set_type(Optional<int> t) {
	if (!is_editable()) {
		return false;
	}
	if (_type != t) {
		mark_changed();
	}
	_type = t;
	return true;
};

bool TurnRestrictProxy::set_info(Optional<int> i) {
	if (!is_editable()) {
		return false;
	}
	if (_info != i) {
		mark_changed();
	}
	_info = i;
	return true;
};

bool TurnRestrictProxy::set_kind(Optional<int> k) {
	if (!is_editable()) {
		return false;
	}
	if (_kind != k) {
		mark_changed();
	}
	_kind = k;
	return true;
};

bool TurnRestrictProxy::set_valid_period(const Optional<std::string>& v) {
	if (!is_editable()) {
		return false;
	}
	if (_valid_period != v) {
		mark_changed();
	}
	_valid_period = v;
	return true;
};

bool TurnRestrictProxy::set_vehicle_type(const Optional<std::string>& v) {
	if (!is_editable()) {
		return false;
	}
	if (_vehicle_type != v) {
		mark_changed();
	}
	_vehicle_type = v;
	return true;
};


bool TurnRestrictProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::TurnRestrict*>(msg_base);
	if (msg->has_node_id()) {
		_node_id.create(this)->from_message(&msg->node_id());
	}
	else {
		_node_id.reset();
	}
	if (msg->has_tolink_id()) {
		_tolink_id.create(this)->from_message(&msg->tolink_id());
	}
	else {
		_tolink_id.reset();
	}
	if (msg->has_type()) {
		_type = msg->type();
	}
	else {
		_type.reset();
	}
	if (msg->has_info()) {
		_info = msg->info();
	}
	else {
		_info.reset();
	}
	if (msg->has_kind()) {
		_kind = msg->kind();
	}
	else {
		_kind.reset();
	}
	if (msg->has_valid_period()) {
		_valid_period = msg->valid_period();
	}
	else {
		_valid_period.reset();
	}
	if (msg->has_vehicle_type()) {
		_vehicle_type = msg->vehicle_type();
	}
	else {
		_vehicle_type.reset();
	}
	clear_changed();
	return true;
};

bool TurnRestrictProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::TurnRestrict*>(msg_base);
	msg->Clear();
	if (_node_id) {
		_node_id->to_message(msg->mutable_node_id());
	}
	if (_tolink_id) {
		_tolink_id->to_message(msg->mutable_tolink_id());
	}
	if (_type) {
		msg->set_type(*_type);
	}
	if (_info) {
		msg->set_info(*_info);
	}
	if (_kind) {
		msg->set_kind(*_kind);
	}
	if (_valid_period) {
		msg->set_valid_period(*_valid_period);
	}
	if (_vehicle_type) {
		msg->set_vehicle_type(*_vehicle_type);
	}
	return true;
};

bool TurnRestrictProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
									const std::shared_ptr<const FeatureProxyBase>& base,
									const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const TurnRestrictProxy>(remote);
	auto o = std::dynamic_pointer_cast<const TurnRestrictProxy>(base);
	return ProxyMerger<TurnRestrictProxy>(policy, mgr, this, "TurnRestrictProxy", r, o)
										.merge(_node_id, "node_id")
										.merge(_tolink_id, "tolink_id")
										.merge(_type, "type")
										.merge(_info, "info")
										.merge(_kind, "kind")
										.merge(_valid_period, "valid_period")
										.merge(_vehicle_type, "vehicle_type")
										.is_conflict();
}

bool TurnRestrictProxy::is_valid() const {
	if (!_node_id.id() || !_tolink_id.id()) {
		return false;
	}
	return true;
};

int TurnRestrictProxy::compare(const TurnRestrictProxy& rhs) const {
	int ret = value_compare(_type, rhs._type);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_info, rhs._info);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_kind, rhs._kind);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_valid_period, rhs._valid_period);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_vehicle_type, rhs._vehicle_type);
	if (ret != 0) {
		return ret;
	}
	ret = proxy_compare(_node_id.id(), rhs._node_id.id());
	if (ret != 0) {
		return ret;
	}
	return proxy_compare(_tolink_id.id(), rhs._tolink_id.id());
};*/

FeatureReference<NodeProxy>* LinkProxy::mutable_snode_id() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_snode_id;
};
FeatureReference<NodeProxy>* LinkProxy::mutable_enode_id() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_enode_id;
};
bool LinkProxy::set_direction(Optional<int> d) {
	if (!is_editable()) {
		return false;
	}
	if (_direction != d) {
		mark_changed();
	}
	_direction = d;
	return true;
};
/*bool LinkProxy::set_lane_count(Optional<int> l) {
	if (!is_editable()) {
		return false;
	}
	if (_lane_count != l) {
		mark_changed();
	}
	_lane_count = l;
	return true;
};
bool LinkProxy::set_left_lanes(Optional<int> l) {
	if (!is_editable()) {
		return false;
	}
	if (_left_lanes != l) {
		mark_changed();
	}
	_left_lanes = l;
	return true;
};
bool LinkProxy::set_right_lanes(Optional<int> l) {
	if (!is_editable()) {
		return false;
	}
	if (_right_lanes != l) {
		mark_changed();
	}
	_right_lanes = l;
	return true;
};
bool LinkProxy::set_multiply_digitized(Optional<int> m) {
	if (!is_editable()) {
		return false;
	}
	if (_multiply_digitized != m) {
		mark_changed();
	}
	_multiply_digitized = m;
	return true;
};*/
bool LinkProxy::set_link_type(Optional<int> l) {
	if (!is_editable()) {
		return false;
	}
	if (_link_type != l) {
		mark_changed();
	}
	_link_type = l;
	return true;
};
bool LinkProxy::set_road_class(Optional<int> c) {
	if (!is_editable()) {
		return false;
	}
	if (_road_class != c) {
		mark_changed();
	}
	_road_class = c;
	return true;
};
/*bool LinkProxy::set_urban_flag(Optional<int> f) {
	if (!is_editable()) {
		return false;
	}
	if (_urban_flag != f) {
		mark_changed();
	}
	_urban_flag = f;
	return true;
};
bool LinkProxy::set_toll_area(Optional<int> t) {
	if (!is_editable()) {
		return false;
	}
	if (_toll_area != t) {
		mark_changed();
	}
	_toll_area = t;
	return true;
};*/
bool LinkProxy::set_link_length(Optional<double> l) {
	if (!is_editable()) {
		return false;
	}
	if (_link_length != l) {
		mark_changed();
	}
	_link_length = l;
	return true;
};
/*bool LinkProxy::set_open_status(Optional<int> s) {
	if (!is_editable()) {
		return false;
	}
	if (_open_status != s) {
		mark_changed();
	}
	_open_status = s;
	return true;
};
bool LinkProxy::set_state(Optional<int> s) {
	if (!is_editable()) {
		return false;
	}
	if (_state != s) {
		mark_changed();
	}
	_state = s;
	return true;
};*/
PolylineProxy* LinkProxy::mutable_geom() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_geom) {
		_geom.reset(FeatureProxyBase::create_proxy<PolylineProxy>(this));
	}
	return _geom.get();
};
bool LinkProxy::clear_geom() {
	if (!is_editable()) {
		return false;
	}
	if (_geom) {
		mark_changed();
	}
	_geom.reset();
	return true;
};
/*SharedProxyVector<LinkTollInfoProxy>* LinkProxy::mutable_tollinfos() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_tollinfos;
};
SharedProxyVector<TurnRestrictProxy>* LinkProxy::mutable_turn_restricts() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_turn_restricts;
};
bool LinkProxy::set_road_grade(Optional<int> r) {
	if (!is_editable()) {
		return false;
	}
	if (_road_grade != r) {
		mark_changed();
	}
	_road_grade = r;
	return true;
};
bool LinkProxy::set_net_grade(Optional<int> n) {
	if (!is_editable()) {
		return false;
	}
	if (_net_grade != n) {
		mark_changed();
	}
	_net_grade = n;
	return true;
};
bool LinkProxy::set_open_time(const Optional<std::string>& o) {
	if (!is_editable()) {
		return false;
	}
	if (_open_time != o) {
		mark_changed();
	}
	_open_time = o;
	return true;
};
bool LinkProxy::set_valid_period(const Optional<std::string>& v) {
	if (!is_editable()) {
		return false;
	}
	if (_valid_period != v) {
		mark_changed();
	}
	_valid_period = v;
	return true;
};
bool LinkProxy::set_vehicle_type(const Optional<std::string>& v) {
	if (!is_editable()) {
		return false;
	}
	if (_vehicle_type != v) {
		mark_changed();
	}
	_vehicle_type = v;
	return true;
};*/
SharedProxyVector<LangNameProxy>* LinkProxy::mutable_names() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_names;
};
bool LinkProxy::set_provincecode(Optional<int> p) {
	if (!is_editable()) {
		return false;
	}
	if (_provincecode != p) {
		mark_changed();
	}
	_provincecode = p;
	return true;
};
FeatureMemberVector<LaneGroupProxy>* LinkProxy::mutable_forward_groups() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_forward_groups;
};
FeatureMemberVector<LaneGroupProxy>* LinkProxy::mutable_backward_groups() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_backward_groups;
};

const Vector3D& LinkProxy::global_stp_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	sync_wgs_to_global_pos(mgr);
	return _global_stp_pos;
};

const Vector3D& LinkProxy::global_edp_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	sync_wgs_to_global_pos(mgr);
	return _global_edp_pos;
};

Vector3D LinkProxy::wgs84_pos() const {
	if (_geom && _geom->pts().size() > 0) {
		return *_geom->pts().front();
	}
	return Vector3D(0, 0, 0);
};

void LinkProxy::sync_wgs_to_global_pos(const RoadGeometryManager* mgr) const {
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
				LOG_BEGIN(DEBUG) << "LinkProxy::sync_wgs_to_global() failed ("
					<< _global_stp_pos << ", " << _global_edp_pos << ")"; LOG_END;
				_global_stp_pos = { 0, 0, 0 };
				_global_edp_pos = { 0, 0, 0 };
			}

		}
		else {
			_global_stp_pos = { 0, 0, 0 };
			_global_edp_pos = { 0, 0, 0 };
			LOG_BEGIN(ERROR) << "LinkProxy::sync_wgs_to_global() failed for empty pos"; LOG_END;
		}
	}
	else {
		_pos_sync_cnt = _change_count;
	}
};

bool LinkProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::Link*>(msg_base);
	msg->Clear();
	id_to_message(msg);
	if (_snode_id) {
		_snode_id->to_message(msg->mutable_snode_id());
	}
	if (_enode_id) {
		_enode_id->to_message(msg->mutable_enode_id());
	}
	if (_direction) {
		msg->set_direction(*_direction);
	}
	/*if (_lane_count) {
		msg->set_lane_count(*_lane_count);
	}
	if (_left_lanes) {
		msg->set_left_lanes(*_left_lanes);
	}
	if (_right_lanes) {
		msg->set_right_lanes(*_right_lanes);
	}
	if (_multiply_digitized) {
		msg->set_multiply_digitized(*_multiply_digitized);
	}*/
	if (_link_type) {
		msg->set_link_type(*_link_type);
	}
	if (_road_class) {
		msg->set_road_class(*_road_class);
	}
	/*if (_urban_flag) {
		msg->set_urban_flag(*_urban_flag);
	}
	if (_toll_area) {
		msg->set_toll_area(*_toll_area);
	}*/
	if (_link_length) {
		msg->set_link_length(*_link_length);
	}
	/*if (_open_status) {
		msg->set_open_status(*_open_status);
	}
	if (_state) {
		msg->set_state(*_state);
	}*/
	if (_geom) {
		_geom->to_message(msg->mutable_geom());
	}
	/*_tollinfos.to_message(msg->mutable_tollinfos());
	_turn_restricts.to_message(msg->mutable_turn_restricts());
	if (_road_grade) {
		msg->set_road_grade(*_road_grade);
	}
	if (_net_grade) {
		msg->set_net_grade(*_net_grade);
	}
	if (_open_time) {
		msg->set_open_time(*_open_time);
	}
	if (_valid_period) {
		msg->set_valid_period(*_valid_period);
	}
	if (_vehicle_type) {
		msg->set_vehicle_type(*_vehicle_type);
	}*/
	_names.to_message(msg->mutable_names());
	if (_provincecode) {
		msg->set_provincecode((RoadPB::ProvinceCode)*_provincecode);
	}
	_forward_groups.to_message(msg->mutable_forward_groups());
	_backward_groups.to_message(msg->mutable_backward_groups());
	return true;
};

bool LinkProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::Link*>(msg_base);
	id_from_message(msg);
	if (msg->has_snode_id()) {
		_snode_id.create(this)->from_message(&msg->snode_id());
	}
	else {
		_snode_id.reset();
	}
	if (msg->has_enode_id()) {
		_enode_id.create(this)->from_message(&msg->enode_id());
	}
	else {
		_enode_id.reset();
	}
	if (msg->has_direction()) {
		_direction = msg->direction();
	}
	else {
		_direction.reset();
	}
	/*if (msg->has_lane_count()) {
		_lane_count = msg->lane_count();
	}
	else {
		_lane_count.reset();
	}
	if (msg->has_left_lanes()) {
		_left_lanes = msg->left_lanes();
	}
	else {
		_left_lanes.reset();
	}
	if (msg->has_right_lanes()) {
		_right_lanes = msg->right_lanes();
	}
	else {
		_right_lanes.reset();
	}
	if (msg->has_multiply_digitized()) {
		_multiply_digitized = msg->multiply_digitized();
	}
	else {
		_multiply_digitized.reset();
	}*/
	if (msg->has_link_type()) {
		_link_type = msg->link_type();
	}
	else {
		_link_type.reset();
	}
	if (msg->has_road_class()) {
		_road_class = msg->road_class();
	}
	else {
		_road_class.reset();
	}
	/*if (msg->has_urban_flag()) {
		_urban_flag = msg->urban_flag();
	}
	else {
		_urban_flag.reset();
	}
	if (msg->has_toll_area()) {
		_toll_area = msg->toll_area();
	}
	else {
		_toll_area.reset();
	}*/
	if (msg->has_link_length()) {
		_link_length = msg->link_length();
	}
	else {
		_link_length.reset();
	}
	/*if (msg->has_open_status()) {
		_open_status = msg->open_status();
	}
	else {
		_open_status.reset();
	}
	if (msg->has_state()) {
		_state = msg->state();
	}
	else {
		_state.reset();
	}*/
	if (msg->has_geom()) {
		_geom.reset(FeatureProxyBase::create_proxy<PolylineProxy>(this));
		_geom->from_message(&msg->geom());
	}
	else {
		_geom.reset();
	}
	/*_tollinfos.from_message(msg->tollinfos());
	_turn_restricts.from_message(msg->turn_restricts());
	if (msg->has_road_grade()) {
		_road_grade = msg->road_grade();
	}
	else {
		_road_grade.reset();
	}
	if (msg->has_net_grade()) {
		_net_grade = msg->net_grade();
	}
	else {
		_net_grade.reset();
	}
	if (msg->has_open_time()) {
		_open_time = msg->open_time();
	}
	else {
		_open_time.reset();
	}
	if (msg->has_valid_period()) {
		_valid_period = msg->valid_period();
	}
	else {
		_valid_period.reset();
	}
	if (msg->has_vehicle_type()) {
		_vehicle_type = msg->vehicle_type();
	}
	else {
		_vehicle_type.reset();
	}*/
	_names.from_message(msg->names());
	if (msg->has_provincecode()) {
		_provincecode = msg->provincecode();
	}
	else {
		_provincecode.reset();
	}
	_forward_groups.from_message(msg->forward_groups());
	_backward_groups.from_message(msg->backward_groups());
	clear_changed();
	return true;
};

bool LinkProxy::is_valid() const {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	if (!is_message_valid<LinkProxy>()) {
		LOG_BEGIN(ERROR) << "LinkProxy::is_valid(): failed for " << sid
			<< " : is_message_valid() failed!"; LOG_END;
		return false;
	}
	if (!is_editable() || is_deleted()) {
		//LOG_BEGIN(DEBUG) << "LinkProxy::is_valid() skip for " << sid; LOG_END;
		return true;
	}
	if (!is_ref_valid()) {
		LOG_BEGIN(ERROR) << "LinkProxy::is_valid() failed for " << sid
			<< " : has " << owner_count() << " owners"; LOG_END;
		return false;
	}
	if (!_forward_groups.is_valid(true) || !_backward_groups.is_valid(true)) {
		LOG_BEGIN(ERROR) << "LinkProxy::is_valid() failed for " << sid
			<< " : has " << _forward_groups.size() << " forwards, "
			<< _backward_groups.size() << " backwards"; LOG_END;
		return false;
	}
	if (!_geom || _geom->pts().empty()) {
		LOG_BEGIN(ERROR) << "LinkProxy::is_valid() failed for " << sid
			<< " : has empty link!"; LOG_END;
		return false;
	}

	if (!_geom->is_valid()) {
		LOG_BEGIN(ERROR) << "LinkProxy::is_valid() failed for " << sid
			<< " : has invalid geom "; LOG_END;
		return false;
	}
	if (_snode_id && !_snode_id->is_valid()) {
		LOG_BEGIN(ERROR) << "LinkProxy::is_valid() failed for " << sid
			<< " : has invalid snode_id " << _snode_id->to_string(); LOG_END;
		return false;
	}
	if (_enode_id && !_enode_id->is_valid()) {
		LOG_BEGIN(ERROR) << "LinkProxy::is_valid() failed for " << sid
			<< " : has invalid enode_id " << _enode_id->to_string(); LOG_END;
		return false;
	}
	/*if (!_tollinfos.is_valid()) {
		LOG_BEGIN(INFO) << "LinkProxy::is_valid() failed for " << sid
			<< " : has invalid tollinfos " << _tollinfos.size(); LOG_END;
		return false;
	}
	if (!_turn_restricts.is_valid()) {
		LOG_BEGIN(INFO) << "LinkProxy::is_valid() failed for " << sid
			<< " : has invalid turn_restricts " << _turn_restricts.size(); LOG_END;
		return false;
	}*/
	return true;
};

bool LinkProxy::judge_editable(RoadGeometryManager* mgr) {
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

bool LinkProxy::correct_tile_refs(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted() || !_geom || !_id) {
		return false;
	}
	bool bc = false;
	std::set<int> ref_tiles;
	std::vector<const TileInfoProxy*> tiles;
	for (auto& l : _forward_groups) {
		if (!l || !l.proxy()) {
			continue;
		}
		auto ts = l.proxy()->referenced_tiles();
		tiles.insert(tiles.end(), ts.begin(), ts.end());
	}
	for (auto& l : _backward_groups) {
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
			FeatureReference<LinkProxy> e;
			e = mutable_changed_id(mgr->current_version());
			tile->link_refs().push_back(e);
			//LOG_BEGIN(INFO) << "LinkProxy::correct_tile_refs(): add tile "
			//	<< tile->tile_id() << " 's ref " << e->to_string(); LOG_END;
			bc = true;
		}
	}

	for (auto p : _geom->pts()) {
		int tid = mgr->WGS84_to_tileID(*p);
		if (tid <= 0) {
			LOG_BEGIN(INFO) << "LinkProxy::correct_tile_refs() for " << _id->to_string()
				<< " found error point @ " << p->x() << ", " << p->y() << ", " << p->z(); LOG_END;
		}
		if (ref_tiles.count(tid) > 0) {
			continue;
		}
		ref_tiles.insert(tid);
		auto tile = mgr->get_road_tile(tid);
		if (add_missed_ref_tiles(tile.get())) {
			FeatureReference<LinkProxy> e;
			e = mutable_changed_id(mgr->current_version());
			tile->link_refs().push_back(e);
			//LOG_BEGIN(INFO) << "LinkProxy::correct_tile_refs(): add tile "
			//	<< tile->tile_id() << " 's ref " << e->to_string(); LOG_END;
			bc = true;
		}
	}

	tiles = referenced_tiles();
	for (auto& tw : tiles) {
		auto tile = const_cast<TileInfoProxy*>(tw);
		if (tile && ref_tiles.count(tile->tile_id()) <= 0) {
			auto ref = tile->link_refs().find(*id());
			if (ref && ref->is_deleted()) {
				continue;
			}
			auto did = mutable_deleted_id(mgr->current_version());
			tile->link_refs().replace(id(), did);
			LOG_BEGIN(INFO) << "LinkProxy::correct_tile_refs(): remove tile "
				<< tile->tile_id() << " 's ref " << did->to_string(); LOG_END;
			bc = true;
		}
	}
	return bc;
};

bool LinkProxy::correct_content(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted()) {
		return false;
	}
	bool bc = false;
	if (_id) {
		if (!_id->is_type(feature_id_type())) {
			LOG_BEGIN(INFO) << "LinkProxy::correct_content() correct id->type:"
				<< _id->type() << " to " << feature_id_type(); LOG_END;
			_id->set_type(feature_id_type());
			bc = true;
		}
		bc |= _id->correct_content(mgr);
	}
	else {
		LOG_BEGIN(ERROR) << "LinkProxy::correct_content(): no id in element"; LOG_END;
		return false;
	}
	/*if (_tollinfos.remove_invalid()) {
		mark_changed();
		LOG_BEGIN(INFO) << "LinkProxy::correct_content(): correct tollinfos for "
			<< _id->to_string() << ", " << _tollinfos.size() << " left"; LOG_END;
		bc = true;
	}
	if (_turn_restricts.remove_invalid()) {
		mark_changed();
		LOG_BEGIN(INFO) << "LinkProxy::correct_content(): remove invalid turn_restricts for "
			<< _id->to_string() << ", " << _turn_restricts.size() << " left"; LOG_END;
		bc = true;
	}*/
	if (_forward_groups.remove_invalid(true)) {
		mark_changed();
		LOG_BEGIN(INFO) << "LinkProxy::correct_content(): remove invalid forward_groups for "
			<< _id->to_string() << ", " << _forward_groups.size() << " left"; LOG_END;
		bc = true;
	}
	if (_backward_groups.remove_invalid(true)) {
		mark_changed();
		LOG_BEGIN(INFO) << "LinkProxy::correct_content(): remove invalid backward_groups for "
			<< _id->to_string() << ", " << _backward_groups.size() << " left"; LOG_END;
		bc = true;
	}
	if (_geom && _geom->correct_content(mgr)) {
		mark_changed();
		LOG_BEGIN(INFO) << "LinkProxy::correct_content(): remove invalid geom for "
			<< _id->to_string(); LOG_END;
		bc = true;
	}
	if (!_geom || _geom->pts().empty()) {
		LOG_BEGIN(ERROR) << "LinkProxy::correct_content(): set deleted for "
			<< _id->to_string() << ": geom is empty"; LOG_END;
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
	auto np = std::dynamic_pointer_cast<LinkProxy>(shared_from_this());
	auto ptr = std::dynamic_pointer_cast<LinkProxy>(shared_from_this());
	TileInfoPtr nti;
	if (mgr->remake_id(np, pos, nti, true)) {
		nti->links().push_back(ptr);
		auto tile = mgr->get_road_tile(np->id()->tileid());
		tile->links().replace(ptr, np);
		auto tiles = referenced_tiles();
		for (auto& tw : tiles) {
			auto tile = const_cast<TileInfoProxy*>(tw);
			if (tile) {
				tile->link_refs().replace(ptr->id(), np->mutable_id());
				LOG_BEGIN(INFO) << "LinkProxy::correct_content(): replace tile "
					<< tile->tile_id() << " 's ref " << ptr->id()->to_string() << " to "
					<< np->id()->to_string(); LOG_END;
			}
		}
		bc = true;
	}
		
	bc |= correct_tile_refs(mgr);

	bc |= correct_version(mgr);

	if (!cached_is_valid()) {
		LOG_BEGIN(ERROR) << "LinkProxy::correct_content(): set deleted for "
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

bool LinkProxy::remake_proxy(RoadGeometryManager* mgr) {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	bool bf = true;
	if (_snode_id) {
		if (!_snode_id->is_valid()) {
			_snode_id.reset();
		}
		else {
			auto p = mgr->get_link(*_snode_id.id());
			if (p) {
				_snode_id.set(p->id());
			}
			else {
				LOG_BEGIN(INFO) << "LinkProxy::remake_proxy(): " << sid
					<< " has unresolved snode_id " << _snode_id->to_string();
				LOG_END;
				bf = false;
			}
		}
	}
	if (_enode_id) {
		if (!_enode_id->is_valid()) {
			_enode_id.reset();
		}
		else {
			auto p = mgr->get_link(*_enode_id.id());
			if (p) {
				_enode_id.set(p->id());
			}
			else {
				LOG_BEGIN(INFO) << "LinkProxy::remake_proxy(): " << sid
					<< " has unresolved enode_id " << _enode_id->to_string();
				LOG_END;
				bf = false;
			}
		}
	}
	if (!_forward_groups.resolve_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "LinkProxy::remake_proxy(): " << sid
			<< " has unresolved forward_groups!"; LOG_END;
	}
	if (!_backward_groups.resolve_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "LinkProxy::remake_proxy(): " << sid
			<< " has unresolved backward_groups!"; LOG_END;
	}
	/*if (!_turn_restricts.remake_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "LaneGroupProxy::remake_proxy(): " << sid
			<< " has unresolved lanes!"; LOG_END;
	}*/
	return bf;
};

bool LinkProxy::merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) {
	return FeatureMerger<LinkProxy>(policy, mgr, this, "Link")
										.merge(_snode_id, "snode_id")
										.merge(_enode_id, "enode_id")
										.merge(_direction, "direction")
										//.merge(_lane_count, "lane_count")
										//.merge(_left_lanes, "left_lanes")
										//.merge(_right_lanes, "right_lanes")
										//.merge(_multiply_digitized, "multiply_digitized")
										.merge(_link_type, "link_type")
										//.merge(_link_kind, "link_kind")
										//.merge(_urban_flag, "urban_flag")
										//.merge(_toll_area, "toll_area")
										.merge(_link_length, "link_length")
										//.merge(_open_status, "open_status")
										//.merge(_state, "state")
										.merge(_geom, "geom")
										//.merge_elemental(_tollinfos, "tollinfos")
										//.merge_elemental(_turn_restricts, "turn_restricts")
										//.merge(_road_grade, "road_grade")
										//.merge(_net_grade, "net_grade")
										//.merge(_open_time, "open_time")
										//.merge(_valid_period, "valid_period")
										//.merge(_vehicle_type, "vehicle_type")
										.merge(_names, "names")
										.merge(_provincecode, "provincecode")
										.merge_unordered(_forward_groups, "forward_groups")
										.merge_unordered(_backward_groups, "backward_groups")
										.is_conflict();
};

bool LinkExtProxy::set_lane_count(Optional<int> l) {
	if (!is_editable()) {
		return false;
	}
	if (_lane_count != l) {
		mark_changed();
	}
	_lane_count = l;
	return true;
};
bool LinkExtProxy::set_left_lanes(Optional<int> l) {
	if (!is_editable()) {
		return false;
	}
	if (_left_lanes != l) {
		mark_changed();
	}
	_left_lanes = l;
	return true;
};
bool LinkExtProxy::set_right_lanes(Optional<int> l) {
	if (!is_editable()) {
		return false;
	}
	if (_right_lanes != l) {
		mark_changed();
	}
	_right_lanes = l;
	return true;
};
bool LinkExtProxy::set_multiply_digitized(Optional<int> m) {
	if (!is_editable()) {
		return false;
	}
	if (_multiply_digitized != m) {
		mark_changed();
	}
	_multiply_digitized = m;
	return true;
};
bool LinkExtProxy::set_urban_flag(Optional<int> f) {
	if (!is_editable()) {
		return false;
	}
	if (_urban_flag != f) {
		mark_changed();
	}
	_urban_flag = f;
	return true;
};
bool LinkExtProxy::set_road_grade(Optional<int> r) {
	if (!is_editable()) {
		return false;
	}
	if (_road_grade != r) {
		mark_changed();
	}
	_road_grade = r;
	return true;
};
bool LinkExtProxy::set_net_grade(Optional<int> n) {
	if (!is_editable()) {
		return false;
	}
	if (_net_grade != n) {
		mark_changed();
	}
	_net_grade = n;
	return true;
};
bool LinkExtProxy::set_pavement_info(Optional<int> p) {
	if (!is_editable()) {
		return false;
	}
	if (_pavement_info != p) {
		mark_changed();
	}
	_pavement_info = p;
	return true;
};
bool LinkExtProxy::set_pass_type(Optional<int> p) {
	if (!is_editable()) {
		return false;
	}
	if (_pass_type != p) {
		mark_changed();
	}
	_pass_type = p;
	return true;
};
bool LinkExtProxy::set_grade(Optional<int> g) {
	if (!is_editable()) {
		return false;
	}
	if (_grade != g) {
		mark_changed();
	}
	_grade = g;
	return true;
};

bool LinkExtProxy::to_message(google::protobuf::Message* msg_base) const {
	LinkProxy::to_message(msg_base);
	auto msg = static_cast<RoadPB::Link*>(msg_base);
	if (_lane_count) {
		msg->set_lane_count(*_lane_count);
	}
	if (_left_lanes) {
		msg->set_left_lanes(*_left_lanes);
	}
	if (_right_lanes) {
		msg->set_right_lanes(*_right_lanes);
	}
	if (_multiply_digitized) {
		msg->set_multiply_digitized(*_multiply_digitized);
	}
	if (_urban_flag) {
		msg->set_urban_flag(*_urban_flag);
	}
	if (_road_grade) {
		msg->set_road_grade(*_road_grade);
	}
	if (_net_grade) {
		msg->set_net_grade(*_net_grade);
	}
	if (_pavement_info) {
		msg->set_pavement_info(*_pavement_info);
	}
	if (_pass_type) {
		msg->set_pass_type(*_pass_type);
	}
	if (_grade) {
		msg->set_grade(*_grade);
	}
	return true;
};

bool LinkExtProxy::from_message(const google::protobuf::Message* msg_base) {
	LinkProxy::from_message(msg_base);
	auto msg = static_cast<const RoadPB::Link*>(msg_base);
	if (msg->has_lane_count()) {
		_lane_count = msg->lane_count();
	}
	else {
		_lane_count.reset();
	}
	if (msg->has_left_lanes()) {
		_left_lanes = msg->left_lanes();
	}
	else {
		_left_lanes.reset();
	}
	if (msg->has_right_lanes()) {
		_right_lanes = msg->right_lanes();
	}
	else {
		_right_lanes.reset();
	}
	if (msg->has_multiply_digitized()) {
		_multiply_digitized = msg->multiply_digitized();		
	}
	else {
		_multiply_digitized.reset();
	}
	if (msg->has_urban_flag()) {
		_urban_flag = msg->urban_flag();
	}
	else {
		_urban_flag.reset();
	}
	if (msg->has_road_grade()) {
		_road_grade = msg->road_grade();
	}
	else {
		_road_grade.reset();
	}
	if (msg->has_net_grade()) {
		_net_grade = msg->net_grade();
	}
	else {
		_net_grade.reset();
	}
	if (msg->has_pavement_info()) {
		_pavement_info = msg->pavement_info();
	}
	else {
		_pavement_info.reset();
	}
	if (msg->has_pass_type()) {
		_pass_type = msg->pass_type();
	}
	else {
		_pass_type.reset();
	}
	if (msg->has_grade()) {
		_grade = msg->grade();
	}
	else {
		_grade.reset();
	}
	clear_changed();
	return true;
};

bool LinkExtProxy::merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) {
	bool bf = LinkProxy::merge(policy, mgr);
	return FeatureMerger<LinkExtProxy>(policy, mgr, this, "LinkExt")
										.merge(_lane_count, "lane_count")
										.merge(_left_lanes, "left_lanes")
										.merge(_right_lanes, "right_lanes")
										.merge(_multiply_digitized, "multiply_digitized")
										.merge(_urban_flag, "urban_flag")
										.merge(_road_grade, "road_grade")
										.merge(_net_grade, "net_grade")
										.merge(_pavement_info, "pavement_info")
										.merge(_pass_type, "pass_type")
										.merge(_grade, "grade")
										.is_conflict()
										|| bf;
};

PointProxy* NodeProxy::mutable_geom() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_geom) {
		_geom.reset(FeatureProxyBase::create_proxy<PointProxy>(this));
	}
	return _geom.get();
};
bool NodeProxy::clear_geom() {
	if (!is_editable()) {
		return false;
	}
	if (_geom) {
		mark_changed();
	}
	_geom.reset();
	return true;
};
/*bool NodeProxy::set_node_type(Optional<int> n) {
	if (!is_editable()) {
		return false;
	}
	if (_node_type != n) {
		mark_changed();
	}
	_node_type = n;
	return true;
};
bool NodeProxy::set_node_form(std::vector<int> n) {
	if (!is_editable()) {
		return false;
	}
	if (vector_equal(_node_form, n)) {
		mark_changed();
	}
	_node_form = n;
	return true;
};
bool NodeProxy::set_traffic_signal(Optional<int> t) {
	if (!is_editable()) {
		return false;
	}
	if (_traffic_signal != t) {
		mark_changed();
	}
	_traffic_signal = t;
	return true;
};
FeatureReference<NodeProxy>* NodeProxy::mutable_parent_nodeid() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_parent_nodeid;
};*/
FeatureReferenceVector<LinkProxy>* NodeProxy::mutable_link_ids() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_link_ids;
};
FeatureReference<JunctionProxy>* NodeProxy::mutable_junction_id() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_junction_id;
};

bool NodeProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::Node*>(msg_base);
	msg->Clear();
	id_to_message(msg);
	if (_geom) {
		_geom->to_message(msg->mutable_geom());
	}
	/*if (_node_type) {
		msg->set_node_type(*_node_type);
	}
	for (size_t i = 0; i < _node_form.size(); ++i) {
		msg->add_node_form(_node_form[i]);
	}
	if (_traffic_signal) {
		msg->set_traffic_signal(*_traffic_signal);
	}
	if (_parent_nodeid) {
		_parent_nodeid->to_message(msg->mutable_parent_nodeid());
	}*/
	if (_junction_id) {
		_junction_id->to_message(msg->mutable_junction_id());
	}
	_link_ids.to_message(msg->mutable_link_ids());
	return true;
};

Vector3D NodeProxy::wgs84_pos() const {
	Vector3D pos(0, 0, 0);
	int cnt = 0;
	if (!_id) {
		return pos;
	}
	if (_geom) {
		pos = *_geom;
		return pos;
	}
	for (auto& lid : _link_ids) {
		auto l = lid.proxy();
		if (!l) {
			continue;
		}
		if (l->snode_id() && l->snode_id()->is_equal(*_id) && l->geom() && l->geom()->pts().size() > 0) {
			pos += *(l->geom()->pts().front());
			cnt++;
		}
		else if (l->enode_id() && l->enode_id()->is_equal(*_id) && l->geom() && l->geom()->pts().size() > 0) {
			pos += *(l->geom()->pts().back());
			cnt++;
		}
	}
	if (cnt > 0) {
		_global_pos = pos / cnt;
		return _global_pos;
	}
	return pos;
};

const Vector3D& NodeProxy::global_pos(const RoadGeometryManager* mgr /* = nullptr */) const {
	_global_pos = { 0, 0, 0 };
	auto pos = wgs84_pos();	
	if (pos.Length() > 1e-4 && mgr->global_point_from_WGS84(pos)) {
		_global_pos = pos;
	}
	return _global_pos;
}

bool NodeProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::Node*>(msg_base);
	id_from_message(msg);
	if (msg->has_geom()) {
		_geom.reset(FeatureProxyBase::create_proxy<PointProxy>(this));
		_geom->from_message(&msg->geom());
	}
	else {
		_geom.reset();
	}
	/*if (msg->has_node_type()) {
		_node_type = msg->node_type();
	}
	else {
		_node_type.reset();
	}
	_node_form.clear();
	for (int i = 0; i < msg->node_form_size(); ++i) {
		_node_form.push_back(msg->node_form(i));
	}
	if (msg->has_traffic_signal()) {
		_traffic_signal = msg->traffic_signal();
	}
	else {
		_traffic_signal.reset();
	}
	if (msg->has_parent_nodeid()) {
		_parent_nodeid.create(this)->from_message(&msg->parent_nodeid());
	}
	else {
		_parent_nodeid.reset();
	}*/
	if (msg->has_junction_id()) {
		_junction_id.create(this)->from_message(&msg->junction_id());
	}
	else {
		_junction_id.reset();
	}
	_link_ids.from_message(msg->link_ids());
	clear_changed();
	return true;
};

bool NodeProxy::is_valid() const {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	if (!is_message_valid<NodeProxy>()) {
		LOG_BEGIN(ERROR) << "NodeProxy::is_valid(): failed for " << sid
			<< " : is_message_valid() failed!"; LOG_END;
		return false;
	}
	if (!is_editable() || is_deleted()) {
		///LOG_BEGIN(DEBUG) << "NodeProxy::is_valid() skip for " << sid; LOG_END;
		return true;
	}
	if (!is_ref_valid()) {
		LOG_BEGIN(ERROR) << "NodeProxy::is_valid() failed for " << sid
			<< " : has " << owner_count() << " owners"; LOG_END;
		return false;
	}
	if (_link_ids.size() < 1 || !_link_ids.is_valid(true)) {
		LOG_BEGIN(ERROR) << "NodeProxy::is_valid() failed for " << sid
			<< " : has " << _link_ids.size() << " connections"; LOG_END;
		return false;
	}
	return true;
};

bool NodeProxy::correct_tile_refs(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted() || !_id) {
		return false;
	}
	bool bc = false;
	std::set<int> ref_tiles;
	std::vector<const TileInfoProxy*> tiles;
	for (auto& l : _link_ids) {
		if (!l || !l.proxy()) {
			continue;
		}
		auto ts = l.proxy()->referenced_tiles();
		tiles.insert(tiles.end(), ts.begin(), ts.end());
	}
	/*if (_parent_nodeid) {
		auto t = mgr->get_road_tile(_parent_nodeid->tileid());
		if (t) {
			tiles.push_back(t.get());
		}
	}*/
	if (_junction_id) {
		auto t = mgr->get_road_tile(_junction_id->tileid());
		if (t) {
			tiles.push_back(t.get());
		}
	}
	for (auto t : tiles) {
		if (ref_tiles.count(t->tile_id()) > 0) {
			continue;
		}
		ref_tiles.insert(t->tile_id());
		auto tile = mgr->get_road_tile(t->tile_id());
		if (add_missed_ref_tiles(tile.get())) {
			FeatureReference<NodeProxy> e;
			e = mutable_changed_id(mgr->current_version());
			tile->node_refs().push_back(e);
			//LOG_BEGIN(INFO) << "NodeProxy::correct_tile_refs(): add tile "
			//	<< tile->tile_id() << " 's ref " << e->to_string(); LOG_END;
			bc = true;
		}
	}

	tiles = referenced_tiles();
	for (auto& tw : tiles) {
		auto tile = const_cast<TileInfoProxy*>(tw);
		if (tile && ref_tiles.count(tile->tile_id()) <= 0) {
			auto ref = tile->node_refs().find(*id());
			if (ref && ref->is_deleted()) {
				continue;
			}
			auto did = mutable_deleted_id(mgr->current_version());
			tile->node_refs().replace(id(), did);
			LOG_BEGIN(INFO) << "NodeProxy::correct_tile_refs(): remove tile "
				<< tile->tile_id() << " 's ref " << did->to_string(); LOG_END;
			bc = true;
		}
	}
	return bc;
};

bool NodeProxy::correct_content(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted()) {
		return false;
	}
	bool bc = false;
	if (_id) {
		if (!_id->is_type(feature_id_type())) {
			LOG_BEGIN(INFO) << "NodeProxy::correct_content() correct id->type:"
				<< _id->type() << " to " << feature_id_type(); LOG_END;
			_id->set_type(feature_id_type());
			bc = true;
		}
		bc |= _id->correct_content(mgr);
	}
	else {
		LOG_BEGIN(ERROR) << "NodeProxy::correct_content(): no id in element"; LOG_END;
		return false;
	}
	
	if (_link_ids.remove_invalid(mgr, true)) {
		mark_changed();
		LOG_BEGIN(INFO) << "NodeProxy::correct_content(): remove invalid link_ids for "
			<< _id->to_string() << ", " << _link_ids.size() << " left"; LOG_END;
		bc = true;
	}

	auto pos = wgs84_pos();
	auto np = std::dynamic_pointer_cast<NodeProxy>(shared_from_this());
	auto ptr = std::dynamic_pointer_cast<NodeProxy>(shared_from_this());
	TileInfoPtr nti;
	if (mgr->remake_id(np, pos, nti, true)) {
		nti->nodes().push_back(ptr);
		auto tile = mgr->get_road_tile(np->id()->tileid());
		tile->nodes().replace(ptr, np);
		auto tiles = referenced_tiles();
		for (auto& tw : tiles) {
			auto tile = const_cast<TileInfoProxy*>(tw);
			if (tile) {
				tile->node_refs().replace(ptr->id(), np->mutable_id());
				LOG_BEGIN(INFO) << "NodeProxy::correct_content(): replace tile "
					<< tile->tile_id() << " 's ref " << ptr->id()->to_string() << " to "
					<< np->id()->to_string(); LOG_END;
			}
		}
		bc = true;
	}

	bc |= correct_tile_refs(mgr);

	bc |= correct_version(mgr);

	if (!cached_is_valid()) {
		LOG_BEGIN(ERROR) << "NodeProxy::correct_content(): set deleted for "
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

bool NodeProxy::remake_proxy(RoadGeometryManager* mgr) {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	bool bf = true;
	/*if (_parent_nodeid) {
		auto p = _parent_nodeid->resolve_id(mgr);
		if (p) {
			_parent_nodeid = p->id();
		}
		else {
			LOG_BEGIN(INFO) << "NodeProxy::remake_proxy(): " << sid
				<< " has unresolved pareent_nodeid " << _parent_nodeid->to_string();
			LOG_END;
			bf = false;
		}
	}*/
	if (_junction_id) {
		auto p = _junction_id->resolve_id(mgr);
		if (p) {
			_junction_id = p->id();
		}
		else {
			LOG_BEGIN(INFO) << "NodeProxy::remake_proxy(): " << sid
				<< " has unresolved junction_id " << _junction_id->to_string();
			LOG_END;
			bf = false;
		}
	}
	if (!_link_ids.resolve_proxy(mgr)) {
		LOG_BEGIN(INFO) << "NodeProxy::remake_proxy(): " << sid
			<< " has unresolved links!"; LOG_END;
		return false;
	}
	return bf;
};

bool NodeProxy::merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) {
	return FeatureMerger<NodeProxy>(policy, mgr, this, "Node")
		.merge(_geom, "geom")
		.merge(_node_type, "node_type")
		.merge(_node_form, "node_form")
		.merge(_traffic_signal, "traffic_signal")
		//.merge(_parent_nodeid, "parent_nodeid")
		.merge(_junction_id, "junction_id")
		.merge_unordered(_link_ids, "link_ids")
		.is_conflict();
};
};

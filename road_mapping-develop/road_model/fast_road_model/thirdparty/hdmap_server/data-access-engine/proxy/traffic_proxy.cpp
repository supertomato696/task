#include "traffic_proxy.h"
#include "data_access_engine.h"
#include "manager/road_geometry_mgr.h"
#include "position_proxy.h"
#include "lane_proxy.h"
#include "tile_proxy.h"
#include "feature_merger.h"

namespace data_access_engine {

bool TrafficConditionProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::TrafficCondition*>(msg_base);
	if (msg->has_type()) {
		_type = msg->type();
	}
	else {
		_type.reset();
	}
	if (msg->has_value()) {
		_value = msg->value();
	}
	else {
		_value.reset();
	}
	if (msg->has_vehicle_type_mask()) {
		_vehicle_type_mask = msg->vehicle_type_mask();
	}
	else {
		_vehicle_type_mask.reset();
	}
	if (msg->has_load_type()) {
		_load_type = msg->load_type();
	}
	else {
		_load_type.reset();
	}
	if (msg->has_start_minues()) {
		_start_minues = msg->start_minues();
	}
	else {
		_start_minues.reset();
	}
	if (msg->has_end_minues()) {
		_end_minues = msg->end_minues();
	}
	else {
		_end_minues.reset();
	}
	if (msg->has_weather()) {
		_weather = msg->weather();
	}
	else {
		_weather.reset();
	}
	if (msg->has_direction()) {
		_direction = msg->direction();
	}
	else {
		_direction.reset();
	}
	clear_changed();
	return true;
};

bool TrafficConditionProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::TrafficCondition*>(msg_base);
	msg->Clear();
	if (_type) {
		msg->set_type(*_type);
	}
	if (_value) {
		msg->set_value(*_value);
	}
	if (_vehicle_type_mask) {
		msg->set_vehicle_type_mask(*_vehicle_type_mask);
	}
	if (_load_type) {
		msg->set_load_type(*_load_type);
	}
	if (_start_minues) {
		msg->set_start_minues(*_start_minues);
	}
	if (_end_minues) {
		msg->set_end_minues(*_end_minues);
	}
	if (_weather) {
		msg->set_weather(*_weather);
	}
	if (_fuzzy_time) {
		msg->set_fuzzy_time(*_fuzzy_time);
	}
	if (_direction) {
		msg->set_direction(*_direction);
	}
	return true;
};

bool TrafficConditionProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
										const std::shared_ptr<const FeatureProxyBase>& base,
										const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const TrafficConditionProxy>(remote);
	auto o = std::dynamic_pointer_cast<const TrafficConditionProxy>(base);
	return ProxyMerger<TrafficConditionProxy>(policy, mgr, this, "TrafficConditionProxy", r, o)
											.merge(_type, "type")
											.merge(_value, "value")
											.merge(_vehicle_type_mask, "vehicle_type_mask")
											.merge(_load_type, "load_type")
											.merge(_start_minues, "start_minues")
											.merge(_end_minues, "end_minues")
											.merge(_weather, "weather")
											.merge(_fuzzy_time, "fuzzy_time")
											.merge(_direction, "direction")
											.is_conflict();
};

bool TrafficConditionProxy::is_valid() const {
	if (!_type && !_value && !_vehicle_type_mask && !_load_type && !_start_minues
			&& _end_minues && !_weather && !_fuzzy_time && !_direction) {
		return false;
	}
	return true;
};

int TrafficConditionProxy::compare(const TrafficConditionProxy& rhs) const {
	int ret = value_compare(_type, rhs._type);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_value, rhs._value);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_vehicle_type_mask, rhs._vehicle_type_mask);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_load_type, rhs._load_type);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_start_minues, rhs._start_minues);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_end_minues, rhs._end_minues);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_weather, rhs._weather);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_fuzzy_time, rhs._fuzzy_time);
	if (ret != 0) {
		return ret;
	}
	return value_compare(_direction, rhs._direction);
};

bool TrafficSignProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::TrafficSign*>(msg_base);
	if (msg->has_type()) {
		_type = msg->type();
	}
	else {
		_type.reset();
	}
	if (msg->has_shape()) {
		_shape = msg->shape();
	}
	else {
		_shape.reset();
	}
	if (msg->has_value()) {
		_value = msg->value();
	}
	else {
		_value.reset();
	}
	if (msg->has_content()) {
		_content = msg->content();
	}
	else {
		_content.reset();
	}
	if (msg->has_variable()) {
		_variable = msg->variable();
	}
	else {
		_variable.reset();
	}
	_panels.from_message(msg->panels());
	clear_changed();
	return true;
};

bool TrafficSignProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::TrafficSign*>(msg_base);
	msg->Clear();
	if (_type) {
		msg->set_type(*_type);
	}
	if (_shape) {
		msg->set_shape(*_shape);
	}
	if (_value) {
		msg->set_value(*_value);
	}
	if (_content) {
		msg->set_content(*_content);
	}
	if (_variable) {
		msg->set_variable(*_variable);
	}
	_panels.to_message(msg->mutable_panels());
	return true;
};

bool TrafficSignProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
									const std::shared_ptr<const FeatureProxyBase>& base,
									const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const TrafficSignProxy>(remote);
	auto o = std::dynamic_pointer_cast<const TrafficSignProxy>(base);
	return ProxyMerger<TrafficSignProxy>(policy, mgr, this, "TrafficSignProxy", r, o)
										.merge(_type, "type")
										.merge(_shape, "shape")
										.merge(_value, "value")
										.merge(_content, "content")
										.merge(_variable, "variable")
										.merge_elemental(_panels, "panels")
										.is_conflict();
};

bool TrafficSignProxy::is_valid() const {
	if (!_type && !_value && !_shape && !_content && !_variable) {
		return false;
	}
	return _panels.is_valid();
};

int TrafficSignProxy::compare(const TrafficSignProxy& rhs) const {
	int ret = value_compare(_type, rhs._type);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_shape, rhs._shape);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_value, rhs._value);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_content, rhs._content);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_variable, rhs._variable);
	if (ret != 0) {
		return ret;
	}
	return proxys_compare(_panels, rhs._panels);
};

/*bool LightBulbProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::TrafficLight_LightBulb*>(msg_base);
	if (msg->has_color()) {
		_color = msg->color();
	}
	else {
		_color.reset();
	}
	if (msg->has_shape()) {
		_shape = msg->shape();
	}
	else {
		_shape.reset();
	}
	if (msg->has_border()) {
		_border.reset(FeatureProxyBase::create_proxy<CircleProxy>(this));
		_border->from_message(&msg->border());
	}
	clear_changed();
	return true;
};

bool LightBulbProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::TrafficLight_LightBulb*>(msg_base);
	msg->Clear();
	if (_color) {
		msg->set_color(*_color);
	}
	if (_shape) {
		msg->set_shape(*_shape);
	}
	if (_border) {
		_border->to_message(msg->mutable_border());
	}
	return true;
};

bool LightBulbProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
									const std::shared_ptr<const FeatureProxyBase>& base,
									const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const LightBulbProxy>(remote);
	auto o = std::dynamic_pointer_cast<const LightBulbProxy>(base);
	return ProxyMerger<LightBulbProxy>(policy, mgr, this, "LightBulbProxy", r, o)
										.merge(_color, "color")
										.merge(_shape, "shape")
										.merge(_border, "border")
										.is_conflict();
};

bool LightBulbProxy::is_valid() const {
	if (!_border) {
		return false;
	}
	return _border->is_valid();
};

int LightBulbProxy::compare(const LightBulbProxy& rhs) const {
	int ret = value_compare(_color, rhs._color);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_shape, rhs._shape);
	if (ret != 0) {
		return ret;
	}
	return proxy_compare(_border, rhs._border);
};*/

bool TrafficLightProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::TrafficLight*>(msg_base);
	if (msg->has_type()) {
		_type = msg->type();
	}
	else {
		_type.reset();
	}
	//_bulbs.from_message(msg->bulbs());
	if (msg->has_arrange()) {
		_arrange = msg->arrange();
	}
	else {
		_arrange.reset();
	}
	/*if (msg->has_usage_obj()) {
		_usage_obj = msg->usage_obj();
	}
	else {
		_usage_obj.reset();
	}
	if (msg->has_usage_type()) {
		_usage_type = msg->usage_type();
	}
	else {
		_usage_type.reset();
	}*/
	if (msg->has_bulbs_num()) {
		_bulbs_num = msg->bulbs_num();
	}
	else {
		_bulbs_num.reset();
	}
	clear_changed();
	return true;
};

bool TrafficLightProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::TrafficLight*>(msg_base);
	msg->Clear();
	if (_type) {
		msg->set_type(*_type);
	}
	//_bulbs.to_message(msg->mutable_bulbs());
	if (_arrange) {
		msg->set_arrange(*_arrange);
	}
	/*if (_usage_obj) {
		msg->set_usage_obj(*_usage_obj);
	}
	if (_usage_type) {
		msg->set_usage_type(*_usage_type);
	}*/
	if (_bulbs_num) {
		msg->set_bulbs_num(*_bulbs_num);
	}
	return true;
};

bool TrafficLightProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
									const std::shared_ptr<const FeatureProxyBase>& base,
									const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const TrafficLightProxy>(remote);
	auto o = std::dynamic_pointer_cast<const TrafficLightProxy>(base);
	return ProxyMerger<TrafficLightProxy>(policy, mgr, this, "TrafficLightProxy", r, o)
											.merge(_type, "type")
											//.merge_elemental(_bulbs, "bulbs")
											.merge(_arrange, "arrange")
											//.merge(_usage_obj, "usage_obj")
											//.merge(_usage_type, "usage_type")
											.merge(_bulbs_num, "bulbs_num")
											.is_conflict();
};

bool TrafficLightProxy::is_valid() const {
	//return _bulbs.is_valid();
	return true;
};

int TrafficLightProxy::compare(const TrafficLightProxy& rhs) const {
	int ret = value_compare(_type, rhs._type);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_arrange, rhs._arrange);
	if (ret != 0) {
		return ret;
	}
	/*ret = value_compare(_usage_obj, rhs._usage_obj);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_usage_type, rhs._usage_type);
	if (ret != 0) {
		return ret;
	}
	return proxys_compare(_bulbs, rhs._bulbs);*/
	return value_compare(_bulbs_num, rhs._bulbs_num);
};

/*bool TrafficLightGroupProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::TrafficLightGroup*>(msg_base);
	_lights.from_message(msg->lights());
	clear_changed();
	return true;
};

bool TrafficLightGroupProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::TrafficLightGroup*>(msg_base);
	msg->Clear();
	_lights.to_message(msg->mutable_lights());
	return true;
};

bool TrafficLightGroupProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
	const std::shared_ptr<const FeatureProxyBase>& base,
	const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const TrafficLightGroupProxy>(remote);
	auto o = std::dynamic_pointer_cast<const TrafficLightGroupProxy>(base);
	return ProxyMerger<TrafficLightGroupProxy>(policy, mgr, this, "TrafficLightGroupProxy", r, o)
		.merge_unordered(_lights, "lights")
		.is_conflict();
};

bool TrafficLightGroupProxy::is_valid() const {
	if (_lights.empty()) {
		return false;
	}
	return _lights.is_valid(true);
};

int TrafficLightGroupProxy::compare(const TrafficLightGroupProxy& rhs) const {
	return refs_compare(_lights, rhs._lights);
};*/

bool LaneMarkingProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::LaneMarking*>(msg_base);
	if (msg->has_type()) {
		_type = msg->type();
	}
	else {
		_type.reset();
	}
	if (msg->has_value()) {
		_value = msg->value();
	}
	else {
		_value.reset();
	}
	if (msg->has_content()) {
		_content = msg->content();
	}
	else {
		_content.reset();
	}
	clear_changed();
	return true;
};

bool LaneMarkingProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::LaneMarking*>(msg_base);
	msg->Clear();
	if (_type) {
		msg->set_type(*_type);
	}
	if (_value) {
		msg->set_value(*_value);
	}
	if (_content) {
		msg->set_content(*_content);
	}
	return true;
};

bool LaneMarkingProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
									const std::shared_ptr<const FeatureProxyBase>& base,
									const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const LaneMarkingProxy>(remote);
	auto o = std::dynamic_pointer_cast<const LaneMarkingProxy>(base);
	return ProxyMerger<LaneMarkingProxy>(policy, mgr, this, "LaneMarkingProxy", r, o)
										.merge(_type, "type")
										.merge(_value, "value")
										.merge(_content, "content")
										.is_conflict();
};

bool LaneMarkingProxy::is_valid() const {
	if (!_type) {
		return false;
	}
	return true;
};

int LaneMarkingProxy::compare(const LaneMarkingProxy& rhs) const {
	int ret = value_compare(_type, rhs._type);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_value, rhs._value);
	if (ret != 0) {
		return ret;
	}
	return value_compare(_content, rhs._content);
};

bool SpeedBumpProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::SpeedBump*>(msg_base);
	if (msg->has_height()) {
		_height = msg->height();
	}
	else {
		_height.reset();
	}
	/*if (msg->has_speed_limit()) {
		_speed_limit = msg->speed_limit();
	}
	else {
		_speed_limit.reset();
	}*/
	clear_changed();
	return true;
};

bool SpeedBumpProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::SpeedBump*>(msg_base);
	msg->Clear();
	if (_height) {
		msg->set_height(*_height);
	}
	/*if (_speed_limit) {
		msg->set_speed_limit(*_speed_limit);
	}*/
	return true;
};

bool SpeedBumpProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
								const std::shared_ptr<const FeatureProxyBase>& base,
								const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const SpeedBumpProxy>(remote);
	auto o = std::dynamic_pointer_cast<const SpeedBumpProxy>(base);
	return ProxyMerger<SpeedBumpProxy>(policy, mgr, this, "SpeedBumpProxy", r, o)
										.merge(_height, "height")
										//.merge(_speed_limit, "speed_limit")
										.is_conflict();
};

bool SpeedBumpProxy::is_valid() const {
	/*if (!_height && !_speed_limit) {
		return false;
	}*/
	return true;
};

int SpeedBumpProxy::compare(const SpeedBumpProxy& rhs) const {
	int ret = value_compare(_height, rhs._height);
	//if (ret != 0) {
		return ret;
	//}
	//return value_compare(_speed_limit, rhs._speed_limit);
};

FeatureReferenceVector<PositionObjectProxy>* TrafficInfoProxy::mutable_objs() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_objs;
};
bool TrafficInfoProxy::set_type(Optional<int> t) {
	if (!is_editable()) {
		return false;
	}
	if (_type != t) {
		mark_changed();
	}
	_type = t;
	return true;
};
FeatureReference<LaneProxy>* TrafficInfoProxy::mutable_lanes() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_lanes;
};
FeatureReference<LaneGroupProxy>* TrafficInfoProxy::mutable_lanegroups() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_lanegroups;
};
TrafficSignProxy* TrafficInfoProxy::mutable_sign() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_sign) {
		_sign.reset(FeatureProxyBase::create_proxy<TrafficSignProxy>(this));
	}
	return _sign.get();
};
bool TrafficInfoProxy::clear_sign() {
	if (!is_editable()) {
		return false;
	}
	if (_sign) {
		mark_changed();
	}
	_sign.reset();
	return true;
};
TrafficLightProxy* TrafficInfoProxy::mutable_light() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_light) {
		_light.reset(FeatureProxyBase::create_proxy<TrafficLightProxy>(this));
	}
	return _light.get();
};
bool TrafficInfoProxy::clear_light() {
	if (!is_editable()) {
		return false;
	}
	if (_light) {
		mark_changed();
	}
	_light.reset();
	return true;
};
/*TrafficLightGroupProxy* TrafficInfoProxy::mutable_light_group() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_light_group) {
		_light_group.reset(FeatureProxyBase::create_proxy<TrafficLightGroupProxy>(this));
	}
	return _light_group.get();
};
bool TrafficInfoProxy::clear_light_group() {
	if (!is_editable()) {
		return false;
	}
	if (_light_group) {
		mark_changed();
	}
	_light_group.reset();
	return true;
};*/
LaneMarkingProxy* TrafficInfoProxy::mutable_marking() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_marking) {
		_marking.reset(FeatureProxyBase::create_proxy<LaneMarkingProxy>(this));
	}
	return _marking.get();
};
bool TrafficInfoProxy::clear_marking() {
	if (!is_editable()) {
		return false;
	}
	if (_marking) {
		mark_changed();
	}
	_marking.reset();
	return true;
};
SpeedBumpProxy* TrafficInfoProxy::mutable_bump() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_bump) {
		_bump.reset(FeatureProxyBase::create_proxy<SpeedBumpProxy>(this));
	}
	return _bump.get();
};
bool TrafficInfoProxy::clear_bump() {
	if (!is_editable()) {
		return false;
	}
	if (_bump) {
		mark_changed();
	}
	_bump.reset();
	return true;
};

Vector3D TrafficInfoProxy::wgs84_pos() const {
	for (size_t i = 0; i < _objs.size(); ++i) {
		auto& o = _objs[i].proxy();
		if (o) {
			return o->wgs84_pos();
		}
	}
	if (_lanes && _lanes.proxy()) {
		return _lanes.proxy()->wgs84_pos();
	}
	if (_lanegroups && _lanegroups.proxy()) {
		return _lanegroups.proxy()->wgs84_pos();
	}
	return Vector3D(0, 0, 0);
};
const Vector3D& TrafficInfoProxy::global_pos(const RoadGeometryManager* mgr) const {	
	static Vector3D dummy(0, 0, 0);
	for (size_t i = 0; i < _objs.size(); ++i) {
		auto& o = _objs[i].proxy();
		if (o) {
			return o->global_pos(mgr);
		}
	}
	if (_lanes && _lanes.proxy()) {
		return _lanes.proxy()->global_pos(mgr);
	}
	if (_lanegroups && _lanegroups.proxy()) {
		return _lanegroups.proxy()->global_pos(mgr);
	}
	return dummy;
};

bool TrafficInfoProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::TrafficInfo*>(msg_base);
	msg->Clear();
	id_to_message(msg);
	_objs.to_message(msg->mutable_objs());
	if (_type) {
		msg->set_type(*_type);
	}
	if (_lanes) {
		_lanes->to_message(msg->mutable_lanes());
	}
	if (_lanegroups) {
		_lanegroups->to_message(msg->mutable_lanegroups());
	}
	if (_sign) {
		_sign->to_message(msg->mutable_sign());
	}
	if (_light) {
		_light->to_message(msg->mutable_light());
	}
	/*if (_light_group) {
		_light_group->to_message(msg->mutable_light_group());
	}*/
	if (_marking) {
		_marking->to_message(msg->mutable_marking());
	}
	if (_bump) {
		_bump->to_message(msg->mutable_bump());
	}
	return true;
};

bool TrafficInfoProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::TrafficInfo*>(msg_base);
	id_from_message(msg);
	_objs.from_message(msg->objs());
	if (msg->has_type()) {
		_type = msg->type();
	}
	else {
		_type.reset();
	}
	if (msg->has_lanes()) {
		_lanes.create(this)->from_message(&msg->lanes());
	}
	else {
		_lanes.reset();
	}
	if (msg->has_lanegroups()) {
		_lanegroups.create(this)->from_message(&msg->lanegroups());
	}
	else {
		_lanegroups.reset();
	}
	if (msg->has_sign()) {
		_sign.reset(FeatureProxyBase::create_proxy<TrafficSignProxy>(this));
		_sign->from_message(&msg->sign());
	}
	else {
		_sign.reset();
	}
	if (msg->has_light()) {
		_light.reset(FeatureProxyBase::create_proxy<TrafficLightProxy>(this));
		_light->from_message(&msg->light());
	}
	else {
		_light.reset();
	}
	/*if (msg->has_light_group()) {
		_light_group.reset(FeatureProxyBase::create_proxy<TrafficLightGroupProxy>(this));
		_light_group->from_message(&msg->light_group());
	}
	else {
		_light_group.reset();
	}*/
	if (msg->has_marking()) {
		_marking.reset(FeatureProxyBase::create_proxy<LaneMarkingProxy>(this));
		_marking->from_message(&msg->marking());
	}
	else {
		_marking.reset();
	}
	if (msg->has_bump()) {
		_bump.reset(FeatureProxyBase::create_proxy<SpeedBumpProxy>(this));
		_bump->from_message(&msg->bump());
	}
	else {
		_bump.reset();
	}
	clear_changed();
	return true;
};

bool TrafficInfoProxy::is_valid() const {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	if (!is_message_valid<TrafficInfoProxy>()) {
		LOG_BEGIN(ERROR) << "TrafficInfoProxy::is_valid(): failed for " << sid
			<< " : is_message_valid() failed!"; LOG_END;
		return false;
	}
	if (!is_editable() || is_deleted()) {
		//LOG_BEGIN(DEBUG) << "TrafficInfoProxy::is_valid() skip for " << sid; LOG_END;
		return true;
	}
	if (!is_ref_valid()) {
		LOG_BEGIN(ERROR) << "TrafficInfoProxy::is_valid() failed for " << sid
			<< " : has " << owner_count() << " owners"; LOG_END;
		return false;
	}
	if (_objs.empty() && !_lanes.proxy() && !_lanegroups.proxy()) {
		LOG_BEGIN(ERROR) << "TrafficinfoProxy::is_valid() failed for " << sid
			<< " : has empty traffics!"; LOG_END;
		return false;
	}

	if (!_objs.is_valid(true)) {
		LOG_BEGIN(ERROR) << "TrafficInfoProxy::is_valid() failed for " << sid
			<< " : has invalid objs " << _objs.size(); LOG_END;
		return false;
	}
	if (_sign && !_sign->is_valid()) {
		LOG_BEGIN(ERROR) << "TrafficInfoProxy::is_valid() failed for " << sid
			<< " : has invalid sign"; LOG_END;
		return false;
	}
	if (_light && !_light->is_valid()) {
		LOG_BEGIN(ERROR) << "TrafficInfoProxy::is_valid() failed for " << sid
			<< " : has invalid light "; LOG_END;
		return false;
	}
	/*if (_light_group && !_light_group->is_valid()) {
		LOG_BEGIN(ERROR) << "TrafficInfoProxy::is_valid() failed for " << sid
			<< " : has invalid light_group " << _light_group->lights().size(); LOG_END;
		return false;
	}*/
	if (_marking && !_marking->is_valid()) {
		LOG_BEGIN(ERROR) << "TrafficInfoProxy::is_valid() failed for " << sid
			<< " : has invalid marking "; LOG_END;
		return false;
	}
	if (_bump && !_bump->is_valid()) {
		LOG_BEGIN(ERROR) << "TrafficInfoProxy::is_valid() failed for " << sid
			<< " : has invalid bump "; LOG_END;
		return false;
	}
	return true;
};

bool TrafficInfoProxy::correct_tile_refs(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted() || _id) {
		return false;
	}
	bool bc = false;
	std::set<int> ref_tiles;
	std::vector<const TileInfoProxy*> tiles;
	for (auto& l : _objs) {
		if (!l || !l.proxy()) {
			continue;
		}
		auto ts = l.proxy()->referenced_tiles();
		tiles.insert(tiles.end(), ts.begin(), ts.end());
	}
	if (_lanes && _lanes.proxy()) {
		auto ts = _lanes.proxy()->referenced_tiles();
		tiles.insert(tiles.end(), ts.begin(), ts.end());
	}
	if (_lanegroups && _lanegroups.proxy()) {
		auto ts = _lanegroups.proxy()->referenced_tiles();
		tiles.insert(tiles.end(), ts.begin(), ts.end());
	}
	for (auto t : tiles) {
		if (ref_tiles.count(t->tile_id()) > 0) {
			continue;
		}
		ref_tiles.insert(t->tile_id());
		auto tile = mgr->get_road_tile(t->tile_id());
		if (add_referenced_tiles(tile.get())) {
			FeatureReference<TrafficInfoProxy> e;
			e = mutable_changed_id(mgr->current_version());
			tile->traffic_info_refs().push_back(e);
			LOG_BEGIN(INFO) << "TrafficInfoProxy::correct_tile_refs(): add tile "
				<< tile->tile_id() << " 's ref " << e->to_string(); LOG_END;
			bc = true;
		}
	}

	tiles = referenced_tiles();
	for (auto& tw : tiles) {
		auto tile = const_cast<TileInfoProxy*>(tw);
		if (tile && ref_tiles.count(tile->tile_id()) <= 0) {
			auto ref = tile->traffic_info_refs().find(*id());
			if (ref && ref->is_deleted()) {
				continue;
			}
			auto did = mutable_deleted_id(mgr->current_version());
			tile->traffic_info_refs().replace(id(), did);
			LOG_BEGIN(INFO) << "TrafficInfoProxy::correct_tile_refs(): remove tile "
				<< tile->tile_id() << " 's ref " << did->to_string(); LOG_END;
			bc = true;
		}
	}
	return bc;
};

bool TrafficInfoProxy::correct_content(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted()) {
		return false;
	}
	bool bc = false;
	if (_id) {
		if (!_id->is_type(feature_id_type())) {
			LOG_BEGIN(INFO) << "TrafficInfoProxy::correct_content() correct id->type:"
				<< _id->type() << " to " << feature_id_type(); LOG_END;
			_id->set_type(feature_id_type());
			bc = true;
		}
		bc |= _id->correct_content(mgr);
	}
	else {
		LOG_BEGIN(ERROR) << "TrafficInfoProxy::correct_content(): no id in element"; LOG_END;
		return false;
	}
	if (_objs.remove_invalid(mgr, true)) {
		mark_changed();
		LOG_BEGIN(INFO) << "TrafficInfoProxy::correct_content(): correct objs for "
			<< _id->to_string() << ", " << _objs.size() << " left"; LOG_END;
		bc = true;
	}
	/*if (_light_group && _light_group->mutable_lights()->remove_invalid(mgr, true)) {
		mark_changed();
		LOG_BEGIN(INFO) << "TrafficInfoProxy::correct_content(): remove invalid light_group for "
			<< _id->to_string() << ", " << _light_group->lights().size() << " left"; LOG_END;
		bc = true;
	}*/
	
	if (_objs.empty() && !_lanes && !_lanegroups) {
		LOG_BEGIN(ERROR) << "TrafficInfoProxy::correct_content(): set deleted for "
			<< _id->to_string() << ": traffic is empty"; LOG_END;
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
	auto np = std::dynamic_pointer_cast<TrafficInfoProxy>(shared_from_this());
	auto ptr = std::dynamic_pointer_cast<TrafficInfoProxy>(shared_from_this());
	TileInfoPtr nti;
	if (mgr->remake_id(np, pos, nti, true)) {
		nti->traffic_infos().push_back(ptr);
		auto tile = mgr->get_road_tile(np->id()->tileid());
		tile->traffic_infos().replace(ptr, np);
		auto tiles = referenced_tiles();
		for (auto& tw : tiles) {
			auto tile = const_cast<TileInfoProxy*>(tw);
			if (tile) {
				tile->traffic_info_refs().replace(ptr->id(), np->mutable_id());
				LOG_BEGIN(INFO) << "TrafficInfoProxy::correct_content(): replace tile "
					<< tile->tile_id() << " 's ref " << ptr->id()->to_string() << " to "
					<< np->id()->to_string(); LOG_END;
			}
		}
		bc = true;
	}

	bc |= correct_tile_refs(mgr);

	bc |= correct_version(mgr);

	if (!cached_is_valid()) {
		LOG_BEGIN(ERROR) << "TrafficInfoProxy::correct_content(): set deleted for "
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

bool TrafficInfoProxy::remake_proxy(RoadGeometryManager* mgr) {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	bool bf = true;
	if (_lanes) {
		auto p = mgr->get_lane(*_lanes.id());
		if (p) {
			_lanes.set(p->id());
		}
		else {
			LOG_BEGIN(INFO) << "TrafficInfoProxy::remake_proxy(): " << sid
				<< " has unresolved lanes " << _lanes->to_string();
			LOG_END;
			bf = false;
		}
	}
	if (_lanegroups) {
		auto p = mgr->get_lane(*_lanegroups.id());
		if (p) {
			_lanegroups.set(p->id());
		}
		else {
			LOG_BEGIN(INFO) << "TrafficInfoProxy::remake_proxy(): " << sid
				<< " has unresolved lanegroups " << _lanegroups->to_string();
			LOG_END;
			bf = false;
		}
	}
	if (!_objs.resolve_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "TrafficInfoProxy::remake_proxy(): " << sid
			<< " has unresolved objs!"; LOG_END;
	}
	/*if (_light_group && !_light_group->mutable_lights()->resolve_proxy(mgr)) {
		bf = false;
		LOG_BEGIN(INFO) << "TrafficInfoProxy::remake_proxy(): " << sid
			<< " has unresolved light_group!"; LOG_END;
	}*/
	return bf;
};

bool TrafficInfoProxy::merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) {
	return FeatureMerger<TrafficInfoProxy>(policy, mgr, this, "TrafficInfo")
											.merge(_objs, "objs", false)
											.merge(_type, "type")
											.merge(_lanes, "lanes", false)
											.merge(_lanegroups, "lanegroups", false)
											.merge(_sign, "sign")
											.merge(_light, "light")
											//.merge(_light_group, "light_group")
											.merge(_marking, "marking")
											.merge(_bump, "bump")
											.is_conflict();
};

}; // data_access_engine
#include "confidence_proxy.h"
#include "data_access_engine.h"
#include "manager/road_geometry_mgr.h"
#include "tile_proxy.h"
#include "feature_merger.h"

namespace data_access_engine {

bool DataQualityProxy::set_data_source(Optional<int> s) {
	if (!is_editable()) {
		return false;
	}
	if (_data_source != s) {
		mark_changed();
	}
	_data_source = s;
	return true;
};

bool DataQualityProxy::set_data_weather(Optional<int> w) {
	if (!is_editable()) {
		return false;
	}
	if (_data_weather != w) {
		mark_changed();
	}
	_data_weather = w;
	return true;
};

bool DataQualityProxy::set_sensor_type(Optional<int> s) {
	if (!is_editable()) {
		return false;
	}
	if (_sensor_type != s) {
		mark_changed();
	}
	_sensor_type = s;
	return true;
};

bool DataQualityProxy::set_work_manner(Optional<int> w) {
	if (!is_editable()) {
		return false;
	}
	if (_work_manner != w) {
		mark_changed();
	}
	_work_manner = w;
	return true;
};

bool DataQualityProxy::set_create_time(Optional<int64_t> c) {
	if (!is_editable()) {
		return false;
	}
	if (_create_time != c) {
		mark_changed();
	}
	_create_time = c;
	return true;
};

bool DataQualityProxy::set_confidence(Optional<int> c) {
	if (!is_editable()) {
		return false;
	}
	if (_confidence != c) {
		mark_changed();
	}
	_confidence = c;
	return true;
};

FeatureReferenceAny* DataQualityProxy::mutable_feat_id() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_feat_id;
};

bool DataQualityProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::DataQuality*>(msg_base);
	msg->Clear();
	id_to_message(msg);
	if (_data_source) {
		msg->set_data_source(*_data_source);
	}
	if (_data_weather) {
		msg->set_data_weather(*_data_weather);
	}
	if (_sensor_type) {
		msg->set_sensor_type(*_sensor_type);
	}
	if (_work_manner) {
		msg->set_work_manner(*_work_manner);
	}
	if (_create_time) {
		msg->set_create_time(*_create_time);
	}
	if (_confidence) {
		msg->set_confidence(*_confidence);
	}
	if (_feat_id) {
		_feat_id->to_message(msg->mutable_feat_id());
	}
	return true;
};

bool DataQualityProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::DataQuality*>(msg_base);
	id_from_message(msg);
	if (msg->has_data_source()) {
		_data_source = msg->data_source();
	}
	else {
		_data_source.reset();
	}
	if (msg->has_data_weather()) {
		_data_weather = msg->data_weather();
	}
	else {
		_data_weather.reset();
	}
	if (msg->has_sensor_type()) {
		_sensor_type = msg->sensor_type();
	}
	else {
		_sensor_type.reset();
	}
	if (msg->has_work_manner()) {
		_work_manner = msg->work_manner();
	}
	else {
		_work_manner.reset();
	}
	if (msg->has_create_time()) {
		_create_time = msg->create_time();
	}
	else {
		_create_time.reset();
	}
	if (msg->has_confidence()) {
		_confidence = msg->confidence();
	}
	else {
		_confidence.reset();
	}
	if (msg->has_feat_id()) {
		_feat_id.create(this)->from_message(&msg->feat_id());
	}
	else {
		_feat_id.reset();
	}
	clear_changed();
	return true;
};

bool DataQualityProxy::is_valid() const {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	if (!is_message_valid<DataQualityProxy>()) {
		LOG_BEGIN(ERROR) << "DataQualityProxy::is_valid() failed for " << sid
			<< " : is_message_valid() failed!"; LOG_END;
		return false;
	}
	if (!is_editable() || is_deleted()) {
		//LOG_BEGIN(DEBUG) << "DataQualityProxy::is_valid() skip for " << sid; LOG_END;
		return true;
	}
	if (!is_ref_valid()) {
		LOG_BEGIN(ERROR) << "DataQualityProxy::is_valid() failed for " << sid
			<< " : has " << owner_count() << " owners"; LOG_END;
		return false;
	}
	if (_feat_id && !_feat_id->cached_is_valid()) {
		LOG_BEGIN(ERROR) << "DataQualityProxy::is_valid() failed: feat_id "
			<< _feat_id->to_string() << " is invalid"; LOG_END;
		return false;
	}	
	return true;
};

bool DataQualityProxy::is_editable() const {
	if (_feat_id && _feat_id.id()) {
		return _feat_id.id()->container()->is_editable();
	}	
	return true;
};

bool DataQualityProxy::correct_tile_refs(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted() || !_id) {
		return false;
	}
	bool bc = false;
	std::vector<const TileInfoProxy*> tiles;
	if (_feat_id && _feat_id.id()->container()) {
		auto cont = const_cast<FeatureWithIDProxyBase*>(_feat_id.id()->container());
		tiles = cont->referenced_tiles();
	}	
	else {
		return bc;
	}
	std::set<int> ref_tiles;
	for (auto t : tiles) {
		auto tile = const_cast<TileInfoProxy*>(t);
		if (tile == nullptr) {
			continue;
		}
		ref_tiles.insert(tile->tile_id());
		if (add_missed_ref_tiles(tile)) {
			FeatureReference<DataQualityProxy> e;
			e = mutable_changed_id(mgr->current_version());
			tile->data_quality_refs().push_back(e);
			LOG_BEGIN(INFO) << "DataQualityProxy::correct_tile_refs(): add tile "
				<< tile->tile_id() << " 's ref " << e->to_string(); LOG_END;
			bc = true;
		}
	}
	auto ts = referenced_tiles();
	for (auto& tw : ts) {
		auto tile = const_cast<TileInfoProxy*>(tw);
		if (tile && ref_tiles.count(tile->tile_id()) <= 0) {
			auto ref = tile->data_quality_refs().find(*id());
			if (ref && ref->is_deleted()) {
				continue;
			}
			auto did = mutable_deleted_id(mgr->current_version());
			tile->data_quality_refs().replace(id(), did);
			LOG_BEGIN(INFO) << "DataQualityProxy::correct_tile_refs(): remove tile "
				<< tile->tile_id() << " 's ref " << did->to_string(); LOG_END;
			bc = true;
		}
	}
	return bc;
}

bool DataQualityProxy::correct_content(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted()) {
		return false;
	}
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	if (!_feat_id) {
		LOG_BEGIN(INFO) << "DataQualityProxy::correct_content() failed for " << sid
			<< " : no road"; LOG_END;
		set_deleted();
		return true;
	}

	return correct_tile_refs(mgr);
};

bool DataQualityProxy::remake_proxy(RoadGeometryManager* mgr) {
	bool bf = true;
	if (_feat_id) {
		auto p = _feat_id->resolve_id(mgr);
		if (p) {
			_feat_id = p->id();
		}
		else {
			bf = false;
			LOG_BEGIN(INFO) << "DataQualityProxy::remake_proxy(): failed for "
				<< _feat_id->to_string(); LOG_END;
		}
	}	
	return bf;
};

bool DataQualityProxy::merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) {
	return FeatureMerger<DataQualityProxy>(policy, mgr, this, "DataQuality")
			.merge(_data_source, "data_source")
			.merge(_data_weather, "data_weather")
			.merge(_sensor_type, "sensor_type")
			.merge(_work_manner, "work_manner")
			.merge(_confidence, "confidence")
			.merge(_feat_id, "feat_id")
			.is_conflict();
};

};
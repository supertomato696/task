#include "dynamics_proxy.h"
#include "data_access_engine.h"
#include "manager/road_geometry_mgr.h"
#include "tile_proxy.h"
#include "feature_merger.h"

namespace data_access_engine {

bool RelationProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::Dynamic_Relation*>(msg_base);
	if (msg->has_id()) {
		_id.create(const_cast<FeatureWithIDProxyBase*>(container()))->from_message(&msg->id());
	}
	else {
		_id.reset();
	}
	if (msg->has_stp()) {
		_stp->from_message(&msg->stp());
	}
	else {
		_stp.reset();
	}
	if (msg->has_edp()) {
		_edp->from_message(&msg->edp());
	}
	else {
		_edp.reset();
	}
	clear_changed();
	return true;
};

bool RelationProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::Dynamic_Relation*>(msg_base);
	msg->Clear();
	if (_id) {
		_id.id()->to_message(msg->mutable_id());
	}
	if (_stp) {
		_stp->to_message(msg->mutable_stp());
	}
	if (_edp) {
		_edp->to_message(msg->mutable_edp());
	}
	return true;
};

bool RelationProxy::is_valid() const {
	if (!_id) {
		return false;
	}
	return true;
};

int RelationProxy::compare(const RelationProxy& rhs) const {
	int ret = proxy_compare(_id.id(), rhs._id.id());
	if (ret != 0) {
		return ret;
	}
	ret = proxy_compare(_stp, rhs._stp);
	if (ret != 0) {
		return ret;
	}
	return proxy_compare(_edp, rhs._edp);
};

FeatureReferenceAny* RelationProxy::mutable_id() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_id) {
		_id.create(const_cast<FeatureWithIDProxyBase*>(container()));
	}
	return &_id;
};

PointProxy* RelationProxy::mutable_stp() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_stp) {
		_stp.reset(FeatureProxyBase::create_proxy<PointProxy>(this));
	}
	return _stp.get();
};

bool RelationProxy::clear_stp() {
	if (!is_editable()) {
		return false;
	}
	if (_stp) {
		mark_changed();
	}
	_stp.reset();
	return true;
};

PointProxy* RelationProxy::mutable_edp() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_edp) {
		_edp.reset(FeatureProxyBase::create_proxy<PointProxy>(this));
	}
	return _edp.get();
};

bool RelationProxy::clear_edp() {
	if (!is_editable()) {
		return false;
	}
	if (_edp) {
		mark_changed();
	}
	_edp.reset();
	return true;
};

bool RelationProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
	const std::shared_ptr<const FeatureProxyBase>& base,
	const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const RelationProxy>(remote);
	auto o = std::dynamic_pointer_cast<const RelationProxy>(base);
	return ProxyMerger<RelationProxy>(policy, mgr, this, "RelationProxy", r, o)
		.merge(_id, "id")
		.merge(_stp, "stp")
		.merge(_edp, "edp")
		.is_conflict();
};

bool DynamicProxy::set_is_odd(Optional<bool> o) {
	if (!is_editable()) {
		return false;
	}
	if (_is_odd != o) {
		mark_changed();
	}
	_is_odd = o;
	return true;
};

CircleProxy* DynamicProxy::mutable_range() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_range) {
		_range.reset(FeatureProxyBase::create_proxy<CircleProxy>(this));
	}
	return _range.get();
};
bool DynamicProxy::clear_range() {
	if (!is_editable()) {
		return false;
	}
	if (_range) {
		mark_changed();
	}
	_range.reset();
	return true;
};

PolylineProxy* DynamicProxy::mutable_lines() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_lines) {
		_lines.reset(FeatureProxyBase::create_proxy<PolylineProxy>(this));
	}
	return _lines.get();
};
bool DynamicProxy::clear_lines() {
	if (!is_editable()) {
		return false;
	}
	if (_lines) {
		mark_changed();
	}
	_lines.reset();
	return true;
};

PolygonProxy* DynamicProxy::mutable_area() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_area) {
		_area.reset(FeatureProxyBase::create_proxy<PolygonProxy>(this));
	}
	return _area.get();
};
bool DynamicProxy::clear_area() {
	if (!is_editable()) {
		return false;
	}
	if (_area) {
		mark_changed();
	}
	_area.reset();
	return true;
};

RelationProxy* DynamicProxy::mutable_relation() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_relation) {
		_relation.reset(FeatureProxyBase::create_proxy<RelationProxy>(this));
	}
	return _relation.get();
};
bool DynamicProxy::clear_relation() {
	if (!is_editable()) {
		return false;
	}
	if (_relation) {
		mark_changed();
	}
	_relation.reset();
	return true;
};
bool DynamicProxy::set_heading(Optional<float> h) {
	if (!is_editable()) {
		return false;
	}
	if (_heading != h) {
		mark_changed();
	}
	_heading = h;
	return true;
};

bool DynamicProxy::set_type(Optional<int> t) {
	if (!is_editable()) {
		return false;
	}
	if (_type != t) {
		mark_changed();
	}
	_type = t;
	return true;
};

bool DynamicProxy::set_reason(Optional<int64_t> r) {
	if (!is_editable()) {
		return false;
	}
	if (_reason != r) {
		mark_changed();
	}
	_reason = r;
	return true;
};

bool DynamicProxy::set_action(Optional<int> a) {
	if (!is_editable()) {
		return false;
	}
	if (_action != a) {
		mark_changed();
	}
	_action = a;
	return true;
};

bool DynamicProxy::set_int_val(Optional<int64_t> i) {
	if (!is_editable()) {
		return false;
	}
	if (_int_val != i) {
		mark_changed();
	}
	_int_val = i;
	return true;
};

bool DynamicProxy::set_double_val(Optional<double> d) {
	if (!is_editable()) {
		return false;
	}
	if (_double_val != d) {
		mark_changed();
	}
	_double_val = d;
	return true;
};

bool DynamicProxy::set_binary_val(const Optional<std::string>& b) {
	if (!is_editable()) {
		return false;
	}
	if (_binary_val != b) {
		mark_changed();
	}
	_binary_val = b;
	return true;
};

bool DynamicProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::Dynamic*>(msg_base);
	msg->Clear();
	id_to_message(msg);
	if (_is_odd) {
		msg->set_is_odd(*_is_odd);
	}
	if (_range) {
		_range->to_message(msg->mutable_range());
	}
	if (_lines) {
		_lines->to_message(msg->mutable_lines());
	}
	if (_area) {
		_area->to_message(msg->mutable_area());
	}
	if (_relation) {
		_relation->to_message(msg->mutable_relation());
	}
	if (_heading) {
		msg->set_heading(*_heading);
	}
	if (_type) {
		msg->set_type(*_type);
	}
	if (_reason) {
		msg->set_reason(*_reason);
	}
	if (_action) {
		msg->set_action(*_action);
	}
	if (_int_val) {
		msg->set_int_val(*_int_val);
	}
	if (_double_val) {
		msg->set_double_val(*_double_val);
	}
	if (_binary_val) {
		msg->set_binary_val(*_binary_val);
	}
	return true;
};

bool DynamicProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::Dynamic*>(msg_base);
	id_from_message(msg);
	if (msg->has_is_odd()) {
		_is_odd = msg->is_odd();
	}
	else {
		_is_odd.reset();
	}
	if (msg->has_range()) {
		_range.reset(FeatureProxyBase::create_proxy<CircleProxy>(this));
		_range->from_message(&msg->range());
	}
	else {
		_range.reset();
	}
	if (msg->has_lines()) {
		_lines.reset(FeatureProxyBase::create_proxy<PolylineProxy>(this));
		_lines->from_message(&msg->lines());
	}
	else {
		_lines.reset();
	}
	if (msg->has_area()) {
		_area.reset(FeatureProxyBase::create_proxy<PolygonProxy>(this));
		_area->from_message(&msg->area());
	}
	else {
		_area.reset();
	}
	if (msg->has_relation()) {
		_relation.reset(FeatureProxyBase::create_proxy<RelationProxy>(this));
		_relation->from_message(&msg->relation());
	}
	else {
		_relation.reset();
	}
	if (msg->has_heading()) {
		_heading = msg->heading();
	}
	else {
		_heading.reset();
	}
	if (msg->has_type()) {
		_type = msg->type();
	}
	else {
		_type.reset();
	}
	if (msg->has_reason()) {
		_reason = msg->reason();
	}
	else {
		_reason.reset();
	}
	if (msg->has_action()) {
		_action = msg->action();
	}
	else {
		_action.reset();
	}
	if (msg->has_int_val()) {
		_int_val = msg->int_val();
	}
	else {
		_int_val.reset();
	}
	if (msg->has_double_val()) {
		_double_val = msg->double_val();
	}
	else {
		_double_val.reset();
	}
	if (msg->has_binary_val()) {
		_binary_val = msg->binary_val();
	}
	else {
		_binary_val.reset();
	}
	clear_changed();
	return true;
};

bool DynamicProxy::is_valid() const {
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	if (!is_message_valid<DynamicProxy>()) {
		LOG_BEGIN(ERROR) << "DynamicProxy::is_valid() failed for " << sid
			<< " : is_message_valid() failed!"; LOG_END;
		return false;
	}
	if (!is_editable() || is_deleted()) {
		//LOG_BEGIN(DEBUG) << "DynamicProxy::is_valid() skip for " << sid; LOG_END;
		return true;
	}
	if (!is_ref_valid()) {
		LOG_BEGIN(ERROR) << "DynamicProxy::is_valid() failed for " << sid
			<< " : has " << owner_count() << " owners"; LOG_END;
		return false;
	}
	if (_relation && !_relation->cached_is_valid()) {
		LOG_BEGIN(ERROR) << "DynamicProxy::is_valid() failed: link "
			<< _relation->id()->to_string() << " is invalid"; LOG_END;
		return false;
	}
	return true;
};

bool DynamicProxy::is_editable() const {
	if (_relation && _relation->id() && _relation->id()->container()) {
		return _relation->id()->container()->is_editable();
	}
	return true;
};

bool DynamicProxy::correct_tile_refs(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted() || !_id) {
		return false;
	}
	bool bc = false;
	std::vector<const TileInfoProxy*> tiles;
	if (_relation && _relation->id() && _relation->id()->container()) {
		auto cont = const_cast<FeatureWithIDProxyBase*>(_relation->id()->container());
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
			FeatureReference<DynamicProxy> e;
			e = mutable_changed_id(mgr->current_version());
			tile->dynamic_refs().push_back(e);
			LOG_BEGIN(INFO) << "DynamicProxy::correct_tile_refs(): add tile "
				<< tile->tile_id() << " 's ref " << e->to_string(); LOG_END;
			bc = true;
		}
	}
	auto ts = referenced_tiles();
	for (auto& tw : ts) {
		auto tile = const_cast<TileInfoProxy*>(tw);
		if (tile && ref_tiles.count(tile->tile_id()) <= 0) {
			auto ref = tile->dynamic_refs().find(*id());
			if (ref && ref->is_deleted()) {
				continue;
			}
			auto did = mutable_deleted_id(mgr->current_version());
			tile->dynamic_refs().replace(id(), did);
			LOG_BEGIN(INFO) << "DynamicProxy::correct_tile_refs(): remove tile "
				<< tile->tile_id() << " 's ref " << did->to_string(); LOG_END;
			bc = true;
		}
	}
	return bc;
};

bool DynamicProxy::correct_content(RoadGeometryManager* mgr) {
	if (!is_editable() || is_deleted()) {
		return false;
	}
	std::string sid("!no_id!");
	if (_id) {
		sid = _id->to_string();
	}
	/*if (!_relation || !_relation->id()) {
		LOG_BEGIN(INFO) << "DynamicProxy::correct_content() failed for " << sid
			<< " : no road"; LOG_END;
		set_deleted();
		return true;
	}*/

	return correct_tile_refs(mgr);
};

bool DynamicProxy::remake_proxy(RoadGeometryManager* mgr) {
	bool bf = true;
	if (_relation && _relation->id()) {
		auto p = _relation->id()->resolve_id(mgr);
		if (p) {
			_relation->_id = p->id();
		}
		else {
			bf = false;
			LOG_BEGIN(INFO) << "DynamicProxy::remake_proxy(): failed for "
				<< _relation->id()->to_string(); LOG_END;
		}
	}
	return bf;
};

bool DynamicProxy::merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) {
	return FeatureMerger<DynamicProxy>(policy, mgr, this, "Dynamic")
			.merge(_is_odd, "is_odd")
			.merge(_range, "range")
			.merge(_lines, "lines")
			.merge(_area, "area")
			.merge(_relation, "relation")
			.merge(_heading, "heading")
			.merge(_type, "type")
			.merge(_reason, "reason")
			.merge(_action, "action")
			.merge(_int_val, "int_val")
			.merge(_double_val, "double_val")
			.merge(_binary_val, "binary_val")
			.is_conflict();
};

}; //data_access_engine
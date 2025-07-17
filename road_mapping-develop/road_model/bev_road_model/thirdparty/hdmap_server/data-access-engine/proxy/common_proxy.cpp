#include "common_proxy.h"
#include "feature_merger.h"

namespace data_access_engine {

bool LangNameProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::LangName*>(msg_base);
	if (msg->has_lang_code()) {
		_lang_code = msg->lang_code();
	}
	else {
		_lang_code.reset();
	}
	if (msg->has_name_type()) {
		_name_type = msg->name_type();
	}
	else {
		_name_type.reset();
	}
	if (msg->has_name()) {
		_name = msg->name();
	}
	else {
		_name.reset();
	}
	clear_changed();
	return true;
};

bool LangNameProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::LangName*>(msg_base);
	msg->Clear();
	if (_name_type) {
		msg->set_name_type(*_name_type);
	}
	if (_lang_code) {
		msg->set_lang_code(*_lang_code);
	}
	if (_name) {
		msg->set_name(*_name);
	}
	return true;
};

bool LangNameProxy::is_valid() const {
	if (!_name && !_name_type && !_lang_code) {
		return false;
	}
	return true;
};

int LangNameProxy::compare(const LangNameProxy& rhs) const {
	if (_name_type != rhs._name_type) {
		return _name_type < rhs._name_type ? -1 : 1;
	}
	if (_lang_code != rhs._lang_code) {
		return _lang_code < rhs._lang_code ? -1 : 1;
	}
	if (_name == rhs._name) {
		return 0;
	}
	return _name < rhs._name ? -1 : 1;
};

bool LangNameProxy::set_name_type(const Optional<std::string>& nt) {
	if (!is_editable()) {
		return false;
	}
	if (_name_type != nt) {
		mark_changed();
	}
	_name_type = nt;
	return true;
};

bool LangNameProxy::set_lang_code(const Optional<std::string>& lc) {
	if (!is_editable()) {
		return false;
	}
	if (_lang_code != lc) {
		mark_changed();
	}
	_lang_code = lc;
	return true;
};

bool LangNameProxy::set_name(const Optional<std::string>& n) {
	if (!is_editable()) {
		return false;
	}
	if (_name != n) {
		mark_changed();
	}
	_name = n;
	return true;
};

bool LangNameProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
								const std::shared_ptr<const FeatureProxyBase>& base,
								const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const LangNameProxy>(remote);
	auto o = std::dynamic_pointer_cast<const LangNameProxy>(base);
	return ProxyMerger<LangNameProxy>(policy, mgr, this, "LangNameProxy", r, o)
				.merge(_name_type, "name_type")
				.merge(_lang_code, "lang_code")
				.merge(_name, "name")
				.is_conflict();
};

bool PointProxy::to_message(google::protobuf::Message * msg_base) const {
	auto msg = static_cast<RoadPB::Point*>(msg_base);
	if (std::isnan(_x)) {
		msg->clear_x();
	}
	else {
		msg->set_x(_x);
	}
	if (std::isnan(_y)) {
		msg->clear_y();
	}
	else {
		msg->set_y(_y);
	}
	if (std::isnan(_z)) {
		msg->clear_z();
	}
	else {
		msg->set_z(_z);
	}
	return true;
};

bool PointProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::Point*>(msg_base);
	if (msg->has_x()) {
		_x = msg->x();
	}
	else {
		_x = std::numeric_limits<double>::quiet_NaN();
	}
	if (msg->has_y()) {
		_y = msg->y();
	}
	else {
		_y = std::numeric_limits<double>::quiet_NaN();
	}
	if (msg->has_z()) {
		_z = msg->z();
	}
	else {
		_z = std::numeric_limits<double>::quiet_NaN();
	}
	clear_changed();
	return true;
};

bool PointProxy::is_valid() const {
	return (!std::isnan(_x) && !std::isnan(_y) && !std::isnan(_z));
};

int PointProxy::compare(const PointProxy& rhs) const {
	int ret = value_compare(_x, rhs._x);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_y, rhs._y);
	if (ret != 0) {
		return ret;
	}
	return value_compare(_z, rhs._z);
};

bool PolylineProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::Polyline*>(msg_base);
	_pts.from_message(msg->pts());
	if (msg->has_width()) {
		_width = msg->width();
	}
	else {
		_width.reset();
	}
	if (msg->has_height()) {
		_height = msg->height();
	}
	else {
		_height.reset();
	}
	clear_changed();
	return true;
};

bool PolylineProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::Polyline*>(msg_base);
	msg->Clear();
	_pts.to_message(msg->mutable_pts());
	if (_width) {
		msg->set_width(*_width);
	}
	if (_height) {
		msg->set_height(*_height);
	}
	return true;
};

bool PolylineProxy::is_valid() const {
	if (_pts.size() < 2) {
		return false;
	}
	return true;
};

bool PolylineProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
	const std::shared_ptr<const FeatureProxyBase>& base,
	const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const PolylineProxy>(remote);
	auto o = std::dynamic_pointer_cast<const PolylineProxy>(base);
	return ProxyMerger<PolylineProxy>(policy, mgr, this, "PolylineProxy", r, o)
										.merge_elemental(_pts, "pts")
										.merge(_width, "width")
										.merge(_height, "height")
										.is_conflict();
};

int PolylineProxy::compare(const PolylineProxy& rhs) const {
	int ret = value_compare(_width, rhs._width);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_height, rhs._height);
	if (ret != 0) {
		return ret;
	}
	return proxys_compare(_pts, rhs._pts);
};

bool PolylineProxy::calc_length_in_tiles(RoadGeometryManager* mgr, std::map<int, double>& tid2lens) const {
	if (_pts.empty()) {
		return false;
	}
	Vector3D pp = *_pts[0];
	int ptid = mgr->WGS84_to_tileID(pp);
	Vector3D gpp = pp;
	mgr->global_point_from_WGS84(gpp);
	bool bfound = false;
	for (size_t i = 1; i < _pts.size(); i++) {
		Vector3D cp = *_pts[i];
		int ctid = mgr->WGS84_to_tileID(cp);
		Vector3D gcp = cp;
		mgr->global_point_from_WGS84(gcp);
		if (ctid <= 0) {
			continue;
		}
		if (ptid <= 0) {
			ptid = ctid;
			pp = cp;
			gpp = gcp;
			continue;
		}
		bfound = true;
		if (ptid == ctid) {
			tid2lens[ctid] += (gpp - gcp).Length();
		}
		else {
			double pminx = 0;
			double pminy = 0;
			double pmaxx = 0;
			double pmaxy = 0;
			mgr->get_tile_box_WGS(ptid, pminx, pminy, pmaxx, pmaxy);
			Vector3D dpb = (cp - pp);
			double dpl = 1;
			Vector3D ppb = cp;
			if (pminx > cp.X()) {
				ppb = pp + dpb / (cp.X() - pp.X()) * (pminx - pp.X());
				dpl = (cp.X() - pp.X()) * (pminx - pp.X());
			}
			else if (pmaxx < cp.X()) {
				ppb = pp + dpb / (cp.X() - pp.X()) * (pmaxx - pp.X());
				dpl = dpl / (cp.X() - pp.X()) * (pmaxx - pp.X());
			}
			if (pminy > cp.Y() && dpl > (pminx - pp.Y()) / (cp.Y() - pp.Y())) {
				ppb = pp + dpb / (cp.Y() - pp.Y()) * (pminy - pp.Y());
			}
			else if (pmaxy < cp.Y() && dpl > (pmaxy - pp.Y()) / (cp.Y() - pp.Y())){
				ppb = pp + dpb / (cp.Y() - pp.Y()) * (pmaxy - pp.Y());
			}
			Vector3D gppb = ppb;
			mgr->global_point_from_WGS84(gppb);
			tid2lens[ptid] += (gpp - gppb).Length();

			double cminx = 0;
			double cminy = 0;
			double cmaxx = 0;
			double cmaxy = 0;
			mgr->get_tile_box_WGS(ctid, cminx, cminy, cmaxx, cmaxy);
			Vector3D dcb = (pp - cp);
			double dcl = 1;
			Vector3D cpb = pp;
			if (cminx > pp.X()) {
				cpb = cp + dcb / (pp.X() - cp.X()) * (cminx - cp.X());
				dcl = (pp.X() - cp.X()) * (cminx - cp.X());
			}
			else if (cmaxx < pp.X()) {
				cpb = cp + dcb / (pp.X() - cp.X()) * (cmaxx - cp.X());
				dcl = dcl / (pp.X() - cp.X()) * (cmaxx - cp.X());
			}
			if (cminy > pp.Y() && dcl > (cminx - cp.Y()) / (pp.Y() - cp.Y())) {
				cpb = cp + dcb / (pp.Y() - cp.Y()) * (cminy - cp.Y());
			}
			else if (cmaxy < pp.Y() && dcl >(cmaxy - cp.Y()) / (pp.Y() - cp.Y())) {
				cpb = cp + dcb / (pp.Y() - cp.Y()) * (cmaxy - cp.Y());
			}
			Vector3D gcpb = cpb;
			mgr->global_point_from_WGS84(gcpb);
			tid2lens[ctid] += (gcp - gcpb).Length();

			double len = (gcpb - gppb).Length();
			if (len > 0.01) {
				Vector3D p = (cpb + ppb) / 2;
				int tid = mgr->WGS84_to_tileID(p);
				tid2lens[tid] += len;
			}			
		}
		ptid = ctid;
		pp = cp;
		gpp = gcp;
	}
	return bfound;
};

SharedProxyVector<PointProxy>* PolylineProxy::mutable_pts() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_pts;
};

bool PolylineProxy::set_width(Optional<float> w) {
	if (!is_editable()) {
		return false;
	}
	if (_width != w) {
		mark_changed();
	}
	_width = w;
	return true;
};

bool PolylineProxy::set_height(Optional<float> h) {
	if (!is_editable()) {
		return false;
	}
	if (_height != h) {
		mark_changed();
	}
	_height = h;
	return true;
};

bool PolygonProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::Polygon*>(msg_base);
	_pts.from_message(msg->pts());
	if (msg->has_normal()) {
		_normal.reset(FeatureProxyBase::create_proxy<PointProxy>(this));
		_normal->from_message(&msg->normal());
	}
	else {
		_normal.reset();	
	}
	if (msg->has_edge_width()) {
		_edge_width = msg->edge_width();
	}
	else {
		_edge_width.reset();
	}
	if (msg->has_orientation()) {
		_orientation.reset(FeatureProxyBase::create_proxy<PointProxy>(this));
		_orientation->from_message(&msg->orientation());
	}
	else {
		_orientation.reset();
	}
	clear_changed();
	return true;
};

bool PolygonProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::Polygon*>(msg_base);
	msg->Clear();
	_pts.to_message(msg->mutable_pts());
	if (_normal) {
		_normal->to_message(msg->mutable_normal());
	}
	if (_edge_width) {
		msg->set_edge_width(*_edge_width);
	}
	if (_orientation) {
		_orientation->to_message(msg->mutable_orientation());
	}
	return true;
};

bool PolygonProxy::is_valid() const {
	if (_pts.size() < 3) {
		return false;
	}
	return true;
};

bool PolygonProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
								const std::shared_ptr<const FeatureProxyBase>& base,
								const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const PolygonProxy>(remote);
	auto o = std::dynamic_pointer_cast<const PolygonProxy>(base);
	return ProxyMerger<PolygonProxy>(policy, mgr, this, "PolygonProxy", r, o)
										.merge(_pts, "pts")
										.merge(_normal, "normal")
										.merge(_edge_width, "edge_width")
										.merge(_orientation, "orientation")
										.is_conflict();
};

int PolygonProxy::compare(const PolygonProxy& rhs) const {
	int ret = value_compare(_edge_width, rhs._edge_width);
	if (ret != 0) {
		return ret;
	}
	ret = proxy_compare(_normal, rhs._normal);
	if (ret != 0) {
		return ret;
	}
	ret = proxy_compare(_orientation, rhs._orientation);
	if (ret != 0) {
		return ret;
	}
	return proxys_compare(_pts, rhs._pts);
};

SharedProxyVector<PointProxy>* PolygonProxy::mutable_pts() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_pts;
};

bool PolygonProxy::set_edge_width(Optional<float> w) {
	if (!is_editable()) {
		return false;
	}
	if (_edge_width != w) {
		mark_changed();
	}
	_edge_width = w;
	return true;
};

PointProxy* PolygonProxy::mutable_normal() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_normal) {
		_normal.reset(FeatureProxyBase::create_proxy<PointProxy>(this));
	}
	return _normal.get();
};

bool PolygonProxy::clear_normal() {
	if (!is_editable()) {
		return false;
	}
	if (_normal) {
		mark_changed();
	}
	_normal.reset();
	return true;
};

PointProxy* PolygonProxy::mutable_orientation() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_orientation) {
		_orientation.reset(FeatureProxyBase::create_proxy<PointProxy>(this));
	}
	return _orientation.get();
};

bool PolygonProxy::clear_orientation() {
	if (!is_editable()) {
		return false;
	}
	if (_orientation) {
		mark_changed();
	}
	_orientation.reset();
	return true;
};

bool CircleProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::Circle*>(msg_base);
	if (msg->has_center()) {
		_center.reset(FeatureProxyBase::create_proxy<PointProxy>(this));
		_center->from_message(&msg->center());
	}
	else {
		_center.reset();
	}
	if (msg->has_normal()) {
		_normal.reset(FeatureProxyBase::create_proxy<PointProxy>(this));
		_normal->from_message(&msg->normal());
	}
	else {
		_normal.reset();
	}
	if (msg->has_radius()) {
		_radius = msg->radius();
	}
	else {
		_radius.reset();
	}
	if (msg->has_edge_width()) {
		_edge_width = msg->edge_width();
	}
	else {
		_edge_width.reset();
	}
	clear_changed();
	return true;
};

bool CircleProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::Circle*>(msg_base);
	msg->Clear();
	if (_center) {
		_center->to_message(msg->mutable_center());
	}
	if (_normal) {
		_normal->to_message(msg->mutable_normal());
	}
	if (_radius) {
		msg->set_radius(*_radius);
	}
	if (_edge_width) {
		msg->set_edge_width(*_edge_width);
	}	
	return true;
};

bool CircleProxy::is_valid() const {
	if (!_center || !_radius) {
		return false;
	}
	return true;
};

bool CircleProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
	const std::shared_ptr<const FeatureProxyBase>& base,
	const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const CircleProxy>(remote);
	auto o = std::dynamic_pointer_cast<const CircleProxy>(base);
	return ProxyMerger<CircleProxy>(policy, mgr, this, "CircleProxy", r, o)
										.merge(_center, "center")
										.merge(_normal, "normal")
										.merge(_radius, "radius")
										.merge(_edge_width, "edge_width")
										.is_conflict();
};

int CircleProxy::compare(const CircleProxy& rhs) const {
	int ret = value_compare(_radius, rhs._radius);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_edge_width, rhs._edge_width);
	if (ret != 0) {
		return ret;
	}
	ret = proxy_compare(_normal, rhs._normal);
	if (ret != 0) {
		return ret;
	}
	return proxy_compare(_center, rhs._center);
};

bool CircleProxy::set_radius(Optional<float> r) {
	if (!is_editable()) {
		return false;
	}
	if (_radius != r) {
		mark_changed();
	}
	_radius = r;
	return true;
};

bool CircleProxy::set_edge_width(Optional<float> w) {
	if (!is_editable()) {
		return false;
	}
	if (_edge_width != w) {
		mark_changed();
	}
	_edge_width = w;
	return true;
};

PointProxy* CircleProxy::mutable_center() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_center) {
		_center.reset(FeatureProxyBase::create_proxy<PointProxy>(this));
	}
	return _center.get();
};

bool CircleProxy::clear_center() {
	if (!is_editable()) {
		return false;
	}
	if (_center) {
		mark_changed();
	}
	_center.reset();
	return true;
};

PointProxy* CircleProxy::mutable_normal() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_normal) {
		_normal.reset(FeatureProxyBase::create_proxy<PointProxy>(this));
	}
	return _normal.get();
};

bool CircleProxy::clear_normal() {
	if (!is_editable()) {
		return false;
	}
	if (_normal) {
		mark_changed();
	}
	_normal.reset();
	return true;
};

bool CylinderProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::Cylinder*>(msg_base);
	_points.from_message(msg->points());
	if (msg->has_radius()) {
		_radius = msg->radius();
	}
	else {
		_radius.reset();
	}
	clear_changed();
	return true;
};

bool CylinderProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::Cylinder*>(msg_base);
	msg->Clear();
	_points.to_message(msg->mutable_points());
	if (_radius) {
		msg->set_radius(*_radius);
	}
	return true;
};

bool CylinderProxy::is_valid() const {
	if (_points.size() < 2) {
		return false;
	}
	return true;
};

bool CylinderProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
								const std::shared_ptr<const FeatureProxyBase>& base,
								const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const CylinderProxy>(remote);
	auto o = std::dynamic_pointer_cast<const CylinderProxy>(base);
	return ProxyMerger<CylinderProxy>(policy, mgr, this, "CylinderProxy", r, o)
										.merge_elemental(_points, "points")
										.merge(_radius, "radius")
										.is_conflict();
};

int CylinderProxy::compare(const CylinderProxy& rhs) const {
	int ret = value_compare(_radius, rhs._radius);
	if (ret != 0) {
		return ret;
	}
	return proxys_compare(_points, rhs._points);
};

SharedProxyVector<PointProxy>* CylinderProxy::mutable_points() {
	if (!is_editable()) {
		return nullptr;
	}
	return &_points;
};

bool CylinderProxy::set_radius(Optional<float> r) {
	if (!is_editable()) {
		return false;
	}
	if (_radius != r) {
		mark_changed();
	}
	_radius = r;
	return true;
};

bool FixedSpeedLimitProxy::from_message(const google::protobuf::Message* msg_base) {
	auto msg = static_cast<const RoadPB::FixedSpeedLimit*>(msg_base);
	if (msg->has_geom()) {
		_geom.reset(FeatureProxyBase::create_proxy<PolylineProxy>(this));
		_geom->from_message(&msg->geom());
	}
	else {
		_geom.reset();
	}
	if (msg->has_direction()) {
		_direction = msg->direction();
	}
	else {
		_direction.reset();
	}
	if (msg->has_max_speed()) {
		_max_speed = msg->max_speed();
	}
	else {
		_max_speed.reset();
	}
	if (msg->has_max_source()) {
		_max_source = msg->max_source();
	}
	if (msg->has_min_speed()) {
		_min_speed = msg->min_speed();
	}
	else {
		_min_speed.reset();
	}
	if (msg->has_min_source()) {
		_min_source = msg->min_source();
	}
	else {
		_min_source.reset();
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

bool FixedSpeedLimitProxy::to_message(google::protobuf::Message* msg_base) const {
	auto msg = static_cast<RoadPB::FixedSpeedLimit*>(msg_base);
	msg->Clear();
	if (_geom) {
		_geom->to_message(msg->mutable_geom());
	}
	if (_direction) {
		msg->set_direction(*_direction);
	}
	if (_max_speed) {
		msg->set_max_speed(*_max_speed);
	}
	if (_max_source) {
		msg->set_max_source(*_max_source);
	}
	if (_min_speed) {
		msg->set_min_speed(*_min_speed);
	}
	if (_min_source) {
		msg->set_min_source(*_min_source);
	}
	if (_valid_period) {
		msg->set_valid_period(*_valid_period);
	}
	if (_vehicle_type) {
		msg->set_vehicle_type(*_vehicle_type);
	}
	return true;
};

bool FixedSpeedLimitProxy::is_valid() const {
	if (!_max_speed && !_min_speed) {
		return false;
	}
	return true;
};

bool FixedSpeedLimitProxy::merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
										const std::shared_ptr<const FeatureProxyBase>& base,
										const std::shared_ptr<const FeatureProxyBase>& remote) {
	auto r = std::dynamic_pointer_cast<const FixedSpeedLimitProxy>(remote);
	auto o = std::dynamic_pointer_cast<const FixedSpeedLimitProxy>(base);
	return ProxyMerger<FixedSpeedLimitProxy>(policy, mgr, this, "FixedSpeedLimitProxy", r, o)
												.merge(_geom, "geom")
												.merge(_direction, "direction")
												.merge(_max_speed, "max_speed")
												.merge(_max_source, "max_source")
												.merge(_min_speed, "min_speed")
												.merge(_min_source, "min_source")
												.merge(_valid_period, "valid_period")
												.merge(_vehicle_type, "vehicle_type")
												.is_conflict();
};

int FixedSpeedLimitProxy::compare(const FixedSpeedLimitProxy& rhs) const {
	int ret = value_compare(_direction, rhs._direction);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_max_speed, rhs._max_speed);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_max_source, rhs._max_source);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_min_speed, rhs._min_speed);
	if (ret != 0) {
		return ret;
	}
	ret = value_compare(_min_source, rhs._min_source);
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
	return proxy_compare(_geom, rhs._geom);
};

PolylineProxy* FixedSpeedLimitProxy::mutable_geom() {
	if (!is_editable()) {
		return nullptr;
	}
	if (!_geom) {
		_geom.reset(FeatureProxyBase::create_proxy<PolylineProxy>(this));
	}
	return _geom.get();
};

bool FixedSpeedLimitProxy::clear_geoms() {
	if (!is_editable()) {
		return false;
	}
	if (_geom) {
		mark_changed();
	}
	_geom.reset();
	return true;
};

bool FixedSpeedLimitProxy::set_direction(Optional<int> d) {
	if (!is_editable()) {
		return false;
	}
	if (_direction != d) {
		mark_changed();
	}
	_direction = d;
	return true;
};

bool FixedSpeedLimitProxy::set_max_speed(Optional<float> s) {
	if (!is_editable()) {
		return false;
	}
	if (_max_speed != s) {
		mark_changed();
	}
	_max_speed = s;
	return true;
};

bool FixedSpeedLimitProxy::set_max_source(Optional<int> s) {
	if (!is_editable()) {
		return false;
	}
	if (_max_source != s) {
		mark_changed();
	}
	_max_source = s;
	return true;
};

bool FixedSpeedLimitProxy::set_min_speed(Optional<float> s) {
	if (!is_editable()) {
		return false;
	}
	if (_min_speed != s) {
		mark_changed();
	}
	_min_speed = s;
	return true;
};

bool FixedSpeedLimitProxy::set_min_source(Optional<int> s) {
	if (!is_editable()) {
		return false;
	}
	if (_min_source != s) {
		mark_changed();
	}
	_min_source = s;
	return true;
};

bool FixedSpeedLimitProxy::set_valid_period(const Optional<std::string>& v) {
	if (!is_editable()) {
		return false;
	}
	if (_valid_period != v) {
		mark_changed();
	}
	_valid_period = v;
	return true;
}

bool FixedSpeedLimitProxy::set_vehicle_type(const Optional<std::string>& v) {
	if (!is_editable()) {
		return false;
	}
	if (_vehicle_type != v) {
		mark_changed();
	}
	_vehicle_type = v;
	return true;
}

}; //data_access_engine
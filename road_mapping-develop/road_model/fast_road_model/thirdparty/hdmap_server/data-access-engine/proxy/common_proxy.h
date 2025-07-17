#pragma once
#include <memory>
#include <string>
#include "common/common.pb.h"
#include "feature_proxy_base.h"
#include "public/vector3.h"

namespace data_access_engine {

class LangNameProxy : public FeatureProxyBase
{
public:
	typedef RoadPB::LangName message_type;

	LangNameProxy() {};
	LangNameProxy(FeatureProxyBase* p) : FeatureProxyBase(p) {};
	virtual ~LangNameProxy() {};

	virtual bool from_message(const google::protobuf::Message* msg) override;
	virtual bool to_message(google::protobuf::Message* msg) const override;

	virtual bool is_valid() const override;
	
	virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
		const std::shared_ptr<const FeatureProxyBase>& base,
		const std::shared_ptr<const FeatureProxyBase>& remote) override;

	bool operator==(const LangNameProxy& rhs) const {
		return (compare(rhs) == 0);
	};
	bool operator<(const LangNameProxy& rhs) const {
		return (compare(rhs) < 0);
	};
	bool operator!=(const LangNameProxy& rhs) const {
		return (compare(rhs) != 0);
	};
	int compare(const LangNameProxy& rhs) const;

public:
	const Optional<std::string>& name_type() const {
		return _name_type;
	};
	bool set_name_type(const Optional<std::string>& nt);
	const Optional<std::string>& lang_code() const {
		return _lang_code;
	};
	bool set_lang_code(const Optional<std::string>& lc);
	const Optional<std::string> name() const {
		return _name;
	}
	bool set_name(const Optional<std::string>& n);

private:
	Optional<std::string> _name_type;
	Optional<std::string> _lang_code;
	Optional<std::string> _name;
};

class PointProxy : public FeatureProxyBase
{
public:
	typedef RoadPB::Point message_type;

	PointProxy() : _x(std::numeric_limits<double>::quiet_NaN()),
			_y(std::numeric_limits<double>::quiet_NaN()),
			_z(std::numeric_limits<double>::quiet_NaN()) {};
	PointProxy(FeatureProxyBase* p) : FeatureProxyBase(p),
			_x(std::numeric_limits<double>::quiet_NaN()),
			_y(std::numeric_limits<double>::quiet_NaN()),
			_z(std::numeric_limits<double>::quiet_NaN()) {};
	virtual ~PointProxy() {};

	virtual bool from_message(const google::protobuf::Message* msg) override;
	virtual bool to_message(google::protobuf::Message* msg) const override;

	virtual bool is_valid() const override;
	
	bool operator==(const PointProxy& rhs) const {
		return (std::fabs(_x - rhs._x) < ZERO_TOLERANCE
			&& std::fabs(_y - rhs._y) < ZERO_TOLERANCE
			&& std::fabs(_z - rhs._z) < ZERO_TOLERANCE);
	};
	bool operator<(const PointProxy& rhs) const {
		if (_x < rhs._x - ZERO_TOLERANCE) {
			return true;
		}
		if (_x > rhs._x + ZERO_TOLERANCE) {
			return false;
		}
		if (_y < rhs._y - ZERO_TOLERANCE) {
			return true;
		}
		if (_y > rhs._y + ZERO_TOLERANCE) {
			return false;
		}
		if (_z < rhs._z - ZERO_TOLERANCE) {
			return true;
		}
		return false;
	};
	bool operator!=(const PointProxy& rhs) const {
		return (compare(rhs) != 0);
	};
	int compare(const PointProxy& rhs) const;

public:
	bool set_x(double x) {
		if (!is_editable()) {
			return false;
		}
		if (std::fabs(_x - x) > ZERO_TOLERANCE) {
			mark_changed();
		}
		_x = x;
		return true;
	};
	double x() const { return _x; };
	bool set_y(double y) {
		if (!is_editable()) {
			return false;
		}
		if (std::fabs(_y - y) > ZERO_TOLERANCE) {
			mark_changed();
		}
		_y = y;
		return true;
	};
	double y() const { return _y; };
	bool set_z(double z) {
		if (!is_editable()) {
			return false;
		}
		if (std::fabs(_z - z) > ZERO_TOLERANCE) {
			mark_changed();
		}
		_z = z;
		return true;
	};
	double z() const { return _z; };

	operator Vector3D() const { return Vector3D(_x, _y, _z); };

	bool set(const Vector3D& p) {
		if (!is_editable()) {
			return false;
		}
		if (std::fabs(_x - p.X()) > ZERO_TOLERANCE
				|| std::fabs(_y - p.Y()) > ZERO_TOLERANCE
				|| std::fabs(_z - p.Z()) > ZERO_TOLERANCE) {
			mark_changed();
		}
		_x = p.X();
		_y = p.Y();
		_z = p.Z();
		return true;
	};

private:
	double _x;
	double _y;
	double _z;
};

class PolylineProxy : public FeatureProxyBase
{
public:
	typedef RoadPB::Polyline message_type;

	PolylineProxy() {};
	PolylineProxy(FeatureProxyBase* p) : FeatureProxyBase(p), _pts(this) {};
	virtual ~PolylineProxy() {};

	virtual bool from_message(const google::protobuf::Message* msg) override;
	virtual bool to_message(google::protobuf::Message* msg) const override;

	virtual bool is_valid() const override;
	
	virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
		const std::shared_ptr<const FeatureProxyBase>& base,
		const std::shared_ptr<const FeatureProxyBase>& remote) override;

	bool operator==(const PolylineProxy& rhs) const {
		return (compare(rhs) == 0);
	};
	bool operator<(const PolylineProxy& rhs) const {
		return (compare(rhs) < 0);
	};
	bool operator!=(const PolylineProxy& rhs) const {
		return (compare(rhs) != 0);
	};
	int compare(const PolylineProxy& rhs) const;

public:
	SharedProxyVector<PointProxy>* mutable_pts();
	const SharedProxyVector<PointProxy>& pts() const { return _pts; };
	Optional<float> width() const { return _width; };
	bool set_width(Optional<float> w);
	Optional<float> height() const { return _height; };
	bool set_height(Optional<float> h);

	bool calc_length_in_tiles(RoadGeometryManager* mgr, std::map<int, double>& tid2lens) const;
	
private:
	SharedProxyVector<PointProxy> _pts;
	Optional<float> _width;
	Optional<float> _height;
};

class PolygonProxy : public FeatureProxyBase
{
public:
	typedef RoadPB::Polygon message_type;

	PolygonProxy() {};
	PolygonProxy(FeatureProxyBase* p) : FeatureProxyBase(p), _pts(this) {};
	virtual ~PolygonProxy() {};

	virtual bool from_message(const google::protobuf::Message* msg) override;
	virtual bool to_message(google::protobuf::Message* msg) const override;

	virtual bool is_valid() const override;
	
	virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
		const std::shared_ptr<const FeatureProxyBase>& base,
		const std::shared_ptr<const FeatureProxyBase>& remote) override;

	bool operator==(const PolygonProxy& rhs) const {
		return (compare(rhs) == 0);
	};
	bool operator<(const PolygonProxy& rhs) const {
		return (compare(rhs) < 0);
	};
	bool operator!=(const PolygonProxy& rhs) const {
		return (compare(rhs) != 0);
	};
	int compare(const PolygonProxy& rhs) const;

public:
	SharedProxyVector<PointProxy>* mutable_pts();
	const SharedProxyVector<PointProxy>& pts() const { return _pts; };
	const PointProxy* normal() const { return _normal.get(); };
	PointProxy* mutable_normal();
	bool clear_normal();
	Optional<float> edge_width() const { return _edge_width; };
	bool set_edge_width(Optional<float> w);
	const PointProxy* orientation() const { return _orientation.get(); };
	PointProxy* mutable_orientation();
	bool clear_orientation();

private:
	SharedProxyVector<PointProxy> _pts;
	std::shared_ptr<PointProxy> _normal;
	Optional<float> _edge_width;
	std::shared_ptr<PointProxy> _orientation;	
};

class CircleProxy : public FeatureProxyBase
{
public:
	typedef RoadPB::Circle message_type;

	CircleProxy() {};
	CircleProxy(FeatureProxyBase* p) : FeatureProxyBase(p) {};
	virtual ~CircleProxy() {};

	virtual bool from_message(const google::protobuf::Message* msg) override;
	virtual bool to_message(google::protobuf::Message* msg) const override;

	virtual bool is_valid() const override;
	
	virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
		const std::shared_ptr<const FeatureProxyBase>& base,
		const std::shared_ptr<const FeatureProxyBase>& remote) override;

	bool operator==(const CircleProxy& rhs) const {
		return (compare(rhs) == 0);
	};
	bool operator<(const CircleProxy& rhs) const {
		return (compare(rhs) < 0);
	};
	bool operator!=(const CircleProxy& rhs) const {
		return (compare(rhs) != 0);
	};
	int compare(const CircleProxy& rhs) const;

public:
	const PointProxy* center() const { return _center.get(); };
	PointProxy* mutable_center();
	bool clear_center();
	const PointProxy* normal() const { return _normal.get(); };
	PointProxy* mutable_normal();
	bool clear_normal();
	Optional<float> radius() const { return _radius; };
	bool set_radius(Optional<float> r);
	Optional<float> edge_width() const { return _edge_width; };
	bool set_edge_width(Optional<float> w);
	
private:
	std::shared_ptr<PointProxy> _center;	
	std::shared_ptr<PointProxy> _normal;
	Optional<float> _radius;
	Optional<float> _edge_width;
};

class CylinderProxy : public FeatureProxyBase
{
public:
	typedef RoadPB::Cylinder message_type;

	CylinderProxy() {};
	CylinderProxy(FeatureProxyBase* p) : FeatureProxyBase(p), _points(this) {};
	virtual ~CylinderProxy() {};

	virtual bool from_message(const google::protobuf::Message* msg) override;
	virtual bool to_message(google::protobuf::Message* msg) const override;

	virtual bool is_valid() const override;
	
	virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
		const std::shared_ptr<const FeatureProxyBase>& base,
		const std::shared_ptr<const FeatureProxyBase>& remote) override;
	
	bool operator==(const CylinderProxy& rhs) const {
		return (compare(rhs) == 0);
	};
	bool operator<(const CylinderProxy& rhs) const {
		return (compare(rhs) < 0);
	};
	bool operator!=(const CylinderProxy& rhs) const {
		return (compare(rhs) != 0);
	};
	int compare(const CylinderProxy& rhs) const;

public:
	SharedProxyVector<PointProxy>* mutable_points();
	const SharedProxyVector<PointProxy>& points() const { return _points; };
	Optional<float> radius() const { return _radius; };
	bool set_radius(Optional<float> r);

private:
	SharedProxyVector<PointProxy> _points;
	Optional<float> _radius;
};

class FixedSpeedLimitProxy : public FeatureProxyBase
{
public:
	typedef RoadPB::FixedSpeedLimit message_type;

	FixedSpeedLimitProxy() {};
	FixedSpeedLimitProxy(FeatureProxyBase* p) : FeatureProxyBase(p) {};
	virtual ~FixedSpeedLimitProxy() {};

	virtual bool from_message(const google::protobuf::Message* msg) override;
	virtual bool to_message(google::protobuf::Message* msg) const override;

	virtual bool is_valid() const override;
	
	virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
		const std::shared_ptr<const FeatureProxyBase>& base,
		const std::shared_ptr<const FeatureProxyBase>& remote) override;
	
	bool operator==(const FixedSpeedLimitProxy& rhs) const {
		return (compare(rhs) == 0);
	};
	bool operator<(const FixedSpeedLimitProxy& rhs) const {
		return (compare(rhs) < 0);
	};
	bool operator!=(const FixedSpeedLimitProxy& rhs) const {
		return (compare(rhs) != 0);
	};
	int compare(const FixedSpeedLimitProxy& rhs) const;

public:
	const PolylineProxy* geom() const { return _geom.get(); };
	PolylineProxy* mutable_geom();
	bool clear_geoms();
	Optional<int> direction() const { return _direction; };
	bool set_direction(Optional<int> d);
	Optional<float> max_speed() const { return _max_speed; };
	bool set_max_speed(Optional<float> s);
	Optional<int> max_source() const { return _max_source; };
	bool set_max_source(Optional<int> s);
	Optional<float> min_speed() const { return _min_speed; };
	bool set_min_speed(Optional<float> s);
	Optional<int> min_source() const { return _min_source; };
	bool set_min_source(Optional<int> s);
	const Optional<std::string>& valid_period() const { return _valid_period; };
	bool set_valid_period(const Optional<std::string>& v);
	const Optional<std::string>& vehicle_type() const { return _vehicle_type; };
	bool set_vehicle_type(const Optional<std::string>& v);

private:
	std::shared_ptr<PolylineProxy> _geom;
	Optional<int> _direction;
	Optional<float> _max_speed;
	Optional<int> _max_source;
	Optional<float> _min_speed;
	Optional<int> _min_source;
	Optional<std::string> _valid_period;
	Optional<std::string> _vehicle_type;
};


}
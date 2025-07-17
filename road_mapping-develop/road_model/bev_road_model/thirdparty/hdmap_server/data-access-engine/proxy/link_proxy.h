#pragma once
#include <memory>
#include <string>
#include "link/link.pb.h"
#include "feature_with_id_proxy_base.h"
#include "common_proxy.h"

namespace data_access_engine {

/*class LinkTollInfoProxy : public FeatureProxyBase
{
public:
	typedef RoadPB::LinkTollInfo message_type;

	LinkTollInfoProxy() {};
	LinkTollInfoProxy(FeatureProxyBase* p) : FeatureProxyBase(p) {};
	virtual ~LinkTollInfoProxy() {};

	virtual bool from_message(const google::protobuf::Message* msg) override;
	virtual bool to_message(google::protobuf::Message* msg) const override;

	virtual bool is_valid() const override;
    virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
                            const std::shared_ptr<const FeatureProxyBase>& base,
                            const std::shared_ptr<const FeatureProxyBase>& remote) override;


	bool operator==(const LinkTollInfoProxy& rhs) const {
		return (compare(rhs) == 0);
	};
	bool operator<(const LinkTollInfoProxy& rhs) const {
		return (compare(rhs) < 0);
	};
	bool operator!=(const LinkTollInfoProxy& rhs) const {
		return (compare(rhs) != 0);
	};
	int compare(const LinkTollInfoProxy& rhs) const;

public:
	const PolylineProxy* geom() const { return _geom.get(); };
	PolylineProxy* mutable_geom();
	bool clear_geom();
	Optional<int> toll_type() const { return _toll_type; };
	bool set_toll_type(Optional<int> t);
	Optional<int> fee_type() const { return _fee_type; };
	bool set_fee_type(Optional<int> f);
	Optional<double> toll_standard() const { return _toll_standard; };
	bool set_toll_standard(Optional<double> t);
	const Optional<std::string>& valid_period() const { return _valid_period; };
	bool set_valid_period(const Optional<std::string>& v);
	const Optional<std::string>& vehicle_type() const { return _vehicle_type; };
	bool set_vehicle_type(const Optional<std::string>& v);
	Optional<double> attribute_len() const { return _attribute_len; };
	bool set_attribute_len(Optional<double> a);

private:
	std::shared_ptr<PolylineProxy> _geom;
	Optional<int> _toll_type;
	Optional<int> _fee_type;
	Optional<double> _toll_standard;
	Optional<std::string> _valid_period;
	Optional<std::string> _vehicle_type;
	Optional<double> _attribute_len;
};

class NodeProxy;
class LinkProxy;

class TurnRestrictProxy : public FeatureWithRefProxyBase
{
public:
	typedef RoadPB::TurnRestrict message_type;

	TurnRestrictProxy() : FeatureWithRefProxyBase() {};
	TurnRestrictProxy(FeatureProxyBase* p) : FeatureWithRefProxyBase(p) {
		make_ref_record();		
	};
	virtual ~TurnRestrictProxy() {
		_node_id.reset();
		_tolink_id.reset();
	};

	virtual void make_ref_record() const override {
		_ref_record.reset(new TRRefRecord(const_cast<TurnRestrictProxy*>(this)));
		if (container() && container()->reference_record()) {
			_ref_record->reset_container(container()->reference_record());
		}
	};
	virtual bool from_message(const google::protobuf::Message* msg) override;
	virtual bool to_message(google::protobuf::Message* msg) const override;

	virtual bool is_valid() const override;
	virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
		const std::shared_ptr<const FeatureProxyBase>& base,
		const std::shared_ptr<const FeatureProxyBase>& remote) override;


	bool operator==(const TurnRestrictProxy& rhs) const {
		return (compare(rhs) == 0);
	};
	bool operator<(const TurnRestrictProxy& rhs) const {
		return (compare(rhs) < 0);
	};
	bool operator!=(const TurnRestrictProxy& rhs) const {
		return (compare(rhs) != 0);
	};
	int compare(const TurnRestrictProxy& rhs) const;

public:
	const FeatureReference<NodeProxy> node_id() const { return _node_id; };
	FeatureReference<NodeProxy>* mutable_node_id();
	const FeatureReference<LinkProxy> tolink_id() const { return _tolink_id; };
	FeatureReference<LinkProxy>* mutable_tolink_id();
	Optional<int> type() const { return _type; };
	bool set_type(Optional<int> t);
	Optional<int> info() const { return _info; };
	bool set_info(Optional<int> i);
	Optional<int> kind() const { return _kind; };
	bool set_kind(Optional<int> k);
	const Optional<std::string>& valid_period() const { return _valid_period; };
	bool set_valid_period(const Optional<std::string>& v);
	const Optional<std::string>& vehicle_type() const { return _vehicle_type; };
	bool set_vehicle_type(const Optional<std::string>& v);

private:
	FeatureReference<NodeProxy> _node_id;
	FeatureReference<LinkProxy> _tolink_id;
	Optional<int> _type;
	Optional<int> _info;
	Optional<int> _kind;
	Optional<std::string> _valid_period;
	Optional<std::string> _vehicle_type;

	class TRRefRecord : public RefRecordBase
	{
	public:
		TRRefRecord(FeatureWithRefProxyBase* f) : RefRecordBase(f) {};

		virtual const int* conain_ref_offsets() const {
			static int _offs[] = { offsetof(TurnRestrictProxy, _node_id),
								   offsetof(TurnRestrictProxy, _tolink_id), -1 };
			return _offs;
		}
	};
};*/

class NodeProxy;
class LaneGroupProxy;
class JunctionProxy;

class LinkProxy : public FeatureWithIDProxyBase
{
public:
	typedef RoadPB::Link message_type;
	const static int ELEM_ID_TYPE = RoadPB::FeatureID::LINK;

	LinkProxy() : FeatureWithIDProxyBase(), _pos_sync_cnt(-1), /*_turn_restricts(this),*/
					_names(this), _forward_groups(this), _backward_groups(this) {};
	LinkProxy(FeatureProxyBase* p) : FeatureWithIDProxyBase(p), _pos_sync_cnt(-1),
					/*_turn_restricts(this),*/ _names(this), _forward_groups(this),
					_backward_groups(this) { make_ref_record();	};
	virtual ~LinkProxy() {
		_snode_id.reset();
		_enode_id.reset();
		//_turn_restricts.clear();
		_names.clear();
		_forward_groups.clear();
		_backward_groups.clear();
	};
	virtual void make_ref_record() const override {
		_ref_record.reset(new LinkRefRecord(const_cast<LinkProxy*>(this)));
		_ref_record->reset_container(_ref_record);
	};

	virtual int feature_id_type() const override { return ELEM_ID_TYPE; };
	virtual bool from_message(const google::protobuf::Message* msg) override;
	virtual bool to_message(google::protobuf::Message* msg) const override;
	virtual void clear_changed() override {
		_change_count = 0;
		_pos_sync_cnt = -1;
	};

	virtual bool is_valid() const override;
	virtual bool correct_tile_refs(RoadGeometryManager* mgr) override;
	virtual bool correct_content(RoadGeometryManager* mgr) override;
	virtual bool remake_proxy(RoadGeometryManager* mgr) override;
	virtual bool judge_editable(RoadGeometryManager* mgr) override;
    
	virtual bool make_id_index() override {
		return make_index<RoadPB::Link>();
	};
	virtual bool merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) override;
	virtual bool revert() override {
		return revert_base<LinkProxy>();
	};
	virtual void mark_changed() override {
		mark_changed_and_copy<LinkProxy>();
	};

	const std::shared_ptr<const LinkProxy> origin() const {
		return origin_elem<LinkProxy>();
	};
	const std::shared_ptr<const LinkProxy> remote() const {
		return remote_elem<LinkProxy>();
	};
	const std::shared_ptr<const LinkProxy> local() const {
		return local_elem<LinkProxy>();
	};

public:
	const FeatureReference<NodeProxy>& snode_id() const { return _snode_id; };
	FeatureReference<NodeProxy>* mutable_snode_id();
	const FeatureReference<NodeProxy>& enode_id() const { return _enode_id; };
	FeatureReference<NodeProxy>* mutable_enode_id();
	Optional<int> direction() const { return _direction; };
	bool set_direction(Optional<int> d);
	/*Optional<int> lane_count() const { return _lane_count; };
	bool set_lane_count(Optional<int> l);
	Optional<int> left_lanes() const { return _left_lanes; };
	bool set_left_lanes(Optional<int> l);
	Optional<int> right_lanes() const { return _right_lanes; };
	bool set_right_lanes(Optional<int> l);
	Optional<int> multiply_digitized() const { return _multiply_digitized; };
	bool set_multiply_digitized(Optional<int> m);*/
	Optional<int> link_type() const { return _link_type; };
	bool set_link_type(Optional<int> l);
	Optional<int> road_class() const { return _road_class; };
	bool set_road_class(Optional<int> c);
	/*Optional<int> urban_flag() const { return _urban_flag; };
	bool set_urban_flag(Optional<int> f);
	Optional<int> toll_area() const { return _toll_area; };
	bool set_toll_area(Optional<int> t);*/
	Optional<double> link_length() const { return _link_length; };
	bool set_link_length(Optional<double> l);
	/*Optional<int> open_status() const { return _open_status; };
	bool set_open_status(Optional<int> s);
	Optional<int> state() const { return _state; };
	bool set_state(Optional<int> s);*/
	const PolylineProxy* geom() const { return _geom.get(); };
	PolylineProxy* mutable_geom();
	bool clear_geom();
	/*const SharedProxyVector<LinkTollInfoProxy>& tollinfos() const { return _tollinfos; };
	SharedProxyVector<LinkTollInfoProxy>* mutable_tollinfos();
	const SharedProxyVector<TurnRestrictProxy>& turn_restricts() const { return _turn_restricts; };
	SharedProxyVector<TurnRestrictProxy>* mutable_turn_restricts();
	Optional<int> road_grade() const { return _road_grade; };
	bool set_road_grade(Optional<int> r);
	Optional<int> net_grade() const { return _net_grade; };
	bool set_net_grade(Optional<int> n);
	const Optional<std::string>& open_time() const { return _open_time; };
	bool set_open_time(const Optional<std::string>& o);
	Optional<std::string> valid_period() const { return _valid_period; };
	bool set_valid_period(const Optional<std::string>& v);
	const Optional<std::string>& vehicle_type() const { return _vehicle_type; };
	bool set_vehicle_type(const Optional<std::string>& v);*/
	const SharedProxyVector<LangNameProxy>& names() const { return _names; };
	SharedProxyVector<LangNameProxy>* mutable_names();
	Optional<int> provincecode() const { return _provincecode; };
	bool set_provincecode(Optional<int> p);
	const FeatureMemberVector<LaneGroupProxy>& forward_groups() const { return _forward_groups; };
	FeatureMemberVector<LaneGroupProxy>* mutable_forward_groups();
	const FeatureMemberVector<LaneGroupProxy>& backward_groups() const { return _backward_groups; };
	FeatureMemberVector<LaneGroupProxy>* mutable_backward_groups();

	virtual Vector3D wgs84_pos() const override;
	virtual const Vector3D& global_pos(const RoadGeometryManager* mgr = nullptr) const override {
		return global_stp_pos(mgr);
	};
	virtual const Vector3D& global_stp_pos(const RoadGeometryManager* mgr = nullptr) const;
	virtual const Vector3D& global_edp_pos(const RoadGeometryManager* mgr = nullptr) const;

protected:
	mutable Vector3D _global_stp_pos;
	mutable Vector3D _global_edp_pos;

private:
	void sync_wgs_to_global_pos(const RoadGeometryManager* mgr) const;

	FeatureReference<NodeProxy> _snode_id;
	FeatureReference<NodeProxy> _enode_id;
	Optional<int> _direction;
	/*Optional<int> _lane_count;
	Optional<int> _left_lanes;
	Optional<int> _right_lanes;
	Optional<int> _multiply_digitized;*/
	Optional<int> _link_type;
	Optional<int> _road_class;
	/*Optional<int> _urban_flag;
	Optional<int> _toll_area;*/
	Optional<double> _link_length;
	/*Optional<int> _open_status;
	Optional<int> _state;*/
	std::shared_ptr<PolylineProxy> _geom;
	/*SharedProxyVector<LinkTollInfoProxy> _tollinfos;
	SharedProxyVector<TurnRestrictProxy> _turn_restricts;
	Optional<int> _road_grade;
	Optional<int> _net_grade;
	Optional<std::string> _open_time;
	Optional<std::string> _valid_period;
	Optional<std::string> _vehicle_type;*/
	SharedProxyVector<LangNameProxy> _names;
	Optional<int> _provincecode;
	FeatureMemberVector<LaneGroupProxy> _forward_groups;
	FeatureMemberVector<LaneGroupProxy> _backward_groups;

	mutable int _pos_sync_cnt;

	class LinkRefRecord : public RefRecordWithTileBase
	{
	public:
		LinkRefRecord(FeatureWithIDProxyBase* elem) : RefRecordWithTileBase(elem) {};

		virtual const int* contain_members_offsets() const {
			static int _offs[] = { offsetof(LinkProxy, _forward_groups),
								   offsetof(LinkProxy, _backward_groups), -1 };
			return _offs;
		};
		virtual const int* contain_ref_offsets() const {
			static int _offs[] = { offsetof(LinkProxy, _snode_id),
								   offsetof(LinkProxy, _enode_id), -1 };
			return _offs;
		};
		/*virtual const int* contain_sub_refs_offs() const {
			static int _offs[] = { offsetof(LinkProxy, _turn_restricts), -1 };
			return _offs;
		};*/
	};
};

class LinkExtProxy : public LinkProxy
{
public:
	typedef RoadPB::Link message_type;
	const static int ELEM_ID_TYPE = RoadPB::FeatureID::LINK;

	LinkExtProxy() : LinkProxy() {};
	LinkExtProxy(FeatureProxyBase* p) : LinkProxy(p) {};
	virtual ~LinkExtProxy() {};

	virtual bool from_message(const google::protobuf::Message* msg) override;
	virtual bool to_message(google::protobuf::Message* msg) const override;
	
	virtual bool merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) override;
	
public:
	Optional<int> lane_count() const { return _lane_count; };
	bool set_lane_count(Optional<int> l);
	Optional<int> left_lanes() const { return _left_lanes; };
	bool set_left_lanes(Optional<int> l);
	Optional<int> right_lanes() const { return _right_lanes; };
	bool set_right_lanes(Optional<int> l);
	Optional<int> multiply_digitized() const { return _multiply_digitized; };
	bool set_multiply_digitized(Optional<int> m);	
	Optional<int> urban_flag() const { return _urban_flag; };
	bool set_urban_flag(Optional<int> f);	
	Optional<int> road_grade() const { return _road_grade; };
	bool set_road_grade(Optional<int> r);
	Optional<int> net_grade() const { return _net_grade; };
	bool set_net_grade(Optional<int> n);
	Optional<int> pavement_info() const { return _pavement_info; };
	bool set_pavement_info(Optional<int> p);
	Optional<int> pass_type() const { return _pass_type; };
	bool set_pass_type(Optional<int> p);
	Optional<int> grade() const { return _grade; };
	bool set_grade(Optional<int> g);

private:
	Optional<int> _lane_count;
	Optional<int> _left_lanes;
	Optional<int> _right_lanes;
	Optional<int> _multiply_digitized;
	Optional<int> _urban_flag;
	Optional<int> _road_grade;
	Optional<int> _net_grade;
	Optional<int> _pavement_info;
	Optional<int> _pass_type;
	Optional<int> _grade;
};

class NodeProxy : public FeatureWithIDProxyBase
{
public:
	typedef RoadPB::Node message_type;
	const static int ELEM_ID_TYPE = RoadPB::FeatureID::NODE;

	NodeProxy() : FeatureWithIDProxyBase(), _link_ids(this) {};
	NodeProxy(FeatureProxyBase* p) : FeatureWithIDProxyBase(p), _link_ids(this) {
		make_ref_record();
	};
	virtual ~NodeProxy() {
		//_parent_nodeid.reset();
		_junction_id.reset();
		_link_ids.clear();
	};
	virtual void make_ref_record() const override {
		_ref_record.reset(new NodeRefRecord(const_cast<NodeProxy*>(this)));
		_ref_record->reset_container(_ref_record);
	};

	virtual int feature_id_type() const override { return ELEM_ID_TYPE; };
	virtual bool from_message(const google::protobuf::Message* msg) override;
	virtual bool to_message(google::protobuf::Message* msg) const override;

	virtual bool is_valid() const override;
	virtual bool correct_tile_refs(RoadGeometryManager* mgr) override;
	virtual bool correct_content(RoadGeometryManager* mgr) override;
	virtual bool remake_proxy(RoadGeometryManager* mgr) override;

	virtual bool make_id_index() override {
		return make_index<RoadPB::Node>();
	};
	virtual bool is_editable() const {
		return _editable;
	};
	virtual void mark_changed() override {
		mark_changed_and_copy<NodeProxy>();
	};
	virtual bool merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) override;
	virtual bool revert() override {
		return revert_base<NodeProxy>();
	};

	const std::shared_ptr<const NodeProxy> origin() const {
		return origin_elem<NodeProxy>();
	};
	const std::shared_ptr<const NodeProxy> remote() const {
		return remote_elem<NodeProxy>();
	};
	const std::shared_ptr<const NodeProxy> local() const {
		return local_elem<NodeProxy>();
	};

public:
	const PointProxy* geom() const { return _geom.get(); };
	PointProxy* mutable_geom();
	bool clear_geom();
	/*Optional<int> node_type() const { return _node_type; };
	bool set_node_type(Optional<int> n);
	std::vector<int> node_form() const { return _node_form; };
	bool set_node_form(std::vector<int> n);
	Optional<int> traffic_signal() const { return _traffic_signal; };
	bool set_traffic_signal(Optional<int> t);
	const FeatureReference<NodeProxy>& parent_nodeid() const { return _parent_nodeid; };
	FeatureReference<NodeProxy>* mutable_parent_nodeid();*/
	const FeatureReferenceVector<LinkProxy>& link_ids() const { return _link_ids; };
	FeatureReferenceVector<LinkProxy>* mutable_link_ids();
	const FeatureReference<JunctionProxy>& junction_id() const { return _junction_id; };
	FeatureReference<JunctionProxy>* mutable_junction_id();

	virtual Vector3D wgs84_pos() const override;
	virtual const Vector3D& global_pos(const RoadGeometryManager* mgr = nullptr) const override;

private:
	std::shared_ptr<PointProxy> _geom;
	Optional<int> _node_type;
	std::vector<int> _node_form;
	Optional<int> _traffic_signal;
	//FeatureReference<NodeProxy> _parent_nodeid;
	FeatureReferenceVector<LinkProxy> _link_ids;
	FeatureReference<JunctionProxy> _junction_id;

	class NodeRefRecord : public RefRecordWithTileBase
	{
	public:
		NodeRefRecord(FeatureWithIDProxyBase* elem) : RefRecordWithTileBase(elem) {};

		virtual const int* contain_refs_offsets() const {
			static int _offs[] = { offsetof(NodeProxy, _link_ids), -1 };
			return _offs;
		};
		virtual const int* contain_ref_offsets() const {
			//static int _offs[] = { offsetof(NodeProxy, _parent_nodeid), -1 };
			static int _offs[] = { offsetof(NodeProxy, _junction_id), -1 };
			return _offs;
		};

	};

	mutable Vector3D _global_pos;
};

}; //data_access_engine
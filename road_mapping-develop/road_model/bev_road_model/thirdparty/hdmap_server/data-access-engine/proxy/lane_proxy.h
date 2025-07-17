#pragma once
#include <memory>
#include <string>
#include "lane/lane.pb.h"
#include "feature_with_id_proxy_base.h"
#include "common_proxy.h"

namespace data_access_engine {

class LaneDirectionProxy : public FeatureProxyBase
{
public:
	typedef RoadPB::LaneDirection message_type;

	LaneDirectionProxy() {};
	LaneDirectionProxy(FeatureProxyBase* p) : FeatureProxyBase(p) {};
	virtual ~LaneDirectionProxy() {};

	virtual bool from_message(const google::protobuf::Message* msg) override;
	virtual bool to_message(google::protobuf::Message* msg) const override;

	virtual bool is_valid() const override;
    virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
                            const std::shared_ptr<const FeatureProxyBase>& base,
                            const std::shared_ptr<const FeatureProxyBase>& remote) override;


	bool operator==(const LaneDirectionProxy& rhs) const {
		return (compare(rhs) == 0);
	};
	bool operator<(const LaneDirectionProxy& rhs) const {
		return (compare(rhs) < 0);
	};
	bool operator!=(const LaneDirectionProxy& rhs) const {
		return (compare(rhs) != 0);
	};
	int compare(const LaneDirectionProxy& rhs) const;

public:
	Optional<int> direction() const { return _direction; };
	bool set_direction(Optional<int> d);
	const Optional<std::string>& valid_period() const { return _valid_period; };
	bool set_valid_period(const Optional<std::string>& v);
	const std::vector<int>& allowed_vehicle_types() const { return _allowed_vehicle_types; };
	bool set_allowed_vehicle_types(const std::vector<int>& vs);

private:
	Optional<int> _direction;
	Optional<std::string> _valid_period;
	std::vector<int> _allowed_vehicle_types;
};

class LaneBoundaryProxy : public FeatureWithIDProxyBase
{
public:
    typedef RoadPB::LaneBoundary message_type;
    const static int ELEM_ID_TYPE = RoadPB::FeatureID::LANE_BOUNDARY;

    LaneBoundaryProxy() : FeatureWithIDProxyBase(), _pos_sync_cnt(-1) {};
    LaneBoundaryProxy(FeatureProxyBase* p) : FeatureWithIDProxyBase(p), _pos_sync_cnt(-1) {
        make_ref_record();
    };
    virtual ~LaneBoundaryProxy() {};

    virtual void make_ref_record() const override {
        _ref_record.reset(new RefRecordWithTileBase(const_cast<LaneBoundaryProxy*>(this)));
    };

    virtual int feature_id_type() const override { return ELEM_ID_TYPE; };
    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;
    virtual void clear_changed() override {
        _change_count = 0;
        _pos_sync_cnt = -1;
    };
    virtual bool is_valid() const override;
    virtual bool is_ref_valid() const override;
    virtual bool correct_tile_refs(RoadGeometryManager* mgr) override;
    virtual bool correct_content(RoadGeometryManager* mgr) override;
    virtual bool judge_editable(RoadGeometryManager* mgr) override;
    
    virtual bool make_id_index() override {
        return make_index<RoadPB::LaneBoundary>();
    };
    virtual void mark_changed() override {
        mark_changed_and_copy<LaneBoundaryProxy>();
    };
    virtual bool merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) override;
    virtual bool revert() override {
        return revert_base<LaneBoundaryProxy>();
    };

    const std::shared_ptr<const LaneBoundaryProxy> origin() const {
        return origin_elem<LaneBoundaryProxy>();
    };
    const std::shared_ptr<const LaneBoundaryProxy> remote() const {
        return remote_elem<LaneBoundaryProxy>();
    };
    const std::shared_ptr<const LaneBoundaryProxy> local() const {
        return local_elem<LaneBoundaryProxy>();
    };

public:
    const PolylineProxy* geom() const { return _geom.get(); };
    PolylineProxy* mutable_geom();
    bool clear_geom();
    Optional<int> color() const { return _color; };
    bool set_color(Optional<int> c);
    std::vector<int> types() const { return _types; };
    bool set_types(const std::vector<int>& ts);
    Optional<int> marking() const { return _marking; };
    bool set_marking(Optional<int> m);
    //Optional<int> confidence() const { return _confidence; };
    //bool set_confidence(Optional<int> c);
    Optional<bool> ldm() const { return  _ldm; };
    bool set_ldm(Optional<bool> b);

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

    std::shared_ptr<PolylineProxy> _geom;
    Optional<int> _color;
    std::vector<int> _types;
    Optional<int> _marking;
    //Optional<int> _confidence;
    Optional<bool> _ldm;

    mutable int _pos_sync_cnt;
};

class LaneBoundaryRangeProxy : public FeatureWithRefProxyBase
{
public:
    typedef RoadPB::LaneBoundaryRange message_type;

    LaneBoundaryRangeProxy() {};
    LaneBoundaryRangeProxy(FeatureProxyBase* p) : FeatureWithRefProxyBase(p) {
        make_ref_record();
    };
    virtual ~LaneBoundaryRangeProxy() {
        _bound_id.reset();
    };

    virtual void make_ref_record() const override {
        _ref_record.reset(new LBRefRecord(const_cast<LaneBoundaryRangeProxy*>(this)));
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

    bool operator==(const LaneBoundaryRangeProxy& rhs) const {
        return (compare(rhs) == 0);
    };
    bool operator<(const LaneBoundaryRangeProxy& rhs) const {
        return (compare(rhs) < 0);
    };
    bool operator!=(const LaneBoundaryRangeProxy& rhs) const {
        return (compare(rhs) != 0);
    };
    int compare(const LaneBoundaryRangeProxy& rhs) const;

public:
    const FeatureMember<LaneBoundaryProxy>& bound_id() const { return _bound_id; };
    FeatureMember<LaneBoundaryProxy>* mutable_bound_id();
    const PointProxy* start_pt() const { return _start_pt.get(); };
    PointProxy* mutable_start_pt();
    bool clear_start_pt();
    const PointProxy* end_pt() const { return _end_pt.get(); };
    PointProxy* mutable_end_pt();
    bool clear_end_pt();

private:
    friend class LaneSectionProxy;

    FeatureMember<LaneBoundaryProxy> _bound_id;
    std::shared_ptr<PointProxy> _start_pt;
    std::shared_ptr<PointProxy> _end_pt;

    class LBRefRecord : public RefRecordBase 
    {
    public:
        LBRefRecord(FeatureWithRefProxyBase* f) : RefRecordBase(f) {};

        virtual const int* contain_member_offsets() const {
            static int _offs[] = { offsetof(LaneBoundaryRangeProxy, _bound_id), -1 };
            return _offs;
        };
    };
};

class RoadBoundaryProxy : public FeatureWithIDProxyBase
{
public:
    typedef RoadPB::RoadBoundary message_type;
    const static int ELEM_ID_TYPE = RoadPB::FeatureID::ROAD_BOUNDARY;

    RoadBoundaryProxy() : FeatureWithIDProxyBase(), _pos_sync_cnt(-1) {};
    RoadBoundaryProxy(FeatureProxyBase* p) : FeatureWithIDProxyBase(p), _pos_sync_cnt(-1) {
        make_ref_record();
    };
    virtual ~RoadBoundaryProxy() {};

    virtual void make_ref_record() const override {
        _ref_record.reset(new RefRecordWithTileBase(const_cast<RoadBoundaryProxy*>(this)));
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
    virtual bool judge_editable(RoadGeometryManager* mgr) override;
    
    virtual bool make_id_index() override {
        return make_index<RoadPB::RoadBoundary>();
    };
    virtual void mark_changed() override {
        mark_changed_and_copy<RoadBoundaryProxy>();
    };
    virtual bool merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) override;
    virtual bool revert() override {
        return revert_base<RoadBoundaryProxy>();
    };

    const std::shared_ptr<const RoadBoundaryProxy> origin() const {
        return origin_elem<RoadBoundaryProxy>();
    };
    const std::shared_ptr<const RoadBoundaryProxy> remote() const {
        return remote_elem<RoadBoundaryProxy>();
    };
    const std::shared_ptr<const RoadBoundaryProxy> local() const {
        return local_elem<RoadBoundaryProxy>();
    };

public:
    const PolylineProxy* geom() const { return _geom.get(); };
    PolylineProxy* mutable_geom();
    bool clear_geom();
    Optional<int> type() const { return _type; };
    bool set_type(Optional<int> t);
    //Optional<int> confidence() const { return _confidence; };
    //bool set_confidence(Optional<int> c);
    //std::vector<int> restriction() const { return  _restriction; };
    //bool set_restriction(const std::vector<int>& r);

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

    std::shared_ptr<PolylineProxy> _geom;
    Optional<int> _type;
    //Optional<int> _confidence;
    //std::vector<int> _restriction;
        
    mutable int _pos_sync_cnt;
};

class PositionObjectProxy;
class TrafficInfoProxy;
class LaneSectionProxy : public FeatureWithRefProxyBase
{
public:
    typedef RoadPB::LaneSection message_type;
    LaneSectionProxy() : _lstp_sync_cnt(-1), _ledp_sync_cnt(-1),
            _rstp_sync_cnt(-1), _redp_sync_cnt(-1), _speed_limits(this),
            _objects(this), _traffics(this) {};
    LaneSectionProxy(FeatureProxyBase* p) : FeatureWithRefProxyBase(p), _lstp_sync_cnt(-1),
            _ledp_sync_cnt(-1), _rstp_sync_cnt(-1), _redp_sync_cnt(-1),
            _speed_limits(this), _objects(this), _traffics(this) {
        make_ref_record();
    };
    virtual ~LaneSectionProxy() {
        _left_boundary.reset();
        _right_boundary.reset();
        _objects.clear();
        _traffics.clear();
    };

    virtual void make_ref_record() const override {
        _ref_record.reset(new LaneSectionRefRecord(const_cast<LaneSectionProxy*>(this)));
    };
    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;
    virtual void clear_changed() override {
        _change_count = 0;
        _lstp_sync_cnt = -1;
        _ledp_sync_cnt = -1;
        _rstp_sync_cnt = -1;
        _redp_sync_cnt = -1;
    };

    virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
                            const std::shared_ptr<const FeatureProxyBase>& base,
                            const std::shared_ptr<const FeatureProxyBase>& remote) override;

    virtual bool is_valid() const override;
    virtual bool correct_content(RoadGeometryManager* mgr) override;
    virtual bool remake_proxy(RoadGeometryManager* mgr) override;

    bool operator==(const LaneSectionProxy& rhs) const {
        return (compare(rhs) == 0);
    };
    bool operator<(const LaneSectionProxy& rhs) const {
        return (compare(rhs) < 0);
    };
    bool operator!=(const LaneSectionProxy& rhs) const {
        return (compare(rhs) != 0);
    };
    int compare(const LaneSectionProxy& rhs) const;

public:
    const LaneBoundaryRangeProxy* left_boundary() const { return _left_boundary.get(); };
    LaneBoundaryRangeProxy* mutable_left_boundary();
    bool clear_left_boundary();
    const LaneBoundaryRangeProxy* right_boundary() const { return _right_boundary.get(); };
    LaneBoundaryRangeProxy* mutable_right_boundary();
    bool clear_right_boundary();
    const SharedProxyVector<FixedSpeedLimitProxy>& speed_limits() const { return _speed_limits; };
    SharedProxyVector<FixedSpeedLimitProxy>* mutable_speed_limits();
    const FeatureReferenceVector<PositionObjectProxy>& objects() const { return _objects; };
    FeatureReferenceVector<PositionObjectProxy>* mutable_objects();
    const FeatureReferenceVector<TrafficInfoProxy>& traffics() const { return _traffics; };
    FeatureReferenceVector<TrafficInfoProxy>* mutable_traffics();

    Optional<float> width() const { return _width; };
    bool set_width(Optional<float> w);
    Optional<float> height_limit() const { return _height_limit; };
    bool set_height_limit(Optional<float> h);
    Optional<float> weight_limit() const { return _weight_limit; };
    bool set_weight_limit(Optional<float> w);
    Optional<float> width_limit() const { return _width_limit; };
    bool set_width_limit(Optional<float> w);

    const Vector3D& global_left_start_pos(const RoadGeometryManager* mgr = nullptr) const;
    bool set_global_left_start_pos(const Vector3D& p, const RoadGeometryManager* mgr = nullptr);
    const Vector3D& global_left_end_pos(const RoadGeometryManager* mgr = nullptr) const;
    bool set_global_left_end_pos(const Vector3D& p, const RoadGeometryManager* mgr = nullptr);
    const Vector3D& global_right_start_pos(const RoadGeometryManager* mgr = nullptr) const;
    bool set_global_right_start_pos(const Vector3D& p, const RoadGeometryManager* mgr = nullptr);
    const Vector3D& global_right_end_pos(const RoadGeometryManager* mgr = nullptr) const;
    bool set_global_right_end_pos(const Vector3D& p, const RoadGeometryManager* mgr = nullptr);

protected:
    mutable Vector3D _global_lstp_pos;
    mutable Vector3D _global_ledp_pos;
    mutable Vector3D _global_rstp_pos;
    mutable Vector3D _global_redp_pos;

private:
    std::shared_ptr<LaneBoundaryRangeProxy> _left_boundary;
    std::shared_ptr<LaneBoundaryRangeProxy> _right_boundary;
    SharedProxyVector<FixedSpeedLimitProxy> _speed_limits;
    FeatureReferenceVector<PositionObjectProxy> _objects;
    FeatureReferenceVector<TrafficInfoProxy> _traffics;
    Optional<float> _width;
    Optional<float> _height_limit;
    Optional<float> _weight_limit;
    Optional<float> _width_limit;

    mutable int _lstp_sync_cnt;
    mutable int _ledp_sync_cnt;
    mutable int _rstp_sync_cnt;
    mutable int _redp_sync_cnt;

    class LaneSectionRefRecord : public RefRecordBase 
    {
    public:
        LaneSectionRefRecord(FeatureWithRefProxyBase* f) : RefRecordBase(f) {};

        virtual const int* contain_sub_member_offs() const {
            static int _offs[] = { offsetof(LaneSectionProxy, _left_boundary),
                                   offsetof(LaneSectionProxy, _right_boundary), -1 };
            return _offs;
        };
        virtual const int* contain_refs_offsets() const {
            static int _offs[] = { offsetof(LaneSectionProxy, _objects),
                                   offsetof(LaneSectionProxy, _traffics), -1 };
            return _offs;
        };
    };
};

class LaneProxy : public FeatureWithIDProxyBase
{
public:
    typedef RoadPB::Lane message_type;
    const static int ELEM_ID_TYPE = RoadPB::FeatureID::LANE;

    LaneProxy() : FeatureWithIDProxyBase(), _lanes(this), _drivelines(this),
                    _directions(this), _preds(this), _succs(this) {};
    LaneProxy(FeatureProxyBase* p) : FeatureWithIDProxyBase(p), _lanes(this), _drivelines(this),
                    _directions(this), _preds(this), _succs(this) {
        make_ref_record();
    };
    virtual ~LaneProxy() {
        _lanes.clear();
        _drivelines.clear();
        _directions.clear();
        _preds.clear();
        _succs.clear();
    };
    virtual void make_ref_record() const override {
        _ref_record.reset(new LaneRefRecord(const_cast<LaneProxy*>(this)));
    };

    virtual int feature_id_type() const override { return ELEM_ID_TYPE; };
    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;

    virtual bool is_valid() const override;
    virtual bool correct_tile_refs(RoadGeometryManager* mgr) override;
    virtual bool correct_content(RoadGeometryManager* mgr) override;
    virtual bool remake_proxy(RoadGeometryManager* mgr) override;
    virtual bool judge_editable(RoadGeometryManager* mgr) override;
    
    virtual bool make_id_index() override {
        return make_index<RoadPB::Lane>();
    };
    virtual bool merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) override;
    virtual bool revert() override {
        return revert_base<LaneProxy>();
    };
    virtual void mark_changed() override {
        mark_changed_and_copy<LaneProxy>();
    };

    const std::shared_ptr<const LaneProxy> origin() const {
        return origin_elem<LaneProxy>();
    };
    const std::shared_ptr<const LaneProxy> remote() const {
        return remote_elem<LaneProxy>();
    };
    const std::shared_ptr<const LaneProxy> local() const {
        return local_elem<LaneProxy>();
    };

public:
    const SharedProxyVector<LaneSectionProxy>& lanes() const { return _lanes; };
    SharedProxyVector<LaneSectionProxy>* mutable_lanes();
    const FeatureMemberVector<LaneBoundaryProxy>& drivelines() const { return _drivelines; };
    FeatureMemberVector<LaneBoundaryProxy>* mutable_drivelines();
    //Optional<int> function() const { return _function; };
    //bool set_function(Optional<int> f);
    Optional<uint64_t> type() const { return _type; };
    bool set_type(Optional<uint64_t> t);
    //Optional<int> kind() const { return _kind; };
    //bool set_kind(Optional<int> k);
    const SharedProxyVector<LaneDirectionProxy>& directions() const { return _directions; };
    SharedProxyVector<LaneDirectionProxy>* mutable_directions();
    //Optional<int> priority() const { return _priority; };
    //bool set_priority(Optional<int> p);
    Optional<float> length() const { return _length; };
    bool set_length(Optional<float> l);
    Optional<int> seq_no() const { return _seq_no; };
    bool set_seq_no(Optional<int> s);
    const FeatureReferenceVector<LaneProxy>& preds() const { return _preds; };
    FeatureReferenceVector<LaneProxy>* mutable_preds();
    const FeatureReferenceVector<LaneProxy>& succs() const { return _succs; };
    FeatureReferenceVector<LaneProxy>* mutable_succs();

    virtual Vector3D wgs84_pos() const override;
    virtual const Vector3D& global_pos(const RoadGeometryManager* mg = nullptr) const override;
    const Vector3D& global_left_start_pos(const RoadGeometryManager* mgr = nullptr) const;
    const Vector3D& global_left_end_pos(const RoadGeometryManager* mgr = nullptr) const;
    const Vector3D& global_right_start_pos(const RoadGeometryManager* mgr = nullptr) const;
    const Vector3D& global_right_end_pos(const RoadGeometryManager* mgr = nullptr) const;
    
private:
    SharedProxyVector<LaneSectionProxy> _lanes;
    FeatureMemberVector<LaneBoundaryProxy> _drivelines;
    //Optional<int> _function;
    Optional<uint64_t> _type;
    //Optional<int> _kind;
    SharedProxyVector<LaneDirectionProxy> _directions;
    //Optional<int> _priority;
    Optional<float> _length;
    Optional<int> _seq_no;
    FeatureReferenceVector<LaneProxy> _preds;
    FeatureReferenceVector<LaneProxy> _succs;

    class LaneRefRecord : public RefRecordWithTileBase
    {
    public:
        LaneRefRecord(FeatureWithIDProxyBase* elem) : RefRecordWithTileBase(elem) {};

        virtual const int* contain_members_offsets() const {
            static int _offs[] = { offsetof(LaneProxy, _drivelines), -1 };
            return _offs;
        };        
        virtual const int* contain_refs_offsets() const {
            static int _offs[] = { offsetof(LaneProxy, _preds),
                                   offsetof(LaneProxy, _succs), -1 };
            return _offs;
        };
        virtual const int* contain_sub_refs_offs() const {
            static int _offs[] = {  offsetof(LaneProxy, _lanes), -1 };
            return _offs;
        };
        virtual const int* contain_sub_members_offs() const {
            static int _offs[] = {  offsetof(LaneProxy, _lanes), -1 };
            return _offs;
        };
    };
};

class LaneExtProxy : public LaneProxy
{
public:
    typedef RoadPB::Lane message_type;
    const static int ELEM_ID_TYPE = RoadPB::FeatureID::LANE;

    LaneExtProxy() : LaneProxy() {};
    LaneExtProxy(FeatureProxyBase* p) : LaneProxy(p) {};
    virtual ~LaneExtProxy() {};
    
    virtual int feature_id_type() const override { return ELEM_ID_TYPE; };
    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;

    virtual bool merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) override;
    
public:
    Optional<int> function() const { return _function; };
    bool set_function(Optional<int> f);
    Optional<int> priority() const { return _priority; };
    bool set_priority(Optional<int> p);
    const std::vector<int>& l_restrictions() const { return _l_restrictions; };
    bool set_l_restrictions(const std::vector<int>& ls);
    const std::vector<int>& r_restrictions() const { return _r_restrictions; };
    bool set_r_restrictions(const std::vector<int>& rs);
    Optional<int> transition() const { return _transition; };
    bool set_transition(Optional<int> t);
    
private:
    Optional<int> _function;
    Optional<int> _priority;
    std::vector<int> _l_restrictions;
    std::vector<int> _r_restrictions;
    Optional<int> _transition;
};

class LinkProxy;

class LaneGroupProxy : public FeatureWithIDProxyBase
{
public:
    typedef RoadPB::LaneGroup message_type;
    const static int ELEM_ID_TYPE = RoadPB::FeatureID::LANE_GROUP;

    LaneGroupProxy() : FeatureWithIDProxyBase(), _lanes(this), _link_id(this),
                    _left_boundarys(this), _right_boundarys(this), _preds(this), _succs(this) {};
    LaneGroupProxy(FeatureProxyBase* p) : FeatureWithIDProxyBase(p), _lanes(this), _link_id(this),
                    _left_boundarys(this), _right_boundarys(this), _preds(this), _succs(this) {
        make_ref_record();
    };
    virtual ~LaneGroupProxy() {
        _lanes.clear();
        _left_boundarys.clear();
        _right_boundarys.clear();
        _preds.clear();
        _succs.clear();
    };
    virtual void make_ref_record() const override {
        _ref_record.reset(new LaneGroupRefRecord(const_cast<LaneGroupProxy*>(this)));
    };

    virtual int feature_id_type() const override { return ELEM_ID_TYPE; };
    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;

    virtual bool is_valid() const override;
    virtual bool correct_tile_refs(RoadGeometryManager* mgr) override;
    virtual bool correct_content(RoadGeometryManager* mgr) override;
    virtual bool remake_proxy(RoadGeometryManager* mgr) override;
    virtual bool judge_editable(RoadGeometryManager* mgr) override;
    
    virtual bool make_id_index() override {
        return make_index<RoadPB::LaneGroup>();
    };
    virtual bool merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) override;
    virtual bool revert() override {
        return revert_base<LaneGroupProxy>();
    };
    virtual void mark_changed() override {
        mark_changed_and_copy<LaneGroupProxy>();
    };

    const std::shared_ptr<const LaneGroupProxy> origin() const {
        return origin_elem<LaneGroupProxy>();
    };
    const std::shared_ptr<const LaneGroupProxy> remote() const {
        return remote_elem<LaneGroupProxy>();
    };
    const std::shared_ptr<const LaneGroupProxy> local() const {
        return local_elem<LaneGroupProxy>();
    };

public:
    const FeatureReference<LinkProxy> link_id() const { return _link_id; };
    FeatureReference<LinkProxy>* mutable_link_id();
    const std::vector<int>& types() const { return _types; };
    bool set_types(const std::vector<int>& t);
    const FeatureMemberVector<RoadBoundaryProxy>& left_boundarys() const { return _left_boundarys; };
    FeatureMemberVector<RoadBoundaryProxy>* mutable_left_boundarys();
    const FeatureMemberVector<RoadBoundaryProxy>& right_boundarys() const { return _right_boundarys; };
    FeatureMemberVector<RoadBoundaryProxy>* mutable_right_boundarys();
    const FeatureMemberVector<LaneProxy>& lanes() const { return _lanes; };
    FeatureMemberVector<LaneProxy>* mutable_lanes();
    const FeatureReferenceVector<LaneGroupProxy>& preds() const { return _preds; };
    FeatureReferenceVector<LaneGroupProxy>* mutable_preds();
    const FeatureReferenceVector<LaneGroupProxy>& succs() const { return _succs; };
    FeatureReferenceVector<LaneGroupProxy>* mutable_succs();

    virtual Vector3D wgs84_pos() const override;
    virtual const Vector3D& global_pos(const RoadGeometryManager* mg = nullptr) const override;
    const Vector3D& global_left_start_pos(const RoadGeometryManager* mgr = nullptr) const;
    const Vector3D& global_left_end_pos(const RoadGeometryManager* mgr = nullptr) const;
    const Vector3D& global_right_start_pos(const RoadGeometryManager* mgr = nullptr) const;
    const Vector3D& global_right_end_pos(const RoadGeometryManager* mgr = nullptr) const;

private:
    FeatureReference<LinkProxy> _link_id;
    std::vector<int> _types;
    FeatureMemberVector<RoadBoundaryProxy> _left_boundarys;
    FeatureMemberVector<RoadBoundaryProxy> _right_boundarys;
    FeatureMemberVector<LaneProxy> _lanes;
    FeatureReferenceVector<LaneGroupProxy> _preds;
    FeatureReferenceVector<LaneGroupProxy> _succs;

    class LaneGroupRefRecord : public RefRecordWithTileBase
    {
    public:
        LaneGroupRefRecord(FeatureWithIDProxyBase* elem) : RefRecordWithTileBase(elem) {};

        virtual const int* contain_members_offsets() const {
            static int _offs[] = { offsetof(LaneGroupProxy, _lanes), 
                                   offsetof(LaneGroupProxy, _left_boundarys),
                                   offsetof(LaneGroupProxy, _right_boundarys), -1 };
            return _offs;
        };
        virtual const int* contain_refs_offsets() const {
            static int _offs[] = { offsetof(LaneGroupProxy, _preds),
                                   offsetof(LaneGroupProxy, _succs), -1 };
            return _offs;
        };
        virtual const int* contain_ref_offsets() const {
            static int _offs[] = { offsetof(LaneGroupProxy, _link_id), -1 };
            return _offs;
        };
    };
};

class ImpassableAreaProxy : public FeatureProxyBase
{
public:
    typedef RoadPB::ImpassableArea message_type;

    ImpassableAreaProxy() {};
    ImpassableAreaProxy(FeatureProxyBase* p) : FeatureProxyBase(p) {};
    virtual ~ImpassableAreaProxy() {};

    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;

    virtual bool is_valid() const override;
    virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
        const std::shared_ptr<const FeatureProxyBase>& base,
        const std::shared_ptr<const FeatureProxyBase>& remote) override;


    bool operator==(const ImpassableAreaProxy& rhs) const {
        return (compare(rhs) == 0);
    };
    bool operator<(const ImpassableAreaProxy& rhs) const {
        return (compare(rhs) < 0);
    };
    bool operator!=(const ImpassableAreaProxy& rhs) const {
        return (compare(rhs) != 0);
    };
    int compare(const ImpassableAreaProxy& rhs) const;

public:
    const PolygonProxy* geom() const { return _geom.get(); };
    PolygonProxy* mutable_geom();
    bool clear_geom();
    Optional<int> type() const { return _type; };
    bool set_type(Optional<int> d);
    Optional<int> kind() const { return _kind; };
    bool set_kind(Optional<int> k);

private:
    std::shared_ptr<PolygonProxy> _geom;
    Optional<int> _type;
    Optional<int> _kind;
};

class NodeProxy;

class JunctionProxy : public FeatureWithIDProxyBase
{
public:
    typedef RoadPB::Junction message_type;
    const static int ELEM_ID_TYPE = RoadPB::FeatureID::JUNCTION;

    JunctionProxy() : FeatureWithIDProxyBase(), _node_id(this), _areas(this), _boundarys(this),
                        _conn_groups(this), _in_groups(this), _out_groups(this) {};
    JunctionProxy(FeatureProxyBase* p) : FeatureWithIDProxyBase(p), _node_id(this), _areas(this),
                        _boundarys(this), _conn_groups(this), _in_groups(this), _out_groups(this) {
        make_ref_record();
    };
    virtual ~JunctionProxy() {
        _node_id.reset();
        _boundarys.clear();
        _conn_groups.clear();
        _in_groups.clear();
        _out_groups.clear();
    };
    virtual void make_ref_record() const override {
        _ref_record.reset(new JunctionRefRecord(const_cast<JunctionProxy*>(this)));
        _ref_record->reset_container(_ref_record);
    };

    virtual int feature_id_type() const override { return ELEM_ID_TYPE; };
    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;

    virtual bool is_valid() const override;
    virtual bool correct_tile_refs(RoadGeometryManager* mgr) override;
    virtual bool correct_content(RoadGeometryManager* mgr) override;
    virtual bool remake_proxy(RoadGeometryManager* mgr) override;
    virtual bool judge_editable(RoadGeometryManager* mgr) override;
    
    virtual bool make_id_index() override {
        return make_index<RoadPB::Junction>();
    };
    virtual bool merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) override;
    virtual bool revert() override {
        return revert_base<JunctionProxy>();
    };
    virtual void mark_changed() override {
        mark_changed_and_copy<JunctionProxy>();
    };

    const std::shared_ptr<const JunctionProxy> origin() const {
        return origin_elem<JunctionProxy>();
    };
    const std::shared_ptr<const JunctionProxy> remote() const {
        return remote_elem<JunctionProxy>();
    };
    const std::shared_ptr<const JunctionProxy> local() const {
        return local_elem<JunctionProxy>();
    };

public:
    const FeatureReference<NodeProxy> node_id() const { return _node_id; };
    FeatureReference<NodeProxy>* mutable_node_id();
    const SharedProxyVector<ImpassableAreaProxy>& areas() const { return _areas; };
    SharedProxyVector<ImpassableAreaProxy>* mutable_areas();
    const FeatureMemberVector<RoadBoundaryProxy>& boundarys() const { return _boundarys; };
    FeatureMemberVector<RoadBoundaryProxy>* mutable_boundarys();
    const FeatureMemberVector<LaneGroupProxy>& conn_groups() const { return _conn_groups; };
    FeatureMemberVector<LaneGroupProxy>* mutable_conn_groups();
    const FeatureReferenceVector<LaneGroupProxy>& in_groups() const { return _in_groups; };
    FeatureReferenceVector<LaneGroupProxy>* mutable_in_groups();
    const FeatureReferenceVector<LaneGroupProxy>& out_groups() const { return _out_groups; };
    FeatureReferenceVector<LaneGroupProxy>* mutable_out_groups();

    virtual Vector3D wgs84_pos() const override;
    virtual const Vector3D& global_pos(const RoadGeometryManager* mg = nullptr) const override;

protected:
    mutable Vector3D _global_pos;
private:
    FeatureReference<NodeProxy> _node_id;
    SharedProxyVector<ImpassableAreaProxy> _areas;
    FeatureMemberVector<RoadBoundaryProxy> _boundarys;
    FeatureMemberVector<LaneGroupProxy> _conn_groups;
    FeatureReferenceVector<LaneGroupProxy> _in_groups;
    FeatureReferenceVector<LaneGroupProxy> _out_groups;

    class JunctionRefRecord : public RefRecordWithTileBase
    {
    public:
        JunctionRefRecord(FeatureWithIDProxyBase* elem) : RefRecordWithTileBase(elem) {};

        virtual const int* contain_members_offsets() const {
            static int _offs[] = { offsetof(JunctionProxy, _boundarys),
                                   offsetof(JunctionProxy, _conn_groups), -1 };
            return _offs;
        };
        virtual const int* contain_refs_offsets() const {
            static int _offs[] = { offsetof(JunctionProxy, _in_groups),
                                   offsetof(JunctionProxy, _out_groups), -1 };
            return _offs;
        };
        virtual const int* contain_ref_offsets() const {
            static int _offs[] = { offsetof(JunctionProxy, _node_id), -1 };
            return _offs;
        };
    };
};

};
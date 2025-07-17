#pragma once
#include <memory>
#include <string>
#include "dynamics/odd.pb.h"
#include "feature_with_id_proxy_base.h"
#include "common_proxy.h"

namespace data_access_engine {

class RelationProxy : public FeatureWithRefProxyBase
{
public:
	typedef RoadPB::Dynamic_Relation message_type;

	RelationProxy() {};
	RelationProxy(FeatureProxyBase* p) : FeatureWithRefProxyBase(p) {
        make_ref_record();
    };
	virtual ~RelationProxy() {
		_id.reset();
	};

    virtual void make_ref_record() const override {
        _ref_record.reset(new RelationRefRecord(const_cast<RelationProxy*>(this)));
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

	bool operator==(const RelationProxy& rhs) const {
		return (compare(rhs) == 0);
	};
	bool operator<(const RelationProxy& rhs) const {
		return (compare(rhs) < 0);
	};
	bool operator!=(const RelationProxy& rhs) const {
		return (compare(rhs) != 0);
	};
	int compare(const RelationProxy& rhs) const;

public:
	const FeatureReferenceAny& id() const { return _id; };
	FeatureReferenceAny* mutable_id();
	const PointProxy* stp() const { return _stp.get(); };
	PointProxy* mutable_stp();
	bool clear_stp();
	const PointProxy* edp() const { return _edp.get(); };
	PointProxy* mutable_edp();
	bool clear_edp();

private:
    friend class DynamicProxy;
    
	FeatureReferenceAny _id;
	std::shared_ptr<PointProxy> _stp;
	std::shared_ptr<PointProxy> _edp;

    class RelationRefRecord : public RefRecordBase
    {
    public:
        RelationRefRecord(FeatureWithRefProxyBase* f) : RefRecordBase(f) {};

        virtual const int* contain_ref_offsets() const {
            static int _offs[] = { offsetof(RelationProxy, _id), -1 };
            return _offs;
        };
    };
};

class DynamicProxy : public FeatureWithIDProxyBase
{
public:
    typedef RoadPB::Dynamic message_type;
    const static int ELEM_ID_TYPE = RoadPB::FeatureID::ODD;

    DynamicProxy() : FeatureWithIDProxyBase() {};
    DynamicProxy(FeatureProxyBase* p) : FeatureWithIDProxyBase(p) {
        make_ref_record();
    };
    virtual ~DynamicProxy() {};

    virtual void make_ref_record() const override {
        _ref_record.reset(new DynamicRefRecord(const_cast<DynamicProxy*>(this)));

    };

    virtual int feature_id_type() const override { return ELEM_ID_TYPE; };
    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;

    virtual bool is_editable() const override;
    virtual bool is_valid() const override;
    virtual bool correct_tile_refs(RoadGeometryManager* mgr) override;
    virtual bool correct_content(RoadGeometryManager* mgr) override;
    virtual bool remake_proxy(RoadGeometryManager* mgr) override;

    virtual bool make_id_index() override {
        return make_index<RoadPB::Dynamic>();
    };
    virtual void mark_changed() override {
        mark_changed_and_copy<DynamicProxy>();
    };
    virtual bool merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) override;
    virtual bool revert() override {
        return revert_base<DynamicProxy>();
    };

    const std::shared_ptr<const DynamicProxy> origin() const {
        return origin_elem<DynamicProxy>();
    };
    const std::shared_ptr<const DynamicProxy> remote() const {
        return remote_elem<DynamicProxy>();
    };
    const std::shared_ptr<const DynamicProxy> local() const {
        return local_elem<DynamicProxy>();
    };

public:
    Optional<bool> is_odd() const { return _is_odd; };
    bool set_is_odd(Optional<bool> o);
    const CircleProxy* range() const { return _range.get(); };
    CircleProxy* mutable_range();
    bool clear_range();
    const PolylineProxy* lines() const { return _lines.get(); };
    PolylineProxy* mutable_lines();
    bool clear_lines();
    const PolygonProxy* area() const { return _area.get(); };
    PolygonProxy* mutable_area();
    bool clear_area();
    const RelationProxy* relation() const { return _relation.get(); };
    RelationProxy* mutable_relation();
    bool clear_relation();
    Optional<float> heading() const { return _heading; };
    bool set_heading(Optional<float> h);

    Optional<int> type() const { return _type; };
    bool set_type(Optional<int> t);
    Optional<int64_t> reason() const { return _reason; };
    bool set_reason(Optional<int64_t> r);
    Optional<int> action() const { return _action; };
    bool set_action(Optional<int> a);

    Optional<int64_t> int_val() const { return _int_val; };
    bool set_int_val(Optional<int64_t> i);
    Optional<double> double_val() const { return _double_val; };
    bool set_double_val(Optional<double> d);
    const Optional<std::string>& binary_val() const { return _binary_val; };
    bool set_binary_val(const Optional<std::string>& b);

private:
    Optional<bool> _is_odd;
    std::shared_ptr<CircleProxy> _range;
    std::shared_ptr<PolylineProxy> _lines;
    std::shared_ptr<PolygonProxy> _area;
    std::shared_ptr<RelationProxy> _relation;
    Optional<float> _heading;

    Optional<int> _type;
    Optional<int64_t> _reason;
    Optional<int> _action;

    Optional<int64_t> _int_val;
    Optional<double> _double_val;
    Optional<std::string> _binary_val;

    class DynamicRefRecord : public RefRecordBase
    {
    public:
        DynamicRefRecord(FeatureWithIDProxyBase* f) : RefRecordBase(f) {};
        
        virtual const int* contain_sub_ref_offs() const override {
            static int _offs[] = { offsetof(DynamicProxy, _relation), -1 };
            return _offs;
        };
    };
};

}; //data_access_engine

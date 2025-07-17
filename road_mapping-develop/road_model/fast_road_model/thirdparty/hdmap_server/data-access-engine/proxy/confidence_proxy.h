#pragma once
#include <memory>
#include <string>
#include "confidence/confidence.pb.h"
#include "feature_with_id_proxy_base.h"

namespace data_access_engine {

class DataQualityProxy : public FeatureWithIDProxyBase
{
public:
	typedef RoadPB::DataQuality message_type;
	const static int ELEM_ID_TYPE = RoadPB::FeatureID::CONFIDENCE;

	DataQualityProxy() : FeatureWithIDProxyBase() {};
	DataQualityProxy(FeatureProxyBase* p) : FeatureWithIDProxyBase(p) {
        make_ref_record();
    };
	virtual ~DataQualityProxy() {
        _feat_id.reset();
    };

	virtual void make_ref_record() const override {
		_ref_record.reset(new DataQualityRefRecord(const_cast<DataQualityProxy*>(this)));
        _ref_record->reset_container(_ref_record);
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
        return make_index<RoadPB::DataQuality>();
    };
    virtual void mark_changed() override {
        mark_changed_and_copy<DataQualityProxy>();
    };
    virtual bool merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) override;
    virtual bool revert() override {
        return revert_base<DataQualityProxy>();
    };

    const std::shared_ptr<const DataQualityProxy> origin() const {
        return origin_elem<DataQualityProxy>();
    };
    const std::shared_ptr<const DataQualityProxy> remote() const {
        return remote_elem<DataQualityProxy>();
    };
    const std::shared_ptr<const DataQualityProxy> local() const {
        return local_elem<DataQualityProxy>();
    };

public:
    Optional<int> data_source() const { return _data_source; };
    bool set_data_source(Optional<int> s);
    Optional<int> data_weather() const { return _data_weather; };
    bool set_data_weather(Optional<int> w);
    Optional<int> sensor_type() const { return _sensor_type; };
    bool set_sensor_type(Optional<int> s);
    Optional<int> work_manner() const { return _work_manner; };
    bool set_work_manner(Optional<int> w);
    Optional<int64_t> create_time() const { return _create_time; };
    bool set_create_time(Optional<int64_t> c);
    Optional<int> confidence() const { return _confidence; };
    bool set_confidence(Optional<int> c);
    const FeatureReferenceAny& feat_id() const { return _feat_id; };
    FeatureReferenceAny* mutable_feat_id();

private:
    Optional<int> _data_source;
    Optional<int> _data_weather;
    Optional<int> _sensor_type;
    Optional<int> _work_manner;
    Optional<int64_t> _create_time;
    Optional<int> _confidence;
    FeatureReferenceAny _feat_id;

    class DataQualityRefRecord : public RefRecordBase
    {
    public:
        DataQualityRefRecord(FeatureWithIDProxyBase* f) : RefRecordBase(f) {};
        
        virtual const int* contain_ref_offsets() const {
            static int _offs[] = { offsetof(DataQualityProxy, _feat_id), -1 };
            return _offs;
        };
    };
};

}; //data_access_engine


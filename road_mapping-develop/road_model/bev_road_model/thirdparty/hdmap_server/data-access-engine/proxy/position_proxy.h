#pragma once
#include <memory>
#include <string>
#include "position/position.pb.h"
#include "feature_with_id_proxy_base.h"
#include "common_proxy.h"

namespace data_access_engine {

class LaneGroupProxy;

class PositionObjectProxy : public FeatureWithIDProxyBase
{
public:
    typedef RoadPB::PositionObject message_type;
    const static int ELEM_ID_TYPE = RoadPB::FeatureID::POSITION_OBJ;

    PositionObjectProxy() : FeatureWithIDProxyBase(), _pos_sync_cnt(-1), _lane_groups(this) {};
    PositionObjectProxy(FeatureProxyBase* p) : FeatureWithIDProxyBase(p), 
                                             _pos_sync_cnt(-1), _lane_groups(this) {
        make_ref_record();
    };
    virtual ~PositionObjectProxy() {
        _lane_groups.clear();
    };

    virtual void make_ref_record() const override {
        _ref_record.reset(new ObjRefRecord(const_cast<PositionObjectProxy*>(this)));
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
    
    virtual bool make_id_index() override {
        return make_index<RoadPB::PositionObject>();
    };
    virtual void mark_changed() override {
        mark_changed_and_copy<PositionObjectProxy>();
    };
    virtual bool merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) override;
    virtual bool revert() override {
        return revert_base<PositionObjectProxy>();
    };

    const std::shared_ptr<const PositionObjectProxy> origin() const {
        return origin_elem<PositionObjectProxy>();
    };
    const std::shared_ptr<const PositionObjectProxy> remote() const {
        return remote_elem<PositionObjectProxy>();
    };
    const std::shared_ptr<const PositionObjectProxy> local() const {
        return local_elem<PositionObjectProxy>();
    };

public:
    const PolygonProxy* border() const { return _border.get(); };
    PolygonProxy* mutable_border();
    bool clear_border();
    const CircleProxy* circle() const { return _circle.get(); };
    CircleProxy* mutable_circle();
    bool clear_circle();
    const PolylineProxy* pole() const { return _pole.get(); };
    PolylineProxy* mutable_pole();
    bool clear_pole();
    Optional<int> type() const { return _type; };
    bool set_type(Optional<int> t);
    Optional<int> sub_type() const { return _sub_type; };
    bool set_sub_type(Optional<int> t);
    const Optional<std::string>& content() const { return _content; };
    bool set_content(const Optional<std::string>& c);
    const FeatureReferenceVector<LaneGroupProxy>& lane_groups() const { return _lane_groups; };
    FeatureReferenceVector<LaneGroupProxy>* mutable_lane_groups();

    virtual Vector3D wgs84_pos() const override;
    virtual const Vector3D& global_pos(const RoadGeometryManager* mgr = nullptr) const override;
   
protected:
    mutable Vector3D _global_pos;

private:
    void sync_wgs_to_global_pos(const RoadGeometryManager* mgr) const;

    std::shared_ptr<PolygonProxy> _border;
    std::shared_ptr<CircleProxy> _circle;
    std::shared_ptr<PolylineProxy> _pole;
    Optional<int> _type;
    Optional<int> _sub_type;
    Optional<std::string> _content;
    FeatureReferenceVector<LaneGroupProxy> _lane_groups;

    mutable int _pos_sync_cnt;

    class ObjRefRecord : public RefRecordWithTileBase
    {
    public:
        ObjRefRecord(FeatureWithIDProxyBase* elem) : RefRecordWithTileBase(elem) {};

        virtual const int* contain_refs_offsets() const {
            static int _offs[] = { offsetof(PositionObjectProxy, _lane_groups), -1 };
            return _offs;
        };
    };
};

}; // data_access_engine
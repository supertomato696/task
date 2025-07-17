#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include "feature_with_id_proxy_base.h"
#include "services/roadserver.pb.h"

namespace data_access_engine {
class LinkProxy;
class NodeProxy;
class LaneProxy;
class LaneBoundaryProxy;
class LaneGroupProxy;
class JunctionProxy;
class TrafficInfoProxy;
class PositionObjectProxy;
class RoadBoundaryProxy;
class DataQualityProxy;
class DynamicProxy;

class TileInfoProxy : public FeatureWithIDProxyBase
{
public:
    typedef services::TileInfo message_type;
    TileInfoProxy();
    TileInfoProxy(FeatureProxyBase* p) : TileInfoProxy() {};
    TileInfoProxy(int tile_id) : TileInfoProxy() {
        _tile_id = tile_id;
        mutable_id()->set_tileid(tile_id);
    };
    virtual ~TileInfoProxy();
    virtual void make_ref_record() const override {
        _ref_record.reset(new RefRecordBase(const_cast<TileInfoProxy*>(this)));
    };

    virtual int feature_id_type() const { return -1; };
    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;
    virtual bool make_id_index() override { return false; }
    virtual void mark_changed() {
        FeatureProxyBase::mark_changed();
    };

    virtual bool is_valid() const override;
    virtual bool correct_tile_refs(RoadGeometryManager* mgr) override;
    virtual bool correct_content(RoadGeometryManager* mgr) override;
    virtual bool remake_proxy(RoadGeometryManager* mgr) override;
    virtual void clear_changed() override;
    virtual bool merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) override;
    virtual bool revert() override;
    virtual void reset_merge() override;

    void load_maps();
    void relink_parent();
    void clear_parent();
    void merge_tile(TileInfoProxy* tile);    
    bool append_to_issue(RoadGeometryManager* mgr);
    void restart_merge() {
        if (_change_rec == nullptr) {
            return;
        }
        _change_rec->_local_elem.reset();
        _change_rec->_remote_elem.reset();
        _change_rec->_origin_elem.reset();
    };

    bool is_empty() const;

public:
    int tile_id() const { return _tile_id; };
    int64_t version() const { return _version; };
    void set_version(int64_t ver) { _version = ver; };
    const Optional<std::string>& last_editor() const { return _last_editor; };
    
    const SharedProxyVector<LinkProxy>& links() const { return _links; };
    SharedProxyVector<LinkProxy>& links() { return _links; };
    SharedProxyVector<LinkProxy>* mutable_links() { return &_links; };
    const FeatureReferenceVector<LinkProxy>& link_refs() const { return _link_refs; };
    FeatureReferenceVector<LinkProxy>& link_refs() { return _link_refs; };
    FeatureReferenceVector<LinkProxy>* mutable_link_refs() { return &_link_refs; };
    
    const SharedProxyVector<NodeProxy>& nodes() const { return _nodes; };
    SharedProxyVector<NodeProxy>& nodes() { return _nodes; };
    SharedProxyVector<NodeProxy>* mutable_nodes() { return &_nodes; };
    const FeatureReferenceVector<NodeProxy>& node_refs() const { return _node_refs; };
    FeatureReferenceVector<NodeProxy>& node_refs() { return _node_refs; };
    FeatureReferenceVector<NodeProxy>* mutable_node_refs() { return &_node_refs; };

    const SharedProxyVector<LaneProxy>& lanes() const { return _lanes; };
    SharedProxyVector<LaneProxy>& lanes() { return _lanes; };
    SharedProxyVector<LaneProxy>* mutable_lanes() { return &_lanes; };
    const FeatureReferenceVector<LaneProxy>& lane_refs() const { return _lane_refs; };
    FeatureReferenceVector<LaneProxy>& lane_refs() { return _lane_refs; };
    FeatureReferenceVector<LaneProxy>* mutable_lane_refs() { return &_lane_refs; };

    const SharedProxyVector<LaneBoundaryProxy>& lane_boundarys() const { return _lane_boundarys; };
    SharedProxyVector<LaneBoundaryProxy>& lane_boundarys() { return _lane_boundarys; };
    SharedProxyVector<LaneBoundaryProxy>* mutable_lane_boundarys() { return &_lane_boundarys; };
    const FeatureReferenceVector<LaneBoundaryProxy>& lane_boundary_refs() const { return _lane_boundary_refs; };
    FeatureReferenceVector<LaneBoundaryProxy>& lane_boundary_refs() { return _lane_boundary_refs; };
    FeatureReferenceVector<LaneBoundaryProxy>* mutable_lane_boundary_refs() { return &_lane_boundary_refs; };

    const SharedProxyVector<LaneGroupProxy>& lane_groups() const { return _lane_groups; };
    SharedProxyVector<LaneGroupProxy>& lane_groups() { return _lane_groups; };
    SharedProxyVector<LaneGroupProxy>* mutable_lane_groups() { return &_lane_groups; };
    const FeatureReferenceVector<LaneGroupProxy>& lane_group_refs() const { return _lane_group_refs; };
    FeatureReferenceVector<LaneGroupProxy>& lane_group_refs() { return _lane_group_refs; };
    FeatureReferenceVector<LaneGroupProxy>* mutable_lane_group_refs() { return &_lane_group_refs; };

    const SharedProxyVector<JunctionProxy>& junctions() const { return _junctions; };
    SharedProxyVector<JunctionProxy>& junctions() { return _junctions; };
    SharedProxyVector<JunctionProxy>* mutable_junctions() { return &_junctions; };
    const FeatureReferenceVector<JunctionProxy>& junction_refs() const { return _junction_refs; };
    FeatureReferenceVector<JunctionProxy>& junction_refs() { return _junction_refs; };
    FeatureReferenceVector<JunctionProxy>* mutable_junction_refs() { return &_junction_refs; };

    const SharedProxyVector<TrafficInfoProxy>& traffic_infos() const { return _traffic_infos; };
    SharedProxyVector<TrafficInfoProxy>& traffic_infos() { return _traffic_infos; };
    SharedProxyVector<TrafficInfoProxy>* mutable_traffic_infos() { return &_traffic_infos; };
    const FeatureReferenceVector<TrafficInfoProxy>& traffic_info_refs() const { return _traffic_info_refs; };
    FeatureReferenceVector<TrafficInfoProxy>& traffic_info_refs() { return _traffic_info_refs; };
    FeatureReferenceVector<TrafficInfoProxy>* mutable_traffic_info_refs() { return &_traffic_info_refs; };

    const SharedProxyVector<PositionObjectProxy>& position_objects() const { return _position_objects; };
    SharedProxyVector<PositionObjectProxy>& position_objects() { return _position_objects; };
    SharedProxyVector<PositionObjectProxy>* mutable_position_objects() { return &_position_objects; };
    const FeatureReferenceVector<PositionObjectProxy>& position_object_refs() const { return _position_object_refs; };
    FeatureReferenceVector<PositionObjectProxy>& position_object_refs() { return _position_object_refs; };
    FeatureReferenceVector<PositionObjectProxy>* mutable_position_object_refs() { return &_position_object_refs; };

    const SharedProxyVector<RoadBoundaryProxy>& road_boundarys() const { return _road_boundarys; };
    SharedProxyVector<RoadBoundaryProxy>& road_boundarys() { return _road_boundarys; };
    SharedProxyVector<RoadBoundaryProxy>* mutable_road_boundarys() { return &_road_boundarys; };
    const FeatureReferenceVector<RoadBoundaryProxy>& road_boundary_refs() const { return _road_boundary_refs; };
    FeatureReferenceVector<RoadBoundaryProxy>& road_boundary_refs() { return _road_boundary_refs; };
    FeatureReferenceVector<RoadBoundaryProxy>* mutable_road_boundary_refs() { return &_road_boundary_refs; };

    const SharedProxyVector<DataQualityProxy>& data_qualitys() const { return _data_qualitys; };
    SharedProxyVector<DataQualityProxy>& data_qualitys() { return _data_qualitys; };
    SharedProxyVector<DataQualityProxy>* mutable_data_qualitys() { return &_data_qualitys; };
    const FeatureReferenceVector<DataQualityProxy>& data_quality_refs() const { return _data_quality_refs; };
    FeatureReferenceVector<DataQualityProxy>& data_quality_refs() { return _data_quality_refs; };
    FeatureReferenceVector<DataQualityProxy>* mutable_data_quality_refs() { return &_data_quality_refs; };

    const SharedProxyVector<DynamicProxy>& dynamics() const { return _dynamics; };
    SharedProxyVector<DynamicProxy>& dynamics() { return _dynamics; };
    SharedProxyVector<DynamicProxy>* mutable_dynamics() { return &_dynamics; };
    const FeatureReferenceVector<DynamicProxy>& dynamic_refs() const { return _dynamic_refs; };
    FeatureReferenceVector<DynamicProxy>& dynamic_refs() { return _dynamic_refs; };
    FeatureReferenceVector<DynamicProxy>* mutable_dynamic_refs() { return &_dynamic_refs; };

    void shallow_copy(TileInfoProxy* tile);

    const std::shared_ptr<const TileInfoProxy> changed_tile() const {
        if (_change_rec == nullptr) {
            _change_rec = new FeatureChangeRecord();
        }
        return std::dynamic_pointer_cast<TileInfoProxy>(_change_rec->_local_elem);
    };
    std::shared_ptr<TileInfoProxy> mutable_changed_tile() {
        if (_change_rec == nullptr) {
            _change_rec = new FeatureChangeRecord();
        }
        if (!_change_rec->_local_elem) {
            auto t = FeatureProxyBase::create_proxy<TileInfoProxy>();
            _change_rec->_local_elem.reset(t);
            t->_tile_id = _tile_id;
            t->_version = _version;
        }
        return std::dynamic_pointer_cast<TileInfoProxy>(_change_rec->_local_elem);
    };
    const std::shared_ptr<const TileInfoProxy> conflicted_tile() const {
        if (_change_rec == nullptr) {
            _change_rec = new FeatureChangeRecord();
        }
        return std::dynamic_pointer_cast<TileInfoProxy>(_change_rec->_remote_elem);
    };
    std::shared_ptr<TileInfoProxy> mutable_conflict_tile() {
        if (_change_rec == nullptr) {
            _change_rec = new FeatureChangeRecord();
        }
        if (!_change_rec->_remote_elem) {
            auto t = FeatureProxyBase::create_proxy<TileInfoProxy>();
            _change_rec->_remote_elem.reset(t);
            t->_tile_id = _tile_id;
            t->_version = _version;
        }
        return std::dynamic_pointer_cast<TileInfoProxy>(_change_rec->_remote_elem);
    };
    const std::shared_ptr<const TileInfoProxy> origin_tile() const {
        if (_change_rec == nullptr) {
            _change_rec = new FeatureChangeRecord();
        }
        return std::dynamic_pointer_cast<TileInfoProxy>(_change_rec->_origin_elem);
    };
    std::shared_ptr<TileInfoProxy> mutable_origin_tile() {
        if (_change_rec == nullptr) {
            _change_rec = new FeatureChangeRecord();
        }
        if (!_change_rec->_origin_elem) {
            auto t = FeatureProxyBase::create_proxy<TileInfoProxy>();
            _change_rec->_origin_elem.reset(t);
            t->_tile_id = _tile_id;
            t->_version = _version;
        }
        return std::dynamic_pointer_cast<TileInfoProxy>(_change_rec->_origin_elem);
    };
    bool add_element_to_tile(std::shared_ptr<FeatureWithIDProxyBase> elem);
    bool add_reference_to_tile(FeatureReferenceBase* ref);

protected:
    friend class RoadGeometryManager;
    template <class T> friend class FeatureReference;

    void set_tile_id(int tid) { _tile_id = tid; };

private:
    int _tile_id;
    int64_t _version;
    Optional<std::string> _last_editor;

    SharedProxyVector<LinkProxy> _links;
    FeatureReferenceVector<LinkProxy> _link_refs;
    SharedProxyVector<NodeProxy> _nodes;
    FeatureReferenceVector<NodeProxy> _node_refs;
    SharedProxyVector<LaneProxy> _lanes;
    FeatureReferenceVector<LaneProxy> _lane_refs;
    SharedProxyVector<LaneBoundaryProxy> _lane_boundarys;
    FeatureReferenceVector<LaneBoundaryProxy> _lane_boundary_refs;
    SharedProxyVector<LaneGroupProxy> _lane_groups;
    FeatureReferenceVector<LaneGroupProxy> _lane_group_refs;
    SharedProxyVector<JunctionProxy> _junctions;
    FeatureReferenceVector<JunctionProxy> _junction_refs;
    SharedProxyVector<TrafficInfoProxy> _traffic_infos;
    FeatureReferenceVector<TrafficInfoProxy> _traffic_info_refs;
    SharedProxyVector<PositionObjectProxy> _position_objects;
    FeatureReferenceVector<PositionObjectProxy> _position_object_refs;
    SharedProxyVector<RoadBoundaryProxy> _road_boundarys;
    FeatureReferenceVector<RoadBoundaryProxy> _road_boundary_refs;
    SharedProxyVector<DataQualityProxy> _data_qualitys;
    FeatureReferenceVector<DataQualityProxy> _data_quality_refs;
    SharedProxyVector<DynamicProxy> _dynamics;
    FeatureReferenceVector<DynamicProxy> _dynamic_refs;

protected:
    std::unordered_map<std::string, std::weak_ptr<const LinkProxy>> _link_map;
    std::unordered_map<std::string, std::weak_ptr<const NodeProxy>> _node_map;
    std::unordered_map<std::string, std::weak_ptr<const LaneProxy>> _lane_map;
    std::unordered_map<std::string, std::weak_ptr<const LaneBoundaryProxy>> _lane_boundary_map;
    std::unordered_map<std::string, std::weak_ptr<const LaneGroupProxy>> _lane_group_map;
    std::unordered_map<std::string, std::weak_ptr<const JunctionProxy>> _junction_map;
    std::unordered_map<std::string, std::weak_ptr<const TrafficInfoProxy>> _traffic_info_map;
    std::unordered_map<std::string, std::weak_ptr<const PositionObjectProxy>> _position_object_map;
    std::unordered_map<std::string, std::weak_ptr<const RoadBoundaryProxy>> _road_boundary_map;
    std::unordered_map<std::string, std::weak_ptr<const DataQualityProxy>> _data_quality_map;
    std::unordered_map<std::string, std::weak_ptr<const DynamicProxy>> _dynamic_map;
};

typedef std::vector<std::shared_ptr<TileInfoProxy>> TileInfoList;
typedef std::shared_ptr<TileInfoProxy> TileInfoPtr;
typedef std::map<int, TileInfoPtr> ID2TileMap;

}; //data_access_engine

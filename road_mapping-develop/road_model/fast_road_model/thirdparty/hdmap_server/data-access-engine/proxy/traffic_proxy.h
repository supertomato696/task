#pragma once
#include <memory>
#include <string>
#include "traffic/traffic.pb.h"
#include "feature_with_id_proxy_base.h"
#include "common_proxy.h"

namespace data_access_engine {

class PositionObjectProxy;
class LaneProxy;
class LaneGroupProxy;

class TrafficConditionProxy : public FeatureProxyBase
{
public:
    typedef RoadPB::TrafficCondition message_type;

    TrafficConditionProxy() {};
    TrafficConditionProxy(FeatureProxyBase* p) : FeatureProxyBase(p) {};
    virtual ~TrafficConditionProxy() {};

    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;

    virtual bool is_valid() const override;
    virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
                            const std::shared_ptr<const FeatureProxyBase>& base,
                            const std::shared_ptr<const FeatureProxyBase>& remote) override;

    bool operator==(const TrafficConditionProxy& rhs) const {
        return (compare(rhs) == 0);
    };
    bool operator<(const TrafficConditionProxy& rhs) const {
        return (compare(rhs) < 0);
    };
    bool operator!=(const TrafficConditionProxy& rhs) const {
        return (compare(rhs) != 0);
    };
    int compare(const TrafficConditionProxy& rhs) const;

public:
    Optional<int> type() const { return _type; };
    bool set_type(Optional<int> t) {
        if (!is_editable()) {
            return false;
        }
        if (_type != t) {
            mark_changed();
        }
        _type = t;
        return true;
    };
    Optional<int> value() const { return _value; };
    bool set_value(Optional<int> v) {
        if (!is_editable()) {
            return false;
        }
        if (_value != v) {
            mark_changed();
        }
        _value = v;
        return true;
    };
    Optional<int> vehicle_type_mask() const { return _vehicle_type_mask; };
    bool set_vehicle_type_mask(Optional<int> v) {
        if (!is_editable()) {
            return false;
        }
        if (_vehicle_type_mask != v) {
            mark_changed();
        }
        _vehicle_type_mask = v;
        return true;
    };
    Optional<int> locd_type() const { return _load_type; };
    bool set_load_type(Optional<int> t) {
        if (!is_editable()) {
            return false;
        }
        if (_load_type != t) {
            mark_changed();
        }
        _load_type = t;
        return true;
    };
    Optional<int> start_minues() const { return _start_minues; };
    bool set_start_minues(Optional<int> t) {
        if (!is_editable()) {
            return false;
        }
        if (_start_minues != t) {
            mark_changed();
        }
        _start_minues = t;
        return true;
    };
    Optional<int> end_minues() const { return _end_minues; };
    bool set_end_minues(Optional<int> t) {
        if (!is_editable()) {
            return false;
        }
        if (_end_minues != t) {
            mark_changed();
        }
        _end_minues = t;
        return true;
    };
    Optional<int> weather() const { return _weather; };
    bool set_weather(Optional<int> w) {
        if (!is_editable()) {
            return false;
        }
        if (_weather != w) {
            mark_changed();
        }
        _weather = w;
        return true;
    };
    Optional<int> fuzzy_time() const { return _fuzzy_time; };
    bool set_fuzzy_time(Optional<int> t) {
        if (!is_editable()) {
            return false;
        }
        if (_fuzzy_time != t) {
            mark_changed();
        }
        _fuzzy_time = t;
        return true;
    };
    Optional<int> direction() const { return _direction; };
    bool set_direction(Optional<int> d) {
        if (!is_editable()) {
            return false;
        }
        if (_direction != d) {
            mark_changed();
        }
        _direction = d;
        return true;
    };

private:
    Optional<int> _type;
    Optional<int> _value;
    Optional<int> _vehicle_type_mask;
    Optional<int> _load_type;
    Optional<int> _start_minues;
    Optional<int> _end_minues;
    Optional<int> _weather;
    Optional<int> _fuzzy_time;
    Optional<int> _direction;
};

class TrafficSignProxy : public FeatureProxyBase {
public:
    typedef RoadPB::TrafficSign message_type;

    TrafficSignProxy() {};
    TrafficSignProxy(FeatureProxyBase* p) : FeatureProxyBase(p) {};
    virtual ~TrafficSignProxy() {};

    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;

    virtual bool is_valid() const override;
    virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
                            const std::shared_ptr<const FeatureProxyBase>& base,
                            const std::shared_ptr<const FeatureProxyBase>& remote) override;

    bool operator==(const TrafficSignProxy& rhs) const {
        return (compare(rhs) == 0);
    };
    bool operator<(const TrafficSignProxy& rhs) const {
        return (compare(rhs) < 0);
    };
    bool operator!=(const TrafficSignProxy& rhs) const {
        return (compare(rhs) != 0);
    };
    int compare(const TrafficSignProxy& rhs) const;

public:
    Optional<int> type() const { return _type; };
    bool set_type(Optional<int> t) {
        if (!is_editable()) {
            return false;
        }
        if (_type != t) {
            mark_changed();
        }
        _type = t;
        return true;
    };
    Optional<int> shape() const { return _shape; };
    bool set_shape(Optional<int> s) {
        if (!is_editable()) {
            return false;
        }
        if (_shape != s) {
            mark_changed();
        }
        _shape = s;
        return true;
    };
    Optional<float> value() const { return _value; };
    bool set_value(Optional<float> v) {
        if (!is_editable()) {
            return false;
        }
        if (_value != v) {
            mark_changed();
        }
        _value = v;
        return true;
    };
    const Optional<std::string>& content() const { return _content; };
    bool set_content(const Optional<std::string>& c) {
        if (!is_editable()) {
            return false;
        }
        if (_content != c) {
            mark_changed();
        }
        _content = c;
        return true;
    };
    Optional<bool> variable() const { return _variable; };
    bool set_variable(Optional<bool> v) {
        if (!is_editable()) {
            return false;
        }
        if (_variable != v) {
            mark_changed();
        }
        _variable = v;
        return true;
    };
    const SharedProxyVector<TrafficConditionProxy>& panels() const { return _panels; };
    SharedProxyVector<TrafficConditionProxy>* mutable_panels() {
        if (!is_editable()) {
            return nullptr;
        }
        return &_panels;
    };

private:
    Optional<int> _type;
    Optional<int> _shape;
    Optional<float> _value;
    Optional<std::string> _content;
    Optional<bool> _variable;
    SharedProxyVector<TrafficConditionProxy> _panels;
};

/*class LightBulbProxy : public FeatureProxyBase {
public:
    typedef RoadPB::TrafficLight_LightBulb message_type;

    LightBulbProxy() {};
    LightBulbProxy(FeatureProxyBase* p) : FeatureProxyBase(p) {};
    virtual ~LightBulbProxy() {};

    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;

    virtual bool is_valid() const override;
    virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
        const std::shared_ptr<const FeatureProxyBase>& base,
        const std::shared_ptr<const FeatureProxyBase>& remote) override;

    bool operator==(const LightBulbProxy& rhs) const {
        return (compare(rhs) == 0);
    };
    bool operator<(const LightBulbProxy& rhs) const {
        return (compare(rhs) < 0);
    };
    bool operator!=(const LightBulbProxy& rhs) const {
        return (compare(rhs) != 0);
    };
    int compare(const LightBulbProxy& rhs) const;

public:
    Optional<int> color() const { return _color; };
    bool set_color(Optional<int> c) {
        if (!is_editable()) {
            return false;
        }
        if (_color != c) {
            mark_changed();
        }
        _color = c;
        return true;
    };
    Optional<int> shape() const { return _shape; };
    bool set_shape(Optional<int> s) {
        if (!is_editable()) {
            return false;
        }
        if (_shape != s) {
            mark_changed();
        }
        _shape = s;
        return true;
    };
    const CircleProxy* border() const { return _border.get(); };
    CircleProxy* mutable_border() {
        if (!is_editable()) {
            return nullptr;
        }
        if (!_border) {
            _border.reset(FeatureProxyBase::create_proxy<CircleProxy>(this));
        }
        return _border.get();
    };
    bool clear_border() {
        if (!is_editable()) {
            return false;
        }
        if (_border) {
            mark_changed();
        }
        _border.reset();
        return true;
    }

private:
    Optional<int> _color;
    Optional<int> _shape;
    std::shared_ptr<CircleProxy> _border;
};*/

class TrafficLightProxy : public FeatureProxyBase
{
public:
    typedef RoadPB::TrafficLight message_type;

    TrafficLightProxy() {};
    TrafficLightProxy(FeatureProxyBase* p) : FeatureProxyBase(p) {};
    virtual ~TrafficLightProxy() {};

    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;

    virtual bool is_valid() const override;
    virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
        const std::shared_ptr<const FeatureProxyBase>& base,
        const std::shared_ptr<const FeatureProxyBase>& remote) override;

    bool operator==(const TrafficLightProxy& rhs) const {
        return (compare(rhs) == 0);
    };
    bool operator<(const TrafficLightProxy& rhs) const {
        return (compare(rhs) < 0);
    };
    bool operator!=(const TrafficLightProxy& rhs) const {
        return (compare(rhs) != 0);
    };
    int compare(const TrafficLightProxy& rhs) const;

public:
    Optional<int> type() const { return _type; };
    bool set_type(Optional<int> t) {
        if (!is_editable()) {
            return false;
        }
        if (_type != t) {
            mark_changed();
        }
        _type = t;
        return true;
    };
    /*const SharedProxyVector<LightBulbProxy>& bulbs() const { return _bulbs; };
    SharedProxyVector<LightBulbProxy>* mutable_bulbs() {
        if (!is_editable()) {
            return nullptr;
        }
        return &_bulbs;
    };*/
    Optional<int> arrange() const { return _arrange; };
    bool set_arrange(Optional<int> a) {
        if (!is_editable()) {
            return false;
        }
        if (_arrange != a) {
            mark_changed();
        }
        _arrange = a;
        return true;
    };
    /*Optional<int> usage_obj() const { return _usage_obj; };
    bool set_usage_obj(Optional<int> u) {
        if (!is_editable()) {
            return false;
        }
        if (_usage_obj != u) {
            mark_changed();
        }
        _usage_obj = u;
        return true;
    };
    Optional<int> usage_type() const { return _usage_type; };
    bool set_usage_type(Optional<int> u) {
        if (!is_editable()) {
            return false;
        }
        if (_usage_type != u) {
            mark_changed();
        }
        _usage_type = u;
        return true;
    };*/
    Optional<int> bulbs_num() const { return _bulbs_num; };
    bool set_bulbs_num(Optional<int> n) {
        if (!is_editable()) {
            return false;
        }
        if (_bulbs_num != n) {
            mark_changed();
        }
        _bulbs_num = n;
        return true;
    };
    
private:
    Optional<int> _type;
    //SharedProxyVector<LightBulbProxy> _bulbs;
    Optional<int> _arrange;
    //Optional<int> _usage_obj;
    //Optional<int> _usage_type;
    Optional<int> _bulbs_num;
};

class TrafficInfoProxy;

/*class TrafficLightGroupProxy : public FeatureWithRefProxyBase
{
public:
    typedef RoadPB::TrafficLightGroup message_type;

    TrafficLightGroupProxy() : _lights(this) {};
    TrafficLightGroupProxy(FeatureProxyBase* p) : FeatureWithRefProxyBase(p), _lights(this) {
        make_ref_record();
    };
    virtual ~TrafficLightGroupProxy() {};

    virtual void make_ref_record() const override {
        _ref_record.reset(new LGRefRecord(const_cast<TrafficLightGroupProxy*>(this)));
    };

    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;

    virtual bool is_valid() const override;
    virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
                            const std::shared_ptr<const FeatureProxyBase>& base,
                            const std::shared_ptr<const FeatureProxyBase>& remote) override;

    bool operator==(const TrafficLightGroupProxy& rhs) const {
        return (compare(rhs) == 0);
    };
    bool operator<(const TrafficLightGroupProxy& rhs) const {
        return (compare(rhs) < 0);
    };
    bool operator!=(const TrafficLightGroupProxy& rhs) const {
        return (compare(rhs) != 0);
    };
    int compare(const TrafficLightGroupProxy& rhs) const;

public:
    const FeatureReferenceVector<TrafficInfoProxy>& lights() const { return _lights; };
    FeatureReferenceVector<TrafficInfoProxy>* mutable_lights() {
        if (!is_editable()) {
            return nullptr;
        }
        return &_lights;
    };

private:
    FeatureReferenceVector<TrafficInfoProxy> _lights;

    class LGRefRecord : public RefRecordBase
    {
    public:
        LGRefRecord(FeatureWithRefProxyBase* f) : RefRecordBase(f) {};

        virtual const int* contain_refs_offsets() const {
            static int _offs[] = { offsetof(TrafficLightGroupProxy, _lights), -1 };
            return _offs;
        };
    };
};*/

class LaneMarkingProxy : public FeatureProxyBase {
public:
    typedef RoadPB::LaneMarking message_type;

    LaneMarkingProxy() {};
    LaneMarkingProxy(FeatureProxyBase* p) : FeatureProxyBase(p) {};
    virtual ~LaneMarkingProxy() {};

    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;

    virtual bool is_valid() const override;
    virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
        const std::shared_ptr<const FeatureProxyBase>& base,
        const std::shared_ptr<const FeatureProxyBase>& remote) override;

    bool operator==(const LaneMarkingProxy& rhs) const {
        return (compare(rhs) == 0);
    };
    bool operator<(const LaneMarkingProxy& rhs) const {
        return (compare(rhs) < 0);
    };
    bool operator!=(const LaneMarkingProxy& rhs) const {
        return (compare(rhs) != 0);
    };
    int compare(const LaneMarkingProxy& rhs) const;

public:
    Optional<int> type() const { return _type; };
    bool set_type(Optional<int> t) {
        if (!is_editable()) {
            return false;
        }
        if (_type != t) {
            mark_changed();
        }
        _type = t;
        return true;
    };
    Optional<float> value() const { return _value; };
    bool set_value(Optional<float> v) {
        if (!is_editable()) {
            return false;
        }
        if (_value != v) {
            mark_changed();
        }
        _value = v;
        return true;
    };
    const Optional<std::string> content() const { return _content; };
    bool set_content(const Optional<std::string>& c) {
        if (!is_editable()) {
            return false;
        }
        if (_content != c) {
            mark_changed();
        }
        _content = c;
        return true;
    };

private:
    Optional<int> _type;
    Optional<float> _value;
    Optional<std::string> _content;
};

class SpeedBumpProxy : public FeatureProxyBase
{
public:
    typedef RoadPB::SpeedBump message_type;

    SpeedBumpProxy() {};
    SpeedBumpProxy(FeatureProxyBase* p) : FeatureProxyBase(p) {};
    virtual ~SpeedBumpProxy() {};

    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;

    virtual bool is_valid() const override;
    virtual bool merge_proxy(MergeConflictPolicy policy, RoadGeometryManager* mgr,
                            const std::shared_ptr<const FeatureProxyBase>& base,
                            const std::shared_ptr<const FeatureProxyBase>& remote) override;

    bool operator==(const SpeedBumpProxy& rhs) const {
        return (compare(rhs) == 0);
    };
    bool operator<(const SpeedBumpProxy& rhs) const {
        return (compare(rhs) < 0);
    };
    bool operator!=(const SpeedBumpProxy& rhs) const {
        return (compare(rhs) != 0);
    };
    int compare(const SpeedBumpProxy& rhs) const;

public:
    Optional<float> height() const { return _height; };
    bool set_height(Optional<float> h) {
        if (!is_editable()) {
            return false;
        }
        if (_height != h) {
            mark_changed();
        }
        _height = h;
        return true;
    };
    /*Optional<float> speed_limit() const { return _speed_limit; };
    bool set_speed_limit(Optional<float> s) {
        if (!is_editable()) {
            return false;
        }
        if (_speed_limit != s) {
            mark_changed();
        }
        _speed_limit = s;
        return true;
    };*/
    
private:
    Optional<float> _height;
    //Optional<float> _speed_limit;
};

class TrafficInfoProxy : public FeatureWithIDProxyBase
{
public:
    typedef RoadPB::TrafficInfo message_type;
    const static int ELEM_ID_TYPE = RoadPB::FeatureID::TRAFFIC_INFO;

    TrafficInfoProxy() : FeatureWithIDProxyBase(), _objs(this) {};
    TrafficInfoProxy(FeatureProxyBase* p) : FeatureWithIDProxyBase(p), _objs(this) {
        make_ref_record();
    };
    virtual ~TrafficInfoProxy() {
        _objs.clear();
        _lanes.reset();
        _lanegroups.reset();
    };

    virtual void make_ref_record() const override {
        _ref_record.reset(new TrafficRefRecord(const_cast<TrafficInfoProxy*>(this)));
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
        return make_index<RoadPB::TrafficInfo>();
    };
    virtual void mark_changed() override {
        mark_changed_and_copy<TrafficInfoProxy>();
    };
    virtual bool merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) override;
    virtual bool revert() override {
        return revert_base<TrafficInfoProxy>();
    };

    const std::shared_ptr<const TrafficInfoProxy> origin() const {
        return origin_elem<TrafficInfoProxy>();
    };
    const std::shared_ptr<const TrafficInfoProxy> remote() const {
        return remote_elem<TrafficInfoProxy>();
    };
    const std::shared_ptr<const TrafficInfoProxy> local() const {
        return local_elem<TrafficInfoProxy>();
    };

public:
    const FeatureReferenceVector<PositionObjectProxy>& objs() const { return _objs; };
    FeatureReferenceVector<PositionObjectProxy>* mutable_objs();
    Optional<int> type() const { return _type; };
    bool set_type(Optional<int> t);
    const FeatureReference<LaneProxy> lanes() const { return _lanes; };
    FeatureReference<LaneProxy>* mutable_lanes();
    const FeatureReference<LaneGroupProxy> lanegroups() const { return _lanegroups; };
    FeatureReference<LaneGroupProxy>* mutable_lanegroups();
    const TrafficSignProxy* sign() const { return _sign.get(); };
    TrafficSignProxy* mutable_sign();
    bool clear_sign();
    const TrafficLightProxy* light() const { return _light.get(); };
    TrafficLightProxy* mutable_light();
    bool clear_light();
    //const TrafficLightGroupProxy* light_group() const { return _light_group.get(); };
    //TrafficLightGroupProxy* mutable_light_group();
    bool clear_light_group();
    const LaneMarkingProxy* marking() const { return _marking.get(); };
    LaneMarkingProxy* mutable_marking();
    bool clear_marking();
    const SpeedBumpProxy* bump() const { return _bump.get(); };
    SpeedBumpProxy* mutable_bump();
    bool clear_bump();

    virtual Vector3D wgs84_pos() const override;
    virtual const Vector3D& global_pos(const RoadGeometryManager* mgr = nullptr) const override;

private:
    FeatureReferenceVector<PositionObjectProxy> _objs;
    Optional<int> _type;
    FeatureReference<LaneProxy> _lanes;
    FeatureReference<LaneGroupProxy> _lanegroups;
    std::shared_ptr<TrafficSignProxy> _sign;
    std::shared_ptr<TrafficLightProxy> _light;
    //std::shared_ptr<TrafficLightGroupProxy> _light_group;
    std::shared_ptr<LaneMarkingProxy> _marking;
    std::shared_ptr<SpeedBumpProxy> _bump;

    class TrafficRefRecord : public RefRecordWithTileBase
    {
    public:
        TrafficRefRecord(FeatureWithIDProxyBase* elem) : RefRecordWithTileBase(elem) {};

        virtual const int* contain_refs_offsets() const {
            static int _offs[] = { offsetof(TrafficInfoProxy, _objs), -1 };
            return _offs;
        };
        virtual const int* contain_ref_offsets() const {
            static int _offs[] = { offsetof(TrafficInfoProxy, _lanes),
                                   offsetof(TrafficInfoProxy, _lanegroups), -1 };
            return _offs;
        };
        /*virtual const int* contain_sub_ref_offs() const {
            static int _offs[] = { offsetof(TrafficInfoProxy, _light_group), -1 };
            return _offs;
        };*/
    };
};

}; // data_access_engine
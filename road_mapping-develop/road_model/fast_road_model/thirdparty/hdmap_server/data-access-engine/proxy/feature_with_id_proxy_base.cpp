#include "feature_with_id_proxy_base.h"
#include "feature_id_proxy.h"
#include "tile_proxy.h"
#include "manager/road_geometry_mgr.h"

namespace data_access_engine {

std::mutex RefRecordBase::_init_mutex;

const FeatureWithIDProxyBase* RefRecordBase::element() const {
    if (!_elem) {
        return nullptr;
    }
    return _elem->container();
};

void RefRecordBase::reset_container(std::shared_ptr<RefRecordBase> p) {
    auto mems = contain_members();
    for (auto m : mems) {
        auto mm = const_cast<FeatureMemberBase*>(m);
        mm->reset_container(p);
    }
    auto refs = contain_refs();
    for (auto r : refs) {
        auto mr = const_cast<FeatureReferenceBase*>(r);
        mr->reset_container(p);
    }
};

void RefRecordBase::release_elem() {
    if (_elem == nullptr) {
        LOG_BEGIN(WARNING) << "RefRecord::release_elem(): elem is already released!"; LOG_END;
        return;
    }
    auto elem = _elem;
    _elem = nullptr;
    {
        std::vector<const FeatureMemberBase*> owners(_owner_elems.begin(), _owner_elems.end());
        for (auto o : owners) {
            auto owner = const_cast<FeatureMemberBase*>(o);
            if (owner == nullptr) {
                LOG_BEGIN(WARNING) << "RefRecord::release_elem(): found null in owners"; LOG_END;
                continue;
            }
            if (owner->id()->container() == elem) {
                owner->reset(false);
            }
            else {
                LOG_BEGIN(WARNING) << "RefRecord::release_elem(): found inconsist owner: "
                    << elem->container()->id().get(); LOG_END;
            }
        }
    }
    {
        std::vector<const FeatureReferenceBase*> refs(_reference_elems.begin(), _reference_elems.end());
        for (auto r : refs) {
            auto ref = const_cast<FeatureReferenceBase*>(r);
            if (ref == nullptr) {
                LOG_BEGIN(WARNING) << "RefRecord::release_elem(): found null in refs"; LOG_END;
                continue;
            }
            if (ref->id()->container() == elem) {
                ref->reset(false);
            }
            else {
                LOG_BEGIN(WARNING) << "RefRecord::release_elem(): found inconsist ref: "
                    << elem->container()->id().get(); LOG_END;
            }
        }
    }    
};

bool RefRecordBase::add_owner(const FeatureMemberBase* owner) {
    if (owner == nullptr) {
        return false;
    }
    auto it = std::find(_owner_elems.begin(), _owner_elems.end(), owner);
    if (it != _owner_elems.end()) {
        return false;
    }
    _owner_elems.push_back(owner);
    return true;
};

bool RefRecordBase::release_owner(const FeatureMemberBase* owner) {
    auto it = std::find(_owner_elems.begin(), _owner_elems.end(), owner);
    if (it != _owner_elems.end()) {
        _owner_elems.erase(it);
        return true;
    }
    return false;
};

bool RefRecordBase::add_reference(const FeatureReferenceBase* ref) {
    if (ref == nullptr) {
        return false;
    }
    auto it = std::find(_reference_elems.begin(), _reference_elems.end(), ref);
    if (it != _reference_elems.end()) {
        return false;
    }
    _reference_elems.push_back(ref);
    return true;
};

bool RefRecordBase::release_reference(const FeatureReferenceBase* ref) {
    auto it = std::find(_reference_elems.begin(), _reference_elems.end(), ref);
    if (it != _reference_elems.end()) {
        _reference_elems.erase(it);
        return true;
    }
    return false;
};

std::vector<const FeatureMemberBase*> RefRecordBase::contain_members() const {
    if (_elem == nullptr) {
        return std::vector<const FeatureMemberBase*>();
    }
    const int* members_offs = contain_members_offsets();
    const int* member_offs = contain_member_offsets();
    std::vector<const FeatureMemberBase*> members;
    for (int i = 0; members_offs[i] >= 0; ++i) {
        auto& ems = *((FeatureMemberVector<FeatureWithIDProxyBase>*)((uint8_t*)_elem + members_offs[i]));
        members.reserve((ems.size() + members.size()) * 2);
        for (size_t j = 0; j < ems.size(); ++j) {
            if (ems[j]) {
                members.push_back(&ems[j]);
            }
        }
    }
    for (int i = 0; member_offs[i] >= 0; ++i) {
        auto e = ((FeatureMemberBase*)((uint8_t*)_elem + member_offs[i]));
        if (e) {
            members.push_back(e);
        }
    }
    const int* subs_offs = contain_sub_members_offs();
    const int* sub_offs = contain_sub_member_offs();
    for (int i = 0; subs_offs[i] >= 0; ++i) {
        auto& ems = *((SharedProxyVector<FeatureWithRefProxyBase>*)((uint8_t*)_elem + subs_offs[i]));
        members.reserve((ems.size() + members.size()) * 2);
        for (size_t j = 0; j < ems.size(); ++j) {
            if (ems[j]) {
                auto vs = ems[j]->contained_members();
                members.insert(members.end(), vs.begin(), vs.end());
            }
        }
    }
    for (int i = 0; sub_offs[i] >= 0; ++i) {
        auto e = *((std::shared_ptr<FeatureWithRefProxyBase>*)((uint8_t*)_elem + sub_offs[i]));
        if (e) {
            auto vs = e->contained_members();
            members.insert(members.end(), vs.begin(), vs.end());
        }
    }
    return members;
};

std::vector<const FeatureReferenceBase*> RefRecordBase::contain_refs() const {
    if (_elem == nullptr) {
        return std::vector<const FeatureReferenceBase*>();
    }
    const int* refs_offs = contain_refs_offsets();
    const int* ref_offs = contain_ref_offsets();
    std::vector<const FeatureReferenceBase*> refs;
    for (int i = 0; refs_offs[i] >= 0; ++i) {
        auto& ers = *((FeatureReferenceVector<FeatureWithIDProxyBase>*)((uint8_t*)_elem + refs_offs[i]));
        refs.reserve((refs.size() + ers.size()) * 2);
        for (size_t j = 0; j < ers.size(); ++j) {
            if (ers[j]) {
                refs.push_back(&ers[j]);
            }
        }
    }
    for (int i = 0; ref_offs[i] >= 0; ++i) {
        auto e = ((FeatureReferenceBase*)((uint8_t*)_elem + ref_offs[i]));
        if (e) {
            refs.push_back(e);
        }
    }
    const int* subs_offs = contain_sub_refs_offs();
    const int* sub_offs = contain_sub_ref_offs();
    for (int i = 0; subs_offs[i] >= 0; ++i) {
        auto& ems = *((SharedProxyVector<FeatureWithRefProxyBase>*)((uint8_t*)_elem + subs_offs[i]));
        refs.reserve((ems.size() + refs.size()) * 2);
        for (size_t j = 0; j < ems.size(); ++j) {
            if (ems[j]) {
                auto vs = ems[j]->contained_references();
                refs.insert(refs.end(), vs.begin(), vs.end());
            }
        }
    }
    for (int i = 0; sub_offs[i] >= 0; ++i) {
        auto e = *((std::shared_ptr<FeatureWithRefProxyBase>*)((uint8_t*)_elem + sub_offs[i]));
        if (e) {
            auto vs = e->contained_references();
            refs.insert(refs.end(), vs.begin(), vs.end());
        }
    }
    return refs;
};

std::vector<const TileInfoProxy*> RefRecordWithTileBase::referenced_tiles() const {
    std::vector<const TileInfoProxy*> ts;
    ts.reserve(_referenced_tiles.size());
    for (auto& tit : _referenced_tiles) {
        if (tit.first != nullptr && tit.second > 0) {
            ts.push_back(tit.first);
        }
    }
    return ts;
};

bool RefRecordWithTileBase::add_missed_ref_tile(const TileInfoProxy* tile) {
    if (tile == nullptr) {
        return false;
    }
    auto tit = std::find_if(_referenced_tiles.begin(), _referenced_tiles.end(),
        [tile](std::pair<const TileInfoProxy*, int>& it) { return it.first == tile; });
    if (tit != _referenced_tiles.end()) {
        return false;
    }
    _referenced_tiles.push_back(std::make_pair(tile, 1));
    return true;
};
bool RefRecordWithTileBase::add_referenced_tile(const TileInfoProxy* tile) {
    if (tile == nullptr) {
        return false;
    }
    auto tit = std::find_if(_referenced_tiles.begin(), _referenced_tiles.end(),
        [tile](std::pair<const TileInfoProxy*, int>& it) { return it.first == tile; });
    if (tit != _referenced_tiles.end()) {
        ++tit->second;
        return false;
    }
    _referenced_tiles.push_back(std::make_pair(tile, 1));
    return true;
};

bool RefRecordWithTileBase::release_referenced_tile(const TileInfoProxy* tile) {
    auto tit = std::find_if(_referenced_tiles.begin(), _referenced_tiles.end(),
        [tile](std::pair<const TileInfoProxy*, int>& it) { return it.first == tile; });
    if (tit == _referenced_tiles.end() || tit->second <= 0) {
        return false;
    }
    --tit->second;
    if (tit->second <= 0) {
        _referenced_tiles.erase(tit);
        return true;
    }
    return false;
};

FeatureWithIDProxyBase::~FeatureWithIDProxyBase() {
    release_ref_record();
    if (_id) {
        _id->set_owner(nullptr);
    }
    if (_change_rec != nullptr) {
        if (_change_rec->_deleted_id) {
            _change_rec->_deleted_id->set_owner(nullptr);
        }
        if (_change_rec->_changed_id) {
            _change_rec->_changed_id->set_owner(nullptr);
        }
        delete _change_rec;
        _change_rec = nullptr;
    }
};

size_t FeatureWithRefProxyBase::owner_count() const {
    std::unordered_set<const FeatureWithIDProxyBase*> owners(7);
    if (!_ref_record) {
        return 0;
    }
    for (auto r : _ref_record->owner_elems()) {
        if (r != nullptr && r->container() != nullptr) {
            owners.insert(r->container());
        }
    }
    return owners.size();
};

std::unordered_set<const FeatureWithIDProxyBase*> FeatureWithRefProxyBase::owner_elems() {
    if (!_ref_record) {
        return std::unordered_set<const FeatureWithIDProxyBase*>();
    }
    std::unordered_set<const FeatureWithIDProxyBase*> owners(_ref_record->_owner_elems.size() * 2);
    for (auto r : _ref_record->owner_elems()) {
        if (r != nullptr && r->container() != nullptr) {
            owners.insert(r->container());
        }
    }
    return owners;
};

std::unordered_set<const FeatureWithIDProxyBase*> FeatureWithRefProxyBase::referenced_elems() {
    if (!_ref_record) {
        return std::unordered_set<const FeatureWithIDProxyBase*>();
    }
    std::unordered_set<const FeatureWithIDProxyBase*> refs(_ref_record->reference_elems().size() * 2);
    for (auto r : _ref_record->reference_elems()) {
        if (r != nullptr && r->container() != nullptr) {
            refs.insert(r->container());
        }
    }
    return refs;
};

std::vector<const TileInfoProxy*> FeatureWithRefProxyBase::referenced_tiles() const {
    if (!_ref_record) {
        return std::vector<const TileInfoProxy*>();
    }
    return _ref_record->referenced_tiles();
};

bool FeatureWithRefProxyBase::add_owner_reference(FeatureMemberBase* elem) const {
    if (elem == nullptr || !_ref_record) {
        return false;
    }
    return _ref_record->add_owner(elem);
};
bool FeatureWithRefProxyBase::remove_owner_reference(FeatureMemberBase* elem) const {
    if (!_ref_record) {
        return false;
    }
    return _ref_record->release_owner(elem);
};

bool FeatureWithRefProxyBase::add_weak_reference(FeatureReferenceBase* elem) const {
    if (elem == nullptr || !_ref_record) {
        return false;
    }
    return _ref_record->add_reference(elem);
};
bool FeatureWithRefProxyBase::remove_weak_reference(FeatureReferenceBase* elem) const {
    if (!_ref_record) {
        return false;
    }
    return _ref_record->release_reference(elem);
};

bool FeatureWithIDProxyBase::add_missed_ref_tiles(const TileInfoProxy* tile) const {
    if (!tile /*|| (_id && tile->tile_id() == _id->tileid())*/ || !_ref_record) {
        return false;
    }
    return _ref_record->add_missed_ref_tile(tile);
};
bool FeatureWithIDProxyBase::add_referenced_tiles(const TileInfoProxy* tile) const {
    if (!tile /*|| (_id && tile->tile_id() == _id->tileid())*/ || !_ref_record) {
        return false;
    }
    return _ref_record->add_referenced_tile(tile);
};
bool FeatureWithIDProxyBase::remove_referenced_tiles(const TileInfoProxy* tile) const {
    if (!_ref_record) {
        return false;
    }
    return _ref_record->release_referenced_tile(tile);
};
bool FeatureWithIDProxyBase::add_referenced_tiles(const FeatureProxyBase* p) const {
    if (!p || p->feature_id_type() != -1) {
        return false;
    }
    auto info = static_cast<const TileInfoProxy*>(p);
    return add_referenced_tiles(info);
};
bool FeatureWithIDProxyBase::remove_referenced_tiles(const FeatureProxyBase* p) const {
    if (!p || p->feature_id_type() != -1) {
        return false;
    }
    auto info = static_cast<const TileInfoProxy*>(p);
    return remove_referenced_tiles(info);
};

bool FeatureWithIDProxyBase::correct_version(RoadGeometryManager* mgr) {
    if (_id && _id->version() <= 0) {
        _id->set_version(mgr->current_version());
        LOG_BEGIN(DEBUG) << "correct_version() finish for " << _id->to_string(); LOG_END;
        return true;
    }
    if (!_id || !is_changed()) {
        return false;
    }
    if (_id->version() <= mgr->download_version()) {
        _id->set_version(mgr->current_version());
        LOG_BEGIN(DEBUG) << "correct_version() finish for " << _id->to_string(); LOG_END;
        return true;
    }
    return false;
};

bool FeatureWithIDProxyBase::judge_editable(RoadGeometryManager* mgr)  {
    _editable = true;
    if (_id) {
        _editable = mgr->is_tile_15_downloaded(_id->tileid());
    }
    return _editable;
};

bool FeatureWithIDProxyBase::is_editable() const {
    if (!_ref_record || _ref_record->_owner_elems.empty()) {
        return _editable;
    }
    bool bf = _editable;
    for (auto o : _ref_record->_owner_elems) {
        if (o != nullptr && o->container()) {
            if (o->container()->is_editable()) {
                return true;
            }
            else {
                bf = false;
            }
        }        
    }
    return bf;
};

bool FeatureWithIDProxyBase::is_ref_valid() const {
    if (!_ref_record || !_id || !_id->cached_is_valid() || _id->type() != feature_id_type()) {
        return false;
    }
    const FeatureWithIDProxyBase* cur_owner = nullptr;
    for (auto oe : _ref_record->_owner_elems) {
        if (oe == nullptr) {
            continue;
        }
        auto owner = oe->container();
        if (owner != nullptr) {
            if (cur_owner == nullptr) {
                cur_owner = owner;
            }
            else if (cur_owner != owner) {
                return false;
            }
        }
    }
    return true;
};

bool FeatureWithIDProxyBase::set_deleted(bool no_transpose) {
    if (!is_editable() || !_id) {
        return false;
    }
    if (_id->is_deleted().get_value_or(false)) {
        return true;
    }
    _id->set_is_deleted(true);
    if (!_ref_record || no_transpose) {
        return true;
    }
    auto members = _ref_record->contain_members();
    for (auto w : members) {
        if (w == nullptr || !w->id() || w->id()->owner() == nullptr) {
            continue;
        }
        auto ref_rec = w->id()->owner()->reference_record();
        if (ref_rec == nullptr) {
            continue;
        }
        bool bmore = false;
        std::vector<const FeatureMemberBase*> oes(ref_rec->_owner_elems.begin(),
                                                  ref_rec->_owner_elems.end());
        for (auto oe : oes) {
            if (oe == nullptr) {
                continue;
            }
            auto owner = const_cast<FeatureWithIDProxyBase*>(oe->container());
            if (owner == this) {
                ref_rec->release_owner(oe);
            }
            else {
                bmore = true;
            }
        }
        if (!bmore) {
            auto p = const_cast<FeatureWithIDProxyBase*>(w->id()->owner());
            p->set_deleted();
        }        
    }
    return true;
};

bool FeatureWithIDProxyBase::is_deleted() const {
    return _id && _id->is_deleted().get_value_or(false);
};

std::shared_ptr<FeatureIDProxy> FeatureWithIDProxyBase::mutable_id() {
    if (!is_editable()) {
        return nullptr;
    }
    if (!_id) {
        _id.reset(FeatureProxyBase::create_proxy<FeatureIDProxy>(this));
        _id->set_owner(this);
    }
    return _id;
};
bool FeatureWithIDProxyBase::clear_id() {
    if (!is_editable()) {
        return false;
    }
    if (_id) {
        mark_changed();
    }
    _id.reset();
    return true;
};

bool FeatureWithIDProxyBase::to_issue(RoadPB::Issue* issue, const RoadGeometryManager* mgr) const {
    if (_change_rec == nullptr || issue == nullptr || !_id) {
        return false;
    }
    _change_rec->_conflict_issue = issue;
    issue->mutable_id()->set_tileid(_id->tileid());
    issue->mutable_id()->set_id(_id->id());
    issue->mutable_id()->set_version(mgr->current_version());
    issue->mutable_id()->set_type(_id->type());
    Vector3D pos = wgs84_pos();
    auto pt = issue->mutable_pt();
    pt->set_x(pos.X());
    pt->set_y(pos.Y());
    pt->set_z(pos.Z());
    issue->set_type(RoadPB::ISSUE_DADIAN_POINT);
    issue->set_manual(false);
    issue->set_state(RoadPB::STATE_NONE);
    issue->set_zy_state(RoadPB::STATE_NONE);
    issue->set_geotype(RoadPB::GEO_POINT);
    auto p = issue->add_error_points();
    p->mutable_position()->set_x(pos.X());
    p->mutable_position()->set_y(pos.Y());
    p->mutable_position()->set_z(pos.Z());
    p->set_min_angle(0);
    p->set_min_distance(0);
    return true;
};

}; // data_access_engine

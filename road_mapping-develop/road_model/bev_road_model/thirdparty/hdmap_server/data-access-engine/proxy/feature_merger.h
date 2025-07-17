#pragma once
#include <stdint.h>
#include "feature_proxy_base.h"
#include "feature_with_id_proxy_base.h"
#include "manager/road_geometry_mgr.h"

namespace data_access_engine {

//用于合并路网元素的封装内，统一封装各种合并方法，简化编码流程，并记录合并结果
template <class FeatType>
class ProxyMerger
{
public:
    ProxyMerger() : _changed(false), _conflicted(false), _finished(false), _mgr(nullptr) {};
    ProxyMerger(MergeConflictPolicy p, RoadGeometryManager* mgr, FeatType* elem, const char* name,
                    std::shared_ptr<const FeatType> r, std::shared_ptr<const FeatType> o) {
        _policy = p;
        _mgr = mgr;
        _elem = elem;
        _remote = r;
        _origin = o;        
        _changed = true;
        _conflicted = false;
        _finished = false;
    };

    virtual void start_merge() {};

    template<typename T>
    ProxyMerger& merge(T& local, const char* name) {
        if (_finished) {
            return (*this);
        }
        ptrdiff_t offset = (uint8_t*)&local - (uint8_t*)_elem;
        const T& base = *((T*)((uint8_t*)_origin.get() + offset));
        const T& remote = *((T*)((uint8_t*)_remote.get() + offset));
        if (local == remote) {
            return (*this);
        }
        if (_policy == MERGE_COMPARE_ONLY) {
            _conflicted = true;
            _finished = true;
            _stream << name << " changed, ";
            return (*this);
        }
        /*LOG_BEGIN(INFO) << __FUNCTION__ << " for elem " << _elem->id()->to_string()
            << " with " << _remote->id()->to_string(); 
        LOG_END;*/
        start_merge();
        if (_elem->three_way_merge(_policy, base, remote, local)) {
            _stream << name << " conflict, ";
            _conflicted = true;
        }
        return (*this);
    };

    template<typename T>
    ProxyMerger& merge(std::shared_ptr<T>& local, const char* name) {
        if (_finished) {
            return (*this);
        }
        ptrdiff_t offset = (uint8_t*)&local - (uint8_t*)_elem;
        const std::shared_ptr<T>& remote = *((std::shared_ptr<T>*)((uint8_t*)_remote.get() + offset));
        if (local == remote || (local && remote && (*local) == (*remote))) {
            return (*this);
        }
        if (_policy == MERGE_COMPARE_ONLY) {
            _conflicted = true;
            _finished = true;
            _stream << name << " changed, ";
            return (*this);
        }
        /*LOG_BEGIN(INFO) << __FUNCTION__ << " for elem " << _elem->id()->to_string()
            << " with " << _remote->id()->to_string();
        LOG_END;*/
        start_merge();
        const std::shared_ptr<T>& base = *((std::shared_ptr<T>*)((uint8_t*)_origin.get() + offset));
        if (three_way_merge_proxy<T>(_policy, _mgr, base, remote, local)) {
            _stream << name << " conflict, ";
            _conflicted = true;
        }
        return (*this);
    };

    template<typename T>
    ProxyMerger& merge(std::vector<T>& local, const char* name) {
        if (_finished) {
            return (*this);
        }
        ptrdiff_t offset = (uint8_t*)&local - (uint8_t*)_elem;
        const std::vector<T>& base = *((std::vector<T>*)((uint8_t*)_origin.get() + offset));
        const std::vector<T>& remote = *((std::vector<T>*)((uint8_t*)_remote.get() + offset));
        if (vector_equal(local, remote)) {
            return (*this);
        }
        if (_policy == MERGE_COMPARE_ONLY) {
            _conflicted = true;
            _finished = true;
            _stream << name << " changed, ";
            return (*this);
        }
        /*LOG_BEGIN(INFO) << __FUNCTION__ << " for elem " << _elem->id()->to_string()
            << " with " << _remote->id()->to_string();
        LOG_END;*/
        start_merge();
        if (_elem->three_way_merge_vector(_policy, base, remote, local))  {
            _stream << name << " conflict, ";
            _conflicted = true;
        }
        return (*this);
    };
    /*template<typename T>
    FeatureMerger& merge_unordered(std::vector<T>& local, const char* name) {
        return merge(local, name);
    };*/

    template<typename T>
    ProxyMerger& merge(SharedProxyVector<T>& local, const char* name) {
        if (_finished) {
            return (*this);
        }
        ptrdiff_t offset = (uint8_t*)&local - (uint8_t*)_elem;
        const SharedProxyVector<T>& base = *((SharedProxyVector<T>*)((uint8_t*)_origin.get() + offset));
        const SharedProxyVector<T>& remote = *((SharedProxyVector<T>*)((uint8_t*)_remote.get() + offset));
        if (proxys_equal(local, remote)) {
            return (*this);
        }
        if (_policy == MERGE_COMPARE_ONLY) {
            _conflicted = true;
            _finished = true;
            _stream << name << " changed, ";
            return (*this);
        }
        /*LOG_BEGIN(INFO) << __FUNCTION__ << " for elem " << _elem->id()->to_string()
            << " with " << _remote->id()->to_string();
        LOG_END;*/
        start_merge();
        if (three_way_merge_proxys(_policy, _mgr, base, remote, local))  {
            _stream << name << " conflict, ";
            _conflicted = true;
        }
        return (*this);
    };
    template<typename T>
    ProxyMerger& merge_elemental(SharedProxyVector<T>& local, const char* name) {
        if (_finished) {
            return (*this);
        }
        
        ptrdiff_t offset = (uint8_t*)&local - (uint8_t*)_elem;
        SharedProxyVector<T> local_sorted;
        local_sorted.sorted_copy(local);
        SharedProxyVector<T>& base = *((SharedProxyVector<T>*)((uint8_t*)_origin.get() + offset));
        SharedProxyVector<T> base_sorted;
        base_sorted.sorted_copy(base);
        SharedProxyVector<T>& remote = *((SharedProxyVector<T>*)((uint8_t*)_remote.get() + offset));
        SharedProxyVector<T> remote_sorted;
        remote_sorted.sorted_copy(remote);
        if (proxys_equal(local_sorted, remote_sorted)) {
            return (*this);
        }
        if (_policy == MERGE_COMPARE_ONLY) {
            _conflicted = true;
            _finished = true;
            _stream << name << " changed, ";
            return (*this);
        }
        /*LOG_BEGIN(INFO) << __FUNCTION__ << " for elem " << _elem->id()->to_string()
            << " with " << _remote->id()->to_string();
        LOG_END;*/
        start_merge();
        if (three_way_merge_elemental(_policy, base_sorted, remote_sorted, local_sorted, local))  {
            _stream << name << " conflict, ";
            _conflicted = true;
        }
        return (*this);
    };

    template<typename T>
    ProxyMerger& merge(FeatureMember<T>& local, const char* name) {
        if (_finished) {
            return (*this);
        }
        ptrdiff_t offset = (uint8_t*)&local - (uint8_t*)_elem;
        const FeatureMember<T>& base = *((FeatureMember<T>*)((uint8_t*)_origin.get() + offset));
        const FeatureMember<T>& remote = *((FeatureMember<T>*)((uint8_t*)_remote.get() + offset));
        if (local == remote) {
            return (*this);
        }
        if (_policy == MERGE_COMPARE_ONLY) {
            _conflicted = true;
            _finished = true;
            _stream << name << " changed, ";
            return (*this);
        }        
        /*LOG_BEGIN(INFO) << __FUNCTION__ << " for elem " << _elem->id()->to_string()
            << " with " << _remote->id()->to_string(); 
        LOG_END;*/
        start_merge();
        if (_elem->three_way_merge(_policy, base, remote, local)) {
            _stream << name << " conflict, ";
            _conflicted = true;
        }
        return (*this);
    };

    template<typename T>
    ProxyMerger& merge(FeatureMemberVector<T>& local, const char* name) {
        if (_finished) {
            return (*this);
        }
        ptrdiff_t offset = (uint8_t*)&local - (uint8_t*)_elem;
        const FeatureMemberVector<T>& base = *((FeatureMemberVector<T>*)((uint8_t*)_origin.get() + offset));
        const FeatureMemberVector<T>& remote = *((FeatureMemberVector<T>*)((uint8_t*)_remote.get() + offset));
        if (members_equal(local, remote)) {
            return (*this);
        }
        if (_policy == MERGE_COMPARE_ONLY) {
            _conflicted = true;
            _finished = true;
            _stream << name << " changed, ";
            return (*this);
        }
        /*LOG_BEGIN(INFO) << __FUNCTION__ << " for elem " << _elem->id()->to_string()
            << " with " << _remote->id()->to_string();
        LOG_END;*/
        start_merge();
        if (three_way_merge_members(_policy, base, remote, local)) {
            _stream << name << " conflict, ";
            _conflicted = true;
        }
        return (*this);
    };
    
    template<typename T>
    ProxyMerger& merge(FeatureReference<T>& local, const char* name, bool skip_id = true) {
        if (_finished) {
            return (*this);
        }
        ptrdiff_t offset = (uint8_t*)&local - (uint8_t*)_elem;
        const FeatureReference<T>& base = *((FeatureReference<T>*)((uint8_t*)_origin.get() + offset));
        const FeatureReference<T>& remote = *((FeatureReference<T>*)((uint8_t*)_remote.get() + offset));
        if (skip_id && _policy == MERGE_COMPARE_ONLY) {
            if ((local && !remote) || (!local && remote)) {
                _conflicted = true;
                _finished = true;
                _stream << name << " changed, ";
            }
            return (*this);
        }
        if (local == remote) {
            return (*this);
        }
        if (_policy == MERGE_COMPARE_ONLY) {
            _conflicted = true;
            _finished = true;
            _stream << name << " changed, ";
            return (*this);
        }
        /*LOG_BEGIN(INFO) << __FUNCTION__ << " for elem " << _elem->id()->to_string()
            << " with " << _remote->id()->to_string(); 
        LOG_END;*/
        start_merge();
        if (_elem->three_way_merge(_policy, base, remote, local)) {
            _stream << name << " conflict, ";
            _conflicted = true;
        }
        return (*this);
    };

    template<typename T>
    ProxyMerger& merge(FeatureReferenceVector<T>& local, const char* name, bool skip_id = true) {
        if (_finished) {
            return (*this);
        }
        ptrdiff_t offset = (uint8_t*)&local - (uint8_t*)_elem;
        const FeatureReferenceVector<T>& base = *((FeatureReferenceVector<T>*)((uint8_t*)_origin.get() + offset));
        const FeatureReferenceVector<T>& remote = *((FeatureReferenceVector<T>*)((uint8_t*)_remote.get() + offset));
        if (skip_id && _policy == MERGE_COMPARE_ONLY) {
            if (base.size() != remote.size()) {
                _conflicted = true;
                _finished = true;
                _stream << name << " changed, ";
            }
            return (*this);
        }
        if (refs_equal(local, remote)) {
            return (*this);
        }
        if (_policy == MERGE_COMPARE_ONLY) {
            _conflicted = true;
            _finished = true;
            _stream << name << " changed, ";
            return (*this);
        }
        /*LOG_BEGIN(INFO) << __FUNCTION__ << " for elem " << _elem->id()->to_string()
            << " with " << _remote->id()->to_string(); 
        LOG_END;*/
        start_merge();
        if (three_way_merge_refs(_policy, base, remote, local))  {
            _stream << name << " conflict, ";
            _conflicted = true;
        }
        return (*this);
    };
    template<typename T>
    ProxyMerger& merge_unordered(FeatureMemberVector<T>& local, const char* name) {
        if (_finished) {
            return (*this);
        }
        ptrdiff_t offset = (uint8_t*)&local - (uint8_t*)_elem;
        FeatureMemberVector<T> local_sorted(nullptr);
        local_sorted.sorted_copy(local);
        FeatureMemberVector<T>& base = *((FeatureMemberVector<T>*)((uint8_t*)_origin.get() + offset));
        FeatureMemberVector<T> base_sorted(nullptr);
        base_sorted.sorted_copy(base);
        FeatureMemberVector<T>& remote = *((FeatureMemberVector<T>*)((uint8_t*)_remote.get() + offset));
        FeatureMemberVector<T> remote_sorted(nullptr);
        remote_sorted.sorted_copy(remote);
        if (members_equal(local_sorted, remote_sorted)) {
            return (*this);
        }
        if (_policy == MERGE_COMPARE_ONLY) {
            _conflicted = true;
            _finished = true;
            _stream << name << " changed, ";
            return (*this);
        }
        start_merge();
        if (three_way_merge_members(_policy, base_sorted, remote_sorted, local_sorted)) {
            _stream << name << " conflict, ";
            _conflicted = true;
        }
        FeatureMemberVector<T> ls(nullptr);
        ls.sorted_copy(local);
        if (!members_equal(ls, local_sorted)) {
            local.clear();
            for (auto& e : local_sorted) {
                local.push_back(e);
            }
        }
        return (*this);
    };

    template<typename T>
    ProxyMerger& merge_unordered(FeatureReferenceVector<T>& local, const char* name, bool skip_id = true) {
        if (_finished) {
            return (*this);
        }
        ptrdiff_t offset = (uint8_t*)&local - (uint8_t*)_elem;
        FeatureReferenceVector<T> local_sorted(nullptr);
        local_sorted.sorted_copy(local);
        FeatureReferenceVector<T>& base = *((FeatureReferenceVector<T>*)((uint8_t*)_origin.get() + offset));
        FeatureReferenceVector<T> base_sorted(nullptr);
        base_sorted.sorted_copy(base);
        FeatureReferenceVector<T>& remote = *((FeatureReferenceVector<T>*)((uint8_t*)_remote.get() + offset));
        FeatureReferenceVector<T> remote_sorted(nullptr);
        remote_sorted.sorted_copy(remote);
        if (skip_id && _policy == MERGE_COMPARE_ONLY) {
            if (base.size() != remote.size()) {
                _conflicted = true;
                _finished = true;
                _stream << name << " changed, ";
            }
            return (*this);
        }
        if (refs_equal(local_sorted, remote_sorted)) {
            return (*this);
        }
        if (_policy == MERGE_COMPARE_ONLY) {
            _conflicted = true;
            _finished = true;
            _stream << name << " changed, ";
            return (*this);
        }
        start_merge();
        if (three_way_merge_unordered(_policy, base_sorted, remote_sorted, local_sorted, local))  {
            _stream << name << " conflict, ";
            _conflicted = true;
        }
        return (*this);
    };

    bool is_changed() const { return _changed; };
    bool is_conflict() const { return _conflicted; };
    bool is_finished() const { return _finished; };

protected:
    bool _changed;
    bool _conflicted;
    bool _finished;
    MergeConflictPolicy _policy;
    RoadGeometryManager* _mgr;
    FeatType* _elem;
    std::shared_ptr<const FeatType> _origin;
    std::shared_ptr<const FeatType> _remote;
    std::stringstream _stream;
};

template<class FeatType>
class FeatureMerger : public ProxyMerger<FeatType>
{
public:
    FeatureMerger(MergeConflictPolicy p, RoadGeometryManager* mgr, FeatType* elem, const char* name) : ProxyMerger<FeatType>() {
        ProxyMerger<FeatType>::_policy = p;
        ProxyMerger<FeatType>::_mgr = mgr;
        ProxyMerger<FeatType>::_elem = elem;
        ProxyMerger<FeatType>::_changed = false;
        ProxyMerger<FeatType>::_conflicted = false;
        ProxyMerger<FeatType>::_finished = false;
        if (!elem || !elem->_change_rec || !elem->_change_rec->_remote_elem) {
            if (elem && p == MERGE_ALL_REMOTE) {
                ProxyMerger<FeatType>::_changed = true;
                end_merge();
                return;
            }
            ProxyMerger<FeatType>::_finished = true;
        }
        else {
            ProxyMerger<FeatType>::_remote = elem->template remote_elem<FeatType>();
            ProxyMerger<FeatType>::_origin = elem->template origin_elem<FeatType>();
            if (ProxyMerger<FeatType>::_policy != MERGE_COMPARE_ONLY) {
                ProxyMerger<FeatType>::_stream << name << ' ' << elem->id()->to_string() << " conflict: ";
            }
            else {
                ProxyMerger<FeatType>::_stream << name << ' ' << elem->id()->to_string() << " changed: ";
                return;
            }

            if (ProxyMerger<FeatType>::_origin && ProxyMerger<FeatType>::_origin->id()->version() == ProxyMerger<FeatType>::_remote->id()->version()) {
                ProxyMerger<FeatType>::_finished = true;
            }
            else if (ProxyMerger<FeatType>::_elem->is_deleted() != ProxyMerger<FeatType>::_remote->is_deleted()) {
                LOG_BEGIN(INFO) << __FUNCTION__ << "() for deleting elem " << elem->id()->to_string()
                    << " with " << ProxyMerger<FeatType>::_remote->id()->to_string();
                LOG_END;
                start_merge();
                if (ProxyMerger<FeatType>::_policy == MERGE_CONFLICT_REMOTE && ProxyMerger<FeatType>::_policy == MERGE_ALL_REMOTE) {
                    ProxyMerger<FeatType>::_elem->mutable_id()->set_value(ProxyMerger<FeatType>::_remote->id()->tileid(),
                        ProxyMerger<FeatType>::_remote->id()->type(), ProxyMerger<FeatType>::_remote->id()->id(), 
                        ProxyMerger<FeatType>::_remote->id()->version());
                }
                if (ProxyMerger<FeatType>::_policy != MERGE_ALL_REMOTE && ProxyMerger<FeatType>::_policy != MERGE_ALL_LOCAL) {
                    ProxyMerger<FeatType>::_conflicted = true;
                    ProxyMerger<FeatType>::_stream << " local is " << (ProxyMerger<FeatType>::_elem->is_deleted() ? " deleted " : " not deleted ")
                        << " while remote is " << (ProxyMerger<FeatType>::_remote->is_deleted() ? " deleted" : " not deleted");
                }
                end_merge();
            }
            else if (ProxyMerger<FeatType>::_elem->is_deleted() && ProxyMerger<FeatType>::_elem->id()->version() != ProxyMerger<FeatType>::_remote->id()->version()) {
                LOG_BEGIN(INFO) << __FUNCTION__ << "() for deleted elem " << elem->id()->to_string()
                    << " with " << ProxyMerger<FeatType>::_remote->id()->to_string();
                LOG_END;
                start_merge();
                if (ProxyMerger<FeatType>::_policy != MERGE_ALL_REMOTE && ProxyMerger<FeatType>::_policy != MERGE_ALL_LOCAL) {
                    ProxyMerger<FeatType>::_conflicted = true;
                    ProxyMerger<FeatType>::_stream << " local is deleted while remote is deleted @ " << ProxyMerger<FeatType>::_remote->id()->version();
                }
                end_merge();
            }
        }
    };
    ~FeatureMerger() {
        end_merge();
    };

    virtual void start_merge() {
        if (ProxyMerger<FeatType>::_finished || ProxyMerger<FeatType>::_changed) {
            return;
        }
        ProxyMerger<FeatType>::_changed = true;
        LOG_BEGIN(DEBUG) << __FUNCTION__ << "() for elem " << ProxyMerger<FeatType>::_elem->id()->to_string()
            << " with " << ProxyMerger<FeatType>::_remote->id()->to_string();
        LOG_END;
        ProxyMerger<FeatType>::_elem->mutable_id()->set_version(std::max(ProxyMerger<FeatType>::_elem->id()->version(),
                                                                     ProxyMerger<FeatType>::_remote->id()->version()));
                
        if (ProxyMerger<FeatType>::_elem->_change_rec == nullptr || !ProxyMerger<FeatType>::_elem->_change_rec->_origin_elem) {
            LOG_BEGIN(DEBUG) << __FUNCTION__ << "() for unchanged elem " << ProxyMerger<FeatType>::_elem->id()->to_string()
                << " with " << ProxyMerger<FeatType>::_remote->id()->to_string();
            LOG_END;
            ProxyMerger<FeatType>::_elem->template copy_on_write<FeatType>();
            ProxyMerger<FeatType>::_origin = ProxyMerger<FeatType>::_elem->template origin_elem<FeatType>();
            ProxyMerger<FeatType>::_elem->template make_local_elem<FeatType>();
            typename FeatType::message_type msg;
            if (ProxyMerger<FeatType>::_remote->to_message(&msg)) {
                if (ProxyMerger<FeatType>::_elem->from_message(&msg)) {
                    ProxyMerger<FeatType>::_elem->remake_proxy(ProxyMerger<FeatType>::_mgr);
                }
            }
            end_merge();
            return;
        }
        ProxyMerger<FeatType>::_elem->template make_local_elem<FeatType>();
        if (ProxyMerger<FeatType>::_policy == MERGE_ALL_LOCAL) {
            LOG_BEGIN(DEBUG) << __FUNCTION__ << "() use local for elem " << ProxyMerger<FeatType>::_elem->id()->to_string()
                << " with " << ProxyMerger<FeatType>::_remote->id()->to_string();
            LOG_END;
            end_merge();
            return;
        }
        if (ProxyMerger<FeatType>::_policy == MERGE_ALL_REMOTE) {
            LOG_BEGIN(DEBUG) << __FUNCTION__ << "() use remote for elem " << ProxyMerger<FeatType>::_elem->id()->to_string()
                << " with " << ProxyMerger<FeatType>::_remote->id()->to_string();
            LOG_END;
            typename FeatType::message_type msg;
            if (ProxyMerger<FeatType>::_remote->to_message(&msg)) {
                if (ProxyMerger<FeatType>::_elem->from_message(&msg)) {
                    ProxyMerger<FeatType>::_elem->remake_proxy(ProxyMerger<FeatType>::_mgr);
                }
            }
            end_merge();
            return;
        }
        if (ProxyMerger<FeatType>::_policy == MERGE_BEST) {
            if (ProxyMerger<FeatType>::_elem->id()->less_version(ProxyMerger<FeatType>::_remote->id())) {
                ProxyMerger<FeatType>::_policy = MERGE_CONFLICT_REMOTE;
                LOG_BEGIN(DEBUG) << __FUNCTION__ << "() prefer remote for elem " << ProxyMerger<FeatType>::_elem->id()->to_string()
                    << " with " << ProxyMerger<FeatType>::_remote->id()->to_string();
                LOG_END;
            }
            else {
                ProxyMerger<FeatType>::_policy = MERGE_CONFLICT_LOCAL;
                LOG_BEGIN(DEBUG) << __FUNCTION__ << "() prefer local for elem " << ProxyMerger<FeatType>::_elem->id()->to_string()
                    << " with " << ProxyMerger<FeatType>::_remote->id()->to_string();
                LOG_END;
            }
        }
    };

    void end_merge() {
        if (ProxyMerger<FeatType>::_finished) {
            return;
        }
        if (ProxyMerger<FeatType>::_changed && ProxyMerger<FeatType>::_mgr != nullptr 
                && ProxyMerger<FeatType>::_elem != nullptr && ProxyMerger<FeatType>::_elem->id()) {
            auto tile = ProxyMerger<FeatType>::_mgr->get_road_tile(ProxyMerger<FeatType>::_elem->id()->tileid());
            auto elem = std::dynamic_pointer_cast<FeatType>(ProxyMerger<FeatType>::_elem->shared_from_this());
            if (tile) {
                if (ProxyMerger<FeatType>::_conflicted) {
                    LOG_BEGIN(INFO) << __FUNCTION__ << "() record conflicted elem "
                        << ProxyMerger<FeatType>::_elem->template local_elem<FeatType>()->id()->to_string()
                        << " with " << ProxyMerger<FeatType>::_remote->id()->to_string() << " : " << ProxyMerger<FeatType>::_stream.str();
                    LOG_END;
                    tile->mutable_conflict_tile()->add_element_to_tile(elem);
                    auto issue = std::make_shared<RoadPB::Issue>();
                    if (elem->to_issue(issue.get(), ProxyMerger<FeatType>::_mgr)) {
                        issue->set_desc(ProxyMerger<FeatType>::_stream.str());
                        ProxyMerger<FeatType>::_mgr->get_merge_issues().push_back(issue);
                    }
                }
                else if (ProxyMerger<FeatType>::_remote) {
                    LOG_BEGIN(INFO) << __FUNCTION__ << "() record changed elem " <<
                        ProxyMerger<FeatType>::_elem->template local_elem<FeatType>()->id()->to_string()
                        << " with " << ProxyMerger<FeatType>::_remote->id()->to_string();
                    LOG_END;
                }
                tile->mutable_changed_tile()->add_element_to_tile(elem);
            }
        }
        ProxyMerger<FeatType>::_finished = true;
    };
};

}; // data_access_engine


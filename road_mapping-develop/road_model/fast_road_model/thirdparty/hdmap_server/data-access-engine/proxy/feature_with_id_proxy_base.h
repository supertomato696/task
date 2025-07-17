#pragma once
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>
#include "feature_proxy_base.h"
#include "feature_id_proxy.h"
#include "public/sha1.hpp"
#include "dynamics/issue.pb.h"
#include "public/vector3.h"

namespace data_access_engine {

class TileInfoProxy;
class RoadGeometryManager;
class FeatureWithRefProxyBase;
class FeatureWithIDProxyBase;
class FeatureMemberBase;
class FeatureReferenceBase;

//记录一个路网元素和所有对它的强/弱引用信息
class RefRecordBase
{
public:
    RefRecordBase(FeatureWithRefProxyBase* elem) : _elem(elem) {};
    virtual ~RefRecordBase() { 
        _elem = nullptr; 
        _owner_elems.clear();
        _reference_elems.clear();
    };

    void reset_container(std::shared_ptr<RefRecordBase> pthis);
    const FeatureWithIDProxyBase* element() const;
    void release_elem();
    virtual bool add_owner(const FeatureMemberBase* owner);
    virtual bool release_owner(const FeatureMemberBase* owner);
    virtual bool add_reference(const FeatureReferenceBase* ref);
    virtual bool release_reference(const FeatureReferenceBase* ref);
    virtual bool add_missed_ref_tile(const TileInfoProxy* tile) { return false; };
    virtual bool add_referenced_tile(const TileInfoProxy* tile) { return false; };
    virtual bool release_referenced_tile(const TileInfoProxy* tile) { return false; };

    const std::vector<const FeatureMemberBase*>& owner_elems() const {
        return _owner_elems;
    };
    const std::vector<const FeatureReferenceBase*>& reference_elems() const {
        return _reference_elems;
    };
    virtual std::vector<const FeatureMemberBase*> contain_members() const;
    virtual std::vector<const FeatureReferenceBase*> contain_refs() const;
    virtual std::vector<const TileInfoProxy*> referenced_tiles() const {
        return std::vector<const TileInfoProxy*>();
    };

    virtual const int* contain_members_offsets() const {
        static int _offs[] = { -1 };
        return _offs;
    };
    virtual const int* contain_member_offsets() const {
        static int _offs[] = { -1 };
        return _offs;
    };
    virtual const int* contain_refs_offsets() const {
        static int _offs[] = { -1 };
        return _offs;
    };
    virtual const int* contain_ref_offsets() const {
        static int _offs[] = { -1 };
        return _offs;
    };
    virtual const int* contain_sub_members_offs() const {
        static int _offs[] = { -1 };
        return _offs;
    };
    virtual const int* contain_sub_member_offs() const {
        static int _offs[] = { -1 };
        return _offs;
    };
    virtual const int* contain_sub_refs_offs() const {
        static int _offs[] = { -1 };
        return _offs;
    };
    virtual const int* contain_sub_ref_offs() const {
        static int _offs[] = { -1 };
        return _offs;
    };

protected:
    friend class FeatureWithRefProxyBase;
    friend class FeatureWithIDProxyBase;
    const FeatureWithRefProxyBase* _elem;
    std::vector<const FeatureMemberBase*> _owner_elems;
    std::vector<const FeatureReferenceBase*> _reference_elems;
    static std::mutex _init_mutex;
};

class RefRecordWithTileBase : public RefRecordBase
{
public:
    RefRecordWithTileBase(FeatureWithRefProxyBase* elem) : RefRecordBase(elem) {};
    virtual ~RefRecordWithTileBase() {
        _referenced_tiles.clear();
    };

    virtual bool add_missed_ref_tile(const TileInfoProxy* tile);
    virtual bool add_referenced_tile(const TileInfoProxy* tile);
    virtual bool release_referenced_tile(const TileInfoProxy* tile);
    virtual std::vector<const TileInfoProxy*> referenced_tiles() const override;

protected:
    std::vector<std::pair<const TileInfoProxy*, int>> _referenced_tiles;
};


class FeatureChangeRecord
{
public:
    std::shared_ptr<FeatureWithIDProxyBase> _origin_elem;
    std::shared_ptr<FeatureWithIDProxyBase> _remote_elem;
    std::shared_ptr<FeatureWithIDProxyBase> _local_elem;
    RoadPB::Issue* _conflict_issue;
    std::shared_ptr<FeatureIDProxy> _deleted_id;
    std::shared_ptr<FeatureIDProxy> _changed_id;

    FeatureChangeRecord() : _conflict_issue(nullptr) {};
};

class FeatureWithRefProxyBase : public FeatureProxyBase
{
public:
    FeatureWithRefProxyBase() : FeatureProxyBase() {};
    FeatureWithRefProxyBase(FeatureProxyBase* p) : FeatureProxyBase(p) {};
    virtual ~FeatureWithRefProxyBase() { release_ref_record(); };
    
    virtual void release_ref_record() {
        std::shared_ptr<RefRecordBase> ref_rec;
        if (_ref_record && _ref_record->_elem == this) {
            _ref_record->release_elem();
            ref_rec.swap(_ref_record);
        }
        else if (_ref_record) {
            LOG_BEGIN(WARNING) << "~FeatureWithIDProxyBase(): _ref_record mismatch with elem "
                << this; LOG_END;
        }
    };

    virtual void make_ref_record() const = 0;
    std::shared_ptr<RefRecordBase> reference_record() const {
        return _ref_record;
    };

    //返回强引用本元素的元素的数目
    size_t owner_count() const;
    //返回只读的强引用本元素的引用关系列表
    const std::vector<const FeatureMemberBase*>& owner_relations() const {
        if (!_ref_record) {
            static std::vector<const FeatureMemberBase*> _dummy;
            return _dummy;
        }
        return _ref_record->_owner_elems;
    };
    //返回只读的弱引用本元素的引用关系列表
    const std::vector<const FeatureReferenceBase*>& reference_relations() const {
        if (!_ref_record) {
            static std::vector<const FeatureReferenceBase*> _dummy;
            return _dummy;
        }
        return _ref_record->_reference_elems;
    };
    //返回只读的本元素的强引用关系列表
    std::vector<const FeatureMemberBase*> contained_members() const {
        if (!_ref_record) {
            return std::vector<const FeatureMemberBase*>();
        }
        return _ref_record->contain_members();
    };
    //返回只读的本元素的弱引用关系列表
    std::vector<const FeatureReferenceBase*> contained_references() const {
        if (!_ref_record) {
            return std::vector<const FeatureReferenceBase*>();
        }
        return _ref_record->contain_refs();
    }

    //返回只读的强引用本元素的元素列表
    std::unordered_set<const FeatureWithIDProxyBase*> owner_elems();
    //返回只读的弱引用本元素的元素列表
    std::unordered_set<const FeatureWithIDProxyBase*> referenced_elems();
    //返回只读的引用本元素的tile列表
    std::vector<const TileInfoProxy*> referenced_tiles() const;

    virtual void set_parent(FeatureProxyBase* parent) override {
        FeatureProxyBase::set_parent(parent);
        if (parent != nullptr && !_ref_record) {
            make_ref_record();
        }
        else if (parent == nullptr && _ref_record) {
            if (_ref_record->_elem == this) {
                _ref_record->release_elem();
            }
            _ref_record.reset();
        }
    };

    //记录对本元素的强引用
    bool add_owner_reference(FeatureMemberBase* elem) const;
    //移除对本元素的强引用
    bool remove_owner_reference(FeatureMemberBase* elem) const;
    //记录对本元素的弱引用
    bool add_weak_reference(FeatureReferenceBase* elem) const;
    //移除被本元素引用的元素
    bool remove_weak_reference(FeatureReferenceBase* elem) const;
    //记录本元素包含的强引用
    bool add_contained_member(FeatureMemberBase* elem) const;
    
protected:
    mutable std::shared_ptr<RefRecordBase> _ref_record;
};

//本类似所有服务器管理的路网元素的基类，每个路网元素都有一个FeatureID，本基类封装了相关的操作
//在Proxy的公共基类基础上，本类型记录了此元素引用的元素和引用此元素的元素/tile，并封装了相关操作
class FeatureWithIDProxyBase : public FeatureWithRefProxyBase
{
public:
    //构造一个没有外层父对象的路网元素基类
    FeatureWithIDProxyBase() : FeatureWithRefProxyBase(), _editable(true), _change_rec(nullptr) {};
    //构造一个指定外层对象的路网元素，此处的外层对象应该是一个TileInfoProxy
    FeatureWithIDProxyBase(FeatureProxyBase* p) : FeatureWithRefProxyBase(p), _editable(true), _change_rec(nullptr) {};
    virtual ~FeatureWithIDProxyBase();
        
    //覆盖基类方法，本对象就是路网元素，不用再追溯上层对象了
    virtual const FeatureWithIDProxyBase* container() const override {
        return this;
    };
    
    bool operator==(const FeatureWithIDProxyBase& rhs) const {
        return _id == rhs._id || (_id && rhs._id && (*_id) == (*rhs._id));
    };
    bool operator<(const FeatureWithIDProxyBase& rhs) const {
        if (!_id && !rhs._id) {
            return false;
        }
        if (_id && rhs._id) {
            return (*_id) < (*rhs._id);
        }
        if (!_id) {
            return true;
        }
        return false;
    };

    //根据本元素的内容产生理论上不重复的64位index
    virtual bool make_id_index() = 0;
    //子类需覆盖此方法，支持copy-on-write，
    //标记本对象变更，传递变更给外层父对象，同时把变更传递给强引用本对象的对象
    virtual void mark_changed() = 0;
    template <class ProxyType> 
    void mark_changed_and_copy();

    //记录引用本元素的tile信息
    bool add_missed_ref_tiles(const TileInfoProxy* tile) const;
    bool add_referenced_tiles(const TileInfoProxy* tile) const;
    bool add_referenced_tiles(const FeatureProxyBase* p) const;
    //移除引用本元素的tile信息    
    bool remove_referenced_tiles(const TileInfoProxy* elem) const;
    bool remove_referenced_tiles(const FeatureProxyBase* p) const;

    template <class ElemType>
    bool revert_base(RoadGeometryManager* mgr = nullptr) {
        if (_change_rec == nullptr || !_change_rec->_origin_elem 
                || _change_rec->_origin_elem.get() == this) {
            return false;
        }
        typename ElemType::message_type msg;
        if (!_change_rec->_origin_elem->to_message(&msg)) {
            return false;
        }
        if (!from_message(&msg)) {
            return false;
        }
        clear_changed();
        remake_proxy(mgr);
        return true;
    };
    
    virtual bool merge(MergeConflictPolicy policy, RoadGeometryManager* mgr) = 0;
    virtual bool revert() = 0;

    //重新判断本兑现是否拥有编辑权限
    virtual bool judge_editable(RoadGeometryManager* mgr);
    //覆盖基类方法，默认实现通过判断强引用本对象的对象是否可编辑
    virtual bool is_editable() const override;
    //覆盖基类方法，对于没有编辑权限的要素直接返回没有变化
    virtual bool is_changed() const override { 
        if (!is_editable()) {
            return false;
        }
        return _change_count > 0; 
    };
    //判断本元素引用的元素信息是否完整
    virtual bool is_completed() const {
        return true;
    };
    //判断本元素是否可被引用、是否被多重引用
    virtual bool is_ref_valid() const;
    //标记删除本元素，同时会把删除标记传递给被它唯一强引用的元素
    bool set_deleted(bool no_transpose = false);
    //判断本元素是否被删除、或者没有合法ID
    bool is_deleted() const;

    //返回本元素ID的只读智能指针
    const std::shared_ptr<const FeatureIDProxy> id() const { return _id; };
    //返回本元素可修改的ID智能指针，如果没有构造id，会自动默认构造一个新的id对象
    std::shared_ptr<FeatureIDProxy> mutable_id();
    //清除已构造的id对象
    bool clear_id();
    //修正id中的version信息，保证version>0，变更元素version>本次下载的时间戳
    bool correct_version(RoadGeometryManager* mgr);
    
    //默认实现的id中index生成算法，把本要素序列化为字符串在计算sha1的hash后截取低64位
    template <class ProtoType>
    bool make_index() {
        ProtoType msg;
        std::string val;
        if (to_message(&msg) && msg.SerializeToString(&val)) {
            boost::uuids::detail::sha1 sha;
            sha.process_bytes(val.c_str(), val.length());
            uint32_t digest[5];
            sha.get_digest(digest);
            _id->set_id(*(uint64_t*)digest);
            return true;
        }
        return false;
    };

    template <class ProxyType>
    void copy_on_write() {
        if (_parent == nullptr || _parent->feature_id_type() != -1
                || !_ref_record || !_id || _id->version() <= 0) {
            return;
        }
        if (_change_rec == nullptr) {
            _change_rec = new FeatureChangeRecord();
        }
        if (!_change_rec->_origin_elem) {
            //LOG_BEGIN(INFO) << "copy_on_write() for Feature " << _id->to_string(); LOG_END;
            _change_rec->_origin_elem = dump<ProxyType>();
            _change_rec->_origin_elem->_ref_record.reset();
        }
    };

    //封装元素中从pb生成id的相关代码
    template <class MsgType>
    bool id_from_message(const MsgType* msg) {
        _change_count = -2;
        if (msg->has_id()) {
            if (!_id) {
                _id.reset(FeatureProxyBase::create_proxy<FeatureIDProxy>(this));
                _id->set_owner(this);
            }
            _id->from_message(&msg->id());
            return true;
        }
        else {
            _id.reset();
        }
        return false;
    };
    //封装元素中把id导出到pb中的相关代码
    template <class MsgType>
    bool id_to_message(MsgType* msg) const {
        if (_id) {
            _id->to_message(msg->mutable_id());
            return true;
        }
        return false;
    };

    //返回本元素在下载时拿到的元素版本，此版本会通过copy-on-write的方式在本元素第一次mark_changed()时生成
    //修改时复制出来的本元素不会解析id引用关系，也不会记录外层对象。如果本对象下载后未修改会直接返回本对象指针
    template <class ElemType>
    const std::shared_ptr<const ElemType> origin_elem() const {
        if (_change_rec != nullptr && _change_rec->_origin_elem) {
            return std::dynamic_pointer_cast<ElemType>(_change_rec->_origin_elem);
        }
        return std::dynamic_pointer_cast<const ElemType>(shared_from_this());
    };
    
    //在进行数据合并时用于返回本对象的服务器修改后的版本，会在上传过程中生成并维护。
    //同样本对象不会解析id引用关系，也不会记录外层引用。如果上传发现服务端本对象没有修改时直接返回本对象指针
    template <class ElemType>
    const std::shared_ptr<const ElemType> remote_elem() const {
        if (_change_rec != nullptr && _change_rec->_remote_elem) {
            return std::dynamic_pointer_cast<ElemType>(_change_rec->_remote_elem);
        }
        return std::dynamic_pointer_cast<const ElemType>(this->shared_from_this());
    };
    template <class ElemType>
    void set_remote_elem(const FeatureWithIDProxyBase* elem) {
        if (_change_rec == nullptr) {
            _change_rec = new FeatureChangeRecord();
        }
        _change_rec->_remote_elem.reset();
        if (!elem || !elem->id() || !elem->id()->is_equal(*id())) {
            return;
        }
        _change_rec->_remote_elem = elem->dump<ElemType>();
        _change_rec->_remote_elem->_ref_record.reset();
    };
    template <class ElemType>
    void set_remote_proxy(std::shared_ptr<ElemType>& elem) {
        if (_change_rec == nullptr) {
            _change_rec = new FeatureChangeRecord();
        }
        _change_rec->_remote_elem.reset();
        if (elem && elem->id()) {
            _change_rec->_remote_elem = elem;
            if (elem->id()->is_equal(*id())) {
                _change_rec->_remote_elem->_ref_record.reset();
            }
        }
    };
    
    //在进行数据合并时用于保存本对象的合并前的本地版本，会在合并过程中生成并维护。
    //同样本对象不会解析id引用关系，也不会记录外层引用。如果不需要合并或合并中使用了本地版本时直接返回本对象指针
    template <class ElemType>
    const std::shared_ptr<const ElemType> local_elem() const {
        if (_change_rec != nullptr && _change_rec->_local_elem) {
            return std::dynamic_pointer_cast<ElemType>(_change_rec->_local_elem);
        }
        return std::dynamic_pointer_cast<const ElemType>(this->shared_from_this());
    };
    template <class ElemType>
    void make_local_elem() {
        if (_change_rec == nullptr) {
            _change_rec = new FeatureChangeRecord();
        }
        if (!_change_rec->_local_elem) {
            _change_rec->_local_elem = dump<ElemType>();
            _change_rec->_local_elem->_ref_record.reset();
        }
    };
    
    virtual void reset_merge() {
        if (_change_rec == nullptr) {
            return;
        }
        _change_rec->_origin_elem = std::dynamic_pointer_cast<FeatureWithIDProxyBase>(shared_from_this());
        _change_rec->_local_elem.reset();
        _change_rec->_remote_elem.reset();
    };

    virtual void confirm_merge() {
        if (_change_rec == nullptr) {
            return;
        }
        if (_change_rec->_origin_elem && _change_rec->_remote_elem && _change_rec->_origin_elem->id()->version() + 1
                == _change_rec->_remote_elem->id()->version()) {
            _change_rec->_origin_elem = _change_rec->_remote_elem;
            _change_rec->_origin_elem->mutable_id()->set_version(id()->version());
            _change_rec->_remote_elem.reset();
        }
        else {
            _change_rec->_origin_elem = _change_rec->_remote_elem;
            _change_rec->_remote_elem.reset();
        }
        _change_rec->_conflict_issue = nullptr;
    };

    virtual const Vector3D& global_pos(const RoadGeometryManager* pMgr = nullptr) const {
        static Vector3D dummy(0, 0, 0);
        return dummy;
    };
    virtual Vector3D wgs84_pos() const {
        return Vector3D(0, 0, 0);
    };    
    virtual bool to_issue(RoadPB::Issue* issue, const RoadGeometryManager* pMgr = nullptr) const;
    bool is_merged() const {
        if (_change_rec == nullptr) {
            return false;
        }
        return !!_change_rec->_local_elem;
    };
    bool is_conflict() const {
        if (_change_rec == nullptr) {
            return false;
        }
        return (_change_rec->_conflict_issue != nullptr);
    };
    
protected:
    friend class FeatureIDProxy;
    template <class ElemType>
    friend class ProxyMerger;
    template <class ElemType>
    friend class FeatureMerger;

    template <class ElemType>
    std::shared_ptr<ElemType> origin_elem() {
        if (_change_rec != nullptr && _change_rec->_origin_elem) {
            return std::dynamic_pointer_cast<ElemType>(_change_rec->_origin_elem);
        }
        return std::dynamic_pointer_cast<ElemType>(this->shared_from_this());
    };
    template <class ElemType>
    std::shared_ptr<ElemType> remote_elem() {
        if (_change_rec != nullptr && _change_rec->_remote_elem) {
            return std::dynamic_pointer_cast<ElemType>(_change_rec->_remote_elem);
        }
        return std::dynamic_pointer_cast<ElemType>(this->shared_from_this());
    };
    template <class ElemType>
    std::shared_ptr<ElemType> local_elem() {
        if (_change_rec != nullptr && _change_rec->_local_elem) {
            return std::dynamic_pointer_cast<ElemType>(_change_rec->_local_elem);
        }
        return std::dynamic_pointer_cast<ElemType>(this->shared_from_this());
    };
    mutable FeatureChangeRecord* _change_rec;

    std::shared_ptr<FeatureIDProxy> _id;
    
    //复制本元素id并标记为删除后在返回，用于在元素几何变化后提交给服务器删除特定tile对本元素的引用
    std::shared_ptr<FeatureIDProxy> mutable_deleted_id(int64_t ver) {
        if (_change_rec == nullptr) {
            _change_rec = new FeatureChangeRecord();
        }
        if (!_change_rec->_deleted_id && _id) {
            _change_rec->_deleted_id = _id->dump<FeatureIDProxy>();
            _change_rec->_deleted_id->set_is_deleted(true);
            _change_rec->_deleted_id->set_version(ver);
            _change_rec->_deleted_id->set_parent(this);
            _change_rec->_deleted_id->set_owner(this);
        }
        return _change_rec->_deleted_id;
    };
    std::shared_ptr<FeatureIDProxy> mutable_changed_id(int64_t ver) {
        if (_change_rec == nullptr) {
            _change_rec = new FeatureChangeRecord();
        }
        if (!_change_rec->_changed_id && _id) {
            _change_rec->_changed_id = _id->dump<FeatureIDProxy>();
            _change_rec->_changed_id->set_version(ver);
            _change_rec->_changed_id->set_parent(this);
            _change_rec->_changed_id->set_owner(this);
        }
        return _change_rec->_changed_id;
    }    

    bool _editable;    
};

template <class ProxyType>
const std::shared_ptr<const ProxyType> FeatureIDProxy::as() const {
    if (_owner == nullptr) {
        return nullptr;
    }
    if (_is_deleted && *_is_deleted) {
        return nullptr;
    }
    return std::dynamic_pointer_cast<const ProxyType>(_owner->shared_from_this());
};
template <class ProxyType>
const std::shared_ptr<ProxyType> FeatureIDProxy::as() {
    if (_owner == nullptr) {
        return nullptr;
    }
    if (_is_deleted && *_is_deleted) {
        return nullptr;
    }
    return std::dynamic_pointer_cast<ProxyType>(_owner->shared_from_this());
};

template <class Proxy>
class FeatureReference;

//封装路网元素中通过id表达的一个强引用关系，使用上可以看做一个类似
//std::shared_ptr<const FeatureIDProxy>的值类型
//通过重载赋值运算符在赋值时自动维护相关对象的引用、被引用关系
class FeatureMemberBase {
public:
    //构造一个没有外层元素的强引用对象，只用于临时存储强引用关系
    FeatureMemberBase() {};
    //构造被指定路网元素包含的强引用对象
    FeatureMemberBase(const FeatureWithIDProxyBase* e) {
        if (e != nullptr) {
            _container = e->reference_record();
        }
        else {
            LOG_BEGIN(INFO) << "FeatureMemberBase() construct with empty container!"; LOG_END;
        }
    };
    //构造指定外层父proxy的强引用对象
    FeatureMemberBase(FeatureProxyBase* p) {
        if (p != nullptr) {
            auto cont = p->container();
            if (cont) {
                _container = cont->reference_record();
            }
            else {
                LOG_BEGIN(INFO) << "FeatureMemberBase() construct without container in parent "
                    << p; LOG_END;
            }
        }
        else {
            LOG_BEGIN(INFO) << "FeatureMemberBase() construct with empty parent!"; LOG_END;
        }
    };
    ~FeatureMemberBase() {
        reset(false);
        _container.reset();
    };
    FeatureMemberBase(const FeatureMemberBase& e) = delete;
    FeatureMemberBase(FeatureMemberBase&& e) = delete;
    FeatureMemberBase& operator=(const FeatureMemberBase&) = delete;
    FeatureMemberBase& operator=(FeatureMemberBase&&) = delete;

    //类似shared_ptr通过本对象能够直接访问内部的FeatureIDProxy
    const FeatureIDProxy* operator->() const { return _id.get(); };
    FeatureIDProxy* operator->() { return _id.get(); };
    const FeatureIDProxy& operator*() const { return *_id; };
    //判断本引用是否已设置id
    explicit operator bool() const { return _id.operator bool(); };
    //返回包含本引用的路网元素
    const FeatureWithIDProxyBase* container() const { 
        if (_container) {
            return _container->element();
        }
        return nullptr; 
    };
    bool release_container(const FeatureWithIDProxyBase* e) {
        if (e != nullptr && e->reference_record() == _container) {
            _container.reset();
            return true;
        }
        return false;
    }
    bool reset_container(std::shared_ptr<RefRecordBase> rec) {
        if (_container || _id) {
            return false;
        }
        _container = rec;
        return true;
    };

    //清除本对象记录的强引用信息，参数控制是否标记变化
    void reset(bool bchanged = true) {
        if (!_id) {
            return;
        }
        auto cont = const_cast<FeatureWithIDProxyBase*>(container());
        if (bchanged && cont != nullptr) {
            cont->mark_changed();
        }
        if (_id->owner() != nullptr) {
            _id->owner()->remove_owner_reference(this);
        }
        _id.reset();
    };
        
    //本强引用关系中被引用对象的ID
    const std::shared_ptr<const FeatureIDProxy> id() const { return _id; };
    const std::shared_ptr<FeatureIDProxy>& id() { return _id; };

protected:
    //设置本强引用中引用的id，维护相应对象的引用关系信息
    void set_element(const std::shared_ptr<FeatureIDProxy>& id) {
        if (_id == id) {
            return;
        }
        bool bchanged = (!id || !_id || !_id->is_equal(*id));
        reset(bchanged);
        _id = id;
        if (id) {
            if (id->owner() != nullptr) {
                id->owner()->add_owner_reference(this);
            }
        }
    };
    void set_element(const std::shared_ptr<const FeatureIDProxy>& id) {
        auto eid = std::const_pointer_cast<FeatureIDProxy>(id);
        set_element(eid);
    };

protected:
    template <class Proxy>
    friend class FeatureMember;
    template <class Proxy>
    friend class FeatureMemberVector;
    std::shared_ptr<FeatureIDProxy> _id;
    std::shared_ptr<RefRecordBase> _container;
};

//封装对特定类型的元素的强引用，继承FeatureMemberBase的全部方法
template<class Proxy>
class FeatureMember : public FeatureMemberBase {
public:
    //构造一个没有外层元素的强引用对象，只用于临时存储强引用关系
    FeatureMember() : FeatureMemberBase() {};
    //构造被指定路网元素包含的强引用对象
    FeatureMember(const FeatureWithIDProxyBase* e) : FeatureMemberBase(e) {};
    //构造指定外层父proxy的强引用对象
    FeatureMember(FeatureProxyBase* p) : FeatureMemberBase(p) {};
    ~FeatureMember() {};
    FeatureMember(const FeatureMember& e) : FeatureMemberBase() {
        _container = e._container;
        set_element(e._id);
    };
    FeatureMember(FeatureMember<Proxy>&& e) {
        _container = e._container;
        set_element(e._id);
        e.reset();
        e._container.reset();
    };

    //重载赋值操作符，在改变路网元素引用的路网元素时维护两个路网元素内部记录的引用信息
    FeatureMember<Proxy>& operator=(const std::shared_ptr<const FeatureIDProxy>& id) {
        auto eid = std::const_pointer_cast<FeatureIDProxy>(id);
        if (eid && eid->type() != 0 && !eid->is_type(Proxy::ELEM_ID_TYPE)) {
            throw std::runtime_error("FeatureMember set to id with error type!");
        }
        set_element(eid);
        return (*this);
    };
    void set(const std::shared_ptr<const FeatureIDProxy>& id) {
        (*this) = id;
    };
    FeatureMember<Proxy>& operator=(const std::shared_ptr<FeatureIDProxy>& id) {
        if (id && id->type() != 0 && !id->is_type(Proxy::ELEM_ID_TYPE)) {
            throw std::runtime_error("FeatureMember set to id with error type!");
        }
        set_element(id);
        return (*this);
    };
    void set(const std::shared_ptr<FeatureIDProxy>& id) {
        (*this) = id;
    };
    FeatureMember<Proxy>& operator=(const FeatureReference<Proxy>& e) {
        set_element(e.id());
        return (*this);
    };
    void set(const FeatureReference<Proxy>& e) {
        (*this) = e;
    };
    FeatureMember<Proxy>& operator=(const FeatureMember<Proxy>& e) {
        set_element(e.id());
        return (*this);
    };
    void set(const FeatureMember<Proxy>& e) {
        (*this) = e;
    };
    FeatureMember<Proxy>& operator=(FeatureMember<Proxy>&& e) {
        set_element(e.id());
        e.reset(false);
        return (*this);
    };

    //类似shared_ptr通过本对象能够直接访问内部的FeatureIDProxy
    const FeatureIDProxy* operator->() const { return _id.get(); };
    FeatureIDProxy* operator->() { return _id.get(); };
    const FeatureIDProxy& operator*() const { return *_id; };
    //判断本引用是否已设置id
    explicit operator bool() const { return _id.operator bool(); };

    bool operator==(const FeatureMember<Proxy>& rhs) const {
        return ((!rhs._id && !_id) || (rhs._id && _id && (*rhs._id == *_id)));
    };
    bool operator<(const FeatureMember<Proxy>& rhs) const {
        if (!_id && !rhs._id) {
            return false;
        }
        if (!_id) {
            return true;
        }
        if (!rhs._id) {
            return false;
        }
        return (*_id) < (*rhs._id);
    };
    bool operator!=(const FeatureMember<Proxy>& rhs) const {
        return !operator==(rhs);
    };
    int compare(const FeatureMember<Proxy>& rhs) const {
        if (!_id && !rhs._id) {
            return 0;
        }
        if (!_id) {
            return -1;
        }
        if (!rhs._id) {
            return 1;
        }
        return _id->compare(*rhs._id);
    };

    //本强引用被引用的道路元素的指针
    const std::shared_ptr<const Proxy> proxy() const {
        if (!_id) {
            return nullptr;
        }
        return _id->as<Proxy>();
    };
    std::shared_ptr<Proxy> proxy() {
        if (!_id) {
            return nullptr;
        }
        return _id->as<Proxy>();
    };
    //交换两个强引用对象的信息
    void swap(FeatureMember<Proxy>& e) {
        std::swap(e._container, _container);
        _id.swap(e._id);
    };
    //为本对象重新构造一个空的引用ID
    FeatureIDProxy* create(FeatureProxyBase* e) {
        reset(true);
        if (!_container) {
            if (e->container()) {
                reset_container(e->container()->reference_record());
            }
        }
        _id.reset(FeatureProxyBase::create_proxy<FeatureIDProxy>(e));
        return _id.get();
    };
};

//封装路网元素中通过id表达的一个弱引用关系，使用上可以看做一个类似
//std::shared_ptr<const FeatureIDProxy>的值类型
//通过重载赋值运算符在赋值时自动维护相关对象的引用、被引用关系
class FeatureReferenceBase {
public:
    //构造一个没有外层元素的弱引用对象，只用于临时存储弱引用关系
    FeatureReferenceBase() {};
    //构造被指定路网元素包含的弱引用对象
    FeatureReferenceBase(const FeatureWithIDProxyBase* e) {
        if (e != nullptr) {
            _container = e->reference_record();
        }
        else {
            LOG_BEGIN(INFO) << "FeatureReferenceBase() construct with empty container!"; LOG_END;
        }
    };
    FeatureReferenceBase(FeatureProxyBase* p) {
        if (p) {
            auto cont = p->container();
            if (cont) {
                _container = cont->reference_record();
            }
            else {
                LOG_BEGIN(INFO) << "FeatureReferenceBase() construct without container in parent "
                    << p; LOG_END;
            }
        }
        else {
            LOG_BEGIN(INFO) << "FeatureReferenceBase() construct with empty parent!"; LOG_END;
        }
    };
    ~FeatureReferenceBase() {
        reset(false);
        _container.reset();
    };
    FeatureReferenceBase(const FeatureReferenceBase& e) = delete;
    FeatureReferenceBase(FeatureReferenceBase&& e) = delete;
    FeatureReferenceBase& operator=(const FeatureReferenceBase&) = delete;
    FeatureReferenceBase& operator=(FeatureReferenceBase&&) = delete;
        
    //类似shared_ptr通关本对象能够直接访问内部的FeatureIDProxy，对于弱引用只提供const接口
    const FeatureIDProxy* operator->() const { return _id.get(); };
    const FeatureIDProxy& operator*() const { return *_id; };
    //判断本引用是否已设置过id
    explicit operator bool() const { return _id.operator bool(); };
    //返回包含本引用的路网元素
    const FeatureWithIDProxyBase* container() const {
        if (_container) {
            return _container->element();
        }
        return nullptr;
    };
    bool release_container(const FeatureWithIDProxyBase* e) { 
        if (e != nullptr && e->reference_record() == _container) {
            _container.reset();
            return true;
        }
        return false;
    };
    bool reset_container(std::shared_ptr<RefRecordBase> rec) {
        if (_container || _id) {
            return false;
        }
        _container = rec;
        return true;
    };
    //清除本兑现记录的弱引用信息，参数控制是否标记变化
    void reset(bool bchanged = true) {
        if (!_id) {
            return;
        }
        auto cont = const_cast<FeatureWithIDProxyBase*>(container());
        if (bchanged && cont != nullptr) {
            cont->mark_changed();
        }        
        if (cont != nullptr && cont->feature_id_type() == -1) {
            if (_id->owner() != nullptr) {
                auto tile = reinterpret_cast<TileInfoProxy*>(cont);
                _id->owner()->remove_referenced_tiles(tile);
            }
        }
        else {
            if (_id->owner() != nullptr) {
                _id->owner()->remove_weak_reference(this);
            }
        }
        _id.reset();
    };
    
    //本弱引用关系中的被引用对象
    const std::shared_ptr<const FeatureIDProxy> id() const { return _id; };

protected:
    //设置本弱引用中引用的id，维护相应对象的引用关系信息
    void set_element(const std::shared_ptr<FeatureIDProxy>& id) {
        if (_id == id) {
            return;
        }        
        bool bchanged = (!id || !_id || !_id->is_equal(*id));
        reset(bchanged);
        _id = id;
        if (container() != nullptr && container()->feature_id_type() == -1) {
            auto tile = reinterpret_cast<const TileInfoProxy*>(container());
            if (id && id->owner() != nullptr) {
                id->owner()->add_referenced_tiles(tile);                
            }
        }
        else if (id) {
            if (id->owner() != nullptr) {
                id->owner()->add_weak_reference(this);
            }
        }
    };
    void set_element(const std::shared_ptr<const FeatureIDProxy>& id) {
        auto eid = std::const_pointer_cast<FeatureIDProxy>(id);
        set_element(eid);
    };
        
protected:
    friend class RefRecordBase;
    template <class Proxy>
    friend class FeatureReference;
    template <class Proxy>
    friend class FeatureReferenceVector;
    std::shared_ptr<FeatureIDProxy> _id;
    std::shared_ptr<RefRecordBase> _container;
};

//封装对非类型的元素的弱引用，继承FeatureReferenceBase的全部方法
class FeatureReferenceAny : public FeatureReferenceBase
{
public:
    //构造一个没有外层元素的弱引用对象，只用于临时存储弱引用关系
    FeatureReferenceAny() : FeatureReferenceBase() {};
    //构造被指定路网元素包含的弱引用对象
    FeatureReferenceAny(const FeatureWithIDProxyBase* e) : FeatureReferenceBase(e) {};
    FeatureReferenceAny(FeatureProxyBase* p) : FeatureReferenceBase(p) {};
    ~FeatureReferenceAny() {};
    FeatureReferenceAny(const FeatureReferenceAny& e) : FeatureReferenceBase() {
        _container = e._container;
        set_element(e._id);
    };
    FeatureReferenceAny(FeatureReferenceAny&& e) {
        _container = e._container;
        set_element(e._id);
        e.reset();
        e._container.reset();
    };

    //重载赋值操作符，在改变路网元素引用的路网元素时维护两个路网元素内部记录的引用信息
    FeatureReferenceAny& operator=(const std::shared_ptr<const FeatureIDProxy>& id) {
        auto eid = std::const_pointer_cast<FeatureIDProxy>(id);
        if (eid && !eid->is_valid()) {
            throw std::runtime_error("FeatureReference set to id with error type!");
        }
        set_element(eid);
        return (*this);
    };
    void set(const std::shared_ptr<const FeatureIDProxy>& id) {
        (*this) = id;
    };
    FeatureReferenceAny& operator=(const std::shared_ptr<FeatureIDProxy>& id) {
        if (id && id->type() != 0) {
            throw std::runtime_error("FeatureReference set to id with error type!");
        }
        set_element(id);
        return (*this);
    };
    void set(const std::shared_ptr<FeatureIDProxy>& id) {
        (*this) = id;
    };
    FeatureReferenceAny& operator=(const FeatureReferenceBase& e) {
        set_element(e.id());
        return (*this);
    };
    FeatureReferenceAny& operator=(const FeatureReferenceAny& e) {
        set_element(e.id());
        return (*this);
    };
    void set(const FeatureReferenceBase& e) {
        (*this) = e;
    };
    FeatureReferenceAny& operator=(const FeatureMemberBase& e) {
        set_element(e.id());
        return (*this);
    };
    void set(const FeatureMemberBase& e) {
        (*this) = e;
    };
    FeatureReferenceAny& operator=(FeatureReferenceAny&& e) {
        set_element(e.id());
        e.reset(false);
        return (*this);
    };

    //类似shared_ptr通关本对象能够直接访问内部的FeatureIDProxy，对于弱引用只提供const接口
    const FeatureIDProxy* operator->() const { return _id.get(); };
    const FeatureIDProxy& operator*() const { return *_id; };
    //判断本引用是否已设置过id
    explicit operator bool() const { return _id.operator bool(); };

    bool operator==(const FeatureReferenceAny& rhs) const {
        return ((!rhs._id && !_id) || (rhs._id && _id && rhs._id->is_equal(*_id)));
    };
    bool operator<(const FeatureReferenceAny& rhs) const {
        if (!_id && !rhs._id) {
            return false;
        }
        if (!_id) {
            return true;
        }
        if (!rhs._id) {
            return false;
        }
        return _id->is_less(*rhs._id);
    };
    bool operator!=(const FeatureReferenceAny& rhs) const {
        return !operator==(rhs);
    };
    int compare(const FeatureReferenceAny& rhs) const {
        if (!_id && !rhs._id) {
            return 0;
        }
        if (!_id) {
            return -1;
        }
        if (!rhs._id) {
            return 1;
        }
        return _id->compare(*rhs._id);
    };

    //交换两个弱引用对象的信息
    void swap(FeatureReferenceAny& e) {
        std::swap(e._container, _container);
        _id.swap(e._id);
    };
    //为本对象重新构造一个空的引用ID
    FeatureIDProxy* create(FeatureProxyBase* e) {
        reset(true);
        if (_container) {
            if (e->container()) {
                reset_container(e->container()->reference_record());
            }
        }
        _id.reset(FeatureProxyBase::create_proxy<FeatureIDProxy>(e));
        return _id.get();
    };
};

//封装对特定类型的元素的弱引用，继承FeatureReferenceBase的全部方法
template <class Proxy>
class FeatureReference : public FeatureReferenceBase
{
public:
    //构造一个没有外层元素的弱引用对象，只用于临时存储弱引用关系
    FeatureReference() : FeatureReferenceBase() {};
    //构造被指定路网元素包含的弱引用对象
    FeatureReference(const FeatureWithIDProxyBase* e) : FeatureReferenceBase(e) {};
    FeatureReference(FeatureProxyBase* p) : FeatureReferenceBase(p) {};
    ~FeatureReference() {};
    FeatureReference(const FeatureReference<Proxy>& e) : FeatureReferenceBase() {
        _container = e._container;
        set_element(e._id);
    };
    FeatureReference(FeatureReference<Proxy>&& e) {
        _container = e._container;
        set_element(e._id);
        e.reset();
        e._container.reset();
    };

    //重载赋值操作符，在改变路网元素引用的路网元素时维护两个路网元素内部记录的引用信息
    FeatureReference<Proxy>& operator=(const std::shared_ptr<const FeatureIDProxy>& id) {
        auto eid = std::const_pointer_cast<FeatureIDProxy>(id);
        if (eid && eid->type() != 0 && !eid->is_type(Proxy::ELEM_ID_TYPE)) {
            throw std::runtime_error("FeatureReference set to id with error type!");
        }
        set_element(eid);
        return (*this);
    };
    void set(const std::shared_ptr<const FeatureIDProxy>& id) {
        (*this) = id;
    };
    FeatureReference<Proxy>& operator=(const std::shared_ptr<FeatureIDProxy>& id) {
        if (id && id->type() != 0 && !id->is_type(Proxy::ELEM_ID_TYPE)) {
            throw std::runtime_error("FeatureReference set to id with error type!");
        }
        set_element(id);
        return (*this);
    };
    void set(const std::shared_ptr<FeatureIDProxy>& id) {
        (*this) = id;
    };
    FeatureReference<Proxy>& operator=(const FeatureReference<Proxy>& e) {
        set_element(e._id);
        return (*this);
    };
    void set(const FeatureReference<Proxy>& e) {
        (*this) = e;
    };
    FeatureReference<Proxy>& operator=(const FeatureMember<Proxy>& e) {
        set_element(e.id());
        return (*this);
    };
    void set(const FeatureMember<Proxy>& e) {
        (*this) = e;
    };
    FeatureReference<Proxy>& operator=(FeatureReference<Proxy>&& e) {
        set_element(e.id());
        e.reset(false);
        return (*this);
    };

    //类似shared_ptr通关本对象能够直接访问内部的FeatureIDProxy，对于弱引用只提供const接口
    const FeatureIDProxy* operator->() const { return _id.get(); };
    const FeatureIDProxy& operator*() const { return *_id; };
    //判断本引用是否已设置过id
    explicit operator bool() const { return _id.operator bool(); };

    bool operator==(const FeatureReference<Proxy>& rhs) const {
        return ((!rhs._id && !_id) || (rhs._id && _id && rhs._id->is_equal(*_id)));
    };
    bool operator<(const FeatureReference<Proxy>& rhs) const {
        if (!_id && !rhs._id) {
            return false;
        }
        if (!_id) {
            return true;
        }
        if (!rhs._id) {
            return false;
        }
        return _id->is_less(*rhs._id);
    };
    bool operator!=(const FeatureReference<Proxy>& rhs) const {
        return !operator==(rhs);
    };
    int compare(const FeatureReference<Proxy>& rhs) const {
        if (!_id && !rhs._id) {
            return 0;
        }
        if (!_id) {
            return -1;
        }
        if (!rhs._id) {
            return 1;
        }
        return _id->compare(*rhs._id);
    };

    //本弱引用被引用的道路元素的指针
    const std::shared_ptr<const Proxy> proxy() const {
        if (!_id) {
            return nullptr;
        }
        return _id->as<Proxy>();
    };
    std::shared_ptr<Proxy> proxy() {
        if (!_id) {
            return nullptr;
        }
        return _id->as<Proxy>();
    };
    //交换两个弱引用对象的信息
    void swap(FeatureReference<Proxy>& e) {
        std::swap(e._container, _container);
        _id.swap(e._id);
    };
    //为本对象重新构造一个空的引用ID
    FeatureIDProxy* create(FeatureProxyBase* e) {
        reset(true);
        _id.reset(FeatureProxyBase::create_proxy<FeatureIDProxy>(e));
        return _id.get();
    };
};

//封装一个强引用关系的数组，会传递外层路网元素信息给每个引用关系，操作中会维护路网元素间引用关系记录
template <class Proxy>
class FeatureMemberVector {
public:
    //构造属于特定外层元素的强引用数组，对于临时存储用的数组可以传入nullptr
    FeatureMemberVector(FeatureProxyBase* p) : _parent(p) {};
    ~FeatureMemberVector() { clear(); };

    //屏蔽拷贝构造函数，避免通过复制一个新数组来绕过只读访问权限控制
    FeatureMemberVector(const FeatureMemberVector&) = delete;
    FeatureMemberVector& operator=(const FeatureMemberVector&) = delete;

    //返回内部强引用的数目
    size_t size() const { return _proxys.size(); };
    //返回本数组是否为空
    bool empty() const { return _proxys.empty(); };

    //返回数组中的特定下标的元素，不进行越界检查，直接传递给std::vector来检查
    const FeatureMember<Proxy>& at(size_t index) const { return _proxys[index]; };
    FeatureMember<Proxy>& at(size_t index) { return _proxys[index]; };
    const FeatureMember<Proxy>& operator[](size_t index) const { return _proxys[index]; };
    FeatureMember<Proxy>& operator[](size_t index) { return _proxys[index]; };
    const FeatureMember<Proxy>& front() const { return _proxys.front(); };
    FeatureMember<Proxy>& front() { return _proxys.front(); };
    const FeatureMember<Proxy>& back() const { return _proxys.back(); };
    FeatureMember<Proxy>& back() { return _proxys.back(); };

    //构造一个新引用关系，追加到数组末尾，返回其指针
    FeatureMember<Proxy>* add() {
        if (_parent) {
            if (!_parent->is_editable()) {
                return nullptr;
            }
            _parent->mark_changed();
            _proxys.push_back(FeatureMember<Proxy>());
            auto cont = _parent->container();
            if (cont) {
                _proxys.back()._container = cont->reference_record();
            }
            _proxys.back().create(_parent);
            return &_proxys.back();
        }
        _proxys.push_back(FeatureMember<Proxy>());
        _proxys.back().create(_parent);
        return &_proxys.back();
    };
    //把一个强引用关系加入数组末尾，并维护相关引用关系
    bool push_back(const FeatureMember<Proxy>& p) {
        auto e = add();
        if (e == nullptr) {
            return false;
        }
        *e = p;
        return true;
    };
    bool push_back(const std::shared_ptr<const FeatureIDProxy>& id) {
        auto e = add();
        if (e == nullptr) {
            return false;
        }
        *e = id;
        return true;
    };
    //清空本数组
    bool clear() {
        if (_parent) {
            if (!_parent->is_editable()) {
                return false;
            }
            if (!_proxys.empty()) {
                _parent->mark_changed();
            }
        }
        _proxys.clear();
        return true;
    };
    //在特定下标位置插入一个强引用信息
    bool insert(size_t index, const FeatureMember<Proxy>& p) {
        if (_parent) {
            if (!_parent->is_editable()) {
                return false;
            }
            _parent->mark_changed();
        }
        FeatureMember<Proxy> e(_parent);
        e = p;
        _proxys.insert(_proxys.begin() + index, e);
        return true;
    };
    bool insert(size_t index, const std::shared_ptr<const FeatureIDProxy>& id) {
        if (_parent) {
            if (!_parent->is_editable()) {
                return false;
            }
            _parent->mark_changed();
        }
        FeatureMember<Proxy> e(_parent);
        e = id;
        _proxys.insert(_proxys.begin() + index, e);
        return true;
    };
    //移除数据中特定下标的引用关系
    bool erase(size_t index) {
        if (_parent) {
            if (!_parent->is_editable()) {
                return false;
            }
            _parent->mark_changed();
        }
        _proxys.erase(_proxys.begin() + index);
        return true;
    };
    //把本数组中的某个特定强引用替换为一个新的引用
    bool replace(const std::shared_ptr<const FeatureIDProxy>& old_id,
            const std::shared_ptr<FeatureIDProxy>& new_id) {
        if (_parent) {
            if (!_parent->is_editable()) {
                return false;
            }
            _parent->mark_changed();
        }
        for (auto& p : _proxys) {
            if (p.id() == old_id) {
                p = new_id;
                return true;
            }
        }
        return false;
    };
    //设置本数组预留空间的大小
    void reserve(size_t rs) {
        _proxys.reserve(rs);
    };
    //重设本数组的大小，扩大的场景会自动构造空的强引用对象
    bool resize(size_t ns) {
        if (_parent) {
            if (!_parent->is_editable()) {
                return false;
            }
            _parent->mark_changed();
        }
        size_t oldsz = _proxys.size();
        _proxys.resize(ns);
        for (size_t i = oldsz; i < ns; ++i) {
            FeatureMember<Proxy> e(_parent);
            _proxys[i].swap(e);
        }
        return true;
    };
    //判断本数组中的强引用是否都有对应路网元素对象
    bool is_completed() const {
        for (auto& e : _proxys) {
            if (e.id() && !e.proxy()) {
                return false;
            }
        }
        return true;
    };
    //判断本数组中的引用关系是否全部有效，且不包含重复的引用
    bool is_valid(bool no_deleted) const {
        for (auto& e : _proxys) {
            if (!e.id() || !e.id()->cached_is_valid() || !e.proxy()) {
                std::string sid("!no_id!");
                if (e) {
                    sid = e->to_string();
                }
                LOG_BEGIN(INFO) << "FeatureMemberVector::is_valid() has invalid id "
                    << sid; LOG_END;
                return false;
            }
            if (no_deleted && e->is_deleted()) {
                std::string sid("!no_id!");
                if (e) {
                    sid = e->to_string();
                }
                LOG_BEGIN(INFO) << "FeatureMemberVector::is_valid() has deleted id "
                    << sid; LOG_END;
                return false;
            }
        }
        if (_proxys.size() < 16) {
            for (size_t i = 0; i + 1 < _proxys.size(); ++i) {
                for (size_t j = i + 1; j < _proxys.size(); ++j) {
                    if (_proxys[i]->is_equal(*(_proxys[j].id()))) {
                        LOG_BEGIN(INFO) << "FeatureMemberVector::is_valid() has duplicated id "
                            << _proxys[j]->to_string() << " at " << j << " with "
                            << _proxys[i]->to_string() << " at " << i << " / "
                            << _proxys.size(); LOG_END;
                        return false;
                    }
                }
            }
        }
        else {
            std::unordered_multimap<uint64_t, const FeatureIDProxy*> ids(_proxys.size() * 2);
            for (size_t i = 0; i < _proxys.size(); ++i) {
                auto& e = _proxys[i];
                auto rng = ids.equal_range(e->id());
                for (auto it = rng.first; it != rng.second; ++it) {
                    if (it->second->is_equal(*e.id())) {
                        LOG_BEGIN(INFO) << "FeatureMemberVector::is_valid() has duplicated id "
                            << _proxys[i]->to_string() << " at " << i << " / "
                            << _proxys.size(); LOG_END;
                        return false;
                    }
                }
                ids.insert(std::make_pair(e->id(), e.id().get()));
            }
        }
        return true;
    };
    //从本数组中移除错误的、重复的引用信息
    bool remove_invalid(bool no_deleted) {
        if (_parent) {
            if (!_parent->is_editable()) {
                return false;
            }
        }
        bool bf = false;
        for (size_t i = 0; i < _proxys.size(); ++i) {
            auto& p = _proxys[i];
            if (!p || !p->cached_is_valid() || !p.proxy()
                || (no_deleted && p->is_deleted())) {
                std::string sid("!no_id!");
                if (p) {
                    sid = p->to_string();
                }
                LOG_BEGIN(INFO) << "FeatureMemberVector::remove_invalid(): id " << sid
                    << " at " << i << '/' << _proxys.size(); LOG_END;
                if (!bf && _parent) {
                    _parent->mark_changed();
                }
                _proxys.erase(_proxys.begin() + i);
                --i;
                bf = true;
                continue;
            }
        }
        if (_proxys.size() < 16) {
            for (size_t i = 0; i < _proxys.size(); ++i) {
                auto& p = _proxys[i];
                for (size_t j = i + 1; j < _proxys.size(); ++j) {
                    if (p->is_equal(*_proxys[j].id())) {
                        std::string sid("!no_id!");
                        if (p) {
                            sid = p->to_string();
                        }
                        LOG_BEGIN(INFO) << "FeatureMemberVector::remove_invalid():"
                            << " remove duplicated id " << sid
                            << " at " << j << " with " << i << " / " << _proxys.size(); LOG_END;
                        if (!bf && _parent) {
                            _parent->mark_changed();
                        }
                        _proxys.erase(_proxys.begin() + j);
                        --j;
                        bf = true;
                    }
                }
            }
        }
        else {
            std::unordered_multimap<uint64_t, const FeatureIDProxy*> ids(_proxys.size() * 2);
            for (size_t i = 0; i < _proxys.size(); ++i) {
                auto& e = _proxys[i];
                auto rng = ids.equal_range(e->id());
                bool bfound = false;
                for (auto it = rng.first; it != rng.second; ++it) {
                    if (it->second->is_equal(*e.id())) {
                        std::string sid("!no_id!");
                        if (e) {
                            sid = e->to_string();
                        }
                        LOG_BEGIN(INFO) << "FeatureMemberVector::remove_invalid():"
                            << " remove duplicated id " << sid
                            << " at " << i << " / " << _proxys.size(); LOG_END;
                        if (!bf && _parent) {
                            _parent->mark_changed();
                        }
                        _proxys.erase(_proxys.begin() + i);
                        --i;
                        bf = true;
                        bfound = true;
                        break;
                    }
                }
                if (!bfound) {
                    ids.insert(std::make_pair(e->id(), e.id().get()));
                }
            }
        }        
        return bf;
    };
    //获取数组中每个id对应的路网元素，把元素的id设置其强引用中记录的id
    bool resolve_proxy(RoadGeometryManager* mgr) {
        bool bf = true;
        for (auto& e : _proxys) {
            if (e.id()) {
                auto p = e.id()->resolve_id(mgr);
                if (p) {
                    e = p->id();
                }
                else {
                    LOG_BEGIN(DEBUG) << "resolve_proxy() failed for " << e->to_string(); LOG_END;
                    bf = false;
                }
            }
            else {
                LOG_BEGIN(ERROR) << "resolve_proxy() failed for empty id"; LOG_END;
                bf = false;
            }
        }
        return bf;
    };
    //清除本数组中全部引用关系的更新标记
    void clear_changed() {
        for (auto& e : _proxys) {
            if (e.id()) {
                e->clear_changed();
            }
        }
    };

    //交换两个数组的内容，可以用于释放内存/重设parent
    void swap(FeatureMemberVector<Proxy>& v) {
        std::swap(v._parent, _parent);
        v._proxys.swap(_proxys);
    };

    //从pb的repeated字段中生成本数组的引用信息
    bool from_message(const google::protobuf::RepeatedPtrField<RoadPB::FeatureID>& ids) {
        clear();
        reserve(ids.size());
        for (int i = 0; i < ids.size(); ++i) {
            auto e = add();
            if (e != nullptr) {
                e->id()->from_message(&ids.Get(i));
            }
            else {
                return false;
            }
        }
        return true;
    };
    //把本数组中记录的引用关系写入到特定的pb的repeated字段中
    bool to_message(google::protobuf::RepeatedPtrField<RoadPB::FeatureID>* ids) const {
        ids->Reserve((int)size());
        ids->Clear();
        for (auto& e : _proxys) {
            if (e) {
                e->to_message(ids->Add());
            }
        }
        return true;
    };

    void sorted_copy(FeatureMemberVector<Proxy>& org) {
        _proxys.clear();
        _proxys.reserve(org._proxys.size());
        for (size_t i = 0; i < org._proxys.size(); ++i) {
            bool bf = false;
            for (size_t j = 0; j < i; ++j) {
                if (_proxys[j].compare(org._proxys[i]) > 0) {
                    insert(j, org._proxys[i]);
                    bf = true;
                    break;
                }
            }
            if (!bf) {
                push_back(org._proxys[i]);
            }
        }
    };

    typedef typename std::vector<FeatureMember<Proxy> >::iterator elem_iterator;
    typedef typename std::vector<FeatureMember<Proxy> >::reverse_iterator elem_reverse_iterator;
    typedef typename std::vector<FeatureMember<Proxy> >::const_iterator elem_const_iterator;
    typedef typename std::vector<FeatureMember<Proxy> >::const_reverse_iterator elem_const_reverse_iterator;
    //透传内部std::vector的iterator，提供与std::vector相同的接口
    elem_iterator begin() { return _proxys.begin(); };
    elem_const_iterator begin() const { return _proxys.begin(); };
    elem_iterator end() { return _proxys.end(); };
    elem_const_iterator end() const { return _proxys.end(); };
    elem_reverse_iterator rbegin() { return _proxys.rbegin(); };
    elem_const_reverse_iterator rbegin() const { return _proxys.rbegin(); };
    elem_reverse_iterator rend() { return _proxys.rend(); };
    elem_const_reverse_iterator rend() const { return _proxys.rend(); };
    elem_iterator erase(elem_const_iterator it) { 
        if (_parent) {
            if (!_parent->is_editable()) {
                return _proxys.end();
            }
            _parent->mark_changed();
        }
        return _proxys.erase(it); 
    };
    elem_reverse_iterator erase(elem_const_reverse_iterator it) {
        if (_parent) {
            if (!_parent->is_editable()) {
                return _proxys.rend();
            }
            _parent->mark_changed();
        }
        return _proxys.erase(it);
    };
    elem_const_iterator find(const std::shared_ptr<const FeatureIDProxy>& id) const {
        for (auto it = _proxys.begin(); it != _proxys.end(); ++it) {
            if (it->id() == id || it->id()->is_equal(*id)) {
                return it;
            }
        }
        return _proxys.end();
    };

private:
    std::vector<FeatureMember<Proxy>> _proxys;
    FeatureProxyBase* _parent;
};

//封装一个弱引用关系的数组，会传递外层路网元素信息给每个引用关系，操作中会维护路网元素间引用关系记录
template <class Proxy>
class FeatureReferenceVector {
public:
    //构造属于特定外层元素的弱引用数组，对于临时存储用的数据可以传入nullptr
    FeatureReferenceVector(FeatureProxyBase* p) : _parent(p) {};
    ~FeatureReferenceVector() { clear(); };

    //屏蔽拷贝构造函数，避免通过复制一个新的数据来绕过只读访问权限控制
    FeatureReferenceVector(const FeatureReferenceVector&) = delete;
    FeatureReferenceVector& operator=(const FeatureReferenceVector&) = delete;

    //返回数组中弱引用的数目
    size_t size() const { return _proxys.size(); };
    //返回本数据是否为空
    bool empty() const { return _proxys.empty(); };

    //返回数组中的特定下标的元素，不进行越界检查，直接传递给std::vector来检查
    const FeatureReference<Proxy>& at(size_t index) const { return _proxys[index]; };
    FeatureReference<Proxy>& at(size_t index) { return _proxys[index]; };
    const FeatureReference<Proxy>& operator[](size_t index) const { return _proxys[index]; };
    FeatureReference<Proxy>& operator[](size_t index) { return _proxys[index]; };
    const FeatureReference<Proxy>& front() const { return _proxys.front(); };
    FeatureReference<Proxy>& front() { return _proxys.front(); };
    const FeatureReference<Proxy>& back() const { return _proxys.back(); };
    FeatureReference<Proxy>& back() { return _proxys.back(); };

    //构造一个新引用关系，追加到数组末尾，返回其指针
    FeatureReference<Proxy>* add() {
        if (_parent) {
            if (!_parent->is_editable()) {
                return nullptr;
            }
            _parent->mark_changed();
            _proxys.push_back(FeatureReference<Proxy>());
            auto cont = _parent->container();
            if (cont) {
                _proxys.back()._container = cont->reference_record();
            }
            _proxys.back().create(_parent);
            return &_proxys.back();
        }
        _proxys.push_back(FeatureReference<Proxy>());
        _proxys.back().create(_parent);
        return &_proxys.back();
    };
    //把一个弱引用关系加入数组末尾，并维护相关引用关系
    bool push_back(const FeatureReference<Proxy>& p) {
        auto e = add();
        if (e == nullptr) {
            return false;
        }
        *e = p;
        return true;
    };
    bool push_back(const std::shared_ptr<const FeatureIDProxy>& id) {
        auto e = add();
        if (e == nullptr) {
            return false;
        }
        *e = id;
        return true;
    };
    //清空本数组
    bool clear() {
        if (_parent) {
            if (!_parent->is_editable()) {
                return false;
            }
            if (!_proxys.empty()) {
                _parent->mark_changed();
            }
        }
        _proxys.clear();
        return true;
    };
    //在特定下标位置插入一个强引用信息
    bool insert(size_t index, const FeatureReference<Proxy>& p) {
        if (_parent) {
            if (!_parent->is_editable()) {
                return false;
            }
            _parent->mark_changed();
        }
        FeatureReference<Proxy> e(_parent);
        e = p;
        _proxys.insert(_proxys.begin() + index, e);
        return true;
    };
    bool insert(size_t index, const std::shared_ptr<const FeatureIDProxy>& id) {
        if (_parent) {
            if (!_parent->is_editable()) {
                return false;
            }
            _parent->mark_changed();
        }
        FeatureReference<Proxy> e(_parent);
        e = id;
        _proxys.insert(_proxys.begin() + index, e);
        return true;
    };
    //移除数据中特定下标的引用关系
    bool erase(size_t index) {
        if (_parent) {
            if (!_parent->is_editable()) {
                return false;
            }
            _parent->mark_changed();
        }
        _proxys.erase(_proxys.begin() + index);
        return true;
    };
    //把本数组中的某个特定引用替换为一个新的引用
    bool replace(const std::shared_ptr<const FeatureIDProxy>& old_id,
        const std::shared_ptr<FeatureIDProxy>& new_id) {
        if (_parent) {
            if (!_parent->is_editable()) {
                return false;
            }
            _parent->mark_changed();
        }
        for (auto& p : _proxys) {
            if (p.id() == old_id) {
                p = new_id;
                return true;
            }
        }
        return false;
    };
    //设置本数组预留空间的大小
    void reserve(size_t rs) {
        _proxys.reserve(rs);
    };
    //重设本数组的大小，扩大的场景会自动构造空的弱引用对象
    bool resize(size_t ns) {
        if (_parent) {
            if (!_parent->is_editable()) {
                return false;
            }
            _parent->mark_changed();
        }
        size_t oldsz = _proxys.size();
        _proxys.resize(ns);
        for (size_t i = oldsz; i < ns; ++i) {
            FeatureMember<Proxy> e(_parent);
            _proxys[i].swap(e);
        }
        return true;
    };
    //判断本数组中的强引用是否都有对应路网元素对象
    bool is_completed() const {
        for (auto& e : _proxys) {
            if (e.id() && !e.proxy()) {
                return false;
            }
        }
        return true;
    };
    //判断本数组中的引用关系是否全部有效，且不包含重复的引用
    bool is_valid(bool no_deleted) const {
        for (auto& e : _proxys) {
            if (!e.id() || !e.id()->cached_is_valid()) {
                std::string sid("!no_id!");
                if (e) {
                    sid = e->to_string();
                }
                LOG_BEGIN(INFO) << "FeatureMemberVector::is_valid() has invalid id "
                    << sid; LOG_END;
                return false;
            }
            if (no_deleted && e->is_deleted()) {
                std::string sid("!no_id!");
                if (e) {
                    sid = e->to_string();
                }
                LOG_BEGIN(INFO) << "FeatureMemberVector::is_valid() has deleted id "
                    << sid; LOG_END;
                return false;
            }
        }
        if (_proxys.size() < 16) {
            for (size_t i = 0; i + 1 < _proxys.size(); ++i) {
                for (size_t j = i + 1; j < _proxys.size(); ++j) {
                    if (_proxys[i]->is_equal(*_proxys[j].id())) {
                        LOG_BEGIN(INFO) << "FeatureReferenceVector::is_valid() has duplicated id "
                            << _proxys[j]->to_string() << " at " << j << " with "
                            << _proxys[i]->to_string() << " at " << i << " / "
                            << _proxys.size(); LOG_END;
                        return false;
                    }
                }
            }
        }
        else {
            std::unordered_multimap<uint64_t, const FeatureIDProxy*> ids(_proxys.size() * 2);
            for (size_t i = 0; i < _proxys.size(); ++i) {
                auto& e = _proxys[i];
                auto rng = ids.equal_range(e->id());
                for (auto it = rng.first; it != rng.second; ++it) {
                    if (it->second->is_equal(*e.id())) {
                        LOG_BEGIN(INFO) << "FeatureReferenceVector::is_valid() has duplicated id "
                            << _proxys[i]->to_string() << " at " << i << " / "
                            << _proxys.size(); LOG_END;
                        return false;
                    }
                }
                ids.insert(std::make_pair(e->id(), e.id().get()));
            }
        }
        return true;
    };
    //从本数组中移除错误的、重复的引用信息
    bool remove_invalid(RoadGeometryManager* mgr, bool no_deleted) {
        if (_parent) {
            if (!_parent->is_editable()) {
                return false;
            }
        }
        bool bf = false;
        for (size_t i = 0; i < _proxys.size(); ++i) {
            auto& p = _proxys[i];
            if (!p || !p->cached_is_valid() || (no_deleted && p->is_deleted())
                || (!p->is_weak_reference_valid(mgr))) {
                std::string sid("!no_id!");
                if (p) {
                    sid = p->to_string();
                }
                LOG_BEGIN(INFO) << "FeatureReferenceVector::remove_invalid(): id " << sid
                    << " at " << i << '/' << _proxys.size(); LOG_END;
                if (!bf && _parent) {
                    _parent->mark_changed();
                }
                _proxys.erase(_proxys.begin() + i);
                --i;
                bf = true;
                continue;
            }
        }
        if (_proxys.size() < 16) {
            for (size_t i = 0; i < _proxys.size(); ++i) {
                auto& p = _proxys[i];
                for (size_t j = i + 1; j < _proxys.size(); ++j) {
                    if (p->is_equal(*_proxys[j].id())) {
                        std::string sid("!no_id!");
                        if (p) {
                            sid = p->to_string();
                        }
                        LOG_BEGIN(INFO) << "FeatureReferenceVector::remove_invalid():"
                            << " remove duplicated id " << sid
                            << " at " << j << " with " << i << " / " << _proxys.size(); LOG_END;
                        if (!bf && _parent) {
                            _parent->mark_changed();
                        }
                        _proxys.erase(_proxys.begin() + j);
                        --j;
                        bf = true;
                    }
                }
            }
        }
        else {
            std::unordered_multimap<uint64_t, const FeatureIDProxy*> ids(_proxys.size() * 2);
            for (size_t i = 0; i < _proxys.size(); ++i) {
                auto& e = _proxys[i];
                auto rng = ids.equal_range(e->id());
                bool bfound = false;
                for (auto it = rng.first; it != rng.second; ++it) {
                    if (it->second->is_equal(*e.id())) {
                        std::string sid("!no_id!");
                        if (e) {
                            sid = e->to_string();
                        }
                        LOG_BEGIN(INFO) << "FeatureReferenceVector::remove_invalid():"
                            << " remove duplicated id " << sid
                            << " at " << i << " / " << _proxys.size(); LOG_END;
                        if (!bf && _parent) {
                            _parent->mark_changed();
                        }
                        _proxys.erase(_proxys.begin() + i);
                        --i;
                        bf = true;
                        bfound = true;
                        break;
                    }
                }
                if (!bfound) {
                    ids.insert(std::make_pair(e->id(), e.id().get()));
                }
            }
        }        
        return bf;
    };
    //获取数组中每个id对应的路网元素，把元素的id设置其强引用中记录的id
    bool resolve_proxy(RoadGeometryManager* mgr) {
        bool bf = true;
        for (auto& e : _proxys) {
            if (e.id()) {
                auto p = e.id()->resolve_id(mgr);
                if (p) {
                    e = p->id();
                }
                else {
                    LOG_BEGIN(DEBUG) << "resolve_proxy() failed for " << e->to_string(); LOG_END;
                    bf = false;
                }
            }
            else {
                LOG_BEGIN(ERROR) << "resolve_proxy() failed for empty id"; LOG_END;
                bf = false;
            }
        }
        return bf;
    };

    //清除本数组中全部引用关系的更新标记
    void clear_changed() {
        for (auto& e : _proxys) {
            if (e) {
                auto id = std::const_pointer_cast<FeatureIDProxy>(e.id());
                id->clear_changed();
            }
        }
    };

    //查询并返回本数组中特定id的元素
    FeatureReference<Proxy> find(const FeatureIDProxy& id) const {
        for (auto& e : _proxys) {
            if (e->is_equal(id)) {
                return e;
            }
        }
        return FeatureReference<Proxy>();
    };

    //交换两个数组的内容，可以用于释放内存/重设parent
    void swap(FeatureReferenceVector<Proxy>& v) {
        std::swap(v._parent, _parent);
        v._proxys.swap(_proxys);
    };

    //从pb的repeated字段中生成本数组的引用信息
    bool from_message(const google::protobuf::RepeatedPtrField<RoadPB::FeatureID>& ids) {
        clear();
        reserve(ids.size());
        for (int i = 0; i < ids.size(); ++i) {
            auto e = add();
            if (e != nullptr) {
                auto id = std::const_pointer_cast<FeatureIDProxy>(e->id());
                id->from_message(&ids.Get(i));
            }
            else {
                return false;
            }
        }
        return true;
    };
    //把本数组中记录的引用关系写入到特定的pb的repeated字段中
    bool to_message(google::protobuf::RepeatedPtrField<RoadPB::FeatureID>* ids) const {
        ids->Reserve((int)size());
        ids->Clear();
        for (auto& e : _proxys) {
            if (e) {
                e->to_message(ids->Add());
            }
        }
        return true;
    };

    typedef typename std::vector<FeatureReference<Proxy> >::iterator elem_iterator;
    typedef typename std::vector<FeatureReference<Proxy> >::reverse_iterator elem_reverse_iterator;
    typedef typename std::vector<FeatureReference<Proxy> >::const_iterator elem_const_iterator;
    typedef typename std::vector<FeatureReference<Proxy> >::const_reverse_iterator elem_const_reverse_iterator;
    //透传内部std::vector的iterator，提供与std::vector相同的接口
    elem_iterator begin() { return _proxys.begin(); };
    elem_const_iterator begin() const { return _proxys.begin(); };
    elem_iterator end() { return _proxys.end(); };
    elem_const_iterator end() const { return _proxys.end(); };
    elem_reverse_iterator rbegin() { return _proxys.rbegin(); };
    elem_const_reverse_iterator rbegin() const { return _proxys.rbegin(); };
    elem_reverse_iterator rend() { return _proxys.rend(); };
    elem_const_reverse_iterator rend() const { return _proxys.rend(); };
    elem_iterator erase(elem_iterator it) {
        if (_parent) {
            if (!_parent->is_editable()) {
                return _proxys.end();
            }
            _parent->mark_changed();
        }
        return _proxys.erase(it); 
    };
    elem_reverse_iterator erase(elem_reverse_iterator it) {
        if (_parent) {
            if (!_parent->is_editable()) {
                return _proxys.rend();
            }
            _parent->mark_changed();
        }
        return _proxys.erase(it);
    };
    elem_iterator find(const std::shared_ptr<const FeatureIDProxy>& id) {
        for (auto it = _proxys.begin(); it != _proxys.end(); ++it) {
            if (it->id() == id || it->id()->is_equal(*id)) {
                return it;
            }
        }
        return _proxys.end();
    };
    elem_const_iterator find(const std::shared_ptr<const FeatureIDProxy>& id) const {
        for (auto it = _proxys.begin(); it != _proxys.end(); ++it) {
            if (it->id() == id || it->id()->is_equal(*id)) {
                return it;
            }
        }
        return _proxys.end();
    };
    void sorted_copy(FeatureReferenceVector<Proxy>& org) {
        _proxys.clear();
        _proxys.reserve(org._proxys.size());
        for (size_t i = 0; i < org._proxys.size(); ++i) {
            bool bf = false;
            for (size_t j = 0; j < i; ++j) {
                if (_proxys[j].compare(org._proxys[i]) > 0) {
                    insert(j, org._proxys[i]);
                    bf = true;
                    break;
                }
            }
            if (!bf) {
                push_back(org._proxys[i]);
            }
        }
    };

protected:
    std::vector<FeatureReference<Proxy>> _proxys;
    FeatureProxyBase* _parent;
};

template <typename T>
inline bool members_equal(const FeatureMemberVector<T>& ls, const FeatureMemberVector<T>& rs) {
    if (ls.size() != rs.size()) {
        return false;
    }
    for (size_t i = 0; i < ls.size(); ++i) {
        if (ls[i] != rs[i]) {
            return false;
        }
    }
    return true;
};
template <typename T>
inline int members_compare(const FeatureMemberVector<T>& ls, const FeatureMemberVector<T>& rs) {
    if (ls.size() != rs.size()) {
        return ls.size() < rs.size() ? -1 : 1;
    }
    for (size_t i = 0; i < ls.size(); ++i) {
        int ret = ls[i].compare(rs[i]);
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
};

template<typename T>
inline bool members_assign(FeatureMemberVector<T>& ls, const FeatureMemberVector<T>& rs) {
    ls.clear();
    ls.reserve(rs.size());
    for (size_t i = 0; i < rs.size(); ++i) {
        (*ls.add()) = rs[i].id();
    }
    return true;
};

template <typename T>
inline bool three_way_merge_members(MergeConflictPolicy policy, const FeatureMemberVector<T>& base,
                             const FeatureMemberVector<T>& remote, FeatureMemberVector<T>& local) {
    if (members_equal(local, remote) || members_equal(base, remote)) {
        return false;
    }
    if (members_equal(base, local)) {
        members_assign(local, remote);
        return false;
    }
//     if (policy == MERGE_CONFLICT_REMOTE) {
//         members_assign(local, remote);
//     }
    bool bconflict = false;
    int local_ind = 0;
    std::vector<int> local2base(local.size());
    int remote_ind = 0;
    std::vector<int> remote2base(remote.size());
    for (int bi = 0; bi < (int)base.size(); ++bi) {
        auto& b = base[bi];        
        for (int li = local_ind; li < (int)local.size(); ++li) {
            auto& l = local[li];
            if (b->is_equal(*l.id())) {
                for (int i = local_ind; i < li; ++i) {
                    local2base[i] = -bi - 1;
                }
                local2base[li] = (l->version() == b->version() ? bi : -bi - 1);
                local_ind = li + 1;
                break;
            }
        }
        for (int ri = local_ind; ri < (int)remote.size(); ++ri) {
            auto& r = remote[ri];
            if (r->is_equal(*r.id())) {
                for (int i = remote_ind; i < ri; ++i) {
                    remote2base[i] = -bi - 1;
                }
                remote2base[ri] = (r->version() == b->version() ? bi : -bi - 1);
                remote_ind = ri + 1;
                break;
            }
        }
    }
        
    int cur_local = 0;
    for (int i = local_ind; i < (int)local.size(); ++i) {
        local2base[i] = -(int)base.size() - 1;
    }
    local_ind = 0;
    for (int i = remote_ind; i < (int)remote.size(); ++i) {
        remote2base[i] = -(int)base.size() - 1;
    }
    remote_ind = 0;
    for (int bi = 0; bi < (int)base.size() + 1; ++bi) {
        int li = local_ind;
        for (; li < (int)local2base.size(); ++li) {
            if (local2base[li] > bi || -local2base[li] - 1 > bi) {
                break;
            }
        }
        int ri = remote_ind;
        for (; ri < (int)remote2base.size(); ++ri) {
            if (remote2base[ri] > bi || -remote2base[ri] - 1 > bi) {
                break;
            }
        }
        if (local_ind == li && remote_ind == ri) {
            continue;
        }
        if (local_ind < li && remote_ind == ri) {
            if (local_ind + 1 == li && local2base[local_ind] >= 0) {
                local.erase(cur_local);
                local_ind = li;
                continue;
            }
            if (policy == MERGE_CONFLICT_LOCAL) {
                cur_local += (li - local_ind);
                local_ind = li;
                bconflict = true;
                continue;
            }
            else {
                for (int i = local_ind; i < li; ++i) {
                    local.erase(cur_local);
                }
                local_ind = li;
                bconflict = true;
                continue;
            }
        }
        else if (local_ind == li && remote_ind < ri) {
            if (remote_ind + 1 == ri && remote2base[remote_ind] >= 0) {
                remote_ind = ri;
                continue;
            }
            if (policy == MERGE_CONFLICT_REMOTE) {
                for (int i = remote_ind; i < ri; ++i) {
                    local.insert(cur_local, remote[i]);
                    cur_local++;
                }
                remote_ind = ri;
                bconflict = true;
                continue;
            }
            else {
                bconflict = true;
                remote_ind = ri;
                continue;
            }
        }
        else {
            if (local_ind + 1 == li && remote_ind + 1 == ri) {
                if (local2base[local_ind] >= 0 && remote2base[remote_ind] >= 0) {
                    local_ind = li;
                    cur_local++;
                    remote_ind = ri;
                    continue;
                }
                if (local2base[local_ind] >= 0) {
                    local[cur_local] = remote[remote_ind];
                    local_ind = li;
                    cur_local++;
                    remote_ind = ri;
                    continue;
                }
                else if (remote2base[remote_ind] >= 0) {
                    local_ind = li;
                    cur_local++;
                    remote_ind = ri;
                    continue;
                }
                bconflict = true;
                if (policy == MERGE_CONFLICT_REMOTE) {
                    local[cur_local] = remote[remote_ind];
                }
                local_ind = li;
                cur_local++;
                remote_ind = ri;
                continue;                
            }
            if (local_ind + 1 == li) {
                if (local2base[local_ind] >= 0) {
                    for (int i = remote_ind; i + 1 < ri; ++i) {
                        local.insert(cur_local, remote[i]);
                        cur_local++;
                    }
                    local[cur_local] = remote[ri - 1];
                    cur_local++;
                    local_ind = li;
                    remote_ind = ri;
                    continue;
                }
                else {
                    if (policy == MERGE_CONFLICT_REMOTE) {
                        for (int i = remote_ind; i + 1 < ri; ++i) {
                            local.insert(cur_local, remote[i]);
                            cur_local++;
                        }
                        local[cur_local] = remote[ri - 1];
                    }
                    bconflict = true;
                    local_ind = li;
                    cur_local++;
                    remote_ind = ri;
                    continue;
                }
            }
            else if (remote_ind + 1 == ri) {
                if (remote2base[remote_ind] >= 0) {
                    cur_local += li - local_ind;
                    local_ind = li;
                    remote_ind = ri;
                    continue;
                }
                else {
                    if (policy == MERGE_CONFLICT_REMOTE) {
                        for (int i = local_ind; i + 1 < li; ++i) {
                            local.erase(cur_local);
                        }
                        local[cur_local] = remote[remote_ind];
                        cur_local++;
                        local_ind = li;
                    }
                    else {
                        cur_local += li - local_ind;
                        local_ind = li;
                    }
                    bconflict = true;
                    remote_ind = ri;
                    continue;
                }
            }
            else {
                if (li - local_ind == ri - remote_ind) {
                    bool beq = true;
                    for (int i = local_ind; i < li; ++i) {
                        auto& l = local[cur_local + i - local_ind];
                        auto& r = remote[i - local_ind + remote_ind];
                        if (l->version() != r->version() || !l->is_equal(*r.id())) {
                            beq = false;
                            break;
                        }
                    }
                    if (beq) {
                        cur_local += li - local_ind;
                        local_ind = li;
                        remote_ind = ri;
                        continue;
                    }
                }
                bconflict = true;
                if (policy == MERGE_CONFLICT_REMOTE) {
                    for (int i = local_ind; i + 1 < li; ++i) {
                        local.erase(cur_local);
                    }
                    for (int i = remote_ind; i + 1 < ri; ++i) {
                        local.insert(cur_local, remote[i]);
                        ++cur_local;
                    }
                    local[cur_local] = remote[ri - 1];
                    ++cur_local;                    
                }
                else {
                    cur_local += li - local_ind;
                }
                local_ind = li;
                remote_ind = ri;
                continue;
            }
        }
    }
    return bconflict;
};

template <typename T>
inline bool refs_equal(const FeatureReferenceVector<T>& ls, const FeatureReferenceVector<T>& rs) {
    if (ls.size() != rs.size()) {
        return false;
    }
    for (size_t i = 0; i < ls.size(); ++i) {
        if (!ls[i]->is_equal(*(rs[i].id()))) {
            return false;
        }
    }
    return true;
};
template <typename T>
inline int refs_compare(const FeatureReferenceVector<T>& ls, const FeatureReferenceVector<T>& rs) {
    if (ls.size() != rs.size()) {
        return ls.size() < rs.size() ? -1 : 1;
    }
    for (size_t i = 0; i < ls.size(); ++i) {
        int ret = ls[i].compare(rs[i]);
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
};

template<typename T>
inline bool refs_assign(FeatureReferenceVector<T>& ls, const FeatureReferenceVector<T>& rs) {
    ls.clear();
    ls.reserve(rs.size());
    for (size_t i = 0; i < rs.size(); ++i) {
        (*ls.add()) = rs[i].id();
    }
    return true;
};

template <typename T>
inline bool three_way_merge_refs(MergeConflictPolicy policy, const FeatureReferenceVector<T>& base,
                          const FeatureReferenceVector<T>& remote, FeatureReferenceVector<T>& local) {
    if (refs_equal(local, remote) || refs_equal(base, remote)) {
        return false;
    }
    if (refs_equal(base, local)) {
        refs_assign(local, remote);
        return false;
    }
//     if (policy == MERGE_CONFLICT_REMOTE) {
//         refs_assign(local, remote);
//     }
    bool bconflict = false;
    int local_ind = 0;
    std::vector<int> local2base(local.size());
    int remote_ind = 0;
    std::vector<int> remote2base(remote.size());
    for (int bi = 0; bi < (int)base.size(); ++bi) {
        auto& b = base[bi];
        for (int li = local_ind; li < (int)local.size(); ++li) {
            auto& l = local[li];
            if (b->is_equal(*l.id())) {
                for (int i = local_ind; i < li; ++i) {
                    local2base[i] = -bi - 1;
                }
                local2base[li] = bi;
                local_ind = li + 1;
                break;
            }
        }
        for (int ri = local_ind; ri < (int)remote.size(); ++ri) {
            auto& r = remote[ri];
            if (r->is_equal(*r.id())) {
                for (int i = remote_ind; i < ri; ++i) {
                    remote2base[i] = -bi - 1;
                }
                remote2base[ri] = bi;
                remote_ind = ri + 1;
                break;
            }
        }
    }

    int cur_local = 0;
    for (int i = local_ind; i < (int)local.size(); ++i) {
        local2base[i] = -(int)base.size() - 1;
    }
    local_ind = 0;
    for (int i = remote_ind; i < (int)remote.size(); ++i) {
        remote2base[i] = -(int)base.size() - 1;
    }
    remote_ind = 0;
    for (int bi = 0; bi < (int)base.size() + 1; ++bi) {
        int li = local_ind;
        for (; li < (int)local2base.size(); ++li) {
            if (local2base[li] > bi || -local2base[li] - 1 > bi) {
                break;
            }
        }
        int ri = remote_ind;
        for (; ri < (int)remote2base.size(); ++ri) {
            if (remote2base[ri] > bi || -remote2base[ri] - 1 > bi) {
                break;
            }
        }
        if (local_ind == li && remote_ind == ri) {
            continue;
        }
        if (local_ind < li && remote_ind == ri) {
            if (local_ind + 1 == li && local2base[local_ind] >= 0) {
                local.erase(cur_local);
                local_ind = li;
                continue;
            }
            if (policy == MERGE_CONFLICT_LOCAL) {
                cur_local += (li - local_ind);
                local_ind = li;
                bconflict = true;
                continue;
            }
            else {
                for (int i = local_ind; i < li; ++i) {
                    local.erase(cur_local);
                }
                local_ind = li;
                bconflict = true;
                continue;
            }
        }
        else if (local_ind == li && remote_ind < ri) {
            if (remote_ind + 1 == ri && remote2base[remote_ind] >= 0) {
                remote_ind = ri;
                continue;
            }
            if (policy == MERGE_CONFLICT_REMOTE) {
                for (int i = remote_ind; i < ri; ++i) {
                    local.insert(cur_local, remote[i]);
                    cur_local++;
                }
                remote_ind = ri;
                bconflict = true;
                continue;
            }
            else {
                bconflict = true;
                remote_ind = ri;
                continue;
            }
        }
        else {
            if (local_ind + 1 == li && remote_ind + 1 == ri) {
                if (local2base[local_ind] >= 0 && remote2base[remote_ind] >= 0) {
                    local_ind = li;
                    cur_local++;
                    remote_ind = ri;
                    continue;
                }
                if (local2base[local_ind] >= 0) {
                    local[cur_local] = remote[remote_ind];
                    local_ind = li;
                    cur_local++;
                    remote_ind = ri;
                    continue;
                }
                else if (remote2base[remote_ind] >= 0) {
                    local_ind = li;
                    cur_local++;
                    remote_ind = ri;
                    continue;
                }
                bconflict = true;
                if (policy == MERGE_CONFLICT_REMOTE) {
                    local[cur_local] = remote[remote_ind];
                }
                local_ind = li;
                cur_local++;
                remote_ind = ri;
                continue;
            }
            if (local_ind + 1 == li) {
                if (local2base[local_ind] >= 0) {
                    for (int i = remote_ind; i + 1 < ri; ++i) {
                        local.insert(cur_local, remote[i]);
                        cur_local++;
                    }
                    local[cur_local] = remote[ri - 1];
                    cur_local++;
                    local_ind = li;
                    remote_ind = ri;
                    continue;
                }
                else {
                    if (policy == MERGE_CONFLICT_REMOTE) {
                        for (int i = remote_ind; i + 1 < ri; ++i) {
                            local.insert(cur_local, remote[i]);
                            cur_local++;
                        }
                        local[cur_local] = remote[ri - 1];
                    }
                    bconflict = true;
                    local_ind = li;
                    cur_local++;
                    remote_ind = ri;
                    continue;
                }
            }
            else if (remote_ind + 1 == ri) {
                if (remote2base[remote_ind] >= 0) {
                    cur_local += li - local_ind;
                    local_ind = li;
                    remote_ind = ri;
                    continue;
                }
                else {
                    if (policy == MERGE_CONFLICT_REMOTE) {
                        for (int i = local_ind; i + 1 < li; ++i) {
                            local.erase(cur_local);
                        }
                        local[cur_local] = remote[remote_ind];
                        cur_local++;
                        local_ind = li;
                    }
                    else {
                        cur_local += li - local_ind;
                        local_ind = li;
                    }
                    bconflict = true;
                    remote_ind = ri;
                    continue;
                }
            }
            else {
                if (li - local_ind == ri - remote_ind) {
                    bool beq = true;
                    for (int i = local_ind; i < li; ++i) {
                        auto& l = local[cur_local + 1 - local_ind];
                        auto& r = remote[i - local_ind + remote_ind];
                        if (!l->is_equal(*r.id())) {
                            beq = false;
                            break;
                        }
                    }
                    if (beq) {
                        cur_local += li - local_ind;
                        local_ind = li;
                        remote_ind = ri;
                        continue;
                    }
                }
                bconflict = true;
                if (policy == MERGE_CONFLICT_REMOTE) {
                    for (int i = local_ind; i + 1 < li; ++i) {
                        local.erase(cur_local);
                    }
                    for (int i = remote_ind; i + 1 < ri; ++i) {
                        local.insert(cur_local, remote[i]);
                        ++cur_local;
                    }
                    local[cur_local] = remote[ri - 1];
                    ++cur_local;
                }
                else {
                    cur_local += li - local_ind;
                }
                local_ind = li;
                remote_ind = ri;
                continue;
            }
        }
    }
    return bconflict;
};

template <typename T>
inline bool three_way_merge_unordered(MergeConflictPolicy policy, const FeatureReferenceVector<T>& base_sorted,
    const FeatureReferenceVector<T>& remote_sorted, FeatureReferenceVector<T>& local_sorted, FeatureReferenceVector<T>& local) {
    if (refs_equal(local_sorted, remote_sorted) || refs_equal(base_sorted, remote_sorted)) {
        return false;
    }
    if (refs_equal(base_sorted, local_sorted)) {
        refs_assign(local, remote_sorted);
        return false;
    }
//     if (policy == MERGE_CONFLICT_REMOTE) {
//         refs_assign(local, remote);
//     }
    FeatureReferenceVector<T> merged(nullptr);
    merged.reserve(std::max(local_sorted.size(), remote_sorted.size()));
    auto bit = base_sorted.begin();
    auto lit = local_sorted.begin();
    auto rit = remote_sorted.begin();
    bool bconflict = false;
    while (bit != base_sorted.end() || lit != base_sorted.end() || rit != remote_sorted.end()) {
        if (bit == base_sorted.end()) {
            if (lit == local_sorted.end()) {
                merged.push_back(*rit);
                ++rit;
                continue;
            }
            if (rit == remote_sorted.end()) {
                merged.push_back(*lit);
                ++lit;
                continue;
            }
            if (*lit == *rit) {
                merged.push_back(*lit);
                ++lit;
                ++rit;
                continue;
            }
            else if (*lit < *rit) {
                merged.push_back(*lit);
                ++lit;
                continue;
            }
            else {
                merged.push_back(*rit);
                ++rit;
                continue;
            }
        }
        if (lit == local_sorted.end() && rit == remote_sorted.end()) {
            ++bit;
            continue;
        }
        else if (lit == local_sorted.end()) {
            if (*bit == *rit) {
                ++bit;
                ++rit;
                continue;
            }
            if (*rit < *bit) {
                merged.push_back(*rit);
                ++rit;
                continue;
            }
            else {
                ++bit;
                continue;
            }
        }
        else if (rit == remote_sorted.end()) {
            if (*bit == *lit) {
                ++bit;
                ++lit;
                continue;
            }
            if (*lit < *bit) {
                merged.push_back(*lit);
                ++lit;
                continue;
            }
            else {
                ++bit;
                continue;
            }
        }
        else {
            if (*bit == *lit && *bit == *rit) {
                merged.push_back(*lit);
                ++bit;
                ++lit;
                ++rit;
                continue;
            }
            else if (*bit == *lit) {
                if (*rit < *bit) {
                    merged.push_back(*rit);
                    ++rit;
                    continue;
                }
                else {
                    ++bit;
                    ++lit;
                    continue;
                }
            }
            else if (*bit == *rit) {
                if (*lit < *bit) {
                    merged.push_back(*lit);
                    ++lit;
                    continue;
                }
                else {
                    ++bit;
                    ++rit;
                    continue;
                }
            }
            else {
                if (*lit == *rit) {
                    while (*bit < *lit) {
                        ++bit;
                        if (bit == base_sorted.end()) {
                            break;
                        }
                    }
                    merged.push_back(*lit);
                    ++lit;
                    ++rit;
                    continue;
                }
                if (*lit < *rit) {
                    if (*bit < *lit) {
                        ++bit;
                        continue;
                    }
                    else {
                        bconflict = true;
                        merged.push_back(*lit);
                        ++lit;
                        continue;
                    }
                }
                else {
                    if (*bit < *rit) {
                        ++bit;
                        continue;
                    }
                    else {
                        bconflict = true;
                        merged.push_back(*rit);
                        ++rit;
                        continue;
                    }
                }
            }
        }
    }
    if (refs_compare(local_sorted, merged) != 0) {
        local.clear();
        for (auto& e : merged) {
            local.push_back(e);
        }
    }
    return bconflict;
};

template <class ProxyType>
void FeatureWithIDProxyBase::mark_changed_and_copy() {
    if (_change_count < -1) {
        return;
    }
    copy_on_write<ProxyType>();
    if (_ref_record) {
        for (auto o : _ref_record->_owner_elems) {
            if (o != nullptr && o->container() != nullptr) {
                auto c = const_cast<FeatureWithIDProxyBase*>(o->container());
                c->mark_changed();
            }
        }
    }
    FeatureProxyBase::mark_changed();
};

}; // data_access_engine


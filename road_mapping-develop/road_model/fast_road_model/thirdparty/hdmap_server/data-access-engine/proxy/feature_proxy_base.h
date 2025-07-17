#pragma once
#include <stdint.h>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include "google/protobuf/repeated_field.h"
#include "public/optional.hpp"
#include "public/platform.h"

namespace data_access_engine {

enum MergeConflictPolicy {
    MERGE_BEST = 0,
    MERGE_ALL_LOCAL = 1,
    MERGE_ALL_REMOTE = 2,
    MERGE_CONFLICT_LOCAL = 3,
    MERGE_CONFLICT_REMOTE = 4,
    MERGE_COMPARE_ONLY = 5
};

class RoadGeometryManager;

class FeatureProxyBase;
template <class T> 
class FeatureProxyCreator {
public:
    virtual T* create() {
        return new T();
    };
    virtual T* create(FeatureProxyBase* p) {
        return new T(p);
    };
};

class FeatureWithIDProxyBase;

//所有对应pb中message的proxy类型的基类，提供各种公共接口需函数定义，以及维护proxy间的包含关系和变更状态
class FeatureProxyBase : public std::enable_shared_from_this<FeatureProxyBase>
{
public:
    //默认构造函数，构造一个孤立的顶层Proxy
    FeatureProxyBase() : _change_count(0), _valid_cache_count(-2), _valid_cache(true), _parent(nullptr) {};
    //构造一个proxy，它作为parent的成员存在
    FeatureProxyBase(FeatureProxyBase* parent) : _change_count(0), _valid_cache_count(-2), _valid_cache(true), _parent(nullptr) {
        set_parent(parent);
    };
    virtual ~FeatureProxyBase() {
        set_parent(nullptr);
        for (auto c : _children) {
            if (c->_parent == this) {
                c->_parent = nullptr;
            }
        }
        _children.clear();
    };

    //屏蔽拷贝构造函数，避免使用方无意中通过只读引用复制并获得一个可修改的新对象
    FeatureProxyBase(const FeatureProxyBase&) = delete;
    FeatureProxyBase& operator=(const FeatureProxyBase&) = delete;

public:
    //从pb中对应的message生成对应的proxy对象，用于数据反序列化
    virtual bool from_message(const google::protobuf::Message* msg) = 0;
    //把proxy信息刷到对应pb的message中，用于数据序列化
    virtual bool to_message(google::protobuf::Message* msg) const = 0;
    
    //本proxy对象对应的元素类型，只在我们路网要素的子类上有重载返回FeatureID::FeatType枚举中的类型，其余都返回0
    virtual int feature_id_type() const { return 0; };
    //判断本proxy对象是否正确，本方法判定为不正确的元素我们就认为是脏数据
    virtual bool is_valid() const = 0;
    virtual bool cached_is_valid() const;
    //尝试修复本对象中可能存在错误数据，例如去除无效引用、删除重复元素等,返回是否发生更改
    virtual bool correct_content(RoadGeometryManager* mgr) { return false; };
    //尝试修复本元素所经过的tile信息，返回是否发生变更
    virtual bool correct_tile_refs(RoadGeometryManager* mgr) { return false; };
    //重设本对象内部通过id引用的其它对象的引用关系，把相应的id和其引用的proxy实际关联起来
    virtual bool remake_proxy(RoadGeometryManager* mgr) { return true; };

    //返回此proxy是否可以进行编辑修改，子元素的编辑权限是由其外层对象决定
    virtual bool is_editable() const;
    //判断本proxy是否发生变更，每次修改操作会把内部变更计数+1
    virtual bool is_changed() const { return _change_count > 0; };
    //标记本proxy已经发生变更，并且会把变更状态传递给其外层父proxy
    virtual void mark_changed();
    //清楚本proxy的变更计数信息，注意此方法默认实现不会清除本proxy的下级值proxy的变更计数
    virtual void clear_changed() { _change_count = 0; };

    //返回本proxy的外层父对象
    const FeatureProxyBase* parent() const { return _parent; };
    //重设本对象的外层父对象，注意：使用方一般不应该调用此方法进行设置
    virtual void set_parent(FeatureProxyBase* parent) { 
        if (parent == _parent) {
            return;
        }
        if (_parent != nullptr) {
            _parent->_children.erase(this);
        }
        if (parent != nullptr) {
            parent->_children.insert(this);
        }
        _parent = parent;
    };
    //包含此对象的外层路网元素，由于message可能多层嵌套，词方法会通过parent()链追溯到啊路网元素对象上
    virtual const FeatureWithIDProxyBase* container() const;

    //注册/获取特定proxy类型的自定义子类型的构造器，通过函数内静态变量来保存各种特定类型的构造器
    template <class ProxyType>
    static FeatureProxyCreator<ProxyType>* register_creator(FeatureProxyCreator<ProxyType>* c) {
        static std::unique_ptr<FeatureProxyCreator<ProxyType>> s_creator(new FeatureProxyCreator<ProxyType>);
        if (c) {
            auto ret = s_creator.release();
            s_creator.reset(c);
            return ret;
        }
        return s_creator.get();
    };

    //创建特定类型proxy对象，无外层对象，如果注册过相应的构造器，可以直接构造特定的子类对象
    template <class ProxyType>
    static ProxyType* create_proxy() {
        auto c = register_creator<ProxyType>(nullptr);
        return c->create();
    };
    //创建特定类型proxy对象，构造时指定外层对象，如果注册过相应构造器，可以直接构造出特定的子类对象
    template <class ProxyType>
    static ProxyType* create_proxy(FeatureProxyBase* parent) {
        auto c = register_creator<ProxyType>(nullptr);
        return c->create(parent);
    };

    //通过pb的序列化反序列化的方式深拷贝当前的proxy，需要指定本proxy的具体类型
    template <class ProxyType>
    std::shared_ptr<ProxyType> dump() const {
        typename ProxyType::message_type msg;
        if (!to_message(&msg)) {
            return nullptr;
        }
        std::shared_ptr<ProxyType> ptr(FeatureProxyBase::create_proxy<ProxyType>());
        if (ptr->from_message(&msg)) {
            return ptr;
        }
        return nullptr;
    };

    //检查本元素转换为pb的message后是否能够正常序列化+反序列化(required字段是否完整)
    template <class ProxyType>
    bool is_message_valid() const {
#ifndef NO_MASSAGE_VALID_CHECK
        typename ProxyType::message_type msg;
        if (!to_message(&msg)) {
            return false;
        }
        std::stringstream ss;
        if (!msg.SerializeToOstream(&ss)) {
            return false;
        }
        typename ProxyType::message_type tm;
        if (!tm.ParseFromIstream(&ss)) {
            return false;
        }
#endif
        return true;
    };

    template<typename T>
    inline bool three_way_merge(MergeConflictPolicy policy, const T& base, const T& remote, T& local);
    
    template <typename T>
    inline bool three_way_merge_vector(MergeConflictPolicy policy, const std::vector<T>& base,
        const std::vector<T>& remote, std::vector<T>& local);

    virtual bool merge_proxy(MergeConflictPolicy /*policy*/, RoadGeometryManager* /*mgr*/,
                                const std::shared_ptr<const FeatureProxyBase>& /*base*/,
                                const std::shared_ptr<const FeatureProxyBase>& /*remote*/) {        
        return true;
    };

protected:
    mutable FeatureProxyBase* _parent;
    std::unordered_set<FeatureProxyBase*> _children;
    int _change_count;
    mutable int _valid_cache_count;
    mutable bool _valid_cache;
};

template<typename T>
inline bool value_equal(const T& l, const T& r) {
    return l == r;
};
template<>
inline bool value_equal<float>(const float& l, const float& r) {
    return l - r > -FLOAT_TOLERANCE && l - r < FLOAT_TOLERANCE;
};
template<>
inline bool value_equal<double>(const double& l, const double& r) {
    return l - r > -FLOAT_TOLERANCE && l - r < FLOAT_TOLERANCE;
};

template<typename T>
inline int value_compare(const T& l, const T& r) {
    return (l < r ? -1 : (r < l ? 1 : 0));
};
template<>
inline int value_compare<float>(const float& l, const float& r) {
    if (value_equal(l, r)) {
        return 0;
    }
    if (l < r) {
        return -1;
    }
    return 1;
};
template<>
inline int value_compare<double>(const double& l, const double& r) {
    if (value_equal(l, r)) {
        return 0;
    }
    if (l < r) {
        return -1;
    }
    return 1;
};

template <typename T>
inline int vector_compare(const std::vector<T>& ls, const std::vector<T>& rs) {
    if (ls.size() != rs.size()) {
        return ls.size() < rs.size() ? -1 : 1;
    }
    for (size_t i = 0; i < ls.size(); ++i) {
        int ret = value_compare(ls[i], rs[i]);
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
};
template <typename T>
inline bool vector_equal(const std::vector<T>& ls, const std::vector<T>& rs) {
    return (vector_compare(ls, rs) == 0);
};

template <typename T>
inline bool proxy_equal(const std::shared_ptr<const T>& ls, const std::shared_ptr<const T>& rs) {
    if (!ls && !rs) {
        return true;
    }
    if (!ls || !rs) {
        return false;
    }
    return *ls == *rs;
};

template<typename T>
    inline bool FeatureProxyBase::three_way_merge(MergeConflictPolicy policy, const T& base, const T& remote, T& local) {
        if (value_equal(local, remote) || value_equal(base, remote)) {
            return false;
        }
        if (value_equal(base, local)) {
            mark_changed();
            local = remote;
            return false;
        }
        if (policy == MERGE_CONFLICT_REMOTE) {
            mark_changed();
            local = remote;
        }
        return true;
    };
    
    template <typename T>
    inline bool FeatureProxyBase::three_way_merge_vector(MergeConflictPolicy policy, const std::vector<T>& base,
        const std::vector<T>& remote, std::vector<T>& local) {
        if (vector_equal(local, remote) || vector_equal(base, remote)) {
            return false;
        }
        if (vector_equal(base, local)) {
            mark_changed();
            local = remote;
            return false;
        }

        if (policy == MERGE_CONFLICT_REMOTE) {
            mark_changed();
            local = remote;
        }
        return true;
    };

template <typename T>
inline bool three_way_merge_proxy(MergeConflictPolicy policy, 
                    RoadGeometryManager* mgr, const std::shared_ptr<const T>& base,
                    const std::shared_ptr<const T>& remote, std::shared_ptr<T>& local) {
    if (proxy_equal<T>(local, remote) || proxy_equal<T>(base, remote)) {
        return false;
    }
    if (proxy_equal<T>(base, local)) {        
        if (!remote) {
            local.reset();
        }
        else {
            local = remote->template dump<T>();
        }
        return false;
    }
    if (!remote) {
        if (policy == MERGE_CONFLICT_REMOTE) {
            local.reset();
        }
        return true;
    }
    if (!local) {
        if (policy == MERGE_CONFLICT_REMOTE) {
            local = remote->template dump<T>();
        }
        return true;
    }
    return local->merge_proxy(policy, mgr, base, remote);
};

//封装SharedProxyVector<ProxyType>使用的iterator，实现读写接口分离
//const时其元素类型为const std::shared_ptr<const ProxyType>
//非const时其元素类型为const std::shared_ptr<ProxyType>
template <class ProxyType>
class ProxyPtrIterator {
public:
    typedef ProxyPtrIterator<ProxyType> ptr_iterator;

    // Let the compiler know that these are type names, so we don't have to
    // write "typename" in front of them everywhere.
    typedef const std::shared_ptr<ProxyType>& ptr_reference;
    typedef const std::shared_ptr<ProxyType>* ptr_pointer;
    typedef const std::shared_ptr<const ProxyType>& ptr_const_reference;
    typedef const std::shared_ptr<const ProxyType>* ptr_const_pointer;
    typedef ptrdiff_t ptr_difference_type;
    typedef const std::shared_ptr<ProxyType> ptr_value;

    using iterator_category = std::random_access_iterator_tag;
    using difference_type = ptr_difference_type;
    using value_type = ptr_value;
    using pointer = ptr_pointer;
    using reference = ptr_reference;

    ProxyPtrIterator() : it_(nullptr) {}
    explicit ProxyPtrIterator(std::shared_ptr<ProxyType>* it) : it_(it) {}

    template<typename OtherFeature>
    ProxyPtrIterator(const ProxyPtrIterator<OtherFeature>& other)
        : it_(other.it_) {}

    // dereferenceable
    ptr_const_reference operator*() const { return *(reinterpret_cast<std::shared_ptr<const ProxyType>*>(it_)); }
    ptr_const_pointer operator->() const { return (reinterpret_cast<std::shared_ptr<const ProxyType>*>(it_)); }
    ptr_reference operator*() { return *(reinterpret_cast<std::shared_ptr<ProxyType>*>(it_)); }
    ptr_pointer operator->() { return (reinterpret_cast<std::shared_ptr<ProxyType>*>(it_)); }

    // {inc,dec}rementable
    ptr_iterator& operator++() { ++it_; return *this; }
    ptr_iterator  operator++(int) { return ptr_iterator(it_++); }
    ptr_iterator& operator--() { --it_; return *this; }
    ptr_iterator  operator--(int) { return ptr_iterator(it_--); }

    // equality_comparable
    bool operator==(const ptr_iterator& x) const { return it_ == x.it_; }
    bool operator!=(const ptr_iterator& x) const { return it_ != x.it_; }

    // less_than_comparable
    bool operator<(const ptr_iterator& x) const { return it_ < x.it_; }
    bool operator<=(const ptr_iterator& x) const { return it_ <= x.it_; }
    bool operator>(const ptr_iterator& x) const { return it_ > x.it_; }
    bool operator>=(const ptr_iterator& x) const { return it_ >= x.it_; }

    // addable, subtractable
    ptr_iterator& operator+=(ptr_difference_type d) {
        it_ += d;
        return *this;
    }
    friend ptr_iterator operator+(ptr_iterator it, ptr_difference_type d) {
        it += d;
        return it;
    }
    friend ptr_iterator operator+(ptr_difference_type d, ptr_iterator it) {
        it += d;
        return it;
    }
    ptr_iterator& operator-=(ptr_difference_type d) {
        it_ -= d;
        return *this;
    }
    friend ptr_iterator operator-(ptr_iterator it, ptr_difference_type d) {
        it -= d;
        return it;
    }

    // indexable
    ptr_const_reference& operator[](ptr_difference_type d) const { return *(*this + d); }
    ptr_reference& operator[](ptr_difference_type d) { return *(*this + d); }
    // random access iterator
    ptr_difference_type operator-(const ptr_iterator& x) const { return it_ - x.it_; }

private:
    template<typename OtherFeature>
    friend class ProxyPtrIterator;

    // The internal iterator.
    std::shared_ptr<ProxyType>* it_;
};

//封装一个元素中proxy子元素的数组，对应pb中的repeated XXXMessage
//维护外层父对象信息，并传递变更等
template <class T>
class SharedProxyVector {
public:
    //构造一个vector，指定它的外层对象，这个对象会传递给放入本vector中的proxy以维护对象关系链
    SharedProxyVector(FeatureProxyBase* p) : _parent(p) {};
    //构造一个作为临时容器的vector，放入此对象的proxy不会变更对象间的引用关系
    SharedProxyVector() : _parent(nullptr) {};
    ~SharedProxyVector() { clear(); };

    //屏蔽默认拷贝构造函数，避免使用方无意中通过只读引用复制并获得一个可修改的新vector
    SharedProxyVector(const SharedProxyVector&) = delete;
    SharedProxyVector& operator=(const SharedProxyVector&) = delete;

    //返回数组的长度
    size_t size() const { return _proxys.size(); };
    //判断数组是否为空
    bool empty() const { return _proxys.empty(); };
    //获取特定下标的数组元素，注意：没有进行越界检查，越界检查交由内部的std::vector进行
    const std::shared_ptr<const T> at(size_t index) const { return _proxys[index]; };
    const std::shared_ptr<T>& at(size_t index) { return _proxys[index]; };
    //重载[]操作符，获取指定下标元素，注意：没有进行越界检查，越界检查交由内部的std::vector进行
    const std::shared_ptr<const T> operator[](size_t index) const { return _proxys[index]; };
    const std::shared_ptr<T>& operator[](size_t index) { return _proxys[index]; };
    typedef std::vector<std::shared_ptr<T>> vec_type;
    typename vec_type::const_reference front() const { return _proxys.front(); };
    typename vec_type::reference front() { return _proxys.front(); };
    typename vec_type::const_reference back() const { return _proxys.back(); };
    typename vec_type::reference back() { return _proxys.back(); };

    //构造一个新的proxy并追加到数组尾部，返回构造出的对象指针，如果没有编辑权限，返回nullptr
    std::shared_ptr<T> add() {
        if (_parent != nullptr) {
            if (!_parent->is_editable()) {
                return nullptr;
            }
            _parent->mark_changed();
            std::shared_ptr<T> p(FeatureProxyBase::create_proxy<T>(_parent));
            _proxys.push_back(p);
            return p;
        }
        std::shared_ptr<T> p(FeatureProxyBase::create_proxy<T>());
        _proxys.push_back(p);
        return p;
    }

    //把特定proxy追加到本数组末尾，同时会把proxy的父对象设置为数组的父对象
    bool push_back(std::shared_ptr<T> p) {
        if (!p) {
            return false;
        }
        if (_parent != nullptr) {
            if (!_parent->is_editable()) {
                return false;
            }
            _parent->mark_changed();
        }
        _proxys.push_back(p);
        if (_parent != nullptr) {
            p->set_parent(_parent);
        }
        return true;
    }

    //清空本对象的内部proxy，同时清空内部proxy的父对象信息
    bool clear() {
        if (_parent != nullptr) {
            if (!_parent->is_editable()) {
                return false;
            }
            if (!_proxys.empty()) {
                _parent->mark_changed();
            }
        }
        for (auto& ptr : _proxys) {
            auto p = reinterpret_cast<FeatureProxyBase*>(ptr.get());
            if (p && p->parent() == _parent) {
                p->set_parent(nullptr);
            }
        }
        _proxys.clear();
        return true;
    }

    //在vector中的特定位置插入proxy，并维护父对象的信息
    bool insert(size_t index, std::shared_ptr<T> p) {
        if (!p) {
            return false;
        }
        if (_parent != nullptr) {
            if (!_parent->is_editable()) {
                return false;
            }
            _parent->mark_changed();
        }
        _proxys.insert(_proxys.begin() + index, p);
        if (_parent != nullptr) {
            p->set_parent(_parent);
        }
        return true;
    }

    //移除特定下标的proxy元素，并维护父对象信息
    bool erase(size_t index) {
        if (_parent != nullptr) {
            if (!_parent->is_editable()) {
                return false;
            }
            _parent->mark_changed();
        }
        auto p = _proxys[index];
        if (p && p->parent() == _parent) {
            p->set_parent(nullptr);
        }
        _proxys.erase(_proxys.begin() + index);
        return true;
    }

    //在本数组中把特定proxy替换为另的proxy，并维护外层父元素信息
    bool replace(std::shared_ptr<T> old_ptr, std::shared_ptr<T> new_ptr) {
        if (_parent != nullptr) {
            if (!_parent->is_editable()) {
                return false;
            }
            _parent->mark_changed();
        }
        for (auto& p : _proxys) {
            if (p == old_ptr) {
                if (p && p->parent() == _parent) {
                    p->set_parent(nullptr);
                }
                if (_parent != nullptr) {
                    new_ptr->set_parent(_parent);
                }
                p = new_ptr;
                return true;
            }
        }
        return false;
    }

    //本数组预留特定大小的空间
    void reserve(size_t rs) {
        _proxys.reserve(rs);
    }

    //把本数组的大小改变为指定大小，如果扩大，新增部分会默认构造出proxy对象，
    //如果数组缩小，缩小部分元素会从数组中移除并维护其父对象信息
    bool resize(size_t ns) {
        if (_parent != nullptr) {
            if (!_parent->is_editable()) {
                return false;
            }
            _parent->mark_changed();
        }
        for (size_t i = ns; i < _proxys.size(); ++i) {
            auto p = _proxys[i];
            if (p && p->parent() == _parent) {
                p->set_parent(nullptr);
            }
        }
        size_t os = _proxys.size();
        _proxys.resize(ns);
        return true;
    }

    //判断本数组内元素是否合法
    bool is_valid() const {
        for (auto& p : _proxys) {
            if (!p || !p->cached_is_valid()) {
                return false;
            }
        }
        if (_proxys.size() < 16) {
            for (size_t i = 0; i + 1 < _proxys.size(); ++i) {
                auto p = _proxys[i];
                for (size_t j = i + 1; j < _proxys.size(); ++j) {
                    if (p == _proxys[j]) {
                        return false;
                    }
                }
            }
        }
        else {
            std::unordered_set<T*> ps(_proxys.size() * 2);
            for (size_t i = 0; i < _proxys.size(); ++i) {
                auto p = _proxys[i];
                if (ps.count(p.get()) > 0) {
                    return false;
                }
                ps.insert(p.get());
            }
        }
        return true;
    }

    //移除本数组中非法和重复的数据元素
    bool remove_invalid() {
        if (_parent != nullptr) {
            if (!_parent->is_editable()) {
                return false;
            }
        }
        bool bf = false;
        for (size_t i = 0; i < _proxys.size(); ++i) {
            auto& p = _proxys[i];
            if (!p || !p->cached_is_valid()) {
                if (!bf && _parent != nullptr) {
                    _parent->mark_changed();
                }
                _proxys.erase(_proxys.begin() + i);
                --i;
                bf = true;
                continue;
            }
        }
        if (_proxys.size() < 16) {
            for (size_t i = 0; i + 1 < _proxys.size(); ++i) {
                auto p = _proxys[i];
                for (size_t j = i + 1; j < _proxys.size(); ++j) {
                    if (p == _proxys[j]) {
                        if (!bf && _parent != nullptr) {
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
            std::unordered_set<T*> ps(_proxys.size() * 2);
            for (size_t i = 0; i < _proxys.size(); ++i) {
                auto p = _proxys[i];
                if (ps.count(p.get()) > 0) {
                    if (!bf && _parent != nullptr) {
                        _parent->mark_changed();
                    }
                    _proxys.erase(_proxys.begin() + i);
                    --i;
                    bf = true;
                    continue;
                }
                ps.insert(p.get());
            }
        }        
        return bf;
    }
    //聚合方法，对本数组中的全部要素做错误数据修复，然后移除数组中的错误元素
    bool correct_content(RoadGeometryManager* mgr) {
        if (_parent != nullptr) {
            if (!_parent->is_editable()) {
                return false;
            }
        }
        bool bc = false;
        for (size_t i = 0; i < _proxys.size(); ++i) {
            auto p = _proxys[i];
            if (p) {
                bc |= p->correct_content(mgr);
            }
        }
        bc |= remove_invalid();
        return bc;
    }
    bool correct_tile_refs(RoadGeometryManager* mgr) {
        if (_parent != nullptr) {
            if (!_parent->is_editable()) {
                return false;
            }
        }
        bool bc = false;
        for (size_t i = 0; i < _proxys.size(); ++i) {
            auto p = _proxys[i];
            if (p) {
                bc |= p->correct_tile_refs(mgr);
            }
        }
        return bc;
    }
    //清除数组内元素的变更状态
    void clear_changed() {
        for (auto& p : _proxys) {
            if (p) {
                p->clear_changed();
            }
        }
    }
    //聚合方法，对本数组中的全部要素重建引用关系
    bool remake_proxy(RoadGeometryManager* mgr) {
        bool bf = true;
        for (auto& p : _proxys) {
            bf &= p->remake_proxy(mgr);
        }
        return bf;
    }
    bool judge_editable(RoadGeometryManager* mgr) {
        bool bf = true;
        for (auto& p : _proxys) {
            bf &= p->judge_editable(mgr);
        }
        return bf;
    }

    //聚合方法，从pb数据中生成本数组中的全部元素
    template <class MsgType>
    bool from_message(const google::protobuf::RepeatedPtrField<MsgType>& ps) {
        clear();
        reserve(ps.size());
        for (int i = 0; i < ps.size(); ++i) {
            auto e = add();
            if (e != nullptr) {
                e->from_message(&ps.Get(i));
            }
            else {
                return false;
            }
        }
        clear_changed();
        return true;
    };

    //交换两个数组的内容，可以用于释放内存/重设parent
    void swap(SharedProxyVector<T>& v) {
        std::swap(v._parent, _parent);
        v._proxys.swap(_proxys);
        if (_parent != nullptr) {
            _parent->mark_changed();
            for (auto& p : _proxys) {
                p->set_parent(_parent);
            }
        }
        if (v._parent != nullptr) {
            v._parent->mark_changed();
            for (auto& p : v._proxys) {
                p->set_parent(v._parent);
            }
        }
    };

    //聚合方法，把本数组中的全部元素填入pb结构中
    template <class MsgType>
    bool to_message(google::protobuf::RepeatedPtrField<MsgType>* ps) const {
        ps->Reserve((int)size());
        ps->Clear();
        for (auto& p : _proxys) {
            p->to_message(ps->Add());
        }
        return true;
    };

    typedef ProxyPtrIterator<T> iterator;
    typedef const ProxyPtrIterator<T> const_iterator;
    //模仿stl容器的begin(),end()定义
    iterator begin() { return iterator(_proxys.data()); };
    const_iterator begin() const { return iterator(const_cast<std::shared_ptr<T>*>(_proxys.data())); };
    iterator end() { return iterator((_proxys.data() + _proxys.size())); };
    const_iterator end() const { return iterator(const_cast<std::shared_ptr<T>*>(_proxys.data() + _proxys.size())); };
    iterator erase(const_iterator it) { 
        size_t index = it - begin();
        erase(index); 
        return iterator(_proxys.data() + index); 
    };
    void sorted_copy(SharedProxyVector<T>& org) {
        _proxys.clear();
        _proxys.reserve(org._proxys.size());
        for (size_t i = 0; i < org._proxys.size(); ++i) {
            bool bf = false;
            for (size_t j = 0; j < i; ++j) {
                if (proxy_compare(_proxys[j], org._proxys[i]) > 0) {
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
    std::vector<std::shared_ptr<T>> _proxys;
    FeatureProxyBase* _parent;
};

template <typename T>
inline int proxy_compare(const std::shared_ptr<T>& lp, const std::shared_ptr<T>& rp) {
    if (!lp && !rp) {
        return 0;
    }
    if (!lp) {
        return -1;
    }
    if (!rp) {
        return 1;
    }
    return lp->compare(*rp);
};
template <typename T>
inline int proxy_compare(const std::shared_ptr<const T>& lp, const std::shared_ptr<const T>& rp) {
    if (!lp && !rp) {
        return 0;
    }
    if (!lp) {
        return -1;
    }
    if (!rp) {
        return 1;
    }
    return lp->compare(*rp);
};

template <typename T>
inline bool proxys_equal(const SharedProxyVector<T>& ls, const SharedProxyVector<T>& rs) {
    if (ls.size() != rs.size()) {
        return false;
    }
    for (size_t i = 0; i < ls.size(); ++i) {
        if (ls[i] == rs[i]) {
            continue;
        }
        if (ls[i] && rs[i] && (ls[i]->compare(*rs[i]) == 0)) {
            continue;
        }
        return false;
    }
    return true;
};

template <typename T>
inline int proxys_compare(const SharedProxyVector<T>& ls, const SharedProxyVector<T>& rs) {
    if (ls.size() < rs.size()) {
        return -1;
    }
    if (ls.size() > rs.size()) {
        return 1;
    }
    for (size_t i = 0; i < ls.size(); ++i) {
        int ret = proxy_compare(ls[i], rs[i]);
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
};

template<typename T>
inline bool proxys_assign(SharedProxyVector<T>& ls, const SharedProxyVector<T>& rs) {
    ls.clear();
    ls.reserve(rs.size());
    for (size_t i = 0; i < rs.size(); ++i) {
        if (rs[i]) {
            ls.push_back(rs[i]->template dump<T>());
        }
    }
    return true;
};

template <typename T>
inline bool three_way_merge_proxys(MergeConflictPolicy policy, RoadGeometryManager* mgr,
            const SharedProxyVector<T>& base, const SharedProxyVector<T>& remote, SharedProxyVector<T>& local) {
    if (proxys_equal(local, remote) || proxys_equal(base, remote)) {
        return false;
    }
    if (proxys_equal(base, local)) {
        proxys_assign(local, remote);
        return false;
    }
    if (local.size() == remote.size() && local.size() == base.size()) {
        bool bc = false;
        for (size_t i = 0; i < local.size(); ++i) {
            auto l = local[i];
            bc |= three_way_merge_proxy<T>(policy, mgr, base[i], remote[i], l);
        }
    }
    if (policy == MERGE_CONFLICT_REMOTE) {
        proxys_assign(local, remote);
    }
    return true;
};

template <typename T>
inline bool three_way_merge_elemental(MergeConflictPolicy policy, const SharedProxyVector<T>& base_sorted,
        const SharedProxyVector<T>& remote_sorted, SharedProxyVector<T>& local_sorted, SharedProxyVector<T>& local) {
    if (proxys_equal(local_sorted, remote_sorted) || proxys_equal(base_sorted, remote_sorted)) {
        return false;
    }
    if (proxys_equal(base_sorted, local_sorted)) {
        proxys_assign(local, remote_sorted);
        return false;
    }
    /*if (policy == MERGE_CONFLICT_REMOTE) {
        proxys_assign(local, remote_sorted);
    }*/
    SharedProxyVector<T> merged;
    merged.reserve(std::max(local_sorted.size(), remote_sorted.size()));
    auto bit = base_sorted.begin();
    auto lit = local_sorted.begin();
    auto rit = remote_sorted.begin();
    bool bconflict = false;
    while (bit != base_sorted.end() || lit != local_sorted.end() || rit != remote_sorted.end()) {
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
            if (**lit == **rit) {
                if (policy == MERGE_CONFLICT_REMOTE) {
                    merged.push_back(*rit);
                    bconflict |= ((*lit)->compare(**rit) != 0);
                }
                else {
                    merged.push_back(*lit);
                    bconflict |= ((*lit)->compare(**rit) != 0);
                }
                ++lit;
                ++rit;
                continue;
            }
            if (**lit < **rit) {
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
            int res = (*bit)->compare(**rit);
            if (res == 0) {
                ++bit;
                ++rit;
                continue;
            }
            if (**bit == **rit) {
                bconflict = true;
                if (policy == MERGE_CONFLICT_REMOTE) {
                    merged.push_back(*rit);
                }
                ++bit;
                ++rit;
                continue;
            }
            if (res > 0) {
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
            int res = (*bit)->compare(**lit);
            if (res == 0) {
                ++bit;
                ++lit;
                continue;
            }
            if (**bit == **lit) {
                bconflict = true;
                if (policy == MERGE_CONFLICT_LOCAL) {
                    merged.push_back(*lit);
                }
                ++bit;
                ++lit;
                continue;
            }
            if (res > 0) {
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
            int b2l = (*bit)->compare(**lit);
            int b2r = (*bit)->compare(**rit);
            int l2r = (*lit)->compare(**rit);
            if (b2l == 0 && b2r == 0) {
                merged.push_back(*lit);
                ++bit;
                ++lit;
                ++rit;
                continue;
            }
            else if (b2l == 0) {
                if (**bit == **rit) {
                    merged.push_back(*rit);
                    ++bit;
                    ++lit;
                    ++rit;
                    continue;
                }
                if (b2r > 0) {
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
            else if (b2r == 0) {
                if (**bit == **lit) {
                    merged.push_back(*lit);
                    ++bit;
                    ++lit;
                    ++rit;
                    continue;
                }
                if (b2l < 0) {
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
                if (l2r == 0) {
                    while (b2l < 0) {
                        ++bit;
                        if (bit == base_sorted.end()) {
                            break;
                        }
                        b2l = (*bit)->compare(**lit);
                    }
                    merged.push_back(*lit);
                    ++lit;
                    ++rit;
                    continue;
                }
                if (**lit == **rit) {
                    while (b2l < 0 || **lit == **bit) {
                        ++bit;
                        if (bit == base_sorted.end()) {
                            break;
                        }
                        b2l = (*bit)->compare(**lit);
                    }
                    bconflict = true;
                    if (policy == MERGE_CONFLICT_REMOTE) {
                        merged.push_back(*rit);
                    }
                    else {
                        merged.push_back(*lit);
                    }
                    ++rit;
                    ++lit;
                    continue;
                }
                if (l2r < 0) {
                    if (b2l < 0) {
                        ++bit;
                        continue;
                    }
                    if (**lit == **bit) {
                        bconflict = true;
                        if (policy == MERGE_CONFLICT_LOCAL) {
                            merged.push_back(*lit);
                        }
                        ++bit;
                        ++lit;
                        continue;
                    }
                    else {
                        merged.push_back(*lit);
                        ++lit;
                        continue;
                    }
                }
                else {
                    if (b2r < 0) {
                        ++bit;
                        continue;
                    }
                    if (**rit == **bit) {
                        bconflict = true;
                        if (policy == MERGE_CONFLICT_REMOTE) {
                            merged.push_back(*rit);
                        }
                        ++bit;
                        ++rit;
                        continue;
                    }
                    else {
                        merged.push_back(*rit);
                        ++rit;
                        continue;
                    }
                }
            }
        }
    }
    if (proxys_compare(local_sorted, merged) != 0) {
        local.swap(merged);
    }
    return bconflict;
};

}; // data_access_engine


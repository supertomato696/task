#pragma once
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include "metadata/metadata.pb.h"
#include "feature_proxy_base.h"

namespace data_access_engine {

class FeatureWithIDProxyBase;
class TileInfoProxy;

struct TileIDAndType {
    uint64_t type : 8;//占8bit,一个字节
    uint64_t tid : 32;//占32bit,四个字节
    uint64_t ext : 24;//占24bit,三个字节
};


//用来表示各种元素的唯一的ID
class FeatureIDProxy : public FeatureProxyBase
{
public:
    typedef RoadPB::FeatureID message_type;
    //构造空的id对象
    FeatureIDProxy() : _type(0), _id(0), _tileid(0), _version(0), _owner(nullptr) {};
    //构造指定父proxy的id对象，注意父proxy不一定是包含本对象的路网元素
    FeatureIDProxy(FeatureProxyBase* parent) : FeatureProxyBase(parent),
            _type(0), _id(0), _tileid(0), _version(0), _owner(nullptr) {};
    virtual ~FeatureIDProxy() { _owner = nullptr; };

    //覆盖基类方法用于与pb结构互转
    virtual bool from_message(const google::protobuf::Message* msg) override;
    virtual bool to_message(google::protobuf::Message* msg) const override;
    
    //覆盖基类方法，判断本ID是否合法
    virtual bool is_valid() const override;
    //覆盖基类方法，修正本ID中明显的错误
    virtual bool correct_content(RoadGeometryManager* pmgr) override;
   
public:
    //返回本ID引用的proxy对象指针，注意此proxy不保证存在
    template <class ProxyType>
    const std::shared_ptr<const ProxyType> as() const;
    template <class ProxyType>
    const std::shared_ptr<ProxyType> as();

    //获取/设置本ID的各种字段
    bool set_tileid(int tileid);
    int tileid() const { return _tileid; };
    bool set_type(int type);
    int type() const { return _type; };
    bool set_id(uint64_t id);
    uint64_t id() const { return _id; };
    bool set_version(int64_t ver);
    int64_t version() const { return _version; };
    Optional<bool> is_deleted() const { return _is_deleted; };
    bool set_is_deleted(Optional<bool> d);
    
public:
    //一次性设置所有有字段，但不保证成功
    void set_value(int tileid, int type, uint64_t id, int64_t ver) {
        set_tileid(tileid);
        set_type(type);
        set_id(id);
        set_version(ver);
    };    
    //基于id的version的大小比较方法
    bool less_version(const std::shared_ptr<const FeatureIDProxy>& e) const {
        return _version < e->version();
    };
    //把本id转换为二进制数据块，主要用于获得map的key
    std::string to_binary_string() const {
        ((TileIDAndType*)_id_buf)->tid = _tileid;
        ((TileIDAndType*)_id_buf)->type = _type;
        *(int64_t*)(&_id_buf[5]) = _id;
        return std::string(_id_buf, 13);
    };
    //返回包含id数据的内部缓存区
    const char* get_data_buffer() const {
        ((TileIDAndType*)_id_buf)->tid = _tileid;
        ((TileIDAndType*)_id_buf)->type = _type;
        *(int64_t*)(&_id_buf[5]) = _id;
        return _id_buf;
    };
    //把本id转换为可读的字符串，主要用于日志输出
    std::string to_string() const {
        return std::to_string(_tileid) + ":" + std::to_string(_type) + "|" +
            std::to_string(_id) + "@" + std::to_string(_version);
        //char buf[128];
        //snprintf(buf, sizeof(buf), "%08x|%x:%016lx@%ld", _tileid, _type, _id, _version);
        //return buf;
    };
    //基于值判断两个id是否相同，忽略id中的version字段
    bool is_equal(const FeatureIDProxy& id) const {
        return id._id == _id && id._tileid == _tileid && id._type == _type;
    };
    bool is_less(const FeatureIDProxy& rhs) const {
        if (_type < rhs._type) {
            return true;
        }
        if (_type > rhs._type) {
            return false;
        }
        if (_tileid < rhs._tileid) {
            return true;
        }
        if (_tileid > rhs._tileid) {
            return false;
        }
        return _id < rhs._id;
    };
    //判断本id元素类型是否给定类型
    bool is_type(int src_type) const {
        if (_type == src_type) {
            return true;
        }
        return false;
    };
    bool is_del() const {
        if (_is_deleted && *_is_deleted) {
            return true;
        }
        return false;
    };
    bool operator==(const FeatureIDProxy& rhs) const {
        return _type == rhs._type && _tileid == rhs._tileid && _id == rhs._id && _version == rhs._version;
    };
    bool operator<(const FeatureIDProxy& rhs) const {
        if (_type < rhs._type) {
            return true;
        }
        if (_type > rhs._type) {
            return false;
        }
        if (_tileid < rhs._tileid) {
            return true;
        }
        if (_tileid > rhs._tileid) {
            return false;
        }
        
        if (_id < rhs._id) {
            return true;
        }
        if (_id > rhs._id) {
            return false;
        }
        return _version < rhs._version;
    };
    int compare(const FeatureIDProxy& rhs) const {
        if (_type != rhs._type) {
            return _type - rhs._type;
        }        
        if (_tileid != rhs._tileid) {
            return _tileid - rhs._tileid;
        }
        if (_id != rhs._id) {
            return (_id < rhs._id ? -1 : 1);
        }
        if (_version < rhs._version) {
            return -1;
        }
        if (_version > rhs._version) {
            return 1;
        }
        return 0;
    };

    //返回包含本id的路网元素指针
    const FeatureWithIDProxyBase* owner() const {
        return _owner;
    };

    //返回id等于本id的元素的指针
    std::shared_ptr<const FeatureWithIDProxyBase> resolve_id(RoadGeometryManager* mgr) const;
    //判断本id表示的弱引用是否合法（如果在本次处理范围之内的元素无法获得其proxy对象，则这个弱引用非法)
    bool is_weak_reference_valid(RoadGeometryManager* mgr) const;

protected:
    //可写访问包含本id的路网元素信息，正常使用方不需要访问这两个接口
    FeatureWithIDProxyBase* owner() {
        return _owner;
    };
    void set_owner(FeatureWithIDProxyBase* owner);

    friend class FeatureMemberBase;
    friend class FeatureReferenceBase;
    friend class FeatureWithIDProxyBase;
    friend class RoadGeometryManager;

private:
    int _tileid;
    int _type;
    uint64_t _id;
    int64_t _version;
    Optional<bool> _is_deleted;

    mutable char _id_buf[13];
    FeatureWithIDProxyBase* _owner;
};

}; //data_access_engine

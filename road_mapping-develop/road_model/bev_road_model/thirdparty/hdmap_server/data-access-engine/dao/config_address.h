#pragma once

#include <map>
#include <memory>
#include <string>

namespace data_access_engine{

struct TileIDBranch
{
    int task_id;
    int tile_11_id;
    std::string road_server_branch;
    std::string tile_server_branch;
    std::string road_server_version;
    std::string tile_server_version;
    std::string road_source_branch;
};

//配置要访问的各服务器地址和数据分支信息，目前只有路网服务器的地址和分支实际使用
class ConfigAddress
{    
public:
    //默认构造&复制构造函数
    ConfigAddress() {};
    ConfigAddress(ConfigAddress* copy) {
        _road_server_address = copy->_road_server_address;
        _road_server_download_branch = copy->_road_server_download_branch;
        _road_server_upload_branch = copy->_road_server_upload_branch;
        _tile_server_download_branch = copy->_tile_server_download_branch;
        _tile_server_upload_branch = copy->_tile_server_upload_branch;
        _tile_server_address = copy->_tile_server_address;        
    }
    ~ConfigAddress() {};

    //用于单例模式的构造/返回本对象唯一实例的方法
    static ConfigAddress* get_instance();

    //访问路网服务器地址
    std::string road_server_address() {
        return _road_server_address;
    }
    void set_road_server_address(const std::string& address) {
        _road_server_address = address;
    }
    //访问路网下载分支，设置下载分支时如果上传分支未设置，把上传分支也设置为同样值
    std::string road_server_download_branch() {
        return _road_server_download_branch;
    }
    void set_road_server_download_branch(const std::string& branch) {
        _road_server_download_branch = branch;
        if (_road_server_upload_branch.empty()) {
            _road_server_upload_branch = branch;
        }
    }
    //访问路网上传分支
    std::string road_server_upload_branch() {
        return _road_server_upload_branch;
    }
    void set_road_server_upload_branch(const std::string& branch) {
        _road_server_upload_branch = branch;
    }
    //访问tile下载分支
    std::string tile_server_download_branch() {
        return _tile_server_download_branch;
    }
    void set_tile_server_download_branch(const std::string& branch) {
        _tile_server_download_branch = branch;
        if (_tile_server_upload_branch.empty()) {
            _tile_server_upload_branch = branch;
        }
    }
    //访问tile上传分支
    std::string tile_server_upload_branch() {
        return _tile_server_upload_branch;
    }
    void set_tile_server_upload_branch(const std::string& branch) {
        _tile_server_upload_branch = branch;
    }
    //访问tile服务器地址
    std::string tile_server_address() {
        return _tile_server_address;
    }
    void set_tile_server_address(const std::string& address) {
        _tile_server_address = address;
    }
    
    void set_all_tileidbranches(std::map<int, data_access_engine::TileIDBranch>& tileidbranches) {
        _tileid2branches = tileidbranches;
    }
    std::map<int, data_access_engine::TileIDBranch> get_all_tileidbranches() {
        return _tileid2branches;
    }
protected:
    static std::unique_ptr<ConfigAddress> _s_config_instance;

private:
    std::string _road_server_address;
    std::string _road_server_download_branch;
    std::string _road_server_upload_branch;
    std::string _tile_server_address;
    std::string _tile_server_download_branch;
    std::string _tile_server_upload_branch;

    std::map<int, data_access_engine::TileIDBranch> _tileid2branches;
};

}; //data_access_engine


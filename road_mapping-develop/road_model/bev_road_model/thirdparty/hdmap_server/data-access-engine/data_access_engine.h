#pragma once
#include <memory>
#include <mutex>
#include <set>
#include <vector>
#include "dao/config_address.h"
#include "public/proj_helper.h"

namespace data_access_engine
{
//记录路网数据下载的一些参数设置信息，直接修改其中内容即可，也提供了一些包装函数
struct RoadTileDownloadParam {
    //要下载的tile按15级tile分组，用户不用填写，manager内部使用
    std::vector<std::vector<int>> tile_15_ids;
    std::vector<std::vector<int>> tile_ext_ids; //当前下载的tile关联元素所在的tile
    int64_t req_ver; //设置下载时要求的提交版本信息，大于0时下载返回不大于这个版本的tile信息
    std::string editor_name; //访问服务器时本机使用的用户名
    std::set<int> req_types; //设置需要下载的元素类型
    int64_t pb_size;
    bool judge_editable;
    bool load_ref;

    RoadTileDownloadParam() : req_ver(0), pb_size(0), judge_editable(true), load_ref(true) {
        req_types = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
    };
};

class ConfigAddress;
class RoadGeometryManager;
//整个路网访问引擎对外的接口都像
class DAEInterface
{
public:
    //默认构造函数，本对象目前支持构造多个实例
    DAEInterface();
    //构造使用特定地址、分支配置信息的接口对象
    DAEInterface(ConfigAddress* config);
    ~DAEInterface();

public:
    //单例方式使用的访问函数，创建/返回默认实例
    static DAEInterface* get_instance();
    //创建并返回本接口对象相应的路网管理对象
    RoadGeometryManager* get_road_geometry_manager();

    //返回用于13级路网tile坐标系进行坐标变换的帮助内，本次盗用
    ProjectionHelper<15>* get_projection_helper();

    void set_destination_tile_id(int tile_id);
    int get_destination_tile_id();

    //设置/访问要下载的全部/当前调用的tileid列表，这一系列方法目前没有实际用途
    void set_all_tileids(std::vector<int>& all_tileids);
    std::vector<int> get_all_tileids();
    void set_current_download_tiles(std::vector<int>& cur_tileids);
    std::vector<int> get_current_download_tileids();
    void set_downloaded_tileids(std::vector<int>& downloaded_tileids);
    std::vector<int> get_downloaded_tileids();
    
private:
    ConfigAddress* _config;
    std::unique_ptr<RoadGeometryManager> _road_geometry_manager;

    std::vector<int> _all_tileids;
    std::vector<int> _current_download_tileids;
    std::vector<int> _downloaded_tileids;

    std::unique_ptr<ProjectionHelper<15>> _projection_helper;
    int _destination_tile_id;

    static std::recursive_mutex s_interface_mutex;
    static std::unique_ptr<DAEInterface> s_dae_interface;
};

};


#pragma once

#include <mutex>
#include <set>
#include <string>
#include <map>

#include "proxy/feature_proxy_base.h"
#include "proxy/feature_id_proxy.h"

#include "proxy/tile_proxy.h"
#include "dynamics/issue.pb.h"
#include "public/vector3.h"

namespace data_access_engine
{
class DAEInterface;
struct RoadTileDownloadParam;
class RoadTileDao;
class ConfigAddress;

class RoadGeometryManager
{
public:
    //默认构造函数，内部会使用单例获取的接口对象和配置对象
    RoadGeometryManager();
    //构造一个使用特定配置对象和接口对象的manager，
    //如果指针为nullptr，会使用其对应的单例对象
    //manager内部直接记录传入的对象指针，因此传入的指针要再manager生命周期内有效
    RoadGeometryManager(ConfigAddress* c, DAEInterface* dae);
    ~RoadGeometryManager();

public:
    //初始化本次要处理的15级tile范围，编辑权限在这些15级tile内的元素被认为是可以修改的
    //本方法会根据传入的11级tile选择统一坐标系并初始化相关坐标转换函数
    bool init_tiles_by_id(const std::vector<int>& tile_15_ids);
    //向服务器下载特定的一组15级tile，可以传入额外的下载数据配置对象，可以设置下载内部使用的并发线程数
    //本方法内部会调用init_tiles_by_id()初始化处理范围和建立统一坐标系。
    //thread_num<=1时本方向单线程顺序下载，否则会建立要求数目的线程并行下载各个11级tile
    bool load_tiles_by_id(const std::vector<int>& tile_15_ids,  
                          RoadTileDownloadParam* param = nullptr, int thread_num = 0);
    //从服务器获取特定15级tile的数据版本和服务器当前时间，更新内部时间同步信息
    bool fetch_tile_versions(const std::vector<int32_t>& tids, const std::string& editor, int64_t& read_ver,
        std::map<int32_t, int64_t>& tile_vers);
    //从设置中的下载数据分支向上传数据分支合并特定类型、特定版本的一组tile，会进行关联扩展数据范围
    bool merge_tiles(const std::vector<int>& tile_15_ids, const std::vector<int>& types, const std::string& editor, int64_t req_ver, int thread_num, int64_t& read_ver);
    //对上传分支上的特定类型的一组tile的元素回滚到特定版，如果版本<=0，所有元素会被删除
    bool revert_tiles(const std::vector<int>& tile_15_ids, const std::string& editor, int64_t req_ver, int thread_num, int64_t& read_ver);
    
    //把下载分支上特定类型特定版本的元素和此分支上最新版本的相应元素进行diff，diff元素会存入diff_tiles中
    bool diff_tiles(const std::vector<int>& tile_15_ids, const std::vector<int>& types, const std::string& editor, int64_t req_ver,
                    int thread_num, TileInfoList& diff_tiles);
    //按要素内容对tiles中的元素进行去重+合并
    void merge_features(ID2TileMap& tiles_map);

    //向服务器提交一组tile数据，本方法在实际上传前会检查要提交的数据是否合法，不合法的数据会直接返回失败
    //本方法可以设定单个提交请求最多包含多少15级tile的数据以避免对服务器压力过大
    bool upload_tiles(const std::string& editor, const TileInfoList& tiles, int max_tile11_num, int thread_num = 0);
    
    //刷新manager内部记录的当前时间信息，此当前实现会通过下载/上传请求时与服务器对时，基本不受本地时钟的影响
    //但内部的当前时间不会实时刷新，必须手工调用本方法进行刷新
    bool make_new_version();
    //人工设置manager中的当前时间信息，并更新内部的对时逻辑，不建议用户直接使用
    bool set_current_version(int64_t ver);
    //返回manager内部记录的当前时间信息，主要用于设置路网元素的id中的version字段
    //如果不调用上面两个方法刷新当前时间，本方法会始终返回同一个值
    int64_t current_version() const;
    //返回manager中记录的下载数据的时间，所有未被修改的元素版本应小于等于此时间，
    //而修改过的元素上传时其版本应该大于此时间
    int64_t download_version() const;

    int64_t get_sync_version() const {
        std::lock_guard<std::mutex> locker(_road_geo_manager_mutex);
        return _sync_ver;
    }

    //获取manager中管理的全部tile数据
    bool get_road_tiles(ID2TileMap& tiles);
    //获取manager中管理的特定tile数据，如果manager中本来没有此tile，会构造新的结构再返回
    TileInfoPtr get_road_tile(int tile_id);
    //把特定tile加入本manager中进行管理和访问，一般情况下不推荐使用本方法
    //注意如果manager中本来已经存在同一个tile的数据，此方法会丢弃原数据加入传入的tile
    bool set_road_tile(int tile_id, TileInfoPtr& tile);
    //获得本次下载请求服务器返回的tile数据
    bool get_current_download_tiles(ID2TileMap& tiles);
    //从manager管理的所有tile中提取全部发生变更的元素，刷新其版本加入到一组新构造的tile数据结构中
    //本方法一般用于给上传接口准备数据，本地修改上传服务器时只需要包含变更要素就可以了
    bool get_changed_tile_data(TileInfoList& tiles);
    //从manager中提起全部有编辑权限的要素，刷新变更元素版本后加入到一组新构造的tile数据结构中
    //本方法一般用于给上传接口准备数据，没有编辑权限的元素一般是不能上传服务器的
    bool get_editable_tile_data(TileInfoList& tiles);

    //判断特定11级tile是否在本次要处理的数据范围内
    bool is_tile_15_downloaded(int tile_id) const;
    bool is_tile_15_current_downloaded(int tile_id) const;
    //使用一组内建的错误修复规则对manager中全部要素尝试进行错误修复
    //如果错误数据无法修复，会把相应元素标记为删除
    //对于下载-修改-上传流程的应用推荐在下载后调用此方法解决服务器上数据本来就有的错误
    bool correct_tiles();
    //修复所有元素对tile的引用关系
    bool correct_tile_refs();
    //从数据中过滤掉非法数据，注意只是从索引中移除相关数据，不会把数据实际删除
    bool filter_invalids();
    //针对已有数据重建相互间的引用关系
    bool resolve_tiles();
    //重新计算各元素在特定tile列表中的编辑权限
    void judge_editable(std::set<int>& tids);

    //清除已下载的所有内容，并撤销统一坐标系设置
    void clear_all();
    //清除已下载的所有tile，但维持统一坐标系设置
    void clear_tiles();
    //清除本次下载的所有tile信息，不影响已下载的全部tile信息
    void clear_current_download_tiles();

    //为放置在给定位置(统一坐标系下的坐标)的路网元素生成元素id，并返回其所属tile的指针
    bool make_new_id(const Vector3D& pos, std::shared_ptr<FeatureWithIDProxyBase> elem,
                    TileInfoPtr& tile, bool bwgs = false);
    //根据元素的统一坐标系位置重新计算其所属tile，如果发生变化，会重新生成id，返回新id所在的tile，并在原tile中留下一个被标记删除的复制元素
    //此方法主要用于处理几何变更后元素所属tile变化的场景
    template <class T>
    bool remake_id(std::shared_ptr<T>& ptr, const Vector3D& pos, TileInfoPtr& tile, bool bwgs = false);
    //功能同上一个方法，但定义为非模板函数以方便使用
    bool remake_elem_id(std::shared_ptr<FeatureWithIDProxyBase>& ptr, const Vector3D& pos, TileInfoPtr& tile, bool bwgs = false);

    //把统一坐标系下的点转换到特定的15级tile坐标系下
    bool tile_local_project(int tile_id, Vector3D& pos) const;
    //把特定13级tile坐标系下的点转换到统一坐标系下
    bool tile_local_unproject(int tile_id, Vector3D& pos) const;
    //获取统一坐标系下的点所在的15级tileid
    int global_point_to_tileID(const Vector3D& pos) const;
    int WGS84_to_tileID(const Vector3D& pos) const;

    bool is_editable_in_tiles(const std::map<int, double>& tid2lens);

    bool global_point_to_WGS84(Vector3D& pos) const;
    bool global_point_from_WGS84(Vector3D& pos) const;
    bool get_tile_box_WGS(int tid, double& minx, double& miny, double& maxx, double& maxy);

    bool load_and_merge(MergeConflictPolicy p, int thread_num, TileInfoList& merged_tiles, TileInfoList& conflict_tiles);
    bool get_merged_tiles(TileInfoList& merged_tiles) {
        merged_tiles = _merged_tiles;
        return true;
    };
    bool get_conflict_tiles(TileInfoList& conflict_tiles) {
        conflict_tiles = _conflict_tiles;
        return true;
    };
    std::vector<std::shared_ptr<RoadPB::Issue>>& get_merge_issues() {
        return _merge_issues;
    };
    RoadPB::Issue* add_merge_issue() {
        auto issue = std::make_shared<RoadPB::Issue>();
        _merge_issues.push_back(issue);
        return issue.get();
    };

    bool load_road_data_from_pbfile(const std::string& strPbFile);
    bool save_road_data_to_pbfile(const std::string& strPbFile);

    std::set<int> get_tile15_ids()
    {
        return _tile15_ids;
    }

public:
    //通过元素id查找manager内管理特定元素对象
    std::shared_ptr<const FeatureWithIDProxyBase> get_feature_by_id(const FeatureIDProxy& eid) const;
    std::shared_ptr<const LinkProxy> get_link(const FeatureIDProxy& eid) const;
    std::shared_ptr<const NodeProxy> get_node(const FeatureIDProxy& eid) const;
    std::shared_ptr<const LaneProxy> get_lane(const FeatureIDProxy& eid) const;
    std::shared_ptr<const LaneBoundaryProxy> get_lane_boundary(const FeatureIDProxy& eid) const;
    std::shared_ptr<const LaneGroupProxy> get_lane_group(const FeatureIDProxy& eid) const;
    std::shared_ptr<const JunctionProxy> get_junction(const FeatureIDProxy& eid) const;
    std::shared_ptr<const TrafficInfoProxy> get_traffic_info(const FeatureIDProxy& eid) const;
    std::shared_ptr<const PositionObjectProxy> get_position_object(const FeatureIDProxy& eid) const;
    std::shared_ptr<const RoadBoundaryProxy> get_road_boundary(const FeatureIDProxy& eid) const;
    std::shared_ptr<const DataQualityProxy> get_data_quality(const FeatureIDProxy& eid) const;
    std::shared_ptr<const DynamicProxy> get_dynamic(const FeatureIDProxy& eid) const;
private:
    bool load_tiles(const std::vector<int>& tile_11_ids, RoadTileDownloadParam* param, int thread_num,
                    std::vector<TileInfoPtr>& tiles, bool update_ver);
    bool remake_tile_proxy(std::vector<TileInfoPtr>& tiles, int thread_num = 1);
    void merge_tiles(std::vector<TileInfoPtr>& tiles, 
                     std::vector<services::TileInfo*>& pb_tiles,
                     std::map<int, std::pair<TileInfoPtr, int>>& tile_infos,
                     std::set<int>& download_tiles, int thread_num);
    bool split_tile_list_by_tile11(const TileInfoList& tiles, int max_tile11_num, std::vector<TileInfoList>& sub_tiles);
    TileInfoPtr get_tile_in_map(int tile_id, bool link_parent, ID2TileMap& id2tiles);

private:
    ID2TileMap _tiles;
    ID2TileMap _current_download_tiles;
    std::set<int> _tile15_ids;

    int64_t _current_ver;
    int64_t _sync_ver;
    uint64_t _prev_tick;
    uint64_t _last_tick;
    mutable bool _tile_inited;

    std::unique_ptr<RoadTileDao> _road_tile_dao;
    DAEInterface* _interface;
    mutable std::mutex _road_geo_manager_mutex;
    std::unique_ptr<RoadTileDownloadParam> _download_param;
    
    TileInfoList _merged_tiles;
    TileInfoList _conflict_tiles;
    std::vector<std::shared_ptr<RoadPB::Issue>> _merge_issues;
};

}; // data_access_engine ;


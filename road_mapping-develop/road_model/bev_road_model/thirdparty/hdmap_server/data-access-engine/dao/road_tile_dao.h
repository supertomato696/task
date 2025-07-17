#pragma once
#include <mutex>
#include <set>
#include "data_access_engine.h"
#include "services/roadserver.pb.h"
#include "proxy/tile_proxy.h"

namespace data_access_engine {
    
class RoadTileDao {
public:
    RoadTileDao() { _config = ConfigAddress::get_instance(); };
    RoadTileDao(ConfigAddress* c) {
        if (c) {
            _config = c;
        }
        else {
            _config = ConfigAddress::get_instance();
        }
    };
    ~RoadTileDao() {};

public:
    bool download_tiles_one_by_one(RoadTileDownloadParam& download_param, int64_t& ver, 
                                   std::vector<services::TileInfo*>& tiles);
    bool download_tiles_parallel(RoadTileDownloadParam& download_param, int thread_num,
                                 int64_t& ver, std::vector<services::TileInfo*>& tiles);
    bool upload_tile(const std::string& cur_editor, int64_t& version, const TileInfoList& tiles);
    bool merge_tiles(const std::vector<int>& tids, const std::vector<int>& types, const std::string& editor, int64_t req_ver, int64_t& ver);
    bool revert_tiles(const std::vector<int>& tids, const std::vector<int>& types, const std::string& editor, int64_t req_ver, int64_t& ver);
    bool save_tiles(const TileInfoList& tiles, const std::string& editor, const int64_t version, const char* path);
    bool load_tiles(const char* path, std::string& editor, TileInfoList& tiles);
    bool fetch_tile_versions(const std::vector<int32_t>& tids, const std::string& editor, int64_t& read_ver,
                             std::map<int32_t, int64_t>& tile_vers);
    bool fetch_tile_versions(const std::map<int, std::vector<int32_t>>& map_11_tids, const std::string& editor, 
                             int64_t& read_ver, std::map<int32_t, int64_t> tile_vers);
    bool commit_change(TileInfoList& tiles, std::string& editor, int64_t& cur_ver);

private:
    struct DownLoadParam {
        std::vector<services::TileInfo*>* tiles;
        RoadTileDownloadParam param;
        std::string branch;        
        int tile_index;
        int64_t version;
    };

    bool download_tiles_15(const std::vector<int>& tiles15, DownLoadParam& param);
    void close();
    void refresh_tile_elem_version(int64_t old_version, int64_t new_version, std::unordered_map<int, int64_t>& id2ver,
                                    const TileInfoPtr& tile_info);
    ConfigAddress* _config;
    std::mutex _road_tile_dao_mutex;
};

}; // data_access_engine ;
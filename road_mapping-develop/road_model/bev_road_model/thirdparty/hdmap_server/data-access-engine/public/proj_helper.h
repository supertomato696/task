#pragma once

#include <cstdint>
#include <cmath>
#include <vector>
#include <unordered_map>
#ifdef WIN32
#include "proj4/include/proj_api.h"
#else
#include "proj_api.h"
#endif
namespace data_access_engine {
const double HADMAP_RAD_TO_DEG = 57.29577951308232;
const double HADMAP_DEG_TO_RAD = 0.0174532925199432958;

struct Proj4ContextWrapper {
    projCtx ctx;

    Proj4ContextWrapper() {
        ctx = pj_ctx_alloc();
    };
    ~Proj4ContextWrapper() {
        if (ctx) {
            pj_ctx_free(ctx);
            ctx = nullptr;
        }
    };

    operator projCtx& () {
        return ctx;
    };

    void swap(Proj4ContextWrapper& wp) {
        projCtx w = wp.ctx;
        wp.ctx = ctx;
        ctx = w;
    };
};

struct Proj4PJWrapper {
    projPJ pj;

    Proj4PJWrapper() : pj(nullptr) {};

    Proj4PJWrapper(const char* strInit) {
        pj = pj_init_plus(strInit);
    };
    Proj4PJWrapper(projCtx ctx, const char* strInit) {
        pj = pj_init_plus_ctx(ctx, strInit);
    };
    ~Proj4PJWrapper() {
        if (pj) {
            pj_free(pj);
            pj = nullptr;
        }
    };

    operator projPJ& () {
        return pj;
    };

    void swap(Proj4PJWrapper& wp) {
        projPJ w = wp.pj;
        wp.pj = pj;
        pj = w;
    };
    void reset() {
        if (pj) {
            pj_free(pj);
            pj = nullptr;
        }
    };
};

static inline int GetBit(int32_t v, int32_t i) {
    return ((uint32_t)v >> i) & 1;
}

template <int TileLevel>
class ProjectionHelper {    
public:
    ProjectionHelper() : _proj_ctx(), _tile_id(0), _cache_pj(NULL), _cache_tile_id(0),
            _proj_dst(NULL), _proj_wgs(_proj_ctx, "+proj=latlong +datum=WGS84") {};
    ~ProjectionHelper() {
        _tile_projs.clear();
        _proj_dst = nullptr;
        _cache_pj = nullptr;
        _proj_dst_tmp.reset();
        _proj_wgs.reset();
    };
    bool Gauss_Kruger_projection(double& px, double& py, double& pz) {
        double x = px * HADMAP_DEG_TO_RAD;
        double y = py * HADMAP_DEG_TO_RAD;
        double z = pz;
        if (pj_transform(_proj_wgs, *_proj_dst, 1, 3, &x, &y, &z)) {
            return false;
        }
        px = x;
        py = y;
        pz = z;
        return true;
    };

    bool Gauss_Kruger_unprojection(double& px, double& py, double& pz) {
        double x = px;
        double y = py;
        double z = pz;
        if (pj_transform(*_proj_dst, _proj_wgs, 1, 3, &x, &y, &z)) {
            return false;
        }
        px = x * HADMAP_RAD_TO_DEG;
        py = y * HADMAP_RAD_TO_DEG;
        pz = z;
        return true;
    };

    int32_t WGS_to_tile_ID(double x, double y, double z) {
        int32_t ix = (int32_t)(x / (90. / (1 << 30)));
        int32_t iy = (int32_t)(y / (90. / (1 << 30)));
        int32_t tile_id = GetBit(ix, 31);
        for (int32_t i = 30; i > (30 - TileLevel); --i) {
            tile_id <<= 2;
            tile_id |= GetBit(iy, i) << 1 | GetBit(ix, i);
        }
        return tile_id;
    };

    int32_t point_to_tile_ID(double x, double y, double z) {
        if (!Gauss_Kruger_unprojection(x, y, z)) {
            return 0;
        }
        return WGS_to_tile_ID(x, y, z);
    };

    Proj4PJWrapper& get_tile_Proj4_wrapper(int32_t tile_id) {
        if (_cache_tile_id == tile_id) {
            return *_cache_pj;
        }
        int32_t lx = 0;
        for (int i = TileLevel - 1; i >= 0; --i) {
            lx <<= 1;
            lx |= GetBit(tile_id, i * 2);
        }
        if (_tile_projs.find(lx) == _tile_projs.end()) {
            double l = (lx + 0.5) * 90. / (1 << (TileLevel - 1));
            char buf[256];            
#ifdef _MSC_VER
            sprintf_s(buf, "+proj=tmerc +ellps=WGS84 +datum=WGS84 +k_0=1 +lon_0=%.12lf +lat_0=0 +x_0=500000 +y_0=0 +units=m", l);
#else
            snprintf(buf, sizeof(buf), "+proj=tmerc +ellps=WGS84 +datum=WGS84 +k_0=1 +lon_0=%.12lf +lat_0=0 +x_0=500000 +y_0=0 +units=m", l);
#endif
            Proj4PJWrapper pj(_proj_ctx, buf);
            _tile_projs[lx].swap(pj);
        }
        _cache_tile_id = tile_id;
        _cache_pj = &_tile_projs[lx];
        return *_cache_pj;
    };

    bool tile_local_project(int32_t tile_id, double& px, double& py, double& pz) {
        if (tile_id == _tile_id) {
            return true;
        }
        auto& pj = get_tile_Proj4_wrapper(tile_id);
        if (!pj) {
            return false;
        }
        double x = px;
        double y = py;
        double z = pz;
        if (pj_transform(*_proj_dst, pj, 1, 3, &x, &y, &z)) {
            return false;
        }
        px = x;
        py = y;
        pz = z;
        return true;
    };

    bool tile_local_unproject(int32_t tile_id, double& px, double& py, double& pz) {
        if (tile_id == _tile_id) {
            return true;
        }
        auto& pj = get_tile_Proj4_wrapper(tile_id);
        if (!pj) {
            return false;
        }
        double x = px;
        double y = py;
        double z = pz;
        if (pj_transform(pj, *_proj_dst, 1, 3, &x, &y, &z)) {
            return false;
        }
        px = x;
        py = y;
        pz = z;
        return true;
    };

    bool tile_local_to_WGS(int32_t tile_id, double& px, double& py, double& pz) {
        auto& pj = get_tile_Proj4_wrapper(tile_id);
        if (!pj) {
            return false;
        }
        double x = px;
        double y = py;
        double z = pz;
        if (pj_transform(pj, _proj_wgs, 1, 3, &x, &y, &z)) {
            return false;
        }
        px = x * HADMAP_RAD_TO_DEG;
        py = y * HADMAP_RAD_TO_DEG;
        pz = z;
        return true;
    };

    bool WGS_to_tile_local(int32_t tile_id, double& px, double& py, double& pz) {
        auto& pj = get_tile_Proj4_wrapper(tile_id);
        if (!pj) {
            return false;
        }
        double x = px * HADMAP_DEG_TO_RAD;
        double y = py * HADMAP_DEG_TO_RAD;
        double z = pz;
        if (pj_transform(_proj_wgs, pj, 1, 3, &x, &y, &z)) {
            return false;
        }
        px = x;
        py = y;
        pz = z;
        return true;
    };

    std::string get_tile_Proj4_string(int32_t tile_id) {
        int32_t lx = 0;
        for (int i = TileLevel - 1; i >= 0; --i) {
            lx <<= 1;
            lx |= GetBit(tile_id, i * 2);
        }
        double l = (lx + 0.5) * 90. / (1 << (TileLevel - 1));
        char buf[256];
        
#ifdef _MSC_VER
        sprintf_s(buf, "+proj=tmerc +ellps=WGS84 +datum=WGS84 +k_0=1 +lon_0=%.12lf +lat_0=0 +x_0=500000 +y_0=0 +units=m", l);
#else
        snprintf(buf, sizeof(buf), "+proj=tmerc +ellps=WGS84 +datum=WGS84 +k_0=1 +lon_0=%.12lf +lat_0=0 +x_0=500000 +y_0=0 +units=m", l);
#endif
        std::string pj(buf);
        return pj;
    };

    void set_destination_tile(int32_t tile_id) {
        _tile_id = tile_id;
        _proj_str = get_tile_Proj4_string(tile_id);
        _proj_dst = &get_tile_Proj4_wrapper(tile_id);
    };

    void set_destination_Proj4_string(const std::string& pj) {
        if (_tile_id < 0 && pj == _proj_str) {
            return;
        }
        _tile_id = -1;
        _proj_str = pj;
        Proj4PJWrapper p(_proj_ctx, pj.c_str());
        _proj_dst_tmp.swap(p);
        _proj_dst = &_proj_dst_tmp;
    };

    bool get_tile_center_WGS(int32_t tile_id, double& wgsx, double& wgsy) {
        int32_t lx = 0;
        int32_t ly = 0;

        for (int i = (TileLevel - 1); i >= 0; --i) {
            lx <<= 1;
            lx |= GetBit(tile_id, i * 2);
            ly <<= 1;
            ly |= GetBit(tile_id, i * 2 + 1);
        }

        wgsx = (lx + 0.5) * 180. / (1 << TileLevel);
        wgsy = (ly + 0.5) * 180. / (1 << TileLevel);
        return true;
    };

    bool get_tile_center(int32_t tile_id, double& cpx, double& cpy) {
        if (!get_tile_center_WGS(tile_id, cpx, cpy)) {
            return false;
        }
        double z = 0;
        if (!WGS_to_tile_local(tile_id, cpx, cpy, z)) {
            return false;
        }
        cpx = std::round(cpx);
        cpy = std::round(cpy);
        return true;
    };

    bool get_tile_box_WGS(int32_t tile_id, double& minx, double& miny, double& maxx, double& maxy) {
        double cx = 0;
        double cy = 0;
        if (!get_tile_center_WGS(tile_id, cx, cy)) {
            return false;
        }
        minx = cx - 0.5 * 180. / (1 << TileLevel);
        miny = cy - 0.5 * 180. / (1 << TileLevel);
        maxx = cx + 0.5 * 180. / (1 << TileLevel);
        maxy = cy + 0.5 * 180. / (1 << TileLevel);
        return true;
    };

    std::vector<int> get_adjacent_tile_ID(int32_t tileid) {
        std::vector<int> tile_IDs;
        double wgsx = 0;
        double wgsy = 0;
        get_tile_center_WGS(tileid, wgsx, wgsy);
        double rightx = wgsx + 180. / (1 << TileLevel);
        double righty = wgsy;
        int tile_id = WGS_to_tile_ID(rightx, righty, 0.0);
        tile_IDs.push_back(tile_id);
        double upx = wgsx;
        double upy = wgsy + 180. / (1 << TileLevel);
        tile_id = WGS_to_tile_ID(upx, upy, 0.0);
        tile_IDs.push_back(tile_id);
        double leftx = wgsx - 180. / (1 << TileLevel);
        double lefty = wgsy;
        tile_id = WGS_to_tile_ID(leftx, lefty, 0.0);
        tile_IDs.push_back(tile_id);
        double downx = wgsx;
        double downy = wgsy - 180. / (1 << TileLevel);
        tile_id = WGS_to_tile_ID(downx, downy, 0.0);
        tile_IDs.push_back(tile_id);
        return tile_IDs;
    };

private:
    std::unordered_map<int, Proj4PJWrapper> _tile_projs;
    Proj4ContextWrapper _proj_ctx;
    Proj4PJWrapper _proj_wgs;
    Proj4PJWrapper* _proj_dst;
    std::string _proj_str;
    int _tile_id;
    int _cache_tile_id;
    Proj4PJWrapper* _cache_pj;
    Proj4PJWrapper _proj_dst_tmp;
};

}; // hadmap

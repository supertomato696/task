/**
 * @file gen_topo_layer.h
 * @author Fei Han (fei.han@horizon.ai)
 * @brief
 * @version 0.1
 * @date 2019-10-30
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef INTERFACE_GEN_TOPO_LAYER_H_
#define INTERFACE_GEN_TOPO_LAYER_H_

#include <memory>
#include <vector>
#include "../../proto/ndm.pb.h"
#include "../interface/index.h"
#include "../interface/map_component.h"

namespace map_interface {
void GetMinAndMaxPointOfMap(
    const std::vector<std::shared_ptr<StaticMapComponent>>& map_components,
    ndm_proto::Point* min_point, ndm_proto::Point* max_point);

bool GetTileIdOfMapSegment(ndm_proto::MapEnvMsg* map_segment, int* x_idx,
                           int* y_idx, Eigen::Vector3d* min_point,
                           const int& tile_size);
void GenTilesOfMap(
    const std::vector<std::shared_ptr<StaticMapComponent>>& map_components,
    ndm_proto::TopologicalLayer* topo_layer, map_interface::Index* pMap_index,
    const int& tile_size);
}  // namespace map_interface
#endif  // INTERFACE_GEN_TOPO_LAYER_H_

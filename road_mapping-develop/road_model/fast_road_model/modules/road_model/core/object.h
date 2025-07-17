#pragma once
#include <core/geometry.h>
namespace fast_road_model
{
  struct OBJPoint : public BasePoint<OBJPoint>
  {

  };
  using OBJPointPtr = std::shared_ptr<OBJPoint>;
  struct Object
  {
    int id;
    std::string type;
    std::vector<OBJPointPtr> points;
  };
  using ObjectPtr = std::shared_ptr<Object>;
}

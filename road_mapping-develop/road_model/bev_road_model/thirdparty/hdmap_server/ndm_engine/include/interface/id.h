/**
 * @file id.h
 * @author Han Fei
 * @brief MAP ID
 * @version 0.1
 * @date 2019-08-28
 *
 * @copyright Copyright (c) 2019
 *
 */
#ifndef INTERFACE_ID_H_
#define INTERFACE_ID_H_

#include <string>
#include "../interface/sole.hpp"

namespace map_interface {
class ID {
 public:
  static ID *GetInstance() {
    // local static对象, c++11多线程同步
    static ID instance;
    return &instance;
  }

  std::string GenId() {
    sole::uuid u1 = sole::uuid1();
    // sole::uuid u1 = sole::uuid4();
    return u1.str();
  }

 private:
  ID() {}

  ID(const ID &);
  ID &operator=(const ID &);

  ~ID() {}
};
}  // end namespace map_interface
#endif  // INTERFACE_ID_H_

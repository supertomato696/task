#pragma once
#include <iostream>
#include <set>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
namespace fast_road_model
{
    template <typename T>
    using SmartPtr = std::shared_ptr<T>;

    template <typename T>
    struct BasePoint
    {
        Eigen::Vector3d dir;
        Eigen::Vector3d pos;
        std::set<T *> prev_all;
        std::set<T *> next_all;
        void init(const Eigen::Vector3d &ipos, const Eigen::Vector3d &idir)
        {
            pos = ipos;
            dir = idir;
        }
        void set_dir(const Eigen::Vector3d &idir)
        {
            dir = idir;
        }
        bool add_prev(T *data)
        {
            if (prev_all.count(data))
            {
                return false;
            }
            else
            {
                prev_all.insert(data);
                return true;
            }
        }
        bool add_next(T *data)
        {
            if (next_all.count(data))
            {
                return false;
            }
            else
            {
                next_all.insert(data);
                return true;
            }
        }
        std::set<T *> &get_prev(void)
        {
            return prev_all;
        }
        std::set<T *> &get_next(void)
        {
            return next_all;
        }
        bool in_prev(T *data) const
        {
            return prev_all.count(data) ? true : false;
        }
        bool in_next(T *data) const
        {
            return next_all.count(data) ? true : false;
        }
    };
}

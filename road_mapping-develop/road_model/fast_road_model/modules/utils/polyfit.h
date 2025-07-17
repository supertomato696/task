#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <numeric>
#include <vector>
#include "utils/macro_util.h"
#include "utils/log_util.h"

namespace fsdmap
{
    class PolyFit
    {
    private:
        Eigen::Matrix3d Rlw;
        Eigen::VectorXd coefficients;
       void calc_center(void)
        {
            if(!options.use_calc_center)
            {
                return ;
            }
            auto &points = problem.points;
            double n = points.size();
            Eigen::Vector3d mean = Eigen::Vector3d::Zero();
            for (const auto &point : points)
            {
                mean = mean.eval()+point;
            }
            mean /= n;
            problem.center=mean;
        }
        Eigen::Vector2d compute_principal_direction(void)
        {
            auto &points = problem.points;
            double n = points.size();
            Eigen::Vector2d mean = Eigen::Vector2d::Zero();
            for (const auto &point : points)
            {
                mean += Eigen::Vector2d(point.x(), point.y());
            }
            mean /= n;
            Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
            for (const auto &point : points)
            {
                Eigen::Vector2d diff(point.x() - mean.x(), point.y() - mean.y());
                cov += diff * diff.transpose();
            }
            cov /= n;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
            Eigen::Vector2d principalDirection = solver.eigenvectors().col(1); // 最大特征值对应的特征向量
            return principalDirection;
        }
        Eigen::Matrix3d compute_rotation_matrix(const Eigen::Vector2d &pd)
        {
            Eigen::Vector2d dir = {problem.dir.x(),problem.dir.y()};
            if (options.use_calc_direction)
            {
                dir = pd;
            }
            double angle = atan2(dir.y(), dir.x());
            //
            // LOG_INFO("angle:{}",angle)
            Eigen::Matrix3d rotationMatrix = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            //
            return rotationMatrix.transpose();
        }
        std::vector<Eigen::Vector3d> trans_to_local(void)
        {
            auto &ipoints = problem.points;
            std::vector<Eigen::Vector3d> lp;
            for (auto p : ipoints)
            {
                Eigen::Vector3d pt = p - problem.center;
                pt = Rlw * pt.eval();
                lp.push_back(pt);
            }
            return lp;
        }

        Eigen::VectorXd polyfit(const std::vector<Eigen::Vector3d> &ipoints)
        {
            int n = ipoints.size();
            Eigen::MatrixXd A(n, problem.degree + 1);
            Eigen::MatrixXd b(n, 1);
            for (int i = 0; i < n; ++i)
            {
                b(i, 0) = ipoints[i].y();
                for (int j = 0; j <= problem.degree; ++j)
                {
                    A(i, j) = std::pow(ipoints[i].x(), j);
                }
            }
            Eigen::VectorXd coefficients = A.colPivHouseholderQr().solve(b);
            return coefficients;
        }
        double eval(double x)
        {
            double y = 0;
            for (int j = 0; j <= problem.degree; ++j)
            {
                y += coefficients[j] * std::pow(x, j);
            }
            return y;
        }
        double curvature(double x)
        {
            if(problem.degree<3)
            {
                return 0;
            }
            auto f1 = [&](double x)
            {
                return coefficients[0] + 2.0 * coefficients[1] * x + 3.0 * coefficients[2] * x * x;
            };
            auto f2 = [&](double x)
            {
                return 2.0 * coefficients[1] + 6.0 * coefficients[2] * x;
            };
            double y=std::abs(f2(x)) / std::pow((1.0 + f1(x) * f1(x)), 1.5);
         
            return y;
        }

    public:
        struct Problem
        {
            Eigen::Vector3d dir;
            Eigen::Vector3d center;
            std::vector<Eigen::Vector3d> points;
            int degree = {3};
        };
        struct Options
        {
            bool use_calc_direction{false};
            bool use_calc_center{true};
        };
        PolyFit::Problem problem;
        PolyFit::Options options;

        PolyFit() {
        };

        PolyFit(const PolyFit::Problem &iproblem, const PolyFit::Options &ioption)
        {
            // std::cout<<"start polyfit"<<std::endl;
            problem=iproblem;
            options=ioption;
            // LOG_INFO("problem:{}",problem.points.size());
        }
        void solver(void)
        {
            calc_center();
            Eigen::Vector2d pd = compute_principal_direction();
            Rlw = compute_rotation_matrix(pd);
            auto pl = trans_to_local();
            coefficients = polyfit(pl);
        }
       double  curvature(const Eigen::Vector3d &p)
        {
            Eigen::Vector3d pl = (p - problem.center);
            pl = Rlw * pl.eval();
            return  curvature(pl.x());
        }

        Eigen::Vector3d eval(const Eigen::Vector3d &p)
        {
            Eigen::Vector3d pl = (p - problem.center);
            pl = Rlw * pl.eval();
            double y = eval(pl.x());
            // loss=std::abs(pl.y()-y);
            Eigen::Vector3d ret = {pl.x(), y, p.z()};
            ret = Rlw.transpose() * ret.eval();
            return (ret + problem.center);
        }
        void sample(void)
        {
            std::vector<Eigen::Vector3d> ps={{0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {2.0, 2.0, 0.0}, {3.0, 3.0, 0.0}, {4.0, 4.0, 0.0}};
            Eigen::Vector3d center={0,0,0};
            PolyFit::Problem problem;
            problem.points=ps;
            problem.center={0,0,0};
            problem.dir={0,0,0};
            PolyFit::Options option;
            option.use_calc_direction=true;
            PolyFit fit=PolyFit(problem,option); 
            fit.solver();       
            auto  p=fit.eval({5,2,0});
        }
    };
}
#pragma once
#include <vector>
#include <memory>
#include <Eigen/Dense>

// 必须放到 <osqp/osqp.h> 前面
#include <opencv2/opencv.hpp>

#include <osqp/osqp.h>


struct SplineParameters {
    double lambda1 = 1.0; 
    double lambda2 = 0.01;
    double alpha = 0.5;
    bool use_chord_length = true;
    bool use_all_pnt_constrain = false;
};


class SplineInterpolator {
public:

    SplineInterpolator(SplineParameters params = SplineParameters());
    ~SplineInterpolator();

    // 主要接口函数
    bool fitSpline(const std::vector<Eigen::Vector3d>& points,
                   const Eigen::Vector3d& start_direction,
                   const Eigen::Vector3d& end_direction);

    Eigen::Vector3d evaluate(double t) const;

    Eigen::Vector3d evaluateDerivative(double t) const;

    Eigen::Vector3d evaluateSecondDerivative(double t) const;

    const Eigen::Matrix<double, 3, 4>& getCoefficients() const { return coefficients_; }

    double getFittingError() const { return fitting_error_; }

    std::vector<Eigen::Vector3d> sample_pnts(int num_samples = 100);

    std::vector<Eigen::Vector3d> resample_pnts();
private:
    std::vector<double> computeParameterization(const std::vector<Eigen::Vector3d>& points);

    void buildObjectiveMatrices(const std::vector<Eigen::Vector3d>& points,
                               const std::vector<double>& t_values,
                               Eigen::MatrixXd& Q,
                               Eigen::VectorXd& q);

    void buildConstraintMatrices(const std::vector<Eigen::Vector3d>& points,
                                const std::vector<double>& t_values,
                                const Eigen::Vector3d& start_direction,
                                const Eigen::Vector3d& end_direction,
                                Eigen::MatrixXd& A,
                                Eigen::VectorXd& b);

    Eigen::Vector4d basisFunction(double t) const;
    Eigen::Vector4d basisFunctionDerivative(double t) const;
    Eigen::Vector4d basisFunctionSecondDerivative(double t) const;

    Eigen::Matrix4d computeSmoothnessMatrix() const;

    void checkMatrixCondition(const Eigen::MatrixXd& matrix, const std::string& name);

    bool validateOSQPData(OSQPData* data);

    bool solveConstrainedQP(const Eigen::MatrixXd& Q,
                            const Eigen::VectorXd& q,
                            const Eigen::MatrixXd& A,
                            const Eigen::VectorXd& b,
                            Eigen::VectorXd& x);
    bool solveQP(const Eigen::MatrixXd& Q, const Eigen::VectorXd& q,
                 const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                 Eigen::VectorXd& solution);

    SplineParameters params_;
    Eigen::Matrix<double, 3, 4> coefficients_;
    double fitting_error_;
    bool is_fitted_;
    std::vector<double> t_values_;

    void* osqp_work_;
    void* osqp_settings_;
    void* osqp_data_;

public:
    // 可视化功能
    void visualize2D(const std::string& window_name = "Cubic Spline 2D",
                     int width = 800, int height = 600);
    void visualize3D(const std::string& window_name = "Cubic Spline 3D",
                     int width = 800, int height = 600) const;
    void visualizeProjections(const std::string& window_name = "Cubic Spline Projections",
                              int width = 1200, int height = 400) const;
private:
    // 可视化
    std::vector<Eigen::Vector3d> control_points_;
    std::vector<Eigen::Vector3d> fitted_points_;
    std::vector<Eigen::Vector3d> fitted_points_visual_;
    Eigen::Vector3d start_direction_;
    Eigen::Vector3d end_direction_;


    // 可视化辅助函数
    cv::Point2f project3Dto2D(const Eigen::Vector3d& point, const cv::Mat& projection_matrix) const;
    void drawCurve(cv::Mat& image, const std::vector<Eigen::Vector3d>& points,
                   const cv::Mat& projection_matrix, const cv::Scalar& color, int thickness = 2) const;
    void drawPoint(cv::Mat& image, const Eigen::Vector3d& point,
                   const cv::Mat& projection_matrix, const cv::Scalar& color, int radius = 5) const;
    void drawVector(cv::Mat& image, const Eigen::Vector3d& start, const Eigen::Vector3d& vector,
                    const cv::Mat& projection_matrix, const cv::Scalar& color, int thickness = 2) const;
    cv::Mat createProjectionMatrix(const std::string& view_type,
                                   const cv::Size& image_size,
                                   const Eigen::Vector3d& center, double scale) const;

    void getBoundingBox(Eigen::Vector3d& min_pt, Eigen::Vector3d& max_pt) const;
};


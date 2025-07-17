#include "spline_interpolator.h"
#include "utils/log_util.h"

#include <cmath>
#include <iostream>
#include <algorithm>
#include <iomanip>

SplineInterpolator::SplineInterpolator(SplineParameters params)
    : params_(params), fitting_error_(0.0), is_fitted_(false),
      osqp_work_(nullptr), osqp_settings_(nullptr), osqp_data_(nullptr) {
    coefficients_.setZero();
}

SplineInterpolator::~SplineInterpolator() {
    if (osqp_work_) {
        osqp_cleanup(reinterpret_cast<OSQPWorkspace*>(osqp_work_));
    }
    if (osqp_settings_) {
        delete reinterpret_cast<OSQPSettings*>(osqp_settings_);
    }
    if (osqp_data_) {
        delete reinterpret_cast<OSQPData*>(osqp_data_);
    }
}

bool SplineInterpolator::fitSpline(const std::vector<Eigen::Vector3d>& points,
                                   const Eigen::Vector3d& start_direction,
                                   const Eigen::Vector3d& end_direction) {
    if (points.size() < 2) {
        std::cerr << "至少需要2个点来拟合样条曲线" << std::endl;
        return false;
    }
    control_points_ = points;
    start_direction_ = start_direction;
    end_direction_ = end_direction;

    // 1. 计算参数化
    t_values_ = computeParameterization(points);

    // 2. 构建目标函数矩阵
    Eigen::MatrixXd Q(12, 12);
    Eigen::VectorXd q(12);
    buildObjectiveMatrices(points, t_values_, Q, q);

    // 3. 构建约束矩阵
    Eigen::MatrixXd A(12, 12);  // 6个等式约束
    Eigen::VectorXd b(12);
    buildConstraintMatrices(points, t_values_, start_direction, end_direction, A, b);

    // 4. 求解QP问题
    Eigen::VectorXd solution(12);
    if (!solveQP(Q, q, A, b, solution)) {
        std::cerr << "求解QP问题失败" << std::endl;
        return false;
    }

    // 5. 提取系数
    coefficients_(0, 0) = solution(0);  // a_x0
    coefficients_(0, 1) = solution(1);  // a_x1
    coefficients_(0, 2) = solution(2);  // a_x2
    coefficients_(0, 3) = solution(3);  // a_x3
    coefficients_(1, 0) = solution(4);  // a_y0
    coefficients_(1, 1) = solution(5);  // a_y1
    coefficients_(1, 2) = solution(6);  // a_y2
    coefficients_(1, 3) = solution(7);  // a_y3
    coefficients_(2, 0) = solution(8);  // a_z0
    coefficients_(2, 1) = solution(9);  // a_z1
    coefficients_(2, 2) = solution(10); // a_z2
    coefficients_(2, 3) = solution(11); // a_z3

    bool check_other_method = true;
    bool use_other_method = false;
    use_other_method = true; // TODO:qzc 暂时直接用这个
    if(check_other_method) {
        Eigen::VectorXd x;
        bool ret = solveConstrainedQP(Q, q, A, b, x);
        if(ret) {
            std::cout << std::fixed << std::setprecision(6);
            // std::cout << x.transpose() << std::endl;
            std::cout << "使用 eigen 求解: solveConstrainedQP " << std::endl;
            for (int i = 0; i < 3; ++i) {
                int index = 4*i;
                char coord = (i == 0) ? 'x' : (i == 1) ? 'y' : 'z';
                std::cout << "  " << coord << ": ["
                        << x(index+0) << ", " << x(index+1) << ", "
                        << x(index+2) << ", " << x(index+3) << "]" << std::endl;

                if(use_other_method) {
                    coefficients_(i, 0) = x(index+0);
                    coefficients_(i, 1) = x(index+1);
                    coefficients_(i, 2) = x(index+2);
                    coefficients_(i, 3) = x(index+3);
                }
            }
        }
    }

    // 6. 计算拟合误差

    
    is_fitted_ = true;
    fitting_error_ = 0.0;
    fitted_points_.clear();
    for (size_t i = 0; i < points.size(); ++i) {
        Eigen::Vector3d fitted = evaluate(t_values_[i]);
        fitting_error_ += (points[i]-fitted).norm();
        fitted_points_.push_back(fitted);
        // if (i % 10 == 0) {
        //     std::cout << " fitted " << fitted.transpose() << std::endl;
        // }
    }
    // for (int i = 0; i < fitted_points_.size(); i++) {
    //     if(i % 10 == 0) {
    //         std::cout <<"qzc " << fitted_points_[i].transpose() << std::endl;
    //     }
    // }
    fitting_error_ /= points.size();


    return true;
}

std::vector<double> SplineInterpolator::computeParameterization(const std::vector<Eigen::Vector3d>& points) {
    size_t n = points.size();
    std::vector<double> t_values(n);
    std::vector<double> distances(n);

    distances[0] = 0.0;
    for (size_t i = 1; i < n; ++i) {
        double dist = (points[i]-points[i-1]).norm();
        if (params_.use_chord_length) {
            distances[i] = distances[i-1] + dist;
        } else {
            // 向心参数化
            distances[i] = distances[i-1] + std::pow(dist, params_.alpha);
        }
    }

    double total_length = distances[n-1];
    if (total_length < 1e-10) {
        // 所有点都重合，使用均匀参数化
        for (size_t i = 0; i < n; ++i) {
            t_values[i] = static_cast<double>(i) / (n - 1);
        }
    } else {
        for (size_t i = 0; i < n; ++i) {
            t_values[i] = distances[i] / total_length;
        }
    }

    return t_values;
}

void SplineInterpolator::buildObjectiveMatrices(const std::vector<Eigen::Vector3d>& points,
                                               const std::vector<double>& t_values,
                                               Eigen::MatrixXd& Q,
                                               Eigen::VectorXd& q) {
    Q.setZero(12, 12);
    q.setZero(12);

    for (size_t i = 0; i < points.size(); ++i) {
        double t = t_values[i];
        Eigen::Vector4d B = basisFunction(t);

        // 对于每个维度 (x, y, z)
        for (int dim = 0; dim < 3; ++dim) {
            int offset = dim * 4;

            Q.block<4, 4>(offset, offset) += params_.lambda1 * B * B.transpose();

            double coord = (dim == 0) ? points[i].x() : (dim == 1) ? points[i].y() : points[i].z();
            q.segment<4>(offset) += params_.lambda1 * coord * B;
        }
    }

    Eigen::Matrix4d G = computeSmoothnessMatrix();
    for (int dim = 0; dim < 3; ++dim) {
        int offset = dim * 4;
        Q.block<4, 4>(offset, offset) += params_.lambda2 * G;
    }

    double regularization = 1e-8;
    for (int i = 0; i < 12; ++i) {
        Q(i, i) += regularization;
    }
}

void SplineInterpolator::buildConstraintMatrices(const std::vector<Eigen::Vector3d>& points,
                                                const std::vector<double>& t_values,
                                                const Eigen::Vector3d& start_direction,
                                                const Eigen::Vector3d& end_direction,
                                                Eigen::MatrixXd& A,
                                                Eigen::VectorXd& b) {

    int num_constraints_start_end = 12;  //  6个方向约束 + 6个位置约束
    
    // 计算需要的约束数量
    int num_key_points = 2; // 默认是起点和终点
    int num_constraints_mid_points = 0; 
    if(params_.use_all_pnt_constrain == true) {
        std::cout << "use use_all_pnt_constrain" << std::endl;
        num_key_points = std::min(3, (int)points.size()); // 选择前3个关键点
        num_constraints_mid_points = (num_key_points-2) * 3;  // 关键点约束(去掉起始和结束位置)
    }
    
    int num_constraints = num_constraints_start_end + num_constraints_mid_points; // 原有12个约束(6个位置约束 + 6个方向约束) + 关键点约束

    std::cout << "total constraints: " << num_constraints << std::endl;

    A.setZero(num_constraints, 12);
    b.setZero(num_constraints);

    // 端点朝向约束
    Eigen::Vector4d Bp0 = basisFunctionDerivative(0.0);
    Eigen::Vector4d Bp1 = basisFunctionDerivative(1.0);

    // 起始点朝向约束
    A.block<1, 4>(0, 0) = Bp0.transpose();   // x分量
    A.block<1, 4>(1, 4) = Bp0.transpose();   // y分量
    A.block<1, 4>(2, 8) = Bp0.transpose();   // z分量
    b(0) = start_direction.x();
    b(1) = start_direction.y();
    b(2) = start_direction.z();

    // 终点朝向约束
    A.block<1, 4>(3, 0) = Bp1.transpose();   // x分量
    A.block<1, 4>(4, 4) = Bp1.transpose();  // y分量
    A.block<1, 4>(5, 8) = Bp1.transpose();  // z分量
    b(3) = end_direction.x();
    b(4) = end_direction.y();
    b(5) = end_direction.z();

    // 端点位置约束
    Eigen::Vector4d B0 = basisFunction(0.0);
    Eigen::Vector4d B1 = basisFunction(1.0);

    // 起始点位置约束
    A.block<1, 4>(6, 0) = B0.transpose();  // x分量
    A.block<1, 4>(7, 4) = B0.transpose();  // y分量
    A.block<1, 4>(8, 8) = B0.transpose();  // z分量
    b(6) = points[0].x();
    b(7) = points[0].y();
    b(8) = points[0].z();

    // 终点位置约束
    A.block<1, 4>(9, 0) = B1.transpose();  // x分量
    A.block<1, 4>(10, 4) = B1.transpose();  // y分量
    A.block<1, 4>(11, 8) = B1.transpose();  // z分量
    b(9) = points.back().x();
    b(10) = points.back().y();
    b(11) = points.back().z();

    // 添加中间关键点约束
    for (int i = 1; i < num_key_points - 1; ++i) {  // 跳过起点和终点
        double t = t_values[i];
        Eigen::Vector4d B = basisFunction(t);

        int constraint_idx = num_constraints_start_end + (i-1) * 3;
        A.block<1, 4>(constraint_idx, 0) = B.transpose();    // x
        A.block<1, 4>(constraint_idx+1, 4) = B.transpose();  // y
        A.block<1, 4>(constraint_idx+2, 8) = B.transpose();  // z

        b(constraint_idx) = points[i].x();
        b(constraint_idx+1) = points[i].y();
        b(constraint_idx+2) = points[i].z();
    }
}

Eigen::Vector4d SplineInterpolator::basisFunction(double t) const {
    return Eigen::Vector4d(1.0, t, t*t, t*t*t);
}

Eigen::Vector4d SplineInterpolator::basisFunctionDerivative(double t) const {
    return Eigen::Vector4d(0.0, 1.0, 2.0*t, 3.0*t*t);
}

Eigen::Vector4d SplineInterpolator::basisFunctionSecondDerivative(double t) const {
    return Eigen::Vector4d(0.0, 0.0, 2.0, 6.0*t);
}

Eigen::Matrix4d SplineInterpolator::computeSmoothnessMatrix() const {
    Eigen::Matrix4d G;
    G.setZero();

    G(2, 2) = 4.0; 
    G(2, 3) = 6.0; 
    G(3, 2) = 6.0; 
    G(3, 3) = 12.0;

    return G;
}

bool SplineInterpolator::solveQP(const Eigen::MatrixXd& Q, const Eigen::VectorXd& q,
                                 const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                                 Eigen::VectorXd& solution) {
    int n = Q.rows();
    int m = A.rows();

    std::string name("check_condition");
    checkMatrixCondition(Q, name);

    OSQPData* data = (OSQPData*)c_malloc(sizeof(OSQPData));
    OSQPSettings* settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));

    if (!data || !settings) {
        return false;
    }

    // 设置默认参数
    osqp_set_default_settings(settings);
    settings->verbose = 0;
    // settings->eps_abs = 1e-6;
    // settings->eps_rel = 1e-6;

    // Q矩阵
    std::vector<c_float> Q_data;
    std::vector<c_int> Q_indices;
    std::vector<c_int> Q_indptr;

    Q_indptr.push_back(0);
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i <= j; ++i) {  // 只存储上三角部分
            if (std::abs(Q(i, j)) > 1e-12) {
                Q_data.push_back(Q(i, j));
                Q_indices.push_back(i);

                // std::cout << Q(i, j) << " ";
            }
        }
        // std::cout << std::endl;
        Q_indptr.push_back(Q_data.size());
    }

    // A矩阵
    std::vector<c_float> A_data;
    std::vector<c_int> A_indices;
    std::vector<c_int> A_indptr;

    A_indptr.push_back(0);
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < m; ++i) {
            if (std::abs(A(i, j)) > 1e-12) {
                A_data.push_back(A(i, j));
                A_indices.push_back(i);
            }
        }
        A_indptr.push_back(A_data.size());
    }

    // q向量
    std::vector<c_float> q_data(n);
    for (int i = 0; i < n; ++i) {
        q_data[i] = -q(i);
    }

    c_float l_th = 0;
    c_float u_th = 0;
    std::vector<c_float> l_data(m), u_data(m);
    for (int i = 0; i < m; ++i) {
        if(i >= 12) {
            l_th = -0.5;
            u_th = 0.5;
        }
        l_data[i] = b(i)+l_th;
        u_data[i] = b(i)+u_th;

    }

    // 设置OSQP数据
    data->n = n;
    data->m = m;
    data->P = csc_matrix(n, n, Q_data.size(), Q_data.data(), Q_indices.data(), Q_indptr.data());
    data->q = q_data.data();
    data->A = csc_matrix(m, n, A_data.size(), A_data.data(), A_indices.data(), A_indptr.data());
    data->l = l_data.data();
    data->u = u_data.data();

    // 验证数据
    if (!validateOSQPData(data)) {
        std::cerr << "OSQP数据验证失败" << std::endl;
        // 清理内存
        if (data->P) c_free(data->P); //csc_spfree(data->P);
        if (data->A) c_free(data->A); //csc_spfree(data->A);
        c_free(data);
        c_free(settings);
        return false;
    }

    // 创建工作空间
    OSQPWorkspace* work = nullptr;
    c_int exitflag = osqp_setup(&work, data, settings);

    switch (exitflag) {
        case OSQP_DATA_VALIDATION_ERROR:
            std::cerr << "OSQP 数据验证错误" << std::endl;
            break;
        case OSQP_SETTINGS_VALIDATION_ERROR:
            std::cerr << "OSQP 设置验证错误" << std::endl;
            break;
        case OSQP_LINSYS_SOLVER_INIT_ERROR:
            std::cerr << "OSQP 线性系统求解器初始化错误" << std::endl;
            break;
        case OSQP_NONCVX_ERROR:
            std::cerr << "OSQP 非凸问题错误" << std::endl;
            break;
        default:
            if (exitflag != 0) {
                std::cerr << "OSQP setup 失败，错误代码: " << exitflag << std::endl;
            }
            break;
    }

    if (exitflag != 0) {
        // std::cerr << "1 OSQP setup failed with exit flag: " << exitflag << std::endl;
        // return false;
        std::cerr << "OSQP setup失败，错误代码: " << exitflag << std::endl;
        // 清理内存
        if (data->P) c_free(data->P); //csc_spfree(data->P);
        if (data->A) c_free(data->A); //csc_spfree(data->A);
        c_free(data);
        c_free(settings);
        return false;
    }

    // 求解
    exitflag = osqp_solve(work);

    if (exitflag != 0) {
        // std::cerr << "2 OSQP solve failed with exit flag: " << exitflag << std::endl;
        // osqp_cleanup(work);
        // return false;
        std::cerr << "OSQP solve失败，错误代码: " << exitflag << std::endl;
        osqp_cleanup(work);
        if (data->P) c_free(data->P); //csc_spfree(data->P);
        if (data->A) c_free(data->A); //csc_spfree(data->A);
        c_free(data);
        c_free(settings);
        return false;
    }

    // 提取解
    solution.resize(n);
    for (int i = 0; i < n; ++i) {
        solution(i) = work->solution->x[i];
    }

    // 清理
    osqp_cleanup(work);
    // delete data;
    // delete settings;
    // TODO:下面两行有重复释放,错误如下
    //      smooth_line2(41300,0x117de8e00) malloc: *** error for object 0x7fd92d406050: pointer being freed was not allocated
    //      smooth_line2(41300,0x117de8e00) malloc: *** set a breakpoint in malloc_error_break to debug
    if (data->P) c_free(data->P); //csc_spfree(data->P);
    if (data->A) c_free(data->A); //csc_spfree(data->A);
    c_free(data);
    c_free(settings);

    return true;
}

Eigen::Vector3d SplineInterpolator::evaluate(double t) const {
    if (!is_fitted_) {
        // std::cerr << "evaluate: 样条曲线尚未拟合" << std::endl;
        return Eigen::Vector3d();
    }

    Eigen::Vector4d B = basisFunction(t);
    Eigen::Vector3d result;
    result.x() = coefficients_.row(0).dot(B);
    result.y() = coefficients_.row(1).dot(B);
    result.z() = coefficients_.row(2).dot(B);

    return result;
}

Eigen::Vector3d SplineInterpolator::evaluateDerivative(double t) const {
    if (!is_fitted_) {
        std::cerr << "evaluateDerivative: 样条曲线尚未拟合" << std::endl;
        return Eigen::Vector3d();
    }

    Eigen::Vector4d Bp = basisFunctionDerivative(t);
    Eigen::Vector3d result;
    result.x() = coefficients_.row(0).dot(Bp);
    result.y() = coefficients_.row(1).dot(Bp);
    result.z() = coefficients_.row(2).dot(Bp);

    return result;
}

Eigen::Vector3d SplineInterpolator::evaluateSecondDerivative(double t) const {
    if (!is_fitted_) {
        std::cerr << "evaluateSecondDerivative: 样条曲线尚未拟合" << std::endl;
        return Eigen::Vector3d();
    }

    Eigen::Vector4d Bpp = basisFunctionSecondDerivative(t);
    Eigen::Vector3d result;
    result.x() = coefficients_.row(0).dot(Bpp);
    result.y() = coefficients_.row(1).dot(Bpp);
    result.z() = coefficients_.row(2).dot(Bpp);

    return result;
}

// 辅助函数：检查矩阵条件数
void SplineInterpolator::checkMatrixCondition(const Eigen::MatrixXd& matrix, const std::string& name) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
    double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
    std::cout << name << " 条件数: " << cond << std::endl;

    if (cond > 1e12) {
        std::cout << "警告: " << name << " 条件数过大，可能存在数值不稳定" << std::endl;
    }
}

// 补充：求解约束二次规划问题的函数
bool SplineInterpolator::solveConstrainedQP(const Eigen::MatrixXd& Q,
                                           const Eigen::VectorXd& q,
                                           const Eigen::MatrixXd& A,
                                           const Eigen::VectorXd& b,
                                           Eigen::VectorXd& x) {
    int n = Q.rows();  // 变量数量
    int m = A.rows();  // 约束数量

    Eigen::MatrixXd KKT(n + m, n + m);
    Eigen::VectorXd rhs(n + m);

    KKT.block(0, 0, n, n) = Q;
    KKT.block(0, n, n, m) = A.transpose();
    KKT.block(n, 0, m, n) = A;
    KKT.block(n, n, m, m).setZero();

    rhs.head(n) = -q;
    rhs.tail(m) = b;

    Eigen::VectorXd solution = KKT.ldlt().solve(rhs);

    if (solution.size() == 0) {
        return false;  // 求解失败
    }

    x = solution.head(n);  // 提取变量解
    return true;
}

bool SplineInterpolator::validateOSQPData(OSQPData* data) {
    // 检查维度
    if (data->n <= 0 || data->m < 0) {
        std::cerr << "无效的维度: n=" << data->n << ", m=" << data->m << std::endl;
        return false;
    }

    // 检查 P 矩阵（应该是半正定的）
    if (data->P->m != data->n || data->P->n != data->n) {
        std::cerr << "P 矩阵维度不匹配" << std::endl;
        return false;
    }

    // 检查 A 矩阵
    if (data->A->m != data->m || data->A->n != data->n) {
        std::cerr << "A 矩阵维度不匹配" << std::endl;
        return false;
    }

    // 检查约束边界
    for (int i = 0; i < data->m; i++) {
        if (data->l[i] > data->u[i]) {
            std::cerr << "约束边界错误: l[" << i << "] > u[" << i << "]" << std::endl;
            return false;
        }
    }


    // 检查CSC格式
    for (int j = 0; j < data->P->n; j++) {
        for (int i = data->P->p[j]; i < data->P->p[j+1]; i++) {
            if (data->P->i[i] < 0 || data->P->i[i] >= data->P->m) {
                printf("行索引越界\n");
            }
        }
    }

    // 检查是否有NaN或无穷大值
    for (int i = 0; i < data->P->nzmax; i++) {
        if (isnan(data->P->x[i]) || isinf(data->P->x[i])) {
            printf("P矩阵包含NaN或无穷大值\n");
        }
    }

    return true;
}

std::vector<Eigen::Vector3d> SplineInterpolator::sample_pnts(int num_samples)
{
    if(num_samples == 1){
        return {Eigen::Vector3d(0, 0, 0)};
    }
    // int num_samples = 100;
    std::vector<Eigen::Vector3d> res;
    for (int i = 0; i <= num_samples; ++i) {
        double t = static_cast<double>(i) / num_samples;
        Eigen::Vector3d pos = evaluate(t);
        res.push_back(pos);
        // Eigen::Vector3d vel = interpolator.evaluateDerivative(t);
        // Eigen::Vector3d acc = interpolator.evaluateSecondDerivative(t);
        // outfile << t << " "
        //         << pos.x() << " " << pos.y() << " " << pos.z() << " "
        //         << vel.x() << " " << vel.y() << " " << vel.z() << " "
        //         << acc.x() << " " << acc.y() << " " << acc.z() << std::endl;
    }
    return res;
}

std::vector<Eigen::Vector3d> SplineInterpolator::resample_pnts()
{
    std::vector<Eigen::Vector3d> res = {};

    fitting_error_ = 0.0;
    for (int i = 0; i < control_points_.size(); ++i) {
        Eigen::Vector3d fitted = evaluate(t_values_[i]);
        fitting_error_ += (control_points_[i] - fitted).norm();
        res.push_back(fitted);
    }
    fitting_error_ /= control_points_.size();

    LOG_INFO("fitting_error_: {}", fitting_error_);

    return res;
}

// 获取曲线的边界框
void SplineInterpolator::getBoundingBox(Eigen::Vector3d& min_pt, Eigen::Vector3d& max_pt) const {
    if (control_points_.empty()) return;

    min_pt = max_pt = control_points_[0];
    for (const auto& pt : control_points_) {
        min_pt.x() = std::min(min_pt.x(), pt.x());
        min_pt.y() = std::min(min_pt.y(), pt.y());
        min_pt.z() = std::min(min_pt.z(), pt.z());
        max_pt.x() = std::max(max_pt.x(), pt.x());
        max_pt.y() = std::max(max_pt.y(), pt.y());
        max_pt.z() = std::max(max_pt.z(), pt.z());
    }

    // 也考虑生成的曲线点
    for (const auto& pt : fitted_points_visual_) {
        min_pt.x() = std::min(min_pt.x(), pt.x());
        min_pt.y() = std::min(min_pt.y(), pt.y());
        min_pt.z() = std::min(min_pt.z(), pt.z());
        max_pt.x() = std::max(max_pt.x(), pt.x());
        max_pt.y() = std::max(max_pt.y(), pt.y());
        max_pt.z() = std::max(max_pt.z(), pt.z());
    }
}


// 2D可视化 (XY平面投影)
void SplineInterpolator::visualize2D(const std::string& window_name,
                    int width, int height) {
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);

    if (control_points_.empty()) {
        cv::putText(image, "No control points", cv::Point(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::imshow(window_name, image);
        return;
    }

    // std::cout << "1 fitted_points_: " << fitted_points_.size() << std::endl;
    // for (int i = 0; i < t_values_.size(); i++) {
    //     if(i % 8 == 0) {
    //         std::cout<<"t_values_: " << t_values_[i]<< std::endl;
    //     }
    // }
    // for (int i = 0; i < fitted_points_.size(); i++) {
    //     if(i % 10 == 0) {
    //         std::cout << fitted_points_[i].transpose() << std::endl;
    //     }
    // }
    
    if(fitted_points_.size() <= 2) {
        fitted_points_visual_ = sample_pnts(100);
    } else {
        fitted_points_visual_ = fitted_points_;
    }

    // 计算边界框
    Eigen::Vector3d min_pt, max_pt;
    getBoundingBox(min_pt, max_pt);

    // 计算缩放和偏移
    double range_x = max_pt.x() - min_pt.x();
    double range_y = max_pt.y() - min_pt.y();
    double max_range = std::max(range_x, range_y);

    if (max_range == 0) max_range = 1;

    double scale = std::min(width, height) * 0.8 / max_range;
    double offset_x = width / 2.0 - (min_pt.x() + max_pt.x()) * scale / 2.0;
    double offset_y = height / 2.0 - (min_pt.y() + max_pt.y()) * scale / 2.0;


    // 绘制曲线
    if (fitted_points_visual_.size() > 1) {
        std::vector<cv::Point> curve_2d;
        for (const auto& pt : fitted_points_visual_) {
            int x = static_cast<int>(pt.x() * scale + offset_x);
            int y = static_cast<int>(pt.y() * scale + offset_y);
            curve_2d.push_back(cv::Point(x, y));
        }

        for (size_t i = 1; i < curve_2d.size(); ++i) {
            cv::line(image, curve_2d[i-1], curve_2d[i], cv::Scalar(0, 255, 0), 5);
        }
    }

    // 绘制控制点
    for (size_t i = 0; i < control_points_.size(); ++i) {
        int x = static_cast<int>(control_points_[i].x() * scale + offset_x);
        int y = static_cast<int>(control_points_[i].y() * scale + offset_y);
        cv::circle(image, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);

        // 标注控制点序号
        cv::putText(image, std::to_string(i), cv::Point(x + 10, y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }

    // 添加坐标轴
    cv::line(image, cv::Point(50, height-50), cv::Point(150, height-50), cv::Scalar(255, 255, 255), 2);
    cv::line(image, cv::Point(50, height-50), cv::Point(50, height-150), cv::Scalar(255, 255, 255), 2);
    cv::putText(image, "X", cv::Point(155, height-40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    cv::putText(image, "Y", cv::Point(40, height-155), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

    cv::imshow(window_name, image);
}

// 3D可视化 (等轴测投影)
void SplineInterpolator::visualize3D(const std::string& window_name,
                    int width, int height) const {
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);

    if (control_points_.empty()) {
        cv::putText(image, "No control points", cv::Point(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::imshow(window_name, image);
        return;
    }

    // 计算边界框和中心点
    Eigen::Vector3d min_pt, max_pt;
    getBoundingBox(min_pt, max_pt);
    Eigen::Vector3d center((min_pt.x() + max_pt.x()) / 2, (min_pt.y() + max_pt.y()) / 2, (min_pt.z() + max_pt.z()) / 2);

    double max_range = std::max({max_pt.x() - min_pt.x(), max_pt.y() - min_pt.y(), max_pt.z() - min_pt.z()});
    if (max_range == 0) max_range = 1;
    double scale = std::min(width, height) * 0.6 / max_range;

    // 创建等轴测投影矩阵
    cv::Mat projection_matrix = createProjectionMatrix("isometric", cv::Size(width, height), center, scale);

    // 绘制坐标轴
    drawVector(image, center, Eigen::Vector3d(max_range * 0.3, 0, 0), projection_matrix, cv::Scalar(0, 0, 255), 3);
    drawVector(image, center, Eigen::Vector3d(0, max_range * 0.3, 0), projection_matrix, cv::Scalar(0, 255, 0), 3);
    drawVector(image, center, Eigen::Vector3d(0, 0, max_range * 0.3), projection_matrix, cv::Scalar(255, 0, 0), 3);

    // 绘制曲线
    if (!fitted_points_visual_.empty()) {
        drawCurve(image, fitted_points_visual_, projection_matrix, cv::Scalar(255, 255, 0), 2);
    }

    // 绘制控制点
    for (size_t i = 0; i < control_points_.size(); ++i) {
        drawPoint(image, control_points_[i], projection_matrix, cv::Scalar(0, 255, 255), 6);

        // 标注控制点序号
        cv::Point2f pt_2d = project3Dto2D(control_points_[i], projection_matrix);
        cv::putText(image, std::to_string(i), cv::Point(pt_2d.x + 10, pt_2d.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }

    // 添加标签
    cv::putText(image, "X", cv::Point(width - 150, height - 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    cv::putText(image, "Y", cv::Point(width - 120, height - 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    cv::putText(image, "Z", cv::Point(width - 90, height - 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);

    cv::imshow(window_name, image);
}

// 多视图投影可视化
void SplineInterpolator::visualizeProjections(const std::string& window_name,
                            int width, int height) const {
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);

    if (control_points_.empty()) {
        cv::putText(image, "No control points", cv::Point(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::imshow(window_name, image);
        return;
    }

    // 计算边界框和中心点
    Eigen::Vector3d min_pt, max_pt;
    getBoundingBox(min_pt, max_pt);
    Eigen::Vector3d center((min_pt.x() + max_pt.x()) / 2, (min_pt.y() + max_pt.y()) / 2, (min_pt.z() + max_pt.z()) / 2);

    double max_range = std::max({max_pt.x() - min_pt.x(), max_pt.y() - min_pt.y(), max_pt.z() - min_pt.z()});
    if (max_range == 0) max_range = 1;
    double scale = height * 0.7 / max_range;

    int view_width = width / 3;

    // XY视图 (俯视)
    cv::Mat xy_projection = createProjectionMatrix("xy", cv::Size(view_width, height), center, scale);
    drawCurve(image, fitted_points_visual_, xy_projection, cv::Scalar(255, 255, 0), 2);
    for (size_t i = 0; i < control_points_.size(); ++i) {
        drawPoint(image, control_points_[i], xy_projection, cv::Scalar(0, 255, 255), 4);
    }
    cv::putText(image, "XY View (Top)", cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

    // XZ视图 (正视)
    cv::Mat xz_projection = createProjectionMatrix("xz", cv::Size(view_width, height), center, scale);
    xz_projection.at<double>(0, 2) += view_width; // 偏移到中间
    drawCurve(image, fitted_points_visual_, xz_projection, cv::Scalar(255, 255, 0), 2);
    for (size_t i = 0; i < control_points_.size(); ++i) {
        drawPoint(image, control_points_[i], xz_projection, cv::Scalar(0, 255, 255), 4);
    }
    cv::putText(image, "XZ View (Front)", cv::Point(view_width + 10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

    // YZ视图 (侧视)
    cv::Mat yz_projection = createProjectionMatrix("yz", cv::Size(view_width, height), center, scale);
    yz_projection.at<double>(0, 2) += 2 * view_width; // 偏移到右边
    drawCurve(image, fitted_points_visual_, yz_projection, cv::Scalar(255, 255, 0), 2);
    for (size_t i = 0; i < control_points_.size(); ++i) {
        drawPoint(image, control_points_[i], yz_projection, cv::Scalar(0, 255, 255), 4);
    }
    cv::putText(image, "YZ View (Side)", cv::Point(2 * view_width + 10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

    // 添加分割线
    cv::line(image, cv::Point(view_width, 0), cv::Point(view_width, height), cv::Scalar(100, 100, 100), 1);
    cv::line(image, cv::Point(2 * view_width, 0), cv::Point(2 * view_width, height), cv::Scalar(100, 100, 100), 1);

    cv::imshow(window_name, image);
}

// 3D点投影到2D
cv::Point2f SplineInterpolator::project3Dto2D(const Eigen::Vector3d& point, const cv::Mat& projection_matrix) const {
    // 使用齐次坐标
    cv::Mat point_3d = (cv::Mat_<double>(4, 1) << point.x(), point.y(), point.z(), 1.0);
    cv::Mat point_2d = projection_matrix * point_3d;

    return cv::Point2f(point_2d.at<double>(0, 0), point_2d.at<double>(1, 0));
}

// 绘制曲线
void SplineInterpolator::drawCurve(cv::Mat& image, const std::vector<Eigen::Vector3d>& points,
                const cv::Mat& projection_matrix, const cv::Scalar& color, int thickness) const {
    if (points.size() < 2) return;

    std::vector<cv::Point> curve_2d;
    for (const auto& pt : points) {
        cv::Point2f pt_2d = project3Dto2D(pt, projection_matrix);
        curve_2d.push_back(cv::Point(static_cast<int>(pt_2d.x), static_cast<int>(pt_2d.y)));
    }

    for (size_t i = 1; i < curve_2d.size(); ++i) {
        // 检查点是否在图像范围内
        if (curve_2d[i-1].x >= 0 && curve_2d[i-1].x < image.cols &&
            curve_2d[i-1].y >= 0 && curve_2d[i-1].y < image.rows &&
            curve_2d[i].x >= 0 && curve_2d[i].x < image.cols &&
            curve_2d[i].y >= 0 && curve_2d[i].y < image.rows) {
            cv::line(image, curve_2d[i-1], curve_2d[i], color, thickness);
        }
    }
}

// 绘制点
void SplineInterpolator::drawPoint(cv::Mat& image, const Eigen::Vector3d& point,
                const cv::Mat& projection_matrix, const cv::Scalar& color, int radius) const {
    cv::Point2f pt_2d = project3Dto2D(point, projection_matrix);
    cv::Point pt_int(static_cast<int>(pt_2d.x), static_cast<int>(pt_2d.y));

    // 检查点是否在图像范围内
    if (pt_int.x >= 0 && pt_int.x < image.cols && pt_int.y >= 0 && pt_int.y < image.rows) {
        cv::circle(image, pt_int, radius, color, -1);
    }
}

// 绘制向量
void SplineInterpolator::drawVector(cv::Mat& image, const Eigen::Vector3d& start, const Eigen::Vector3d& vector,
                const cv::Mat& projection_matrix, const cv::Scalar& color, int thickness) const {
    Eigen::Vector3d end(start.x() + vector.x(), start.y() + vector.y(), start.z() + vector.z());

    cv::Point2f start_2d = project3Dto2D(start, projection_matrix);
    cv::Point2f end_2d = project3Dto2D(end, projection_matrix);

    cv::Point start_int(static_cast<int>(start_2d.x), static_cast<int>(start_2d.y));
    cv::Point end_int(static_cast<int>(end_2d.x), static_cast<int>(end_2d.y));

    // 检查点是否在图像范围内
    if (start_int.x >= 0 && start_int.x < image.cols && start_int.y >= 0 && start_int.y < image.rows &&
        end_int.x >= 0 && end_int.x < image.cols && end_int.y >= 0 && end_int.y < image.rows) {
        cv::line(image, start_int, end_int, color, thickness);

        // 绘制箭头
        double angle = std::atan2(end_int.y - start_int.y, end_int.x - start_int.x);
        int arrow_length = 10;
        cv::Point arrow1(end_int.x - arrow_length * std::cos(angle - M_PI/6),
                        end_int.y - arrow_length * std::sin(angle - M_PI/6));
        cv::Point arrow2(end_int.x - arrow_length * std::cos(angle + M_PI/6),
                        end_int.y - arrow_length * std::sin(angle + M_PI/6));
        cv::line(image, end_int, arrow1, color, thickness);
        cv::line(image, end_int, arrow2, color, thickness);
    }
}

// 创建投影矩阵
cv::Mat SplineInterpolator::createProjectionMatrix(const std::string& view_type,
                                const cv::Size& image_size,
                                const Eigen::Vector3d& center, double scale) const {
    cv::Mat projection_matrix = cv::Mat::eye(4, 4, CV_64F);

    if (view_type == "xy") {
        // XY平面投影 (俯视图)
        projection_matrix = (cv::Mat_<double>(4, 4) <<
            scale, 0, 0, image_size.width / 2.0 - center.x() * scale,
            0, -scale, 0, image_size.height / 2.0 + center.y() * scale,
            0, 0, 0, 0,
            0, 0, 0, 1);
    }
    else if (view_type == "xz") {
        // XZ平面投影 (正视图)
        projection_matrix = (cv::Mat_<double>(4, 4) <<
            scale, 0, 0, image_size.width / 2.0 - center.x() * scale,
            0, 0, scale, image_size.height / 2.0 - center.z() * scale,
            0, 0, 0, 0,
            0, 0, 0, 1);
    }
    else if (view_type == "yz") {
        // YZ平面投影 (侧视图)
        projection_matrix = (cv::Mat_<double>(4, 4) <<
            0, scale, 0, image_size.width / 2.0 - center.y() * scale,
            0, 0, scale, image_size.height / 2.0 - center.z() * scale,
            0, 0, 0, 0,
            0, 0, 0, 1);
    }
    else if (view_type == "isometric") {
        // 等轴测投影
        double cos30 = std::cos(M_PI / 6);
        double sin30 = std::sin(M_PI / 6);
        projection_matrix = (cv::Mat_<double>(4, 4) <<
            scale * cos30, -scale * cos30, 0, image_size.width / 2.0,
            scale * sin30, scale * sin30, scale, image_size.height / 2.0,
            0, 0, 0, 0,
            0, 0, 0, 1);

        // 应用中心偏移
        projection_matrix.at<double>(0, 3) -= (center.x() * cos30 - center.y() * cos30) * scale;
        projection_matrix.at<double>(1, 3) -= (center.x() * sin30 + center.y() * sin30 + center.z()) * scale;
    }

    return projection_matrix;
}

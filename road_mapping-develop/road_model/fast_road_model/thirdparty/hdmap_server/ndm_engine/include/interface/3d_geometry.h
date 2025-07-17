/*
 * @Description: some 2D/3D geometry libs.
 * @Author: ming01.jian@horizon.ai
 * @Date: 2019-06-15 10:31:40
 * @LastEditTime: 2019-07-17 11:56:00
 * @LastEditors: jinglin.zhang@horizon.ai
 * @copyright Copyright (c) 2020
 */
#ifndef INTERFACE_SMOOTH_PT_3D_GEOMETRY_H_
#define INTERFACE_SMOOTH_PT_3D_GEOMETRY_H_
#include <Eigen/Core>
#include <iomanip>
#include <iostream>
#include <utility>
#include <vector>

namespace lane_smooth {

float compPoint2CubicCurveDis(float x, float y,
                              Eigen::Matrix<float, 4, 1> coeffs);

/**
 * caculate the vertical point from a 3D point to 3D line.
 * @param line_start_pt: the first point of the input line
 * @param line_end_pt: the second point of the input line, and the direction is:
 * line_start_pt -> line_end_pt
 * @param pt_3d: a arbitrary 3D spatial point
 * @param pt_vertical: the vertical point from pt_3d to line[line_start_pt ->
 * line_end_pt].
*/
bool compVerticalPt3D(const Eigen::Matrix<float, 3, 1> &line_start_pt,
                      const Eigen::Matrix<float, 3, 1> &line_end_pt,
                      const Eigen::Matrix<float, 3, 1> &pt_3d,
                      Eigen::Matrix<float, 3, 1> &pt_vertical);

/**
 * Quaternary equation of one variable Sovler.
 * https://baike.baidu.com/item/%E4%B8%80%E5%85%83%E5%9B%9B%E6%AC%A1%E6%96%B9%E7%A8%8B%E6%B1%82%E6%A0%B9%E5%85%AC%E5%BC%8F/10721996?fr=aladdin
 * ax^4 + bx^3 + cx^2 + dx + e = 0;
*/
template <typename Type>
Type quaternaryEquationSolver(Type a, Type b, Type c, Type d, Type e) {
  std::cout << "Quaternary equation coeffs: " << a << ", " << b << ", " << c
            << ", " << d << ", " << e << std::endl;
  Type P = (c * c + 12 * a * e - 3 * b * d) / 9.0;
  Type Q = (27 * a * d * d + 2 * c * c * c + 27 * b * b * e - 72 * a * c * e -
            9 * b * c * d) /
           54.0;
  Type D = std::sqrt(Q * Q - P * P * P);

  std::cout << "P = " << P << "\nQ = " << Q << "\nD = " << D << std::endl;
  Type u = 0.0;
  Type u1 = std::pow(static_cast<double>(Q + D), 1.0 / 3);
  Type u2 = std::pow(static_cast<double>(Q - D), 1.0 / 3);
  if (std::abs(u1) > std::abs(u2))
    u = u1;
  else
    u = u2;

  Type v = 0.0;
  if (u > 0.0 || u < 0.0) v = P / u;

  std::cout << "u1 = " << u1 << "\nu2 = " << u2 << "\nu = " << u
            << "\nv = " << v << std::endl;
  Type root3 = std::sqrt(3);
  std::vector<std::pair<Type, Type>> w_vec;  // w = -1/2 + sqrt(3)/2 * i;
  std::vector<std::pair<Type, Type>>
      eta_vec;  // pair<a, b>; eta = a + b*i = w^(k-1)*u + w^(4-k)*v;
  std::vector<std::pair<Type, Type>> m2_vec;  // m2 = b*b - 8/3 * a*c + 4*a*eta;
  w_vec.reserve(4);
  eta_vec.reserve(3);
  m2_vec.reserve(3);

  // w^0
  w_vec.push_back(std::pair<Type, Type>(1.0, 0.0));
  // w^1
  w_vec.push_back(std::pair<Type, Type>(-0.5, root3 / 2.0));
  // w^2
  w_vec.push_back(std::pair<Type, Type>(-0.5, -root3 / 2.0));
  // w^3
  w_vec.push_back(std::pair<Type, Type>(1.0, 0.0));
  w_vec.resize(4);

  // compute eta and m2
  for (int k = 1; k < 4; ++k) {
    int idx_k_1 = k - 1;  // w^(k-1)
    int idx_4_k = 4 - k;  // w^(4-k)
    std::pair<Type, Type> w_k_1 = w_vec[idx_k_1];
    std::pair<Type, Type> w_4_k = w_vec[idx_4_k];
    Type real = w_k_1.first * u + w_4_k.first * v;
    Type virtua = w_k_1.second * u + w_4_k.second * v;
    eta_vec.push_back(std::pair<Type, Type>(real, virtua));  // eta
    // m2
    real = b * b - (8.0 / 3.0) * a * c + 4.0 * a * real;
    virtua = 4.0 * a * virtua;
    m2_vec.push_back(std::pair<Type, Type>(real, virtua));  // m2
  }
  eta_vec.resize(3);
  m2_vec.resize(3);

  // compute m, m = sqrt(m2);
  std::pair<Type, Type> best_eta, m;
  Type max_mod = 0.0;
  bool flag = false;             // if mod(m) is equal 0.0;
  for (int i = 0; i < 3; ++i) {  // (x + y*i)^2 = A + B*i;
    Type A = m2_vec[i].first;
    Type B = m2_vec[i].second;
    Type x2 = (std::sqrt(A * A + B * B) + A) / 2.0;
    Type y2 = (std::sqrt(A * A + B * B) - A) / 2.0;
    Type x = std::sqrt(x2);  // x = +-(std::sqrt(x2))
                             // 此处x,y的取值有两种情况，他们互为相反数，但是我们只取x为正。
    Type y = std::sqrt(y2);  // y = +-(std::sqrt(y2))
    if (B < 0.0) y = -y;
    if (x2 + y2 > max_mod) {
      m.first = x;
      m.second = y;
      best_eta = eta_vec[i];
      max_mod = x2 + y2;
    }
    if ((x2 + y2) < 1 * e - 8)
      flag = true;
    else
      flag = false;
  }

  // compute S, T
  std::pair<Type, Type> S, T;
  if (flag) {  // |m1|==0 && |m2|==0 && |m3|==0
    // m
    m.first = 0.0;
    m.second = 0.0;
    // S
    S.first = b * b - (8.0 / 3.0) * a * c;
    S.second = 0.0;
    // T
    T.first = 0.0;
    T.second = 0.0;
  } else {
    // S
    S.first = 2 * b * b - (16.0 / 3.0) * a * c - 4 * a * best_eta.first;
    S.second = -4 * a * best_eta.second;
    // T
    Type temp = 8 * a * b * c - 16 * a * a * d - 2 * b * b * b;
    T.first = temp * m.first / max_mod;
    T.second = -temp * m.second / max_mod;
  }

  // compute the four roots of equation: ax^4 + bx^3 + cx^2 + dx + e = 0
  std::vector<Type> temp;  // (-1)^(n/2), n = 1, 2, 3, 4;
  // (-1)^(1/2), n = 1
  temp.push_back(-1.0);
  // (-1)^(1) , n = 2
  temp.push_back(-1.0);
  // (-1)^(3/2), n = 3
  temp.push_back(1.0);
  // (-1)^(2), n =4
  temp.push_back(1.0);
  temp.resize(4);

  // sqrt(S + (-1)^(ceil(0.5*n))*T)
  std::vector<std::pair<Type, Type>> temp_S_T;
  for (int n = 1; n < 5; ++n) {
    Type A = S.first + temp[n - 1] * T.first;  // (x + y*i)^2 = A + B*i;
    Type B = S.second + temp[n - 1] * T.second;
    Type x2 = (std::sqrt(A * A + B * B) + A) / 2.0;
    Type y2 = (std::sqrt(A * A + B * B) - A) / 2.0;
    Type x = std::sqrt(x2);  // x = +-(std::sqrt(x2))
                             // 此处x,y的取值有两种情况，他们互为相反数，但是我们只取x为正。
    Type y = std::sqrt(y2);  // y = +-(std::sqrt(y2))
    if (B < 0.0) y = -y;
    // std::cout << "real = " << real << std::endl;
    Type real =
        (-b + temp[n - 1] * m.first + std::pow(-1, n + 1) * x) / (4.0 * a);
    Type virtua =
        (temp[n - 1] * m.second + std::pow(-1, n + 1) * y) / (4.0 * a);
    std::cout << "real = " << real << std::endl;
    std::cout << "virtua = " << virtua << std::endl << std::endl;
  }
  return 0.0;
}

/**
 * cubic equation solver by ShengJin method
 * a*t^3 + b*t^2 + c*t + d = 0, a!=0
 * https://zh.wikipedia.org/wiki/%E4%B8%89%E6%AC%A1%E6%96%B9%E7%A8%8B#%E7%9B%9B%E9%87%91%E5%85%AC%E5%BC%8F%E6%B3%95
*/
template <typename Type>
void cubicEquationSolver(Type a, Type b, Type c, Type d,
                         std::vector<Type> &root_vec) {
  Type X1, X2, X3;
  bool flag_x1 = false;
  bool flag_x2 = false;
  bool flag_x3 = false;
  Type A, B, C, delt;
  A = b * b - 3 * a * c;
  B = b * c - 9 * a * d;
  C = c * c - 3 * b * d;

  A = pow(b, 2) - 3 * a * c;
  B = b * c - 9 * a * d;
  C = pow(c, 2) - 3 * b * d;
  delt = pow(B, 2) - 4 * A * C;

  std::cout.flags(std::ios::fixed);
  std::cout.precision(9);  //设置输出精度

  // 当A=B=0时，公式1.+定理6
  if ((A == 0 && B == 0) || (delt == 0 && A == 0)) {
    X1 = -b / (3 * a);
    X2 = X1;
    X3 = X1;
    flag_x1 = true;
    flag_x2 = true;
    flag_x3 = true;
    // std::cout << "公式1\n";
    // std::cout << "X1 = " << X1 << ", X2 = "<< X2 << ", X3 = " << X3 <<
    // std::endl;
    root_vec.push_back(X1);
  }

  // Δ=B2－4AC>0时，公式2
  if (delt > 0) {
    Type Y1 = A * b + 3 * a * ((-B + pow(delt, 0.5)) / 2);
    Type Y2 = A * b + 3 * a * ((-B - pow(delt, 0.5)) / 2);
    // 解决负数开三方出问题
    Type Y1_three, Y2_three;
    if (Y1 < 0)
      Y1_three = -pow(-Y1, 1.0 / 3.0);
    else
      Y1_three = pow(Y1, 1.0 / 3.0);
    if (Y2 < 0)
      Y2_three = -pow(-Y2, 1.0 / 3.0);
    else
      Y2_three = pow(Y2, 1.0 / 3.0);
    X1 = (-b - (Y1_three + Y2_three)) / (3 * a);
    // X1,X2为复数，这里不需要，未实现。
    flag_x1 = true;
    // std::cout << "公式2\n";
    // std::cout << "X1 = " << X1  << std::endl;
    root_vec.push_back(X1);
  }

  // 当Δ=B2－4AC=0时，公式3
  if (delt == 0 && (A != 0)) {
    Type K = B / A;
    X1 = -b / a + K;
    X2 = -K / 2.0;
    X3 = X2;
    flag_x1 = true;
    flag_x2 = true;
    flag_x3 = true;
    // std::cout << "公式3\n";
    // std::cout << "X1 = " << X1 << ", X2 = "<< X2 << ", X3 = " << X3 <<
    // std::endl;
    root_vec.push_back(X1);
    root_vec.push_back(X2);
  }

  // 当Δ=B2－4AC<0时，公式4
  if (delt < 0) {
    Type T = (2 * A * b - 3 * a * B) / (2 * A * pow(A, 0.5));
    Type theita = acos(T);
    X1 = (-b - 2 * pow(A, 0.5) * cos(theita / 3.0)) / (3 * a);
    X2 = (-b +
          pow(A, 0.5) * (cos(theita / 3.0) + pow(3, 0.5) * sin(theita / 3.0))) /
         (3 * a);
    X3 = (-b +
          pow(A, 0.5) * (cos(theita / 3.0) - pow(3, 0.5) * sin(theita / 3.0))) /
         (3 * a);
    flag_x1 = true;
    flag_x2 = true;
    flag_x3 = true;
    // std::cout << "公式４\n";
    // std::cout << "X1 = " << X1 << ", X2 = "<< X2 << ", X3 = " << X3 <<
    // std::endl;
    root_vec.push_back(X1);
    root_vec.push_back(X2);
    root_vec.push_back(X3);
  }
}

/**
 *  quintic equation solver by Newton method
 * a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t + f = 0;
 *
 * diverage: fd = 5*a*t^4 + 4*b*t^3 + 3*c*t^2 + 2*d*t + e
*/
template <typename Type>
Type quinticEquationSolverByNewton(Type a, Type b, Type c, Type d, Type e,
                                   Type f) {
  // compute intinial value
  Type t = -f / e;  // 这里实际上要考虑f,e不能为零的情况,
  // 有时候偏差较大，是不是e==0的原因???(初值不够好的原因)
  Type t0, fun, fun_d;
  //  std::cout.flags(std::ios::fixed);
  //  std::cout.precision(9); //设置输出精度
  //  std::cout << "a = " << a
  //          << "\nb = " << b
  //          << "\nc = " << c
  //          << "\nd = " << d
  //          << "\ne = " << e
  //          << "\nf = " << f << std::endl;

  //  std::cout << "\ninitial t = (-f/e) = " << t << std::endl;
  int num = 0;
  while (std::abs(t - t0) >= 1e-6 && num < 100) {
    t0 = t;
    fun = a * std::pow(t0, 5) + b * std::pow(t0, 4) + c * std::pow(t0, 3) +
          d * t0 * t0 + e * t0 + f;
    fun_d = 5 * a * std::pow(t0, 4) + 4 * b * std::pow(t0, 3) +
            3 * c * t0 * t0 + 2 * d * t0 + e;
    t = t0 - fun / fun_d;
    // std::cout << "t0 = "<< t0 << ", fun = "<< fun << ", fun_d = " << fun_d
    // <<", t = " << t << std::endl;
    ++num;
  }
  //  std::cout << "num = " << num << std::endl;
  //  std::cout << "t_arc = " << t << std::endl;

  std::vector<Type> root_vec;
  root_vec.reserve(3);
  cubicEquationSolver(c, d, e, f, root_vec);
  for (auto root : root_vec) {
    //    std::cout << "root = " << root << std::endl;
    if (root < 0.0) continue;
    t = root;
    //    std::cout << "\ninitial t = " << t << std::endl;
    int num = 0;
    while (std::abs(t - t0) >= 1e-6 && num < 100) {
      t0 = t;
      fun = a * std::pow(t0, 5) + b * std::pow(t0, 4) + c * std::pow(t0, 3) +
            d * t0 * t0 + e * t0 + f;
      fun_d = 5 * a * std::pow(t0, 4) + 4 * b * std::pow(t0, 3) +
              3 * c * t0 * t0 + 2 * d * t0 + e;
      t = t0 - fun / fun_d;
      // std::cout << "t0 = "<< t0 << ", fun = "<< fun << ", fun_d = " << fun_d
      // <<", t = " << t << std::endl;
      ++num;
    }
    //    std::cout << "num = " << num << std::endl;
    //    std::cout << "t_arc = " << t << std::endl;
  }
  return t;
}

/**
 * compute the virtical point Pv from 3D point P to 3D line L.
 * 3D line L : direct vector n = [nx, ny, nz]', point P1 = [x1, y1, z1]'
*/
template <typename Type>
Eigen::Matrix<Type, 3, 1> compu3DVirtialPoint(Eigen::Matrix<Type, 3, 1> &P,
                                              Eigen::Matrix<Type, 3, 1> &n,
                                              Eigen::Matrix<Type, 3, 1> &P1) {
  n.normalize();
  Type x, y, z;  // 任意空间点
  x = P(0);
  y = P(1);
  z = P(2);
  Type x1, y1, z1;  // 直线上一点
  x1 = P1(0);
  y1 = P1(1);
  z1 = P1(2);
  Type nx, ny, nz;
  nx = n(0);
  ny = n(1);
  nz = n(2);

  Type k = -((x1 - x) * nx + (y1 - y) * ny + (z1 - z) * nz);

  Eigen::Matrix<Type, 3, 1> vertical_pt;
  vertical_pt(0) = k * nx + x1;
  vertical_pt(1) = k * ny + y1;
  vertical_pt(2) = k * nz + z1;
  return vertical_pt;

  // Eigen::Matrix<Type, 3, 1> &p0,
  // Eigen::Matrix<Type, 3, 1> &p1,
  // Eigen::Matrix<Type, 3, 1> &p2

  //   Type x0, y0, z0; // 任意空间点
  //   x0 = p0(0);
  //   y0 = p0(1);
  //   z0 = p0(2);
  //   Type x1, y1, z1; // 直线上一点
  //   x1 = p1(0);
  //   y1 = p1(1);
  //   z1 = p1(2);
  //   Type x2, y2, z2;
  //   x2 = p2(0);
  //   y2 = p2(1);
  //   z2 = p2(2);

  //   Type k = -((x1-x0)*(x2-x1) + (y1-y0)*(y2-y1) +
  //   (z1-z0)*(z2-z1))/((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1));

  //   Eigen::Matrix<Type, 3, 1> vertical_pt;
  //   vertical_pt(0) = k*(x2-x1) + x1;
  //   vertical_pt(1) = k*(y2-y1) + y1;
  //   vertical_pt(2) = k*(z2-z1) + z1;
  //   return vertical_pt;
}

// /**
//  * compute the virtical point Pv from 3D point P to 3D line L.
//  * 3D line L : point P1 = [x1, y1, z1]', point P2 = [x2, y2, z2]'
// */
// template <typename Type>
// Eigen::Matrix<Type, 3, 1> compu3DVirtialPoint(Eigen::Matrix<Type, 3, 1> &P,
//                                               Eigen::Matrix<Type, 3, 1> &P1,
//                                               Eigen::Matrix<Type, 3, 1> &P2)
//                                               {
//   Eigen::Matrix<Type, 3, 1> n = P2 - P1;
//   compu3DVirtialPoint()
// }

template <typename Type>
Eigen::Matrix<Type, 3, 1> computeNearest3DPoint(Type a, Type b, Type c, Type d,
                                                Type e, Type f,
                                                Eigen::Matrix<Type, 3, 1> P0) {}

}  // end namespace lane_smooth

#endif  // INTERFACE_SMOOTH_PT_3D_GEOMETRY_H_

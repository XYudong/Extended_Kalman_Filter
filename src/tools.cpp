#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * Calculate the RMSE here.
   */
  VectorXd mean_sq_err(4);
  for(auto it_est = estimations.begin(), it_gt = ground_truth.begin(); it_est < estimations.end()
      && it_gt < ground_truth.end(); it_est++, it_gt++) {
    VectorXd err = *it_est - *it_gt;
    err.array().pow(2);
//    std::cout << err << std::endl;
//    std::cout << mean_sq_err << std::endl;
    mean_sq_err += err;
  }
  mean_sq_err.array() / estimations.size();

  return mean_sq_err.cwiseSqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * Calculate a Jacobian here.
   */
   double px = x_state[0];
   double py = x_state[1];
   double vx = x_state[2];
   double vy = x_state[3];

   double xy_sq = px * px + py * py;
   double xy_sqrt = sqrt(xy_sq);

   MatrixXd H_j(3, 4);
   H_j.row(0) << px / xy_sqrt, py/ xy_sqrt, 0, 0;
   H_j.row(1) << -py / xy_sq, px / xy_sq, 0, 0;
   H_j.row(2) << py * (vx * py - vy * px) / (xy_sq * xy_sqrt), px * (vy * px - vx * py) / (xy_sq * xy_sqrt), px / xy_sqrt, py / xy_sqrt;

   return H_j;
}

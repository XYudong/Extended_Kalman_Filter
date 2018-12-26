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
  mean_sq_err << 0, 0, 0, 0;
//  std::cout << "initial total sq err:" << mean_sq_err << std::endl;
  for(auto it_est = estimations.begin(), it_gt = ground_truth.begin(); it_est != estimations.end()
      && it_gt != ground_truth.end(); it_est++, it_gt++) {
    VectorXd err = *it_est - *it_gt;

    for(int i = 0; i < err.size(); i++) {
      err[i] = fabs(err[i]) > 0.0001 ? err[i] : 0.0001;
    }
//    std::cout << "err:\n" << err << std::endl;
    err = err.array() * err.array();
//    std::cout << "sq err:\n" << err << '\n' << std::endl;
//    for(int i = 0; i < err.size(); i++) {
//      if(err[i] < 0 || err[i] > 10000) {
//        std::cout << "Invalid sq err:" << std::endl;
//        std::cout << err << std::endl;
//      }
//    }

    mean_sq_err += err;
//    std::cout << "subtotal sq err:\n" << mean_sq_err << '\n' << std::endl;
  }
  std::cout << "total sq err:\n" << mean_sq_err << '\n' << std::endl;
  mean_sq_err = mean_sq_err.array() / estimations.size();
  std::cout << "MSE:\n" << mean_sq_err << std::endl;

  return mean_sq_err.cwiseSqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * Calculate a Jacobian here.
   */

  MatrixXd H_j(3, 4);
  double px = x_state[0];
  double py = x_state[1];
  double vx = x_state[2];
  double vy = x_state[3];

  double xy_sq = px * px + py * py;
  // a threshold to avoid division by 0
  double EPSILON = 0.0001;
  if (xy_sq < EPSILON) {
   return H_j;
   // i.e. when x,y are too small, just keep H_j as all zeros which makes sense.
  }

  double xy_sqrt = sqrt(xy_sq);
  H_j.row(0) << px / xy_sqrt, py/ xy_sqrt, 0, 0;
  H_j.row(1) << -py / xy_sq, px / xy_sq, 0, 0;
  H_j.row(2) << py * (vx * py - vy * px) / (xy_sq * xy_sqrt), px * (vy * px - vx * py) / (xy_sq * xy_sqrt), px / xy_sqrt, py / xy_sqrt;

  return H_j;
}

#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd accum = VectorXd::Zero(4);
  
  for (int i = 0; i < estimations.size(); i++) {
    VectorXd error = (estimations[i] - ground_truth[i]).array().square();
    accum = accum + error;
  }

  return (accum / estimations.size()).array().sqrt();
}

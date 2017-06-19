#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
typedef CPPAD_TESTVECTOR(double) Dvector;
using namespace std;

class MPC {
 public:

  Dvector vars; // where all the state and actuation variables will be stored

  Dvector vars_lowerbound; //lower limit for each corresponding variable in s
  Dvector vars_upperbound; //upper limit for each corresponding variable in s

  Dvector constraints_lowerbound;
  Dvector constraints_upperbound;

  double steer;
  double throttle;

  std::vector<double> predicted_x_vals;
  std::vector<double> predicted_y_vals;

  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  // vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */

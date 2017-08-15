#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>

using CppAD::AD;
using CppAD::pow;
using CppAD::atan;
using CppAD::cos;
using CppAD::sin;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad2(double x) { return x * pi() / 180; }
double rad2deg2(double x) { return x * 180 / pi(); }


// Set the timestep length and duration
size_t N = 6;
double dt = 0.2;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 40;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector& fg, const ADvector& vars) {
    typedef AD<double> ADd;
    // Computes total COST.
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)

    // Initialize cost to 0
    fg[0] = 0;

    // Reference state cost. Match target values for cte, epsi, and velocity.
    for (int i = 1; i < N; i++)
    {
      fg[0] += 1000 * pow(vars[cte_start + i] - ref_cte, 2);
      fg[0] += 100 * pow(vars[epsi_start + i] - ref_epsi, 2);
      //fg[0] += 0.1/(i+1) * pow(vars[v_start + i] - ref_v, 2);
      fg[0] += 0.1 * pow(vars[v_start + i] - ref_v, 2);
    }

    // Penalize high magnitude steering and acceleration activations.
    for (int i = 0; i < N - 1; i++)
    {
      fg[0] += 0.01 * pow(vars[delta_start + i], 2);
      fg[0] += 0.01 * pow(vars[a_start + i], 2);
    }

    // Penalize high difference in activation between time steps (jerkiness).
    for (int i = 0; i < N - 2; i++)
    {
      fg[0] += 1 * pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += 0.01 * pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }


    // Initial constraints.
    // Add 1 to each starting index due to cost being located at index 0 of 'fg'.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // Implement the dynamics model. For each step, ensure the model at current step t (x0, etc)
    // is bound to the following step t+1 (x1, etc) by the dynamics equations.
    for (int i = 0; i < N - 1; i++)
    {
      // Values at t
      ADd x0 = vars[x_start + i];
      ADd y0 = vars[y_start + i];
      ADd psi0 = vars[psi_start + i];
      ADd v0 = vars[v_start + i];
      ADd cte0 = vars[cte_start + i];
      ADd epsi0 = vars[epsi_start + i];
      ADd delta0 = vars[delta_start + i];
      ADd a0 = vars[a_start + i];

      // Values at t + 1
      ADd x1 = vars[x_start + i + 1];
      ADd y1 = vars[y_start + i + 1];
      ADd psi1 = vars[psi_start + i + 1];
      ADd v1 = vars[v_start + i + 1];
      ADd cte1 = vars[cte_start + i + 1];
      ADd epsi1 = vars[epsi_start + i + 1];

      // Calculate the polynomial value and desired psi
      ADd f0 = coeffs[0] + (coeffs[1] * x0) + (coeffs[2] * x0 * x0) + (coeffs[3] * x0 * x0 * x0);
      ADd psi_des = atan( (3 * coeffs[3] * x0 * x0) + (2 * coeffs[2] * x0) + coeffs[1] );

      // Enforce the model constraints at t + 1 is equal to value at t PLUS the model movements per time step.
      fg[2 + x_start + i] = x1 - (x0 + v0 * cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] = cte1 - (f0 - y0) + (v0 * sin(epsi0) * dt);
      fg[2 + epsi_start + i] = epsi1 - ((psi0 - psi_des) - v0 * delta0 /  Lf * dt);
    }
  }
};




//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

SolveReturns MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  //size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables should be 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuator upper and lower limits to max negative and positive values.
  // x, y, psi, v, cte, epsi. Really big and really small. Don't care to constrain these.
  for (auto i = 0; i < delta_start; i++)
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // Steering angle limit
  for (auto i = delta_start; i < a_start; i++)
  {
    vars_lowerbound[i] = deg2rad2(-25);
    vars_upperbound[i] = deg2rad2(25);
  }

  // Acceleration limit
  for (auto i = a_start; i < n_vars; i++)
  {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  // Lower and upper limits for constraints. Should be 0 besides initial state.
  for (auto i = 0; i < n_constraints; i++)
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Constrain current state (t) to what the real values are.
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

//  auto cost = solution.obj_value;
//  std::cout << "Cost " << cost << std::endl;

  // Bundle up results and send back to caller.
  SolveReturns results;
  results.steer_value = solution.x[delta_start];
  results.throttle_value = solution.x[a_start];

  for (int i = 0; i < N - 1; i++)
  {
    results.x_vals.push_back(solution.x[x_start + i + 1]);
    results.y_vals.push_back(solution.x[y_start + i + 1]);
  }

  return results;
}

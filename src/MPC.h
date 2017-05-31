#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <vector>

typedef CPPAD_TESTVECTOR(double) Dvector;

const int N = 20; // number of predicted states
const double dt = 0.1; // time per state

const double Lf = 2.67; // this is the length from front of vehicle to Center-of-Gravity
const double VELOCITY_MAX = 200.0 * 0.44704; // ideal speed -> mph -> m/s2

const int NUMBER_OF_STATES = 6; // px, py, psi, v, cte, epsi
const int NUMBER_OF_ACTUATIONS = 2; // steering angle, acceleration

const int NX =  N * NUMBER_OF_STATES + (N - 1) * NUMBER_OF_ACTUATIONS; // number of state + actuation variables
const int NG = N * NUMBER_OF_STATES; // number of constraints


// where the first element of each state variable is stored in the vector to be feeded the optimization algorithm
const int ID_FIRST_px = 0;
const int ID_FIRST_py = ID_FIRST_px + N;
const int ID_FIRST_psi = ID_FIRST_py + N;
const int ID_FIRST_v = ID_FIRST_psi + N;
const int ID_FIRST_cte = ID_FIRST_v + N;
const int ID_FIRST_epsi = ID_FIRST_cte + N;
const int ID_FIRST_delta = ID_FIRST_epsi + N;
const int ID_FIRST_a = ID_FIRST_delta + N - 1;

// weights for cost computations
const double W_cte = 1000.0;
const double W_epsi = 30000.0;
const double W_v = 0.50;
const double W_delta = 2.0;
const double W_a = 10.0;
const double W_ddelta = 10000000.0; // weight cost for high difference between consecutive steering actuations
const double W_da = 100000.0; // weight cost for high difference between consecutive acceleration actuations

class MPC {

public:

    double steer;
    double throttle;

    Dvector x; // where all the state and actuation variables will be stored
    Dvector x_lowerbound; //lower limit for each corresponding variable in x
    Dvector x_upperbound; //upper limit for each corresponding variable in x
    Dvector g_lowerbound; // value constraint for each corresponding constraint expression
    Dvector g_upperbound; // value constraint for each corresponding constraint expression

    std::vector<double> future_xs;
    std::vector<double> future_ys;

    MPC();
    virtual ~MPC();

    // this function solves the model given the current state and road curve coefficients.
    void solve(Eigen::VectorXd state, Eigen::VectorXd K);
};

#endif /* MPC_H */
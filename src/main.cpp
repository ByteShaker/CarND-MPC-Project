#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

const int PORT = 4567;

// for convenience
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); ++i) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); ++j) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      string sdata = string(data).substr(0, length);
      //cout << sdata << endl;
      if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
        string s = hasData(sdata);
        if (s != "") {
          auto j = json::parse(s);
          string event = j[0].get<string>();
          if (event == "telemetry") {

            //**************************************************************
            //* GET CURRENT STATE AND CONVERT TO SI
            //**************************************************************

            // j[1] is the data JSON object
            std::vector<double> points_xs = j[1]["ptsx"];
            std::vector<double> points_ys = j[1]["ptsy"];

            const double px = j[1]["x"];
            const double py = j[1]["y"];
            const double psi = j[1]["psi"]; // rad
            const double speed_mph = j[1]["speed"]; // -> m/s
            const double speed_mps = speed_mph * 0.44704;
            const double delta = j[1]["steering_angle"]; // rad (left -> negative)
            const double throttle = j[1]["throttle"];
            const double a = throttle * 5; // 1 -> 5 m/s2

            //**************************************************************
            //* CONVERT TO DELAYED STATE
            //**************************************************************

            const double delay = 100; //ms
            const double Lf = 2.67;

            //Using the kinematic model
            const double delayed_px = px + speed_mps * cos(psi) * delay/1000;
            const double delayed_py = py + speed_mps * sin(psi) * delay/1000;
            const double delayed_psi = psi + (speed_mps * tan(-delta) / Lf) * delay/1000 + ( (a * tan(-delta) / (2*Lf)) * pow(delay/1000,2));
            const double delayed_v = speed_mps + a * delay/1000;

            //**************************************************************
            //* TRANSFORM WAYPOINTS INTO VEHICLE SYSTEM
            //**************************************************************
            const int NUMBER_OF_WAYPOINTS = points_xs.size();
            Eigen::VectorXd waypoints_xs(NUMBER_OF_WAYPOINTS);
            Eigen::VectorXd waypoints_ys(NUMBER_OF_WAYPOINTS);

            for(int i = 0; i < NUMBER_OF_WAYPOINTS; ++i) {

              const double dx = points_xs[i] - delayed_px;
              const double dy = points_ys[i] - delayed_py;

              waypoints_xs[i] = dx * cos(-delayed_psi) - dy * sin(-delayed_psi);
              waypoints_ys[i] = dy * cos(-delayed_psi) + dx * sin(-delayed_psi);
            }

            //**************************************************************
            //* POLYNOMAL FIT WITH 2ND ORDER
            //**************************************************************
            const int ORDER = 2;
            auto K = polyfit(waypoints_xs, waypoints_ys, ORDER);

            //**************************************************************
            //* GENERATE WAYPOINT VISUALISATION FROM POLYEVAL
            //**************************************************************
            std::vector<double> next_xs(N);
            std::vector<double> next_ys(N);
            const double T = 20.0;

            for (int i = 0; i < N; ++i) {

              double dx = T * i;
              double dy = polyeval(K,dx);

              next_xs[i] = dx;
              next_ys[i] = dy;
            }

            //**************************************************************
            //* CALCULATE DELAYED ERROR (cte, epsi)
            //**************************************************************

            // current CTE is fitted polynomial (road curve) evaluated at px = 0.0
            const double cte = K[0];

            // current heading error epsi is the tangent to the road curve at px = 0.0
            const double epsi = -atan(K[1]);


            const int NUMBER_OF_STATES = 6;
            Eigen::VectorXd state(NUMBER_OF_STATES);
            state << 0, 0, 0, delayed_v, cte, epsi;

            //**************************************************************
            //* DETERMINE NEXT COURSE OF ACTION AND PREDICTED STATES
            //* USING MODEL PREDICTIVE CONTROL
            //**************************************************************
            mpc.solve(state, K);

            json msgJson;
            msgJson["steering_angle"] = mpc.steer / 0.43633; //rad -> [-1,1]
            msgJson["throttle"] = mpc.throttle / 3.5; //m/s2 -> [-1,1]

            msgJson["mpc_x"] = mpc.future_xs;
            msgJson["mpc_y"] = mpc.future_ys;

            msgJson["next_x"] = next_xs;
            msgJson["next_y"] = next_ys;

            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            std::cout << mpc.steer << ' ' << mpc.throttle << std::endl;

            // Latency
            this_thread::sleep_for(chrono::milliseconds(int(delay))); //delay is 100 ms

            // sent do simulator
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      }
  });

  // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
      const std::string s = "<h1>Hello world!</h1>";
      if (req.getUrl().valueLength == 1) {
        res->end(s.data(), s.length());
      } else {
        // i guess this should be done more gracefully?
        res->end(nullptr, 0);
      }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
      ws.close();
      std::cout << "Disconnected" << std::endl;
  });

  if (h.listen(PORT)) {
    std::cout << "Listening to port " << PORT << std::endl;
  } else {
    std::cerr << "Failed to listen to port " << PORT << std::endl;
    return -1;
  }
  h.run();
}
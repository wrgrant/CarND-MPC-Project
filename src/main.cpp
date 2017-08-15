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

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          // Shift for left/right opposite from simulator
          delta *= -1;
          double acceleration = j[1]["throttle"];

          // Convert simulator speed in mph to m/s
          v *= 0.447;

          // Predict state forward 100 ms to offset latency.
          double latency = 0.1;
          const double Lf = 2.67;
          px = px + v * cos(psi) * latency;
          py = py + v * sin(psi) * latency;
          v = v + acceleration * latency;
          psi = psi + v * delta / Lf * latency;

          // Shift from map coordinates to car coordinates.
          for (int i = 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;

            ptsx[i] = shift_x * cos(0 - psi) - shift_y * sin(0 - psi);
            ptsy[i] = shift_x * sin(0 - psi) + shift_y * cos(0 - psi);
          }

          // Get fit coefficients for the waypoints line.
          double* ptrx = &ptsx[0];
          double* ptry = &ptsy[0];
          using Eigen::Map;
          using Eigen::VectorXd;
          Map<VectorXd> ptsx_transform(ptrx, 6);
          Map<VectorXd> ptsy_transform(ptry, 6);
          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

          // Calculate cte and epsi
          // Car shifted to [0,0] so evaluate coeffs at x=0 to get the y value which is cte.
          double cte = polyeval(coeffs, 0);
          //double epsi_orig = psi - atan(coeffs[1] + 2*px*coeffs[2] + 3*px*px*coeffs[3]);
          // when px=0 and psi=0 the simplification is as follows:
          double epsi = -atan(coeffs[1]);

          // Convert back to mph for use in the mpc code.
          v /= 0.447;

          VectorXd state(6);
          // state vector: x, y, theta, v, cte, epsi
          state << 0, 0, 0, v, cte, epsi;

          SolveReturns vals = mpc.Solve(state, coeffs);

          json msgJson;
          // Normalize steering value from radians back to [-1 1] range.
          msgJson["steering_angle"] = vals.steer_value/deg2rad(25);
          msgJson["throttle"] = vals.throttle_value;

          //Display the MPC predicted trajectory
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = vals.x_vals;
          msgJson["mpc_y"] = vals.y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          double poly_inc = 2;
          int num_pts = 20;
          for (int i = 1; i < num_pts; i++)
          {
            double x = i * poly_inc;
            next_x_vals.push_back(x);
            next_y_vals.push_back(polyeval(coeffs, x));
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

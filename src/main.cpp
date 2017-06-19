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
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
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
    cout << sdata << endl;
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
          double a = j[1]["throttle"];

          // Convert waypoints to vehicle space
          int num_waypoints = ptsx.size();
          Eigen::VectorXd waypoints_x(num_waypoints);
          Eigen::VectorXd waypoints_y(num_waypoints);

          for(int i = 0; i < num_waypoints; ++i) {
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;

            waypoints_x[i] = shift_x * cos(-psi) - shift_y * sin(-psi);
            waypoints_y[i] = shift_y * cos(-psi) + shift_x * sin(-psi);
          }
          
          // Fit the waypoints with polynomial
          const int order = 3;
          auto coeffs = polyfit(waypoints_x, waypoints_y, order);

          // Use the polynomial and the polyeval function to calculate the current cte
          double cte = polyeval(coeffs, 0);
          double epsi = - atan(coeffs[1]);

          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          // Get the current state
          const double Lf = 2.67;

          // const double delta_t = 0.01;
          // const double current_px = 0.0 + v * delta_t;
          // const double current_py = 0.0;
          // const double current_psi = 0.0 + (v * (-delta) / Lf) * delta_t;
          // const double current_v = v + a * delta_t;
          // const double current_cte = cte + v * sin(epsi) * delta_t;
          // const double current_epsi = epsi + (v * (-delta) / Lf) * delta_t;

          const double current_px = 0.0;
          const double current_py = 0.0;
          const double current_psi = 0.0;
          const double current_v = v;
          const double current_cte = cte;
          const double current_epsi = epsi;

          const int num_states = 6;
          Eigen::VectorXd state(num_states);
          state << current_px, current_py, current_psi, current_v, current_cte, current_epsi;

          mpc.Solve(state, coeffs);
          
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = mpc.steer / (deg2rad(25) * Lf);
          // msgJson["steering_angle"] = mpc.steer;
          msgJson["throttle"] = mpc.throttle;

          //Display the MPC predicted trajectory 
          msgJson["mpc_x"] = mpc.predicted_x_vals;
          msgJson["mpc_y"] = mpc.predicted_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals(num_waypoints);
          vector<double> next_y_vals(num_waypoints);

          double units = 5.0;

          for (int i = 0; i < num_waypoints; ++i) {
            double x_step = units * i;
            double y_step = polyeval(coeffs, x_step);

            next_x_vals[i] = x_step;
            next_y_vals[i] = y_step;
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
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

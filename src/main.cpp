#include <math.h>
#include <uWS/uWS.h>    // uWebSocket
#include <iostream>
#include "json.hpp"
#include "FusionEKF.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;


// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string &s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of('[');
  auto b2 = s.find_first_of(']');
  if (found_null != string::npos) {
    return "";      // means there is a match of "null"
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  } else {
    return "";
  }
}

int main() {
  uWS::Hub h;

  // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  h.onMessage([&fusionEKF,&tools,&estimations,&ground_truth]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          string sensor_measurement = j[1]["sensor_measurement"];

          MeasurementPackage meas_package;
          std::istringstream iss(sensor_measurement);

          long long timestamp;

          // reads first element from the current measurement
          string sensor_type;
          iss >> sensor_type;

          if (sensor_type == "L") {
            meas_package.sensor_type_ = SensorType::LASER;
            meas_package.raw_measurements_ = Eigen::Vector2d(2);    // memory allocation
            float px;
            float py;
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;     // coefficient initialization
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          } else if (sensor_type == "R") {
            meas_package.sensor_type_ = SensorType::RADAR;
            meas_package.raw_measurements_ = Eigen::Vector3d(3);
            float rho;
            float theta;
            float rho_dot;
            iss >> rho;
            iss >> theta;
            iss >> rho_dot;
            meas_package.raw_measurements_ << rho, theta, rho_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          }
          // ground truth
          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;

          VectorXd gt_values(4);
          gt_values << x_gt, y_gt, vx_gt, vy_gt;
          ground_truth.push_back(gt_values);

          // Call ProcessMeasurement(meas_package) for Kalman filter
          fusionEKF.ProcessMeasurement(meas_package);

          // Push the current estimated x,y position from the Kalman filter's state vector

          VectorXd estimate(4);

          double p_x = fusionEKF.ekf_.x_(0);
          double p_y = fusionEKF.ekf_.x_(1);
          double v1  = fusionEKF.ekf_.x_(2);
          double v2 = fusionEKF.ekf_.x_(3);

          estimate(0) = p_x;
          estimate(1) = p_y;
          estimate(2) = v1;
          estimate(3) = v2;

          estimations.push_back(estimate);
          std::cout << "ground truth:\n" << gt_values << std::endl;
          std::cout << "est:\n" << estimate << std::endl;

          VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
          std::cout << "RMSE:\n" << RMSE << std::endl;

          json msgJson;
          msgJson["estimate_x"] = p_x;
          msgJson["estimate_y"] = p_y;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }  // end "telemetry" if

      } else {
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if

  }); // end h.onMessage

//  // We don't need this since we're not using HTTP but if it's removed the program
//  // doesn't compile :-(
//  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
//    const std::string s = "<h1>Hello world!</h1>";
//    if (req.getUrl().valueLength == 1)
//    {
//      res->end(s.data(), s.length());
//    }
//    else
//    {
//      // i guess this should be done more gracefully?
//      res->end(nullptr, 0);
//    }
//  });

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


//int main(){
////  Eigen::MatrixXd m = MatrixXd::Identity(3, 4);
//  MatrixXd m(3,3);
//  Eigen::Matrix3d n = Eigen::MatrixXd::Constant(3, 3, 1);
//  Eigen::VectorXd v(3);
//  v << 1, 2, 3;
//  std::cout << v << std::endl;
//  std::cout << v.array().pow(2) << '\n' << std::endl;
//  std::cout << v << std::endl;
//}
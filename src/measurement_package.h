#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

enum class SensorType {
  LASER,
  RADAR
};

class MeasurementPackage {
 public:
  SensorType sensor_type_;

  long long timestamp_;

  Eigen::VectorXd raw_measurements_;    // default constructor of VectorXd
};

#endif // MEASUREMENT_PACKAGE_H_

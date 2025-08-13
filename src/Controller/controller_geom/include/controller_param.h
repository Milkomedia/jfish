#ifndef CONTROLLER_PARAM_H
#define CONTROLLER_PARAM_H

#include <Eigen/Dense>

namespace controller_param {
  
struct ControlParameters {
  bool use_decoupled_yaw;
  Eigen::Vector3d kX;  // Position gain [x, y, z]
  Eigen::Vector3d kV;  // Velocity gain [x, y, z]
  Eigen::Vector3d kR;  // Rotational gain [roll, pitch, yaw]
  Eigen::Vector3d kW;  // angular Velocity gain [roll, pitch, yaw]
};

struct IntegralParameters {
  bool use_integral;
  double kIX;
  double ki;
  double kIR;
  double kI;
  double kyI;
  double c1;
  double c2;
  double c3;
};

struct UAVParameters {
  Eigen::Matrix3d J;  // inertia
  double m;           // [kg]
  double g;           // [m/s^2]
};


inline ControlParameters getControlParameters() {
  ControlParameters param;
  param.use_decoupled_yaw = true;
  param.kX << 18.0, 18.0, 20.0;
  param.kV << 7.0, 7.0, 5.0;
  param.kR << 20.0, 20.0, 12.5;
  param.kW << 7.0, 7.0, 3.0;
  return param;
}

inline IntegralParameters getIntegralParameters() {
  IntegralParameters param;
  param.kIX = 15.0; //위치값
  
  param.ki = 0.0;  //이용안하는 값
  param.kIR = 0.0; //이용안하는 값
  
  param.kI = 0.00; //자세값
  param.kyI = 1.0; //yaw값

  param.c1 = 1.0; //위치 (바꾸지말기)
  param.c2 = 0.0; //자세
  param.c3 = 1.0; //yaw (바꾸지말기)
  return param;
}

inline UAVParameters getUAVParameters() {
  UAVParameters param;
  param.m = 8.0; 
  param.g = 9.80665;
  param.J << 0.3, -0.0006, -0.0006,
             -0.0006,  0.3, 0.0006,
             -0.0006, 0.0006, 0.5318;
  return param;
}

}

#endif // CONTROLLER_PARAM_H


/*
last simulator gain

inline ControlParameters getControlParameters() {
  ControlParameters param;
  param.use_decoupled_yaw = true;
  param.kX << 10.0, 10.0, 3.0;
  param.kV << 9.0, 9.0, 12.0;
  param.kR << 4.0, 4.0, 2.0;
  param.kW << 3, 3, 3;
  return param;
}

inline IntegralParameters getIntegralParameters() {
  IntegralParameters param;
  param.kIX = 0.0;
  param.ki = 0.0;
  param.kIR = 0.0;
  param.kI = 0.0;
  param.kyI = 0.0;
  param.c1 = 0.0;
  param.c2 = 0.0;
  param.c3 = 0.0;
  return param;
}

inline UAVParameters getUAVParameters() {
  UAVParameters param;
  param.m = 4.7;
  param.g = 9.80665;
  return param;
}

*/

/* ㄹㅇㅋㅋ

inline ControlParameters getControlParameters() {
  ControlParameters param;
  param.use_decoupled_yaw = true;
  param.kX << 10.0, 10.0, 20.0;
  param.kV << 0.0, 0.0, 0.0;
  param.kR << 20.0, 20.0, 0.2;
  param.kW << 3.0, 3.0, 0.0;
  return param;
}

inline IntegralParameters getIntegralParameters() {
  IntegralParameters param;
  param.kIX = 4.0; //위치값
  
  param.ki = 0.0;  //이용안하는 값
  param.kIR = 0.0; //이용안하는 값
  
  param.kI = 0.00; //자세값
  param.kyI = 0.0; //yaw값

  param.c1 = 1.0; //위치
  param.c2 = 0.0; //자세
  param.c3 = 0.0; //yaw 
  return param;
}
  
*/
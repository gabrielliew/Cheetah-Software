/*! @file LegController.h
 *  @brief Common Leg Control Interface and Leg Control Algorithms
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards (the low level leg control boards)
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#ifndef PROJECT_LEGCONTROLLER_H
#define PROJECT_LEGCONTROLLER_H

#include "cppTypes.h"
#include "leg_control_command_lcmt.hpp"
#include "leg_control_data_lcmt.hpp"
#include "Dynamics/Quadruped.h"
#include "SimUtilities/SpineBoard.h"
#include "SimUtilities/ti_boardcontrol.h"
// #include <iostream>
// #include <chrono>
// #include <iomanip> // put_time
// #include <fstream>
// #include <sstream> // stringstream

/*!
 * Data sent from the control algorithm to the legs.
 */
template <typename T>
struct LegControllerCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerCommand() { zero(); }

  void zero();

  Vec3<T> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes;
  Mat3<T> kpCartesian, kdCartesian, kpJoint, kdJoint;
};

/*!
 * Data returned from the legs to the control code.
 */
template <typename T>
struct LegControllerData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerData() { zero(); }

  void setQuadruped(Quadruped<T>& quad) { quadruped = &quad; }

  void zero();

  Vec3<T> q, qd, p, v;
  Mat3<T> J;
  Vec3<T> tauEstimate;
  Quadruped<T>* quadruped;
};

/*!
 * Controller for 4 legs of a quadruped.  Works for both Mini Cheetah and Cheetah 3
 */
template <typename T>
class LegController {
 public:
  LegController(Quadruped<T>& quad) : _quadruped(quad) {
    for (auto& data : datas) data.setQuadruped(_quadruped);
    // now = std::chrono::high_resolution_clock::now();
    // auto UTC = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    // myfile.open (std::to_string(UTC) + ".csv");
    // myfile << "q00,qd00,t00,q01,qd01,t01,q02,qd02,t02,q10,qd10,t10,q11,qd11,t11,q12,qd12,t12,q20,qd20,t20,q21,qd21,t21,q22,qd22,t22,q30,qd30,t30,q31,qd31,t31,q32,qd32,t32\n";
  }

  void zeroCommand();
  void edampCommand(RobotType robot, T gain);
  void updateData(const SpiData* spiData);
  void updateData(const TiBoardData* tiBoardData);
  void updateCommand(SpiCommand* spiCommand);
  void updateCommand(TiBoardCommand* tiBoardCommand);
  void setEnabled(bool enabled) { _legsEnabled = enabled; };
  void setLcm(leg_control_data_lcmt* data, leg_control_command_lcmt* command);

  /*!
   * Set the maximum torque.  This only works on cheetah 3!
   */
  void setMaxTorqueCheetah3(T tau) { _maxTorque = tau; }
  // std::ofstream myfile;
  // std::chrono::time_point<std::chrono::high_resolution_clock> now;
  LegControllerCommand<T> commands[4];
  LegControllerData<T> datas[4];
  Quadruped<T>& _quadruped;
  bool _legsEnabled = false;
  T _maxTorque = 0;
  bool _zeroEncoders = false;
  u32 _calibrateEncoders = 0;
};

template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J,
                                   Vec3<T>* p, int leg);

#endif  // PROJECT_LEGCONTROLLER_H

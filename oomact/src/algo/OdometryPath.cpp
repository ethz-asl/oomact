/** \file OdometryPath.cpp
    \brief This file contains a utility to build a pose using calibrated or uncalibrated.
  */

#include <vector>
#include <cmath>
#include <utility>
#include <limits>

#include "aslam/calibration/algo/OdometryPath.h"

#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/Transformation.hpp>

namespace aslam {
  namespace calibration {
  using namespace sm::kinematics;
/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  // Returns the local velocity with the given odometry model
  std::pair<Eigen::Vector3d, Eigen::Vector3d> odomKinemModel(const double Wl,
                                       const double Wr,
                                       const double Rl,
                                       const double Rr,
                                       const double L){
    return std::make_pair((Eigen::Vector3d()<< 0.5 * (Wr*Rr + Wl*Rl), 0.0, 0.0).finished(),
                          (Eigen::Vector3d()<< 0.0, 0.0, 1/L * (Wr*Rr - Wl*Rl)).finished());
  }
  Eigen::Vector3d directKinematics(const Eigen::Vector4d& rotPose,
                                       const Eigen::Vector3d& /*angVel*/,
                                       const double dt,
                                       const double Wl,
                                       const double Wr,
                                       const double Rl,
                                       const double Rr,
                                       const double L){

        double Vl = Wl*Rl;
        double Vr = Wr*Rr;
        double angVelApprox = (Vr - Vl) / L;
        double dTheta = angVelApprox * dt;
        double V = (Vl + Vr) / 2;
        // Check consistency with spline Angular velocity? Do that in python
        // Todo: Go for a trapezoidal integration, or continuous integration
        // Assuming an interpolation between qk and qk+1 (Needs further derivations)

        double dx;
        double dy;
        // Forward Euler
        // Check this formula
        if(angVelApprox > 1e-3){
          dx = V / angVelApprox * (sin(dTheta));
          dy = V / angVelApprox * (1-cos(dTheta));
        }
        else{
          dx = V * dt;
          dy = 0.0;
        }

        // The increment in the global frame
        return sm::kinematics::quatRotate(rotPose,Eigen::Vector3d(dx,dy,0.0));
      }
  Transformation integrateMotionModel(const Eigen::Vector3d& t_m_f_km1,
                                      const Eigen::Vector4d& q_m_f_km1,
                                      const Eigen::Vector3d& v_f_mf_km1,
                                      const Eigen::Vector3d& w_f_mf_km1,
                                      double dt) {

    Eigen::Vector4d q_m_f_k;
    Eigen::Vector4d q_m_f_k_midPoint;
    q_m_f_k = qplus(q_m_f_km1, qexp(-w_f_mf_km1 * dt));
    q_m_f_k_midPoint = qplus(q_m_f_km1, qexp(-w_f_mf_km1 * 0.5 * dt));

    const Eigen::Vector3d t_m_f_k = t_m_f_km1 + dt * quatRotate(q_m_f_k_midPoint, v_f_mf_km1);
    return Transformation(q_m_f_k, t_m_f_k);
  }
  }
}


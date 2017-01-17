/******************************************************************************
 * Copyright (C) 2014 by Ignazio Indovina                                     *
 * ignazio.indovina@gmail.com                                                 *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file laser3d.h
    \brief This file contains utilities to build Jacobians, calculate covariances for a 3d Laser Scanner (Actually Velodyne).
  */


#include <vector>
#include <cmath>
#include <utility>
#include <limits>
#include "aslam/calibration/laser-scanner/laser3d.h"

#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/rotations.hpp>

namespace aslam {
  namespace calibration {
  using namespace sm::kinematics;

  // Contains stuff for the noise of the laser 3d
/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  // Spherical (Polar) coordinates are equivalent to: T = TRz(phi)*TRy(th)*Ttx(r)
  // Polar coordinates represented by r, th, phi
      Eigen::Vector3d cart2pol(const Eigen::Vector3d& p_c){
        const double& x = p_c(0);
        const double& y = p_c(1);
        const double& z = p_c(2);
        const double r = p_c.norm();
        if(r > std::numeric_limits<double>::epsilon())
          return (Eigen::Vector3d() << r, - asin(z/r), atan2(y,x)).finished();
        return Eigen::Vector3d::Zero();

      }

      Eigen::Vector3d pol2cart(const Eigen::Vector3d& p_p){
        const double& r = p_p(0);
        const double& th = p_p(1);
        const double& phi = p_p(2);
        return (Eigen::Vector3d() << r*cos(th)*cos(phi), r*sin(phi)*cos(th), -r*sin(th)).finished();
      }

      const Eigen::Matrix3d covPol2covCart(const double& r, const double& th, const double& phi,
                                            const double& dr, const double& dth, const double& dphi){
        Eigen::Matrix3d jacobian = jacobianPol(r,th,phi);
        return jacobian * (Eigen::Vector3d() << dr, dth, dphi).finished().asDiagonal() * jacobian.transpose();
      }

      const Eigen::Matrix3d jacobianPol(const double& r, const double& th, const double& phi){
        const double& costh = cos(th);
        const double& sinth = sin(th);
        const double& cosphi = cos(phi);
        const double& sinphi = sin(phi);
        return (Eigen::Matrix3d() << costh*cosphi, -r*sinth*cosphi, -r*costh*sinphi,
                                     costh*sinphi, -r*sinth*sinphi, r*costh*cosphi,
                                     -sinth,       -r*costh,        0.0           ).finished();
      }
      const Eigen::Matrix3d rotFramePol(const double th, const double phi){
        return sm::kinematics::Rz(phi)*sm::kinematics::Ry(th);
      }

      // Approximated covariance, It should be better to linearize
      const Eigen::Matrix3d covPointSurface(bool useSurfaceNormal, const Eigen::Vector3d p_v_cart,const Eigen::Vector3d n_v_p,
                                             const double& dr, const double& dth, const double& dphi, const double& diverAngle,
                                             const double& gainX, const double& gainY, const double& gainZ){

        const auto& r_th_phi = cart2pol(p_v_cart);
        const double& r = r_th_phi(0);
        const double& th = r_th_phi(1);
        const double& phi = r_th_phi(2);
        const auto& R_v_li = rotFramePol(th,phi);

        assert(fabs(R_v_li.determinant()-1) < 1e-2);

        double Var_xl = dr * dr;
        double a2 = (dphi * dphi + diverAngle * diverAngle / 4.0);
        double b2 = (dth * dth + diverAngle * diverAngle / 4.0);
        double Var_yl =  gainY * a2 * r * r;
        double Var_zl =  gainZ * b2 * r * r;

        if(useSurfaceNormal){
          assert(fabs(n_v_p.norm()-1) < 1e-2);
          // ||r x n|| = ||r||*sin(alfa)   /// It should be more correct tan(alfa)
          const auto& crossTerm = p_v_cart.cross(n_v_p).norm();
          // Normal in the laser Ref frame
          const auto& n_li_p = R_v_li.transpose() * n_v_p;
          const auto ny2 = n_li_p(1) * n_li_p(1);
          const auto nz2= n_li_p(2) * n_li_p(2);
          double distCorr2 = 0.0;
          if(ny2 > std::numeric_limits<double>::epsilon())
            distCorr2 = b2 + (a2 - b2) * ny2/(ny2+nz2);
          Var_xl += gainX * distCorr2*crossTerm;
        }
        return R_v_li * (Eigen::Vector3d() << Var_xl, Var_yl, Var_zl).finished().asDiagonal() * R_v_li.transpose();
      }
  }

}

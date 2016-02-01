/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Clearpath Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Enrique Fernández
 */

#ifndef RIGID_BODY_MOTION_H_
#define RIGID_BODY_MOTION_H_

#include <Eigen/Dense>

#include <limits>

namespace diff_drive_controller
{

/**
 * \brief Compute rigid body motion in SE(2) using element-wise equations.
 * \param[in, out] x   Pose x component
 * \param[in, out] y   Pose y component
 * \param[in, out] yaw Pose yaw component
 * \param[in] v_x   Velocity/Twist x   component
 * \param[in] v_y   Velocity/Twist y   component
 * \param[in] v_yaw Velocity/Twist yaw component
 * \param[in] dt Time step
 */
static void integrate_motion(double& x, double &y, double &yaw,
    const double v_x, const double v_y, const double v_yaw,
    const double dt)
{
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  x   += (v_x * cos_yaw - v_y * sin_yaw) * dt;
  y   += (v_x * sin_yaw + v_y * cos_yaw) * dt;
  yaw += v_yaw * dt;
}

/**
 * \brief Compute rigid body motion in SE(2) using element-wise equations.
 * Also computes the Jacobians wrt the pose and the velocity/twist.
 * \param[in, out] x   Pose x component
 * \param[in, out] y   Pose y component
 * \param[in, out] yaw Pose yaw component
 * \param[in] v_x   Velocity/Twist x   component
 * \param[in] v_y   Velocity/Twist y   component
 * \param[in] v_yaw Velocity/Twist yaw component
 * \param[in] dt Time step
 * \param[out] J_pose Jacobian wrt the pose
 * \param[out] J_twist Jacobian wrt the velocity/twist
 */
static void integrate_motion(double& x, double &y, double &yaw,
    const double v_x, const double v_y, const double v_yaw,
    const double dt,
    Eigen::Matrix3d& J_pose, Eigen::Matrix3d& J_twist)
{
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  x   += (v_x * cos_yaw - v_y * sin_yaw) * dt;
  y   += (v_x * sin_yaw + v_y * cos_yaw) * dt;
  yaw += v_yaw * dt;

  J_pose << 1.0, 0.0, (-v_x * sin_yaw - v_y * cos_yaw) * dt,
            0.0, 1.0, ( v_x * cos_yaw - v_y * sin_yaw) * dt,
            0.0, 0.0,                                   1.0;

  J_twist << cos_yaw * dt, -sin_yaw * dt, 0.0,
             sin_yaw * dt,  cos_yaw * dt, 0.0,
                      0.0,           0.0,  dt;
}

}

#endif /* RIGID_BODY_MOTION_H_ */

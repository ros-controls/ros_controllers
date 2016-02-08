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

#include <diff_drive_controller/quadratic_meas_covariance_model.h>

namespace diff_drive_controller
{

QuadraticMeasCovarianceModel::QuadraticMeasCovarianceModel(
    const double wheel_resolution)
  : MeasCovarianceModel(wheel_resolution)
{
  meas_covariance_.setZero();
}

const MeasCovarianceModel::MeasCovariance&
QuadraticMeasCovarianceModel::compute(const double dp_l, const double dp_r)
{
  /// Measurement (wheel position increment) covaraince model, proportional
  /// to the wheel position increment squared (quadratic) for each wheel:
  const double dp_std_l = k_l_ * dp_l;
  const double dp_std_r = k_r_ * dp_r;

  const double dp_var_l = dp_std_l * dp_std_l;
  const double dp_var_r = dp_std_r * dp_std_r;

  /// Wheel resolution covariance, which is like a DC offset equal to half of
  /// the resolution, which is the theoretical average error:
  const double dp_std_avg = 0.5 * wheel_resolution_;
  const double dp_var_avg = dp_std_avg * dp_std_avg;

  /// @todo This can be extended to support lateral slippage
  /// k_s_ * [see/find Olson notes]

  /// Set covariance matrix (diagonal):
  meas_covariance_.diagonal() << dp_var_l + dp_var_avg,
                                 dp_var_r + dp_var_avg;
}

}  // namespace diff_drive_controller


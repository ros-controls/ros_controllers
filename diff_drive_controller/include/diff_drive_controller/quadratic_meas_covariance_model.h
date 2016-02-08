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

#ifndef QUADRATIC_MEAS_COVARIANCE_MODEL_H_
#define QUADRATIC_MEAS_COVARIANCE_MODEL_H_

#include <diff_drive_controller/meas_covariance_model.h>

namespace diff_drive_controller
{

  /**
   * \brief Quadratic Meas(urement) Covariance Model
   *
   * The odometry covariance is computed using a similar model to the one
   * presented in:
   *
   * [Siegwart, 2004]:
   *   Roland Siegwart, Illah R. Nourbakhsh
   *   Introduction to Autonomous Mobile Robots
   *   1st Edition, 2004
   *
   * Section:
   *   5.2.4 'An error model for odometric position estimation' (pp. 186-191)
   *
   * Although the twist covariance doesn't appear explicitly, the implementation
   * here is based on the same covariance model used for the pose covariance.
   *
   * Instead of using the std::abs of the wheel position increment dp, this
   * model uses the squared (quadratic) value of dp, so the proportional term
   * (multiplied by k_*_) does NOT depend on the control period dt (see
   * explanation below).
   *
   * This is important because it makes the covariance model independent of
   * the control period dt. Since dp = v * dt, the variance actually gives:
   *
   *   dp^2 = (v * dt)^2 = v^2 * dt^2
   *
   * where dt^2 gets cancelled when the covariance propagation for the twist
   * is applied, i.e. the 1/dt jacobians pre- and post-multiply this
   * covariance model, so the control perdio dt cancels out:
   *
   *   (1/dt) * dt^2 * (1/dt) = dt^2 / dt^2 = 1
   *
   * The model also includes the wheel resolution, as a constant additive
   * diagonal covariance, that can be easily disabled by using a zero
   * (ideal/perfect) wheel resolution.
   *
   * If the wheel resolution (DC offset) term is used, the covariance model
   * would again vary with the control period dt because the wheel
   * resolution is constant and does NOT include/depend on dt.
   */
  class QuadraticMeasCovarianceModel : public MeasCovarianceModel
  {
    public:
      /**
       * \brief Constructor
       * \param[in] wheel_resolution Wheel resolution [rad]
       */
      QuadraticMeasCovarianceModel(const double wheel_resolution = 0.0);

      /**
       * \brief Destructor
       */
      virtual ~QuadraticMeasCovarianceModel()
      {}

      /**
       * \brief Integrates w/o computing the jacobians
       * \param [in] dp_l Left wheel position increment [rad]
       * \param [in] dp_r Right wheel position increment [rad]
       * \return Meas(urement) covariance
       */
      const MeasCovariance& compute(const double dp_l, const double dp_r);
  };

}  // namespace diff_drive_controller

#endif /* QUADRATIC_MEAS_COVARIANCE_MODEL_H_ */

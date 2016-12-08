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

#ifndef LINEAR_MEAS_COVARIANCE_MODEL_H_
#define LINEAR_MEAS_COVARIANCE_MODEL_H_

#include <diff_drive_controller/meas_covariance_model.h>

namespace diff_drive_controller
{

  /**
   * \brief Linear Meas(urement) Covariance Model
   *
   * The odometry covariance is computed according with the model presented in:
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
   * The model also includes the wheel resolution, as a constant additive
   * diagonal covariance, that can be easily disabled by using a zero
   * (ideal/perfect) wheel resolution.
   */
  class LinearMeasCovarianceModel : public MeasCovarianceModel
  {
    public:
      /**
       * \brief Constructor
       * \param[in] wheel_resolution Wheel resolution [rad]
       */
      LinearMeasCovarianceModel(const double wheel_resolution = 0.0);

      /**
       * \brief Destructor
       */
      virtual ~LinearMeasCovarianceModel()
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

#endif /* LINEAR_MEAS_COVARIANCE_MODEL_H_ */

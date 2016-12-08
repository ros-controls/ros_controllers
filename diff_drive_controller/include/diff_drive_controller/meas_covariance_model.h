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

#ifndef MEAS_COVARIANCE_MODEL_H_
#define MEAS_COVARIANCE_MODEL_H_

#include <Eigen/Core>

namespace diff_drive_controller
{

  /**
   * \brief Meas(urement) Covariance Model
   */
  class MeasCovarianceModel
  {
    public:
      /// Meas(urement) covariance type:
      typedef Eigen::Matrix2d MeasCovariance;

      /**
       * \brief Constructor
       * \param[in] wheel_resolution Wheel resolution [rad]
       */
      MeasCovarianceModel(const double wheel_resolution = 0.0)
        : wheel_resolution_(wheel_resolution)
      {}

      virtual ~MeasCovarianceModel()
      {}

      /**
       * \brief Integrates w/o computing the jacobians
       * \param [in] dp_l Left wheel position increment [rad]
       * \param [in] dp_r Right wheel position increment [rad]
       * \return Meas(urement) covariance
       */
      virtual const MeasCovariance& compute(const double dp_l, const double dp_r) = 0;

      /**
       * \brief Left wheel covariance gain getter
       * \return Left wheel covariance gain
       */
      double getKl() const
      {
        return k_l_;
      }

      /**
       * \brief Right wheel covariance gain getter
       * \return Right wheel covariance gain
       */
      double getKr() const
      {
        return k_r_;
      }

      /**
       * \brief Wheel resolution getter
       * \return Wheel resolution [rad]
       */
      double getWheelResolution() const
      {
        return wheel_resolution_;
      }

      /**
       * \brief Left wheel covariance gain setter
       * \param[in] k_l Left wheel covariance gain
       */
      void setKl(const double k_l)
      {
        k_l_ = k_l;
      }

      /**
       * \brief Right wheel covariance gain setter
       * \param[in] k_r Right wheel covariance gain
       */
      void setKr(const double k_r)
      {
        k_r_ = k_r;
      }

      /**
       * \brief Wheel resolution setter
       * \param[in] wheel_resolution Wheel resolution [rad]
       */
      void setWheelResolution(const double wheel_resolution)
      {
        wheel_resolution_ = wheel_resolution;
      }

    protected:
      /// Meas(urement) covariance:
      MeasCovariance meas_covariance_;

      /// Meas(urement) Covariance Model gains:
      double k_l_;
      double k_r_;

      /// Wheel resolution [rad]:
      double wheel_resolution_;
  };

}  // namespace diff_drive_controller

#endif /* MEAS_COVARIANCE_MODEL_H_ */

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Clearpath Robotics, Inc.
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

#ifndef RUNGE_KUTTA_2_INTEGRATE_FUNCTOR_H_
#define RUNGE_KUTTA_2_INTEGRATE_FUNCTOR_H_

#include <boost/type_traits.hpp>
#include <boost/static_assert.hpp>

#include <cmath>

namespace diff_drive_controller
{

  /// Runge-Kutta 2nd order integration:
  struct RungeKutta2IntegrateFunctor
  {
    /**
     * \brief Integrates the pose (x, y, yaw) given the velocities (v, w) using
     * 2nd order Runge-Kutta
     * \param [in, out] x   Pose x   component
     * \param [in, out] y   Pose y   component
     * \param [in, out] yaw Pose yaw component
     * \param [in] v Linear  velocity   [m]
     *               (linear  displacement, i.e.   m/s * dt) computed by encoders
     * \param [in] w Angular velocity [rad]
     *               (angular displacement, i.e. rad/s * dt) computed by encoders
     */
    template <typename T>
    void operator()(T& x, T& y, T& yaw, const T& v, const T& w) const
    {
      BOOST_STATIC_ASSERT_MSG(
          !boost::is_pod<T>::value || boost::is_floating_point<T>::value,
          "The pose components must be specified as float values.");

      const T direction = yaw + w * T(0.5);

      x   += v * cos(direction);
      y   += v * sin(direction);
      yaw += w;
    }
  };

} // namespace diff_drive_controller

#endif /* RUNGE_KUTTA_2_INTEGRATE_FUNCTOR_H_ */

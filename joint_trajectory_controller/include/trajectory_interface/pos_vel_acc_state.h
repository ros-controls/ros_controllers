///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Adolfo Rodriguez Tsouroukdissian

#pragma once


#include <vector>

namespace trajectory_interface
{

/**
 * \brief Multi-dof trajectory state containing position, velocity and acceleration data.
 */
template <class ScalarType>
struct PosVelAccState
{
  typedef ScalarType Scalar;

  PosVelAccState() {}

   /**
    * \brief Resource-preallocating constructor.
    *
    * Position, velocity and acceleration vectors are resized to \p size, and their values are set to zero.
    * Note that these two situations are different:
    * \code
    * // 2-dimensional state specifying zero position, velocity and acceleration
    * State zero_pos_vel_acc(2);
    *
    * // 2-dimensional state specifying zero position
    * State zero_pos;
    * zero_pos.position.resize(2);
    * \endcode
    */
  PosVelAccState(const typename std::vector<Scalar>::size_type size)
    : position(    std::vector<Scalar>(size, static_cast<Scalar>(0))),
      velocity(    std::vector<Scalar>(size, static_cast<Scalar>(0))),
      acceleration(std::vector<Scalar>(size, static_cast<Scalar>(0)))
  {}

  std::vector<Scalar> position;
  std::vector<Scalar> velocity;
  std::vector<Scalar> acceleration;
};

} // namespace

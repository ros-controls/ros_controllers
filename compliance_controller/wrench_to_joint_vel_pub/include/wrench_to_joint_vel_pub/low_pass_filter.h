///////////////////////////////////////////////////////////////////////////////
//      Title     : low_pass_filter.h
//      Project   : wrench_to_twist_pub
//      Created   : 1/25/2019
//      Author    : Andy Zelenak
//
// BSD 3-Clause License
//
// Copyright (c) 2018, Los Alamos National Security, LLC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef WRENCH_TO_TWIST_PUB_LOW_PASS_FILTER_H
#define WRENCH_TO_TWIST_PUB_LOW_PASS_FILTER_H

#include <vector>

namespace wrench_to_joint_vel_pub
{
class LowPassFilter
{
public:
  /**
   * Create an object which low-pass filters a datastream.
   * @param filter_param Larger->more smoothing but more lag.
   */
  LowPassFilter(double filter_param);

  /**
   * Apply the low-pass filter to a datastream.
   */
  double filter(const double new_msrmt);

  /**
   * \brief Clear a low-pass filter's history.
   * Often would use an argument of zero.
   */
  void reset(double data);

private:
  std::vector<double> prev_msrmts_ = { 0., 0., 0. };
  std::vector<double> prev_filtered_msrmts_ = { 0., 0. };

  // Related to the cutoff frequency of the filter.
  // filter_param=1 results in a cutoff at 1/4 of the sampling rate.
  // See bitbucket.org/AndyZe/pid for slightly more sophistication.
  // Larger filter_param --> trust the filtered data more, trust the measurements
  // less, i.e. higher cutoff frequency.
  double filter_param_ = 4.;
};
}  // namespace wrench_to_joint_vel_pub

#endif

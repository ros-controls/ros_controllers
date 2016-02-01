///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics, Inc. nor the names of its
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
///////////////////////////////////////////////////////////////////////////////

/// \author Enrique Fernandez

#include <diff_drive_controller/covariance.h>

#include <gtest/gtest.h>

#include <Eigen/Dense>

template <typename T, int N>
void testCovariance(const Eigen::Matrix<T, N, N>& covariance,
    const double max_condition_number = 1e3)
{
  using namespace diff_drive_controller;

  static const Eigen::IOFormat HeavyFmt(
      Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

  typedef Eigen::SelfAdjointEigenSolver< Eigen::Matrix<T, N, N> > EigenSolver;

  EigenSolver eigensolver(covariance);

  EXPECT_TRUE(eigensolver.info() == Eigen::Success)
    << "Covariance =\n" << covariance.format(HeavyFmt);

  EXPECT_TRUE(isSymmetric(covariance))
    << "Covariance =\n" << covariance.format(HeavyFmt);
  EXPECT_TRUE(isPositiveDefinite(covariance, no_tag()))
    << "Covariance =\n" << covariance.format(HeavyFmt) << "\n"
    << "Eigenvalues = " << eigensolver.eigenvalues().transpose().format(HeavyFmt);
  EXPECT_LT(conditionNumber(covariance), max_condition_number)
    << "Covariance =\n" << covariance.format(HeavyFmt) << "\n"
    << "Condition number = " << conditionNumber(covariance) << "\n"
    << "Eigenvalues = " << eigensolver.eigenvalues().transpose().format(HeavyFmt);
}

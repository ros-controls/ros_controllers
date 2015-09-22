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

#ifndef COVARIANCE_H_
#define COVARIANCE_H_

#include <Eigen/Dense>

#include <limits>

namespace diff_drive_controller
{

/*
 * Tags to select different covariance matrix conversion policies:
 * - no_tag    : Use all matrix, not enforcing symmetry
 * - lower_tag : Use lower triangular sub-matrix, enforcing symmetry
 * - upper_tag : Use upper triangular sub-matrix, enforcing symmetry
 * - mean_tag  : Use (lower + upper) / 2.0, enforcing symmetry
 */
struct no_tag {};
struct lower_tag {};
struct upper_tag {};
struct mean_tag {};

/**
 * \brief Converts odometry covariance into message format
 * \param [in]  covariance Covariance 3x3 matrix
 * \param [out] msg        Covariance message with 6x6 matrix in 1x36 array
 */
template <typename T>
static void covarianceToMsg(
    const Eigen::Matrix<T, 3, 3>& covariance,
    boost::array<T, 36>& msg)
{
  covarianceToMsg(covariance, msg, mean_tag());
}

template <typename T>
static void covarianceToMsg(
    const Eigen::Matrix<T, 3, 3>& covariance,
    boost::array<T, 36>& msg,
    no_tag)
{
  Eigen::Map< Eigen::Matrix<T, 6, 6> > C(msg.data());

  C.template topLeftCorner<2, 2>() = covariance.template topLeftCorner<2, 2>();
  C(5, 5) = covariance(2, 2);

  C(0, 5) = covariance(0, 2);
  C(1, 5) = covariance(1, 2);
  C(5, 0) = covariance(2, 0);
  C(5, 1) = covariance(2, 1);
}

template <typename T>
static void covarianceToMsg(
    const Eigen::Matrix<T, 3, 3>& covariance,
    boost::array<T, 36>& msg,
    lower_tag)
{
  Eigen::Map< Eigen::Matrix<T, 6, 6> > C(msg.data());

  C(0, 0) = covariance(0, 0);
  C(1, 1) = covariance(1, 1);
  C(5, 5) = covariance(2, 2);

  C(0, 1) = C(1, 0) = covariance(1, 0);
  C(0, 5) = C(5, 0) = covariance(2, 0);
  C(1, 5) = C(5, 1) = covariance(2, 1);
}

template <typename T>
static void covarianceToMsg(
    const Eigen::Matrix<T, 3, 3>& covariance,
    boost::array<T, 36>& msg,
    upper_tag)
{
  Eigen::Map< Eigen::Matrix<T, 6, 6> > C(msg.data());

  C(0, 0) = covariance(0, 0);
  C(1, 1) = covariance(1, 1);
  C(5, 5) = covariance(2, 2);

  C(0, 1) = C(1, 0) = covariance(0, 1);
  C(0, 5) = C(5, 0) = covariance(0, 2);
  C(1, 5) = C(5, 1) = covariance(1, 2);
}

template <typename T>
static void covarianceToMsg(
    const Eigen::Matrix<T, 3, 3>& covariance,
    boost::array<T, 36>& msg,
    mean_tag)
{
  Eigen::Map< Eigen::Matrix<T, 6, 6> > C(msg.data());

  C(0, 0) = covariance(0, 0);
  C(1, 1) = covariance(1, 1);
  C(5, 5) = covariance(2, 2);

  C(0, 1) = C(1, 0) = 0.5 * (covariance(0, 1) + covariance(1, 0));
  C(0, 5) = C(5, 0) = 0.5 * (covariance(0, 2) + covariance(2, 0));
  C(1, 5) = C(5, 1) = 0.5 * (covariance(1, 2) + covariance(2, 1));
}

/**
 * \brief Converts odometry covariance message into matrix format
 * \param [in]  msg        Covariance message with 6x6 matrix in 1x36 array
 * \param [out] covariance Covariance 3x3 matrix
 */
template <typename T>
static void msgToCovariance(
    const boost::array<T, 36>& msg,
    Eigen::Matrix<T, 3, 3>& covariance)
{
  msgToCovariance(msg, covariance, mean_tag());
}

template <typename T>
static void msgToCovariance(
    const boost::array<T, 36>& msg,
    Eigen::Matrix<T, 3, 3>& covariance,
    no_tag)
{
  const Eigen::Map< const Eigen::Matrix<T, 6, 6> > C(msg.data());
  covariance.template topLeftCorner<2, 2>() = C.template topLeftCorner<2, 2>();
  covariance(2, 2) = C(5, 5);

  covariance(0, 2) = C(0, 5);
  covariance(2, 0) = C(5, 0);
  covariance(1, 2) = C(1, 5);
  covariance(2, 1) = C(5, 1);
}

template <typename T>
static void msgToCovariance(
    const boost::array<T, 36>& msg,
    Eigen::Matrix<T, 3, 3>& covariance,
    lower_tag)
{
  const Eigen::Map< const Eigen::Matrix<T, 6, 6> > C(msg.data());

  covariance(0, 0) = C(0, 0);
  covariance(1, 1) = C(1, 1);
  covariance(2, 2) = C(5, 5);

  covariance(0, 1) = covariance(1, 0) = C(1, 0);
  covariance(0, 2) = covariance(2, 0) = C(5, 0);
  covariance(1, 2) = covariance(2, 1) = C(5, 1);
}

template <typename T>
static void msgToCovariance(
    const boost::array<T, 36>& msg,
    Eigen::Matrix<T, 3, 3>& covariance,
    upper_tag)
{
  const Eigen::Map< const Eigen::Matrix<T, 6, 6> > C(msg.data());

  covariance(0, 0) = C(0, 0);
  covariance(1, 1) = C(1, 1);
  covariance(2, 2) = C(5, 5);

  covariance(0, 1) = covariance(1, 0) = C(0, 1);
  covariance(0, 2) = covariance(2, 0) = C(0, 5);
  covariance(1, 2) = covariance(2, 1) = C(1, 5);
}

template <typename T>
static void msgToCovariance(
    const boost::array<T, 36>& msg,
    Eigen::Matrix<T, 3, 3>& covariance,
    mean_tag)
{
  const Eigen::Map< const Eigen::Matrix<T, 6, 6> > C(msg.data());

  covariance(0, 0) = C(0, 0);
  covariance(1, 1) = C(1, 1);
  covariance(2, 2) = C(5, 5);

  covariance(0, 1) = covariance(1, 0) = 0.5 * (C(0, 1) + C(1, 0));
  covariance(0, 2) = covariance(2, 0) = 0.5 * (C(0, 5) + C(5, 0));
  covariance(1, 2) = covariance(2, 1) = 0.5 * (C(1, 5) + C(5, 1));
}

/**
 * \brief Check is a matrix M is symmetric, i.e. M' M = Id or low(M) = up(M)
 * where low and up are the lower and upper triangular matrices of M
 * \param [in] M Square matrix
 * \return True if the matrix is symmetric
 */
template <typename T, int N>
bool isSymmetric(const Eigen::Matrix<T, N, N>& M)
{
  // Note that triangularView doesn't help to much to simplify the check,
  // so we iterate on the lower and upper triangular matrixes directly:
  for (size_t i = 0; i < N - 1; ++i)
  {
    for (size_t j = i + 1; j < N; ++j)
    {
      if (std::abs(M(i, j) - M(j, i)) > std::numeric_limits<T>::epsilon())
      {
        return false;
      }
    }
  }

  return true;
}

/*
 * Tags to select between positive/negative "strict" or semi-definite matrix
 * checks:
 * - no_tag   : Check for positive/negative "strict" definite, i.e. >  or <
 * - semi_tag : Check for positive/negative     semi-definite, i.e. >= or <=
 */
// no_tag already defined before.
struct semi_tag {};

/**
 * \brief Check if a matrix M is positive definite, i.e. x* M x > 0, or
 *        positive semi-definite, i.e. x* M x >= 0
 * \param [in] M Square matrix
 * \return True if the matrix is positive definite
 */
template <typename T, int N>
bool isPositiveDefinite(const Eigen::Matrix<T, N, N>& M)
{
  return isPositiveDefinite(M, no_tag());
}

template <typename T, int N>
bool isPositiveDefinite(const Eigen::Matrix<T, N, N>& M, no_tag)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, N, N> > eigensolver(M);

  if (eigensolver.info() == Eigen::Success)
  {
    // All eigenvalues must be > 0:
    return (eigensolver.eigenvalues().array() > T(0)).all();
  }

  return false;
}

template <typename T, int N>
bool isPositiveDefinite(const Eigen::Matrix<T, N, N>& M, semi_tag)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, N, N> > eigensolver(M);

  if (eigensolver.info() == Eigen::Success)
  {
    // All eigenvalues must be >= 0:
    return (eigensolver.eigenvalues().array() >= T(0)).all();
  }

  return false;
}

/**
 * \brief Check if a matrix M is a covariance matrix, i.e. it checks that it's
 *        symmetric and positive semi-definite.
 *        Note that since many algorithms doesn't work with positive
 *        semi-definite matrices (i.e. with at least one eigenvalue equal to 0),
 *        this function allows to check for positive definite too.
 *        By default it checks for positive semi-definite tough.
 * \param [in] M Square matrix
 * \return True if the matrix is a covariance matrix, i.e. symmetric and
 *         positive (semi-)definite.
 */
template <typename T, int N>
bool isCovariance(const Eigen::Matrix<T, N, N>& M)
{
  return isCovariance(M, semi_tag());
}

template <typename T, int N>
bool isCovariance(const Eigen::Matrix<T, N, N>& M, no_tag)
{
  return isSymmetric(M) && isPositiveDefinite(M, no_tag());
}

template <typename T, int N>
bool isCovariance(const Eigen::Matrix<T, N, N>& M, semi_tag)
{
  return isSymmetric(M) && isPositiveDefinite(M, semi_tag());
}

/**
 * \brief Compute the condition number of a matrix M
 * \param [in] M Square matrix
 * \return Condition number computed as the abs(s_max / s_min), i.e. the
 *         absolute value of the quotient of the maximum s_max and minimum s_min
 *         singular values, using SVD decomposition.
 *         If any singular values is zero, it returns +Inf.
 */
template <typename T, int N>
T conditionNumber(const Eigen::Matrix<T, N, N>& M)
{
  Eigen::JacobiSVD<Eigen::Matrix<T, N, N> > svd(M);

  const T s_min = svd.singularValues().minCoeff();

  return s_min == 0 ?
         std::numeric_limits<T>::infinity() :
         svd.singularValues().maxCoeff() / s_min;
}

}

#endif /* COVARIANCE_H_ */

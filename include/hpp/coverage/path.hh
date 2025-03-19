// Copyright (c) 2025 CNRS
// Author: Florent Lamiraux
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#ifndef HPP_COVERAGE_PATH_HH
#define  HPP_COVERAGE_PATH_HH

#include <hpp/coverage/config.hh>
#include <hpp/core/steering-method/spline.hh>
#include <hpp/core/path/spline.hh>

namespace hpp{
namespace coverage{

typedef core::PathPtr_t PathPtr_t;
typedef core::ConfigurationIn_t ConfigurationIn_t;
typedef core::value_type value_type ;
typedef core::size_type size_type ;

/// Helper class to create spline paths in SE(3)
class HPP_COVERAGE_DLLAPI Path {
public:
  Path();
  /// Multiply two functions with values in SE(3)
  /// \param p1, p2 two functions from an interval to SE(3).
  ///
  /// p2 definition interval is scaled to fit p1 definition interval
  ///
  /// If \f$p_1\f$ is defined over \f$[0,T_1]\f$ and \f$p_2\f$ over \f$[0,T_2]\f$, the result
  /// if this function is defined over \f$[0,T_1]\f$ by
  /// \f[
  /// res(t) = p_1(t) . p_2(\frac{T_2}{T_1}\;t)
  /// \f]
  PathPtr_t multiply(const PathPtr_t& p1, const PathPtr_t& p2);

  /// Create a spline path with values in SE(3)
  ///
  /// \param pose0, pose1, initial and final values of the path,
  /// \param length length of the interval of definition,
  /// \param order order of continuity of the derivatives: 0 linear interpolation, 1 polynomial
  ///        of degree 3 with zero velocities at beginning and end.
  PathPtr_t createSpline(ConfigurationIn_t pose0, ConfigurationIn_t pose1, value_type length,
			 size_type order);
  const pinocchio::DevicePtr_t& robot()
  {
    return robot_;
  }
private:
  typedef core::steeringMethod::Spline <core::path::BernsteinBasis, 3> SteeringMethod3_t;
  typedef core::steeringMethod::Spline <core::path::BernsteinBasis, 1> SteeringMethod1_t;
  pinocchio::DevicePtr_t robot_;
  // Steering method for linear interpolation in SE(3)
  SteeringMethod1_t::Ptr_t sm1_;
  // Steering method for spline of degree 3 interpolation in SE(3)
  SteeringMethod3_t::Ptr_t sm3_;
}; // class Path
} // namespace coverage
} // namespace hpp
#endif //  HPP_COVERAGE_PATH_HH

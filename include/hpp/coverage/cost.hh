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

#ifndef HPP_COVERAGE_COST_HH
#define HPP_COVERAGE_COST_HH

#include <hpp/coverage/config.hh>
#include <pinocchio/multibody/data.hpp>
#include <hpp/core/path/cost.hh>
#include <hpp/core/weighed-distance.hh>

namespace hpp{
namespace coverage{

HPP_PREDEF_CLASS(ToolRotation);
typedef shared_ptr<ToolRotation> ToolRotationPtr_t;
typedef pinocchio::GripperPtr_t GripperPtr_t;
typedef pinocchio::DevicePtr_t DevicePtr_t;
typedef pinocchio::FrameIndex FrameIndex;
typedef pinocchio::value_type value_type;
typedef core::PathConstPtr_t PathConstPtr_t;

/// Cost corresponding to the amount of rotation of a tool on a mnipulator arm
///
/// The cost is defined by the amount of rotation along the tool x-axis.
/// If \f$\mathbf{q}_1\f$ and \f$\mathbf{q}_2\f$ are the initial and final configurations of
/// the path, and if \f$R_1\f$ and \f$R_2\f$ are the orientation of the tool in each configuration,
/// the cost is defined by:
/// \[
///    C = \left|\mathbf{x}.\log\left(R_1^{-1}\;R_2\right)\right|
/// \]
/// where \f$\mathbf{x}\f$ is the unit vector along \f$x\f$.
///
/// \note a small amount of hpp::core::WeighedDistance (1e-3) is added to the result to
///       make sure that the distance is not zero for different configurations
class HPP_COVERAGE_DLLAPI ToolRotation : public core::path::Cost
{
 public:
  /// Create instance and return shared pointer
  static ToolRotationPtr_t create(const DevicePtr_t& robot, const std::string& gripperName);

  /// Compute the cost of a path
  ///
  virtual value_type eval(const PathConstPtr_t& path);
  
 protected:
  ToolRotation(const DevicePtr_t& robot, const std::string& gripperName);

 private:
  DevicePtr_t robot_;
  FrameIndex frameId_;
  pinocchio::Data data_;
  core::WeighedDistancePtr_t wd_;
}; // class ToolRotation
} // namespace coverage
} // namespace hpp

#endif // HPP_COVERAGE_COST_HH

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

#include <hpp/coverage/cost.hh>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <hpp/pinocchio/device.hh>

namespace hpp{
namespace coverage{

  typedef pinocchio::Configuration_t Configuration_t;
  typedef pinocchio::matrix3_t matrix3_t;
  typedef pinocchio::vector3_t vector3_t;
  
  ToolRotationPtr_t ToolRotation::create(const DevicePtr_t& robot, const std::string& gripperName)
  {
    return ToolRotationPtr_t(new ToolRotation(robot, gripperName));
  }

  ToolRotation::ToolRotation(const DevicePtr_t& robot, const std::string& gripperName) :
    robot_(robot), frameId_(robot->model().getFrameId(gripperName)), data_(robot->model()),
    wd_(core::WeighedDistance::create(robot))
  {
  }

  value_type ToolRotation::eval(const PathConstPtr_t& path)
  {
    Configuration_t q1(path->initial());
    Configuration_t q2(path->end());

    ::pinocchio::framesForwardKinematics(robot_->model(), data_, q1);
    matrix3_t R1(data_.oMf[frameId_].rotation());
    ::pinocchio::framesForwardKinematics(robot_->model(), data_, q2);
    matrix3_t R2(data_.oMf[frameId_].rotation());
    value_type theta;
    matrix3_t R(R1.inverse() * R2);
    vector3_t omega(::pinocchio::log3(R, theta));
    return 1e-3 * (*wd_)(q1, q2) + fabs(omega[0]);
  }
} // namespace coverage
} // namespace hpp

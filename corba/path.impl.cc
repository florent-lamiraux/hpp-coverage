// Copyright (c) 2025, LAAS-CNRS
// Authors: Florent Lamiraux
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

#include <hpp/common-idl.hh>
#include <hpp/corbaserver/servant-base.hh>
#include <hpp/corbaserver/conversions.hh>
#include <hpp/pinocchio_idl/robots-fwd.hh>
#include <hpp/core_idl/paths-fwd.hh>
#include <hpp/core_idl/path_planners-fwd.hh>
#include <hpp/coverage/cost.hh>
#include <../corba/path.impl.hh>
#include <../corba/path.hh>

namespace hpp {
namespace coverage {
namespace impl {

typedef core::Configuration_t Configuration_t;

Path::Path() : server_(0x0), path_() {}

::hpp::core_idl::Path_ptr Path::multiply(::hpp::core_idl::Path_ptr p1,
					 ::hpp::core_idl::Path_ptr p2)
{
  try {
    core::PathPtr_t path1(corbaServer::reference_to_object<core::Path>(server_->parent(), p1));
    core::PathPtr_t path2(corbaServer::reference_to_object<core::Path>(server_->parent(), p2));
    // Sanity checks
    std::ostringstream oss;
    if (p1->length() <= 0) {
      oss << "Path::multiplyPaths: length of p1 should be positive but is equal to "
	  << p1->length();
      throw std::logic_error(oss.str().c_str());
    }
    if (p1->outputSize() != 7) {
      oss << "Path::multiplyPaths: p1 should be with values in SE(3), but output size is "
	  << p1->outputSize();
      throw std::logic_error(oss.str().c_str());
    }
    if (p1->outputDerivativeSize() != 6) {
      oss << "Path::multiplyPaths: p1 should be with values in SE(3), but output derivative size is "
	  << p1->outputDerivativeSize();
      throw std::logic_error(oss.str().c_str());
    }
    if (p2->outputSize() != 7) {
      oss << "Path::multiplyPaths: p2 should be with values in SE(3), but output size is "
	  << p2->outputSize();
      throw std::logic_error(oss.str().c_str());
    }
    if (p2->outputDerivativeSize() != 6) {
      oss << "Path::multiplyPaths: p2 should be with values in SE(3), but output derivative "
	"size is " << p2->outputDerivativeSize();
      throw std::logic_error(oss.str().c_str());
    }
    hpp::core_idl::Path_var d =
      corbaServer::makeServantDownCast<core_impl::Path>(server_->parent(),
	path_.multiply(path1, path2));
    return d._retn();
  } catch(const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

hpp::core_idl::Path_ptr Path::createSpline(const floatSeq& pose0, const floatSeq& pose1,
					   CORBA::Double length, CORBA::ULong order)
{
  try {
    Configuration_t q0(corbaServer::floatSeqToConfig(path_.robot(), pose0, true));
    Configuration_t q1(corbaServer::floatSeqToConfig(path_.robot(), pose1, true));
    hpp::core_idl::Path_var d =
      corbaServer::makeServantDownCast<core_impl::Path>(server_->parent(),
        path_.createSpline(q0, q1, length, order));
    return d._retn();
  } catch(const std::exception& exc) {
    throw hpp::Error(exc.what());
  }
}

void Path::setCost(::hpp::core_idl::Roadmap_ptr roadmap,
		   ::hpp::pinocchio_idl::Device_ptr robot, const char* gripperName)
{
  core::RoadmapPtr_t _roadmap;
  try {
    _roadmap = corbaServer::reference_to_object<core::Roadmap>(server_->parent(), roadmap);
  } catch (const hpp::Error& e) {
    throw std::runtime_error("Failed to get pointer to roadmap in Path::setCost");
  }
  pinocchio::DevicePtr_t _robot;
  try {
    _robot = corbaServer::reference_to_object<pinocchio::Device>(server_->parent(), robot);
  } catch (const hpp::Error& e) {
    throw std::runtime_error("Failed to get pointer to robot in Path::setCost");
  }
  _roadmap->cost(coverage::ToolRotation::create(_robot, gripperName));
}

} // namespace impl
} // namespace coverage
} // namespace hpp

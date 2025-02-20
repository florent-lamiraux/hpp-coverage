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

#ifndef HPP_COVERAGE_CORBA_COVERAGE_IMPL_HH
#define HPP_COVERAGE_CORBA_COVERAGE_IMPL_HH

#include <corba/path-idl.hh>
#include <hpp/manipulation/fwd.hh>
#include <hpp/coverage/path.hh>

namespace hpp {
namespace coverage {
class Server;
namespace impl {

class Path : public virtual POA_hpp::corbaserver::coverage::Path
{
public:
  Path();
  void setServer(Server* server) { server_ = server; }

  virtual hpp::core_idl::Path_ptr multiply(hpp::core_idl::Path_ptr p1,
					   hpp::core_idl::Path_ptr p2);

  virtual hpp::core_idl::Path_ptr createSpline(const floatSeq& pose0, const floatSeq& pose1,
					       CORBA::Double length, CORBA::ULong order);
private:
  Server* server_;
  hpp::coverage::Path path_;
}; // class Coverage
} // namespace impl
} // namespace coverage
} // namespace hpp

#endif // HPP_COVERAGE_CORBA_COVERAGE_IMPL_HH

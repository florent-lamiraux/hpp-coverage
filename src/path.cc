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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <hpp/coverage/path.hh>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/device-data.hh>
#include <hpp/core/path.hh>
#include <hpp/core/problem.hh>

namespace hpp{
namespace coverage{

typedef core::value_type value_type;
typedef core::ConfigurationOut_t ConfigurationOut_t;
typedef core::Configuration_t Configuration_t;
typedef core::ConstraintSetPtr_t ConstraintSetPtr_t;
typedef core::vectorIn_t vectorIn_t;
typedef pinocchio::SE3 SE3;
typedef Eigen::Quaternion<value_type> Quaternion;
HPP_PREDEF_CLASS(Product);
typedef std::shared_ptr<Product> ProductPtr_t;
typedef Eigen::Matrix<value_type, 7, 1> vector7_t;

class Product : public core::Path
{
public:
  /// Return a shared pointer to a new instance
  static ProductPtr_t create(const PathPtr_t& p1, const PathPtr_t& p2)
  {
    ProductPtr_t shPtr(new Product(p1, p2));
    shPtr->init(shPtr);
    return shPtr;
  }

  /// Return a shared pointer to a copy of this
  static ProductPtr_t createCopy(const ProductPtr_t& p)
  {
    ProductPtr_t shPtr(new Product(*p));
    shPtr->init(shPtr);
    return shPtr;
  }

  /// Create copy and return shared pointer
  /// \param path path to copy
  /// \param constraints the path is subject to
  static ProductPtr_t createCopy(const ProductPtr_t& path,
                                 const ConstraintSetPtr_t& constraints) {
    ProductPtr_t shPtr(new Product(*path, constraints));
    shPtr->init(shPtr);
    return shPtr;
  }

  /// Return a shared pointer to a copy of this and set constraints
  ///
  /// \param constraints constraints to apply to the copy
  /// \pre *this should not have constraints.
  virtual PathPtr_t copy(const ConstraintSetPtr_t& constraints) const {
    return createCopy(weak_.lock(), constraints);
  }
  
  /// Return a shared pointer to a copy of this
  ///
  virtual PathPtr_t copy() const { return createCopy(weak_.lock()); }

  /// Get the initial configuration
  Configuration_t initial() const
  {
    vector7_t v1(p1_->initial()), v2(p2_->initial());
    vector7_t res;
    computeSE3Product(v1, v2, res);
    return res;
  }

  /// Get the final configuration
  Configuration_t end() const
  {
    vector7_t v1(p1_->end()), v2(p2_->end());
    vector7_t res;
    computeSE3Product(v1, v2, res);
    return res;
  }

protected:
  Product(const PathPtr_t& p1, const PathPtr_t& p2) : Path(std::make_pair(0, p1->length()), 7, 6),
      p1_(p1), p2_(p2)
  {
    assert(p1->length() > 0);
    assert(p1->outputSize() == 7);
    assert(p1->outputDerivativeSize() == 6);
    assert(p2->outputSize() == 7);
    assert(p2->outputDerivativeSize() == 6);
    assert(fabs(p2->length() - p1->length()) < 1e-7);
  }

  Product(const Product& p) : Path(p), p1_(p.p1_), p2_(p.p2_)
  {
  }

  /// Copy constructor with constraints
  Product(const Product& p, const ConstraintSetPtr_t& constraints) : Path(p, constraints),
      p1_(p.p1_), p2_(p.p2_)
  {
  }

  void computeSE3Product(vectorIn_t v1, vectorIn_t v2, ConfigurationOut_t configuration) const
  {
    SE3 T1(Quaternion(v1.tail<4>()), v1.head<3>());
    SE3 T2(Quaternion(v2.tail<4>()), v2.head<3>());
    SE3 res(T1*T2);
    configuration.tail<4>() = Quaternion(res.rotation()).coeffs();
    configuration.head<3>() = res.translation();
  }
  virtual bool impl_compute(ConfigurationOut_t configuration, value_type param) const
  {
    vector7_t v1, v2;
    if (!p1_->eval(v1, param + p1_->timeRange().first)) return false;
    if (!p2_->eval(v2, param + p2_->timeRange().first)) return false;
    computeSE3Product(v1, v2, configuration);
    return true;
  }

  void init(ProductWkPtr_t weak)
  {
    Path::init(weak);
    weak_ = weak;
  }
private:
  ProductWkPtr_t weak_;
  PathPtr_t p1_, p2_;
}; // class Product

Path::Path() : robot_(pinocchio::Device::create("")), sm1_(), sm3_()
{
  SE3 ISE3; ISE3.setIdentity();
  pinocchio::ModelPtr_t model(new pinocchio::Model);
  pinocchio::GeomModelPtr_t geomModel(new pinocchio::GeomModel);
  ::pinocchio::JointModelFreeFlyer freeflyer;
  model->addJoint(0, freeflyer, ISE3, "");
  robot_->setModel(model);
  robot_->setGeomModel(geomModel);
  robot_->createData();
  robot_->createGeomData();
  core::ProblemPtr_t problem(core::Problem::create(robot_));
  sm1_ = SteeringMethod1_t::create(problem);
  sm3_ = SteeringMethod3_t::create(problem);
}

PathPtr_t Path::multiply(const PathPtr_t& p1, const PathPtr_t& p2)
{
  return Product::create(p1, p2);
}

PathPtr_t Path::createSpline(ConfigurationIn_t pose0, ConfigurationIn_t pose1, value_type length,
			     size_type order)
{
  std::vector<int> empty, one; one.push_back(1);
  Eigen::Matrix<value_type, 7, 0> emptyDeriv;
  Eigen::Matrix<value_type, 6, 1> zeroDeriv; zeroDeriv.fill(0.);
  switch(order) {
  case 0:
    // Linear interpolation
    return sm1_->steer(pose0, empty, emptyDeriv, pose1, empty, emptyDeriv, length, true);
  case 1:
    // cubic spline with zero derivatives at beginning and end
    return sm3_->steer(pose0, one, zeroDeriv, pose1, one, zeroDeriv, length, true);
    break;
  default:
    std::ostringstream oss;
    oss << "hpp::coverage::Path::createSpline: order should be 0 or 1: got " << order;
    throw std::logic_error(oss.str().c_str());
  }
}
} // namespace coverage
} // namespace hpp

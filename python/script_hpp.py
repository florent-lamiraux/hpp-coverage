# Copyright 2025 CNRS
# Author: Florent Lamiraux
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
from math import sqrt, cos, sin, pi
from hpp.corbaserver import loadServerPlugin
from hpp.rostools import process_xacro, retrieve_resource
from hpp.corbaserver.manipulation import ConstraintGraph, ConstraintGraphFactory, Constraints, \
    Robot, newProblem, ProblemSolver
from hpp.gepetto.manipulation import ViewerFactory
from cartesian_trajectory import CartesianTrajectory
from trajectory_generator import centerAxes4and6, TrajectoryGenerator
from hpp.corbaserver.coverage import Client as CovClient

class Part:
    urdfFilename = "package://hpp-coverage/urdf/part.urdf"
    srdfFilename = "package://hpp-coverage/srdf/part.srdf"
    rootJointType = "freeflyer"

# Loading a manipulation server plugin and refresh problem
loadServerPlugin ("corbaserver", "manipulation-corba.so")
loadServerPlugin ("corbaserver", "coverage.so")
newProblem()

Robot.urdfFilename = "package://hpp-coverage/urdf/syabot.urdf"
Robot.srdfFilename = "package://hpp-coverage/srdf/syabot.srdf"

robot= Robot("staubli-part", "staubli", rootJointType="anchor")

ps = ProblemSolver(robot)
vf = ViewerFactory(ps)

vf.loadRobotModel(Part, "part")
robot.setJointBounds("part/root_joint", [-1., 1., -1., 1.,-1., 1.])
q0 = 6*[0.] + [-.7, 0, -0.1] + [0, 0, sqrt(2)/2, sqrt(2)/2]

# Create the constraint graph
cg = ConstraintGraph(robot, 'graph')
factory = ConstraintGraphFactory(cg)
factory.setGrippers(["staubli/tooltip"])
factory.setObjects(["part"], [["part/part_top"]], [[]])
factory.generate()
cg.initialize()

robot = ps.robot
for i in range(1000):
  q = robot.shootRandomConfig()
  res, q1, err = cg.generateTargetConfig("staubli/tooltip > part/part_top | f_01", q0, q)
  centerAxes4and6(robot, q1)
  if not res: continue
  res, msg = robot.isConfigValid(q1)
  if not res: continue
  res, q2, err = cg.generateTargetConfig("staubli/tooltip > part/part_top | f_12", q1, q1)
  if not res: continue
  res, msg = robot.isConfigValid(q2)
  if res: break

tg = TrajectoryGenerator(ps, cg)
# eep1 is a linear path of the end effector that follows the cylinder
eep1 = tg.straightCircle(q2)

ct = tg.cartesianTrajectory
cov = CovClient()

pose_0 = [0,0,0,0,0,0,1]
pose_1 = [0,0,0,sin(pi/8),0,0,cos(pi/8)]

eep2 = cov.path.createSpline(pose_0, pose_1, eep1.length(), 0)

# Rotate tool around x-axis at beginning of end-effector path
eep3 = cov.path.multiply(eep2, eep1)

ct.steeringMethod.trajectory(eep3, True)
eec = ps.client.basic.problem.getConstraint("staubli/tooltip follows target")
ct.configProjector.setRightHandSideFromConfig(q0)
ct.configProjector.setRightHandSideOfConstraint(eec, eep3.initial())
res3, q3 = ct.configProjector.apply(q2)
ct.configProjector.setRightHandSideOfConstraint(eec, eep3.end())
res4, q4 = ct.configProjector.apply(q2)
p1 = ct.computePath(q3, q4)
if p1:
    ps.client.basic.problem.addPath(p1.asVector())


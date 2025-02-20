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

from hpp.corbaserver import wrap_delete
from hpp_idl.hpp import Error as HppError

class CartesianTrajectory:
    threshold = 1e-5
    iterations = 40

    def wd(self, o):
        """! Wrapper to the wrap_delete method
        Automatically deletes the corresponding servant object on the server when 
        the Python object is deleted.
        
        @param o CORBA object
        """
        return wrap_delete(o, self.ps.client.basic._tools)

    def __init__(self, ps, graph):
        self.ps = ps
        self.graph = graph
        self.steeringMethodType = "EndEffectorTrajectory"
        self.pathPlannerType = "EndEffectorTrajectory"

        # Retrieve the current problem from problem solver
        self.currentProblem = \
            self.wd(self.ps.hppcorba.problem.getProblem())
        # Create a new robot from the current problem
        self.crobot = self.wd(self.currentProblem.robot())
        # Create a new problem with the robot
        self.newProblem = self.wd(self.ps.hppcorba.problem.createProblem\
                                (self.crobot))
        # build steering method
        self.steeringMethod = self.wd(self.ps.client.basic.problem.createSteeringMethod\
                                (self.steeringMethodType, self.newProblem))
        self.constraintSet = self.ps.hppcorba.problem.createConstraintSet\
                            (self.crobot, "sm-constraintSet")
        self.configProjector = self.ps.hppcorba.problem.createConfigProjector\
                            (self.crobot, "sm-configProjector", self.threshold, self.iterations)
        self.constraintSet.addConstraint(self.configProjector)
        self.newProblem.setConstraints(self.constraintSet)
        self.newProblem.setSteeringMethod(self.steeringMethod)
        self.croadmap = self.wd(self.ps.client.basic.problem.createRoadmap(
            self.wd(self.newProblem.getDistance()),
            self.crobot))
        self.pathPlanner = self.wd(self.ps.client.basic.problem.createPathPlanner(
            self.pathPlannerType, self.newProblem, self.croadmap)
        )
        self.pathPlanner.maxIterations(1)
        self.pathPlanner.setNRandomConfig(0)
        self.pathPlanner.setNDiscreteSteps(100)
        
        # Add constraint on pose of the part
        self.ps.client.basic.problem.createLockedJoint(
            "locked-part", "part/root_joint", [0,0,0,0,0,0,1])
        self.ps.setConstantRightHandSide("locked-part", False)
        lockPart = self.ps.client.basic.problem.getConstraint("locked-part")
        self.configProjector.add(lockPart, 0)
        # Create a parameterizable grasp constraint
        self.graph.createGrasp("staubli/tooltip follows target", "staubli/tooltip", "part/part_top")
        self.ps.setConstantRightHandSide("staubli/tooltip follows target", False)
        self.trajectoryConstraint = self.wd(self.ps.hppcorba.problem.getConstraint\
            ("staubli/tooltip follows target"))
        self.configProjector.add(self.trajectoryConstraint, 0)
        self.steeringMethod.trajectoryConstraint(self.trajectoryConstraint)

    def computePath(self, qInit, qGoal):
        """
        Compute a path between two configurations
        """
        # Set right hand side of config projector with initial configuration
        self.configProjector.setRightHandSideFromConfig(qInit)
        # Create an empty roadmap
        self.croadmap = self.wd(self.ps.hppcorba.problem.createRoadmap(
            self.wd(self.newProblem.getDistance()), self.crobot))
        self.newProblem.setInitConfig(qInit)
        self.newProblem.resetGoalConfigs()
        self.newProblem.addGoalConfig(qGoal)
        path = self.wd(self.pathPlanner.solve())
        return path

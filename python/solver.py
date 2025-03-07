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

from math import sqrt, cos, sin, pi
import numpy as np
from pinocchio import SE3, Quaternion
from hpp.corbaserver import wrap_delete
from hpp_idl.hpp import Error
from cartesian_trajectory import CartesianTrajectory
from hpp.corbaserver.coverage import Client as CovClient

import logging
logger = logging.getLogger(__name__)
logging.basicConfig(filename='solver.log', filemode='w', level=logging.INFO)

class Solver:

    nOrientations = 8
    """ Number of tool orientations along symmetry axis"""
    segmentTime = 2
    """ The input path is segmented in intervals of this length over which an orientation
        multiplyer is applied.
    """
    def wd(self, o):
        """! Wrapper to the wrap_delete method
        Automatically deletes the corresponding servant object on the server when 
        the Python object is deleted.
        
        @param o CORBA object
        """
        return wrap_delete(o, self.ps.client.basic._tools)
    
    def __init__(self, ps, cg):
        self.ps = ps
        self.robot = ps.robot
        self.cg = cg
        self.ct = CartesianTrajectory(ps, cg)
        self.cp = self.ct.configProjector
        self.client = CovClient()
        self.problem = self.wd(self.ps.hppcorba.problem.createProblem(self.ct.crobot))
        self.distance = self.wd(self.problem.getDistance())
        self.roadmap = self.wd(self.ps.client.basic.problem.createRoadmap(
            self.distance, self.ct.crobot))
        self.pathPlanner = self.wd(self.ps.client.basic.problem.createPathPlanner("SearchInRoadmap",
            self.problem, self.roadmap))

    def computeInitialConfigs(self, toolRotation, toolPose, q0):
        """Compute inverse kinematics solutions for a given tool pose with reorientation

        param q0: provides the pose of the part.
        """
        def appendIfNew(configs, q):
            for config in configs:
                if self.distance.call(config, q) < 1e-3:
                    return None
            configs.append(q)

        configs = list()
        p0 = SE3(Quaternion(np.array(toolRotation[3:7])), np.array(toolRotation[0:3]))
        p1 = SE3(Quaternion(np.array(toolPose[3:7])), np.array(toolPose[0:3]))
        self.cp.setRightHandSideFromConfig(q0)
        p = p0 * p1
        rhs = np.zeros(7)
        rhs[0:3] = p.translation; rhs[3:7] = Quaternion(p.rotation).coeffs()
        self.cp.setRightHandSideOfConstraint(self.ct.trajectoryConstraint, list(rhs))
        for i in range(1000):
            q = self.robot.shootRandomConfig()
            if i == 0: q = q0
            res, q1 = self.cp.apply(q)
            if not res: break
            res, msg = self.robot.isConfigValid(q1)
            if res: appendIfNew(configs, q1)
        return configs

    def compute(self, q0, tooltipTraj):
        res = self.tryConstantOrientations(q0, tooltipTraj)
        if res :
            qInit, qGoal = res
            logger.info(f"qInit={qInit}")
            logger.info(f"qGoal={qGoal}")
            self.problem.setInitConfig(qInit)
            self.problem.resetGoalConfigs()
            self.problem.addGoalConfig(qGoal)
            p = self.pathPlanner.solve()
            return p
        
    def tryConstantOrientations(self, q0, tooltipTraj):
        """
        Compute a robot path given the tooltip trajectory as input

        q0 defines the pose of the part
        """
        n = self.nOrientations
        toolRotations = [[0,0,0,sin(i*pi/n), 0, 0, cos(i*pi/n)] for i in range(n)]
        # First try a straight line for each orientation
        for toolRotation in toolRotations:
            # Generate all inverse kinematics solutions for initial tool pose
            qInits = self.computeInitialConfigs(toolRotation, tooltipTraj.initial(), q0)
            logger.info(f"Generated {len(qInits)} collision free configurations for toolRotation {toolRotation}")
            # Cut the tooltip path into segments of predefined time length.
            remainingTime = tooltipTraj.length()
            t0 = 0.
            lastSegment = False; plannerFailed = False
            qInitsSegment = qInits[:]
            origin = dict()
            while remainingTime > 0 and not plannerFailed:
                qEnds = list()
                if remainingTime > self.segmentTime:
                    toolTraj0 = tooltipTraj.extract(t0, t0 + self.segmentTime)
                    t0 += self.segmentTime
                    remainingTime -= self.segmentTime
                else:
                    toolTraj0 = tooltipTraj.extract(t0, tooltipTraj.length())
                    remainingTime = 0; lastSegment = True
                
                # create a constant multiplyer in SE(3) with the current orientation
                reorient = self.client.path.createSpline(toolRotation, toolRotation,
                                                         toolTraj0.length(), 0)
                toolTraj = self.client.path.multiply(reorient, toolTraj0)
                self.ct.steeringMethod.trajectory(toolTraj, True)
                for qInit in qInitsSegment:
                    qInit = tuple(qInit)
                    # Generate a possible end configuration for this tool motion
                    self.cp.setRightHandSideOfConstraint(self.ct.trajectoryConstraint,
                                                         toolTraj.end())
                    res, qEnd = self.cp.apply(qInit)
                    if not res: continue
                    try:
                        p = self.ct.computePath(qInit, qEnd)
                        qEnd = tuple(p.end())
                        self.ps.client.basic.problem.addPath(p)
                        self.roadmap.addNodeAndEdge(qInit, qEnd, p)
                        qEnds.append(qEnd)
                        if not qInit in origin:
                            origin[qEnd] = qInit
                        else:
                            origin[qEnd] = origin[qInit]
                        if lastSegment: return (origin[qEnd], qEnd)
                        p.deleteThis()
                    except Error as exc:
                        # Stop search for current orientation
                        logger.info(f"Planner failed: {exc}")
                        plannerFailed = True
                qInitsSegment = qEnds[:]
                reorient.deleteThis()
                toolTraj0.deleteThis()
        return None

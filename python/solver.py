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

class NoLogger:
    def info(self, msg):
        pass

logging = False

if logging:
    import logging
    logger = logging.getLogger(__name__)
    logging.basicConfig(filename='solver.log', filemode='w', level=logging.INFO)
else:
    logger = NoLogger()


class Solver:

    nRotations = 8
    """ Number of tool orientations along symmetry axis"""
    segmentTime = 2
    """ The input path is segmented in intervals of this length over which an orientation
        multiplyer is applied.
    """
    distanceThreshold = 1e-4
    """ Distance between configurations below which two configurations are assumed to be equal
    """
    orientationCoeff = 10
    """ coefficient of the change of orientation in the cost
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
        self.steeringMethod = self.wd(self.problem.getSteeringMethod())
        self.roadmap = self.wd(self.ps.client.basic.problem.createRoadmap(
            self.distance, self.ct.crobot))
        self.pathPlanner = self.wd(self.ps.client.basic.problem.createPathPlanner("SearchInRoadmap",
            self.problem, self.roadmap))

    def clearRoadmap(self):
        self.roadmap.clear()
        # maps the origin of the path that leads to each node
        self.origin = dict()
        # maps the travel time from trajectory start to each node
        self.cost = dict()
        # track initial configuration of path that leads to each node
        self.origin = dict()
        # store configuration in a radius corresponding to the numerical threshold
        self.near = dict()

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
        self.clearRoadmap()
        res = self.graphSearch(q0, tooltipTraj)
        if res :
            qInit, qGoal = res
            logger.info(f"qInit={qInit}")
            logger.info(f"qGoal={qGoal}")
            self.problem.setInitConfig(qInit)
            self.problem.resetGoalConfigs()
            self.problem.addGoalConfig(qGoal)
            p = self.pathPlanner.solve()
            return p

    def addNodeAndEdge(self, q0, q1, p):
        """
        Add a new node and an edge to the roadmap between an existing node and the new one

          intput:
            - q0: existing node,
            - q1: new node,
            - p:  path between q0 and q1.
          return:
            - q1, True if q1 was already in the roadmap (up to distance threshold),
            - q_near, False otherwise, where q_near is the closest node in the roadmap to q1

          note: if q1 is very close to an existing node of the roadmap, an edge is inserted
                q1 and the existing node with a straight path between them.
        """
        q_near, d = self.roadmap.nearestNode(q1, False); q_near = tuple(q_near)
        if d < self.distanceThreshold:
            # When several nodes are close to each other, always take the same as nearest node
            if q_near in self.near:
                self.near[tuple(q1)] = self.near[q_near]
                q_near = self.near[tuple(q1)]
            else:
                self.near[tuple(q1)] = q_near
            assert(q_near in self.cost)
            p1 = self.steeringMethod.call(q1, q_near)
            assert(p)
            self.roadmap.addNodeAndEdge(q0, q1, p)
            self.roadmap.addNodeAndEdges(q1, q_near, p1)
            p1.deleteThis()
            return q_near, False
        else:
            self.roadmap.addNodeAndEdge(q0, q1, p)
            return q1, True

    def graphSearch(self, q0, tooltipTraj):
        """
        Build and explore a roadmap with Dijkstra's algorithm.

        the cost to go for each node is the sum of the time with the number of orientation changes
        """
        n = self.nRotations
        toolRotations = [[0,0,0,sin(i*pi/n), 0, 0, cos(i*pi/n)] for i in range(n)]
        # list of triples (configuration, index of tool orientation, time)
        unvisited = list()
        # Generate initial configurations for each orientation
        for i, toolRotation in enumerate(toolRotations):
            # Generate all inverse kinematics solutions for initial tool pose
            qInits = self.computeInitialConfigs(toolRotation, tooltipTraj.initial(), q0)
            logger.info(f"Generated {len(qInits)} collision free configurations for toolRotation {toolRotation}")
            unvisited += [(tuple(q), i, 0) for q in qInits]
            for q in qInits:
                self.cost[tuple(q)] = 0
                self.roadmap.addNode(q)
        finished = False
        nIter = 0
        while not finished:
            for q0, i0, t0 in unvisited:
                # try constant and 2 neighboring orientations
                for i1 in range(i0-1,i0+2):
                    # set i1 between 0 and len(toolRotations)-1
                    if i1 < 0: i1 += len(toolRotations)
                    if i1 >= len(toolRotations): i1-=len(toolRotations)
                    # foreach new orientation, generate to tool trajectory
                    t1 = t0 + self.segmentTime
                    reachedEnd = False
                    if t1 >= tooltipTraj.length():
                        t1 = tooltipTraj.length()
                        reachedEnd = True
                    # extract the relevant sub-interval of the tooltip trajectory
                    toolTraj0 = tooltipTraj.extract(t0, t1)
                    # create a varying multiplyer in SE(3) from orientation i0 to orientation i1
                    tr0 = toolRotations[i0]; tr1 = toolRotations[i1]
                    reorient = self.client.path.createSpline(tr0, tr1, toolTraj0.length(), 1)
                    toolTraj = self.client.path.multiply(reorient, toolTraj0)
                    self.ct.steeringMethod.trajectory(toolTraj, True)
                    # Generate a possible end configuration for this tool motion
                    self.cp.setRightHandSideOfConstraint(self.ct.trajectoryConstraint,
                                                         toolTraj.end())
                    reorient.deleteThis(); toolTraj0.deleteThis(); toolTraj.deleteThis()
                    res, q1 = self.cp.apply(q0)
                    if not res:
                        logger.info(f"Failed to project {q0}")
                        logger.info(f"Projection stopped at {q1}")
                        continue
                    # compute path of robot
                    try:
                        p = self.ct.computePath(q0, q1)
                        q1 = tuple(p.end())
                        self.ps.client.basic.problem.addPath(p)
                        q1, new = self.addNodeAndEdge(q0, q1, p)
                        p.deleteThis()
                        logger.info(f"Added edge between {q0}")
                        logger.info(f"               and {q1}")
                        if reachedEnd:
                            return self.origin[q0], q1
                        cost1 = self.cost[q0] + (t1-t0) + self.orientationCoeff * abs(i1-i0)
                        if new:
                            unvisited.append((q1, i1, t1))
                            self.cost[q1] = cost1
                            self.origin[q1] = self.origin[q0] if q0 in self.origin else q0
                        elif self.cost[q1] > cost1:
                            # if the node already exist with a higher cost, update cost
                            # and origin
                            self.cost[q1] = cost1
                            self.origin[q1] = self.origin[q0] if q0 in self.origin else q0
                    except Error as exc:
                        # Stop search for current orientation
                        logger.info(f"Planner failed between {q0}")
                        logger.info(f"                   and {q1}")
                        continue
                    if finished: break
                if finished: break
                # sort unvisited in increasing cost
                unvisited.remove((q0, i0, t0))
                unvisited.sort(key = lambda x:self.cost[x[0]])
            nIter += 1
        # end while not finished

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
from hpp.corbaserver.coverage import Client as CovClient

def centerAxes4and6(robot, q):
    """
    Rotate axis 6 of robot by +/-2 pi to move away from joint bounds.
    """
    r = robot.rankInConfiguration['staubli/joint_4']
    if q[r] < pi: q[r]+=2*pi
    if q[r] > pi: q[r]-=2*pi
    r = robot.rankInConfiguration['staubli/joint_6']
    if q[r] < pi: q[r]+=2*pi
    if q[r] > pi: q[r]-=2*pi
    
class TrajectoryGenerator:

    linVel = .1
    """Linear velocity of the tool"""

    def __init__(self, ps, cg):
        self.ps = ps
        self.cg = cg
        self.client = CovClient()

    def straightCircle(self, q2):
        """
        Generate a single circular trajectory along the cylinder

        q2 is a configuration in contact with the part and that defines the pose of the part.
        """
        angle = pi/10
        a0 = angle
        a1 = -angle
        r = .5
        t = 2*r*angle/self.linVel
        rhs0 = [r*(1-cos(a0)), r*sin(a0), 0, 0, 0, -sin(a0/2), cos(a0/2)]
        rhs1 = [r*(1-cos(a1)), r*sin(a1), 0, 0, 0, -sin(a1/2), cos(a1/2)]

        p = self.client.path.createSpline(rhs0, rhs1, t, 0)
        return p
        
    def coverRectangle(self, q2, n):
        """
        Generate a rectangular sweeping motion on a cylinder of radius 50cm with n back and forth
        segments

        q2 is a configuration in contact with the part and that defines the pose of the part.
        """
        angle = pi/10
        a0 = angle
        a1 = -angle
        r = .5
        t = 2*r*angle/self.linVel
        rhs0 = [r*(1-cos(a0)), r*sin(a0), -0.01, 0, 0, -sin(a0/2), cos(a0/2)]
        rhs1 = [r*(1-cos(a1)), r*sin(a1), -0.01, 0, 0, -sin(a1/2), cos(a1/2)]

        p = None
        for i in range(n):
            p1 = self.client.path.createSpline(rhs0, rhs1, t, 0)
            if not p:
                p = p1.asVector()
            else:
                p.appendPath(p1)
            p1.deleteThis()
            dy = .002
            ty = dy/self.linVel
            rhs2 = rhs1[:]; rhs2[2] += dy
            rhs3 = rhs0[:]; rhs3[2] += dy
            p1 = self.client.path.createSpline(rhs1, rhs2, ty, 0)
            p.appendPath(p1); p1.deleteThis()
            p1 = self.client.path.createSpline(rhs2, rhs3, t, 0)
            p.appendPath(p1); p1.deleteThis()
            rhs0[2] += 0.004
            rhs1[2] += 0.004
            p1 = self.client.path.createSpline(rhs3, rhs0, ty, 0)
            p.appendPath(p1); p1.deleteThis()

        return p


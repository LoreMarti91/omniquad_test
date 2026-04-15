# new_trajectory.py
import math
import numpy as np


class NewTrajectory:

    def __init__(self):
        self.last_valid_q = None

    # ─────────────────────────────
    # DIRECT KINEMATIC
    # ─────────────────────────────
    def direct_kinematic(self, q, l, offset):
        x = l[0]*math.cos(q[0] + offset[0]) + l[1]*math.cos(q[0]+q[1] + offset[0]+offset[1])
        y = l[0]*math.sin(q[0] + offset[0]) + l[1]*math.sin(q[0]+q[1] + offset[0]+offset[1])
        return [x, y]

    # ─────────────────────────────
    # INVERSE KINEMATIC
    # ─────────────────────────────
    def inverse_kinematic(self, p, l, k, offset):

        r = math.sqrt(p[0]**2 + p[1]**2)

        cos_knee = (r**2 - l[0]**2 - l[1]**2) / (2*l[0]*l[1])
        cos_knee = max(min(cos_knee, 1), -1)

        sin_knee = k * math.sqrt(1 - cos_knee**2)
        q_knee = math.atan2(sin_knee, cos_knee)

        k1 = l[0] + l[1]*math.cos(q_knee)
        k2 = l[1]*math.sin(q_knee)

        q_hip = math.atan2(p[1], p[0]) - math.atan2(k2, k1)

        q = [q_hip - offset[0], q_knee - offset[1]]

        if any(math.isnan(x) for x in q):
            return self.last_valid_q

        self.last_valid_q = q
        return q

    # ─────────────────────────────
    # LINEAR TRAJECTORY (cartesian)
    # ─────────────────────────────
    def linear(self, p0, pf, t0, tf, t):
        p0 = np.array(p0)
        pf = np.array(pf)

        if tf == t0:
            return p0

        return (pf - p0)/(tf - t0)*(t - t0) + p0

    # ─────────────────────────────
    # PARABOLIC
    # ─────────────────────────────
    def parabolic(self, p0, pf, t0, tf, t):
        p0 = np.array(p0)
        pf = np.array(pf)

        a = (pf - p0) / (tf - t0)**2
        return a*(t - t0)**2 + p0

    # ─────────────────────────────
    # LINEAR + IK
    # ─────────────────────────────
    def linear_ik(self, p0, pf, t0, tf, t, l, offset, k):

        p = self.linear(p0, pf, t0, tf, t)

        return self.inverse_kinematic(p, l, k, offset)
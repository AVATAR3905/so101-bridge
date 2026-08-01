"""
Inverse Kinematics solver for SO-101 using damped least squares (Levenberg-Marquardt).
Finds joint angles to reach a target (x, y) position.
"""
import logging
import math

import numpy as np

log = logging.getLogger(__name__)

LINK_LENGTHS = [117.0, 130.0, 124.0, 60.0]  # mm

def fk_3d(joints_deg):
    """3D forward kinematics: [J0..J3] deg → {x, y, z} mm."""
    j0, j1, j2, j3 = [math.radians(j) for j in joints_deg[:4]]
    L = LINK_LENGTHS
    t1 = j1
    t2 = j1 + j2
    t3 = j1 + j2 + j3
    x_local = L[0]*math.cos(t1) + L[1]*math.cos(t2) + (L[2]+L[3])*math.cos(t3)
    y_local = L[0]*math.sin(t1) + L[1]*math.sin(t2) + (L[2]+L[3])*math.sin(t3)
    return {"x": round(x_local * math.cos(j0), 1),
            "y": round(y_local, 1),
            "z": round(x_local * math.sin(j0), 1)}

def _jacobian_numerical(joints_deg):
    """Compute 2x4 Jacobian (dx/dq, dy/dq) numerically for planar IK."""
    eps = 0.001
    J = np.zeros((2, 4))
    for i in range(4):
        jp = list(joints_deg); jp[i] += eps
        jm = list(joints_deg); jm[i] -= eps
        fp = fk_3d(jp); fm = fk_3d(jm)
        J[0, i] = (fp["x"] - fm["x"]) / (2 * eps)
        J[1, i] = (fp["y"] - fm["y"]) / (2 * eps)
    return J

def solve_ik(target_x, target_y, initial_joints, max_iters=50, tol=1.0, damping=0.5):
    """Solve IK for (x, y) target. Returns list of 4 joint angles in degrees."""
    q = np.array(initial_joints[:4], dtype=float)
    for _ in range(max_iters):
        fk = fk_3d(q)
        dx = target_x - fk["x"]
        dy = target_y - fk["y"]
        err = math.sqrt(dx*dx + dy*dy)
        if err < tol:
            break
        J = _jacobian_numerical(q)
        # Damped least squares: dq = J^T (J J^T + λ² I)^(-1) dx
        JJT = J @ J.T
        JJT += np.eye(2) * (damping ** 2)
        dq = J.T @ np.linalg.solve(JJT, np.array([dx, dy]))
        q = q + dq
        # Clamp to reasonable ranges
        q[0] = np.clip(q[0], -100, 100)
        q[1] = np.clip(q[1], -120, 120)
        q[2] = np.clip(q[2], -120, 120)
        q[3] = np.clip(q[3], -90, 90)
    return [round(float(v), 1) for v in q]

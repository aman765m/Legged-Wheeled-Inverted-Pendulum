#!/usr/bin/env python3
"""Procedurally generate the STL meshes of the wheel-legged biped robot with arm.

The authors of Raza et al. (IEEE Access 2021) did not release CAD files, so the
geometry here is built from scratch to resemble Fig. 1 of the paper while
matching the dimensions of Table 1 (wheel radius, track width, arm link lengths).
Only visual meshes are generated; collision geometry uses URDF primitives.

Usage:  python3 generate_meshes.py [output_dir]
Only numpy is required.  All units are metres; every mesh is expressed in the
frame of the URDF link that uses it (see urdf/wheel_legged_robot.urdf.xacro).
"""
import os
import struct
import sys

import numpy as np

# ----------------------------------------------------------------------------
# Dimensions (keep in sync with urdf/wheel_legged_robot.urdf.xacro)
# ----------------------------------------------------------------------------
TORSO = (0.14, 0.32, 0.34)      # x, y, z box of the torso (origin = body CoM)
PEDESTAL_H = 0.06               # shoulder pedestal on top of the torso
LEG_LEN = 0.26                  # thigh and shank length
LEG_R = 0.018
WHEEL_R = 0.1016                # Table 1: r_w
WHEEL_W = 0.04
ARM_L = 0.30                    # Table 1: l1 = l2
ARM_R = 0.016
CAN_R, CAN_H = 0.033, 0.122     # 330 ml can of cola


class Mesh:
    """Minimal triangle soup with helpers for primitive solids."""

    def __init__(self):
        self.tris = []

    def add(self, other, T=None):
        tris = np.asarray(other.tris if isinstance(other, Mesh) else other, dtype=float)
        if tris.size == 0:
            return self
        if T is not None:
            pts = tris.reshape(-1, 3)
            pts = (T[:3, :3] @ pts.T).T + T[:3, 3]
            tris = pts.reshape(-1, 3, 3)
        self.tris.extend(list(tris))
        return self

    def quad(self, a, b, c, d):
        self.tris.append(np.array([a, b, c]))
        self.tris.append(np.array([a, c, d]))

    # ---- primitives --------------------------------------------------------
    @staticmethod
    def box(sx, sy, sz, center=(0, 0, 0)):
        m = Mesh()
        c = np.array(center, float)
        h = np.array([sx, sy, sz]) / 2.0
        v = [c + h * np.array([x, y, z]) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
        idx = lambda x, y, z: v[(x > 0) * 4 + (y > 0) * 2 + (z > 0)]  # noqa: E731
        # six faces, counter-clockwise seen from outside
        m.quad(idx(1, -1, -1), idx(1, 1, -1), idx(1, 1, 1), idx(1, -1, 1))
        m.quad(idx(-1, -1, -1), idx(-1, -1, 1), idx(-1, 1, 1), idx(-1, 1, -1))
        m.quad(idx(-1, 1, -1), idx(-1, 1, 1), idx(1, 1, 1), idx(1, 1, -1))
        m.quad(idx(-1, -1, -1), idx(1, -1, -1), idx(1, -1, 1), idx(-1, -1, 1))
        m.quad(idx(-1, -1, 1), idx(1, -1, 1), idx(1, 1, 1), idx(-1, 1, 1))
        m.quad(idx(-1, -1, -1), idx(-1, 1, -1), idx(1, 1, -1), idx(1, -1, -1))
        return m

    @staticmethod
    def tube(r_out, r_in, z0, z1, n=48):
        """Solid (r_in=0) or hollow cylinder along +z between z0 and z1."""
        m = Mesh()
        th = np.linspace(0, 2 * np.pi, n + 1)
        P = lambda r, t, z: np.array([r * np.cos(t), r * np.sin(t), z])  # noqa: E731
        for i in range(n):
            a, b = th[i], th[i + 1]
            m.quad(P(r_out, a, z0), P(r_out, b, z0), P(r_out, b, z1), P(r_out, a, z1))
            if r_in > 0:
                m.quad(P(r_in, a, z0), P(r_in, a, z1), P(r_in, b, z1), P(r_in, b, z0))
                m.quad(P(r_in, a, z1), P(r_out, a, z1), P(r_out, b, z1), P(r_in, b, z1))
                m.quad(P(r_in, a, z0), P(r_in, b, z0), P(r_out, b, z0), P(r_out, a, z0))
            else:
                c1, c0 = np.array([0, 0, z1]), np.array([0, 0, z0])
                m.tris.append(np.array([c1, P(r_out, a, z1), P(r_out, b, z1)]))
                m.tris.append(np.array([c0, P(r_out, b, z0), P(r_out, a, z0)]))
        return m

    @staticmethod
    def torus(R, r, n=64, k=20):
        """Torus around +z (used for the rounded tyre)."""
        m = Mesh()
        u = np.linspace(0, 2 * np.pi, n + 1)
        w = np.linspace(0, 2 * np.pi, k + 1)
        P = lambda a, b: np.array([(R + r * np.cos(b)) * np.cos(a),  # noqa: E731
                                   (R + r * np.cos(b)) * np.sin(a), r * np.sin(b)])
        for i in range(n):
            for j in range(k):
                m.quad(P(u[i], w[j]), P(u[i + 1], w[j]), P(u[i + 1], w[j + 1]), P(u[i], w[j + 1]))
        return m

    @staticmethod
    def sphere(r, center=(0, 0, 0), n=24, k=12):
        m = Mesh()
        c = np.array(center, float)
        u = np.linspace(0, 2 * np.pi, n + 1)
        w = np.linspace(-np.pi / 2, np.pi / 2, k + 1)
        P = lambda a, b: c + r * np.array([np.cos(b) * np.cos(a), np.cos(b) * np.sin(a), np.sin(b)])  # noqa: E731
        for i in range(n):
            for j in range(k):
                m.quad(P(u[i], w[j]), P(u[i + 1], w[j]), P(u[i + 1], w[j + 1]), P(u[i], w[j + 1]))
        return m

    @staticmethod
    def capsule(r, length, n=32):
        """Capsule along +z from 0 to length."""
        m = Mesh.tube(r, 0, 0, length, n)
        m.add(Mesh.sphere(r, (0, 0, 0), n, 8))
        m.add(Mesh.sphere(r, (0, 0, length), n, 8))
        return m

    # ---- output ------------------------------------------------------------
    def save(self, path):
        tris = np.asarray(self.tris, dtype=np.float32)
        with open(path, 'wb') as f:
            f.write(b'wheel_legged_arm_balance procedural mesh'.ljust(80, b' '))
            f.write(struct.pack('<I', len(tris)))
            for t in tris:
                nrm = np.cross(t[1] - t[0], t[2] - t[0])
                ln = np.linalg.norm(nrm)
                nrm = nrm / ln if ln > 1e-12 else nrm
                f.write(struct.pack('<12fH', *nrm, *t.reshape(-1), 0))
        print(f'  wrote {os.path.basename(path):24s} {len(tris):6d} triangles')


def rot(axis, ang, trans=(0, 0, 0)):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
    T[:3, 3] = trans
    return T


Z2Y = rot([1, 0, 0], -np.pi / 2)   # maps +z axis onto +y axis
Z2X = rot([0, 1, 0], np.pi / 2)    # maps +z axis onto +x axis


def torso():
    sx, sy, sz = TORSO
    m = Mesh.box(sx, sy, sz)
    # front and back cover plates with a small recess look (like Fig. 1b)
    m.add(Mesh.box(0.008, sy - 0.03, sz - 0.03, (sx / 2 + 0.004, 0, 0)))
    m.add(Mesh.box(0.008, sy - 0.03, sz - 0.03, (-sx / 2 - 0.004, 0, 0)))
    # horizontal ribs on the sides
    for z in np.linspace(-sz / 2 + 0.05, sz / 2 - 0.05, 5):
        m.add(Mesh.box(sx - 0.02, 0.006, 0.012, (0, sy / 2 + 0.003, z)))
        m.add(Mesh.box(sx - 0.02, 0.006, 0.012, (0, -sy / 2 - 0.003, z)))
    # shoulder pedestal on top of the torso
    m.add(Mesh.tube(0.03, 0, sz / 2, sz / 2 + PEDESTAL_H))
    m.add(Mesh.box(0.10, 0.10, 0.01, (0, 0, sz / 2 + 0.005)))
    return m


def motor(radius=0.035, width=0.05):
    """Red actuator housing, axis along +z, centred at the origin."""
    m = Mesh.tube(radius, 0, -width / 2, width / 2)
    m.add(Mesh.tube(radius * 0.55, 0, width / 2, width / 2 + 0.008))
    m.add(Mesh.tube(radius * 0.55, 0, -width / 2 - 0.008, -width / 2))
    return m


def leg_link():
    """Thigh / shank: capsule from the origin down to -LEG_LEN along -z."""
    m = Mesh()
    m.add(Mesh.capsule(LEG_R, LEG_LEN), rot([1, 0, 0], np.pi))
    return m


def wheel_tyre():
    m = Mesh()
    r_t = WHEEL_W / 2
    m.add(Mesh.torus(WHEEL_R - r_t, r_t), Z2Y)
    return m


def wheel_rim():
    m = Mesh()
    rim_r = WHEEL_R - WHEEL_W + 0.002
    m.add(Mesh.tube(rim_r, rim_r - 0.008, -0.016, 0.016), Z2Y)       # rim ring
    m.add(Mesh.tube(0.022, 0, -0.022, 0.022), Z2Y)                    # hub
    for k in range(5):                                                 # spokes
        a = 2 * np.pi * k / 5
        spoke = Mesh.box(0.012, 0.010, rim_r - 0.02, (0, 0, (rim_r + 0.02) / 2 - 0.004))
        m.add(spoke, rot([0, 1, 0], a))
    # thin disk between the spokes
    m.add(Mesh.tube(rim_r - 0.004, 0.022, -0.003, 0.003), Z2Y)
    return m


def arm_link():
    """Arm link from the joint at the origin to +ARM_L along +x."""
    m = Mesh()
    m.add(Mesh.capsule(ARM_R, ARM_L), Z2X)
    m.add(Mesh.box(ARM_L * 0.8, 0.012, 0.045, (ARM_L / 2, 0, 0)))
    return m


def gripper():
    """Simple parallel gripper wrapped around the can (origin = can axis)."""
    m = Mesh()
    m.add(Mesh.box(0.02, 0.09, 0.05, (-CAN_R - 0.012, 0, 0)))          # palm
    for s in (-1, 1):
        m.add(Mesh.box(0.075, 0.012, 0.04, (-0.005, s * (CAN_R + 0.008), 0)))
    return m


def can():
    m = Mesh.tube(CAN_R, 0, -CAN_H / 2 + 0.006, CAN_H / 2 - 0.006)
    m.add(Mesh.tube(CAN_R * 0.85, 0, CAN_H / 2 - 0.006, CAN_H / 2))
    m.add(Mesh.tube(CAN_R * 0.85, 0, -CAN_H / 2, -CAN_H / 2 + 0.006))
    return m


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, '..', 'meshes')
    os.makedirs(out, exist_ok=True)
    print(f'Generating meshes in {os.path.abspath(out)}')
    torso().save(os.path.join(out, 'torso.stl'))
    motor().save(os.path.join(out, 'motor.stl'))
    leg_link().save(os.path.join(out, 'leg_link.stl'))
    wheel_tyre().save(os.path.join(out, 'wheel_tyre.stl'))
    wheel_rim().save(os.path.join(out, 'wheel_rim.stl'))
    arm_link().save(os.path.join(out, 'arm_link.stl'))
    gripper().save(os.path.join(out, 'gripper.stl'))
    can().save(os.path.join(out, 'can.stl'))


if __name__ == '__main__':
    main()

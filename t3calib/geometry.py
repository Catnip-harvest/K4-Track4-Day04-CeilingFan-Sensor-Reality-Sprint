"""Extrinsic drift and LiDAR-to-image projection.

A drift is modelled as the LiDAR moving on its mount: the stale calibration T is applied to points
measured in the moved LiDAR frame, which is the same as using T @ drift on the true points.
Axes are the LiDAR's own: x forward, y left, z up. Roll is about x, pitch about y, yaw about z.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

ROTATION_AXES = ("roll", "pitch", "yaw")
TRANSLATION_AXES = ("tx", "ty", "tz")


@dataclass(frozen=True)
class Drift:
    roll: float = 0.0  # degrees
    pitch: float = 0.0
    yaw: float = 0.0
    tx: float = 0.0  # metres
    ty: float = 0.0
    tz: float = 0.0

    def matrix(self) -> np.ndarray:
        drift = np.eye(4)
        drift[:3, :3] = rotation_from_euler(self.roll, self.pitch, self.yaw)
        drift[:3, 3] = [self.tx, self.ty, self.tz]
        return drift

    def applied_to(self, velo_to_cam: np.ndarray) -> np.ndarray:
        return velo_to_cam @ self.matrix()

    @staticmethod
    def along(axis: str, amount: float) -> "Drift":
        return Drift(**{axis: amount})


def rotation_from_euler(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll, pitch, yaw = np.radians([roll_deg, pitch_deg, yaw_deg])
    about_x = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    about_y = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    about_z = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    return about_z @ about_y @ about_x


def drift_between(reference: np.ndarray, estimate: np.ndarray) -> tuple[float, float]:
    """Remaining (rotation in degrees, translation in metres) between two velo_to_cam extrinsics."""
    relative = np.linalg.inv(reference) @ estimate
    cosine = np.clip((np.trace(relative[:3, :3]) - 1) / 2, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine))), float(np.linalg.norm(relative[:3, 3]))


@dataclass
class Projection:
    pixels: np.ndarray  # (N, 2) u, v; meaningless where visible is False
    depth: np.ndarray  # (N,) camera depth in metres
    visible: np.ndarray  # (N,) inside the image and in front of the camera


def project(points: np.ndarray, lidar_to_image: np.ndarray, image_size: tuple[int, int], min_depth: float = 1.0) -> Projection:
    width, height = image_size
    homogeneous = np.c_[points[:, :3], np.ones(len(points))]
    image_coords = homogeneous @ lidar_to_image.T
    depth = image_coords[:, 2]
    in_front = depth > min_depth
    pixels = np.zeros((len(points), 2))
    pixels[in_front] = image_coords[in_front, :2] / depth[in_front, None]
    visible = in_front & (pixels[:, 0] >= 0) & (pixels[:, 0] < width) & (pixels[:, 1] >= 0) & (pixels[:, 1] < height)
    return Projection(pixels, depth, visible)


def bounding_box(pixels: np.ndarray, image_size: tuple[int, int]) -> tuple[float, float, float, float]:
    width, height = image_size
    u0, v0 = np.clip(pixels.min(axis=0), 0, [width - 1, height - 1])
    u1, v1 = np.clip(pixels.max(axis=0), 0, [width - 1, height - 1])
    return float(u0), float(v0), float(u1), float(v1)


def inside_box(pixels: np.ndarray, box: tuple[float, float, float, float]) -> np.ndarray:
    u0, v0, u1, v1 = box
    return (pixels[:, 0] >= u0) & (pixels[:, 0] <= u1) & (pixels[:, 1] >= v0) & (pixels[:, 1] <= v1)


def shrink_box(box: tuple[float, float, float, float], keep: float) -> tuple[float, float, float, float]:
    """Central part of a box: keep=0.5 keeps the middle half in each direction."""
    u0, v0, u1, v1 = box
    du, dv = (u1 - u0) * (1 - keep) / 2, (v1 - v0) * (1 - keep) / 2
    return u0 + du, v0 + dv, u1 - du, v1 - dv

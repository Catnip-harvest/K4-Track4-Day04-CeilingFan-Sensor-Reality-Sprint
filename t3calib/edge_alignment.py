"""Targetless calibration check and refinement after Levinson & Thrun, RSS 2013.

Idea: where the LiDAR sees a depth jump (the edge of a car, a pole, a wall), the camera usually sees an
intensity edge. With a correct extrinsic, projected LiDAR depth edges land on image edges, so

    score(T) = sum_p  w_p * D(project(T, p)) / sum_p w_p      (p over LiDAR edge points in view)

is highest at the true calibration. w_p is the size of the depth jump at LiDAR point p and D is the
image edge map spread out with an exponential fall-off (an "inverse distance transform") so the
score changes smoothly as T moves.

Two uses:
  * monitor: if the current extrinsic is calibrated, every small neighbour of it scores lower. The
    fraction of neighbours that score lower is the health signal (Levinson & Thrun section IV-B).
  * refine: a coordinate search over roll/pitch/yaw that climbs the score from the drifted extrinsic.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass

import cv2
import numpy as np

from .geometry import Drift, project
from .kitti import Calibration, Frame


def image_edge_field(image_rgb: np.ndarray, alpha: float = 1 / 3, decay: float = 0.8, spread_px: int = 15,
                     blur_sigma: float = 1.5) -> np.ndarray:
    """Edge strength in [0, 1], plus a decaying halo so near misses still score something.

    The paper uses decay 0.98; on KITTI's 1242 x 375 images that halo is so wide that the score is
    almost flat and its yaw maximum moves to +3 deg. With decay 0.8 over 15 px the peak of the improved
    variant lies 0.15 deg from the KITTI calibration on drives 0048 + 0005 and 0.21 deg on held-out
    drive 0052 (results/holdout/). The light blur suppresses road texture.
    """
    gray = cv2.GaussianBlur(cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY).astype(np.float32), (0, 0), blur_sigma)
    gradient = np.hypot(cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3))
    edges = np.clip(gradient / max(np.percentile(gradient, 99), 1e-6), 0, 1)

    halo = edges.copy()
    neighbourhood = np.ones((3, 3), np.uint8)
    for _ in range(spread_px):  # each step: a pixel inherits its strongest neighbour, decayed
        np.maximum(halo, cv2.dilate(halo, neighbourhood) * decay, out=halo)
    return alpha * edges + (1 - alpha) * halo


def lidar_depth_edges(scan: np.ndarray, across_rings: bool = True, min_jump_m: float = 0.5, exponent: float = 0.5,
                      min_height_m: float = -1.3) -> tuple[np.ndarray, np.ndarray]:
    """Points that are nearer than a neighbour, with weight jump**exponent.

    KITTI stores a scan ring after ring (64 rings, top to bottom), each ring ordered by azimuth; a ring
    ends where the azimuth wraps by more than half a turn.

    along the ring   (the paper) neighbours left/right -> finds the vertical outline of objects,
                     which pins down yaw but says little about pitch and roll.
    across rings     (our addition) the point at the same azimuth on the ring above/below -> finds
                     horizontal outlines such as car roofs, which pins down pitch. Points below
                     min_height_m (LiDAR is 1.73 m above the road) are skipped, otherwise the spacing
                     between ground rings would look like depth jumps everywhere.
    """
    xyz = scan[:, :3]
    distance = np.linalg.norm(xyz, axis=1)
    azimuth = np.arctan2(xyz[:, 1], xyz[:, 0])
    ring_starts = np.flatnonzero(np.abs(np.diff(azimuth)) > np.pi) + 1
    starts_ring = np.zeros(len(xyz), bool)
    starts_ring[ring_starts] = True
    ends_ring = np.r_[starts_ring[1:], False]

    previous = np.r_[distance[0], distance[:-1]]
    following = np.r_[distance[1:], distance[-1]]
    previous[starts_ring] = distance[starts_ring]
    following[ends_ring] = distance[ends_ring]
    jump = np.maximum(np.maximum(previous - distance, following - distance), 0.0)

    if across_rings:
        rings = np.split(np.arange(len(xyz)), ring_starts)
        for ring, other in list(zip(rings[:-1], rings[1:])) + list(zip(rings[1:], rings[:-1])):
            order = np.argsort(azimuth[other])
            nearest = np.searchsorted(azimuth[other][order], azimuth[ring]).clip(0, len(other) - 1)
            neighbour = other[order][nearest]
            usable = (np.abs(azimuth[neighbour] - azimuth[ring]) < np.radians(0.4)) & (xyz[ring, 2] > min_height_m)
            jump[ring[usable]] = np.maximum(jump[ring[usable]], distance[neighbour[usable]] - distance[ring[usable]])

    keep = (jump >= min_jump_m) & (xyz[:, 0] > 0.5)
    return xyz[keep], jump[keep] ** exponent


@dataclass
class EdgeFrame:
    points: np.ndarray
    weights: np.ndarray
    field: np.ndarray
    image_size: tuple[int, int]


@dataclass
class HealthCheck:
    score: float
    fraction_of_neighbours_worse: float
    best_neighbour_gain: float  # (best neighbour score / current score) - 1; > 0 means a neighbour fits better


class EdgeAlignment:
    def __init__(self, frames: list[Frame], calibration: Calibration, across_rings: bool = True):
        self.calibration = calibration
        self.frames = []
        for frame in frames:
            points, weights = lidar_depth_edges(frame.scan, across_rings=across_rings)
            self.frames.append(EdgeFrame(points, weights, image_edge_field(frame.image), frame.image_size))

    def score(self, velo_to_cam: np.ndarray) -> float:
        matrix = self.calibration.lidar_to_image(velo_to_cam)
        total, weight_sum = 0.0, 0.0
        for frame in self.frames:
            projection = project(frame.points, matrix, frame.image_size)
            u = projection.pixels[projection.visible, 0].astype(int)
            v = projection.pixels[projection.visible, 1].astype(int)
            total += float(np.sum(frame.weights[projection.visible] * frame.field[v, u]))
            weight_sum += float(np.sum(frame.weights[projection.visible]))  # mean over points in view
        return total / max(weight_sum, 1e-9)

    def health_check(self, velo_to_cam: np.ndarray, rotation_step_deg: float, translation_step_m: float) -> HealthCheck:
        """Compare the current extrinsic with its 3x3x3 rotation and 3x3x3 translation neighbours."""
        current = self.score(velo_to_cam)
        neighbour_scores = []
        steps = (-1, 0, 1)
        for roll, pitch, yaw in itertools.product(steps, repeat=3):
            if (roll, pitch, yaw) != (0, 0, 0):
                drift = Drift(roll=roll * rotation_step_deg, pitch=pitch * rotation_step_deg, yaw=yaw * rotation_step_deg)
                neighbour_scores.append(self.score(drift.applied_to(velo_to_cam)))
        for tx, ty, tz in itertools.product(steps, repeat=3):
            if (tx, ty, tz) != (0, 0, 0):
                drift = Drift(tx=tx * translation_step_m, ty=ty * translation_step_m, tz=tz * translation_step_m)
                neighbour_scores.append(self.score(drift.applied_to(velo_to_cam)))
        neighbour_scores = np.array(neighbour_scores)
        return HealthCheck(
            score=current,
            fraction_of_neighbours_worse=float(np.mean(neighbour_scores < current)),
            best_neighbour_gain=float(neighbour_scores.max() / current - 1),
        )

    def refine_rotation(self, velo_to_cam: np.ndarray, search_deg: float = 3.0, rounds: int = 2) -> np.ndarray:
        """Coordinate search over yaw, pitch, roll: a full 0.1 deg grid over +-search_deg, then 0.02 deg.

        A full grid per axis (not hill climbing) steps over the false yaw peaks near +-1.5 deg.
        """
        estimate = velo_to_cam.copy()
        for step, half_width in ((0.1, search_deg), (0.02, 0.3)):
            offsets = np.arange(-half_width, half_width + step / 2, step)
            for _ in range(rounds):
                for axis in ("yaw", "pitch", "roll"):
                    candidates = [Drift.along(axis, offset).applied_to(estimate) for offset in offsets]
                    estimate = candidates[int(np.argmax([self.score(candidate) for candidate in candidates]))]
        return estimate

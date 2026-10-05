"""What an extrinsic drift does to the things a fusion stack consumes.

All metrics compare a drifted extrinsic against the KITTI calibration, which is the ground truth here.

reprojection error   pixel distance between where a LiDAR point lands with the true and the drifted
                     extrinsic, grouped by depth (rotation shifts every depth equally, translation
                     shifts near points the most).
association          share of an object's own LiDAR points that still land inside its 2D box. The 2D
                     box is the true projection of the tracklet's 3D box, i.e. a perfect 2D detector.
fused range          the late-fusion recipe "camera box + median LiDAR depth inside the box centre",
                     compared with the object's true depth. A miss is > max(1 m, 10 %) or no points.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import bounding_box, inside_box, project, shrink_box
from .kitti import Calibration, Frame

DEPTH_BANDS_M = ((0, 10), (10, 20), (20, 40), (40, 80))
DISTANCE_GROUPS_M = (("near < 20 m", 0, 20), ("far >= 20 m", 20, 200))


@dataclass
class ObjectView:
    frame_position: int  # index into the frame list
    label: str
    own_points: np.ndarray  # (K, 3) LiDAR points inside the 3D box
    box: tuple[float, float, float, float]  # true 2D box (u0, v0, u1, v1)
    true_depth: float  # median camera depth of the object's own points
    distance: float  # metres from the LiDAR to the box centre


def collect_objects(frames: list[Frame], calibration: Calibration, min_points: int = 20, min_box_px: float = 12) -> list[ObjectView]:
    """Clearly visible tracklet objects with enough LiDAR returns to judge association."""
    matrix = calibration.lidar_to_image()
    views = []
    for position, frame in enumerate(frames):
        for obj in frame.objects:
            if not obj.is_clearly_visible():
                continue
            corners = project(obj.corners(), matrix, frame.image_size, min_depth=0.5)
            if not np.all(corners.depth > 0.5):
                continue
            box = bounding_box(corners.pixels, frame.image_size)
            if box[2] - box[0] < min_box_px or box[3] - box[1] < min_box_px:
                continue
            points = frame.front_points
            own = points[obj.contains(points)]
            own_projection = project(own, matrix, frame.image_size)
            if own_projection.visible.sum() < min_points:
                continue
            views.append(ObjectView(
                frame_position=position, label=obj.label, own_points=own[own_projection.visible], box=box,
                true_depth=float(np.median(own_projection.depth[own_projection.visible])),
                distance=float(np.linalg.norm(obj.bottom_centre[:2])),
            ))
    return views


def reprojection_error_by_depth(frames: list[Frame], calibration: Calibration, drifted: np.ndarray, every_nth_point: int = 5) -> dict[str, float]:
    """Median pixel shift per depth band, plus overall median and 95th percentile."""
    true_matrix, drifted_matrix = calibration.lidar_to_image(), calibration.lidar_to_image(drifted)
    shifts, depths = [], []
    for frame in frames:
        points = frame.front_points[::every_nth_point]
        truth = project(points, true_matrix, frame.image_size)
        moved = project(points, drifted_matrix, frame.image_size)
        both = truth.visible & moved.visible
        shifts.append(np.linalg.norm(moved.pixels[both] - truth.pixels[both], axis=1))
        depths.append(truth.depth[both])
    shifts, depths = np.concatenate(shifts), np.concatenate(depths)
    result = {"reproj_median_px": float(np.median(shifts)), "reproj_p95_px": float(np.percentile(shifts, 95))}
    for low, high in DEPTH_BANDS_M:
        band = (depths >= low) & (depths < high)
        result[f"reproj_median_px_{low}-{high}m"] = float(np.median(shifts[band])) if band.any() else float("nan")
    return result


def association_and_range(frames: list[Frame], objects: list[ObjectView], calibration: Calibration, drifted: np.ndarray,
                          centre_keep: float = 0.5) -> dict[str, float]:
    matrix = calibration.lidar_to_image(drifted)
    retention, range_miss, distance = [], [], []
    projected_frames: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    for view in objects:
        frame = frames[view.frame_position]
        own = project(view.own_points, matrix, frame.image_size)
        retention.append(float(np.mean(own.visible & inside_box(own.pixels, view.box))))

        if view.frame_position not in projected_frames:
            everything = project(frame.front_points, matrix, frame.image_size)
            projected_frames[view.frame_position] = (everything.pixels[everything.visible], everything.depth[everything.visible])
        pixels, depth = projected_frames[view.frame_position]
        in_centre = inside_box(pixels, shrink_box(view.box, centre_keep))
        if in_centre.sum() == 0:
            range_miss.append(True)
        else:
            error = abs(float(np.median(depth[in_centre])) - view.true_depth)
            range_miss.append(error > max(1.0, 0.1 * view.true_depth))
        distance.append(view.distance)

    retention, range_miss, distance = np.array(retention), np.array(range_miss), np.array(distance)
    result = {
        "association_retention_pct": 100 * float(retention.mean()),
        "objects_lost_pct": 100 * float(np.mean(retention < 0.5)),
        "range_miss_pct": 100 * float(range_miss.mean()),
    }
    for name, low, high in DISTANCE_GROUPS_M:
        group = (distance >= low) & (distance < high)
        key = name.split()[0]
        result[f"association_retention_pct_{key}"] = 100 * float(retention[group].mean()) if group.any() else float("nan")
        result[f"range_miss_pct_{key}"] = 100 * float(range_miss[group].mean()) if group.any() else float("nan")
    return result

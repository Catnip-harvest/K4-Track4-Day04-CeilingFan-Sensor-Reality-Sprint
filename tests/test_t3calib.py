"""Small synthetic regression tests for geometry and fusion metrics."""

import numpy as np

from t3calib.geometry import Drift, drift_between, project
from t3calib.kitti import Calibration, Frame
from t3calib.metrics import ObjectView, association_and_range, reprojection_error_by_depth


def test_drift_composition_and_inverse_recovers_extrinsic():
    truth = np.eye(4)
    truth[:3, :3] = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    truth[:3, 3] = [0.4, -0.2, 1.1]
    drift = Drift(roll=0.7, pitch=-1.1, yaw=1.4, tx=0.03, ty=-0.02, tz=0.01)

    moved = drift.applied_to(truth)
    recovered = moved @ np.linalg.inv(drift.matrix())

    assert np.allclose(recovered, truth, atol=1e-12)
    assert drift_between(truth, moved)[0] > 0


def test_project_matches_hand_computed_homogeneous_projection():
    points = np.array([[1.0, 2.0, 4.0]])
    lidar_to_image = np.array([[2.0, 0.0, 10.0, 3.0], [0.0, 3.0, 20.0, -2.0], [0.0, 0.0, 1.0, 0.0]])

    result = project(points, lidar_to_image, (100, 100))

    # Homogeneous image coordinate is [45, 84, 4], hence pixel [11.25, 21].
    assert np.allclose(result.pixels[0], [11.25, 21.0])
    assert result.depth[0] == 4.0
    assert result.visible[0]


def test_zero_drift_has_zero_reprojection_error_and_perfect_fusion_metrics():
    calibration = Calibration(
        velo_to_cam=np.eye(4),
        rectification=np.eye(4),
        projection=np.array([[10.0, 0.0, 50.0, 0.0], [0.0, 10.0, 50.0, 0.0], [0.0, 0.0, 1.0, 0.0]]),
    )
    own_points = np.array([[x, y, 10.0] for x in np.linspace(-0.5, 0.5, 5) for y in np.linspace(-0.5, 0.5, 5)])
    # The metric's frame-level point filter keeps x > 0.5, matching KITTI's forward-view convention.
    scan = np.array([[0.6, y, 10.0, 0.5] for y in np.linspace(-0.5, 0.5, 25)])
    frame = Frame("synthetic", 0, scan, np.zeros((100, 100, 3), dtype=np.uint8))
    frames = [frame]
    view = ObjectView(
        frame_position=0,
        label="Car",
        own_points=own_points,
        box=(40.0, 40.0, 60.0, 60.0),
        true_depth=10.0,
        distance=10.0,
    )
    truth = calibration.velo_to_cam

    projection_metrics = reprojection_error_by_depth(frames, calibration, truth, every_nth_point=1)
    fusion_metrics = association_and_range(frames, [view], calibration, truth)

    assert projection_metrics["reproj_median_px"] == 0.0
    assert projection_metrics["reproj_p95_px"] == 0.0
    assert fusion_metrics["association_retention_pct"] == 100.0
    assert fusion_metrics["objects_lost_pct"] == 0.0
    assert fusion_metrics["range_miss_pct"] == 0.0

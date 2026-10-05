"""Readers for KITTI raw sync drives: calibration, Velodyne scans, left colour images and tracklet boxes.

Expected layout (what get_data.py extracts):
    data/2011_09_26/calib_cam_to_cam.txt
    data/2011_09_26/calib_velo_to_cam.txt
    data/2011_09_26/2011_09_26_drive_0005_sync/image_02/data/0000000000.png
    data/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/data/0000000000.bin
    data/2011_09_26/2011_09_26_drive_0005_sync/tracklet_labels.xml
"""

from __future__ import annotations

import xml.etree.ElementTree as ElementTree
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image

# Tracklet truncation codes from the KITTI raw devkit.
FULLY_IN_IMAGE = 0
# Tracklet occlusion codes: 0 fully visible, 1 partly occluded, 2 largely occluded.
PARTLY_OCCLUDED = 1


def read_calib_file(path: Path) -> dict[str, np.ndarray]:
    values: dict[str, np.ndarray] = {}
    for line in Path(path).read_text().splitlines():
        key, _, numbers = line.partition(":")
        try:
            values[key.strip()] = np.array([float(number) for number in numbers.split()])
        except ValueError:
            continue  # the calib_time line is a date, not numbers
    return values


@dataclass
class Calibration:
    """Camera-LiDAR geometry for the left colour camera (cam 2)."""

    velo_to_cam: np.ndarray  # 4x4 rigid transform: LiDAR frame -> camera 0 frame (the extrinsic we perturb)
    rectification: np.ndarray  # 4x4, R_rect_00
    projection: np.ndarray  # 3x4, P_rect_02

    def lidar_to_image(self, velo_to_cam: np.ndarray | None = None) -> np.ndarray:
        """3x4 matrix taking homogeneous LiDAR points to homogeneous pixels."""
        extrinsic = self.velo_to_cam if velo_to_cam is None else velo_to_cam
        return self.projection @ self.rectification @ extrinsic

    @property
    def focal_length_px(self) -> float:
        return float(self.projection[0, 0])


def load_calibration(date_dir: Path) -> Calibration:
    velo = read_calib_file(date_dir / "calib_velo_to_cam.txt")
    cams = read_calib_file(date_dir / "calib_cam_to_cam.txt")
    velo_to_cam = np.eye(4)
    velo_to_cam[:3, :3] = velo["R"].reshape(3, 3)
    velo_to_cam[:3, 3] = velo["T"]
    rectification = np.eye(4)
    rectification[:3, :3] = cams["R_rect_00"].reshape(3, 3)
    return Calibration(velo_to_cam, rectification, cams["P_rect_02"].reshape(3, 4))


@dataclass
class TrackedObject:
    """One labelled 3D box in one frame, in the LiDAR frame (x forward, y left, z up)."""

    label: str
    height: float
    width: float
    length: float
    bottom_centre: np.ndarray  # (3,) box centre at ground level
    yaw: float  # rotation about the LiDAR z axis, radians
    truncation: int
    occlusion: int

    def corners(self) -> np.ndarray:
        """(8, 3) box corners in the LiDAR frame."""
        half_l, half_w = self.length / 2, self.width / 2
        local = np.array([
            [-half_l, -half_l, half_l, half_l, -half_l, -half_l, half_l, half_l],
            [half_w, -half_w, -half_w, half_w, half_w, -half_w, -half_w, half_w],
            [0, 0, 0, 0, self.height, self.height, self.height, self.height],
        ])
        return (self._rotation() @ local).T + self.bottom_centre

    def contains(self, points: np.ndarray, ground_clearance: float = 0.15) -> np.ndarray:
        """Mask of points inside the box, ignoring the bottom few cm so road points are not counted."""
        local = (points[:, :3] - self.bottom_centre) @ self._rotation()
        return (
            (np.abs(local[:, 0]) <= self.length / 2)
            & (np.abs(local[:, 1]) <= self.width / 2)
            & (local[:, 2] >= ground_clearance)
            & (local[:, 2] <= self.height)
        )

    def is_clearly_visible(self) -> bool:
        return self.truncation == FULLY_IN_IMAGE and self.occlusion <= PARTLY_OCCLUDED

    def _rotation(self) -> np.ndarray:
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def load_tracklets(path: Path) -> dict[int, list[TrackedObject]]:
    objects_by_frame: dict[int, list[TrackedObject]] = defaultdict(list)
    if not path.exists():
        return objects_by_frame
    tracklets = ElementTree.parse(path).getroot().find("tracklets")
    for tracklet in tracklets.findall("item"):
        label = tracklet.findtext("objectType")
        height, width, length = (float(tracklet.findtext(key)) for key in ("h", "w", "l"))
        first_frame = int(tracklet.findtext("first_frame"))
        for offset, pose in enumerate(tracklet.find("poses").findall("item")):
            objects_by_frame[first_frame + offset].append(TrackedObject(
                label=label, height=height, width=width, length=length,
                bottom_centre=np.array([float(pose.findtext(axis)) for axis in ("tx", "ty", "tz")]),
                yaw=float(pose.findtext("rz")),
                truncation=int(pose.findtext("truncation")),
                occlusion=int(pose.findtext("occlusion")),
            ))
    return objects_by_frame


@dataclass
class Frame:
    drive: str
    index: int
    scan: np.ndarray  # (N, 4) full 360-degree scan in sensor order: x, y, z, reflectance
    image: np.ndarray  # (H, W, 3) uint8, left colour camera
    objects: list[TrackedObject] = field(default_factory=list)

    @property
    def front_points(self) -> np.ndarray:
        """Points ahead of the car; only these can land in the forward camera."""
        return self.scan[self.scan[:, 0] > 0.5, :3]

    @property
    def image_size(self) -> tuple[int, int]:
        height, width = self.image.shape[:2]
        return width, height


class Drive:
    def __init__(self, data_dir: Path, drive: str, date: str = "2011_09_26"):
        self.name = drive
        self.date_dir = Path(data_dir) / date
        self.sync_dir = self.date_dir / f"{date}_drive_{drive}_sync"
        if not self.sync_dir.exists():
            raise FileNotFoundError(f"{self.sync_dir} not found - run: python get_data.py --drives {drive}")
        self.calibration = load_calibration(self.date_dir)
        self.objects_by_frame = load_tracklets(self.sync_dir / "tracklet_labels.xml")
        self.frame_indices = sorted(int(path.stem) for path in (self.sync_dir / "velodyne_points" / "data").glob("*.bin"))

    def load_frame(self, index: int) -> Frame:
        scan = np.fromfile(self.sync_dir / "velodyne_points" / "data" / f"{index:010d}.bin", dtype=np.float32).reshape(-1, 4)
        image = np.asarray(Image.open(self.sync_dir / "image_02" / "data" / f"{index:010d}.png").convert("RGB"))
        return Frame(self.name, index, scan.astype(np.float64), image, list(self.objects_by_frame.get(index, [])))

    def load_frames(self, step: int = 1) -> list[Frame]:
        return [self.load_frame(index) for index in self.frame_indices[::step]]

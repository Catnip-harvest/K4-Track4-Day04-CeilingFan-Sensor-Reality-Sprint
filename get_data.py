"""Download the KITTI raw files this benchmark needs, using many parallel HTTP range requests.

One connection to the KITTI S3 bucket runs at only ~50-80 KB/s from Vietnam, so each zip is fetched
as many 4 MB byte ranges at once and written into place. Only the folders the benchmark reads
(left colour camera, Velodyne scans, timestamps, tracklets, calibration) are extracted.

Usage:
    python get_data.py                 # drives 0048 and 0005 into ./data
    python get_data.py --drives 0048   # only the small drive (84 MB)
"""

from __future__ import annotations

import argparse
import concurrent.futures
import time
import urllib.request
import zipfile
from pathlib import Path

KITTI_RAW = "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data"
DATE = "2011_09_26"
CHUNK_BYTES = 4 * 1024 * 1024
NEEDED_FOLDERS = ("image_02/", "velodyne_points/", "tracklet_labels.xml", "calib_")


def remote_size(url: str) -> int:
    request = urllib.request.Request(url, method="HEAD")
    with urllib.request.urlopen(request, timeout=60) as response:
        return int(response.headers["Content-Length"])


def fetch_range(url: str, destination: Path, start: int, end: int, attempts: int = 5) -> int:
    for attempt in range(1, attempts + 1):
        try:
            request = urllib.request.Request(url, headers={"Range": f"bytes={start}-{end}"})
            with urllib.request.urlopen(request, timeout=120) as response:
                payload = response.read()
            if len(payload) != end - start + 1:
                raise IOError(f"short read {len(payload)} of {end - start + 1}")
            with open(destination, "r+b") as handle:
                handle.seek(start)
                handle.write(payload)
            return len(payload)
        except Exception as error:  # network hiccups: retry the same range
            if attempt == attempts:
                raise
            print(f"  retry {attempt} for bytes {start}-{end}: {error}")
            time.sleep(2 * attempt)
    return 0


def download(url: str, destination: Path, connections: int) -> None:
    size = remote_size(url)
    done_marker = destination.with_suffix(destination.suffix + ".done")
    if destination.exists() and done_marker.exists() and destination.stat().st_size == size:
        print(f"{destination.name}: already downloaded ({size / 1e6:.1f} MB)")
        return

    with open(destination, "wb") as handle:
        handle.truncate(size)
    ranges = [(start, min(start + CHUNK_BYTES, size) - 1) for start in range(0, size, CHUNK_BYTES)]
    print(f"{destination.name}: {size / 1e6:.1f} MB in {len(ranges)} chunks, {connections} connections")

    started = time.time()
    received = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=connections) as pool:
        jobs = [pool.submit(fetch_range, url, destination, start, end) for start, end in ranges]
        for job in concurrent.futures.as_completed(jobs):
            received += job.result()
            elapsed = time.time() - started
            print(f"\r  {received / 1e6:7.1f} / {size / 1e6:.1f} MB  "
                  f"{received / 1e6 / max(elapsed, 1e-6):5.2f} MB/s", end="", flush=True)
    print()
    done_marker.touch()


def extract_needed(archive: Path, data_dir: Path) -> None:
    with zipfile.ZipFile(archive) as bundle:
        wanted = [name for name in bundle.namelist() if any(key in name for key in NEEDED_FOLDERS)]
        bundle.extractall(data_dir, members=wanted)
    print(f"  extracted {len(wanted)} files from {archive.name}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--drives", nargs="+", default=["0048", "0005"])
    parser.add_argument("--data-dir", type=Path, default=Path(__file__).parent / "data")
    parser.add_argument("--connections", type=int, default=24)
    args = parser.parse_args()
    args.data_dir.mkdir(parents=True, exist_ok=True)

    files = [f"{DATE}_calib.zip"]
    for drive in args.drives:
        folder = f"{DATE}_drive_{drive}"
        files += [f"{folder}/{folder}_tracklets.zip", f"{folder}/{folder}_sync.zip"]

    for relative in files:
        archive = args.data_dir / Path(relative).name
        download(f"{KITTI_RAW}/{relative}", archive, args.connections)
        extract_needed(archive, args.data_dir)


if __name__ == "__main__":
    main()

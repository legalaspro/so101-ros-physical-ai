# Copyright 2025 Dmitri Manajev
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Rosbag reader utilities and episode discovery.

- Opens a rosbag2 SequentialReader (storage backend inferred from metadata.yaml if possible)
- Topic introspection
- Episode discovery
- Message timestamp helpers
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Dict, List, Optional

import rosbag2_py
import yaml

logger = logging.getLogger(__name__)


def _read_storage_id(bag_dir: Path) -> str:
    """Best-effort storage backend inference from metadata.yaml."""
    meta_path = bag_dir / "metadata.yaml"
    if not meta_path.exists():
        return "mcap"
    try:
        meta = yaml.safe_load(meta_path.read_text()) or {}
        info = meta.get("rosbag2_bagfile_information", {})
        return info.get("storage_identifier", "mcap")
    except Exception:
        return "mcap"


def open_reader(bag_dir: Path) -> rosbag2_py.SequentialReader:
    """Open a rosbag directory with the mcap storage backend

    Args:
        bag_dir (Path):
            Path to a rosbag directory containing ``metadata.yaml``
            and one or more ``.mcap`` files.

    Returns:
        rosbag2_py.SequentialReader: An opened sequential reader ready for iteration.
    """
    storage_id = _read_storage_id(bag_dir)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def get_topic_types(reader: rosbag2_py.SequentialReader) -> Dict[str, str]:
    """Return a mapping of topic name → ROS2 message type from bag metadata.

    Example return: ``{"/camera/image_raw": "sensor_msgs/msg/Image"}``.
    """
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def find_episode_dirs(root: Path) -> List[Path]:
    """Discover episode directories containing valid rosbag data"""
    root = Path(root)

    dirs = sorted(
        d for d in root.iterdir() if d.is_dir() and (d / "metadata.yaml").exists()
    )

    if not dirs:
        logger.warning("No episode directories found in %s", root)
    return dirs


def msg_time_ns(msg: Any, stamp_src: str, bag_ns: int) -> int:
    """Return the authoritative timestamp for a message in nanoseconds.

    Args:
        msg (Any):  Deserialized ROS2 message.
        stamp_src (str):
            `"header"`` to prefer ``header.stamp``, ``"bag"`` to always use the
            bag-level receive timestamp.
        bag_ns (int): The bag-level timestamp in nanoseconds (from ``read_next()``).

    Returns:
        int: The bag-level timestamp in nanoseconds (from ``read_next()``).
    """
    if stamp_src == "header":
        ts = header_stamp_to_ns(msg)
        if ts is not None:
            return ts
        # Fall back to bag timestamp if header is missing
        logger.debug(
            "Message type %s has no header.stamp; falling back to bag timestamp",
            type(msg).__name__,
        )
    return bag_ns


def header_stamp_to_ns(msg: Any) -> Optional[int]:
    """Extract ``header.stamp`` from a ROS2 message as nanoseconds.

    Returns ``None`` if the message has no ``header`` or ``header.stamp``.
    """
    header = getattr(msg, "header", None)
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

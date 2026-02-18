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
Command-line interface for rosbat-to-lerobot conversion.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

from rosbag_to_lerobot.config import load_config
from rosbag_to_lerobot.converter import convert_all_bags

logger = logging.getLogger(__name__)


def main() -> None:

    parser = argparse.ArgumentParser(
        prog="ros2 run rosbag_to_lerobot convert",
        description="Convert ROS 2 rosbag episodes to LeRobot v3.0 datasets",
    )
    parser.add_argument(
        "--input-dir",
        required=True,
        type=Path,
        help="Directory containing episode bag subdirectories",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        type=Path,
        help="Output directory for the LeRobot dataset",
    )
    parser.add_argument(
        "--config", required=True, type=Path, help="Path to YAML config file."
    )
    parser.add_argument(
        "--repo-id",
        required=True,
        help="HuggingFace repo ID (e.g., user/dataset_name)",
    )
    parser.add_argument(
        "--use-videos",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Store images as MP4 video (default) vs individual images",
    )
    parser.add_argument(
        "--vcodec",
        default="libx264",
        help="Video codec for encoding (default: libx264)",
    )
    parser.add_argument(
        "--push-hub",
        action="store_true",
        default=False,
        help="Push final dataset to HuggingFace Hub",
    )

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    # Basic path validation
    if not args.input_dir.exists() or not args.input_dir.is_dir():
        logger.error(
            "--input-dir does not exist or is not a directory: %s", args.input_dir
        )
        sys.exit(2)
    if not args.config.exists() or not args.config.is_file():
        logger.error("--config does not exist or is not a file: %s", args.config)
        sys.exit(2)

    try:
        cfg = load_config(args.config)
        convert_all_bags(
            cfg=cfg,
            input_dir=Path(args.input_dir),
            output_dir=Path(args.output_dir),
            repo_id=args.repo_id,
            use_videos=args.use_videos,
            vcodec=args.vcodec,
            push_to_hub=args.push_hub,
        )
    except Exception as exc:
        logger.error("Conversion failed: %s", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()

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
YAML configuration loader and dataclasses for rosbag_to_lerobot
Decoder-driven design:
- msg_type selects the decoder (registered in rosbag_to_lerobot.decoders)
- YAML only carries mapping + alignment + optional decoding hints (names/shape)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml

_ALLOWED_STAMP_SRC = {"header", "bag"}


@dataclass(frozen=True)
class FeatureSpec:
    """Specification for a single feature to extract from the bag."""

    key: str
    topic: str
    msg_type: str  # e.g. "sensor_msgs/msg/Image"

    # Timestamp source used for alignment (converter)
    stamp_src: str = "bag"  # "header" | "bag"

    # Freshness window for as-of sampling; if None, use Config.default_max_age_s
    max_age_s: Optional[float] = None

    # Optional hints used by decoders and/or feature schema
    names: Optional[List[str]] = None  # ordering for JointState/arrays
    shape: Optional[List[int]] = None  # for images: [H, W, C] (HWC)


@dataclass(frozen=True)
class Config:
    """Top-level conversion configuration."""

    robot_type: str
    fps: int
    reference_topic: str
    task: str
    default_max_age_s: float = 0.2
    features: List[FeatureSpec] = field(default_factory=list)

    def by_topic(self) -> Dict[str, FeatureSpec]:
        return {f.topic: f for f in self.features}

    def by_key(self) -> Dict[str, FeatureSpec]:
        return {f.key: f for f in self.features}

    def reference_spec(self) -> FeatureSpec:
        for f in self.features:
            if f.topic == self.reference_topic:
                return f
        raise ValueError(
            f"reference_topic '{self.reference_topic}' is not listed in features"
        )

    def validate(self) -> None:
        if not self.robot_type:
            raise ValueError("robot_type must be set")
        if self.fps <= 0:
            raise ValueError(f"fps must be > 0 (got {self.fps})")
        if self.default_max_age_s <= 0:
            raise ValueError(
                f"default_max_age_s must be > 0 (got {self.default_max_age_s})"
            )
        if not self.features:
            raise ValueError("features list is empty")

        # Unique keys
        keys = [f.key for f in self.features]
        if len(keys) != len(set(keys)):
            raise ValueError("Duplicate feature.key entries found")

        # Reference topic must exist
        topics = [f.topic for f in self.features]
        if self.reference_topic not in topics:
            raise ValueError(
                f"reference_topic '{self.reference_topic}' is not listed in features"
            )

        for f in self.features:
            if f.stamp_src not in _ALLOWED_STAMP_SRC:
                raise ValueError(
                    f"Invalid stamp_src '{f.stamp_src}' for feature '{f.key}'"
                )
            if f.max_age_s is not None and f.max_age_s <= 0:
                raise ValueError(
                    f"max_age_s must be > 0 for '{f.key}' (got {f.max_age_s})"
                )


def load_config(path: str | Path) -> Config:
    """Load a YAML configuration file and return a validated :class:`Config`.

    Args:
        path (str | Path):  Filesystem path to the YAML config file.

    Returns:
        Config: Parsed configuration object.
    """
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}

    required = ("robot_type", "fps", "reference_topic", "task", "features")
    for k in required:
        if k not in raw:
            raise ValueError(f"Missing required field: {k}")

    features = [FeatureSpec(**feat) for feat in raw["features"]]

    cfg = Config(
        robot_type=raw["robot_type"],
        fps=raw["fps"],
        reference_topic=raw["reference_topic"],
        task=raw["task"],
        default_max_age_s=raw.get("default_max_age_s", 0.2),
        features=features,
    )
    cfg.validate()
    return cfg

# Copyright 2026 Dmitri Manajev
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
Core converter: single-pass reference-topic-driven rosbag -> LeRobot dataset.

Design:
- Read bag sequentially.
- Buffer latest values for all non-reference topics (LastBuffer).
- Every time a reference-topic message arrives -> emit 1 dataset frame:
    * decode reference message
    * sample other features as-of the reference timestamp with max_age_s
    * drop the frame if any required feature is missing/stale

Notes:
- msg_type selects the decoder (see rosbag_to_lerobot.decoders).
- Image features require `shape` in YAML to build LeRobot schema.
- Vector features should have `names` (preferred) or `shape`.
"""

from __future__ import annotations

import logging
import time
import shutil
from pathlib import Path
from typing import Any, Dict, Tuple, Optional

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from rosbag_to_lerobot.bag_reader import (
    find_episode_dirs,
    get_topic_types,
    msg_time_ns,
    open_reader,
)
from rosbag_to_lerobot.buffers import LastBuffer
from rosbag_to_lerobot.config import Config, FeatureSpec
from rosbag_to_lerobot.decoders import decode, get_lerobot_dtype

logger = logging.getLogger(__name__)

# -----------------------------------------------------------------------------
# Schema helpers
# -----------------------------------------------------------------------------

POS_KEYS = {"observation.state", "action"}


def _is_visual(spec: FeatureSpec) -> bool:
    return get_lerobot_dtype(spec.msg_type) in ("video", "image")


def _infer_vector_shape(spec: FeatureSpec) -> Tuple[int, ...]:
    if spec.shape is not None:
        return tuple(int(x) for x in spec.shape)
    if spec.names is not None:
        return (len(spec.names),)
    raise ValueError(
        f"Feature '{spec.key}' must define either 'shape' or 'names' "
        f"(topic={spec.topic}, msg_type={spec.msg_type})"
    )


def _default_lerobot_path(repo_id: str) -> Path:
    return Path.home() / ".cache" / "huggingface" / "lerobot" / repo_id


def _add_pos_suffix(names: list[str]) -> list[str]:
    return [n if n.endswith(".pos") else f"{n}.pos" for n in names]


def _fmt_sync_line(topic: str, s: dict) -> str:
    # seconds -> ms for readability
    mean_ms = (s.get("mean_dt_s") or 0.0) * 1e3
    p95 = s.get("p95_dt_s")
    if isinstance(p95, float) and p95 != p95:  # NaN check
        p95 = None
    p95_ms = (p95 * 1e3) if isinstance(p95, (float, int)) else None
    max_ms = (s.get("max_dt_s") or 0.0) * 1e3

    match_rate = (s.get("match_rate") or 0.0) * 100.0
    miss_empty = int(s.get("miss_empty") or 0)
    miss_future = int(s.get("miss_future") or 0)
    miss_stale = int(s.get("miss_stale") or 0)

    # trim noisy long topic names slightly
    t = topic if len(topic) <= 42 else "…" + topic[-41:]

    if p95_ms is None:
        return (
            f"{t}: match={match_rate:5.1f}%  "
            f"dt_mean={mean_ms:6.1f}ms dt_max={max_ms:6.1f}ms  "
            f"miss(e/f/s)={miss_empty}/{miss_future}/{miss_stale}"
        )
    return (
        f"{t}: match={match_rate:5.1f}%  "
        f"dt_mean={mean_ms:6.1f}ms dt_p95={p95_ms:6.1f}ms dt_max={max_ms:6.1f}ms  "
        f"miss(e/f/s)={miss_empty}/{miss_future}/{miss_stale}"
    )


def _build_lerobot_features(
    cfg: Config,
    use_videos: bool = True,
) -> Dict[str, Any]:
    """Build LeRobotDataset.create(features=...) schema from YAML config + decoder registry.

    Args:
        cfg (Config): Parsed conversion configuration
        use_videos (bool, optional):
            If *True* (default), image features use ``dtype="video"``;
            otherwise ``dtype="image"``. Defaults to True.

    Returns:
        Dict[str, Any]: Feature schema accepted by :func:`LeRobotDataset.create`.
    """
    features: Dict[str, Any] = {}

    for spec in cfg.features:
        if _is_visual(spec):
            if spec.shape is None:
                raise ValueError(
                    f"Visual feature '{spec.key}' must provide shape=[H,W,C] in YAML "
                    f"(topic={spec.topic})"
                )
            features[spec.key] = {
                "dtype": "video" if use_videos else "image",
                "shape": tuple(int(x) for x in spec.shape),  # HWC
                "names": ["height", "width", "channels"],
            }
        else:
            names = list(spec.names) if spec.names is not None else None
            if names is not None and spec.key in POS_KEYS:
                names = _add_pos_suffix(names)

            features[spec.key] = {
                "dtype": get_lerobot_dtype(spec.msg_type),  # e.g. float32
                "shape": _infer_vector_shape(spec),
                "names": names,
            }

    return features


# -----------------------------------------------------------------------------
# Conversion core
# -----------------------------------------------------------------------------


def _prepare_topic_maps(
    cfg: Config, topic_types: Dict[str, str]
) -> Tuple[Dict[str, FeatureSpec], Dict[str, type]]:
    """Build topic->spec and topic->msg_class maps, validating bag types."""
    topic_to_spec: Dict[str, FeatureSpec] = {s.topic: s for s in cfg.features}
    topic_to_msg_class: Dict[str, type] = {}

    # Validate that configured topics exist and types match
    for topic, spec in topic_to_spec.items():
        bag_type = topic_types.get(topic)
        if bag_type is None:
            raise ValueError(
                f"Configured topic not found in bag: {topic} (feature={spec.key})"
            )
        if bag_type != spec.msg_type:
            # strict by default; you can downgrade to warning if desired
            raise ValueError(
                f"Type mismatch for topic {topic}: config msg_type={spec.msg_type} "
                f"but bag has {bag_type}"
            )
        topic_to_msg_class[topic] = get_message(bag_type)  # return message class

    return topic_to_spec, topic_to_msg_class


def _convert_one_bag(
    bag_dir: Path,
    cfg: Config,
    dataset: LeRobotDataset,
    *,
    collect_p95: bool = False,
) -> tuple[int, int]:

    reader = open_reader(bag_dir)
    bag_topic_types = get_topic_types(reader)

    # Build topic->spec and topic->msg_class
    spec_by_topic, msg_cls_by_topic = _prepare_topic_maps(cfg, bag_topic_types)

    if cfg.reference_topic not in spec_by_topic:
        raise ValueError(
            f"reference_topic {cfg.reference_topic!r} not listed in config features"
        )

    ref_spec = spec_by_topic[cfg.reference_topic]

    # Buffers for non-reference topics, keyed by topic (fast lookup)
    buffers: Dict[str, LastBuffer] = {}
    for spec in cfg.features:
        if spec.topic == cfg.reference_topic:
            continue
        max_age = (
            spec.max_age_s if spec.max_age_s is not None else cfg.default_max_age_s
        )
        buffers[spec.topic] = LastBuffer(
            max_age_ns=int(max_age * 1e9),
            collect_p95=collect_p95,
        )

    frame_count = 0
    dropped_count = 0

    while reader.has_next():
        topic, data, bag_ts_ns = reader.read_next()
        spec = spec_by_topic.get(topic)
        if spec is None:
            continue  # skip topics not in config

        # TODO: may be move this code to the bag_reader or something
        msg_class = msg_cls_by_topic[topic]
        msg = deserialize_message(data, msg_class)
        ts_ns = msg_time_ns(msg, spec.stamp_src, bag_ts_ns)

        if topic == cfg.reference_topic:
            # --- Reference tick: emit one frame ---
            frame: Dict[str, Any] = {}

            frame[ref_spec.key] = decode(msg, ref_spec)
            frame["task"] = cfg.task
            # logger.info(
            #     "image delay (bag - header) = %.3f s", (bag_ts_ns - ts_ns) / 1e9
            # )

            drop = False
            # Sample all other features
            for other_spec in cfg.features:
                if other_spec.topic == cfg.reference_topic:
                    continue

                buf = buffers[other_spec.topic]
                value = buf.asof(ts_ns)

                # If missing values better drop state!
                if value is None:
                    drop = True
                    break

                frame[other_spec.key] = value

            if drop:
                dropped_count += 1
                continue

            dataset.add_frame(frame)
            frame_count += 1

        else:
            # --- Non-reference: decode and push to its buffer ---
            value = decode(msg, spec)
            buffers[topic].push(ts_ns, value)
            # logger.info(
            #     f"{spec.topic} delay (bag - header) = {(bag_ts_ns - ts_ns) / 1e9:.3f} s"
            # )

    logger.info(
        "Episode %s: %d frames, %d dropped (%.1f%%)",
        bag_dir.name,
        frame_count,
        dropped_count,
        (100.0 * dropped_count / max(1, frame_count + dropped_count)),
    )

    topic_stats = {topic: buf.summary() for topic, buf in buffers.items()}

    # Lightweight per-episode sync summary
    for topic in sorted(topic_stats.keys()):
        logger.info("  sync %s", _fmt_sync_line(topic, topic_stats[topic]))

    dataset.save_episode()
    return frame_count, dropped_count


# -----------------------------------------------------------------------------
# Public API
# -----------------------------------------------------------------------------


def convert_all_bags(
    cfg: Config,
    input_dir: Path,
    output_dir: Optional[Path],
    repo_id: str,
    use_videos: bool = True,
    vcodec: str = "libsvtav1",
    push_to_hub: bool = False,
    collect_p95: bool = False,
    overwrite: bool = False,
) -> None:
    """Orchestrate end-to-end conversion fo all episode bags

    Args:
        cfg (Config): Parsed conversion configuration
        input_dir (Path): Directory containing episode bag subdirectories.
        output_dir (Path): Destination for the LeRobot dataset (currently unused by
                        ``LeRobotDataset.create`` which writes to HF cache).
        repo_id (str):  Hugging Face repository ID for the dataset.
        use_videos (bool, optional):  If *True*, store images as video; otherwise as individual images. Defaults to True.
        vcodec (str, optional): Video codec for encoding. Defaults to "libsvtav1".
        push_to_hub (bool, optional):   If *True*, push final dataset to Hugging Face Hub. Defaults to False.
        collect_p95 (bool, optional): If *True*, collects additional data during episode sync.
        overwrite (bool, optional): If *True*, delete any existing dataset directory before writing
    """
    # 1. Discover episodes
    episodes = find_episode_dirs(input_dir)
    if not episodes:
        raise RuntimeError(f"No episode directories found in {input_dir}")

    # 2. Build features dict
    features = _build_lerobot_features(cfg, use_videos)

    target = output_dir if output_dir is not None else _default_lerobot_path(repo_id)

    if overwrite and target.exists():
        shutil.rmtree(target)

    # 3. Create dataset
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        root=output_dir,
        fps=cfg.fps,
        robot_type=cfg.robot_type,
        features=features,
        use_videos=use_videos,
        video_backend=vcodec,
    )

    # 4. Convert each episode
    total_frames = 0
    total_dropped = 0
    t_total0 = time.perf_counter()

    logger.info("Found %d episode(s) in %s", len(episodes), input_dir)
    for i, bag_dir in enumerate(episodes, start=1):
        t0 = time.perf_counter()
        logger.info("Converting episode %d/%d: %s", i, len(episodes), bag_dir.name)

        frames, dropped = _convert_one_bag(
            bag_dir, cfg, dataset, collect_p95=collect_p95
        )

        dt = time.perf_counter() - t0
        total_frames += frames
        total_dropped += dropped

        logger.info(
            "Finished %s in %.2fs (frames=%d, dropped=%d, drop=%.1f%%)",
            bag_dir.name,
            dt,
            frames,
            dropped,
            100.0 * dropped / max(1, frames + dropped),
        )

    t_total = time.perf_counter() - t_total0

    # 5. Finalize
    dataset.finalize()

    # 6. Optionally push to hub
    if push_to_hub:
        dataset.push_to_hub(
            tags=[
                "so101",
                "ros2",
                "teleoperation",
                "imitation-learning",
                "so101-ros-physical-ai",
            ],
            license="apache-2.0",
            url="https://github.com/legalaspro/so101-ros-physical-ai",
        )

    # 7. Summary
    logger.info(
        "Conversion complete: %d episodes, %d frames, %d dropped (total time %.2fs)",
        len(episodes),
        total_frames,
        total_dropped,
        t_total,
    )

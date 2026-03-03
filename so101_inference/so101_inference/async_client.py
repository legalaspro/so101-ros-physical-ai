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

"""Transport-agnostic async inference client.

Owns the action queue, aggregation logic, FPS tracking,
and telemetry counters.  Delegates all network I/O to a ``PolicyTransport``.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from queue import Queue
from typing import Optional

import numpy as np

from lerobot.async_inference.helpers import FPSTracker, RemotePolicyConfig, TimedAction, TimedObservation
from so101_inference.transport.base import PolicyTransport

log = logging.getLogger(__name__)

# Aggregate function registry — mirrors lerobot's async_inference/configs.py
AGGREGATE_FUNCTIONS = {
    "weighted_average": lambda old, new: 0.3 * old + 0.7 * new,
    "latest_only": lambda old, new: new,
    "average": lambda old, new: 0.5 * old + 0.5 * new,
    "conservative": lambda old, new: 0.7 * old + 0.3 * new,
}


@dataclass
class ClientCfg:
    """Configuration shared between the ROS2 node and the async client."""

    server_address: str
    policy_type: str
    repo_id: str
    policy_device: str
    client_device: str
    actions_per_chunk: int
    chunk_size_threshold: float
    fps: float
    max_age_s: float
    task: str
    aggregate_fn_name: str = "weighted_average"


class AsyncInferenceClient:
    """Transport-agnostic async inference pipeline.

    Manages the action queue, observation gating, aggregation,
    and telemetry.  All network I/O is delegated to the supplied
    ``PolicyTransport`` instance.

    Args:
        transport: A concrete ``PolicyTransport`` implementation.
        cfg: Client configuration.
        lerobot_features: Feature spec dict for the policy.
        logger: Optional logger override (e.g. a ROS2 logger adapter).
    """

    def __init__(
        self,
        transport: PolicyTransport,
        cfg: ClientCfg,
        lerobot_features: dict,
        logger: logging.Logger | None = None,
    ) -> None:
        self._transport = transport
        self.cfg = cfg
        self.lerobot_features = lerobot_features
        self._log = logger or log

        # --- threading primitives ---
        self.shutdown_event = threading.Event()
        self.latest_action_lock = threading.Lock()
        self.latest_action = -1
        self.action_chunk_size = max(1, cfg.actions_per_chunk)

        self.action_queue: Queue[TimedAction] = Queue()
        self.action_queue_lock = threading.Lock()

        # Pending observation queue: inference thread picks from here.
        self._pending_obs_queue: Queue[TimedObservation] = Queue(maxsize=1)

        self.fps_tracker = FPSTracker(target_fps=cfg.fps)

        # Resolve aggregate function
        fn_name = cfg.aggregate_fn_name
        if fn_name not in AGGREGATE_FUNCTIONS:
            raise ValueError(
                f"Unknown aggregate function '{fn_name}'. Available: {list(AGGREGATE_FUNCTIONS.keys())}"
            )
        self._aggregate_fn = AGGREGATE_FUNCTIONS[fn_name]

        # --- telemetry counters ---
        self._obs_sent_count = 0
        self._obs_dropped_count = 0
        self._actions_executed_count = 0
        self._chunks_received_count = 0
        self._control_loop_count = 0
        self._last_summary_time = time.time()
        self._summary_interval_s = 5.0

        self._inference_in_flight = False
        self._inference_thread: threading.Thread | None = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Connect transport, handshake, send policy config, start receiver."""
        if not self._transport.connect():
            raise RuntimeError("Transport connect() failed")

        if not self._transport.handshake():
            raise RuntimeError("Transport handshake() failed")

        policy_config = RemotePolicyConfig(
            self.cfg.policy_type,
            self.cfg.repo_id,
            self.lerobot_features,
            self.cfg.actions_per_chunk,
            self.cfg.policy_device,
        )
        if not self._transport.send_policy_config(policy_config):
            raise RuntimeError("Transport send_policy_config() failed")

        self._inference_thread = threading.Thread(
            target=self._inference_loop,
            daemon=True,
        )
        self._inference_thread.start()

    def stop(self) -> None:
        """Shut down receiver thread and close transport."""
        self._log.info("🛑 Shutting down AsyncInferenceClient...")
        self.log_periodic_summary()
        self.shutdown_event.set()
        if self._inference_thread is not None:
            self._inference_thread.join(timeout=1.0)
        self._transport.close()
        self._log.info("🛑 Shutdown complete")

    # ------------------------------------------------------------------
    # Observation sending
    # ------------------------------------------------------------------

    def submit_observation(self, raw_obs: dict) -> bool:
        """Build a ``TimedObservation`` and queue it for the inference thread.

        Returns ``True`` if the observation was queued successfully.
        """
        with self.latest_action_lock:
            latest = self.latest_action

        obs = TimedObservation(
            timestamp=time.time(),
            observation=raw_obs,
            timestep=max(latest, 0),
        )

        with self.action_queue_lock:
            queue_size = self.action_queue.qsize()

        self._log.info(f"📸 OBS #{obs.get_timestep()} BUILDING | latest_action={latest} | queue={queue_size}")

        # Drop old observation if queue is full (latest-wins)
        if self._pending_obs_queue.full():
            try:
                self._pending_obs_queue.get_nowait()
                self._obs_dropped_count += 1
            except Exception:
                pass
        self._pending_obs_queue.put_nowait(obs)

        # FPS tracking
        fps_metrics = self.fps_tracker.calculate_fps_metrics(obs.get_timestamp())
        if fps_metrics and self._obs_sent_count % 10 == 0:
            self._log.debug(f"  FPS metrics: {' '.join(f'{k}={v:.1f}' for k, v in fps_metrics.items())}")

        return True

    def ready_to_send(self) -> bool:
        """Return ``True`` when the action queue is low enough to request more."""
        if self._inference_in_flight:
            self._log.debug("⏳ Inference in-flight, skipping")
            return False
        # An observation is already queued for inference
        if not self._pending_obs_queue.empty():
            self._log.debug("⏳ Observation already queued for inference, skipping")
            return False
        with self.action_queue_lock:
            qsize = self.action_queue.qsize()
            ratio = qsize / self.action_chunk_size
            ready = ratio <= self.cfg.chunk_size_threshold
            if ready:
                self._log.debug(
                    f"🔍 Obs trigger: queue={qsize}/{self.action_chunk_size} "
                    f"({ratio:.2f}) ≤ threshold={self.cfg.chunk_size_threshold}"
                )
            return ready

    # ------------------------------------------------------------------
    # Inference thread
    # ------------------------------------------------------------------

    def _inference_loop(self) -> None:
        """Thread 2: wait for observation, call transport.infer(), aggregate actions."""
        self._log.info("🧵 Inference thread STARTED")
        while not self.shutdown_event.is_set():
            # Wait for an observation to be submitted
            try:
                obs = self._pending_obs_queue.get(timeout=0.5)
            except Exception:
                continue

            # Call transport.infer() — this blocks for the full round-trip
            self._inference_in_flight = True
            try:
                self._obs_sent_count += 1
                self._log.info(f"📤 OBS #{obs.get_timestep()} total_sent={self._obs_sent_count}")
                timed_actions = self._transport.infer(obs)
                if timed_actions:
                    self._aggregate_actions(timed_actions)
            except Exception:
                self._log.exception("Inference error")
                time.sleep(0.2)
            finally:
                self._inference_in_flight = False

    def _aggregate_actions(self, timed_actions: list[TimedAction]) -> None:
        """Merge incoming actions into the queue — single-pass, lerobot-style."""
        self._chunks_received_count += 1
        chunk_len = len(timed_actions)
        self.action_chunk_size = max(self.action_chunk_size, chunk_len)

        with self.latest_action_lock:
            latest = self.latest_action

        # Build lookup from the existing queue
        with self.action_queue_lock:
            old_by_ts = {a.get_timestep(): a.get_action() for a in self.action_queue.queue}
            old_size = len(old_by_ts)

        # Single pass over incoming actions
        new_queue: Queue[TimedAction] = Queue()
        for action in timed_actions:
            ts = action.get_timestep()
            if ts <= latest:
                continue
            if ts in old_by_ts:
                merged = self._aggregate_fn(old_by_ts[ts], action.get_action())
                new_queue.put(
                    TimedAction(
                        timestamp=action.get_timestamp(),
                        timestep=ts,
                        action=merged,
                    )
                )
            else:
                new_queue.put(action)

        with self.action_queue_lock:
            self.action_queue = new_queue

        self._log.info(
            f"🔀 CHUNK #{self._chunks_received_count} | "
            f"actions={chunk_len} | latest_executed={latest} | "
            f"queue: {old_size}→{new_queue.qsize()}"
        )

    # ------------------------------------------------------------------
    # Action consumption
    # ------------------------------------------------------------------

    def actions_available(self) -> bool:
        with self.action_queue_lock:
            return not self.action_queue.empty()

    def pop_action(self) -> Optional[np.ndarray]:
        """Pop the next action from the queue and return it as a flat float64 array.

        Returns ``None`` if the queue is empty.
        """
        with self.action_queue_lock:
            if self.action_queue.empty():
                return None
            pre_size = self.action_queue.qsize()
            timed_action = self.action_queue.get_nowait()
            post_size = self.action_queue.qsize()

        action_tensor = timed_action.get_action()
        action_np = action_tensor.detach().cpu().numpy().astype(np.float64).reshape(-1)

        with self.latest_action_lock:
            prev_latest = self.latest_action
            self.latest_action = timed_action.get_timestep()

        self._actions_executed_count += 1

        self._log.debug(
            f"🎯 ACTION #{timed_action.get_timestep()} EXECUTED | "
            f"prev_latest={prev_latest} | queue={pre_size}→{post_size} | "
            f"total_executed={self._actions_executed_count}"
        )
        self._log.debug(f"  action_values: [{' '.join(f'{v:+.3f}' for v in action_np)}]")

        return action_np

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def increment_control_loop(self) -> None:
        self._control_loop_count += 1

    def maybe_log_summary(self) -> None:
        """Log a periodic summary if enough time has elapsed."""
        now = time.time()
        if now - self._last_summary_time >= self._summary_interval_s:
            self.log_periodic_summary()
            self._last_summary_time = now

    def log_periodic_summary(self) -> None:
        with self.action_queue_lock:
            qsize = self.action_queue.qsize()
        with self.latest_action_lock:
            latest = self.latest_action

        elapsed = time.time() - (self.fps_tracker.first_timestamp or time.time())
        effective_fps = self._actions_executed_count / max(elapsed, 0.001)

        self._log.info("=" * 60)
        self._log.info(f"📊 PERIODIC SUMMARY (every {self._summary_interval_s:.0f}s)")
        self._log.info(f"  uptime:             {elapsed:.1f}s")
        self._log.info(f"  obs_sent:           {self._obs_sent_count}")
        self._log.info(f"  chunks_received:    {self._chunks_received_count}")
        self._log.info(f"  actions_executed:    {self._actions_executed_count}")
        self._log.info(f"  effective_fps:      {effective_fps:.1f} (target={self.cfg.fps:.1f})")
        self._log.info(f"  latest_action_ts:   {latest}")
        self._log.info(f"  queue_size:         {qsize}/{self.action_chunk_size}")
        self._log.info(f"  obs_dropped:        {self._obs_dropped_count} (replaced in queue)")
        self._log.info("=" * 60)

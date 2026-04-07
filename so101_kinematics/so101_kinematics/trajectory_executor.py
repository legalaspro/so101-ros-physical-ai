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

import time
import numpy as np


class TrajectoryExecutor:
    """
    Very small time-based executor for precomputed trajectories.

    Responsibilities:
    - own one active trajectory
    - sample q_cmd at current time
    - cancel cleanly

    It does NOT:
    - solve IK
    - plan trajectories
    - publish ROS messages
    """

    def __init__(self):
        self._ts: np.ndarray | None = None
        self._qs: np.ndarray | None = None
        self._t0: float | None = None

    def is_active(self) -> bool:
        return (
            self._ts is not None
            and self._qs is not None
            and self._t0 is not None
        )

    def start(self, ts: np.ndarray, qs: np.ndarray):
        ts = np.array(ts, dtype=float, copy=True)
        qs = np.array(qs, dtype=float, copy=True)

        if ts.ndim != 1:
            raise ValueError(f"ts must be 1D, got shape {ts.shape}")
        if qs.ndim != 2:
            raise ValueError(f"qs must be 2D, got shape {qs.shape}")
        if len(ts) != len(qs):
            raise ValueError(f"len(ts)={len(ts)} must match len(qs)={len(qs)}")
        if len(ts) == 0:
            raise ValueError("trajectory must contain at least one sample")
        if not np.isclose(ts[0], 0.0):
            raise ValueError(f"trajectory must start at t=0, got ts[0]={ts[0]}")
        if np.any(np.diff(ts) < 0.0):
            raise ValueError("ts must be nondecreasing")

        self._ts = ts
        self._qs = qs
        self._t0 = time.perf_counter()

    def cancel(self):
        self._ts = None
        self._qs = None
        self._t0 = None

    def duration(self) -> float:
        if self._ts is None:
            return 0.0
        return float(self._ts[-1])

    def elapsed(self) -> float:
        if self._t0 is None:
            return 0.0
        return max(0.0, time.perf_counter() - self._t0)

    @property
    def q_final(self) -> np.ndarray | None:
        """Final joint configuration, or None if no trajectory loaded."""
        if self._qs is None:
            return None
        return self._qs[-1].copy()

    def sample(self) -> tuple[np.ndarray, bool]:
        """Sample the trajectory at the current wall-clock time.

        Returns (q_cmd, reached_end).

        reached_end=True means elapsed >= duration — the final point is
        returned and will keep being returned on subsequent calls.
        The caller decides when to cancel() (e.g. after checking
        measured joint convergence).
        """
        if not self.is_active():
            raise RuntimeError("No active trajectory")

        assert self._ts is not None
        assert self._qs is not None
        assert self._t0 is not None

        t = self.elapsed()

        # Past the end: hold final point
        if t >= float(self._ts[-1]):
            return self._qs[-1].copy(), True

        # Find segment [idx, idx+1] and lerp
        idx = int(np.searchsorted(self._ts, t, side="right") - 1)
        idx = max(0, min(idx, len(self._ts) - 2))

        t0 = float(self._ts[idx])
        t1 = float(self._ts[idx + 1])

        if t1 <= t0:
            return self._qs[idx + 1].copy(), False

        alpha = (t - t0) / (t1 - t0)
        q = (1.0 - alpha) * self._qs[idx] + alpha * self._qs[idx + 1]
        return q, False

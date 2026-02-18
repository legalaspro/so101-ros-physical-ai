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

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np


@dataclass
class BufferItem:
    ts_ns: int
    value: np.ndarray


class LastBuffer:
    """
    Single-item buffer for as-of sampling (latest value <= reference time).

    Stats collected:
      - tried: how many times asof() was called
      - matched: how many times a value was returned
      - miss_empty: no sample has ever been pushed
      - miss_future: latest sample is in the future (should be rare in sequential bag read)
      - miss_stale: sample exists but older than max_age
      - dt_ns_sum / dt_ns_max: summary of |ref - sample| in ns
      - dt_ns_p95: computed from reservoir (optional, see below)
    """

    def __init__(self, *, max_age_ns: int) -> None:
        self.max_age_ns = int(max_age_ns)
        self.item: Optional[BufferItem] = None

        # Stats (instance attributes!)
        self.tried = 0
        self.total_read = 0
        self.matched = 0
        self.miss_empty = 0
        self.miss_future = 0
        self.miss_stale = 0

        self._dt_ns_sum = 0
        self._dt_ns_max = 0

        # Keep a small sample of dt's for percentiles without storing everything
        self._reservoir = np.empty((0,), dtype=np.int64)
        self._reservoir_cap = 2000  # good enough

    def push(self, ts_ns: int, value: np.ndarray) -> None:
        """Store value if it's newer than (or equal to) the current item."""
        ts_ns = int(ts_ns)
        if self.item is None or ts_ns >= self.item.ts_ns:
            self.total_read += 1
            self.item = BufferItem(ts_ns=ts_ns, value=value)

    def asof(self, ref_ts_ns: int) -> Optional[np.ndarray]:
        """
        Return stored value if:
          - exists
          - not in the future (item.ts <= ref_ts)
          - within freshness window (ref_ts - item.ts <= max_age_ns)
        """
        self.tried += 1
        if self.item is None:
            self.miss_empty += 1
            return None
        if self.item.ts_ns > ref_ts_ns:
            self.miss_future += 1
            return None

        dt = ref_ts_ns - self.item.ts_ns
        if dt > self.max_age_ns:
            self.miss_stale += 1
            return None
        # matched
        self.matched += 1
        self._dt_ns_sum += dt
        if dt > self._dt_ns_max:
            self._dt_ns_max = dt

        # reservoir for percentile
        if self._reservoir.size < self._reservoir_cap:
            self._reservoir = np.append(self._reservoir, dt)
        else:
            # simple downsample: replace random entry with small probability
            j = self.tried % self._reservoir_cap
            self._reservoir[j] = dt

        return self.item.value

    def summary(self) -> Dict[str, float]:
        """Return descriptive statistics (seconds)."""
        match_rate = self.matched / max(1, self.tried)
        mean_dt_s = float("nan")
        p95_dt_s = float("nan")
        max_dt_s = self._dt_ns_max / 1e9

        if self.matched > 0:
            mean_dt_s = (self._dt_ns_sum / self.matched) / 1e9

        if self._reservoir.size > 0:
            p95_dt_s = float(
                np.percentile(self._reservoir.astype(np.float64), 95) / 1e9
            )

        return {
            "max_age_s": self.max_age_ns / 1e9,
            "total_read": float(self.total_read),
            "tried": float(self.tried),
            "matched": float(self.matched),
            "match_rate": float(match_rate),
            "miss_empty": float(self.miss_empty),
            "miss_future": float(self.miss_future),
            "miss_stale": float(self.miss_stale),
            "mean_dt_s": float(mean_dt_s),
            "p95_dt_s": float(p95_dt_s),
            "max_dt_s": float(max_dt_s),
        }

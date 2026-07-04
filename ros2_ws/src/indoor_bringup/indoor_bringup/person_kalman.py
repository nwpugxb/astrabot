"""Simple 2D constant-velocity Kalman filters for person tracks."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass
class Track:
    track_id: int
    kf: "KalmanCV2D"
    hits: int = 1
    misses: int = 0
    last_stamp_s: float = 0.0


class KalmanCV2D:
    """Planar constant-velocity model: state [x, y, vx, vy]."""

    def __init__(
        self,
        x: float,
        y: float,
        dt: float = 0.1,
        process_noise: float = 0.8,
        meas_noise: float = 0.2,
    ) -> None:
        self.dt = dt
        self.x = np.array([x, y, 0.0, 0.0], dtype=np.float64)
        self.P = np.eye(4, dtype=np.float64) * 2.0
        self.F = np.array(
            [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]],
            dtype=np.float64,
        )
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float64)
        q = process_noise
        self.Q = np.array(
            [
                [q * dt**4 / 4, 0, q * dt**3 / 2, 0],
                [0, q * dt**4 / 4, 0, q * dt**3 / 2],
                [q * dt**3 / 2, 0, q * dt**2, 0],
                [0, q * dt**3 / 2, 0, q * dt**2],
            ],
            dtype=np.float64,
        )
        self.R = np.eye(2, dtype=np.float64) * (meas_noise**2)

    def predict(self) -> np.ndarray:
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:2].copy()

    def update(self, z: np.ndarray) -> None:
        y = z - self.H @ self.x
        s = self.H @ self.P @ self.H.T + self.R
        k = self.P @ self.H.T @ np.linalg.inv(s)
        self.x = self.x + k @ y
        self.P = (np.eye(4) - k @ self.H) @ self.P

    @property
    def position(self) -> Tuple[float, float]:
        return float(self.x[0]), float(self.x[1])

    @property
    def velocity(self) -> Tuple[float, float]:
        return float(self.x[2]), float(self.x[3])

    def predict_horizon(self, horizon_s: float) -> Tuple[float, float]:
        return (
            float(self.x[0] + self.x[2] * horizon_s),
            float(self.x[1] + self.x[3] * horizon_s),
        )


class PersonTrackerManager:
    """Greedy nearest-neighbour association + per-track Kalman filters."""

    def __init__(
        self,
        max_association_dist_m: float = 1.5,
        max_misses: int = 8,
        dt: float = 0.1,
        process_noise: float = 0.8,
        meas_noise: float = 0.2,
    ) -> None:
        self.max_dist = max_association_dist_m
        self.max_misses = max_misses
        self.dt = dt
        self.process_noise = process_noise
        self.meas_noise = meas_noise
        self._tracks: Dict[int, Track] = {}
        self._next_id = 1

    @property
    def tracks(self) -> Dict[int, Track]:
        return self._tracks

    def update(
        self,
        measurements: List[Tuple[float, float, float]],
        stamp_s: float,
    ) -> List[Track]:
        """measurements: list of (x, y, radius_m) in base frame."""
        for track in self._tracks.values():
            track.kf.predict()

        assigned: set[int] = set()
        used_meas: set[int] = set()
        pairs: List[Tuple[float, int, int]] = []
        track_ids = list(self._tracks.keys())
        for ti, tid in enumerate(track_ids):
            tx, ty = self._tracks[tid].kf.position
            for mi, (mx, my, _r) in enumerate(measurements):
                d = float(np.hypot(mx - tx, my - ty))
                if d <= self.max_dist:
                    pairs.append((d, ti, mi))
        pairs.sort(key=lambda p: p[0])

        for _d, ti, mi in pairs:
            if ti in assigned or mi in used_meas:
                continue
            tid = track_ids[ti]
            mx, my, _r = measurements[mi]
            track = self._tracks[tid]
            track.kf.update(np.array([mx, my], dtype=np.float64))
            track.hits += 1
            track.misses = 0
            track.last_stamp_s = stamp_s
            assigned.add(ti)
            used_meas.add(mi)

        for ti, tid in enumerate(track_ids):
            if ti not in assigned:
                self._tracks[tid].misses += 1

        for mi, (mx, my, _r) in enumerate(measurements):
            if mi in used_meas:
                continue
            kf = KalmanCV2D(
                mx,
                my,
                dt=self.dt,
                process_noise=self.process_noise,
                meas_noise=self.meas_noise,
            )
            self._tracks[self._next_id] = Track(
                track_id=self._next_id,
                kf=kf,
                last_stamp_s=stamp_s,
            )
            self._next_id += 1

        stale = [tid for tid, t in self._tracks.items() if t.misses > self.max_misses]
        for tid in stale:
            del self._tracks[tid]

        return list(self._tracks.values())

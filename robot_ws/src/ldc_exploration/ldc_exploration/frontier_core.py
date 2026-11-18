"""Pure frontier-exploration helpers.

This module deliberately has no ROS imports so the frontier search, filtering,
blacklist and map-quality decisions can be unit tested without starting nodes.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable, Sequence


FREE = 0
UNKNOWN = -1
OCCUPIED_MIN = 65


@dataclass(frozen=True)
class MapInfo:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.width and 0 <= row < self.height

    def index(self, col: int, row: int) -> int:
        return row * self.width + col

    def cell_center(self, col: int, row: int) -> tuple[float, float]:
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)


@dataclass(frozen=True)
class MapStats:
    total: int
    free: int
    occupied: int
    unknown: int

    @property
    def unknown_ratio(self) -> float:
        return self.unknown / self.total if self.total else 1.0


@dataclass(frozen=True)
class FrontierCluster:
    cells: tuple[tuple[int, int], ...]
    target: tuple[float, float]
    centroid: tuple[float, float]
    distance: float

    @property
    def size(self) -> int:
        return len(self.cells)


def map_stats(data: Sequence[int], info: MapInfo) -> MapStats:
    if len(data) != info.width * info.height:
        return MapStats(0, 0, 0, 0)
    free = unknown = occupied = 0
    for value in data:
        if value == FREE:
            free += 1
        elif value < 0:
            unknown += 1
        elif value >= OCCUPIED_MIN:
            occupied += 1
        else:
            # Unknown-valued occupancy probability is still not usable free space.
            unknown += 1
    return MapStats(info.width * info.height, free, occupied, unknown)


def map_is_explorable(stats: MapStats, min_free: int = 50,
                      max_unknown_ratio: float = 0.98) -> bool:
    """A blank/just-opened map must not trigger autonomous goals."""
    return stats.total > 0 and stats.free >= min_free and stats.unknown_ratio <= max_unknown_ratio


def _neighbour_offsets():
    return ((dr, dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc)


def _frontier_cells(data: Sequence[int], info: MapInfo) -> set[tuple[int, int]]:
    frontier: set[tuple[int, int]] = set()
    offsets = tuple(_neighbour_offsets())
    for row in range(info.height):
        base = row * info.width
        for col in range(info.width):
            if data[base + col] != FREE:
                continue
            for dr, dc in offsets:
                nr, nc = row + dr, col + dc
                if info.in_bounds(nc, nr) and data[info.index(nc, nr)] < 0:
                    frontier.add((col, row))
                    break
    return frontier


def _has_occupied_within(data: Sequence[int], info: MapInfo, col: int, row: int,
                         radius: float) -> bool:
    cells = max(1, int(math.ceil(radius / info.resolution)))
    for dr in range(-cells, cells + 1):
        nr = row + dr
        for dc in range(-cells, cells + 1):
            nc = col + dc
            if not info.in_bounds(nc, nr):
                continue
            value = data[info.index(nc, nr)]
            if value >= OCCUPIED_MIN:
                return True
    return False


def _has_unknown_within(data: Sequence[int], info: MapInfo, col: int, row: int,
                        radius: float) -> bool:
    cells = max(0, int(math.ceil(radius / info.resolution)))
    for dr in range(-cells, cells + 1):
        nr = row + dr
        for dc in range(-cells, cells + 1):
            nc = col + dc
            if not info.in_bounds(nc, nr):
                # Outside the map is unknown/collision space for goal selection.
                return True
            if data[info.index(nc, nr)] < 0:
                return True
    return False


def _safe_goal_cell(data: Sequence[int], info: MapInfo, centroid_cell: tuple[float, float],
                    robot_xy: tuple[float, float], goal_clearance: float,
                    robot_radius: float, search_radius: float) -> tuple[int, int] | None:
    """Pick a free cell near the cluster that is clear of unknown/occupied cells.

    Frontier cells themselves touch unknown space; sending Nav2 a goal exactly on
    the frontier puts the goal inside the costmap inflation and makes NavFn fail.
    Move the goal inward by ``goal_clearance`` instead.
    """
    cx, cy = centroid_cell
    radius_cells = max(1, int(math.ceil(search_radius / info.resolution)))
    candidates: list[tuple[float, int, int]] = []
    for row in range(max(0, int(cy) - radius_cells), min(info.height, int(cy) + radius_cells + 1)):
        for col in range(max(0, int(cx) - radius_cells), min(info.width, int(cx) + radius_cells + 1)):
            if data[info.index(col, row)] != FREE:
                continue
            if _has_occupied_within(data, info, col, row, max(goal_clearance, robot_radius)):
                continue
            if _has_unknown_within(data, info, col, row, goal_clearance):
                continue
            target = info.cell_center(col, row)
            score = math.hypot(target[0] - (info.origin_x + (cx + .5) * info.resolution),
                               target[1] - (info.origin_y + (cy + .5) * info.resolution))
            score += 0.05 * math.hypot(target[0] - robot_xy[0], target[1] - robot_xy[1])
            candidates.append((score, col, row))
    if not candidates:
        return None
    _, col, row = min(candidates)
    return col, row


def _cluster_cells(cells: Iterable[tuple[int, int]]) -> list[list[tuple[int, int]]]:
    remaining = set(cells)
    clusters: list[list[tuple[int, int]]] = []
    while remaining:
        seed = remaining.pop()
        cluster = [seed]
        queue = deque([seed])
        while queue:
            col, row = queue.popleft()
            for dr, dc in _neighbour_offsets():
                neighbour = (col + dc, row + dr)
                if neighbour in remaining:
                    remaining.remove(neighbour)
                    cluster.append(neighbour)
                    queue.append(neighbour)
        clusters.append(cluster)
    return clusters


def find_frontiers(data: Sequence[int], info: MapInfo, robot_xy: tuple[float, float],
                   min_frontier_cells: int = 12, min_robot_distance: float = 0.25,
                   min_goal_distance: float = 0.3, max_goal_distance: float = 8.0,
                   robot_radius: float = 0.15, goal_clearance: float = 0.35) -> list[FrontierCluster]:
    """Return valid frontier clusters sorted nearest-first.

    A frontier cell is a FREE cell with any 8-neighbour UNKNOWN cell. Clusters
    are 8-connected. Targets are free cells closest to each cluster centroid;
    the centroid itself may fall in unknown space.
    """
    if min_frontier_cells < 1 or info.width <= 0 or info.height <= 0:
        return []
    frontier = _frontier_cells(data, info)
    if not frontier:
        return []
    result: list[FrontierCluster] = []
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    for cluster in _cluster_cells(frontier):
        if len(cluster) < min_frontier_cells:
            continue
        cols = [cell[0] for cell in cluster]
        rows = [cell[1] for cell in cluster]
        centroid_cell = (sum(cols) / len(cluster), sum(rows) / len(cluster))
        target_cell = _safe_goal_cell(
            data, info, centroid_cell, (rx, ry), goal_clearance,
            robot_radius, max(1.0, goal_clearance * 3.0))
        if target_cell is None:
            continue
        target = info.cell_center(*target_cell)
        centroid = info.cell_center(round(centroid_cell[0]), round(centroid_cell[1]))
        distance = math.hypot(target[0] - rx, target[1] - ry)
        if distance < min_robot_distance:
            continue
        if distance < min_goal_distance or distance > max_goal_distance:
            continue
        if _has_occupied_within(data, info, target_cell[0], target_cell[1], robot_radius):
            continue
        result.append(FrontierCluster(tuple(sorted(cluster)), target, centroid, distance))
    return sorted(result, key=lambda item: item.distance)


@dataclass(frozen=True)
class BlacklistEntry:
    x: float
    y: float
    expires_at: float


class FrontierBlacklist:
    """Time-limited goal blacklist; all times are monotonic seconds."""

    def __init__(self, radius: float = 0.6):
        self.radius = float(radius)
        self._entries: list[BlacklistEntry] = []

    def prune(self, now: float) -> None:
        self._entries = [entry for entry in self._entries if entry.expires_at > now]

    def add(self, x: float, y: float, now: float, seconds: float = 60.0) -> None:
        self.prune(now)
        self._entries.append(BlacklistEntry(float(x), float(y), float(now) + max(0.0, seconds)))
        # Keep a bounded history even if callers never prune.
        if len(self._entries) > 200:
            self._entries = self._entries[-200:]

    def contains(self, x: float, y: float, now: float) -> bool:
        self.prune(now)
        return any(math.hypot(entry.x - float(x), entry.y - float(y)) <= self.radius
                   for entry in self._entries)

    def blocks(self, cluster: FrontierCluster, now: float) -> bool:
        return self.contains(cluster.target[0], cluster.target[1], now)

    def count(self, now: float) -> int:
        self.prune(now)
        return len(self._entries)


def select_frontier(clusters: Sequence[FrontierCluster], blacklist: FrontierBlacklist,
                    now: float) -> FrontierCluster | None:
    for cluster in clusters:
        if not blacklist.blocks(cluster, now):
            return cluster
    return None


def goal_yaw(robot_xy: tuple[float, float], target_xy: tuple[float, float]) -> float:
    dx, dy = target_xy[0] - robot_xy[0], target_xy[1] - robot_xy[1]
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    return math.atan2(dy, dx)


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    return (math.sin(yaw / 2.0), math.cos(yaw / 2.0))

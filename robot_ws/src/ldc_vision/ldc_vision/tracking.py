"""Pure-Python lightweight multi-object tracker for YOLO box stabilisation.

No ROS/OpenCV/NumPy dependency.  The tracker receives already validated
detections in original-image pixel coordinates and returns the subset that
should be published/drawn after temporal confirmation and smoothing.

Design:
- Greedy IoU matching (one-to-one), default threshold 0.30.
- A track is visible only after ``confirm_frames`` consecutive hits.
- A confirmed track survives ``max_missed - 1`` missed frames; at
  ``max_missed`` it is removed (so with max_missed=3 it survives 1-2 misses,
  disappears on the 3rd).
- Scores below ``min_score`` are never published/drawn, even if confirmed.
- Box and score are EMA-smoothed with ``ema_alpha`` (default 0.45).
- Class name is frozen for a track; a competing class may take over only
  after ``class_switch_hits`` consecutive detections with score at least
  ``class_switch_score`` (default 0.6).
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Optional, Sequence, Tuple

Box = Sequence[float]


def iou(box_a: Box, box_b: Box) -> float:
    """Intersection-over-union of two ``[x1, y1, x2, y2]`` boxes."""
    ax1, ay1, ax2, ay2 = (float(v) for v in box_a)
    bx1, by1, bx2, by2 = (float(v) for v in box_b)
    inter_w = min(ax2, bx2) - max(ax1, bx1)
    inter_h = min(ay2, by2) - max(ay1, by1)
    if inter_w <= 0.0 or inter_h <= 0.0:
        return 0.0
    inter = inter_w * inter_h
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0.0 else 0.0


class Track:
    __slots__ = (
        'track_id', 'xyxy', 'score', 'class_name', 'class_id',
        'hits', 'missed', 'candidate_class', 'candidate_hits', 'last_frame',
    )

    def __init__(self, track_id: int, detection: Dict, frame_index: int):
        self.track_id = track_id
        self.xyxy = [float(value) for value in detection['xyxy']]
        self.score = float(detection['score'])
        self.class_name = str(detection['name'])
        self.class_id = detection.get('class_id')
        self.hits = 1
        self.missed = 0
        self.candidate_class: Optional[str] = None
        self.candidate_hits = 0
        self.last_frame = frame_index

    def visible(self, confirm_frames: int, min_score: float) -> bool:
        return self.hits >= confirm_frames and self.score >= min_score

    def public(self) -> Dict:
        return {
            'track_id': self.track_id,
            'name': self.class_name,
            'class_id': self.class_id,
            'score': self.score,
            'xyxy': list(self.xyxy),
        }


class MultiObjectTracker:
    """Greedy IoU tracker with confirmation, smoothing and class freezing."""

    def __init__(
        self,
        iou_threshold: float = 0.3,
        confirm_frames: int = 2,
        max_missed: int = 3,
        min_score: float = 0.35,
        ema_alpha: float = 0.45,
        class_switch_score: float = 0.6,
        class_switch_hits: int = 2,
    ):
        if not 0.0 < iou_threshold <= 1.0:
            raise ValueError('iou_threshold 必须属于 (0, 1]')
        if int(confirm_frames) < 1:
            raise ValueError('confirm_frames 必须 >= 1')
        if int(max_missed) < 1:
            raise ValueError('max_missed 必须 >= 1')
        if not 0.0 < min_score <= 1.0:
            raise ValueError('min_score 必须属于 (0, 1]')
        if not 0.0 < ema_alpha <= 1.0:
            raise ValueError('ema_alpha 必须属于 (0, 1]')
        if not 0.0 < class_switch_score <= 1.0:
            raise ValueError('class_switch_score 必须属于 (0, 1]')
        if int(class_switch_hits) < 1:
            raise ValueError('class_switch_hits 必须 >= 1')
        self.iou_threshold = float(iou_threshold)
        self.confirm_frames = int(confirm_frames)
        self.max_missed = int(max_missed)
        self.min_score = float(min_score)
        self.ema_alpha = float(ema_alpha)
        self.class_switch_score = float(class_switch_score)
        self.class_switch_hits = int(class_switch_hits)
        self.reset()

    def reset(self) -> None:
        """Drop all tracks and counters; used on filter change/worker restart."""
        self._tracks: List[Track] = []
        self._next_id = 1

    def _smooth(self, track: Track, detection: Dict, frame_index: int) -> None:
        alpha = self.ema_alpha
        old_box = track.xyxy
        new_box = [float(value) for value in detection['xyxy']]
        track.xyxy = [
            alpha * new_box[index] + (1.0 - alpha) * old_box[index]
            for index in range(4)
        ]
        track.score = alpha * float(detection['score']) + (1.0 - alpha) * track.score
        track.hits += 1
        track.missed = 0
        track.last_frame = frame_index

        new_class = str(detection['name'])
        if new_class == track.class_name:
            track.candidate_class = None
            track.candidate_hits = 0
            if detection.get('class_id') is not None:
                track.class_id = detection.get('class_id')
            return

        # Class freeze: only a strong, repeated competing class may take over.
        if float(detection['score']) < self.class_switch_score:
            track.candidate_class = None
            track.candidate_hits = 0
            return
        if track.candidate_class == new_class:
            track.candidate_hits += 1
        else:
            track.candidate_class = new_class
            track.candidate_hits = 1
        if track.candidate_hits >= self.class_switch_hits:
            track.class_name = new_class
            track.class_id = detection.get('class_id')
            track.candidate_class = None
            track.candidate_hits = 0

    def _match(self, detections: List[Dict]) -> Tuple[Dict[int, int], set, set]:
        """Greedy one-to-one matching. Returns track_id -> detection index."""
        pairs = []
        for track in self._tracks:
            for index, detection in enumerate(detections):
                overlap = iou(track.xyxy, detection['xyxy'])
                if overlap >= self.iou_threshold:
                    pairs.append((overlap, track.track_id, index))
        pairs.sort(key=lambda item: (-item[0], item[1], item[2]))
        matches = {}
        used_tracks = set()
        used_detections = set()
        for _, track_id, detection_index in pairs:
            if track_id in used_tracks or detection_index in used_detections:
                continue
            matches[track_id] = detection_index
            used_tracks.add(track_id)
            used_detections.add(detection_index)
        return matches, used_tracks, used_detections

    def update(self, detections: Iterable[Dict], frame_index: int = 0) -> List[Dict]:
        """Update tracks and return the visible, stabilised detections.

        ``detections`` must already be validated/clipped original-pixel boxes
        with ``xyxy``, ``score`` and ``name`` keys (``class_id`` optional).
        """
        detections = [detection for detection in detections]
        matches, matched_tracks, matched_detections = self._match(detections)
        by_id = {track.track_id: track for track in self._tracks}

        for track_id, detection_index in matches.items():
            self._smooth(by_id[track_id], detections[detection_index], frame_index)

        remaining = []
        for track in self._tracks:
            if track.track_id in matched_tracks:
                remaining.append(track)
                continue
            track.missed += 1
            track.candidate_class = None
            track.candidate_hits = 0
            if track.missed >= self.max_missed:
                continue
            remaining.append(track)
        self._tracks = remaining

        for index, detection in enumerate(detections):
            if index in matched_detections:
                continue
            track = Track(self._next_id, detection, frame_index)
            self._next_id += 1
            self._tracks.append(track)

        visible = [
            track.public()
            for track in sorted(self._tracks, key=lambda item: item.track_id)
            if track.visible(self.confirm_frames, self.min_score)
        ]
        return visible

    def stats(self) -> Dict[str, int]:
        """Diagnostics: active confirmed, currently lost, and total tracks."""
        confirmed = sum(
            1 for track in self._tracks if track.hits >= self.confirm_frames
        )
        lost = sum(1 for track in self._tracks if track.missed > 0)
        return {'confirmed': confirmed, 'lost': lost, 'total': len(self._tracks)}

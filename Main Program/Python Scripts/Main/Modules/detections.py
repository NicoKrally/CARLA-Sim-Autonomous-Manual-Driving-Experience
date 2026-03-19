import math
from dataclasses import dataclass
from typing import Any, Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class TLClusterConfig:
    conf_min: float = 0.6
    y_roi_min_ratio: float = 0.10
    y_roi_max_ratio: float = 0.60
    y_thresh_ratio: float = 0.06
    x_gap_max_ratio: float = 0.25
    score_k: float = 0.2
    min_cluster_size: int = 2
    tl_class_ids: Optional[Sequence[int]] = None

    def roi_bounds(self, image_width: int, image_height: int) -> Tuple[float, float, float, float]:
        return (
            0.0,
            self.y_roi_min_ratio * float(image_height),
            float(image_width),
            self.y_roi_max_ratio * float(image_height),
        )

    def y_thresh_px(self, image_height: int) -> float:
        return self.y_thresh_ratio * float(image_height)

    def x_gap_max_px(self, image_width: int) -> float:
        return self.x_gap_max_ratio * float(image_width)


@dataclass(frozen=True)
class TLCandidate:
    detection: Any
    x1: int
    y1: int
    x2: int
    y2: int
    x_c: float
    y_c: float
    area: float
    state: str


@dataclass(frozen=True)
class TLCluster:
    cluster_id: int
    members: Tuple[TLCandidate, ...]
    min_x1: int
    min_y1: int
    max_x2: int
    max_y2: int
    cluster_min_xc: float
    cluster_max_xc: float
    cluster_x: float
    cluster_y: float
    cluster_dx: float
    cluster_size: float
    score: float


@dataclass(frozen=True)
class TLClusterSelectionResult:
    roi_candidates: Tuple[TLCandidate, ...]
    clusters: Tuple[TLCluster, ...]
    selected_cluster: Optional[TLCluster]
    selected_state: str


def _to_int(v: Any) -> int:
    return int(float(v))


def _confidence(det: Any) -> float:
    if hasattr(det, "confidence"):
        return float(det.confidence)
    if hasattr(det, "conf"):
        return float(det.conf)
    if isinstance(det, (tuple, list)) and len(det) >= 5:
        return float(det[4])
    return 0.0


def _class_id(det: Any) -> Optional[int]:
    if hasattr(det, "class_id") and det.class_id is not None:
        return int(det.class_id)
    if isinstance(det, (tuple, list)) and len(det) >= 6:
        try:
            return int(det[5])
        except (TypeError, ValueError):
            return None
    return None


def _label(det: Any) -> str:
    if hasattr(det, "label"):
        return str(det.label)
    return ""


def _state(det: Any) -> str:
    if hasattr(det, "state") and det.state is not None:
        return str(det.state).upper()
    label = _label(det).lower()
    if "red" in label:
        return "RED"
    if "yellow" in label or "amber" in label:
        return "YELLOW"
    if "green" in label:
        return "GREEN"
    return "UNKNOWN"


def _coords(det: Any) -> Tuple[int, int, int, int]:
    if isinstance(det, (tuple, list)) and len(det) >= 4:
        x1, y1, x2, y2 = det[0], det[1], det[2], det[3]
    else:
        x1, y1, x2, y2 = det.x1, det.y1, det.x2, det.y2
    return _to_int(x1), _to_int(y1), _to_int(x2), _to_int(y2)


def _is_traffic_light(det: Any, tl_class_ids: Optional[Sequence[int]]) -> bool:
    cls_id = _class_id(det)
    if tl_class_ids is not None and cls_id is not None:
        return cls_id in set(tl_class_ids)
    return "light" in _label(det).lower()


def _cluster_state(cluster_members: Sequence[TLCandidate]) -> str:
    if not cluster_members:
        return "UNKNOWN"

    valid_states = ("RED", "YELLOW", "GREEN")
    counts = {s: 0 for s in valid_states}
    areas = {s: 0.0 for s in valid_states}

    for member in cluster_members:
        state = member.state if member.state in counts else None
        if state is None:
            continue
        counts[state] += 1
        areas[state] += float(member.area)

    present_states = [s for s in valid_states if counts[s] > 0]
    if not present_states:
        return "UNKNOWN"



    tie_priority = {"GREEN": 0, "YELLOW": 1, "RED": 2}
    winner = max(
        present_states,
        key=lambda s: (counts[s], areas[s], tie_priority[s]),
    )
    return winner


def _build_cluster(cluster_id: int, members: Sequence[TLCandidate], image_width: int, k: float) -> TLCluster:
    cluster_x = sum(m.x_c for m in members) / float(len(members))
    cluster_y = sum(m.y_c for m in members) / float(len(members))
    cluster_dx = abs(cluster_x - (float(image_width) / 2.0))
    cluster_size = sum(m.area for m in members)
    score = cluster_dx - (k * math.log1p(cluster_size))

    return TLCluster(
        cluster_id=cluster_id,
        members=tuple(members),
        min_x1=min(m.x1 for m in members),
        min_y1=min(m.y1 for m in members),
        max_x2=max(m.x2 for m in members),
        max_y2=max(m.y2 for m in members),
        cluster_min_xc=min(m.x_c for m in members),
        cluster_max_xc=max(m.x_c for m in members),
        cluster_x=cluster_x,
        cluster_y=cluster_y,
        cluster_dx=cluster_dx,
        cluster_size=cluster_size,
        score=score,
    )


def select_traffic_light_cluster_single_frame(
    detections: Iterable[Any],
    image_width: int,
    image_height: int,
    config: Optional[TLClusterConfig] = None,
) -> TLClusterSelectionResult:
    cfg = config or TLClusterConfig()
    _, y_roi_min, _, y_roi_max = cfg.roi_bounds(image_width, image_height)
    y_thresh = cfg.y_thresh_px(image_height)
    x_gap_max = cfg.x_gap_max_px(image_width)

    roi_candidates: List[TLCandidate] = []
    for det in detections:
        if not _is_traffic_light(det, cfg.tl_class_ids):
            continue
        if _confidence(det) < cfg.conf_min:
            continue

        x1, y1, x2, y2 = _coords(det)
        x_c = (float(x1) + float(x2)) / 2.0
        y_c = (float(y1) + float(y2)) / 2.0
        area = max(0.0, float(x2 - x1)) * max(0.0, float(y2 - y1))

        if y_roi_min <= y_c <= y_roi_max:
            roi_candidates.append(
                TLCandidate(
                    detection=det,
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                    x_c=x_c,
                    y_c=y_c,
                    area=area,
                    state=_state(det),
                )
            )

    if not roi_candidates:
        return TLClusterSelectionResult(
            roi_candidates=(),
            clusters=(),
            selected_cluster=None,
            selected_state="UNKNOWN",
        )

    sorted_candidates = sorted(roi_candidates, key=lambda c: (c.y_c, c.x_c))
    working_clusters: List[List[TLCandidate]] = []

    for cand in sorted_candidates:
        best_idx = -1
        best_y_gap = float("inf")

        for idx, members in enumerate(working_clusters):
            cluster_y = sum(m.y_c for m in members) / float(len(members))
            cluster_min_xc = min(m.x_c for m in members)
            cluster_max_xc = max(m.x_c for m in members)

            y_ok = abs(cand.y_c - cluster_y) <= y_thresh
            x_ok = (
                cand.x_c <= (cluster_max_xc + x_gap_max)
                and cand.x_c >= (cluster_min_xc - x_gap_max)
            )
            if not (y_ok and x_ok):
                continue

            y_gap = abs(cand.y_c - cluster_y)
            if y_gap < best_y_gap:
                best_y_gap = y_gap
                best_idx = idx

        if best_idx >= 0:
            working_clusters[best_idx].append(cand)
        else:
            working_clusters.append([cand])

    min_cluster_size = max(1, int(cfg.min_cluster_size))
    filtered_clusters = [members for members in working_clusters if len(members) >= min_cluster_size]
    if not filtered_clusters:
        return TLClusterSelectionResult(
            roi_candidates=tuple(sorted_candidates),
            clusters=(),
            selected_cluster=None,
            selected_state="UNKNOWN",
        )

    clusters = tuple(
        _build_cluster(cluster_id=i, members=members, image_width=image_width, k=cfg.score_k)
        for i, members in enumerate(filtered_clusters)
    )
    winner = min(clusters, key=lambda c: c.score)
    winner_state = _cluster_state(winner.members)

    return TLClusterSelectionResult(
        roi_candidates=tuple(sorted_candidates),
        clusters=clusters,
        selected_cluster=winner,
        selected_state=winner_state,
    )


def draw_cluster_debug_overlay_pygame(
    surface: Any,
    image_width: int,
    image_height: int,
    result: TLClusterSelectionResult,
    config: Optional[TLClusterConfig] = None,
    font: Optional[Any] = None,
    roi_color: Optional[Tuple[int, int, int]] = (80, 180, 255),
    centerline_color: Optional[Tuple[int, int, int]] = (255, 255, 255),
    candidate_color: Tuple[int, int, int] = (186, 189, 182),
    cluster_color: Tuple[int, int, int] = (114, 159, 207),
    winner_color: Tuple[int, int, int] = (252, 175, 62),
) -> None:
    import pygame

    cfg = config or TLClusterConfig()
    x_roi_min, y_roi_min, x_roi_max, y_roi_max = cfg.roi_bounds(image_width, image_height)
    x_centerline = float(image_width) / 2.0

    roi_rect = pygame.Rect(
        int(x_roi_min),
        int(y_roi_min),
        max(1, int(x_roi_max - x_roi_min)),
        max(1, int(y_roi_max - y_roi_min)),
    )
    if roi_color is not None:
        pygame.draw.rect(surface, roi_color, roi_rect, 2)
    if centerline_color is not None:
        pygame.draw.line(surface, centerline_color, (int(x_centerline), 0), (int(x_centerline), int(image_height)), 1)

    for cand in result.roi_candidates:
        rect = pygame.Rect(cand.x1, cand.y1, max(1, cand.x2 - cand.x1), max(1, cand.y2 - cand.y1))
        pygame.draw.rect(surface, candidate_color, rect, 1)
        if font is not None:
            dx = abs(cand.x_c - x_centerline)
            label = f"dx={dx:.1f} A={cand.area:.0f} {cand.state}"
            text = font.render(label, True, candidate_color)
            text_y = cand.y1 - 18 if cand.y1 > 20 else cand.y1 + 4
            surface.blit(text, (cand.x1, text_y))

    for cluster in result.clusters:
        is_winner = result.selected_cluster is not None and cluster.cluster_id == result.selected_cluster.cluster_id
        color = winner_color if is_winner else cluster_color
        width = 3 if is_winner else 2
        rect = pygame.Rect(
            cluster.min_x1,
            cluster.min_y1,
            max(1, cluster.max_x2 - cluster.min_x1),
            max(1, cluster.max_y2 - cluster.min_y1),
        )
        pygame.draw.rect(surface, color, rect, width)
        if font is not None:
            label = (
                f"cl#{cluster.cluster_id} n={len(cluster.members)} "
                f"dx={cluster.cluster_dx:.1f} size={cluster.cluster_size:.0f} s={cluster.score:.2f}"
            )
            text = font.render(label, True, color)
            text_y = cluster.min_y1 - 18 if cluster.min_y1 > 20 else cluster.max_y2 + 4
            surface.blit(text, (cluster.min_x1, text_y))

    if result.selected_cluster is not None and font is not None:
        winner_text = font.render(f"WINNER: state={result.selected_state}", True, winner_color)
        surface.blit(winner_text, (12, 12))

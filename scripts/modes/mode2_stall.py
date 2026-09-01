#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Mode 2: 변기칸 진입/탈출 노드 (최종 통합본 v7)

[v7 변경사항]
1) [타원형 정지 경계] 정지 판정을 원(모든 방향 동일 반경)에서 타원으로
   바꿈 - 전방(front_stop_distance)과 좌우(side_stop_distance)에 서로
   다른 반경을 줄 수 있어서, "정면은 더 붙어도 되고 좌우는 더 일찍
   멈춘다" 같은 비대칭 안전거리를 곡선 경계로 표현 가능.
   ellipse_clearance_ratio()가 (x/front)^2+(y/side)^2 비율로 판단하며,
   기존 min_omnidirectional_distance() 기반 판정을 대체(로그용으로는
   유지). front_stop_distance는 다시 0.15로 내렸는데, v4에서 이 값을
   0.25로 올렸던 이유(라이다 원점 기준 0.15는 실제 물체 충돌 확인됨)가
   여전히 유효하면 전방 반경은 다시 올려야 할 수 있음 - 실측 확인 필요.
2) [ALIGNING 겨냥각 편향 - 실험 후 되돌림] "heading을 0이 아니라
   lateral_offset에 비례한 편향각으로 맞추면 재접근 시 더 잘 수렴하지
   않을까" 하는 시도를 해봤는데, 실측해보니 편향 때문에 회전이 벽과
   평행(heading=0)한 지점에서 멈추지 않아 오히려 의도한 동작과 달라짐
   (원하는 흐름: 회전(heading=0)->후진->재접근(DRIVING의 Stanley가
   heading+lateral 다 보면서 방향을 바꿔 오차를 줄임)->다시
   회전(heading=0)->후진->... 반복). 그래서 편향 로직을 제거하고 원래
   대로 ALIGNING은 항상 heading=0(양옆 벽과 평행)만 목표로 하도록 되돌림.
   횡오차 수렴은 그대로 BACKOFF 후 DRIVING 재접근 구간에서 이뤄진다.
3) [재시도 횟수 상향] align_max_retries 2 -> 3.

[v6 변경사항]
1) [회전 후 안전거리 확보 - 라이다 피드백 방식으로 교체] odom으로 "부족한
   거리만큼 후진"하고 끝내는 방식은 odom 적분오차/지연 때문에 실제로는
   더 많이 후진해버리는 오버슈트가 실측에서 확인됨. -> odom을 아예 안
   쓰고, 후진하면서 매 틱마다 라이다(min_omnidirectional_distance)로
   실제 거리를 다시 재서 front_stop_distance에 도달하는 순간 즉시
   정지하는 폐루프(BACKOFF_TO_TARGET) 방식으로 교체.
2) [조향 게인 상향] 저속 주행 중(v가 작을 때) 조향이 잘 안 먹히는 문제가
   실측에서 확인되어(바퀴 속도 데드존 의심), k_heading을 1.0 -> 1.8로
   올려 각도 보정을 더 강하게 걸도록 함.

이전 버전(v5) 변경사항은 아래 유지:
- 후진 순서: 회전 전이 아니라 회전 후, 아직 가까우면 그때 후진
- 정지 판정: omni(전방향 최소거리) 단독 기준
- ENTER 안전 상한시간 45초
- 대각선 방향 근접 안전정지

이 파일의 나머지 구조(벽 인식 파이프라인, ENTER Stanley 조향+ALIGNING/
BACKOFF 재접근 로직, EXIT 순수후진)는 이전 버전 그대로 유지.

- /robot_mode 가 "MODE2_ENTER" 일 때만 진입(도킹) 동작
- /robot_mode 가 "MODE2_EXIT"  일 때만 탈출(후진) 동작
- 그 외의 값이면 정지하고 대기 (mode1, mode_manager와 동일한 패턴)

[벽 인식 파이프라인]  mode2_find.py 그대로 이식, 변경 없음
  (A+B) build_segments()        : 국소 기울기 추종 기반 조각 분리
  (C)   fit_line_pca()          : 각 조각 직선 피팅 (PCA)
  (B2)  merge_adjacent_segments : 방향+근접 기준 조각 재병합
  (D)   group_segments_by_direction : 평행한(마주보는) 조각끼리 그룹핑
  (E)   find_left_right_walls  : 끝점 최근접 쌍으로 2개로 수렴 후 검증

[정렬/조향]
  ENTER(진입): 벽 전체 PCA 기반 Stanley 조향 + 전진속도는 오차 크기에 따라
               조절. 정지 판정은 도킹목표/raw정면/전방향(omni) 중 최솟값
               으로 판단. 정지거리 도달했는데 정렬(heading/lateral)이
               아직 안 맞으면 ALIGNING(제자리 회전)->필요시 BACKOFF(후진
               후 재접근)로 이어짐.
  EXIT(후진): 방향조정 없이 순수 후진만 함 (w=0 고정). 정지 판정은
              "ENTER 때 실제로 이동한 직선거리(enter_distance)"만큼
              후진했는지로 판단 (진입 각도가 세계좌표축과 안 맞아도 정확).

[구독 토픽 - lidar_node 쪽 스펙]
  /lidar/front_points (std_msgs/Float32MultiArray)
    data = [x0, y0, x1, y1, ...]   (각도 오름차순 정렬 유지 필수)
    좌표계: base_link 기준 (x=전방 / y=좌측), 단위 m
"""

import time
import math
import rospy
import paho.mqtt.client as mqtt

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


SEGMENT_COLOR_PALETTE = [
    (1.0, 1.0, 0.2),   # 노랑
    (0.2, 1.0, 0.2),   # 초록
    (0.2, 0.8, 1.0),   # 하늘
    (1.0, 0.2, 1.0),   # 마젠타
    (1.0, 0.6, 0.2),   # 주황
    (0.6, 0.2, 1.0),   # 보라
    (0.2, 1.0, 0.7),   # 민트
]


def points_from_flat(flat_data):
    """Float32MultiArray.data (평탄화된 [x0,y0,x1,y1,...]) -> [(x,y), ...]"""
    return list(zip(flat_data[0::2], flat_data[1::2]))


# ============================================================
# (1) 벽 인식 파이프라인 (mode2_find.py 그대로 이식)
# ============================================================

def _fit_direction(points):
    n = len(points)
    if n < 2:
        return None

    cx = sum(p[0] for p in points) / n
    cy = sum(p[1] for p in points) / n

    sxx = syy = sxy = 0.0
    for x, y in points:
        dx, dy = x - cx, y - cy
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy

    if sxx == 0.0 and syy == 0.0:
        return None

    theta = 0.5 * math.atan2(2 * sxy, sxx - syy)
    return (math.cos(theta), math.sin(theta))


def _angle_between(v1, v2):
    a1 = math.atan2(v1[1], v1[0]) % math.pi
    a2 = math.atan2(v2[1], v2[0]) % math.pi
    d = abs(a1 - a2)
    return min(d, math.pi - d)


def build_segments(points, gap_break_dist=0.20, min_points=5,
                    angle_threshold_deg=20.0, slope_window=5,
                    cumulative_angle_threshold_deg=None):
    if len(points) < min_points:
        return []

    if cumulative_angle_threshold_deg is None:
        cumulative_angle_threshold_deg = angle_threshold_deg

    segments = []
    current = [points[0]]
    anchor_dir = None
    i = 1
    n = len(points)
    angle_th = math.radians(angle_threshold_deg)
    cum_angle_th = math.radians(cumulative_angle_threshold_deg)

    while i < n:
        p = points[i]
        last = current[-1]
        dist = math.hypot(p[0] - last[0], p[1] - last[1])

        local_dir = None
        if len(current) >= 2:
            local_dir = _fit_direction(current[-slope_window:])
            if anchor_dir is None:
                anchor_dir = local_dir

        step_dir = (p[0] - last[0], p[1] - last[1]) if dist > 1e-6 else None

        is_gap = dist > gap_break_dist
        is_corner = False
        if local_dir is not None and step_dir is not None:
            if _angle_between(local_dir, step_dir) > angle_th:
                is_corner = True
        if not is_corner and anchor_dir is not None and step_dir is not None:
            if _angle_between(anchor_dir, step_dir) > cum_angle_th:
                is_corner = True

        if is_gap or is_corner:
            looks_like_outlier = False
            if i + 1 < n and local_dir is not None:
                p_next = points[i + 1]
                dist_p_to_next = math.hypot(p_next[0] - p[0], p_next[1] - p[1])
                dist_next_from_last = math.hypot(
                    p_next[0] - last[0], p_next[1] - last[1]
                )
                step_dir_next = (p_next[0] - last[0], p_next[1] - last[1])
                ang_next = _angle_between(local_dir, step_dir_next)

                looks_like_outlier = (
                    dist_p_to_next > gap_break_dist
                    and dist_next_from_last <= gap_break_dist
                    and ang_next <= angle_th
                )

            if looks_like_outlier:
                i += 1
                continue

            if len(current) >= min_points:
                segments.append(current)
            current = [p]
            anchor_dir = None
            i += 1
            continue

        current.append(p)
        i += 1

    if len(current) >= min_points:
        segments.append(current)

    return segments


def fit_line_pca(points):
    n = len(points)
    cx = sum(p[0] for p in points) / n
    cy = sum(p[1] for p in points) / n

    sxx = syy = sxy = 0.0
    for x, y in points:
        dx, dy = x - cx, y - cy
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    sxx /= n
    syy /= n
    sxy /= n

    theta = 0.5 * math.atan2(2 * sxy, sxx - syy)
    direction = (math.cos(theta), math.sin(theta))
    normal = (-direction[1], direction[0])

    residual_sq = 0.0
    for x, y in points:
        dx, dy = x - cx, y - cy
        perp = dx * normal[0] + dy * normal[1]
        residual_sq += perp * perp
    residual = math.sqrt(residual_sq / n)

    proj = [
        (x - cx) * direction[0] + (y - cy) * direction[1]
        for x, y in points
    ]
    length = max(proj) - min(proj)

    t_min, t_max = min(proj), max(proj)
    ep1 = (cx + direction[0] * t_min, cy + direction[1] * t_min)
    ep2 = (cx + direction[0] * t_max, cy + direction[1] * t_max)

    return {
        "centroid": (cx, cy),
        "direction": direction,
        "normal": normal,
        "residual": residual,
        "length": length,
        "num_points": n,
        "points": points,
        "ep1": ep1,
        "ep2": ep2,
    }


def _endpoint_min_dist(seg_a, seg_b):
    pts_a = (seg_a["ep1"], seg_a["ep2"])
    pts_b = (seg_b["ep1"], seg_b["ep2"])
    best = None
    for pa in pts_a:
        for pb in pts_b:
            d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
            if best is None or d < best:
                best = d
    return best


def _point_to_line_perp_dist(point, line_seg):
    cx, cy = line_seg["centroid"]
    dx, dy = line_seg["direction"]
    normal = (-dy, dx)
    px, py = point[0] - cx, point[1] - cy
    return abs(px * normal[0] + py * normal[1])


def _min_perp_dist_to_line(points, line_seg):
    return min(_point_to_line_perp_dist(p, line_seg) for p in points)


def _min_point_dist_to_points(point, points):
    return min(math.hypot(point[0] - p[0], point[1] - p[1]) for p in points)


def merge_adjacent_segments(fitted_segments, angle_tolerance_deg=10.0,
                             dist_thr=0.05,
                             small_segment_max_points=3,
                             small_segment_merge_dist=0.10):
    segs = list(fitted_segments)
    angle_th = math.radians(angle_tolerance_deg)

    merged_any = True
    while merged_any:
        merged_any = False
        used = [False] * len(segs)
        next_segs = []

        for i in range(len(segs)):
            if used[i]:
                continue
            base = segs[i]
            base_points = list(base["points"])
            used[i] = True

            changed = True
            while changed:
                changed = False
                for j in range(len(segs)):
                    if used[j]:
                        continue
                    other = segs[j]

                    is_small = (base["num_points"] <= small_segment_max_points or
                                other["num_points"] <= small_segment_max_points)

                    if is_small:
                        if other["num_points"] <= base["num_points"]:
                            small_line, big_line = other, base
                        else:
                            small_line, big_line = base, other

                        perp_d = _min_perp_dist_to_line(
                            small_line["points"], big_line
                        )
                        centroid_d = _min_point_dist_to_points(
                            small_line["centroid"], big_line["points"]
                        )
                        should_merge = (
                            perp_d <= small_segment_merge_dist
                            and centroid_d <= small_segment_merge_dist * 2.0
                        )
                    else:
                        ang_diff = _angle_between(base["direction"], other["direction"])
                        d = _endpoint_min_dist(base, other)
                        should_merge = (ang_diff <= angle_th) and (d <= dist_thr)

                    if not should_merge:
                        continue

                    base_points.extend(other["points"])
                    base = fit_line_pca(base_points)
                    used[j] = True
                    changed = True
                    merged_any = True

            next_segs.append(base)

        segs = next_segs

    return segs


def _normalize_direction_angle(direction):
    angle = math.atan2(direction[1], direction[0])
    angle = angle % math.pi
    return angle


def _angle_diff_mod_pi(a1, a2):
    d = abs(a1 - a2)
    return min(d, math.pi - d)


def group_segments_by_direction(segments, angle_tolerance_deg=8.0):
    tol = math.radians(angle_tolerance_deg)
    groups = []
    group_angles = []

    for seg in segments:
        ang = _normalize_direction_angle(seg["direction"])
        seg["angle"] = ang

        matched = False
        for gi, gangle in enumerate(group_angles):
            if _angle_diff_mod_pi(ang, gangle) <= tol:
                groups[gi].append(seg)
                matched = True
                break
        if not matched:
            groups.append([seg])
            group_angles.append(ang)

    return groups


def _shared_direction(dir_a, dir_b):
    if dir_a[0] * dir_b[0] + dir_a[1] * dir_b[1] < 0:
        dir_b = (-dir_b[0], -dir_b[1])
    dx, dy = dir_a[0] + dir_b[0], dir_a[1] + dir_b[1]
    norm = math.hypot(dx, dy)
    return (dx / norm, dy / norm) if norm > 1e-6 else dir_a


def _project_t(points, direction):
    return [p[0] * direction[0] + p[1] * direction[1] for p in points]


def _overlap_range(ts_a, ts_b):
    lo = max(min(ts_a), min(ts_b))
    hi = min(max(ts_a), max(ts_b))
    if lo > hi:
        return None
    return lo, hi


def _count_facing_matches_in_overlap(points_a, points_b, direction, t_tolerance=0.03):
    ts_a = _project_t(points_a, direction)
    ts_b = _project_t(points_b, direction)

    rng = _overlap_range(ts_a, ts_b)
    if rng is None:
        return 0, 0
    lo, hi = rng
    lo -= t_tolerance
    hi += t_tolerance

    pts_a_in_overlap = [p for p, t in zip(points_a, ts_a) if lo <= t <= hi]
    if not pts_a_in_overlap:
        return 0, 0

    matched = _count_facing_matches(pts_a_in_overlap, points_b, direction, t_tolerance)
    return matched, len(pts_a_in_overlap)


def _count_facing_matches(points_a, points_b, direction, t_tolerance=0.03):
    ts_b = sorted(p[0] * direction[0] + p[1] * direction[1] for p in points_b)
    n = len(ts_b)

    matched = 0
    for p in points_a:
        t = p[0] * direction[0] + p[1] * direction[1]

        lo, hi = 0, n
        while lo < hi:
            mid = (lo + hi) // 2
            if ts_b[mid] < t:
                lo = mid + 1
            else:
                hi = mid

        best_diff = None
        if lo < n:
            best_diff = abs(ts_b[lo] - t)
        if lo > 0:
            d = abs(ts_b[lo - 1] - t)
            if best_diff is None or d < best_diff:
                best_diff = d

        if best_diff is not None and best_diff <= t_tolerance:
            matched += 1

    return matched


def merge_group_by_nearest_endpoints(segments, target_count=2):
    segs = list(segments)

    while len(segs) > target_count:
        n = len(segs)
        best_i, best_j, best_d = -1, -1, None

        for i in range(n):
            for j in range(i + 1, n):
                d = _endpoint_min_dist(segs[i], segs[j])
                if best_d is None or d < best_d:
                    best_d = d
                    best_i, best_j = i, j

        merged_points = list(segs[best_i]["points"]) + list(segs[best_j]["points"])
        merged_seg = fit_line_pca(merged_points)

        segs = [s for k, s in enumerate(segs) if k != best_i and k != best_j]
        segs.append(merged_seg)

    return segs


def _evaluate_facing_pair(seg_a, seg_b, t_tolerance=0.03, min_match_ratio=0.5):
    shared_dir = _shared_direction(seg_a["direction"], seg_b["direction"])

    matched_a, n_a_overlap = _count_facing_matches_in_overlap(
        seg_a["points"], seg_b["points"], shared_dir, t_tolerance
    )
    matched_b, n_b_overlap = _count_facing_matches_in_overlap(
        seg_b["points"], seg_a["points"], shared_dir, t_tolerance
    )

    if n_a_overlap == 0 or n_b_overlap == 0:
        return False, -1, None

    ratio_a = matched_a / float(n_a_overlap)
    ratio_b = matched_b / float(n_b_overlap)
    passed = min(ratio_a, ratio_b) >= min_match_ratio

    diag = {
        "ratio_a": ratio_a, "ratio_b": ratio_b,
        "n_a": seg_a["num_points"], "n_b": seg_b["num_points"],
        "n_a_overlap": n_a_overlap, "n_b_overlap": n_b_overlap,
    }

    if not passed:
        return False, -1, diag

    matched_a_total = _count_facing_matches(seg_a["points"], seg_b["points"], shared_dir, t_tolerance)
    matched_b_total = _count_facing_matches(seg_b["points"], seg_a["points"], shared_dir, t_tolerance)
    score = matched_a_total + matched_b_total
    return True, score, diag


def find_left_right_walls(direction_groups, min_group_points=5,
                           t_tolerance=0.03, min_match_ratio=0.5):
    best = None
    best_score = -1
    diag_lines = []

    for gi, group in enumerate(direction_groups):
        total_points = sum(s["num_points"] for s in group)
        group_angle_deg = math.degrees(group[0].get("angle", 0.0))

        if total_points < min_group_points:
            diag_lines.append(
                "group#%d segs=%d pts=%d angle=%.1fdeg -> 점개수 부족(<%d)" %
                (gi, len(group), total_points, group_angle_deg, min_group_points)
            )
            continue

        reduced = merge_group_by_nearest_endpoints(group, target_count=2)

        if len(reduced) < 2:
            diag_lines.append(
                "group#%d segs=%d pts=%d angle=%.1fdeg -> 병합해도 조각 1개뿐"
                "(마주보는 벽 쌍이 아님)" %
                (gi, len(group), total_points, group_angle_deg)
            )
            continue

        seg_a, seg_b = reduced[0], reduced[1]
        passed, score, split_diag = _evaluate_facing_pair(
            seg_a, seg_b, t_tolerance, min_match_ratio
        )

        if not passed:
            if split_diag is None:
                diag_lines.append(
                    "group#%d segs=%d pts=%d angle=%.1fdeg -> 병합 후 두 조각의 "
                    "투영구간이 겹치지 않음(마주보는 벽 아님)" %
                    (gi, len(group), total_points, group_angle_deg)
                )
            else:
                diag_lines.append(
                    "group#%d segs=%d pts=%d angle=%.1fdeg -> 병합 후 "
                    "ratio_a=%.2f(n=%d) ratio_b=%.2f(n=%d) < min_match_ratio=%.2f" %
                    (gi, len(group), total_points, group_angle_deg,
                     split_diag["ratio_a"], split_diag["n_a"],
                     split_diag["ratio_b"], split_diag["n_b"], min_match_ratio)
                )
            continue

        if score <= best_score:
            continue

        if seg_a["centroid"][1] >= seg_b["centroid"][1]:
            left_line, right_line = seg_a, seg_b
        else:
            left_line, right_line = seg_b, seg_a

        best_score = score
        best = (left_line, right_line)

    if best is None and diag_lines:
        rospy.logwarn_throttle(
            1.0,
            "[Mode2] 벽 매칭 실패 진단:\n  " + "\n  ".join(diag_lines)
        )

    return best


# ============================================================
# (2) 정렬/조향 계산 - 벽 전체 PCA 기반 (안정적, 조향의 유일한 소스)
# ============================================================

def _robust_avg_near_t(points, direction, target_t, k=5):
    """direction축 투영값이 target_t에 가장 가까운 최대 k개 점의 평균."""
    if not points:
        return None
    scored = sorted(points, key=lambda p: abs(p[0] * direction[0] + p[1] * direction[1] - target_t))
    chosen = scored[:min(k, len(scored))]
    mx = sum(p[0] for p in chosen) / len(chosen)
    my = sum(p[1] for p in chosen) / len(chosen)
    return (mx, my)


def compute_alignment(left_line, right_line, lateral_ref_k=15):
    """
    좌/우 벽 직선(벽 전체를 PCA로 피팅한 결과)으로부터
      heading_error  : 로봇 x축과 벽(복도) 진행방향 사이의 각도 [rad]
      lateral_offset : 두 벽 중앙선 대비 로봇이 치우친 거리 [m]
    lateral_offset은 로봇 현재 위치(t=0) 근처 점들의 평균으로 계산해서,
    lateral_offset=0이 곧 "로봇 바로 앞에서 실제로 중앙"이 되도록 함.
    """
    if left_line is None and right_line is None:
        return None

    if left_line is not None and right_line is not None:
        direction = _shared_direction(left_line["direction"], right_line["direction"])
    else:
        line = left_line if left_line is not None else right_line
        direction = line["direction"]

    if direction[0] < 0:
        direction = (-direction[0], -direction[1])

    heading_error = math.atan2(direction[1], direction[0])
    normal = (-direction[1], direction[0])

    lateral_offset = None
    if left_line is not None and right_line is not None:
        p_left_near = _robust_avg_near_t(left_line["points"], direction, 0.0, lateral_ref_k)
        p_right_near = _robust_avg_near_t(right_line["points"], direction, 0.0, lateral_ref_k)
        perp_left = (0.0 - p_left_near[0]) * normal[0] + (0.0 - p_left_near[1]) * normal[1]
        perp_right = (0.0 - p_right_near[0]) * normal[0] + (0.0 - p_right_near[1]) * normal[1]
        lateral_offset = 0.5 * (perp_left + perp_right)

    return {
        "direction": direction,
        "normal": normal,
        "heading_error": heading_error,
        "lateral_offset": lateral_offset,
        "has_both_walls": left_line is not None and right_line is not None,
    }


def compute_steering(alignment, k_heading=1.0, k_lateral=1.5, max_w=0.35):
    """Stanley 방식: heading_error + lateral_offset -> 각속도(w)."""
    if alignment is None:
        return 0.0

    w = k_heading * alignment["heading_error"]

    if alignment["lateral_offset"] is not None:
        w -= k_lateral * alignment["lateral_offset"]

    return max(-max_w, min(max_w, w))


def raw_front_distance(points, half_width=0.12):
    """로봇 raw x/y축 기준 정면 최소거리. 벽/변기 인식 실패 시 안전 백업용."""
    candidates = [x for (x, y) in points if x > 0 and abs(y) < half_width]
    if not candidates:
        return None
    return min(candidates)


def min_omnidirectional_distance(points):
    """
    [신규] 전방 180도 전체에서, 방향과 무관하게 로봇으로부터 가장 가까운
    점까지의 순수 거리(Euclidean). 로봇이 비스듬히 서 있을 때 변기가
    정면 좁은 밴드(raw_front_distance) 밖의 대각선 방향에 있으면 그
    함수는 못 잡아내서, 실제로는 매우 가까운데도 계속 전진하다 충돌
    위험이 생길 수 있음 - 이 함수는 방향을 안 가리고 그냥 "가장 가까운
    점이 얼마나 가까운지"만 보므로 그런 경우도 놓치지 않는다.
    """
    if not points:
        return None
    return min(math.hypot(x, y) for x, y in points)


def ellipse_clearance_ratio(points, front_radius, side_radius, side_angle_deg=90.0):
    """
    정지 경계를 원이 아니라 타원으로 판단한다.
    전방(x, 로봇 정면) 방향 반경 front_radius, side_angle_deg 각도(로봇
    정면 기준) 방향 반경 side_radius인 타원을 로봇 주변 안전 경계로 본다.

    각 점 (x,y)에 대해 실제 각도 theta=atan2(y,x)를 구하고, side_radius가
    "완전히 적용되는" 기준각을 90도가 아니라 side_angle_deg로 다시
    스케일링한다:
        theta' = theta * (90도 / side_angle_deg), 단 |theta'| > 90도면 90도로 clamp
    그 다음 표준 타원 극좌표 경계와 비교한 비율을 계산한다:
        ratio = sqrt((r*cos(theta')/front_radius)^2 + (r*sin(theta')/side_radius)^2)
    ratio <= 1.0이면 그 점이 타원 안쪽(=경계보다 가까움).
    반환값은 전체 점 중 최솟값(가장 심하게 파고든 점의 비율) - 이 값이
    1.0 이하로 내려가는 순간이 "정지해야 하는 시점"이다. 점이 없으면 None.

    side_angle_deg=90.0(기본값)이면 순수 x/y축 기준 타원과 동일 - 즉
    side_radius가 정확히 로봇 정면 기준 ±90도(완전 옆)에서만 100% 적용됨.
    side_angle_deg를 90보다 작게 주면(예: 80.0) 그 각도에서 이미
    side_radius가 100% 적용되고, 그보다 더 벌어진 각도(예: 80~90도)는
    side_radius를 그대로 유지한다 - "얼마나 벌어진 각도부터 좌우 반경을
    완전히 적용할지"를 별도로 조절할 수 있다.
    """
    if not points:
        return None
    max_theta = math.pi / 2.0
    scale = max_theta / math.radians(side_angle_deg)
    best = None
    for x, y in points:
        r = math.hypot(x, y)
        if r < 1e-9:
            ratio = 0.0
        else:
            theta_eff = math.atan2(y, x) * scale
            if theta_eff > max_theta:
                theta_eff = max_theta
            elif theta_eff < -max_theta:
                theta_eff = -max_theta
            ratio = math.hypot(
                (r * math.cos(theta_eff)) / front_radius,
                (r * math.sin(theta_eff)) / side_radius,
            )
        if best is None or ratio < best:
            best = ratio
    return best


# ============================================================
# (3) 도킹 목표점 계산 - 정지거리 판정 + 시각화 전용
# ============================================================

def _shared_wall_direction_normal(left_line, right_line):
    d1 = left_line["direction"]
    d2 = right_line["direction"]
    if d1[0] * d2[0] + d1[1] * d2[1] < 0:
        d2 = (-d2[0], -d2[1])
    dx, dy = d1[0] + d2[0], d1[1] + d2[1]
    norm = math.hypot(dx, dy)
    direction = (dx / norm, dy / norm) if norm > 1e-6 else d1
    if direction[0] < 0:
        direction = (-direction[0], -direction[1])
    normal = (-direction[1], direction[0])
    return direction, normal


def compute_dock_target(left_line, right_line, front_points, half_width=0.12, k=5):
    """
    변기 맨 앞 꼭짓점(P_toilet) 근처 위치를 찾아 "front_dist"(정지 판정용
    실제 거리)를 계산한다. heading_error/lateral_offset도 함께 반환하지만,
    시각화/진단용일 뿐 조향에는 사용하지 않는다.
    """
    direction, normal = _shared_wall_direction_normal(left_line, right_line)

    cxl, cyl = left_line["centroid"]
    cxr, cyr = right_line["centroid"]
    center_n = 0.5 * (
        (cxl * normal[0] + cyl * normal[1]) +
        (cxr * normal[0] + cyr * normal[1])
    )

    band = [
        (x, y) for (x, y) in front_points
        if abs((x * normal[0] + y * normal[1]) - center_n) < half_width
        and (x * direction[0] + y * direction[1]) > 0
    ]
    if not band:
        return None

    band.sort(key=lambda p: p[0] * direction[0] + p[1] * direction[1])
    t_toilet = band[0][0] * direction[0] + band[0][1] * direction[1]
    chosen = band[:min(k, len(band))]
    p_toilet = (
        sum(p[0] for p in chosen) / len(chosen),
        sum(p[1] for p in chosen) / len(chosen),
    )

    p_left_near = _robust_avg_near_t(left_line["points"], direction, t_toilet, k)
    p_right_near = _robust_avg_near_t(right_line["points"], direction, t_toilet, k)
    if p_left_near is None or p_right_near is None:
        return {"front_dist": t_toilet, "p_toilet": p_toilet, "p_wall_mid": None}

    p_wall_mid = (
        0.5 * (p_left_near[0] + p_right_near[0]),
        0.5 * (p_left_near[1] + p_right_near[1]),
    )

    return {
        "front_dist": t_toilet,
        "p_toilet": p_toilet,
        "p_wall_mid": p_wall_mid,
    }


def _linear_ratio(value, start, stop, min_ratio):
    """value가 start 이하면 1.0, stop 이상이면 min_ratio, 그 사이는 선형 보간."""
    v = abs(value)
    if v <= start:
        return 1.0
    if v >= stop:
        return min_ratio
    span = stop - start
    return 1.0 - (1.0 - min_ratio) * (v - start) / span


def compute_forward_speed(base_speed, heading_error, lateral_offset,
                           heading_slowdown_start_deg=10.0, heading_stop_deg=35.0,
                           lateral_slowdown_start=0.05, lateral_stop=0.20,
                           min_speed_ratio=0.25):
    """heading_error와 lateral_offset 중 더 심한 쪽 기준으로 전진속도 감속."""
    heading_ratio = _linear_ratio(
        math.degrees(heading_error), heading_slowdown_start_deg,
        heading_stop_deg, min_speed_ratio
    )
    if lateral_offset is None:
        lateral_ratio = 1.0
    else:
        lateral_ratio = _linear_ratio(
            lateral_offset, lateral_slowdown_start, lateral_stop, min_speed_ratio
        )
    ratio = min(heading_ratio, lateral_ratio)
    return base_speed * ratio


class Mode2StallNode(object):

    ENTER_MODE = "MODE2_ENTER"
    EXIT_MODE = "MODE2_EXIT"

    def __init__(self):
        rospy.init_node("mode2_stall_node")

        # ---------------- 파라미터: 기본 ----------------
        self.points_topic = rospy.get_param("~points_topic", "/lidar/front_points")
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.control_rate = rospy.get_param("~control_rate", 20.0)
        self.sensor_timeout = rospy.get_param("~sensor_timeout", 0.5)
        self.min_points_required = rospy.get_param("~min_points_required", 10)

        # [변기 뚜껑 서보 - MQTT] test_lid.py로 검증한 브릿지를 이 노드에
        # 직접 통합. 정렬 다 끝나고 최종 접근(FINAL_DISTANCE_FIX) 시작하는
        # 순간 "ENTER"(뚜껑 열기)를, EXIT phase 시작(후진 시작) 순간
        # "EXIT"(뚜껑 닫기)를 publish한다. 도킹 자체는 MQTT 연결 여부와
        # 무관하게 계속 동작해야 하므로 연결 실패해도 노드가 죽지 않게 함.
        self.mqtt_broker = rospy.get_param("~mqtt_broker", "localhost")
        self.mqtt_port = rospy.get_param("~mqtt_port", 1883)
        self.topic_lid_cmd = rospy.get_param("~topic_lid_cmd", "mode2/lid_cmd")

        self.enter_speed = rospy.get_param("~enter_speed", 0.04)
        self.exit_speed = rospy.get_param("~exit_speed", -0.04)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.8)

        # [수정] 저속 주행 중(v가 작을 때) 조향이 잘 안 먹히는 문제가
        # 실측에서 확인되어(바퀴 속도 데드존 의심), 각도 보정 비중을 올림
        self.k_heading = rospy.get_param("~k_heading", 1.8)
        self.k_lateral = rospy.get_param("~k_lateral", 6.0)

        self.lateral_ref_k = rospy.get_param("~lateral_ref_k", 15)
        self.dock_point_k = rospy.get_param("~dock_point_k", 5)

        self.heading_slowdown_start_deg = rospy.get_param("~heading_slowdown_start_deg", 10.0)
        self.heading_stop_deg = rospy.get_param("~heading_stop_deg", 35.0)
        self.lateral_slowdown_start = rospy.get_param("~lateral_slowdown_start", 0.05)
        self.lateral_stop = rospy.get_param("~lateral_stop", 0.20)
        self.min_speed_ratio = rospy.get_param("~min_speed_ratio", 0.25)

        self.steering_smoothing_alpha = rospy.get_param("~steering_smoothing_alpha", 0.4)
        self._filtered_w = 0.0

        self.seg_min_points = rospy.get_param("~seg_min_points", 5)
        self.gap_break_dist = rospy.get_param("~gap_break_dist", 0.20)
        self.angle_threshold_deg = rospy.get_param("~angle_threshold_deg", 20.0)
        self.slope_window = rospy.get_param("~slope_window", 5)
        self.cumulative_angle_threshold_deg = rospy.get_param(
            "~cumulative_angle_threshold_deg", self.angle_threshold_deg
        )
        self.merge_angle_tolerance_deg = rospy.get_param("~merge_angle_tolerance_deg", 10.0)
        self.merge_dist_thr = rospy.get_param("~merge_dist_thr", 0.05)
        self.small_segment_max_points = rospy.get_param("~small_segment_max_points", 3)
        self.small_segment_merge_dist = rospy.get_param("~small_segment_merge_dist", 0.10)
        self.angle_tolerance_deg = rospy.get_param("~angle_tolerance_deg", 8.0)
        self.min_group_points = rospy.get_param("~min_group_points", 5)
        self.t_tolerance = rospy.get_param("~t_tolerance", 0.03)
        self.min_match_ratio = rospy.get_param("~min_match_ratio", 0.5)

        # [v8] 정렬(회전/재시도 등 방향이 계속 바뀌는 동작)은 전부 라이다
        # 최소감지거리 사각지대 밖(front_stop_distance=0.25)에서 끝내고,
        # 정렬이 다 끝난(성공이든 재시도 소진이든) 뒤에만 마지막으로
        # 순수 직진(w=0)으로 진짜 목표거리(final_dock_distance=0.15)까지
        # 들어간다 (_start_final_distance_fix 참고). 이렇게 하면 사각지대에
        # 들어가는 구간은 "이미 각도/중앙이 맞은 상태의 짧은 직진"뿐이라
        # 위험이 훨씬 줄어든다.
        self.front_stop_distance = rospy.get_param("~front_stop_distance", 0.25)
        self.final_dock_distance = rospy.get_param("~final_dock_distance", 0.10)
        self.side_stop_distance = rospy.get_param("~side_stop_distance", 0.30)
        # side_stop_distance가 100% 적용되는 기준각(로봇 정면 기준). 90도면
        # 순수 옆(±90도)에서만 완전 적용, 80도면 ±80도부터 이미 완전
        # 적용되고 80~90도 구간은 그대로 유지 (ellipse_clearance_ratio 참고).
        self.side_stop_angle_deg = rospy.get_param("~side_stop_angle_deg", 80.0)
        self.front_center_half_width = rospy.get_param("~front_center_half_width", 0.12)
        self.pause_after_approach = rospy.get_param("~pause_after_approach", 1.0)

        # [안전장치] 라이다 최소감지거리 사각지대 대비. 대부분의 저가형
        # 라이다는 너무 가까운 물체는 거리를 못 재고 inf/NaN을 반환하는데,
        # lidar_node가 그런 점을 통째로 버리기 때문에(포인트 자체가 사라짐)
        # "가장 가까운 위험한 점"이 오히려 계산에서 빠져버려 안전정지가
        # 못 걸리고 그대로 충돌하는 문제가 실측에서 확인됨. 직전 프레임에
        # 정면거리가 (front_stop_distance + 이 마진) 이내로 가까웠는데
        # 이번 프레임에 정면 포인트가 통째로 사라지면, "여전히 그만큼
        # 가깝다"고 안전하게 가정하고 강제로 정지 판정한다.
        self.front_dropout_safety_margin = rospy.get_param("~front_dropout_safety_margin", 0.10)

        self.align_heading_tol_deg = rospy.get_param("~align_heading_tol_deg", 4.0)
        self.align_lateral_tol = rospy.get_param("~align_lateral_tol", 0.02)
        self.align_max_duration = rospy.get_param("~align_max_duration", 10.0)

        # [수정] 제자리 회전(v=0) 중 목표에 가까워져 w가 작아지면 바퀴가
        # 정지마찰을 못 이기고 거의 안 도는 데드존이 실측에서 확인됨
        # (heading이 4~5도 근처에서 9초 넘게 거의 안 줄어드는 현상).
        # w_raw가 0은 아니지만 이 값보다 작으면 부호는 유지한 채
        # align_min_w까지 끌어올린다 (v가 있는 DRIVING 중엔 바퀴가 이미
        # 움직이고 있어 이 문제가 덜하므로 ALIGN 전용으로 적용).
        self.align_min_w = rospy.get_param("~align_min_w", 0.30)

        # [수정] 재시도용 후진 목표가 front_stop_distance(0.25, 사각지대
        # 회피용으로 커짐) + align_backoff_dist라서, 0.08을 그대로 두면
        # 왕복 거리가 너무 커진다는 피드백으로 축소.
        self.align_backoff_dist = rospy.get_param("~align_backoff_dist", 0.04)
        self.align_max_retries = rospy.get_param("~align_max_retries", 2)

        # [수정] ALIGN/BACKOFF/BACKOFF_TO_TARGET을 재시도마다 오갈 수 있어서
        # 최초 접근(가변, 관측상 10~15초)만 가정한 짧은 상한시간으로는
        # 마지막 재시도 도중 강제 종료되는 문제가 실측에서 확인됨. 여유 있게
        # 60초로 상향 (align_max_retries=3 기준 넉넉한 안전마진, 정상
        # 상황에선 이보다 훨씬 빨리 끝남).
        self.max_enter_duration = rospy.get_param("~max_enter_duration", 60.0)
        self.exit_goal_tolerance = rospy.get_param("~exit_goal_tolerance", 0.03)
        self.exit_max_duration = rospy.get_param("~exit_max_duration", 10.0)

        self.enable_debug_markers = rospy.get_param("~enable_debug_markers", True)
        self.point_size = rospy.get_param("~point_size", 0.03)

        # ---------------- 상태 변수 ----------------
        self.front_points = []
        self.last_points_time = None

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.pose_received = False

        self.entry_position = None
        self.enter_distance = None
        self.exit_start_position = None

        self.current_robot_mode = None
        self.active_phase = None
        self.phase_state = None
        self.phase_start_time = None
        self.enter_phase_start_time = None

        self._align_retry_count = 0
        self._backoff_margin = None
        self._backoff_on_done = None

        # 라이다 최소감지거리 사각지대 안전장치용 (직전 정면거리 기억)
        self._last_raw_front_distance = None

        self.last_left_line = None
        self.last_right_line = None

        self.stall_id = "?"

        # ---------------- Publisher / Subscriber ----------------
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.status_pub = rospy.Publisher("/mode2_status", String, queue_size=10)

        if self.enable_debug_markers:
            self.segments_pub = rospy.Publisher(
                "/mode2_debug/segments", MarkerArray, queue_size=1
            )
            self.walls_pub = rospy.Publisher(
                "/mode2_debug/walls", MarkerArray, queue_size=1
            )
            self.dock_target_pub = rospy.Publisher(
                "/mode2_debug/dock_target", MarkerArray, queue_size=1
            )
            self.corridor_target_pub = rospy.Publisher(
                "/mode2_debug/corridor_target", MarkerArray, queue_size=1
            )

        rospy.Subscriber(self.points_topic, Float32MultiArray, self.points_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/robot_mode", String, self.mode_callback)
        rospy.Subscriber("/current_stall_id", String, self.stall_id_callback)

        # ---------------- MQTT (변기 뚜껑 서보) ----------------
        self.mqtt_client = None
        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=60)
            self.mqtt_client.loop_start()
        except Exception as e:
            rospy.logwarn("[Mode2] MQTT 연결 실패(%s) -> 뚜껑 제어 없이 도킹만 계속 진행", e)
            self.mqtt_client = None

        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo("[Mode2] 노드 시작. 대기 중 (mode=%s/%s 신호를 기다림)",
                      self.ENTER_MODE, self.EXIT_MODE)

    # ==================== MQTT (변기 뚜껑 서보) ====================
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            rospy.loginfo("[Mode2] MQTT 연결됨 %s:%d", self.mqtt_broker, self.mqtt_port)
        else:
            rospy.logwarn("[Mode2] MQTT 연결 실패, rc=%d", rc)

    def _publish_lid_cmd(self, cmd):
        if self.mqtt_client is None:
            rospy.logwarn("[Mode2] MQTT 미연결 -> 뚜껑 명령(%s) 전송 못함", cmd)
            return
        try:
            self.mqtt_client.publish(self.topic_lid_cmd, cmd)
            rospy.loginfo("[Mode2] MQTT %s -> '%s' publish", self.topic_lid_cmd, cmd)
        except Exception as e:
            rospy.logwarn("[Mode2] 뚜껑 명령(%s) publish 실패: %s", cmd, e)

    # ==================== Callbacks ====================
    def points_callback(self, msg):
        self.front_points = points_from_flat(msg.data)
        self.last_points_time = time.time()

    def stall_id_callback(self, msg):
        self.stall_id = msg.data

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.pose_received = True

    def mode_callback(self, msg):
        new_mode = msg.data
        prev_mode = self.current_robot_mode
        self.current_robot_mode = new_mode

        if new_mode == self.ENTER_MODE and prev_mode != self.ENTER_MODE:
            self.start_phase("ENTER")
        elif new_mode == self.EXIT_MODE and prev_mode != self.EXIT_MODE:
            self.start_phase("EXIT")
        elif new_mode not in (self.ENTER_MODE, self.EXIT_MODE):
            if self.active_phase is not None:
                self.stop_robot()
                rospy.loginfo("[Mode2] 비활성화 -> 정지")
            self.active_phase = None
            self.phase_state = None

    def start_phase(self, phase):
        self.active_phase = phase
        self.phase_state = "DRIVING"
        self.phase_start_time = time.time()

        self.last_left_line = None
        self.last_right_line = None
        self._filtered_w = 0.0
        self._last_raw_front_distance = None

        self.stop_robot()

        if phase == "ENTER":
            self.enter_phase_start_time = self.phase_start_time
            self._align_retry_count = 0

            if self.pose_received:
                self.entry_position = (self.odom_x, self.odom_y)
                rospy.loginfo("[Mode2] ENTER 시작 위치 기록: x=%.3f y=%.3f",
                              self.odom_x, self.odom_y)
            else:
                self.entry_position = None
                rospy.logwarn("[Mode2] odom을 아직 못 받아서 ENTER 시작 위치를 "
                              "기록 못함 -> EXIT 때 시간 기반으로 대체됨")

        elif phase == "EXIT":
            if self.pose_received:
                self.exit_start_position = (self.odom_x, self.odom_y)
                rospy.loginfo(
                    "[Mode2] EXIT 시작. 현재위치=(%.3f,%.3f) 목표 후진거리=%s",
                    self.odom_x, self.odom_y,
                    ("%.3fm" % self.enter_distance) if self.enter_distance is not None else "?(시간기반 대체)"
                )
            else:
                self.exit_start_position = None

            # [변기 뚜껑] MODE2_EXIT 받고 후진 시작하는 시점에 뚜껑 닫기
            # (MQTT "EXIT") publish.
            self._publish_lid_cmd("EXIT")

        rospy.loginfo("[Mode2] %s phase 시작 (stall_id=%s)", phase, self.stall_id)

    # ==================== Helper ====================
    def points_ready(self):
        if self.last_points_time is None:
            return False
        if time.time() - self.last_points_time > self.sensor_timeout:
            return False
        return len(self.front_points) >= self.min_points_required

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def publish_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def apply_steering_filter(self, w_raw):
        alpha = self.steering_smoothing_alpha
        self._filtered_w = alpha * w_raw + (1.0 - alpha) * self._filtered_w
        return self._filtered_w

    def find_walls(self):
        raw_segments = build_segments(
            self.front_points, self.gap_break_dist, self.seg_min_points,
            self.angle_threshold_deg, self.slope_window,
            self.cumulative_angle_threshold_deg
        )

        fitted_segments = []
        result = None
        if raw_segments:
            fitted_segments = [fit_line_pca(s) for s in raw_segments]
            fitted_segments = merge_adjacent_segments(
                fitted_segments, self.merge_angle_tolerance_deg, self.merge_dist_thr,
                self.small_segment_max_points, self.small_segment_merge_dist
            )
            direction_groups = group_segments_by_direction(
                fitted_segments, self.angle_tolerance_deg
            )
            result = find_left_right_walls(
                direction_groups, self.min_group_points,
                self.t_tolerance, self.min_match_ratio
            )

        if self.enable_debug_markers:
            self.publish_segment_markers(fitted_segments)

        if result is not None:
            left_line, right_line = result
            self.last_left_line = left_line
            self.last_right_line = right_line
            if self.enable_debug_markers:
                self.publish_wall_markers(left_line, right_line)
            return left_line, right_line

        if self.last_left_line is not None or self.last_right_line is not None:
            rospy.logwarn_throttle(
                1.0, "[Mode2] 이번 프레임 벽 인식 실패 -> 직전 인식 결과 재사용"
            )
            if self.enable_debug_markers:
                self.publish_wall_markers(self.last_left_line, self.last_right_line)
            return self.last_left_line, self.last_right_line

        rospy.logwarn_throttle(1.0, "[Mode2] 벽 인식 실패, 이전 기록도 없음")
        if self.enable_debug_markers:
            self.publish_wall_markers(None, None)
        return None, None

    # ==================== 디버그 마커 ====================
    def _points_marker(self, ns, mid, points, color, size):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.scale.x = size
        m.scale.y = size
        m.color.r, m.color.g, m.color.b, m.color.a = color
        for x, y in points:
            p = Point()
            p.x, p.y, p.z = x, y, 0.0
            m.points.append(p)
        return m

    def _line_marker(self, ns, mid, p1, p2, color, width):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = width
        m.color.r, m.color.g, m.color.b, m.color.a = color
        for (x, y) in (p1, p2):
            p = Point()
            p.x, p.y, p.z = x, y, 0.0
            m.points.append(p)
        return m

    def _text_marker(self, ns, mid, pos, text, color, size):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = 0.15
        m.scale.z = size
        m.color.r, m.color.g, m.color.b, m.color.a = color
        m.text = text
        return m

    def publish_segment_markers(self, fitted_segments):
        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i, seg in enumerate(fitted_segments):
            color3 = SEGMENT_COLOR_PALETTE[i % len(SEGMENT_COLOR_PALETTE)]
            color = (color3[0], color3[1], color3[2], 1.0)
            marker_array.markers.append(
                self._points_marker("segments", i, seg["points"], color, self.point_size)
            )
            marker_array.markers.append(
                self._text_marker(
                    "segment_labels", i, seg["centroid"],
                    "#%d (%d pts)" % (i, seg["num_points"]), color, 0.12
                )
            )
        self.segments_pub.publish(marker_array)

    def publish_wall_markers(self, left_line, right_line):
        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        if left_line is not None:
            marker_array.markers.append(
                self._points_marker("left_wall", 0, left_line["points"],
                                     (0.2, 0.5, 1.0, 1.0), self.point_size)
            )
            marker_array.markers.append(
                self._line_marker("left_wall_line", 0,
                                   left_line["ep1"], left_line["ep2"],
                                   (0.2, 0.5, 1.0, 0.9), 0.015)
            )

        if right_line is not None:
            marker_array.markers.append(
                self._points_marker("right_wall", 0, right_line["points"],
                                     (1.0, 0.2, 0.2, 1.0), self.point_size)
            )
            marker_array.markers.append(
                self._line_marker("right_wall_line", 0,
                                   right_line["ep1"], right_line["ep2"],
                                   (1.0, 0.2, 0.2, 0.9), 0.015)
            )

        self.walls_pub.publish(marker_array)

    def publish_dock_target_markers(self, target):
        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        if target is not None and target.get("p_toilet") is not None:
            marker_array.markers.append(
                self._points_marker("dock_p_toilet", 0, [target["p_toilet"]],
                                     (0.2, 1.0, 1.0, 1.0), self.point_size * 2)
            )
            if target.get("p_wall_mid") is not None:
                marker_array.markers.append(
                    self._points_marker("dock_p_wall_mid", 0, [target["p_wall_mid"]],
                                         (0.2, 1.0, 1.0, 1.0), self.point_size * 2)
                )
                marker_array.markers.append(
                    self._line_marker("dock_target_line", 0,
                                       target["p_toilet"], target["p_wall_mid"],
                                       (0.2, 1.0, 1.0, 0.9), 0.01)
                )

        self.dock_target_pub.publish(marker_array)

    def publish_corridor_target_markers(self, alignment, front_dist):
        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        marker_array.markers.append(
            self._line_marker("robot_forward", 0, (0.0, 0.0), (0.4, 0.0),
                               (1.0, 1.0, 0.0, 0.9), 0.02)
        )

        if alignment is not None:
            direction = alignment["direction"]
            normal = alignment["normal"]
            lateral_offset = alignment["lateral_offset"]

            if lateral_offset is not None:
                base = (-lateral_offset * normal[0], -lateral_offset * normal[1])
            else:
                base = (0.0, 0.0)

            tip = (base[0] + direction[0] * 0.6, base[1] + direction[1] * 0.6)
            marker_array.markers.append(
                self._line_marker("target_direction", 0, base, tip,
                                   (0.2, 1.0, 0.2, 0.9), 0.02)
            )

            heading_deg = math.degrees(alignment["heading_error"])
            lat_txt = ("%.3fm" % lateral_offset) if lateral_offset is not None else "?(한쪽벽만)"
            fd_txt = ("%.3fm" % front_dist) if front_dist is not None else "?"
            marker_array.markers.append(
                self._text_marker(
                    "target_info", 0, (0.1, 0.35),
                    "heading_err=%.1fdeg lateral=%s front_dist=%s" %
                    (heading_deg, lat_txt, fd_txt),
                    (1.0, 1.0, 1.0, 1.0), 0.08
                )
            )

        self.corridor_target_pub.publish(marker_array)

    # ==================== ENTER: 조향/속도 계산 ====================
    def _clearance_ratio(self, margin=0.0):
        """현재 front_points 기준 타원 정지 경계 침투 비율 (1.0 이하면 정지해야 함).
        ellipse_clearance_ratio() 참고 - front_stop_distance/side_stop_distance로
        전방/좌우 반경이 다른 타원을 쓰고, side_stop_angle_deg로 side_stop_distance가
        100% 적용되는 기준각을 조절함. margin(m)을 주면 전방/좌우 반경에 그만큼
        더 여유를 둔 확장 타원 기준으로 판단한다 (재접근용 여유거리 확보에 사용)."""
        return ellipse_clearance_ratio(
            self.front_points,
            self.front_stop_distance + margin, self.side_stop_distance + margin,
            self.side_stop_angle_deg
        )

    def compute_enter_command(self):
        """
        ENTER용 (v, w, front_dist, alignment, clearance_ratio)를 계산한다.

        [v7] 정지 판정은 이제 원이 아니라 타원 경계 기준
        (clearance_ratio <= 1.0, ellipse_clearance_ratio 참고) - 전방과
        좌우에 서로 다른 안전반경(front_stop_distance/side_stop_distance)을
        줄 수 있다. front_dist(omni_fd)는 로그/디버그 마커 표시용으로만
        남긴다.
        """
        left_line, right_line = self.find_walls()
        alignment = compute_alignment(left_line, right_line, self.lateral_ref_k)

        raw_fd = raw_front_distance(self.front_points, self.front_center_half_width)
        omni_fd = min_omnidirectional_distance(self.front_points)
        front_dist = omni_fd  # 로그/시각화 참고용 (정지 판정은 clearance_ratio로 함)
        clearance_ratio = self._clearance_ratio()

        # [안전장치] 라이다 최소감지거리 사각지대: 직전엔 정면에 가까운
        # 점이 있었는데(라이다가 감지할 수 있는 한계 근처) 이번 프레임에
        # 정면 포인트가 통째로 사라졌다면, 실제로는 여전히(또는 더) 가까울
        # 가능성이 높다. inf로 찍혀 점 자체가 없어지는 하드웨어 특성상
        # "포인트가 없음"을 "안전함"으로 착각하면 그대로 충돌하므로,
        # 이 경우 clearance_ratio를 강제로 0으로 만들어 정지 판정을 낸다.
        if raw_fd is None and self._last_raw_front_distance is not None:
            if self._last_raw_front_distance <= self.front_stop_distance + self.front_dropout_safety_margin:
                rospy.logwarn_throttle(
                    0.5,
                    "[Mode2] 정면 라이다 포인트 소실(직전 정면거리=%.3fm) -> "
                    "최소감지거리 사각지대 진입 의심, 안전 정지 처리",
                    self._last_raw_front_distance
                )
                clearance_ratio = 0.0
        if raw_fd is not None:
            self._last_raw_front_distance = raw_fd

        if alignment is None:
            w = self.apply_steering_filter(0.0)
            if self.enable_debug_markers:
                self.publish_corridor_target_markers(None, front_dist)
                self.publish_dock_target_markers(None)
            rospy.loginfo_throttle(
                1.0, "[Mode2][ENTER] 벽 인식 없음 -> 탐색 중 (정지거리(omni)=%s 참고 raw_front=%s)",
                ("%.3fm" % omni_fd) if omni_fd is not None else "?",
                ("%.3fm" % raw_fd) if raw_fd is not None else "?",
            )
            return self.enter_speed * 0.5, w, front_dist, None, clearance_ratio

        target = None
        if alignment["has_both_walls"]:
            target = compute_dock_target(
                left_line, right_line, self.front_points,
                half_width=self.front_center_half_width, k=self.dock_point_k,
            )

        if self.enable_debug_markers:
            self.publish_dock_target_markers(target)
            self.publish_corridor_target_markers(alignment, front_dist)

        toilet_seen = target is not None and target.get("p_wall_mid") is not None
        dock_fd = target["front_dist"] if target is not None else None
        rospy.loginfo_throttle(
            0.5,
            "[Mode2][ENTER] 변기감지=%s 목표방향(벽평행)=%.1fdeg 횡오차=%s "
            "정지거리(omni)=%s (참고: dock=%s raw=%s)",
            "O" if toilet_seen else "X",
            math.degrees(alignment["heading_error"]),
            ("%.3fm" % alignment["lateral_offset"]) if alignment["lateral_offset"] is not None else "?",
            ("%.3fm" % front_dist) if front_dist is not None else "?",
            ("%.3fm" % dock_fd) if dock_fd is not None else "?",
            ("%.3fm" % raw_fd) if raw_fd is not None else "?",
        )

        w_raw = compute_steering(
            alignment, k_heading=self.k_heading, k_lateral=self.k_lateral,
            max_w=self.max_angular_speed,
        )
        w = self.apply_steering_filter(w_raw)

        v = compute_forward_speed(
            self.enter_speed, alignment["heading_error"], alignment["lateral_offset"],
            self.heading_slowdown_start_deg, self.heading_stop_deg,
            self.lateral_slowdown_start, self.lateral_stop,
            self.min_speed_ratio,
        )

        rospy.loginfo_throttle(
            0.5, "[Mode2][ENTER] cmd v=%.3f w_raw=%.3f w_filtered=%.3f clearance_ratio=%s",
            v, w_raw, w, ("%.2f" % clearance_ratio) if clearance_ratio is not None else "?"
        )

        return v, w, front_dist, alignment, clearance_ratio

    # ==================== Phase handlers ====================
    def _enter_settle(self):
        self.stop_robot()
        self.phase_state = "SETTLING"
        self.phase_start_time = time.time()

        if self.entry_position is not None and self.pose_received:
            traveled = math.hypot(
                self.odom_x - self.entry_position[0],
                self.odom_y - self.entry_position[1],
            )
            self.enter_distance = traveled
            rospy.loginfo("[Mode2] ENTER 정지 -> 시작위치 대비 이동거리=%.3fm "
                          "(EXIT 때 이만큼 후진함)", traveled)
        else:
            self.enter_distance = None

    def _is_aligned(self, alignment):
        if alignment is None:
            return True
        if abs(math.degrees(alignment["heading_error"])) > self.align_heading_tol_deg:
            return False
        if (alignment["lateral_offset"] is not None
                and abs(alignment["lateral_offset"]) > self.align_lateral_tol):
            return False
        return True

    def _heading_aligned(self, alignment):
        return (
            alignment is None
            or abs(math.degrees(alignment["heading_error"])) <= self.align_heading_tol_deg
        )

    def _lateral_aligned(self, alignment):
        return (
            alignment is None
            or alignment["lateral_offset"] is None
            or abs(alignment["lateral_offset"]) <= self.align_lateral_tol
        )

    def _start_aligning(self):
        """정지거리 도달, 정렬 미완료 -> 바로 제자리 회전으로 heading부터 맞춘다.
        (후진은 회전 전이 아니라 회전 끝난 뒤, 아직도 너무 가까우면 그때 함)"""
        self.phase_state = "ALIGNING"
        self.phase_start_time = time.time()
        rospy.loginfo("[Mode2] 정지거리 도달, 정렬 미완료 -> 제자리 회전으로 heading부터 맞춤")

    def step_align(self):
        """
        [되돌림] 목표 heading은 0(양옆 벽과 평행)으로 그대로 맞춘다 - 편향각을
        주는 실험을 해봤는데(lateral_offset에 비례해서 heading을 틀어서
        겨냥), 원하시는 방식과 달라서 되돌림. 대신 횡오차 보정은 원래
        의도대로 "회전(heading=0) -> 후진 -> 재접근(DRIVING, Stanley가
        heading+lateral 둘 다 보며 방향을 바꿔 오차를 줄임) -> 다시
        회전(heading=0) -> 후진 -> ..." 재시도 루프에서 이뤄진다.
        """
        left_line, right_line = self.find_walls()
        alignment = compute_alignment(left_line, right_line, self.lateral_ref_k)

        if self.enable_debug_markers:
            raw_fd = raw_front_distance(self.front_points, self.front_center_half_width)
            self.publish_corridor_target_markers(alignment, raw_fd)

        heading_ok = self._heading_aligned(alignment)
        timed_out = time.time() - self.phase_start_time >= self.align_max_duration

        if heading_ok or timed_out:
            if timed_out and not heading_ok:
                rospy.logwarn("[Mode2] 제자리 회전 상한시간(%.1fs) 초과 (heading 못 맞춤)",
                              self.align_max_duration)
            else:
                rospy.loginfo("[Mode2] 제자리 회전으로 heading 정렬 완료")
            self._after_align(alignment)
            return

        w_raw = compute_steering(
            alignment, k_heading=self.k_heading, k_lateral=0.0,
            max_w=self.max_angular_speed,
        )
        if 0.0 < abs(w_raw) < self.align_min_w:
            w_raw = math.copysign(self.align_min_w, w_raw)
        w = self.apply_steering_filter(w_raw)

        rospy.loginfo_throttle(
            0.5, "[Mode2][ALIGN] heading=%.1fdeg w=%.3f",
            math.degrees(alignment["heading_error"]) if alignment else 0.0, w,
        )

        self.publish_cmd(0.0, w)

    def _after_align(self, alignment):
        """
        회전이 끝난 직후 호출. 여기서 먼저 "회전하고 났더니 아직도
        타원 정지 경계 안쪽인지"부터 확인한다 (회전 중 몸체가 휩쓸려서
        거리가 더 줄었을 수 있음). odom으로 "이만큼만 후진"하고 끝내는 게
        아니라, 후진하면서 매 틱마다 라이다로 실제 거리를 다시 재서
        경계를 벗어나는 순간 바로 멈춘다 (odom 적분오차/지연으로 실제보다
        더 많이 가버리는 오버슈트 문제 방지 - odom은 이 후진 동작에서
        아예 안 씀).
        """
        raw_fd = raw_front_distance(self.front_points, self.front_center_half_width)
        omni_fd = min_omnidirectional_distance(self.front_points)
        clearance_ratio = self._clearance_ratio()
        rospy.loginfo(
            "[Mode2] heading 정렬 직후 전방거리 raw=%s omni=%s (목표 front_stop_distance=%.3fm) clearance_ratio=%s",
            ("%.3fm" % raw_fd) if raw_fd is not None else "?",
            ("%.3fm" % omni_fd) if omni_fd is not None else "?",
            self.front_stop_distance,
            ("%.2f" % clearance_ratio) if clearance_ratio is not None else "?",
        )

        if clearance_ratio is not None and clearance_ratio <= 1.0:
            rospy.loginfo(
                "[Mode2] 회전 후에도 타원 정지 경계 안쪽(ratio=%.2f) -> 라이다로 실거리 보면서 후진",
                clearance_ratio
            )
            self._begin_lidar_backoff(0.0, on_done=self._resume_after_lidar_backoff)
            return

        self._decide_after_align(alignment)

    def _resume_after_lidar_backoff(self):
        """안전거리 확보용 BACKOFF(margin=0)가 끝난 뒤 호출.
        fresh 정렬을 다시 계산해서 횡오차 판단으로 넘어간다."""
        left_line, right_line = self.find_walls()
        fresh_alignment = compute_alignment(left_line, right_line, self.lateral_ref_k)
        self._decide_after_align(fresh_alignment)

    def _decide_after_align(self, alignment):
        """정렬(heading) + 안전거리 확인이 끝난 뒤, 횡오차까지 봐서
        재접근할지, 아니면(정렬 성공이든 재시도 소진이든) 마지막으로
        진짜 목표거리(final_dock_distance)까지 순수 직진해서 마무리할지
        결정한다. 정렬/재시도는 전부 라이다 사각지대 밖(front_stop_distance)
        에서 끝내고, 사각지대 안까지 들어가는 마지막 직진은 이미 각도/
        중앙이 맞은 상태에서만 하도록 하기 위함."""
        if self._lateral_aligned(alignment):
            rospy.loginfo(
                "[Mode2] 정렬 완료 -> 라이다로 전방거리를 최종 목표(%.3fm)까지 맞춘 뒤 정지",
                self.final_dock_distance
            )
            self._start_final_distance_fix()
            return

        if self._align_retry_count >= self.align_max_retries:
            rospy.logwarn(
                "[Mode2] 재접근 %d회 후에도 횡오차 못 줄임(허용 %.3fm 초과) -> "
                "정렬은 포기하고 라이다로 전방거리만 최종 목표(%.3fm)에 맞춘 뒤 정지",
                self._align_retry_count, self.align_lateral_tol, self.final_dock_distance
            )
            self._start_final_distance_fix()
            return

        self._align_retry_count += 1
        rospy.loginfo(
            "[Mode2] 횡오차 남음 -> 라이다로 실거리 보면서 %.3fm 여유 두고 후진한 뒤 재접근 (재시도 %d/%d)",
            self.align_backoff_dist, self._align_retry_count, self.align_max_retries
        )
        self._begin_lidar_backoff(self.align_backoff_dist, on_done=self._resume_driving_after_backoff)

    def _resume_driving_after_backoff(self):
        self.phase_state = "DRIVING"
        self.phase_start_time = time.time()
        rospy.loginfo("[Mode2] 재접근 시작")

    def _begin_lidar_backoff(self, margin, on_done):
        """
        [교체] 예전엔 odom으로 '이만큼만 후진'하고 끝내서, odom 오차/속도
        이상(예: 후진 순간 속도가 튀는 현상)이 있으면 실제로는 목표보다
        훨씬 더/덜 물러나 버리는 문제가 실측에서 확인됨(회전->후진 직후
        바로 다시 정지거리에 걸려버리거나, 반대로 필요 이상으로 멀리
        빠지는 증상). 이제 odom을 아예 안 쓰고, 매 틱마다 라이다로 실제
        거리를 다시 재서 "정지 경계(front/side_stop_distance)보다
        margin(m)만큼 더 여유 있는 지점"에 도달하는 순간 즉시 멈춘다.
        이미 그 여유를 확보한 상태면(현재 ratio가 벌써 1.0 이상이면)
        전혀 후진하지 않고 바로 on_done으로 넘어간다.
        on_done: 후진이 끝나면(정상 도달이든 타임아웃이든) 호출할 콜백.
        """
        self.phase_state = "BACKOFF"
        self.phase_start_time = time.time()
        self._backoff_margin = margin
        self._backoff_on_done = on_done

    def step_backoff(self):
        clearance_ratio = self._clearance_ratio(self._backoff_margin)
        elapsed = time.time() - self.phase_start_time

        if clearance_ratio is not None and clearance_ratio >= 1.0:
            self.stop_robot()
            rospy.loginfo(
                "[Mode2] 라이다 기준 여유거리(+%.3fm) 확보(ratio=%.2f) -> 후진 종료",
                self._backoff_margin, clearance_ratio
            )
            self._backoff_on_done()
            return

        if elapsed >= self.align_max_duration:
            self.stop_robot()
            rospy.logwarn(
                "[Mode2] 여유거리 확보 후진 상한시간(%.1fs) 초과 (마지막 ratio=%s) -> 그대로 진행",
                self.align_max_duration,
                ("%.2f" % clearance_ratio) if clearance_ratio is not None else "?"
            )
            self._backoff_on_done()
            return

        rospy.loginfo_throttle(
            0.5, "[Mode2][BACKOFF] clearance_ratio(margin+%.3fm)=%s (>=1.0 되면 종료)",
            self._backoff_margin,
            ("%.2f" % clearance_ratio) if clearance_ratio is not None else "?",
        )
        self.publish_cmd(-self.enter_speed, 0.0)

    def _start_final_distance_fix(self):
        """
        [v8] 정렬(heading/lateral)이 끝난 뒤(성공이든 재시도 소진이든)
        항상 거치는 마지막 단계: 더 이상 방향은 안 바꾸고(w=0) 순수
        직진/후진만으로 라이다 전방거리를 진짜 목표거리(final_dock_distance,
        front_stop_distance보다 더 가까움 - 라이다 최소감지거리 사각지대
        안쪽일 수 있음)에 맞춘 뒤 정지한다. 정렬/재시도는 전부 사각지대
        밖(front_stop_distance)에서 끝냈기 때문에, 사각지대에 들어가는
        구간은 이미 각도/중앙이 맞은 상태의 짧은 직진뿐이라 안전하다.

        [변기 뚜껑] 여기가 "밖에서 정렬 다 끝내고 안으로 들어가기 시작하는"
        시점이라, 이 순간 뚜껑 열기(MQTT "ENTER")를 publish한다 - 도킹이
        완전히 끝날 때까지 기다리지 않고 미리 열어둬서 서보가 움직일
        시간을 벌어준다.
        """
        self.phase_state = "FINAL_DISTANCE_FIX"
        self.phase_start_time = time.time()
        self._publish_lid_cmd("ENTER")

    def step_final_distance_fix(self):
        """
        raw_front_distance만 본다(omni로 대체하지 않음) - 정면 좁은 밴드
        밖의 엉뚱한 점을 기준삼아 계속 전진해버리는 걸 막기 위함. 사각지대에
        들어가서 포인트가 사라지면(raw_fd=None), "더 안 보인다 = 이미
        그만큼(또는 더) 가깝다"로 보고 그 자리에서 바로 정지한다
        (front_dropout_safety_margin 안이었을 때만 - compute_enter_command의
        사각지대 안전장치와 같은 논리).
        """
        raw_fd = raw_front_distance(self.front_points, self.front_center_half_width)
        elapsed = time.time() - self.phase_start_time
        tol = 0.02

        if raw_fd is None:
            if (self._last_raw_front_distance is not None
                    and self._last_raw_front_distance <= self.front_stop_distance + self.front_dropout_safety_margin):
                rospy.loginfo(
                    "[Mode2] 최종 접근 중 정면 포인트 소실(직전=%.3fm) -> "
                    "사각지대 진입으로 보고 그 자리에서 정지",
                    self._last_raw_front_distance
                )
            else:
                rospy.logwarn("[Mode2] 전방거리 측정 불가 -> 그대로 정지 진행")
            self._enter_settle()
            return

        self._last_raw_front_distance = raw_fd

        if abs(raw_fd - self.final_dock_distance) <= tol:
            rospy.loginfo("[Mode2] 전방거리 맞춤 완료(%.3fm, 목표 %.3fm) -> 정지",
                          raw_fd, self.final_dock_distance)
            self._enter_settle()
            return

        if elapsed >= self.align_max_duration:
            rospy.logwarn(
                "[Mode2] 전방거리 맞춤 상한시간(%.1fs) 초과(마지막 %.3fm/목표 %.3fm) -> 그대로 정지",
                self.align_max_duration, raw_fd, self.final_dock_distance
            )
            self._enter_settle()
            return

        v = self.enter_speed if raw_fd > self.final_dock_distance else -self.enter_speed
        rospy.loginfo_throttle(
            0.5, "[Mode2][FINAL_DISTANCE_FIX] front=%.3fm target=%.3fm v=%.3f",
            raw_fd, self.final_dock_distance, v
        )
        self.publish_cmd(v, 0.0)

    def step_enter(self):
        if self.phase_state == "DRIVING":
            v, w, front_dist, alignment, clearance_ratio = self.compute_enter_command()

            if clearance_ratio is not None and clearance_ratio <= 1.0:
                if self._is_aligned(alignment):
                    self._start_final_distance_fix()
                else:
                    self._start_aligning()
                return

            if time.time() - self.enter_phase_start_time >= self.max_enter_duration:
                rospy.logwarn("[Mode2] ENTER 안전 상한시간(%.1fs) 초과 "
                              "-> 강제 종료", self.max_enter_duration)
                self._enter_settle()
                return

            self.publish_cmd(v, w)
            return

        if self.phase_state == "ALIGNING":
            self.step_align()
            return

        if self.phase_state == "BACKOFF":
            self.step_backoff()
            return

        if self.phase_state == "FINAL_DISTANCE_FIX":
            self.step_final_distance_fix()
            return

        if self.phase_state == "SETTLING":
            self.stop_robot()
            if time.time() - self.phase_start_time >= self.pause_after_approach:
                self.phase_state = "DONE"
                self.status_pub.publish(String(data="MODE2_ENTER_DONE:%s" % self.stall_id))
                rospy.loginfo("[Mode2] 진입(도킹) 완료 (stall_id=%s) -> mode3 대기", self.stall_id)
            return

    def step_exit(self):
        """
        EXIT은 방향조정 없이 순수 후진만 한다 (w=0 고정).
        정지 판정은 "ENTER 때 실제로 이동한 직선거리"만큼 후진했는지로 판단.
        """
        if self.phase_state != "DRIVING":
            return

        elapsed = time.time() - self.phase_start_time

        if self.enter_distance is None or self.exit_start_position is None:
            if elapsed >= self.exit_max_duration:
                self._finish_exit()
                return
            rospy.loginfo_throttle(
                0.5, "[Mode2][EXIT] (시간기반) v=%.3f w=0.0 elapsed=%.1fs",
                self.exit_speed, elapsed
            )
            self.publish_cmd(self.exit_speed, 0.0)
            return

        traveled = math.hypot(
            self.odom_x - self.exit_start_position[0],
            self.odom_y - self.exit_start_position[1],
        )
        remaining = self.enter_distance - traveled

        rospy.loginfo_throttle(
            0.5,
            "[Mode2][EXIT] v=%.3f w=0.0 traveled=%.3fm / target=%.3fm "
            "(remaining=%.3fm)",
            self.exit_speed, traveled, self.enter_distance, remaining
        )

        if remaining <= self.exit_goal_tolerance:
            self._finish_exit()
            return

        if elapsed >= self.exit_max_duration:
            rospy.logwarn("[Mode2] EXIT 안전 상한시간(%.1fs) 초과 "
                          "(traveled=%.3fm / target=%.3fm) -> 강제 종료",
                          self.exit_max_duration, traveled, self.enter_distance)
            self._finish_exit()
            return

        self.publish_cmd(self.exit_speed, 0.0)

    def _finish_exit(self):
        self.stop_robot()
        self.phase_state = "DONE"
        self.status_pub.publish(String(data="MODE2_EXIT_DONE:%s" % self.stall_id))
        rospy.loginfo("[Mode2] 탈출 완료 (stall_id=%s) -> mode1 대기", self.stall_id)
        self.entry_position = None
        self.enter_distance = None
        self.exit_start_position = None

    # ==================== Main loop ====================
    def step(self):
        if self.active_phase is None:
            return

        if self.phase_state == "DONE":
            return

        if not self.points_ready():
            rospy.logwarn_throttle(1.0, "[Mode2] 라이다 포인트 데이터 대기/끊김")
            self.stop_robot()
            return

        if self.active_phase == "ENTER":
            self.step_enter()
        elif self.active_phase == "EXIT":
            self.step_exit()

    def spin(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()

    def shutdown_hook(self):
        rospy.loginfo("[Mode2] 노드 종료. 정지.")
        try:
            self.stop_robot()
            self.stop_robot()
        except Exception:
            pass
        if self.mqtt_client is not None:
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except Exception:
                pass


if __name__ == "__main__":
    try:
        node = Mode2StallNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
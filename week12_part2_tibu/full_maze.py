from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Iterable, Sequence

import cv2
import numpy as np

from planner_core import (
    pixels_to_mm,
    transform_point,
    validate_corners,
)


DIRECTIONS = ("N", "E", "S", "W")
DIRECTION_DELTAS = ((-1, 0), (0, 1), (1, 0), (0, -1))


@dataclass(frozen=True)
class StandardRoute:
    commands: str
    cells: list[tuple[int, int]]
    final_direction: str


@dataclass(frozen=True)
class CoursePlacement:
    top: int
    left: int
    cells: frozenset[tuple[int, int]]
    entrance_cell: tuple[int, int]
    entrance_direction: str
    exit_cell: tuple[int, int]
    exit_direction: str


@dataclass
class FullMazeResult:
    rectified: np.ndarray
    dark_mask: np.ndarray
    horizontal_walls: np.ndarray
    vertical_walls: np.ndarray
    homography: np.ndarray
    placement: CoursePlacement
    pre_route: StandardRoute
    post_route: StandardRoute


def _cyan_grid_marker_mask(image: np.ndarray, config: dict) -> np.ndarray:
    """Keep the small cyan wall clips while rejecting the large coloured rails."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.asarray(
        [
            int(config.get("grid_marker_hue_min", 75)),
            int(config.get("grid_marker_saturation_min", 45)),
            int(config.get("grid_marker_value_min", 35)),
        ],
        dtype=np.uint8,
    )
    upper = np.asarray(
        [int(config.get("grid_marker_hue_max", 110)), 255, 255],
        dtype=np.uint8,
    )
    raw = cv2.inRange(hsv, lower, upper)
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(raw)
    filtered = np.zeros(raw.shape, dtype=np.uint8)
    image_area = image.shape[0] * image.shape[1]
    minimum_area = max(3, int(round(image_area * 0.000002)))
    maximum_area = max(80, int(round(image_area * 0.001)))
    for label in range(1, component_count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if minimum_area <= area <= maximum_area:
            filtered[labels == label] = 1
    return filtered


def _fit_regular_grid_axis(
    histogram: np.ndarray,
    origin_range: tuple[float, float],
    step_range: tuple[float, float],
) -> tuple[float, float, list[float]]:
    smoothed = cv2.GaussianBlur(
        histogram.astype(np.float32).reshape(1, -1), (0, 0), 3.0
    ).reshape(-1)
    best: tuple[float, float, float, list[float]] | None = None
    for step in np.arange(step_range[0], step_range[1] + 0.01, 0.5):
        for origin in np.arange(origin_range[0], origin_range[1] + 0.01, 1.0):
            positions = origin + step * np.arange(10)
            if positions[-1] >= len(smoothed):
                continue
            peaks: list[float] = []
            for position in positions:
                centre = int(round(float(position)))
                sample = smoothed[max(0, centre - 6) : min(len(smoothed), centre + 7)]
                peaks.append(float(np.max(sample)) if len(sample) else 0.0)
            score = sum(math.log1p(value) for value in peaks)
            score += 2.0 * math.log1p(min(peaks))
            if best is None or score > best[0]:
                best = score, float(origin), float(step), peaks
    if best is None:
        raise ValueError("The complete 9 x 9 maze grid could not be detected")
    _, origin, step, peaks = best
    median_peak = float(np.median(peaks))
    supported = sum(value >= median_peak * 0.2 for value in peaks)
    if median_peak < 1.0 or supported < 8:
        raise ValueError(
            "The complete 9 x 9 maze grid could not be detected from the cyan wall markers"
        )
    return origin, step, peaks


def detect_full_maze_corners(image: np.ndarray, config: dict) -> np.ndarray:
    """Automatically locate the logical 9 x 9 square in the fixed overhead view."""
    if image is None or image.ndim != 3:
        raise ValueError("A colour overhead maze image is required")
    height, width = image.shape[:2]
    marker_mask = _cyan_grid_marker_mask(image, config)

    x_mask = marker_mask.copy()
    x_mask[: int(round(0.12 * height)), :] = 0
    x_mask[int(round(0.90 * height)) :, :] = 0
    x_mask[:, : int(round(0.25 * width))] = 0
    x_mask[:, int(round(0.82 * width)) :] = 0
    y_mask = marker_mask.copy()
    y_mask[: int(round(0.08 * height)), :] = 0
    y_mask[int(round(0.92 * height)) :, :] = 0
    y_mask[:, : int(round(0.28 * width))] = 0
    y_mask[:, int(round(0.82 * width)) :] = 0

    x0, x_step, _ = _fit_regular_grid_axis(
        x_mask.sum(axis=0),
        (0.28 * width, 0.42 * width),
        (0.035 * width, 0.055 * width),
    )
    y0, y_step, _ = _fit_regular_grid_axis(
        y_mask.sum(axis=1),
        (0.10 * height, 0.26 * height),
        (0.060 * height, 0.095 * height),
    )
    if abs(x_step - y_step) > 0.15 * max(x_step, y_step):
        raise ValueError("The automatically detected 9 x 9 grid spacing is inconsistent")
    x9 = x0 + 9.0 * x_step
    y9 = y0 + 9.0 * y_step
    if not 0.85 <= (x9 - x0) / (y9 - y0) <= 1.15:
        raise ValueError("The automatically detected 9 x 9 maze is not square")
    corners = np.asarray(
        [[x0, y0], [x9, y0], [x9, y9], [x0, y9]], dtype=np.float32
    )
    validate_corners(corners)
    return corners


def rectify_full_maze(
    image: np.ndarray, corners: Sequence[Sequence[float]], config: dict
) -> tuple[np.ndarray, np.ndarray]:
    source = validate_corners(corners)
    output_pixels = int(config.get("full_maze_rectified_pixels", 1620))
    last = float(output_pixels - 1)
    destination = np.asarray(
        [[0.0, 0.0], [last, 0.0], [last, last], [0.0, last]],
        dtype=np.float32,
    )
    homography = cv2.getPerspectiveTransform(source, destination)
    rectified = cv2.warpPerspective(image, homography, (output_pixels, output_pixels))
    return rectified, homography


def _dark_fraction(mask: np.ndarray, x0: int, y0: int, x1: int, y1: int) -> float:
    x0 = min(max(x0, 0), mask.shape[1])
    x1 = min(max(x1, 0), mask.shape[1])
    y0 = min(max(y0, 0), mask.shape[0])
    y1 = min(max(y1, 0), mask.shape[0])
    if x1 <= x0 or y1 <= y0:
        return 1.0
    return float(np.count_nonzero(mask[y0:y1, x0:x1])) / float((y1 - y0) * (x1 - x0))


def detect_standard_walls(
    rectified: np.ndarray, config: dict
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return horizontal[8,9] and vertical[9,8] internal maze walls."""
    count = int(config.get("full_maze_cell_count", 9))
    size_mm = float(config.get("full_maze_size_mm", 1620.0))
    pixels_per_mm = (rectified.shape[0] - 1) / size_mm
    cell_px = float(config["maze_cell_size_mm"]) * pixels_per_mm
    half_band = max(2, int(round(float(config.get("wall_sample_half_width_mm", 12.0)) * pixels_per_mm)))
    end_trim = int(round(float(config.get("wall_sample_end_trim_mm", 28.0)) * pixels_per_mm))
    threshold = float(config.get("wall_dark_fraction_threshold", 0.25))
    wall_gray_threshold = int(config.get("wall_gray_threshold", 145))
    gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
    dark_mask = np.where(gray <= wall_gray_threshold, 255, 0).astype(np.uint8)

    horizontal = np.zeros((count - 1, count), dtype=bool)
    vertical = np.zeros((count, count - 1), dtype=bool)
    for row in range(count - 1):
        y = int(round((row + 1) * cell_px))
        for col in range(count):
            x0 = int(round(col * cell_px)) + end_trim
            x1 = int(round((col + 1) * cell_px)) - end_trim
            horizontal[row, col] = _dark_fraction(
                dark_mask, x0, y - half_band, x1, y + half_band + 1
            ) >= threshold
    for row in range(count):
        y0 = int(round(row * cell_px)) + end_trim
        y1 = int(round((row + 1) * cell_px)) - end_trim
        for col in range(count - 1):
            x = int(round((col + 1) * cell_px))
            vertical[row, col] = _dark_fraction(
                dark_mask, x - half_band, y0, x + half_band + 1, y1
            ) >= threshold
    return horizontal, vertical, dark_mask


def _forward_cell(
    cell: tuple[int, int], direction_index: int
) -> tuple[int, int]:
    dr, dc = DIRECTION_DELTAS[direction_index]
    return cell[0] + dr, cell[1] + dc


def _edge_is_open(
    cell: tuple[int, int],
    direction_index: int,
    horizontal_walls: np.ndarray,
    vertical_walls: np.ndarray,
) -> bool:
    row, col = cell
    if direction_index == 0:
        return row > 0 and not horizontal_walls[row - 1, col]
    if direction_index == 1:
        return col < vertical_walls.shape[1] and not vertical_walls[row, col]
    if direction_index == 2:
        return row < horizontal_walls.shape[0] and not horizontal_walls[row, col]
    return col > 0 and not vertical_walls[row, col - 1]


def plan_standard_route(
    horizontal_walls: np.ndarray,
    vertical_walls: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    start_direction: str,
    blocked_cells: Iterable[tuple[int, int]] = (),
    goal_direction: str | None = None,
) -> StandardRoute:
    count = horizontal_walls.shape[1]
    blocked = set(blocked_cells)
    for cell, label in ((start, "start"), (goal, "goal")):
        if not (0 <= cell[0] < count and 0 <= cell[1] < count):
            raise ValueError(f"Full-maze {label} cell is outside the 9 x 9 maze")
        if cell in blocked:
            raise ValueError(f"Full-maze {label} cell lies inside the selected obstacle course")
    direction = start_direction.upper()
    if direction not in DIRECTIONS:
        raise ValueError("Start direction must be N, E, S, or W")
    required_direction = goal_direction.upper() if goal_direction else None
    if required_direction is not None and required_direction not in DIRECTIONS:
        raise ValueError("Goal direction must be N, E, S, or W")

    start_state = (start[0], start[1], DIRECTIONS.index(direction))
    queue: list[tuple[int, int, tuple[int, int, int]]] = []
    serial = 0
    heapq.heappush(queue, (10 * (abs(start[0] - goal[0]) + abs(start[1] - goal[1])), serial, start_state))
    cost = {start_state: 0}
    parent: dict[tuple[int, int, int], tuple[tuple[int, int, int], str]] = {}
    closed: set[tuple[int, int, int]] = set()

    while queue:
        _, _, state = heapq.heappop(queue)
        if state in closed:
            continue
        row, col, heading = state
        if (row, col) == goal and (
            required_direction is None or DIRECTIONS[heading] == required_direction
        ):
            actions: list[str] = []
            states = [state]
            while states[-1] != start_state:
                previous, action = parent[states[-1]]
                actions.append(action)
                states.append(previous)
            actions.reverse()
            cells = _cells_from_commands(start, direction, "".join(actions))
            return StandardRoute("".join(actions), cells, DIRECTIONS[heading])
        closed.add(state)

        transitions = (
            ((row, col, (heading - 1) % 4), "l", 2),
            ((row, col, (heading + 1) % 4), "r", 2),
        )
        for neighbour, action, step_cost in transitions:
            new_cost = cost[state] + step_cost
            if new_cost < cost.get(neighbour, math.inf):
                cost[neighbour] = new_cost
                parent[neighbour] = (state, action)
                serial += 1
                heuristic = 10 * (abs(neighbour[0] - goal[0]) + abs(neighbour[1] - goal[1]))
                heapq.heappush(queue, (new_cost + heuristic, serial, neighbour))

        if _edge_is_open((row, col), heading, horizontal_walls, vertical_walls):
            next_cell = _forward_cell((row, col), heading)
            if next_cell not in blocked:
                neighbour = (next_cell[0], next_cell[1], heading)
                new_cost = cost[state] + 10
                if new_cost < cost.get(neighbour, math.inf):
                    cost[neighbour] = new_cost
                    parent[neighbour] = (state, "f")
                    serial += 1
                    heuristic = 10 * (abs(next_cell[0] - goal[0]) + abs(next_cell[1] - goal[1]))
                    heapq.heappush(queue, (new_cost + heuristic, serial, neighbour))
    raise ValueError("No standard-maze route connects the supplied location and obstacle course")


def _direction_after_actions(start_direction: str, commands: str) -> str:
    index = DIRECTIONS.index(start_direction.upper())
    for command in commands:
        if command == "l":
            index = (index - 1) % 4
        elif command == "r":
            index = (index + 1) % 4
    return DIRECTIONS[index]


def _cells_from_commands(
    start: tuple[int, int], start_direction: str, commands: str
) -> list[tuple[int, int]]:
    direction = DIRECTIONS.index(start_direction.upper())
    cell = start
    cells = [cell]
    for command in commands:
        if command == "l":
            direction = (direction - 1) % 4
        elif command == "r":
            direction = (direction + 1) % 4
        elif command == "f":
            cell = _forward_cell(cell, direction)
            cells.append(cell)
    return cells


def _full_point_mm(
    point: Sequence[float], homography: np.ndarray, config: dict
) -> tuple[float, float]:
    output_pixels = int(config.get("full_maze_rectified_pixels", 1620))
    size_mm = float(config.get("full_maze_size_mm", 1620.0))
    return pixels_to_mm(transform_point(point, homography), output_pixels, size_mm)


def _nearest_global_cell(point_mm: Sequence[float], config: dict) -> tuple[int, int]:
    cell_size = float(config["maze_cell_size_mm"])
    count = int(config.get("full_maze_cell_count", 9))
    col = int(round(float(point_mm[0]) / cell_size - 0.5))
    row = int(round(float(point_mm[1]) / cell_size - 0.5))
    return min(max(row, 0), count - 1), min(max(col, 0), count - 1)


def map_course_placement(
    homography: np.ndarray,
    course_corners: Sequence[Sequence[float]],
    entrance_point: Sequence[float],
    exit_point: Sequence[float],
    config: dict,
) -> CoursePlacement:
    cell_size = float(config["maze_cell_size_mm"])
    full_count = int(config.get("full_maze_cell_count", 9))
    course_count = int(config["course_cell_count"])
    mapped_corners = [_full_point_mm(point, homography, config) for point in course_corners]
    center_x = sum(point[0] for point in mapped_corners) / 4.0
    center_y = sum(point[1] for point in mapped_corners) / 4.0
    left = int(round(center_x / cell_size - course_count / 2.0))
    top = int(round(center_y / cell_size - course_count / 2.0))
    if not (0 <= top <= full_count - course_count and 0 <= left <= full_count - course_count):
        raise ValueError("The selected 5 x 5 obstacle area does not fit inside the selected 9 x 9 maze")
    cells = frozenset(
        (row, col)
        for row in range(top, top + course_count)
        for col in range(left, left + course_count)
    )
    entrance = _nearest_global_cell(_full_point_mm(entrance_point, homography, config), config)
    exit_cell = _nearest_global_cell(_full_point_mm(exit_point, homography, config), config)

    def connection(cell: tuple[int, int], label: str) -> str:
        row, col = cell
        if row == top - 1 and left <= col < left + course_count:
            return "S" if label == "entrance" else "N"
        if row == top + course_count and left <= col < left + course_count:
            return "N" if label == "entrance" else "S"
        if col == left - 1 and top <= row < top + course_count:
            return "E" if label == "entrance" else "W"
        if col == left + course_count and top <= row < top + course_count:
            return "W" if label == "entrance" else "E"
        raise ValueError(
            f"The selected course {label} cell is not immediately outside the mapped 5 x 5 area"
        )

    entrance_direction = connection(entrance, "entrance")
    exit_direction = connection(exit_cell, "exit")
    return CoursePlacement(
        top,
        left,
        cells,
        entrance,
        entrance_direction,
        exit_cell,
        exit_direction,
    )


def plan_full_maze(
    image: np.ndarray,
    full_corners: Sequence[Sequence[float]],
    course_corners: Sequence[Sequence[float]],
    course_entrance: Sequence[float],
    course_exit: Sequence[float],
    maze_start: tuple[int, int, str],
    maze_goal: tuple[int, int],
    config: dict,
) -> FullMazeResult:
    rectified, homography = rectify_full_maze(image, full_corners, config)
    horizontal, vertical, dark_mask = detect_standard_walls(rectified, config)
    placement = map_course_placement(
        homography, course_corners, course_entrance, course_exit, config
    )
    pre_route = plan_standard_route(
        horizontal,
        vertical,
        (maze_start[0], maze_start[1]),
        placement.entrance_cell,
        maze_start[2],
        blocked_cells=placement.cells,
        goal_direction=placement.entrance_direction,
    )
    post_route = plan_standard_route(
        horizontal,
        vertical,
        placement.exit_cell,
        maze_goal,
        placement.exit_direction,
        blocked_cells=placement.cells,
    )
    return FullMazeResult(
        rectified,
        dark_mask,
        horizontal,
        vertical,
        homography,
        placement,
        pre_route,
        post_route,
    )


def _global_cell_original_point(
    cell: tuple[int, int], result: FullMazeResult, config: dict
) -> tuple[int, int]:
    cell_size = float(config["maze_cell_size_mm"])
    size_mm = float(config.get("full_maze_size_mm", 1620.0))
    output_pixels = int(config.get("full_maze_rectified_pixels", 1620))
    pixels_per_mm = (output_pixels - 1) / size_mm
    x = (cell[1] + 0.5) * cell_size * pixels_per_mm
    y = (cell[0] + 0.5) * cell_size * pixels_per_mm
    point = np.asarray([[[x, y]]], dtype=np.float32)
    mapped = cv2.perspectiveTransform(point, np.linalg.inv(result.homography))[0, 0]
    return int(round(float(mapped[0]))), int(round(float(mapped[1])))


def render_full_route_on_original(
    annotated_course_image: np.ndarray,
    result: FullMazeResult,
    maze_start: tuple[int, int, str],
    maze_goal: tuple[int, int],
    config: dict,
) -> np.ndarray:
    output = annotated_course_image.copy()

    def draw_cells(cells: Sequence[tuple[int, int]], colour: tuple[int, int, int]) -> None:
        points = np.asarray(
            [_global_cell_original_point(cell, result, config) for cell in cells],
            dtype=np.int32,
        )
        if len(points) >= 2:
            cv2.polylines(output, [points], False, colour, 5, cv2.LINE_AA)
        for point in points[1:-1]:
            cv2.circle(output, tuple(point), 5, colour, -1)

    draw_cells(result.pre_route.cells, (0, 165, 255))
    draw_cells(result.post_route.cells, (255, 200, 0))
    start_point = _global_cell_original_point((maze_start[0], maze_start[1]), result, config)
    goal_point = _global_cell_original_point(maze_goal, result, config)
    cv2.circle(output, start_point, 12, (0, 200, 0), -1)
    cv2.circle(output, goal_point, 12, (0, 0, 255), -1)
    cv2.putText(output, "MAZE START", (start_point[0] + 12, start_point[1] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 120, 0), 2)
    cv2.putText(output, "MAZE GOAL", (goal_point[0] + 12, goal_point[1] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 180), 2)
    return output


def render_full_maze_debug(result: FullMazeResult, config: dict) -> np.ndarray:
    canvas = result.rectified.copy()
    count = int(config.get("full_maze_cell_count", 9))
    cell_px = (canvas.shape[0] - 1) / count
    for row in range(count - 1):
        y = int(round((row + 1) * cell_px))
        for col in range(count):
            if result.horizontal_walls[row, col]:
                cv2.line(canvas, (int(col * cell_px), y), (int((col + 1) * cell_px), y), (0, 0, 255), 5)
    for row in range(count):
        for col in range(count - 1):
            if result.vertical_walls[row, col]:
                x = int(round((col + 1) * cell_px))
                cv2.line(canvas, (x, int(row * cell_px)), (x, int((row + 1) * cell_px)), (0, 0, 255), 5)
    overlay = canvas.copy()
    for row, col in result.placement.cells:
        p0 = (int(round(col * cell_px)), int(round(row * cell_px)))
        p1 = (int(round((col + 1) * cell_px)), int(round((row + 1) * cell_px)))
        cv2.rectangle(overlay, p0, p1, (255, 0, 255), -1)
    canvas = cv2.addWeighted(overlay, 0.16, canvas, 0.84, 0)

    def draw_route(cells: Sequence[tuple[int, int]], colour: tuple[int, int, int]) -> None:
        points = np.asarray(
            [
                (
                    int(round((cell[1] + 0.5) * cell_px)),
                    int(round((cell[0] + 0.5) * cell_px)),
                )
                for cell in cells
            ],
            dtype=np.int32,
        )
        if len(points) >= 2:
            cv2.polylines(canvas, [points], False, colour, 6, cv2.LINE_AA)

    draw_route(result.pre_route.cells, (0, 165, 255))
    draw_route(result.post_route.cells, (255, 200, 0))
    return canvas

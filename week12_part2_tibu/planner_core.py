from __future__ import annotations

import heapq
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class Obstacle:
    x_mm: float
    y_mm: float
    radius_mm: float
    circularity: float


@dataclass(frozen=True)
class Motion:
    turn_deg: float
    distance_mm: float


@dataclass
class PlanResult:
    rectified: np.ndarray
    dark_mask: np.ndarray
    occupancy: np.ndarray
    obstacles: list[Obstacle]
    path_mm: list[tuple[float, float]]
    motions: list[Motion]
    grid_commands: str
    homography: np.ndarray


def load_config(path: str | Path) -> dict:
    with Path(path).open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    validate_config(config)
    return config


def validate_config(config: dict) -> None:
    cell_size = float(config["maze_cell_size_mm"])
    cell_count = int(config["course_cell_count"])
    course_size = float(config["course_size_mm"])
    if not math.isclose(cell_size * cell_count, course_size, abs_tol=0.01):
        raise ValueError("course_size_mm must equal maze_cell_size_mm multiplied by course_cell_count")
    full_count = int(config.get("full_maze_cell_count", 9))
    full_size = float(config.get("full_maze_size_mm", cell_size * full_count))
    if not math.isclose(cell_size * full_count, full_size, abs_tol=0.01):
        raise ValueError("full_maze_size_mm must equal maze_cell_size_mm multiplied by full_maze_cell_count")
    if (
        float(config["robot_length_mm"]) <= 0
        or float(config["robot_width_mm"]) <= 0
        or float(config["safety_margin_mm"]) < 0
    ):
        raise ValueError("Robot dimensions must be positive and safety margin cannot be negative")


def robot_footprint_radius(config: dict) -> float:
    """Return the rectangular chassis corner radius used during in-place turns."""
    half_length = float(config["robot_length_mm"]) / 2.0
    half_width = float(config["robot_width_mm"]) / 2.0
    return math.hypot(half_length, half_width)


def robot_travel_radius(config: dict) -> float:
    """Conservative half-width while the chassis travels without rotating."""
    return max(float(config["robot_length_mm"]), float(config["robot_width_mm"])) / 2.0


def validate_corners(corners: Sequence[Sequence[float]]) -> np.ndarray:
    points = np.asarray(corners, dtype=np.float32)
    if points.shape != (4, 2):
        raise ValueError("Exactly four course corners are required")
    if abs(cv2.contourArea(points)) < 1_000:
        raise ValueError("The selected course area is too small or self-intersecting")
    cross_signs = []
    for index in range(4):
        a = points[(index + 1) % 4] - points[index]
        b = points[(index + 2) % 4] - points[(index + 1) % 4]
        # A valid quadrilateral turns in the same direction at every corner.
        cross_signs.append(float(a[0] * b[1] - a[1] * b[0]))
    if not (all(value > 0 for value in cross_signs) or all(value < 0 for value in cross_signs)):
        raise ValueError("Click the four course corners in clockwise order")
    return points


def _fit_course_boundary_line(
    edge_pixels: np.ndarray,
    start: np.ndarray,
    end: np.ndarray,
    config: dict,
) -> np.ndarray | None:
    """Fit the reliable wall edge nearest a manually selected course side."""
    segment = end - start
    length = float(np.linalg.norm(segment))
    if length <= 0:
        return None
    direction = segment / length
    inward_normal = np.asarray((-direction[1], direction[0]), dtype=np.float64)
    relative = edge_pixels - start
    along = relative @ direction
    distance = relative @ inward_normal
    band = max(8.0, length * float(config.get("corner_snap_band_fraction", 0.06)))
    in_band = (
        (along >= length * 0.08)
        & (along <= length * 0.92)
        & (np.abs(distance) <= band)
    )
    candidates = edge_pixels[in_band]
    candidate_distances = distance[in_band]
    if len(candidates) == 0:
        return None

    radius = int(math.ceil(band))
    bin_edges = np.arange(-radius - 0.5, radius + 1.5, 1.0)
    counts, _ = np.histogram(candidate_distances, bins=bin_edges)
    smoothed = np.convolve(counts, np.ones(3, dtype=np.int32), mode="same")
    strongest = int(smoothed.max(initial=0))
    minimum_support = max(
        20,
        int(round(length * float(config.get("corner_snap_min_support_fraction", 0.12)))),
    )
    if strongest < minimum_support:
        return None

    peak_ratio = float(config.get("corner_snap_peak_ratio", 0.55))
    possible = np.flatnonzero(smoothed >= strongest * peak_ratio)
    if len(possible) == 0:
        return None
    bin_centres = (bin_edges[:-1] + bin_edges[1:]) / 2.0
    # A thick wall produces two strong edges. Follow the edge the operator
    # actually clicked instead of always jumping to the outer edge.
    peak_index = min(possible, key=lambda index: abs(bin_centres[index]))
    peak_offset = float(bin_centres[peak_index])
    support = candidates[np.abs(candidate_distances - peak_offset) <= 2.0]
    if len(support) < minimum_support:
        return None

    vx, vy, x0, y0 = cv2.fitLine(
        support.astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01
    ).flatten()
    fitted_direction = np.asarray((float(vx), float(vy)), dtype=np.float64)
    if float(fitted_direction @ direction) < 0:
        fitted_direction *= -1.0
    cosine = float(np.clip(fitted_direction @ direction, -1.0, 1.0))
    angle = math.degrees(math.acos(cosine))
    if angle > float(config.get("corner_snap_max_angle_deg", 6.0)):
        return None

    normal = np.asarray((-fitted_direction[1], fitted_direction[0]), dtype=np.float64)
    constant = -float(normal @ np.asarray((x0, y0), dtype=np.float64))
    return np.asarray((normal[0], normal[1], constant), dtype=np.float64)


def _intersect_lines(first: np.ndarray, second: np.ndarray) -> np.ndarray | None:
    coefficients = np.asarray(
        ((first[0], first[1]), (second[0], second[1])), dtype=np.float64
    )
    determinant = float(np.linalg.det(coefficients))
    if abs(determinant) < 0.05:
        return None
    return np.linalg.solve(coefficients, -np.asarray((first[2], second[2])))


def refine_course_corners(
    image: np.ndarray,
    corners: Sequence[Sequence[float]],
    config: dict,
) -> np.ndarray:
    """Snap approximate course corners to fitted outer wall-edge intersections."""
    selected = validate_corners(corners).astype(np.float64)
    if not bool(config.get("corner_snap_enabled", True)):
        return selected.astype(np.float32)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    rows, columns = np.nonzero(edges)
    if len(rows) == 0:
        return selected.astype(np.float32)
    edge_pixels = np.column_stack((columns, rows)).astype(np.float64)
    lines: list[np.ndarray] = []
    for index in range(4):
        line = _fit_course_boundary_line(
            edge_pixels,
            selected[index],
            selected[(index + 1) % 4],
            config,
        )
        if line is None:
            return selected.astype(np.float32)
        lines.append(line)

    intersections: list[np.ndarray] = []
    for index in range(4):
        point = _intersect_lines(lines[(index - 1) % 4], lines[index])
        if point is None:
            return selected.astype(np.float32)
        intersections.append(point)
    refined = np.asarray(intersections, dtype=np.float64)

    average_length = float(
        np.mean(
            [
                np.linalg.norm(selected[(index + 1) % 4] - selected[index])
                for index in range(4)
            ]
        )
    )
    maximum_adjustment = average_length * float(
        config.get("corner_snap_max_adjustment_fraction", 0.03)
    )
    if np.any(np.linalg.norm(refined - selected, axis=1) > maximum_adjustment):
        return selected.astype(np.float32)
    selected_area = abs(float(cv2.contourArea(selected.astype(np.float32))))
    refined_area = abs(float(cv2.contourArea(refined.astype(np.float32))))
    if selected_area <= 0 or not (0.8 <= refined_area / selected_area <= 1.2):
        return selected.astype(np.float32)
    try:
        return validate_corners(refined)
    except ValueError:
        return selected.astype(np.float32)


def rectify_course(
    image: np.ndarray,
    corners: Sequence[Sequence[float]],
    output_pixels: int,
) -> tuple[np.ndarray, np.ndarray]:
    source = validate_corners(corners)
    last = float(output_pixels - 1)
    destination = np.array([[0, 0], [last, 0], [last, last], [0, last]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(source, destination)
    rectified = cv2.warpPerspective(image, homography, (output_pixels, output_pixels))
    return rectified, homography


def transform_point(point: Sequence[float], homography: np.ndarray) -> tuple[float, float]:
    source = np.asarray([[point]], dtype=np.float32)
    transformed = cv2.perspectiveTransform(source, homography)[0, 0]
    return float(transformed[0]), float(transformed[1])


def pixels_to_mm(point: Sequence[float], output_pixels: int, course_size_mm: float) -> tuple[float, float]:
    scale = course_size_mm / float(output_pixels - 1)
    return float(point[0]) * scale, float(point[1]) * scale


def make_dark_mask(image: np.ndarray, config: dict) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    threshold = int(config.get("dark_threshold", 0))
    if threshold > 0:
        _, dark_mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)
    else:
        _, dark_mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, kernel_open)
    dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, kernel_close)
    return dark_mask


def _disk_dark_fraction(
    dark_mask: np.ndarray,
    center_x: float,
    center_y: float,
    radius: float,
) -> float:
    """Return the dark-pixel fraction inside a circular candidate."""
    sample_radius = max(2, int(round(radius * 0.72)))
    sample = np.zeros_like(dark_mask)
    cv2.circle(
        sample,
        (int(round(center_x)), int(round(center_y))),
        sample_radius,
        255,
        -1,
    )
    sample_area = cv2.countNonZero(sample)
    if sample_area == 0:
        return 0.0
    return cv2.countNonZero(cv2.bitwise_and(dark_mask, sample)) / sample_area


def _detect_hough_obstacles(
    rectified: np.ndarray,
    dark_mask: np.ndarray,
    config: dict,
    pixels_per_mm: float,
) -> list[Obstacle]:
    """Recover round cylinders merged with another cylinder or a nearby wall."""
    min_diameter = float(config["min_obstacle_diameter_mm"])
    max_diameter = float(config["max_obstacle_diameter_mm"])
    expected_radius = float(config["expected_obstacle_radius_mm"])
    border = float(config["border_rejection_mm"]) * pixels_per_mm
    radius_scale = float(config.get("hough_min_radius_scale", 0.45))
    distance_scale = float(config.get("hough_min_center_distance_scale", 0.8))
    circle_threshold = float(config.get("hough_circle_threshold", 30.0))
    min_dark_fraction = float(config.get("hough_min_dark_fraction", 0.72))

    gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 2.0)
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=max(8.0, min_diameter * pixels_per_mm * distance_scale),
        param1=100,
        param2=circle_threshold,
        minRadius=max(2, int(round(min_diameter * pixels_per_mm * radius_scale))),
        maxRadius=max(3, int(round(max_diameter * pixels_per_mm * 0.5))),
    )
    if circles is None:
        return []

    recovered: list[Obstacle] = []
    for center_x_px, center_y_px, radius_px in circles[0]:
        if (
            center_x_px < border
            or center_y_px < border
            or center_x_px > rectified.shape[1] - 1 - border
            or center_y_px > rectified.shape[0] - 1 - border
        ):
            continue
        if _disk_dark_fraction(
            dark_mask, float(center_x_px), float(center_y_px), float(radius_px)
        ) < min_dark_fraction:
            continue
        measured_radius = float(radius_px) / pixels_per_mm
        recovered.append(
            Obstacle(
                float(center_x_px) / pixels_per_mm,
                float(center_y_px) / pixels_per_mm,
                max(expected_radius, min(measured_radius, max_diameter / 2.0)),
                1.0,
            )
        )
    return recovered


def detect_obstacles(rectified: np.ndarray, config: dict) -> tuple[list[Obstacle], np.ndarray]:
    course_size = float(config["course_size_mm"])
    pixels_per_mm = (rectified.shape[0] - 1) / course_size
    dark_mask = make_dark_mask(rectified, config)

    min_diameter = float(config["min_obstacle_diameter_mm"])
    max_diameter = float(config["max_obstacle_diameter_mm"])
    expected_radius = float(config["expected_obstacle_radius_mm"])
    min_circularity = float(config["min_circularity"])
    max_aspect = float(config["max_obstacle_aspect_ratio"])
    min_solidity = float(config["min_solidity"])
    border = float(config["border_rejection_mm"]) * pixels_per_mm

    obstacles: list[Obstacle] = []
    contours, _ = cv2.findContours(dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area_px = float(cv2.contourArea(contour))
        perimeter = float(cv2.arcLength(contour, True))
        if area_px <= 0 or perimeter <= 0:
            continue
        equivalent_diameter_mm = 2.0 * math.sqrt(area_px / math.pi) / pixels_per_mm
        if not (min_diameter <= equivalent_diameter_mm <= max_diameter):
            continue
        (_, _), (width, height), _ = cv2.minAreaRect(contour)
        short_side = min(width, height)
        if short_side <= 0 or max(width, height) / short_side > max_aspect:
            continue
        circularity = 4.0 * math.pi * area_px / (perimeter * perimeter)
        if circularity < min_circularity:
            continue
        hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
        if hull_area <= 0 or area_px / hull_area < min_solidity:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        center_x_px = moments["m10"] / moments["m00"]
        center_y_px = moments["m01"] / moments["m00"]
        if (
            center_x_px < border
            or center_y_px < border
            or center_x_px > rectified.shape[1] - 1 - border
            or center_y_px > rectified.shape[0] - 1 - border
        ):
            continue
        x_mm = center_x_px / pixels_per_mm
        y_mm = center_y_px / pixels_per_mm
        measured_radius = equivalent_diameter_mm / 2.0
        radius_mm = max(expected_radius, min(measured_radius, max_diameter / 2.0))
        obstacles.append(Obstacle(x_mm, y_mm, radius_mm, circularity))

    duplicate_distance = expected_radius * 0.7
    for recovered in _detect_hough_obstacles(rectified, dark_mask, config, pixels_per_mm):
        if any(
            math.hypot(recovered.x_mm - obstacle.x_mm, recovered.y_mm - obstacle.y_mm)
            < duplicate_distance
            for obstacle in obstacles
        ):
            continue
        obstacles.append(recovered)

    obstacles.sort(key=lambda obstacle: (obstacle.y_mm, obstacle.x_mm))
    return obstacles, dark_mask


def nearest_edge(point: Sequence[float], course_size_mm: float) -> str:
    x, y = float(point[0]), float(point[1])
    distances = {"left": x, "right": course_size_mm - x, "top": y, "bottom": course_size_mm - y}
    return min(distances, key=distances.get)


def snap_to_edge(point: Sequence[float], course_size_mm: float) -> tuple[tuple[float, float], str]:
    edge = nearest_edge(point, course_size_mm)
    x = min(max(float(point[0]), 0.0), course_size_mm)
    y = min(max(float(point[1]), 0.0), course_size_mm)
    if edge == "left":
        x = 0.0
    elif edge == "right":
        x = course_size_mm
    elif edge == "top":
        y = 0.0
    else:
        y = course_size_mm
    return (x, y), edge


def inward_vector(edge: str) -> tuple[float, float]:
    return {
        "left": (1.0, 0.0),
        "right": (-1.0, 0.0),
        "top": (0.0, 1.0),
        "bottom": (0.0, -1.0),
    }[edge]


def inward_heading_deg(edge: str) -> float:
    dx, dy = inward_vector(edge)
    return math.degrees(math.atan2(-dy, dx))


def _draw_boundary_clearance(
    occupancy: np.ndarray,
    clearance_cells: int,
) -> None:
    occupancy[:clearance_cells, :] = True
    occupancy[-clearance_cells:, :] = True
    occupancy[:, :clearance_cells] = True
    occupancy[:, -clearance_cells:] = True


def _carve_opening(
    boundary_mask: np.ndarray,
    edge_point: tuple[float, float],
    edge: str,
    config: dict,
) -> None:
    resolution = float(config["grid_resolution_mm"])
    robot_radius = robot_footprint_radius(config)
    margin = float(config["safety_margin_mm"])
    half_width = max(resolution, float(config["opening_width_mm"]) / 2.0 - robot_radius - margin)
    depth = float(config["approach_distance_mm"]) + robot_radius + margin
    x = edge_point[0] / resolution
    y = edge_point[1] / resolution
    half = max(1, int(math.ceil(half_width / resolution)))
    depth_cells = max(1, int(math.ceil(depth / resolution)))
    cx, cy = int(round(x)), int(round(y))
    rows, cols = boundary_mask.shape
    if edge in ("left", "right"):
        y0, y1 = max(0, cy - half), min(rows, cy + half + 1)
        if edge == "left":
            boundary_mask[y0:y1, : min(cols, depth_cells + 1)] = False
        else:
            boundary_mask[y0:y1, max(0, cols - depth_cells - 1) :] = False
    else:
        x0, x1 = max(0, cx - half), min(cols, cx + half + 1)
        if edge == "top":
            boundary_mask[: min(rows, depth_cells + 1), x0:x1] = False
        else:
            boundary_mask[max(0, rows - depth_cells - 1) :, x0:x1] = False


def build_occupancy(
    obstacles: Sequence[Obstacle],
    entry_mm: Sequence[float],
    exit_mm: Sequence[float],
    config: dict,
    environment_mask: np.ndarray | None = None,
    validate_approaches: bool = True,
    robot_radius_mm: float | None = None,
) -> tuple[np.ndarray, tuple[float, float], tuple[float, float], str, str]:
    size = float(config["course_size_mm"])
    resolution = float(config["grid_resolution_mm"])
    cells = int(math.ceil(size / resolution)) + 1
    robot_radius = robot_footprint_radius(config) if robot_radius_mm is None else robot_radius_mm
    margin = float(config["safety_margin_mm"])
    clearance = robot_radius + margin
    clearance_cells = max(1, int(math.ceil(clearance / resolution)))

    entry, entry_edge = snap_to_edge(entry_mm, size)
    exit_point, exit_edge = snap_to_edge(exit_mm, size)
    if entry_edge == exit_edge and math.dist(entry, exit_point) < float(config["opening_width_mm"]):
        raise ValueError("Entrance and exit must be different openings")

    boundary = np.zeros((cells, cells), dtype=bool)
    _draw_boundary_clearance(boundary, clearance_cells)
    _carve_opening(boundary, entry, entry_edge, config)
    _carve_opening(boundary, exit_point, exit_edge, config)

    if environment_mask is not None:
        # Measure clearance from the original mask before resizing the map.
        free_distance_px = cv2.distanceTransform(
            (environment_mask == 0).astype(np.uint8), cv2.DIST_L2, 5
        )
        pixels_per_mm = (environment_mask.shape[1] - 1) / size
        # Limit the all-clear value before converting pixels to millimetres.
        max_distance_px = math.hypot(*environment_mask.shape[:2])
        clearance_mm = np.minimum(free_distance_px, max_distance_px) / pixels_per_mm
        resized_clearance = cv2.resize(
            clearance_mm,
            (cells, cells),
            interpolation=cv2.INTER_LINEAR,
        )
        obstacle_mask = resized_clearance < clearance
    else:
        obstacle_mask = np.zeros_like(boundary)
        yy, xx = np.indices(boundary.shape)
        x_mm = xx * resolution
        y_mm = yy * resolution
        for obstacle in obstacles:
            keepout = obstacle.radius_mm + clearance
            obstacle_mask |= (x_mm - obstacle.x_mm) ** 2 + (y_mm - obstacle.y_mm) ** 2 <= keepout**2

    occupancy = boundary | obstacle_mask
    entry_dx, entry_dy = inward_vector(entry_edge)
    exit_dx, exit_dy = inward_vector(exit_edge)
    approach = max(float(config["approach_distance_mm"]), clearance + resolution)
    start = (entry[0] + entry_dx * approach, entry[1] + entry_dy * approach)
    goal = (exit_point[0] + exit_dx * approach, exit_point[1] + exit_dy * approach)
    if validate_approaches:
        for point, name in ((start, "entrance approach"), (goal, "exit approach")):
            row, col = _point_to_cell(point, resolution, occupancy.shape)
            if occupancy[row, col]:
                raise ValueError(f"An obstacle blocks the {name}")
    return occupancy, start, goal, entry_edge, exit_edge


def _point_to_cell(
    point: Sequence[float], resolution: float, shape: tuple[int, int]
) -> tuple[int, int]:
    col = min(max(int(round(float(point[0]) / resolution)), 0), shape[1] - 1)
    row = min(max(int(round(float(point[1]) / resolution)), 0), shape[0] - 1)
    return row, col


def _cell_to_point(cell: tuple[int, int], resolution: float) -> tuple[float, float]:
    return cell[1] * resolution, cell[0] * resolution


def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def astar(
    occupancy: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]]:
    if occupancy[start] or occupancy[goal]:
        raise ValueError("Route endpoint lies inside an occupied area")
    directions = ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))
    queue: list[tuple[float, int, tuple[int, int]]] = []
    counter = 0
    heapq.heappush(queue, (_heuristic(start, goal), counter, start))
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {start: 0.0}
    closed: set[tuple[int, int]] = set()
    rows, cols = occupancy.shape
    while queue:
        _, _, current = heapq.heappop(queue)
        if current in closed:
            continue
        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parent[path[-1]])
            path.reverse()
            return path
        closed.add(current)
        for dr, dc in directions:
            neighbour = current[0] + dr, current[1] + dc
            if not (0 <= neighbour[0] < rows and 0 <= neighbour[1] < cols):
                continue
            if occupancy[neighbour]:
                continue
            if dr and dc and (occupancy[current[0] + dr, current[1]] or occupancy[current[0], current[1] + dc]):
                continue
            new_cost = cost[current] + math.hypot(dr, dc)
            if new_cost >= cost.get(neighbour, math.inf):
                continue
            cost[neighbour] = new_cost
            parent[neighbour] = current
            counter += 1
            priority = new_cost + _heuristic(neighbour, goal)
            heapq.heappush(queue, (priority, counter, neighbour))
    raise ValueError("No collision-free route exists between the selected entrance and exit")


def directional_astar(
    occupancy: np.ndarray,
    turn_occupancy: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    initial_direction: tuple[int, int],
    goal_direction: tuple[int, int] | None,
) -> list[tuple[int, int]]:
    """Plan an eight-direction path, turning only where the chassis can rotate."""
    if occupancy[start] or occupancy[goal]:
        raise ValueError("The selected entrance or exit centre is blocked")

    directions = (
        (-1, 0),
        (-1, 1),
        (0, 1),
        (1, 1),
        (1, 0),
        (1, -1),
        (0, -1),
        (-1, -1),
    )
    start_state = (*start, *initial_direction)
    queue: list[tuple[float, int, tuple[int, int, int, int]]] = []
    serial = 0
    heapq.heappush(queue, (_heuristic(start, goal), serial, start_state))
    parent: dict[tuple[int, int, int, int], tuple[int, int, int, int]] = {}
    cost = {start_state: 0.0}
    closed: set[tuple[int, int, int, int]] = set()
    rows, cols = occupancy.shape

    while queue:
        _, _, current = heapq.heappop(queue)
        if current in closed:
            continue
        row, col, current_dr, current_dc = current
        if (row, col) == goal and (
            goal_direction is None or (current_dr, current_dc) == goal_direction
        ):
            states = [current]
            while states[-1] != start_state:
                states.append(parent[states[-1]])
            states.reverse()
            return [(state[0], state[1]) for state in states]
        closed.add(current)

        for dr, dc in directions:
            neighbour = row + dr, col + dc
            if not (0 <= neighbour[0] < rows and 0 <= neighbour[1] < cols):
                continue
            if occupancy[neighbour]:
                continue
            if dr and dc and (occupancy[row + dr, col] or occupancy[row, col + dc]):
                continue
            is_turn = (dr, dc) != (current_dr, current_dc)
            if is_turn and turn_occupancy[row, col]:
                continue
            neighbour_state = (*neighbour, dr, dc)
            step_cost = math.hypot(dr, dc) + (0.35 if is_turn else 0.0)
            new_cost = cost[current] + step_cost
            if new_cost >= cost.get(neighbour_state, math.inf):
                continue
            cost[neighbour_state] = new_cost
            parent[neighbour_state] = current
            serial += 1
            priority = new_cost + _heuristic(neighbour, goal)
            heapq.heappush(queue, (priority, serial, neighbour_state))

    raise ValueError(
        "No collision-free diagonal route exists between the selected entrance and exit"
    )


def compress_grid_path(
    path: Sequence[tuple[int, int]], resolution: float, origin_mm: float = 0.0
) -> list[tuple[float, float]]:
    """Keep endpoints and direction changes, then convert grid cells to millimetres."""
    if len(path) <= 2:
        return [
            (cell[1] * resolution + origin_mm, cell[0] * resolution + origin_mm)
            for cell in path
        ]
    result = [path[0]]
    previous_direction = (path[1][0] - path[0][0], path[1][1] - path[0][1])
    for index in range(1, len(path) - 1):
        next_direction = (
            path[index + 1][0] - path[index][0],
            path[index + 1][1] - path[index][1],
        )
        if next_direction != previous_direction:
            result.append(path[index])
        previous_direction = next_direction
    result.append(path[-1])
    return [
        (cell[1] * resolution + origin_mm, cell[0] * resolution + origin_mm)
        for cell in result
    ]


def extend_occupancy_for_opening_cells(
    occupancy: np.ndarray,
    entry_cell: tuple[int, int],
    exit_cell: tuple[int, int],
    config: dict,
    robot_radius_mm: float,
) -> tuple[np.ndarray, float]:
    """Add one blocked outer ring and open only the selected outer cells."""
    resolution = float(config["grid_resolution_mm"])
    cell_size = float(config["maze_cell_size_mm"])
    margin = float(config["safety_margin_mm"])
    padding_cells = int(round(cell_size / resolution))
    extended = np.pad(
        occupancy,
        ((padding_cells, padding_cells), (padding_cells, padding_cells)),
        constant_values=True,
    )
    count = int(config["course_cell_count"])
    clearance_cells = int(math.ceil((robot_radius_mm + margin) / resolution))

    for row, col in (entry_cell, exit_cell):
        if row == -1:
            row_start, row_end = clearance_cells, padding_cells + 1
            center_col = padding_cells + int(round((col + 0.5) * cell_size / resolution))
            col_start, col_end = center_col - padding_cells + clearance_cells, center_col + padding_cells - clearance_cells + 1
        elif row == count:
            row_start = padding_cells + occupancy.shape[0] - 1
            row_end = extended.shape[0] - clearance_cells
            center_col = padding_cells + int(round((col + 0.5) * cell_size / resolution))
            col_start, col_end = center_col - padding_cells + clearance_cells, center_col + padding_cells - clearance_cells + 1
        elif col == -1:
            col_start, col_end = clearance_cells, padding_cells + 1
            center_row = padding_cells + int(round((row + 0.5) * cell_size / resolution))
            row_start, row_end = center_row - padding_cells + clearance_cells, center_row + padding_cells - clearance_cells + 1
        else:
            col_start = padding_cells + occupancy.shape[1] - 1
            col_end = extended.shape[1] - clearance_cells
            center_row = padding_cells + int(round((row + 0.5) * cell_size / resolution))
            row_start, row_end = center_row - padding_cells + clearance_cells, center_row + padding_cells - clearance_cells + 1
        extended[row_start:row_end, col_start:col_end] = False

    return extended, -cell_size


def line_is_free(a: Sequence[float], b: Sequence[float], occupancy: np.ndarray, resolution: float) -> bool:
    distance = math.dist(a, b)
    samples = max(2, int(math.ceil(distance / (resolution * 0.5))) + 1)
    for t in np.linspace(0.0, 1.0, samples):
        point = (float(a[0]) + (float(b[0]) - float(a[0])) * t, float(a[1]) + (float(b[1]) - float(a[1])) * t)
        row, col = _point_to_cell(point, resolution, occupancy.shape)
        if occupancy[row, col]:
            return False
    return True


def simplify_path(
    path: Sequence[tuple[float, float]], occupancy: np.ndarray, resolution: float
) -> list[tuple[float, float]]:
    if len(path) <= 2:
        return list(path)
    result = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        candidate = len(path) - 1
        while candidate > anchor + 1 and not line_is_free(path[anchor], path[candidate], occupancy, resolution):
            candidate -= 1
        result.append(path[candidate])
        anchor = candidate
    return result


def wrap_angle(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def path_to_motions(path: Sequence[Sequence[float]], initial_heading_deg: float) -> list[Motion]:
    heading = initial_heading_deg
    motions: list[Motion] = []
    for start, end in zip(path, path[1:]):
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        distance = math.hypot(dx, dy)
        if distance < 1.0:
            continue
        segment_heading = math.degrees(math.atan2(-dy, dx))
        motions.append(Motion(wrap_angle(segment_heading - heading), distance))
        heading = segment_heading
    return motions


def opening_to_grid_cell(
    opening_mm: Sequence[float], edge: str, config: dict
) -> tuple[int, int]:
    count = int(config["course_cell_count"])
    cell_size = float(config["maze_cell_size_mm"])
    x_index = min(max(int(float(opening_mm[0]) // cell_size), 0), count - 1)
    y_index = min(max(int(float(opening_mm[1]) // cell_size), 0), count - 1)
    if edge == "top":
        return 0, x_index
    if edge == "bottom":
        return count - 1, x_index
    if edge == "left":
        return y_index, 0
    return y_index, count - 1


def grid_cell_center(cell: tuple[int, int], config: dict) -> tuple[float, float]:
    cell_size = float(config["maze_cell_size_mm"])
    return (cell[1] + 0.5) * cell_size, (cell[0] + 0.5) * cell_size


def selected_outside_cell(
    point_mm: Sequence[float], config: dict, label: str
) -> tuple[tuple[int, int], tuple[int, int], str]:
    """Snap a click to the one-cell-wide ring immediately outside the course."""
    count = int(config["course_cell_count"])
    cell_size = float(config["maze_cell_size_mm"])
    size = float(config["course_size_mm"])
    x, y = float(point_mm[0]), float(point_mm[1])
    distances = {"left": x, "right": size - x, "top": y, "bottom": size - y}
    edge = min(distances, key=distances.get)
    perpendicular_distance = distances[edge]
    inside_tolerance = float(config.get("selection_inside_tolerance_cells", 0.55)) * cell_size
    outside_limit = float(config.get("selection_outside_limit_cells", 1.75)) * cell_size
    if perpendicular_distance > inside_tolerance or perpendicular_distance < -outside_limit:
        raise ValueError(
            f"{label} must be clicked near a grid cell immediately outside the purple 5 x 5 area"
        )

    along = y if edge in ("left", "right") else x
    along_tolerance = inside_tolerance
    if along < -along_tolerance or along > size + along_tolerance:
        raise ValueError(f"{label} is too far past a corner of the purple area")
    index = min(max(math.floor(along / cell_size), 0), count - 1)
    if edge == "top":
        return (-1, index), (0, index), edge
    if edge == "bottom":
        return (count, index), (count - 1, index), edge
    if edge == "left":
        return (index, -1), (index, 0), edge
    return (index, count), (index, count - 1), edge


def boundary_point_for_cell(
    inside_cell: tuple[int, int], edge: str, config: dict
) -> tuple[float, float]:
    cell_size = float(config["maze_cell_size_mm"])
    size = float(config["course_size_mm"])
    x, y = grid_cell_center(inside_cell, config)
    if edge == "top":
        return x, 0.0
    if edge == "bottom":
        return x, size
    if edge == "left":
        return 0.0, y
    return size, y


def build_extended_clearance_map(
    original: np.ndarray,
    homography: np.ndarray,
    config: dict,
    obstacles: Sequence[Obstacle] = (),
) -> tuple[np.ndarray, float, float]:
    """Measure wall clearance over the 5 x 5 area plus one outside cell."""
    output_pixels = int(config["rectified_pixels"])
    course_size = float(config["course_size_mm"])
    cell_size = float(config["maze_cell_size_mm"])
    pixels_per_mm = (output_pixels - 1) / course_size
    padding_px = int(round(cell_size * pixels_per_mm))
    extended_pixels = output_pixels + 2 * padding_px
    translation = np.asarray(
        [[1.0, 0.0, padding_px], [0.0, 1.0, padding_px], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    extended_homography = translation @ homography
    extended = cv2.warpPerspective(
        original,
        extended_homography,
        (extended_pixels, extended_pixels),
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(255, 255, 255),
    )
    dark_mask = make_dark_mask(extended, config)
    for obstacle in obstacles:
        center = (
            int(round(obstacle.x_mm * pixels_per_mm + padding_px)),
            int(round(obstacle.y_mm * pixels_per_mm + padding_px)),
        )
        radius = int(round((obstacle.radius_mm + 8.0) * pixels_per_mm))
        cv2.circle(dark_mask, center, radius, 0, -1)

    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(dark_mask)
    wall_mask = np.zeros_like(dark_mask)
    minimum_span_px = 0.4 * cell_size * pixels_per_mm
    for label in range(1, component_count):
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        if max(width, height) >= minimum_span_px:
            wall_mask[labels == label] = 255
    free_distance_px = cv2.distanceTransform(
        (wall_mask == 0).astype(np.uint8), cv2.DIST_L2, 5
    )
    max_distance_px = math.hypot(*dark_mask.shape[:2])
    clearance_mm = np.minimum(free_distance_px, max_distance_px) / pixels_per_mm
    return clearance_mm, float(padding_px), pixels_per_mm


def extended_line_is_free(
    a: Sequence[float],
    b: Sequence[float],
    clearance_mm: np.ndarray,
    padding_px: float,
    pixels_per_mm: float,
    required_clearance_mm: float,
) -> bool:
    distance = math.dist(a, b)
    samples = max(2, int(math.ceil(distance / 2.5)) + 1)
    height, width = clearance_mm.shape
    for t in np.linspace(0.0, 1.0, samples):
        x_mm = float(a[0]) + (float(b[0]) - float(a[0])) * t
        y_mm = float(a[1]) + (float(b[1]) - float(a[1])) * t
        col = int(round(x_mm * pixels_per_mm + padding_px))
        row = int(round(y_mm * pixels_per_mm + padding_px))
        if not (0 <= row < height and 0 <= col < width):
            return False
        if float(clearance_mm[row, col]) < required_clearance_mm:
            return False
    return True


def grid_astar(
    occupancy: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    config: dict,
    turn_occupancy: np.ndarray | None = None,
    initial_heading_deg: float = 0.0,
    goal_direction: tuple[int, int] | None = None,
) -> list[tuple[int, int]]:
    count = int(config["course_cell_count"])
    resolution = float(config["grid_resolution_mm"])

    def cell_is_free(cell: tuple[int, int]) -> bool:
        point = grid_cell_center(cell, config)
        row, col = _point_to_cell(point, resolution, occupancy.shape)
        return not occupancy[row, col]

    if not cell_is_free(start) or not cell_is_free(goal):
        raise ValueError("The selected entrance or exit cell is blocked")

    heading_direction = {
        90: (-1, 0),
        0: (0, 1),
        -90: (1, 0),
        180: (0, -1),
        -180: (0, -1),
    }[int(round(wrap_angle(initial_heading_deg)))]
    start_state = (start[0], start[1], heading_direction[0], heading_direction[1])
    queue: list[tuple[int, int, tuple[int, int, int, int]]] = []
    serial = 0
    heapq.heappush(
        queue,
        ((abs(start[0] - goal[0]) + abs(start[1] - goal[1])) * 10, serial, start_state),
    )
    parent: dict[tuple[int, int, int, int], tuple[int, int, int, int]] = {}
    cost = {start_state: 0}
    closed: set[tuple[int, int, int, int]] = set()
    for_pop = ((-1, 0), (0, 1), (1, 0), (0, -1))
    while queue:
        _, _, current = heapq.heappop(queue)
        if current in closed:
            continue
        current_cell = current[0], current[1]
        current_direction = current[2], current[3]
        if current_cell == goal:
            final_turn_is_safe = True
            if goal_direction is not None and current_direction != goal_direction:
                if turn_occupancy is not None:
                    turn_cell = _point_to_cell(
                        grid_cell_center(current_cell, config),
                        resolution,
                        turn_occupancy.shape,
                    )
                    final_turn_is_safe = not turn_occupancy[turn_cell]
            if final_turn_is_safe:
                states = [current]
                while states[-1] != start_state:
                    states.append(parent[states[-1]])
                states.reverse()
                return [(state[0], state[1]) for state in states]
        closed.add(current)
        current_point = grid_cell_center(current_cell, config)
        for dr, dc in for_pop:
            neighbour = current_cell[0] + dr, current_cell[1] + dc
            if not (0 <= neighbour[0] < count and 0 <= neighbour[1] < count):
                continue
            if not cell_is_free(neighbour):
                continue
            neighbour_point = grid_cell_center(neighbour, config)
            if not line_is_free(current_point, neighbour_point, occupancy, resolution):
                continue
            is_turn = (dr, dc) != current_direction
            if is_turn and turn_occupancy is not None:
                turn_cell = _point_to_cell(current_point, resolution, turn_occupancy.shape)
                if turn_occupancy[turn_cell]:
                    continue
            neighbour_state = (neighbour[0], neighbour[1], dr, dc)
            new_cost = cost[current] + 10 + (2 if is_turn else 0)
            if new_cost >= cost.get(neighbour_state, math.inf):
                continue
            cost[neighbour_state] = new_cost
            parent[neighbour_state] = current
            serial += 1
            heuristic = abs(neighbour[0] - goal[0]) + abs(neighbour[1] - goal[1])
            heapq.heappush(queue, (new_cost + heuristic * 10, serial, neighbour_state))
    raise ValueError("No one-cell grid route exists between the selected entrance and exit")


def grid_path_to_commands(
    path: Sequence[tuple[int, int]], initial_heading_deg: float
) -> str:
    heading = initial_heading_deg
    commands: list[str] = []
    for start, end in zip(path, path[1:]):
        dr = end[0] - start[0]
        dc = end[1] - start[1]
        if abs(dr) + abs(dc) != 1:
            raise ValueError("Grid path contains a movement longer than one cell")
        target_heading = {
            (-1, 0): 90.0,
            (1, 0): -90.0,
            (0, 1): 0.0,
            (0, -1): 180.0,
        }[(dr, dc)]
        turn = wrap_angle(target_heading - heading)
        if math.isclose(turn, 90.0, abs_tol=0.1):
            commands.append("l")
        elif math.isclose(turn, -90.0, abs_tol=0.1):
            commands.append("r")
        elif math.isclose(abs(turn), 180.0, abs_tol=0.1):
            commands.extend(("l", "l"))
        elif not math.isclose(turn, 0.0, abs_tol=0.1):
            raise ValueError("Grid route produced a non-90-degree turn")
        commands.append("f")
        heading = target_heading
    return "".join(commands)


def _plan_course_with_fixed_corners(
    image: np.ndarray,
    corners: Sequence[Sequence[float]],
    entrance: Sequence[float],
    exit_point: Sequence[float],
    config: dict,
) -> PlanResult:
    validate_config(config)
    output_pixels = int(config["rectified_pixels"])
    size = float(config["course_size_mm"])
    rectified, homography = rectify_course(image, corners, output_pixels)
    start_px = transform_point(entrance, homography)
    goal_px = transform_point(exit_point, homography)
    start_click_mm = pixels_to_mm(start_px, output_pixels, size)
    goal_click_mm = pixels_to_mm(goal_px, output_pixels, size)
    start_outside, start_grid, entry_edge = selected_outside_cell(
        start_click_mm, config, "Start"
    )
    goal_outside, goal_grid, exit_edge = selected_outside_cell(
        goal_click_mm, config, "Goal"
    )
    if start_outside == goal_outside:
        raise ValueError("Start and goal must be different grid cells")
    entry_mm = boundary_point_for_cell(start_grid, entry_edge, config)
    exit_mm = boundary_point_for_cell(goal_grid, exit_edge, config)
    obstacles, dark_mask = detect_obstacles(rectified, config)
    occupancy, _, _, entry_edge, exit_edge = build_occupancy(
        obstacles,
        entry_mm,
        exit_mm,
        config,
        None,
        validate_approaches=False,
        robot_radius_mm=robot_travel_radius(config),
    )
    turn_occupancy, _, _, _, _ = build_occupancy(
        obstacles,
        entry_mm,
        exit_mm,
        config,
        None,
        validate_approaches=False,
        robot_radius_mm=robot_footprint_radius(config),
    )
    initial_heading = inward_heading_deg(entry_edge)
    start_center = grid_cell_center(start_grid, config)
    goal_center = grid_cell_center(goal_grid, config)
    start_outside_center = grid_cell_center(start_outside, config)
    goal_outside_center = grid_cell_center(goal_outside, config)
    extended_clearance, padding_px, pixels_per_mm = build_extended_clearance_map(
        image, homography, config, obstacles
    )
    travel_clearance = robot_travel_radius(config) + float(config["safety_margin_mm"])
    if not extended_line_is_free(
        start_outside_center,
        start_center,
        extended_clearance,
        padding_px,
        pixels_per_mm,
        travel_clearance,
    ):
        raise ValueError("A black wall blocks the selected start cell from the 5 x 5 area")
    if not extended_line_is_free(
        goal_center,
        goal_outside_center,
        extended_clearance,
        padding_px,
        pixels_per_mm,
        travel_clearance,
    ):
        raise ValueError("A black wall blocks the selected goal cell from the 5 x 5 area")
    resolution = float(config["grid_resolution_mm"])
    route_occupancy, route_origin = extend_occupancy_for_opening_cells(
        occupancy,
        start_outside,
        goal_outside,
        config,
        robot_travel_radius(config),
    )
    route_turn_occupancy, _ = extend_occupancy_for_opening_cells(
        turn_occupancy,
        start_outside,
        goal_outside,
        config,
        robot_footprint_radius(config),
    )
    initial_direction = {
        "left": (0, 1),
        "right": (0, -1),
        "top": (1, 0),
        "bottom": (-1, 0),
    }[entry_edge]
    goal_direction = {
        "left": (0, -1),
        "right": (0, 1),
        "top": (-1, 0),
        "bottom": (1, 0),
    }[exit_edge]
    fine_path = directional_astar(
        route_occupancy,
        route_turn_occupancy,
        _point_to_cell(
            (start_outside_center[0] - route_origin, start_outside_center[1] - route_origin),
            resolution,
            route_occupancy.shape,
        ),
        _point_to_cell(
            (goal_outside_center[0] - route_origin, goal_outside_center[1] - route_origin),
            resolution,
            route_occupancy.shape,
        ),
        initial_direction,
        None,
    )
    path_mm = compress_grid_path(fine_path, resolution, route_origin)
    motions = path_to_motions(path_mm, initial_heading)
    final_heading = math.degrees(math.atan2(-goal_direction[0], goal_direction[1]))
    if motions:
        last_segment = path_mm[-2], path_mm[-1]
        last_heading = math.degrees(
            math.atan2(
                -(last_segment[1][1] - last_segment[0][1]),
                last_segment[1][0] - last_segment[0][0],
            )
        )
        final_turn = wrap_angle(final_heading - last_heading)
        if not math.isclose(final_turn, 0.0, abs_tol=0.1):
            motions.append(Motion(final_turn, 0.0))
    return PlanResult(
        rectified, dark_mask, occupancy, obstacles, path_mm, motions, "", homography
    )


def plan_course(
    image: np.ndarray,
    corners: Sequence[Sequence[float]],
    entrance: Sequence[float],
    exit_point: Sequence[float],
    config: dict,
) -> PlanResult:
    """Plan with snapped corners, then safely retry the original clicks if needed."""
    selected_corners = validate_corners(corners)
    refined_corners = refine_course_corners(image, selected_corners, config)
    if np.allclose(refined_corners, selected_corners, atol=0.1):
        return _plan_course_with_fixed_corners(
            image, selected_corners, entrance, exit_point, config
        )
    try:
        return _plan_course_with_fixed_corners(
            image, refined_corners, entrance, exit_point, config
        )
    except ValueError:
        # The raw selection must pass every normal collision and clearance check;
        # this fallback never weakens the physical safety constraints.
        return _plan_course_with_fixed_corners(
            image, selected_corners, entrance, exit_point, config
        )


def render_plan(result: PlanResult, config: dict) -> np.ndarray:
    canvas = result.rectified.copy()
    pixels_per_mm = (canvas.shape[0] - 1) / float(config["course_size_mm"])
    clearance = robot_footprint_radius(config) + float(config["safety_margin_mm"])
    for obstacle in result.obstacles:
        center = (int(round(obstacle.x_mm * pixels_per_mm)), int(round(obstacle.y_mm * pixels_per_mm)))
        safety_radius = int(round((obstacle.radius_mm + clearance) * pixels_per_mm))
        obstacle_radius = int(round(obstacle.radius_mm * pixels_per_mm))
        overlay = canvas.copy()
        cv2.circle(overlay, center, safety_radius, (0, 165, 255), -1)
        canvas = cv2.addWeighted(overlay, 0.25, canvas, 0.75, 0)
        cv2.circle(canvas, center, obstacle_radius, (0, 0, 255), 3)
    points = [
        (int(round(x * pixels_per_mm)), int(round(y * pixels_per_mm)))
        for x, y in result.path_mm
    ]
    if len(points) >= 2:
        cv2.polylines(canvas, [np.asarray(points, dtype=np.int32)], False, (255, 0, 255), 5, cv2.LINE_AA)
    for point in points[1:-1]:
        cv2.circle(canvas, point, 6, (255, 0, 255), -1)
    cv2.circle(canvas, points[0], 10, (0, 200, 0), -1)
    cv2.circle(canvas, points[-1], 10, (0, 0, 255), -1)
    return canvas


def render_plan_on_original(
    original: np.ndarray, result: PlanResult, config: dict
) -> np.ndarray:
    annotated_region = render_plan(result, config)
    height, width = original.shape[:2]
    inverse = np.linalg.inv(result.homography)
    warped_region = cv2.warpPerspective(annotated_region, inverse, (width, height))
    region_mask = cv2.warpPerspective(
        np.full(result.rectified.shape[:2], 255, dtype=np.uint8),
        inverse,
        (width, height),
    )
    output = original.copy()
    output[region_mask > 0] = warped_region[region_mask > 0]

    # Draw the two outside cells after mapping the route back to the photo.
    pixels_per_mm = (result.rectified.shape[0] - 1) / float(config["course_size_mm"])
    rectified_points = np.asarray(
        [[[x * pixels_per_mm, y * pixels_per_mm] for x, y in result.path_mm]],
        dtype=np.float32,
    )
    original_points = cv2.perspectiveTransform(rectified_points, inverse)[0]
    route_points = np.rint(original_points).astype(np.int32)
    if len(route_points) >= 2:
        cv2.polylines(output, [route_points], False, (255, 0, 255), 5, cv2.LINE_AA)
    for point in route_points[1:-1]:
        cv2.circle(output, tuple(point), 6, (255, 0, 255), -1)
    cv2.circle(output, tuple(route_points[0]), 10, (0, 200, 0), -1)
    cv2.circle(output, tuple(route_points[-1]), 10, (0, 0, 255), -1)
    return output


def render_occupancy(result: PlanResult, config: dict) -> np.ndarray:
    image = np.where(result.occupancy, 0, 255).astype(np.uint8)
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    pixels_per_mm = (image.shape[0] - 1) / float(config["course_size_mm"])
    if len(result.path_mm) >= 2:
        points = np.asarray(
            [(int(round(x * pixels_per_mm)), int(round(y * pixels_per_mm))) for x, y in result.path_mm],
            dtype=np.int32,
        )
        cv2.polylines(image, [points], False, (255, 0, 255), 2, cv2.LINE_AA)
    return image


def save_route_json(
    path: str | Path,
    result: PlanResult,
    pre_commands: str,
    post_commands: str,
    extra: dict | None = None,
) -> None:
    payload = {
        "pre_commands": pre_commands,
        "post_commands": post_commands,
        "obstacles": [asdict(obstacle) for obstacle in result.obstacles],
        "path_mm": [[round(x, 2), round(y, 2)] for x, y in result.path_mm],
        "course_motions": [
            {
                "turn_deg": round(motion.turn_deg, 2),
                "distance_mm": round(motion.distance_mm, 2),
            }
            for motion in result.motions
        ],
        "grid_commands": result.grid_commands,
    }
    if extra:
        payload.update(extra)
    Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _validate_commands(commands: str) -> str:
    value = commands.lower().strip()
    if any(command not in "flr" for command in value):
        raise ValueError("Standard-maze commands may only contain f, l, and r")
    return value


def save_arduino_header(path: str | Path, result: PlanResult, pre_commands: str, post_commands: str) -> None:
    pre = _validate_commands(pre_commands)
    post = _validate_commands(post_commands)
    course_motions = [
        f"    {{{motion.turn_deg:.2f}f, {motion.distance_mm:.2f}f}}"
        for motion in result.motions
    ]
    lines = [
        "/*",
        " * Generated by the Week 12 Part 2 route planner.",
        " */",
        "#pragma once",
        "",
        "struct CourseMotion {",
        "    float turnDeg;",
        "    float distanceMm;",
        "};",
        "",
        f'const char PRE_COMMANDS[] = "{pre}";',
        f'const char POST_COMMANDS[] = "{post}";',
        "const CourseMotion COURSE_MOTIONS[] = {",
        ",\n".join(course_motions),
        "};",
        f"const unsigned int COURSE_MOTION_COUNT = {len(course_motions)};",
        "",
    ]
    Path(path).write_text("\n".join(lines), encoding="utf-8")


def imwrite_unicode(path: str | Path, image: np.ndarray) -> None:
    output = Path(path)
    extension = output.suffix or ".png"
    success, encoded = cv2.imencode(extension, image)
    if not success:
        raise ValueError(f"Could not encode image as {extension}")
    encoded.tofile(str(output))


def imread_unicode(path: str | Path) -> np.ndarray:
    data = np.fromfile(str(path), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Could not read image: {path}")
    return image

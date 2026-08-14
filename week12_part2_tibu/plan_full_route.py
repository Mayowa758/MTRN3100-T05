from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

from full_maze import (
    detect_full_maze_corners,
    plan_full_maze,
    render_full_maze_debug,
    render_full_route_on_original,
)
from planner_core import (
    imread_unicode,
    imwrite_unicode,
    load_config,
    plan_course,
    render_occupancy,
    render_plan_on_original,
    save_arduino_header,
    save_route_json,
)
from workflow_ui import (
    ask_full_maze_positions,
    clear_generated_route,
    load_course_selection,
)


SCRIPT_DIR = Path(__file__).resolve().parent


def window_is_open(name: str) -> bool:
    try:
        return cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:
        return False


def _parse_maze_start(value: str):
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 3:
        raise ValueError("--maze-start must use row,column,direction")
    start = int(parts[0]), int(parts[1]), parts[2].upper()
    if (
        not 0 <= start[0] <= 8
        or not 0 <= start[1] <= 8
        or len(start[2]) != 1
        or start[2] not in "NESW"
    ):
        raise ValueError("Invalid full-maze start position")
    return start


def _parse_maze_goal(value: str):
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 2:
        raise ValueError("--maze-goal must use row,column")
    goal = int(parts[0]), int(parts[1])
    if not 0 <= goal[0] <= 8 or not 0 <= goal[1] <= 8:
        raise ValueError("Invalid full-maze goal position")
    return goal


def show_full_result(
    planning_image,
    obstacle_count: int,
    pre_commands: str,
    course_motion_count: int,
    post_commands: str,
) -> None:
    from tkinter import Tk, messagebox

    window = "Second 2 marks result - press any key to close"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.imshow(window, planning_image)
    while True:
        key = cv2.waitKey(20) & 0xFF
        if key != 255 or not window_is_open(window):
            break
    cv2.destroyAllWindows()
    root = Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    messagebox.showinfo(
        "Second 2 marks planning complete",
        f"Detected obstacles: {obstacle_count}\n"
        f"Before course: {pre_commands}\n"
        f"Obstacle course motions: {course_motion_count}\n"
        f"After course: {post_commands}\n\n"
        "The obstacle course uses generated angle-and-distance motions.",
        parent=root,
    )
    root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plan the second 2 marks using the saved obstacle-course selection"
    )
    parser.add_argument("--config", type=Path, default=SCRIPT_DIR / "config.json")
    parser.add_argument("--output", type=Path, default=SCRIPT_DIR / "output")
    parser.add_argument("--selection", type=Path)
    parser.add_argument("--maze-start", help="zero-indexed row,column,direction")
    parser.add_argument("--maze-goal", help="zero-indexed row,column")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    selection_path = args.selection or args.output / "course_selection.json"
    image_path, course_corners, entrance, exit_point = load_course_selection(selection_path)
    image = imread_unicode(image_path)
    config = load_config(args.config)
    full_corners = detect_full_maze_corners(image, config)

    gui_launch = args.maze_start is None and args.maze_goal is None
    if gui_launch:
        maze_start, maze_goal = ask_full_maze_positions()
    elif args.maze_start and args.maze_goal:
        maze_start = _parse_maze_start(args.maze_start)
        maze_goal = _parse_maze_goal(args.maze_goal)
    else:
        raise ValueError("Both full-maze start and goal must be supplied")

    args.output.mkdir(parents=True, exist_ok=True)
    arduino_header = SCRIPT_DIR / "arduino" / "week12_part2" / "GeneratedRoute.h"
    try:
        course_result = plan_course(
            image, course_corners, entrance, exit_point, config
        )
        if not course_result.obstacles:
            raise RuntimeError("No cylindrical obstacles were detected")
        full_result = plan_full_maze(
            image,
            full_corners,
            course_corners,
            entrance,
            exit_point,
            maze_start,
            maze_goal,
            config,
        )
    except (ValueError, RuntimeError) as error:
        clear_generated_route(arduino_header)
        raise type(error)(f"{error}\n\nThe old Arduino route was cleared for safety.") from error

    pre_commands = full_result.pre_route.commands
    post_commands = full_result.post_route.commands
    course_image = render_plan_on_original(image, course_result, config)
    planning_image = render_full_route_on_original(
        course_image, full_result, maze_start, maze_goal, config
    )
    imwrite_unicode(args.output / "planning_result.png", planning_image)
    imwrite_unicode(args.output / "occupancy_map.png", render_occupancy(course_result, config))
    imwrite_unicode(args.output / "full_maze_map.png", render_full_maze_debug(full_result, config))
    save_route_json(
        args.output / "route.json",
        course_result,
        pre_commands,
        post_commands,
        extra={
            "auto_full_maze_corners": full_corners.tolist(),
            "maze_start": list(maze_start),
            "maze_goal": list(maze_goal),
            "course_top_left_cell": [full_result.placement.top, full_result.placement.left],
            "course_entrance_cell": list(full_result.placement.entrance_cell),
            "course_exit_cell": list(full_result.placement.exit_cell),
            "pre_path_cells": [list(cell) for cell in full_result.pre_route.cells],
            "post_path_cells": [list(cell) for cell in full_result.post_route.cells],
            "complete_standard_commands": pre_commands + post_commands,
        },
    )
    save_arduino_header(
        arduino_header, course_result, pre_commands, post_commands
    )
    if gui_launch:
        show_full_result(
            planning_image,
            len(course_result.obstacles),
            pre_commands,
            len(course_result.motions),
            post_commands,
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError) as error:
        if len(sys.argv) == 1:
            from tkinter import Tk, messagebox

            root = Tk()
            root.withdraw()
            root.attributes("-topmost", True)
            messagebox.showerror("Planning failed", str(error), parent=root)
            root.destroy()
        else:
            print(f"Error: {error}")
        raise SystemExit(1)

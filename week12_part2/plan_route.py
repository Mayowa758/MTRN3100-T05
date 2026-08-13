from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

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
from workflow_ui import choose_image, clear_generated_route, save_course_selection


SCRIPT_DIR = Path(__file__).resolve().parent
# Kept for compatibility with the original helper name.
_clear_generated_route = clear_generated_route
POINT_LABELS = (
    "5x5 COURSE top-left",
    "5x5 COURSE top-right",
    "5x5 COURSE bottom-right",
    "5x5 COURSE bottom-left",
    "GREEN entrance cell centre",
    "RED exit cell centre",
)


def window_is_open(name: str) -> bool:
    try:
        return cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:
        return False


def collect_course_selection(image):
    height, width = image.shape[:2]
    scale = min(1.0, 1400.0 / width, 820.0 / height)
    preview = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    points: list[list[float]] = []
    window = "First 2 marks - obstacle course selection"

    def on_mouse(event, x, y, _flags, _parameter):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 6:
            points.append([x / scale, y / scale])
        elif event == cv2.EVENT_RBUTTONDOWN:
            points.clear()

    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window, on_mouse)
    while True:
        display = preview.copy()
        scaled = [(int(round(x * scale)), int(round(y * scale))) for x, y in points]
        if len(scaled) >= 2:
            for index in range(min(len(scaled) - 1, 3)):
                cv2.line(display, scaled[index], scaled[index + 1], (255, 0, 255), 2)
        if len(scaled) >= 4:
            cv2.line(display, scaled[3], scaled[0], (255, 0, 255), 2)
        for index, point in enumerate(scaled):
            colour = (0, 200, 0) if index == 4 else (0, 0, 255) if index == 5 else (255, 0, 255)
            cv2.circle(display, point, 7, colour, -1)
        instruction = (
            f"Click {POINT_LABELS[len(points)]} | Right-click/R: reset | Esc/X: close"
            if len(points) < 6
            else "Enter: plan first 2 marks | Right-click/R: reset | Esc/X: close"
        )
        cv2.rectangle(display, (0, 0), (min(display.shape[1], 1050), 38), (255, 255, 255), -1)
        cv2.putText(display, instruction, (10, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.67, (0, 0, 0), 2)
        cv2.imshow(window, display)
        key = cv2.waitKey(20) & 0xFF
        if not window_is_open(window):
            cv2.destroyAllWindows()
            raise RuntimeError("Selection window was closed")
        if key in (ord("r"), ord("R")):
            points.clear()
        elif key in (10, 13) and len(points) == 6:
            break
        elif key == 27:
            cv2.destroyAllWindows()
            raise RuntimeError("Selection cancelled")
    cv2.destroyWindow(window)
    return points[:4], points[4], points[5]


def show_course_result(planning_image, obstacle_count: int, commands: str) -> None:
    from tkinter import Tk, messagebox

    window = "First 2 marks result - press any key to close"
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
        "First 2 marks planning complete",
        f"Detected obstacles: {obstacle_count}\n"
        f"Obstacle-course commands: {commands}\n\n"
        "The selection was saved for plan_full_route.py.",
        parent=root,
    )
    root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plan only the 5 x 5 obstacle-course route")
    parser.add_argument("image", nargs="?", type=Path, help="fixed overhead maze image")
    parser.add_argument("--config", type=Path, default=SCRIPT_DIR / "config.json")
    parser.add_argument("--output", type=Path, default=SCRIPT_DIR / "output")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    gui_launch = args.image is None
    image_path = choose_image(SCRIPT_DIR.parent) if gui_launch else args.image.resolve()
    image = imread_unicode(image_path)
    config = load_config(args.config)
    corners, entrance, exit_point = collect_course_selection(image)
    args.output.mkdir(parents=True, exist_ok=True)
    save_course_selection(
        args.output / "last_course_selection.json",
        image_path,
        corners,
        entrance,
        exit_point,
    )
    arduino_header = SCRIPT_DIR / "arduino" / "week12_part2" / "GeneratedRoute.h"
    try:
        result = plan_course(image, corners, entrance, exit_point, config)
        if not result.obstacles:
            raise RuntimeError("No cylindrical obstacles were detected")
    except (ValueError, RuntimeError) as error:
        clear_generated_route(arduino_header)
        raise type(error)(f"{error}\n\nThe old Arduino route was cleared for safety.") from error

    planning_image = render_plan_on_original(image, result, config)
    imwrite_unicode(args.output / "course_planning_result.png", planning_image)
    imwrite_unicode(args.output / "course_occupancy_map.png", render_occupancy(result, config))
    save_route_json(args.output / "course_route.json", result, "", "")
    save_arduino_header(arduino_header, result, "", "")
    save_course_selection(
        args.output / "course_selection.json",
        image_path,
        corners,
        entrance,
        exit_point,
    )
    if gui_launch:
        show_course_result(planning_image, len(result.obstacles), result.grid_commands)
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

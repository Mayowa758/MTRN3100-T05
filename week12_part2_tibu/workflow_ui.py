from __future__ import annotations

import json
from pathlib import Path
from typing import Sequence


def choose_image(initial_directory: Path) -> Path:
    from tkinter import Tk, filedialog

    root = Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    selected = filedialog.askopenfilename(
        title="Select the fixed overhead maze image",
        initialdir=str(initial_directory),
        filetypes=(("Image files", "*.jpg *.jpeg *.png *.bmp *.webp"), ("All files", "*.*")),
    )
    root.destroy()
    if not selected:
        raise RuntimeError("No image was selected")
    return Path(selected).resolve()


def save_course_selection(
    path: Path,
    image_path: Path,
    course_corners: Sequence[Sequence[float]],
    entrance: Sequence[float],
    exit_point: Sequence[float],
) -> None:
    payload = {
        "image_path": str(image_path.resolve()),
        "course_corners": [list(point) for point in course_corners],
        "entrance": list(entrance),
        "exit": list(exit_point),
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def load_course_selection(path: Path):
    if not path.exists():
        raise RuntimeError(
            "No saved obstacle-course selection exists. Run plan_route.py successfully first."
        )
    with path.open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    image_path = Path(payload.get("image_path", ""))
    corners = payload.get("course_corners")
    entrance = payload.get("entrance")
    exit_point = payload.get("exit")
    if not image_path.is_file():
        raise RuntimeError(
            "The saved overhead image cannot be found. Run plan_route.py again and select the image."
        )
    if corners is None or len(corners) != 4 or entrance is None or exit_point is None:
        raise RuntimeError(
            "The saved obstacle-course selection is incomplete. Run plan_route.py again."
        )
    return image_path, corners, entrance, exit_point


def ask_full_maze_positions(default_start=None, default_goal=None):
    from tkinter import Button, Entry, Label, StringVar, Tk, messagebox

    root = Tk()
    root.title("Full 9x9 maze positions")
    root.attributes("-topmost", True)
    start_default = default_start or [0, 0, "N"]
    goal_default = default_goal or [0, 0]
    values = [
        StringVar(value=str(start_default[0])),
        StringVar(value=str(start_default[1])),
        StringVar(value=str(start_default[2]).upper()),
        StringVar(value=str(goal_default[0])),
        StringVar(value=str(goal_default[1])),
    ]
    labels = (
        "Start row (0-8)",
        "Start column (0-8)",
        "Start direction (N/E/S/W)",
        "Goal row (0-8)",
        "Goal column (0-8)",
    )
    for row, (label, value) in enumerate(zip(labels, values)):
        Label(root, text=label, anchor="w", width=28).grid(
            row=row, column=0, padx=10, pady=5
        )
        Entry(root, textvariable=value, width=10).grid(
            row=row, column=1, padx=10, pady=5
        )

    result = []

    def submit():
        try:
            start = (
                int(values[0].get()),
                int(values[1].get()),
                values[2].get().strip().upper(),
            )
            goal = (int(values[3].get()), int(values[4].get()))
            if not all(0 <= number <= 8 for number in (*start[:2], *goal)):
                raise ValueError("Rows and columns must be between 0 and 8")
            if len(start[2]) != 1 or start[2] not in "NESW":
                raise ValueError("Start direction must be N, E, S, or W")
        except ValueError as error:
            messagebox.showerror("Invalid positions", str(error), parent=root)
            return
        result.extend((start, goal))
        root.destroy()

    Button(root, text="Generate full route", command=submit, width=20).grid(
        row=5, column=0, padx=10, pady=12
    )
    Button(root, text="Cancel", command=root.destroy, width=10).grid(
        row=5, column=1, padx=10, pady=12
    )
    root.protocol("WM_DELETE_WINDOW", root.destroy)
    root.mainloop()
    if not result:
        raise RuntimeError("Full-maze position input was cancelled")
    return result[0], result[1]


def clear_generated_route(path: Path) -> None:
    path.write_text(
        "/* Planning failed: route intentionally cleared for safety. */\n"
        "#pragma once\n\n"
        "struct CourseMotion {\n"
        "    float turnDeg;\n"
        "    float distanceMm;\n"
        "};\n\n"
        'const char PRE_COMMANDS[] = "";\n'
        'const char POST_COMMANDS[] = "";\n'
        'const CourseMotion COURSE_MOTIONS[] = {{0.0f, 0.0f}};\n'
        'const unsigned int COURSE_MOTION_COUNT = 0;\n',
        encoding="utf-8",
    )

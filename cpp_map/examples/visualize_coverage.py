"""Render the baseline C++ coverage path for a 100x100-cell map."""

import argparse
import csv
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw


def read_cpp_path(binary: Path):
    output = subprocess.check_output([str(binary)], text=True)
    return [(int(row), int(column)) for row, column in csv.reader(output.splitlines())]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, default=Path("build/mower_coverage_demo"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    path = read_cpp_path(args.binary)
    cell = 8
    margin = 70
    legend_height = 105
    image = Image.new("RGB", (100 * cell + margin * 2, 100 * cell + margin * 2 + legend_height), "white")
    draw = ImageDraw.Draw(image)
    left, top = margin, margin

    # Draw the 100x100 map. Coordinates are shown with north at the top.
    for row in range(100):
        for column in range(100):
            color = "#f8fafc"
            if 35 <= row < 51 and 42 <= column < 58:
                color = "#ef4444"
            draw.rectangle((left + column * cell, top + (99 - row) * cell,
                            left + (column + 1) * cell, top + (100 - row) * cell),
                           fill=color, outline="#dbe3ee", width=1)

    points = [(left + column * cell + cell // 2,
               top + (99 - row) * cell + cell // 2) for row, column in path]
    if len(points) > 1:
        draw.line(points, fill="#2563eb", width=2, joint="curve")
    for point in points:
        draw.ellipse((point[0] - 2, point[1] - 2, point[0] + 2, point[1] + 2), fill="#1d4ed8")

    current_row, current_column = path[len(path) // 2]
    x0 = left + (current_column - 3.5) * cell
    y0 = top + (99 - (current_row + 3.5)) * cell
    x1 = left + (current_column + 3.5) * cell
    y1 = top + (99 - (current_row - 3.5)) * cell
    draw.rectangle((x0, y0, x1, y1), fill="#60a5fa", outline="#1e3a8a", width=3)
    draw.rectangle((left + (current_column - 1.5) * cell,
                    top + (99 - (current_row + 1.5)) * cell,
                    left + (current_column + 1.5) * cell,
                    top + (99 - (current_row - 1.5)) * cell),
                   fill="#f59e0b", outline="#92400e", width=2)
    draw.ellipse((left + current_column * cell + cell // 2 - 4,
                  top + (99 - current_row) * cell + cell // 2 - 4,
                  left + current_column * cell + cell // 2 + 4,
                  top + (99 - current_row) * cell + cell // 2 + 4), fill="#111827")

    draw.text((margin, 22), "C++ baseline coverage path | 100 x 100 cells = 20m x 20m", fill="#111827")
    legend_y = 100 * cell + margin + 18
    for x, color, label in [(margin, "#2563eb", f"zigzag path ({len(path)} centers)"),
                             (margin + 220, "#ef4444", "obstacle"),
                             (margin + 350, "#f59e0b", "cut 3x3"),
                             (margin + 455, "#60a5fa", "mower 7x7")]:
        draw.rectangle((x, legend_y, x + 16, legend_y + 16), fill=color, outline="#334155")
        draw.text((x + 22, legend_y + 1), label, fill="#111827")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    image.save(args.output)


if __name__ == "__main__":
    main()

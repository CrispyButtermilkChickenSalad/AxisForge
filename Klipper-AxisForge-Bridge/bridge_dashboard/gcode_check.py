"""Static G-code analysis used by preflight.

Frame contract: X/Y/Z are linear bed coordinates in mm; A/B/C are absolute
rotations in degrees about the X/Y/Z axes. The robot performs all motion;
Klipper handles extrusion (E) only.
"""

import hashlib
import re

WORD = re.compile(r"([A-Za-z])\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+))")
COMMENTS = re.compile(r";.*$|\([^)]*\)")
LINEAR_AXES = ("X", "Y", "Z")
ROTARY_AXES = ("A", "B", "C")
ROBOT_AXES = LINEAR_AXES + ROTARY_AXES


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def analyze_gcode(path):
    """Return per-axis ranges, rotary axes used and total positive extrusion."""
    ranges = {}
    extrusion_mm = 0.0
    extruder_position = 0.0
    relative_extrusion = False
    with open(path, encoding="utf-8", errors="replace") as source:
        for raw_line in source:
            words = [
                (letter.upper(), float(value))
                for letter, value in WORD.findall(COMMENTS.sub("", raw_line))
            ]
            if not words:
                continue
            command = f"{words[0][0]}{int(words[0][1])}" if words[0][0] in "GM" else ""
            if command == "M82":
                relative_extrusion = False
            elif command == "M83":
                relative_extrusion = True
            elif command == "G92":
                for letter, value in words[1:]:
                    if letter == "E":
                        extruder_position = value
            elif command in ("", "G0", "G1", "G2", "G3"):
                for letter, value in words:
                    if letter in ROBOT_AXES:
                        low, high = ranges.get(letter, (value, value))
                        ranges[letter] = (min(low, value), max(high, value))
                    elif letter == "E":
                        delta = value if relative_extrusion else value - extruder_position
                        if not relative_extrusion:
                            extruder_position = value
                        extrusion_mm += max(delta, 0.0)
    return {
        "axes": {letter: list(span) for letter, span in ranges.items()},
        "rotary_axes_used": [axis for axis in ROTARY_AXES if axis in ranges],
        "extrusion_mm": round(extrusion_mm, 3),
    }

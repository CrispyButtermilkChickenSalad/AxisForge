"""Rewrite legacy G-code where A meant nozzle pitch so that pitch is now B.

A, B and C are now absolute rotations in degrees about X, Y and Z. Old files used A
as an absolute pitch offset, which corresponds to B. Usage:
    python migrate_gcode_a_to_b.py input.gcode [output.gcode]
The input is rewritten in place when no output is given.
"""
import re
import sys

WORD = re.compile(r'(?<![A-Za-z0-9])A(?=[-+]?[0-9.])')


def migrate_line(line):
    """Rename A words to B outside comments."""
    code, sep, comment = line.partition(';')
    return WORD.sub('B', code) + sep + comment


def main(argv):
    """Migrate a G-code file."""
    if not 2 <= len(argv) <= 3:
        print(__doc__)
        return 2
    with open(argv[1], encoding='utf-8', newline='') as handle:
        lines = handle.read().splitlines(keepends=True)
    with open(argv[-1], 'w', encoding='utf-8', newline='') as handle:
        handle.writelines(migrate_line(line) for line in lines)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))

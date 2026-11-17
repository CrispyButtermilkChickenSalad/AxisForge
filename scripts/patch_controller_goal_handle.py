#!/usr/bin/env python3
"""Prepare a version-checked, private realtime_tools header for the UR overlay."""
import hashlib
from pathlib import Path
import sys


def prepare(source, destination):
    original = Path(source).read_bytes()
    expected = '178972135ab1abbae31164a9bfbfcad81d24cfbe2abcc3aa40449f168b0c7e00'
    if hashlib.sha256(original).hexdigest() != expected:
        raise SystemExit('Unsupported realtime_tools header; review the upstream version before patching.')
    text = original.decode()
    # rcl_action explicitly permits SUCCEED and ABORT from CANCELING. Preserve
    # the outcome already queued by the RT update instead of stranding it when
    # a cancellation arrives before the non-RT result timer runs.
    text = text.replace('if (req_abort_ && gh_->is_executing()) {',
                        'if (req_abort_ && gh_->is_active()) {', 1)
    text = text.replace('if (req_succeed_ && !gh_->is_canceling()) {',
                        'if (req_succeed_ && gh_->is_active()) {', 1)
    output = Path(destination)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text)


if __name__ == '__main__':
    prepare(*sys.argv[1:])

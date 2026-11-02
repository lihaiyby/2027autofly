#!/usr/bin/env python3
"""兼容入口：交给 autofly.cli（真实实现在 src/autofly/）。

旧调用方式 `mission_driver.py 2.4 2.3` 等价于 `--search-z 2.4 --drop-z 2.3`。
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "src"))

from autofly.cli import main  # noqa: E402

if __name__ == "__main__":
    argv = sys.argv[1:]
    mapped = []
    i = 0
    if argv and not argv[0].startswith("-"):
        mapped += ["--search-z", argv[0]]
        i = 1
        if len(argv) > 1 and not argv[1].startswith("-"):
            mapped += ["--drop-z", argv[1]]
            i = 2
    sys.exit(main(mapped + argv[i:]))

#!/usr/bin/env python3
"""Patch the installed ompl 1.7.0 wheel to stop an interpreter-teardown crash.

Background
----------
The ompl 1.7.0 PyPI wheel loads libompl twice:

  * ``ompl/util/__init__.py`` preloads an un-repaired ``libompl.so`` with
    ``RTLD_GLOBAL`` via ``dll_loader``; while
  * the compiled extension modules (``_base.so`` ...) link the
    auditwheel-repaired ``ompl.libs/libompl-<hash>.so`` through their RPATH.

The two copies have different sonames, so both are mapped. Each holds its own
copy of OMPL's ``DubinsStateSpace`` static tables and registers its own
``atexit`` C++ destructor. At process exit both destructors run and one frees a
pointer owned by the other's heap, aborting with ``free(): invalid pointer`` /
``double free`` (exit 134) -- after all real work has completed.

After the auditwheel repair the RPATH already loads the correct single copy, so
the ``dll_loader`` preload is redundant. Commenting it out leaves exactly one
libompl mapped and the teardown crash disappears.


Usage
-----
Run once after installing dependencies, using the SAME interpreter as the
project environment::

    python scripts/patch_ompl.py

The script is idempotent: re-running it (or running it against an ompl build
that has no preload, e.g. 2.x) is a harmless no-op. Exit code is 0 on success
or no-op, non-zero only if ompl is not importable.
"""

from __future__ import annotations

import os
import re
import sys

MARKER = "tampest-patch: disabled duplicate libompl preload (atexit double-free)"
# Matches an *active* (uncommented) dll_loader('ompl', ...) call at line start.
CALL_RE = re.compile(r"""^(\s*)(dll_loader\(\s*['"]ompl['"].*)$""")


def find_util_init() -> str:
    """Locate the installed ompl/util/__init__.py without importing ompl."""
    import importlib.util

    spec = importlib.util.find_spec("ompl")
    if spec is None or not spec.submodule_search_locations:
        print("ERROR: ompl is not installed in this environment "
              f"({sys.executable}).", file=sys.stderr)
        raise SystemExit(2)
    pkg_dir = list(spec.submodule_search_locations)[0]
    return os.path.join(pkg_dir, "util", "__init__.py")


def patch(path: str) -> bool:
    """Comment out the redundant preload. Return True if a change was made."""
    with open(path, "r", encoding="utf-8") as fh:
        lines = fh.readlines()

    if any(MARKER in ln for ln in lines):
        print(f"Already patched: {path}")
        return False

    changed = False
    out = []
    for ln in lines:
        m = CALL_RE.match(ln.rstrip("\n"))
        if m:
            indent, call = m.group(1), m.group(2)
            out.append(f"{indent}# {call}  # {MARKER}\n")
            changed = True
        else:
            out.append(ln)

    if not changed:
        print(f"Nothing to patch (no active preload found): {path}")
        return False

    with open(path, "w", encoding="utf-8") as fh:
        fh.writelines(out)
    print(f"Patched: {path}")
    return True


def main() -> int:
    path = find_util_init()
    if not os.path.isfile(path):
        # ompl 2.x layout has flat modules and no util/__init__.py preload.
        print(f"No preload file to patch ({path} absent); nothing to do.")
        return 0
    patch(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Check that every built .fap and .fal can resolve its imports at load time.

A FAP links with its imports left undefined on purpose: they are resolved when
the loader runs, against the firmware API table plus, for a plugin, the
app-local table of the app that loads it. So the build never fails on a symbol
that does not exist, and a missing export only shows up on the device as
"Failed to resolve address for symbol X" followed by MissingImports.

That is how an externalised app ships broken: it imports a symbol firmware
never exported, because an internal app needs no exports.

Usage: python3 scripts/check_fap_imports.py [--build build/f7-firmware-C]
"""

import argparse
import csv
import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
TABLE_RE = re.compile(r"API_(?:METHOD|VARIABLE)\(\s*([A-Za-z_]\w*)")

# Every app-local API table in the tree. A plugin loaded by an app resolves
# against firmware plus that app's table, and JS modules additionally resolve
# symbols exported by each other, so anything outside the CLI is checked
# against the union: the question worth answering is whether a symbol can be
# resolved at all, not by exactly which table.
APP_API_TABLE_DIRS = [
    "applications/main/nfc/api",
    "applications/main/subghz/api",
    "applications/system/js_app/plugin_api",
]

# CLI plugins are the strict case. lib/toolbox/cli/shell/cli_shell.c loads them
# with plugin_manager_alloc(..., firmware_api_interface), so they see firmware
# exports and nothing else.
CLI_PREFIX = "cli_"


def firmware_exports(target):
    path = ROOT / f"targets/{target}/api_symbols.csv"
    out = set()
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            if row["entry"] in ("Function", "Variable") and row["status"] == "+":
                out.add(row["name"])
    return out


def table_symbols(dirname):
    out = set()
    d = ROOT / dirname
    if not d.is_dir():
        return out
    for p in d.glob("*.h"):
        out |= set(TABLE_RE.findall(p.read_text(errors="ignore")))
    return out


def undefined(nm, path):
    res = subprocess.run([nm, "-u", str(path)], capture_output=True, text=True)
    return {
        line.split()[1]
        for line in res.stdout.splitlines()
        if len(line.split()) >= 2 and line.split()[0] == "U"
    }


def defined(nm, path):
    res = subprocess.run(
        [nm, "--defined-only", str(path)], capture_output=True, text=True
    )
    out = set()
    for line in res.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[1] in "TtDdRrBbWw":
            out.add(parts[2])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--build", default="build/f7-firmware-C")
    ap.add_argument("--target", default="f7")
    ap.add_argument("--nm", default="toolchain/current/bin/arm-none-eabi-nm")
    args = ap.parse_args()

    nm = ROOT / args.nm
    if not nm.exists():
        nm = pathlib.Path("arm-none-eabi-nm")

    fw = firmware_exports(args.target)
    app_tables = set()
    for d in APP_API_TABLE_DIRS:
        app_tables |= table_symbols(d)
    extapps = ROOT / args.build / ".extapps"
    if not extapps.is_dir():
        print(f"No built apps at {extapps}; build first.")
        return 1

    paths = sorted(list(extapps.glob("*.fap")) + list(extapps.glob("*.fal")))
    # JS modules export helpers to each other, so anything a sibling defines counts
    siblings = set()
    for p in paths:
        siblings |= defined(nm, p)

    failures = 0
    checked = 0
    for path in paths:
        checked += 1
        if path.stem.startswith(CLI_PREFIX):
            resolvable = set(fw)  # CLI plugins see firmware only
        else:
            resolvable = fw | app_tables | siblings
        missing = sorted(undefined(nm, path) - resolvable)
        if missing:
            failures += 1
            print(f"  {path.name}: {len(missing)} unresolved")
            for m in missing[:10]:
                print(f"      {m}")
            if len(missing) > 10:
                print(f"      ... and {len(missing) - 10} more")

    if failures:
        print(f"\n{failures} of {checked} apps would fail to load. See the docstring.")
        return 1
    print(f"All {checked} built apps resolve their imports.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

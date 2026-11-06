#!/usr/bin/env python3
"""Read-only validation of the project's flat Markdown report table and current-report cap."""

import argparse
from collections import Counter
import json
from pathlib import Path
import re
from urllib.parse import unquote

# Report metadata and navigation files are not report entries.
NON_REPORTS = {"AGENTS.md", "INDEX.md", "README.md"}
# The current user-requested cap; callers can supply a newer explicit limit.
DEFAULT_CURRENT_LIMIT = 50


def report_files(directory):
    """List direct Markdown reports without counting attachments or packaged copies."""
    return {path for path in directory.glob("*.md") if path.name not in NON_REPORTS}


def audit(project, limit):
    """Compare actual report files with date-first table rows in INDEX.md."""
    root = project / "agent" / "report"
    index = root / "INDEX.md"
    if not index.is_file():
        return {"error": f"Missing report index: {index}"}, False
    current = report_files(root)
    history = report_files(root / "history")
    actual = {path.relative_to(root).as_posix() for path in current | history}
    entries = []
    invalid_rows = []
    long_summaries = []
    index_text = index.read_text(encoding="utf-8")
    for number, line in enumerate(index_text.splitlines(), 1):
        if not re.match(r"^\|\s*\d{4}-\d{2}-\d{2}\s*\|", line):
            continue
        columns = line.strip().strip("|").split("|")
        match = re.search(r"\]\((<[^>]+>|[^)]+)\)", columns[1]) if len(columns) == 3 else None
        if match is None:
            invalid_rows.append(number)
            continue
        target = unquote(match.group(1).strip().strip("<>").split("#", 1)[0])
        # Normalize relative forms while rejecting links outside the report inventory.
        destination = (root / target).resolve()
        try:
            target = destination.relative_to(root.resolve()).as_posix()
        except ValueError:
            target = str(destination)
        entries.append(target)
        if len(columns[2].strip()) > 100:
            long_summaries.append(number)
    counts = Counter(entries)
    header = re.search(r"共 \*\*(\d+) 份报告\*\*（当前目录(\d+)份，history归档(\d+)份）", index_text)
    header_mismatch = bool(header and tuple(map(int, header.groups())) != (len(actual), len(current), len(history)))
    result = {
        "current_reports": len(current),
        "max_current": limit,
        "over_limit": max(0, len(current) - limit),
        "history_reports": len(history),
        "total_reports": len(actual),
        "index_entries": len(entries),
        "header_count_mismatch": header_mismatch,
        "unindexed": sorted(actual - counts.keys()),
        "missing_or_nonreport_targets": sorted(counts.keys() - actual),
        "duplicate_index_entries": sorted(path for path, count in counts.items() if count > 1),
        "invalid_table_rows": invalid_rows,
        "summary_over_100_characters_rows": long_summaries,
    }
    valid = not any(result[key] for key in (
        "over_limit", "unindexed", "missing_or_nonreport_targets", "duplicate_index_entries",
        "invalid_table_rows", "summary_over_100_characters_rows", "header_count_mismatch",
    ))
    return result, valid


def main():
    """Print a JSON audit and return nonzero for an invalid index or report limit."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project", nargs="?", type=Path, default=Path.cwd())
    parser.add_argument("--max-current", type=int, default=DEFAULT_CURRENT_LIMIT)
    args = parser.parse_args()
    if args.max_current < 0:
        parser.error("--max-current must be nonnegative")
    result, valid = audit(args.project.resolve(), args.max_current)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if valid else 1


if __name__ == "__main__":
    raise SystemExit(main())

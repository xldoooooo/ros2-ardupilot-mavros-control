#!/usr/bin/env python3
"""Read-only checks for report indexes, new-report metadata, and topic navigation."""

import argparse
from collections import Counter
import json
from pathlib import Path
import re
from urllib.parse import unquote, urlsplit

# Report metadata and navigation files are not report entries.
NON_REPORTS = {"AGENTS.md", "INDEX.md", "README.md"}
# The current user-requested cap; callers can supply a newer explicit limit.
DEFAULT_CURRENT_LIMIT = 50
# Earlier reports remain compatible; same-day additions are selected with --report.
LEGACY_THROUGH_DATE = "2026-11-07"
# Short labels used in the shared report workflow, combinable with a slash.
REPORT_TYPES = {"实现", "修复", "分析", "部署", "研究", "维护"}
# Small Markdown metadata block required before the report body.
REQUIRED_FIELDS = ("类型", "主专题", "承接报告", "验证范围")
# Links in this project's navigation use ordinary Markdown inline targets.
MARKDOWN_LINK = re.compile(r"\[[^\]]*\]\((<[^>]+>|[^)]+)\)")


def local_links(path):
    """Resolve local Markdown links, ignoring fenced examples and external URLs."""
    fenced = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.lstrip().startswith(("```", "~~~")):
            fenced = not fenced
            continue
        if fenced:
            continue
        for match in MARKDOWN_LINK.finditer(line):
            target = match.group(1).strip().strip("<>")
            parsed = urlsplit(target)
            if parsed.scheme or parsed.netloc or not parsed.path:
                continue
            yield (path.parent / unquote(parsed.path)).resolve()


def metadata(path):
    """Read the four plain Markdown fields from the beginning of a new report."""
    fields = {}
    for line in path.read_text(encoding="utf-8").splitlines()[:30]:
        match = re.fullmatch(r"(类型|主专题|承接报告|验证范围)：\s*(.*)", line)
        if match:
            fields[match.group(1)] = match.group(2).strip()
    return fields


def metadata_targets(value, path):
    """Extract relative Markdown link destinations from a metadata field."""
    targets = []
    for match in MARKDOWN_LINK.finditer(value):
        parsed = urlsplit(match.group(1).strip().strip("<>"))
        if parsed.scheme or parsed.netloc or not parsed.path:
            continue
        targets.append((path.parent / unquote(parsed.path)).resolve())
    return targets


def check_workflow(project, reports, selected):
    """Check topic navigation and new reports without migrating historical text."""
    root = project / "agent" / "report"
    topics_root = root / "topics"
    topic_index = topics_root / "INDEX.md"
    topics = {p.resolve() for p in topics_root.glob("*.md") if p.name not in NON_REPORTS}
    errors = []
    checked = set()
    report_paths = {p.resolve() for p in reports}
    for value in selected:
        path = (project / value).resolve()
        if path not in report_paths:
            errors.append(f"--report is not an inventoried report: {value}")
        else:
            checked.add(path)
    for path in report_paths:
        date = re.match(r"report-(\d{4}-\d{2}-\d{2})-", path.name)
        if metadata(path) or (date and date.group(1) > LEGACY_THROUGH_DATE):
            checked.add(path)

    if topics or topic_index.exists():
        if not topic_index.is_file():
            errors.append("Missing topic index: agent/report/topics/INDEX.md")
        else:
            indexed = Counter(local_links(topic_index))
            for path in sorted(topics):
                if indexed[path] != 1:
                    errors.append(f"Topic must be indexed once: {path.relative_to(project)}")

    for path in sorted(checked):
        fields = metadata(path)
        label = str(path.relative_to(project))
        for field in REQUIRED_FIELDS:
            if not fields.get(field):
                errors.append(f"{label}: missing metadata field {field}")
        types = {part.strip() for part in fields.get("类型", "").split("/")}
        if not types <= REPORT_TYPES:
            errors.append(f"{label}: unsupported report type")
        topic = fields.get("主专题", "")
        if topic and topic != "无":
            targets = metadata_targets(topic, path)
            if len(targets) != 1 or targets[0] not in topics:
                errors.append(f"{label}: 主专题 must link to one topic page or be 无")
            elif path not in set(local_links(targets[0])):
                errors.append(f"{label}: main topic does not link back to report")
        predecessors = fields.get("承接报告", "")
        if predecessors and predecessors != "无":
            targets = metadata_targets(predecessors, path)
            if not targets or any(p not in report_paths or p == path for p in targets):
                errors.append(f"{label}: 承接报告 must link to other reports or be 无")

    pages = topics | checked
    if topic_index.is_file():
        pages.add(topic_index)
    for page in sorted(pages):
        for target in local_links(page):
            if not target.exists():
                errors.append(f"{page.relative_to(project)}: missing local link {target}")
    return {
        "topic_pages": len(topics),
        "workflow_reports_checked": len(checked),
        "legacy_reports": len(report_paths - checked),
        "workflow_errors": errors,
    }


def report_files(directory):
    """List direct Markdown reports without counting attachments or packaged copies."""
    return {path for path in directory.glob("*.md") if path.name not in NON_REPORTS}


def audit(project, limit, selected=()):
    """Compare report inventory and indexes, then validate workflow navigation."""
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
    result.update(check_workflow(project, current | history, selected))
    valid = not any(result[key] for key in (
        "over_limit", "unindexed", "missing_or_nonreport_targets", "duplicate_index_entries",
        "invalid_table_rows", "summary_over_100_characters_rows", "header_count_mismatch",
        "workflow_errors",
    ))
    return result, valid


def main():
    """Print a JSON audit and return nonzero for an invalid index or report limit."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project", nargs="?", type=Path, default=Path.cwd())
    parser.add_argument("--max-current", type=int, default=DEFAULT_CURRENT_LIMIT)
    parser.add_argument(
        "--report", action="append", default=[],
        help="Require new-report metadata for this project-relative path; repeat as needed.",
    )
    args = parser.parse_args()
    if args.max_current < 0:
        parser.error("--max-current must be nonnegative")
    result, valid = audit(args.project.resolve(), args.max_current, args.report)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if valid else 1


if __name__ == "__main__":
    raise SystemExit(main())

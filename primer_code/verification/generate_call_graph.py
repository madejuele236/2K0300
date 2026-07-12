#!/usr/bin/env python3
"""Generate a conservative, source-level function call graph for primer_code.

The existing refactor verifier already owns the source/function-discovery
boundary.  This report generator reuses that boundary and adds a deliberately
conservative lexical resolver:

* every discovered definition is emitted as a node, including leaf functions;
* calls whose target is uniquely known in the selected source set are linked;
* overloaded/qualified calls that cannot be selected safely become an
  ``ambiguous`` boundary node rather than an invented edge;
* calls outside the source set (vendor headers, libc, OpenCV, callbacks, and
  virtual dispatch) become explicit ``external`` nodes;
* the JSON report is the canonical complete inventory; DOT and Markdown are
  views of the same node/edge set.

This is static evidence, not a runtime trace.  In particular, macro expansion,
function-pointer targets, virtual dispatch, template instantiation, and link
time library bodies are intentionally represented as unresolved boundaries.
"""

from __future__ import annotations

import argparse
import collections
import dataclasses
import json
import pathlib
import re
import sys
from typing import Iterable


HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PROJECT = ROOT / "primer_code" / "project"

# Keep the discovery parser in one place with the equivalence verifier.
sys.path.insert(0, str(HERE))
from compare_function_bodies import CONTROL_WORDS, FunctionBody, active_sources, function_bodies, mask_non_code  # noqa: E402


CPP_EXTENSIONS = {".c", ".cc", ".cpp", ".cxx"}
IDENTIFIER = re.compile(r"^[A-Za-z_]\w*$")
QUALIFIER_TOKENS = {"::", ".", "->"}
CALL_EXCLUSIONS = CONTROL_WORDS | {
    "alignas",
    "decltype",
    "delete",
    "new",
    "sizeof",
    "static_assert",
    "throw",
    "typeid",
    "typeof",
    "using",
}


@dataclasses.dataclass(frozen=True)
class Record:
    key: str
    name: str
    source: str
    line: int
    kind: str
    tokens: tuple[str, ...]


@dataclasses.dataclass(frozen=True)
class NamespaceInterval:
    name: str
    opening: int
    closing: int


@dataclasses.dataclass(frozen=True)
class Edge:
    caller: str
    callee: str
    kind: str
    spelling: str
    count: int


def _normal(path: pathlib.Path) -> str:
    return path.resolve().relative_to(ROOT).as_posix()


def cmake_sources() -> tuple[set[pathlib.Path], set[pathlib.Path]]:
    """Read the explicit application/vendor source lists from CMake."""

    cmake = PROJECT / "user" / "CMakeLists.txt"
    app: set[pathlib.Path] = set()
    vendor: set[pathlib.Path] = set()
    target: set[pathlib.Path] | None = None
    for raw in cmake.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if line.startswith("set(PRIMER_APP_SRCS"):
            target = app
            continue
        if line.startswith("set(PRIMER_VENDOR_SRCS"):
            target = vendor
            continue
        if target is not None and line == ")":
            target = None
            continue
        if target is not None and line.startswith("../"):
            target.add((PROJECT / "user" / line).resolve())
    return app, vendor


def source_set(scope: str) -> tuple[list[tuple[pathlib.Path, str]], str]:
    app, vendor = cmake_sources()
    headers = {
        (PROJECT / "code" / "port" / "low_pass_filter.hpp").resolve(),
        (PROJECT / "code" / "presentation" / "internal" / "page_common.hpp").resolve(),
    }
    if scope == "build":
        selected = [(p, "application") for p in sorted(app)]
        selected += [(p, "vendor") for p in sorted(vendor)]
        selected += [(p, "application-header") for p in sorted(headers)]
        return selected, "CMake PRIMER_APP_SRCS + PRIMER_VENDOR_SRCS + active inline headers"
    if scope == "own":
        selected = [(p, "application") for p in sorted(active_sources())]
        return selected, "all project/user/main.cpp + project/code/**/*.cpp + active inline headers"
    raise ValueError(f"unsupported scope: {scope}")


def _record_key(body: FunctionBody, index: int) -> str:
    return f"{body.source}:{body.line}:{body.name}:{index}"


_NAMESPACE = re.compile(r"\bnamespace(?:\s+([A-Za-z_]\w*(?:::[A-Za-z_]\w*)*)\s*)?\{")


def _namespace_intervals(text: str) -> list[NamespaceInterval]:
    """Find namespace brace intervals without interpreting comments/strings."""

    masked = mask_non_code(text)
    intervals: list[NamespaceInterval] = []
    for match in _NAMESPACE.finditer(masked):
        opening = match.end() - 1
        depth = 0
        closing = None
        for index in range(opening, len(masked)):
            if masked[index] == "{":
                depth += 1
            elif masked[index] == "}":
                depth -= 1
                if depth == 0:
                    closing = index
                    break
        if closing is not None and match.group(1):
            intervals.append(NamespaceInterval(match.group(1), opening, closing))
    return intervals


def _qualified_definition_name(body: FunctionBody, text: str, intervals: list[NamespaceInterval]) -> str:
    if "::" in body.name:
        return body.name
    line_start = 0
    for _ in range(body.line - 1):
        next_newline = text.find("\n", line_start)
        if next_newline < 0:
            return body.name
        line_start = next_newline + 1
    scopes = [item for item in intervals if item.opening < line_start <= item.closing]
    scopes.sort(key=lambda item: item.opening)
    prefix: list[str] = []
    for scope in scopes:
        prefix.extend(scope.name.split("::"))
    return "::".join(prefix + [body.name]) if prefix else body.name


def discover(scope: str) -> tuple[list[Record], list[str], str]:
    selected, description = source_set(scope)
    records: list[Record] = []
    sources: list[str] = []
    for path, kind in selected:
        if not path.exists():
            raise FileNotFoundError(path)
        source = _normal(path)
        sources.append(source)
        text = path.read_text(encoding="utf-8", errors="ignore")
        intervals = _namespace_intervals(text)
        bodies = function_bodies(text, source)
        for index, body in enumerate(bodies):
            name = _qualified_definition_name(body, text, intervals)
            qualified = dataclasses.replace(body, name=name)
            records.append(Record(_record_key(qualified, index), name, source, body.line, kind, body.tokens))
    return records, sorted(sources), description


def _short(name: str) -> str:
    return name.rsplit("::", 1)[-1]


def _qualified_spelling(tokens: tuple[str, ...], index: int) -> str:
    """Recover a qualified spelling immediately preceding ``(``."""

    parts = [tokens[index]]
    cursor = index - 1
    while cursor >= 1 and tokens[cursor] in QUALIFIER_TOKENS and IDENTIFIER.match(tokens[cursor - 1]):
        parts.insert(0, tokens[cursor - 1])
        parts.insert(1, tokens[cursor])
        cursor -= 2
    return "".join(parts)


def _is_call(tokens: tuple[str, ...], index: int) -> bool:
    token = tokens[index]
    if not IDENTIFIER.match(token) or token in CALL_EXCLUSIONS:
        return False
    if index + 1 >= len(tokens) or tokens[index + 1] != "(":
        return False
    # A label/declaration-like occurrence in a body is not a call when it is
    # preceded by a type keyword and followed by a brace; the normal source
    # parser still leaves those tokens in lambda/nested blocks.
    if index and tokens[index - 1] in {"case", "goto"}:
        return False
    return True


def _resolve(
    spelling: str,
    caller: Record,
    by_exact: dict[str, list[Record]],
    by_short: dict[str, list[Record]],
) -> tuple[str, str]:
    """Return ``(edge kind, target key)`` without guessing overloads."""

    exact = by_exact.get(spelling, [])
    if not exact and not spelling.startswith("primer::"):
        exact = by_exact.get(f"primer::{spelling}", [])
    if len(exact) == 1:
        return "internal", exact[0].key
    if len(exact) > 1:
        return "ambiguous", f"ambiguous:{spelling}"

    short = by_short.get(_short(spelling), [])
    # Prefer a same-TU definition if one exists.  This handles unqualified
    # calls inside namespace/class implementation files without broad fanout.
    local = [record for record in short if record.source == caller.source]
    if len(local) == 1:
        return "internal", local[0].key
    if len(short) == 1:
        return "internal", short[0].key
    if len(short) > 1:
        return "ambiguous", f"ambiguous:{spelling}"
    return "external", f"external:{spelling}"


def build_edges(records: list[Record]) -> tuple[list[Edge], dict[str, dict[str, str]]]:
    by_exact: dict[str, list[Record]] = collections.defaultdict(list)
    by_short: dict[str, list[Record]] = collections.defaultdict(list)
    for record in records:
        by_exact[record.name].append(record)
        by_short[_short(record.name)].append(record)

    external_nodes: dict[str, dict[str, str]] = {}
    edge_counts: collections.Counter[tuple[str, str, str, str]] = collections.Counter()
    for caller in records:
        tokens = caller.tokens
        for index, token in enumerate(tokens):
            if not _is_call(tokens, index):
                continue
            spelling = _qualified_spelling(tokens, index)
            kind, callee = _resolve(spelling, caller, by_exact, by_short)
            if kind in {"external", "ambiguous"}:
                external_nodes.setdefault(callee, {"id": callee, "name": spelling, "kind": kind})
            edge_counts[(caller.key, callee, kind, spelling)] += 1
    edges = [Edge(caller, callee, kind, spelling, count) for (caller, callee, kind, spelling), count in sorted(edge_counts.items())]
    return edges, external_nodes


def _json(records: list[Record], sources: list[str], description: str, scope: str, edges: list[Edge], external: dict[str, dict[str, str]]) -> dict:
    internal = sum(edge.kind == "internal" for edge in edges)
    ambiguous = sum(edge.kind == "ambiguous" for edge in edges)
    outside = sum(edge.kind == "external" for edge in edges)
    return {
        "schema": "primer_code.function_call_graph.v1",
        "scope": scope,
        "scope_description": description,
        "source_files": sources,
        "function_count": len(records),
        "edge_count": len(edges),
        "edge_counts": {"internal": internal, "ambiguous": ambiguous, "external": outside},
        "resolution_notes": [
            "Internal edges are lexical source matches, with same-TU preference for unqualified names.",
            "Ambiguous edges intentionally do not choose an overload or virtual target.",
            "External edges represent libraries, macros, function pointers, virtual dispatch, or functions outside the selected source set.",
        ],
        "functions": [
            {"id": r.key, "name": r.name, "source": r.source, "line": r.line, "kind": r.kind}
            for r in records
        ],
        "external_nodes": sorted(external.values(), key=lambda item: item["id"]),
        "edges": [dataclasses.asdict(edge) for edge in edges],
    }


def _dot(report: dict) -> str:
    def quote(value: str) -> str:
        return '"' + value.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'

    lines = ["digraph primer_code_function_call_graph {", "  rankdir=LR;", "  graph [fontname=Helvetica, labelloc=t];", "  node [shape=box, fontname=Helvetica, fontsize=9];"]
    clusters: dict[str, list[dict]] = collections.defaultdict(list)
    for function in report["functions"]:
        module = function["source"].split("/project/code/", 1)[-1] if "/project/code/" in function["source"] else "user"
        clusters[module.split("/", 1)[0]].append(function)
    for module, functions in sorted(clusters.items()):
        lines.append(f"  subgraph cluster_{re.sub(r'[^A-Za-z0-9_]', '_', module)} {{")
        lines.append(f"    label={quote(module)};")
        for function in functions:
            node_id = "f_" + str(report["functions"].index(function))
            label = f"{function['name']}\\n{function['source']}:{function['line']}"
            lines.append(f"    {node_id} [label={quote(label)}];")
        lines.append("  }")
    external_ids: dict[str, str] = {}
    for index, node in enumerate(report["external_nodes"]):
        node_id = f"x_{index}"
        external_ids[node["id"]] = node_id
        style = "dashed" if node["kind"] == "external" else "dotted"
        lines.append(f"  {node_id} [label={quote(node['name'])}, style={style}, color=gray];")
    function_ids = {function["id"]: f"f_{index}" for index, function in enumerate(report["functions"])}
    for edge in report["edges"]:
        source = function_ids[edge["caller"]]
        target = function_ids.get(edge["callee"], external_ids.get(edge["callee"], ""))
        if not target:
            continue
        style = "solid" if edge["kind"] == "internal" else "dashed"
        label = edge["spelling"] if edge["count"] == 1 else f"{edge['spelling']} x{edge['count']}"
        lines.append(f"  {source} -> {target} [label={quote(label)}, style={style}];")
    lines.append("}")
    return "\n".join(lines) + "\n"


def _markdown(report: dict, dot_name: str, json_name: str) -> str:
    incoming: collections.Counter[str] = collections.Counter()
    outgoing: collections.Counter[str] = collections.Counter()
    for edge in report["edges"]:
        outgoing[edge["caller"]] += edge["count"]
        incoming[edge["callee"]] += edge["count"]
    by_source: dict[str, list[dict]] = collections.defaultdict(list)
    for function in report["functions"]:
        by_source[function["source"]].append(function)
    lines = [
        "# primer_code 完整函数调用图",
        "",
        f"- scope：`{report['scope_description']}`。",
        f"- 函数定义：**{report['function_count']}**；源文件：**{len(report['source_files'])}**；唯一调用边：**{report['edge_count']}**。",
        f"- 调用边分类：internal={report['edge_counts']['internal']}，ambiguous={report['edge_counts']['ambiguous']}，external={report['edge_counts']['external']}。",
        f"- 完整机器可读图：[JSON]({json_name})；完整 Graphviz 图：[DOT]({dot_name})。",
        "",
        "## 入口与主链",
        "",
        "```mermaid",
        "flowchart LR",
        "  main[main] --> RunApplication[primer::runtime::RunApplication]",
        "  RunApplication --> InitializeApplication[InitializeApplication]",
        "  RunApplication --> RunForegroundCycle[RunForegroundCycle]",
        "  InitializeApplication --> init[init]",
        "  InitializeApplication --> image_init[image_init]",
        "  RunForegroundCycle --> ImageDeal[ImageDeal]",
        "  RunForegroundCycle --> key_scan[key_scan]",
        "  RunForegroundCycle --> oled_show[oled_show]",
        "  ImageDeal --> vision_pipeline[vision pipeline stages]",
        "  pit_callback[pit_callback] --> active_cycle[active drive cycle]",
        "  active_cycle --> steering[Err_Sum / Image_PID_Calculate]",
        "  active_cycle --> platform[platform actuation]",
        "```",
        "",
        "上图是可读的入口摘要；下方按源文件逐项列出**全部函数定义**。每个函数的完整出边和解析状态以 JSON/DOT 为准。",
        "",
        "## 全部函数清单",
        "",
        "| 函数 | 所在位置 | 出边调用数 | 入边调用数 |",
        "|---|---|---:|---:|",
    ]
    for source in sorted(by_source):
        for function in sorted(by_source[source], key=lambda item: (item["line"], item["name"])):
            lines.append(f"| `{function['name']}` | `{function['source']}:{function['line']}` | {outgoing[function['id']]} | {incoming[function['id']]} |")
    lines += [
        "",
        "## 解析边界",
        "",
        "1. `internal`：在选定源集合内按函数名/同 TU 规则解析出的静态边。",
        "2. `ambiguous`：存在重载、同名定义或虚派发可能，报告保留边界节点，不猜具体实现。",
        "3. `external`：vendor 头文件、OpenCV/NCNN/TFLM/libc、宏展开、函数指针、线程/中断回调或不在源集合中的实现。",
        "4. 图是源码静态图，不是运行时 trace；硬件驱动的实际中断调度、线程时序和动态库内部调用需要单独运行/反汇编证据。",
        "",
        "## 构建范围证据",
        "",
        "构建目标的源集合由 [CMakeLists.txt](../../project/user/CMakeLists.txt) 的 `PRIMER_APP_SRCS` 与 `PRIMER_VENDOR_SRCS` 显式列出；应用重构的函数等价性边界由 [compare_function_bodies.py](../compare_function_bodies.py) 的 `active_sources()` 定义。",
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scope", choices=("build", "own"), default="build")
    parser.add_argument("--output-dir", type=pathlib.Path, default=HERE / "reports")
    args = parser.parse_args()
    records, sources, description = discover(args.scope)
    edges, external = build_edges(records)
    report = _json(records, sources, description, args.scope, edges, external)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = f"function_call_graph_{args.scope}"
    json_path = args.output_dir / f"{stem}.json"
    dot_path = args.output_dir / f"{stem}.dot"
    md_path = args.output_dir / f"{stem}.md"
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    dot_path.write_text(_dot(report), encoding="utf-8")
    md_path.write_text(_markdown(report, dot_path.name, json_path.name), encoding="utf-8")
    print(f"scope={args.scope}")
    print(f"sources={len(sources)} functions={len(records)} edges={len(edges)} external_nodes={len(external)}")
    print(f"internal={report['edge_counts']['internal']} ambiguous={report['edge_counts']['ambiguous']} external={report['edge_counts']['external']}")
    print(f"wrote {json_path} {dot_path} {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

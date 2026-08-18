"""Paper-facing tables and figures for PACE-Bench.

The JSON report is still the canonical data product.  This module mirrors the
layout and visual conventions used by the DaVinciBench/EMNLP2026 analysis:
full-width ``table*`` fragments, ``booktabs`` rules, ``resizebox`` for wide
tables, short category labels, blue heatmap cells, and consistent matplotlib
settings.  All Pass@2 values are computed from the original result records so
independent runs are grouped by benchmark pair before rendering.
"""

from __future__ import annotations

import math
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

from pace_bench.evaluation.metrics import pass_at_k_by_group

PASS_K = 2
MODEL_ORDER = ("Qwen3-4B", "Qwen3-8B", "Qwen3-14B", "Qwen3-32B")
CATEGORY_ORDER = (
    "Category1_Statics_Equilibrium",
    "Category2_Kinematics_Linkages",
    "Category3_Dynamics_Energy",
    "Category4_Granular_FluidInteraction",
    "Category5_Cybernetics_Control",
    "Category6_ExoticPhysics",
)
CATEGORY_SHORT = {
    "Category1_Statics_Equilibrium": "S",
    "Category2_Kinematics_Linkages": "K",
    "Category3_Dynamics_Energy": "D",
    "Category4_Granular_FluidInteraction": "F",
    "Category5_Cybernetics_Control": "C",
    "Category6_ExoticPhysics": "E",
}
METHOD_DISPLAY = {
    "vanilla": "Vanilla",
    "baseline": "Vanilla",
    "reflexion": "Reflexion",
    "self_refine": "Self-Refine",
    "ace": "ACE",
    "expel": "ExpeL",
    "reasoning_bank": "ReasoningBank",
    "tree_of_thoughts": "ToT",
    "codeevolve": "CodeEvolve",
    "ttt_discover": "TTT-Discover",
    "ragen": "RAGEN",
    "seal": "SEAL",
}
METHOD_PARADIGM = {
    "vanilla": "Context",
    "baseline": "Context",
    "reflexion": "Context",
    "self_refine": "Context",
    "ace": "Memory",
    "expel": "Memory",
    "reasoning_bank": "Memory",
    "tree_of_thoughts": "Search",
    "codeevolve": "Search",
    "ttt_discover": "Parameter",
    "ragen": "Parameter",
    "seal": "Parameter",
}
PARADIGM_ORDER = ("Context", "Memory", "Search", "Parameter", "New")
BLUE_PALETTE = ("#deebf7", "#bdd7e7", "#9ecae1", "#6baed6", "#3182bd", "#08519c")
HEATMAP_COLORS = ("#F7FBFF", "#DEEBF7", "#C6DBEF", "#9ECAE1", "#6BAED6", "#4292C6", "#2171B5", "#08519C")
ERROR_COLORS = ("#D73027", "#FC8D59", "#FEE090", "#91BFDB", "#4575B4", "#313695", "#74ADD1", "#B0B0B0", "#5E4FA2")
ERROR_ORDER = (
    "constraint_violation",
    "structural_failure",
    "design_fixation",
    "stagnation",
    "exploration",
    "late_convergence",
    "budget_exhaustion",
    "numerical_instability",
    "catastrophic_collapse",
)


def _finite(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _number(value: Any, digits: int = 2) -> str:
    number = _finite(value)
    return "--" if number is None else f"{number:.{digits}f}"


def _percent(value: Any, digits: int = 1) -> str:
    number = _finite(value)
    return "--" if number is None else f"{number:.{digits}f}\\%"


def _pretty_name(value: Any) -> str:
    text = str(value)
    for number in range(1, 7):
        text = text.replace(f"Category{number}_", f"Category {number}: ")
    return text.replace("_", " ")


def _category_name(value: Any) -> str:
    return CATEGORY_SHORT.get(str(value), _pretty_name(value))


def _method_name(value: Any) -> str:
    return METHOD_DISPLAY.get(str(value), _pretty_name(value))


def _model_names(report: dict[str, Any]) -> list[str]:
    present = set((report.get("by_model") or {}).keys())
    return [model for model in MODEL_ORDER if model in present] + sorted(present - set(MODEL_ORDER))


def _category_names(report: dict[str, Any]) -> list[str]:
    present = set((report.get("by_category") or {}).keys())
    return [category for category in CATEGORY_ORDER if category in present] + sorted(present - set(CATEGORY_ORDER))


def _strategy_names(report: dict[str, Any]) -> list[str]:
    present = set((report.get("by_strategy") or {}).keys())
    return sorted(
        present,
        key=lambda name: (PARADIGM_ORDER.index(METHOD_PARADIGM.get(name, "New")), name),
    )


def _latex_text(value: Any) -> str:
    replacements = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    return "".join(replacements.get(char, char) for char in str(value))


def _heatmap_hex(value: float) -> str | None:
    anchors = (
        (0, 0xF5, 0xF8, 0xFC),
        (8, 0xDE, 0xEB, 0xF6),
        (16, 0xC0, 0xD8, 0xEC),
        (24, 0x98, 0xBF, 0xDE),
        (34, 0x64, 0x9F, 0xCA),
        (50, 0x30, 0x77, 0xB0),
        (100, 0x08, 0x51, 0x9C),
    )
    if value <= 0:
        return None
    for left, right in zip(anchors, anchors[1:]):
        v0, r0, g0, b0 = left
        v1, r1, g1, b1 = right
        if value <= v1:
            fraction = (value - v0) / (v1 - v0)
            return f"{int(r0 + (r1 - r0) * fraction):02X}{int(g0 + (g1 - g0) * fraction):02X}{int(b0 + (b1 - b0) * fraction):02X}"
    return "08519C"


def _heatmap_cell(value: float | None, delta: float | None = None) -> str:
    if value is None:
        return "--"
    color = _heatmap_hex(value)
    cell = f"{value:.1f}" if color is None else f"\\cellcolor[HTML]{{{color}}}{value:.1f}"
    if delta is None:
        return cell
    arrow = r"$\uparrow$" if delta >= 0 else r"$\downarrow$"
    return f"{cell}\\,({{\\scriptsize {arrow}{abs(delta):.1f}}})"


def _write_table(
    path: Path,
    *,
    caption: str,
    label: str,
    headers: list[str],
    rows: Iterable[Iterable[Any]],
    numeric_columns: set[int] | None = None,
    resize: bool = True,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    numeric_columns = numeric_columns or set()
    alignment = "".join("c" if index in numeric_columns else "l" for index in range(len(headers)))
    lines = [
        r"\begin{table*}[t]",
        r"    \centering",
        f"    \\caption{{{caption}}}",
        f"    \\label{{{label}}}",
    ]
    if resize:
        lines.append(r"    \resizebox{\textwidth}{!}{%")
    lines.extend(
        [
            f"    \\begin{{tabular}}{{{alignment}}}",
            r"        \toprule",
            "        " + " & ".join(f"\\textbf{{{_latex_text(header)}}}" for header in headers) + " \\\\",
            r"        \midrule",
        ]
    )
    for row in rows:
        rendered = [
            str(value) if index in numeric_columns else _latex_text(value)
            for index, value in enumerate(row)
        ]
        lines.append("        " + " & ".join(rendered) + " \\\\")
    lines.extend([r"        \bottomrule", r"    \end{tabular}"])
    if resize:
        lines.append(r"    }")
    lines.extend([r"\end{table*}", ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def _group_pass_at_k(results: Any, key_function: Any, fallback: dict[str, Any] | None = None) -> dict[str, dict[str, float | int]]:
    return pass_at_k_by_group(results, key_function, PASS_K) if results else (fallback or {})


def _category_from_result(result: Any) -> str:
    return result.task_path.split("/", 1)[0] if "/" in result.task_path else result.task_id


def _stage_from_result(result: Any) -> str:
    pair = result.environment_pair or result.target_environment or "unknown"
    return pair.split("_to_", 1)[-1]


def _write_main_table(report: dict[str, Any], path: Path, results: Any) -> None:
    models = _model_names(report)
    strategies = _strategy_names(report)
    if not models or not strategies:
        return
    by_strategy_model = report.get("by_strategy_and_model") or {}
    pass_groups = _group_pass_at_k(
        results,
        lambda result: f"{result.strategy}|||{result.model}",
    )
    baseline = "vanilla" if "vanilla" in strategies else strategies[0]
    show_delta = len(strategies) > 1

    lines = [
        r"\begin{table*}[t]",
        r"  \centering",
        r"  \small",
        r"  \caption{Main Results: Pass@2 (\%) and Average Score by Method and Model."
        + (r" Values in parentheses show $\Delta$ vs.\ Vanilla ($\uparrow$ = improvement, $\downarrow$ = degradation).}" if show_delta else "}"),
        r"  \label{tab:main_results}",
    ]
    if len(models) > 3:
        lines.append(r"  \resizebox{\textwidth}{!}{%")
    lines.extend(
        [
            "  \\begin{tabular}{ll" + "cc" * len(models) + "}",
            r"  \toprule",
            "  & & " + " & ".join(
                f"\\multicolumn{{2}}{{c}}{{\\textbf{{{_latex_text(model)}}}}}" for model in models
            ) + " \\\\",
            "  " + " ".join(
                f"\\cmidrule(lr){{{3 + index * 2}-{4 + index * 2}}}" for index in range(len(models))
            ),
            "  \\textbf{Paradigm} & \\textbf{Method} & "
            + " & ".join(r"\textbf{Pass@2} & \textbf{Score}" for _ in models)
            + " \\\\",
            r"  \midrule",
        ]
    )

    for paradigm in PARADIGM_ORDER:
        methods = [method for method in strategies if METHOD_PARADIGM.get(method, "New") == paradigm]
        if not methods:
            continue
        if any(line == r"  \midrule" for line in lines[-1:]) is False and len(lines) > 10:
            lines.append(r"  \midrule")
        for index, method in enumerate(methods):
            paradigm_cell = (
                f"\\multirow{{{len(methods)}}}{{*}}{{{paradigm}}}" if index == 0 and len(methods) > 1 else (paradigm if index == 0 else "")
            )
            cells = []
            for model in models:
                group = (by_strategy_model.get(method) or {}).get(model) or {}
                pass_value = pass_groups.get(f"{method}|||{model}", {}).get("rate")
                if pass_value is None and not results:
                    pass_value = group.get("pass_rate")
                pass_percent = None if pass_value is None else float(pass_value) * 100.0
                score = _finite(group.get("score_mean"))
                delta_pass = None
                delta_score = None
                if show_delta:
                    base_group = (by_strategy_model.get(baseline) or {}).get(model) or {}
                    base_pass = pass_groups.get(f"{baseline}|||{model}", {}).get("rate")
                    if base_pass is None and not results:
                        base_pass = base_group.get("pass_rate")
                    if pass_percent is not None and base_pass is not None:
                        delta_pass = pass_percent - float(base_pass) * 100.0
                    if score is not None and _finite(base_group.get("score_mean")) is not None:
                        delta_score = score - float(base_group["score_mean"])
                cells.extend([_heatmap_cell(pass_percent, delta_pass), _heatmap_cell(score, delta_score)])
            method_cell = _method_name(method)
            lines.append(f"  {paradigm_cell} & {method_cell} & " + " & ".join(cells) + " \\\\")

    lines.extend([r"  \bottomrule", r"  \end{tabular}"])
    if len(models) > 3:
        lines.append(r"  }")
    lines.extend([r"\end{table*}", ""])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def _write_group_analysis_table(
    report: dict[str, Any],
    path: Path,
    *,
    report_key: str,
    caption: str,
    label: str,
    results: Any,
    key_function: Any,
    display_function: Any,
) -> None:
    groups = report.get(report_key) or {}
    pass_groups = _group_pass_at_k(results, key_function)
    rows = []
    for group, values in groups.items():
        pass_value = pass_groups.get(str(group), {}).get("rate")
        if pass_value is None:
            pass_value = values.get("pass_rate")
        rows.append(
            [
                display_function(group),
                _percent(float(pass_value) * 100.0),
                _number(values.get("score_mean")),
                _number(values.get("attempts_used_mean")),
                _number(values.get("best_code_tokens_mean"), 1),
            ]
        )
    _write_table(
        path,
        caption=caption,
        label=label,
        headers=["Group", "Pass@2", "Avg. Score", "Iterations", "Code tokens"],
        rows=rows,
        numeric_columns={1, 2, 3, 4},
    )


def _write_model_category_table(report: dict[str, Any], path: Path, results: Any) -> None:
    models = _model_names(report)
    categories = _category_names(report)
    pass_groups = _group_pass_at_k(results, lambda result: f"{result.model}|||{_category_from_result(result)}")
    rows = []
    for model in models:
        cells = []
        for category in categories:
            value = pass_groups.get(f"{model}|||{category}", {}).get("rate")
            if value is None:
                value = ((report.get("by_model_and_category") or {}).get(model, {}).get(category) or {}).get("pass_rate")
            cells.append(_number(float(value) * 100.0, 1) if value is not None else "--")
        rows.append([model] + cells)
    _write_table(
        path,
        caption="Pass@2 (\\%) by Model and Category",
        label="tab:model_category_pass",
        headers=["Model"] + [_category_name(category) for category in categories],
        rows=rows,
        numeric_columns=set(range(1, len(categories) + 1)),
    )


def _write_overall_metrics(report: dict[str, Any], path: Path) -> None:
    metrics = report.get("metrics") or {}
    tokens = metrics.get("tokens") or {}
    rows = [
        ["Result trajectories", str(report.get("trajectory_count", 0))],
        ["Benchmark pairs", str(report.get("pair_count", 0))],
        ["Pass rate (any run)", _percent(float(report.get("pass_rate", 0.0)) * 100.0)],
        ["Mean best score", _number(report.get("mean_best_score"))],
        ["Mean verified attempts", _number(report.get("mean_verified_attempts"))],
        ["Adaptation efficiency", _percent(float(metrics.get("adaptation_efficiency", 0.0)) * 100.0)],
        ["Mean tokens per pair", _number(tokens.get("mean_per_pair"), 0)],
    ]
    for key, value in sorted((report.get("pass_at_k") or {}).items(), key=lambda item: int(item[0].split("@")[-1])):
        rows.append([key, _percent(float(value.get("rate", 0.0)) * 100.0)])
    _write_table(
        path,
        caption="Overall PACE-Bench metrics.",
        label="tab:pace_overall_metrics",
        headers=["Metric", "Value"],
        rows=rows,
        numeric_columns={1},
        resize=False,
    )


def _write_error_table(report: dict[str, Any], path: Path) -> None:
    taxonomy = report.get("error_taxonomy") or {}
    counts = taxonomy.get("counts") or {}
    rates = taxonomy.get("rates") or {}
    failure_rates = taxonomy.get("failure_only_rates") or {}
    names = [name for name in ERROR_ORDER if name in counts] + sorted(set(counts) - set(ERROR_ORDER))
    rows = [
        [
            _pretty_name(name),
            str(int(counts[name])),
            _percent(float(rates.get(name, 0.0)) * 100.0),
            _percent(float(failure_rates[name]) * 100.0) if name in failure_rates else "--",
        ]
        for name in names
    ]
    _write_table(
        path,
        caption="Error Type Distribution (\\%) by Pair",
        label="tab:error_taxonomy",
        headers=["Outcome / Error", "Pairs", "All pairs", "Failed pairs"],
        rows=rows,
        numeric_columns={1, 2, 3},
    )


def _write_budget_table(report: dict[str, Any], path: Path) -> None:
    points = ((report.get("budget_sensitivity") or {}).get("overall") or {}).get("points") or {}
    attempts = sorted(points, key=lambda value: int(value))
    rows = [
        [
            str(attempt),
            _percent(float(points[attempt].get("rate", 0.0)) * 100.0),
            _percent(float(points[attempt].get("saturation_of_final", 0.0)) * 100.0),
        ]
        for attempt in attempts
    ]
    _write_table(
        path,
        caption="Budget Sensitivity: Pair discovery rate at varying budgets",
        label="tab:budget_sensitivity",
        headers=["Budget", "Discovery rate", "Saturated"],
        rows=rows,
        numeric_columns={0, 1, 2},
    )


def _configure_plt(plt: Any) -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "savefig.bbox": "tight",
            "savefig.pad_inches": 0.10,
            "font.family": "sans-serif",
            "font.sans-serif": ["Arial", "Helvetica", "Liberation Sans", "Nimbus Sans", "DejaVu Sans", "sans-serif"],
            "mathtext.fontset": "dejavusans",
            "font.size": 18,
            "axes.labelsize": 18,
            "axes.titlesize": 20,
            "axes.titleweight": "semibold",
            "axes.linewidth": 1.2,
            "axes.edgecolor": "#2C2C2C",
            "axes.labelcolor": "#2C2C2C",
            "xtick.labelsize": 15,
            "ytick.labelsize": 15,
            "xtick.direction": "out",
            "ytick.direction": "out",
            "xtick.major.width": 0.8,
            "ytick.major.width": 0.8,
            "legend.frameon": True,
            "legend.framealpha": 0.92,
            "legend.edgecolor": "#CCCCCC",
            "legend.fontsize": 15,
            "grid.color": "#B0B0B0",
            "grid.linestyle": "-",
            "grid.linewidth": 0.4,
            "grid.alpha": 0.5,
        }
    )


def _load_matplotlib() -> Any:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return None
    _configure_plt(plt)
    return plt


def _style_axes(ax: Any) -> None:
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color("#2C2C2C")
    ax.spines["bottom"].set_color("#2C2C2C")
    ax.set_axisbelow(True)
    ax.yaxis.grid(True)


def _save_figure(plt: Any, fig: Any, directory: Path, name: str, aliases: tuple[str, ...] = ()) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for output_name in (name,) + aliases:
        for extension in ("pdf", "png"):
            fig.savefig(directory / f"{output_name}.{extension}", facecolor="white", bbox_inches="tight", pad_inches=0.10, dpi=300 if extension == "png" else None)
    plt.close(fig)


def _plot_pass_by_group(plt: Any, groups: list[str], pass_groups: dict[str, Any], directory: Path, name: str, aliases: tuple[str, ...] = ()) -> None:
    values = [float(pass_groups.get(group, {}).get("rate", 0.0)) * 100.0 for group in groups]
    # Keep model names legible when the figure is included at column width.
    # The reference paper rotates long categorical labels instead of letting
    # them collide at the bottom of the axes.
    is_model_plot = name == "bar_pass_by_model"
    fig, ax = plt.subplots(figsize=(6.5 if is_model_plot else 5.5, 4), facecolor="white")
    ax.set_facecolor("white")
    ax.bar(range(len(groups)), values, 0.55, color=[BLUE_PALETTE[min(i, len(BLUE_PALETTE) - 1)] for i in range(len(groups))], edgecolor="white", linewidth=0.3, zorder=3)
    ax.set_xticks(range(len(groups)), groups, rotation=25 if is_model_plot else 0, ha="right" if is_model_plot else "center")
    ax.set_ylabel("Pass@2 (%)")
    ax.set_ylim(0, max(values, default=0.0) * 1.15 + 1)
    _style_axes(ax)
    fig.tight_layout()
    _save_figure(plt, fig, directory, name, aliases)


def _plot_pass_by_model(plt: Any, report: dict[str, Any], results: Any, directory: Path) -> None:
    models = _model_names(report)
    pass_groups = _group_pass_at_k(results, lambda result: result.model)
    _plot_pass_by_group(plt, models, pass_groups, directory, "bar_pass_by_model", ("pass_rate_by_model",))


def _plot_pass_by_category(plt: Any, report: dict[str, Any], results: Any, directory: Path) -> None:
    categories = _category_names(report)
    pass_groups = _group_pass_at_k(results, _category_from_result)
    display_groups = [_category_name(category) for category in categories]
    display_values = { _category_name(category): pass_groups.get(category, {}) for category in categories }
    _plot_pass_by_group(plt, display_groups, display_values, directory, "bar_pass_by_category", ("pass_rate_by_category",))


def _plot_model_category_heatmap(plt: Any, report: dict[str, Any], results: Any, directory: Path) -> None:
    models = _model_names(report)
    categories = _category_names(report)
    pass_groups = _group_pass_at_k(results, lambda result: f"{result.model}|||{_category_from_result(result)}")
    matrix = [[float(pass_groups.get(f"{model}|||{category}", {}).get("rate", 0.0)) * 100.0 for category in categories] for model in models]
    if not matrix:
        return
    from matplotlib.colors import LinearSegmentedColormap

    fig, ax = plt.subplots(figsize=(len(categories) * 1.0 + 3.5, len(models) * 0.6 + 1.8), facecolor="white")
    ax.set_facecolor("#FAFAFA")
    image = ax.imshow(matrix, cmap=LinearSegmentedColormap.from_list("pace_blues", HEATMAP_COLORS, N=256), vmin=0, vmax=100, aspect="auto", interpolation="nearest")
    ax.set_xticks(range(len(categories)), [_category_name(category) for category in categories])
    ax.set_yticks(range(len(models)), models)
    _style_axes(ax)
    for row_index, row in enumerate(matrix):
        for column_index, value in enumerate(row):
            color = "white" if value > 55 else "#2C2C2C"
            ax.text(column_index, row_index, f"{value:.1f}", ha="center", va="center", color=color, fontsize=15)
    colorbar = fig.colorbar(image, ax=ax, shrink=0.95, pad=0.03, aspect=30)
    colorbar.set_label("Pass@2 (%)")
    colorbar.ax.tick_params(labelsize=8)
    fig.tight_layout(rect=(0.05, 0.04, 0.96, 0.96))
    _save_figure(plt, fig, directory, "heatmap_model_category_pass", ("model_category_pass_rate",))


def _plot_stage_degradation(plt: Any, report: dict[str, Any], results: Any, directory: Path) -> None:
    models = _model_names(report)
    stages = sorted((report.get("by_stage") or {}).keys(), key=lambda value: int(value.split("-")[-1]) if "-" in value else 99)
    pass_groups = _group_pass_at_k(results, lambda result: f"{result.model}|||{_stage_from_result(result)}")
    n_models = len(models)
    bar_width = 0.8 / max(n_models, 1)
    fig, ax = plt.subplots(figsize=(6.5, 5), facecolor="white")
    ax.set_facecolor("white")
    for model_index, model in enumerate(models):
        values = [float(pass_groups.get(f"{model}|||{stage}", {}).get("rate", 0.0)) * 100.0 for stage in stages]
        positions = [index + (model_index - (n_models - 1) / 2) * bar_width for index in range(len(stages))]
        ax.bar(positions, values, bar_width, label=model, color=BLUE_PALETTE[min(model_index, len(BLUE_PALETTE) - 1)], edgecolor="white", linewidth=0.3, zorder=3)
    ax.set_xticks(range(len(stages)), stages)
    ax.set_ylabel("Pass@2 (%)")
    ax.set_xlabel("Mutation Stage")
    _style_axes(ax)
    ax.legend(frameon=True, fontsize=8)
    fig.tight_layout()
    _save_figure(plt, fig, directory, "bar_stage_degradation", ("pass_rate_by_stage",))


def _plot_discovery_and_score(plt: Any, report: dict[str, Any], directory: Path) -> None:
    metrics = report.get("metrics") or {}
    discovery = metrics.get("attempt_discovery") or {}
    scores = metrics.get("score_by_attempt") or {}
    attempts = sorted(set(discovery) & set(scores), key=int)
    if not attempts:
        return
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 4), facecolor="white")
    x = [int(attempt) for attempt in attempts]
    ax1.plot(x, [float(discovery[attempt].get("rate", 0.0)) * 100.0 for attempt in attempts], "o-", color=BLUE_PALETTE[-1], linewidth=2.0, markersize=3.5, label="Overall")
    ax2.plot(x, [float(scores[attempt]) for attempt in attempts], "o-", color=BLUE_PALETTE[-1], linewidth=2.0, markersize=3.5, label="Overall")
    for ax, ylabel in ((ax1, "Pair discovery rate (%)"), (ax2, "Avg. Score")):
        ax.set_xlabel("Iterations")
        ax.set_ylabel(ylabel)
        ax.set_xlim(0.5, max(x) + 0.5)
        ax.set_xticks([attempt for attempt in (1, 5, 10, 15, 20) if attempt <= max(x)])
        _style_axes(ax)
    fig.tight_layout(rect=(0.04, 0.04, 0.99, 0.96))
    _save_figure(plt, fig, directory, "discovery_and_score_curves")


def _failure_share(values: dict[str, Any]) -> dict[str, float]:
    taxonomy = values.get("error_taxonomy") or {}
    rates = taxonomy.get("failure_only_rates") or values.get("failure_only_rates") or {}
    return {name: float(rates.get(name, 0.0)) * 100.0 for name in ERROR_ORDER if name in rates}


def _plot_error_combined(plt: Any, report: dict[str, Any], directory: Path) -> None:
    categories = _category_names(report)
    models = _model_names(report)
    by_category = report.get("by_category") or {}
    by_model = report.get("by_model") or {}
    error_names = [name for name in ERROR_ORDER if any(name in _failure_share(values) for values in list(by_category.values()) + list(by_model.values()))]
    if not error_names:
        return
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5), facecolor="white")
    for ax, groups, values_map, labels, xlabel in (
        (ax1, categories, by_category, [_category_name(category) for category in categories], "Category"),
        (ax2, models, by_model, models, "Model"),
    ):
        bottoms = [0.0] * len(groups)
        for error_index, error_name in enumerate(error_names):
            values = [_failure_share(values_map.get(group) or {}).get(error_name, 0.0) for group in groups]
            ax.bar(range(len(groups)), values, 0.6, bottom=bottoms, label=_pretty_name(error_name), color=ERROR_COLORS[error_index % len(ERROR_COLORS)], edgecolor="white", linewidth=0.3, zorder=3)
            bottoms = [left + value for left, value in zip(bottoms, values)]
        ax.set_xticks(range(len(groups)), labels, rotation=35 if ax is ax2 else 0, ha="right" if ax is ax2 else "center")
        ax.set_ylabel("Error share (%)")
        ax.set_xlabel(xlabel)
        ax.set_ylim(0, min(max(bottoms, default=0.0) * 1.08, 105))
        _style_axes(ax)
    handles, labels = ax1.get_legend_handles_labels()
    fig.legend(handles, labels, frameon=True, fontsize=14, loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.04))
    fig.tight_layout(rect=(0.02, 0.02, 0.98, 0.85))
    _save_figure(plt, fig, directory, "bar_error_combined", ("error_taxonomy",))


def _plot_budget_sensitivity(plt: Any, report: dict[str, Any], directory: Path) -> None:
    budget = report.get("budget_sensitivity") or {}
    points = (budget.get("overall") or {}).get("points") or {}
    attempts = sorted(points, key=int)
    if not attempts:
        return
    fig, ax = plt.subplots(figsize=(6, 5), facecolor="white")
    categories = _category_names(report)
    colors = [BLUE_PALETTE[index % len(BLUE_PALETTE)] for index in range(len(categories))]
    for index, category in enumerate(categories):
        category_points = ((budget.get("by_category") or {}).get(category) or {}).get("points") or {}
        x = [int(attempt) for attempt in attempts if attempt in category_points]
        y = [float(category_points[str(attempt)].get("rate", 0.0)) * 100.0 for attempt in x]
        if x:
            ax.plot(x, y, "o-", label=_category_name(category), color=colors[index], linewidth=2.0, markersize=6, markerfacecolor=colors[index], markeredgecolor="none", zorder=3)
    ax.set_xlabel("Budget (max iterations)")
    ax.set_ylabel("Pair discovery rate (%)")
    ax.set_xticks([int(attempt) for attempt in attempts])
    _style_axes(ax)
    ax.legend(frameon=True, fontsize=8)
    fig.tight_layout()
    _save_figure(plt, fig, directory, "line_budget_sensitivity")


def export_paper_artifacts(report: dict[str, Any], table_dir: Path, figure_dir: Path, results: Any = None) -> dict[str, Any]:
    """Write DaVinci/EMNLP-style tables and figures from an aggregate report."""

    table_dir = Path(table_dir)
    figure_dir = Path(figure_dir)
    table_dir.mkdir(parents=True, exist_ok=True)
    figure_dir.mkdir(parents=True, exist_ok=True)

    main_dir = table_dir / "main"
    error_dir = table_dir / "error"
    difficulty_dir = table_dir / "difficulty"
    _write_main_table(report, main_dir / "main_results.tex", results)
    _write_model_category_table(report, main_dir / "model_category_pass.tex", results)
    _write_group_analysis_table(report, main_dir / "category_pass.tex", report_key="by_category", caption="Pass@2 (\\%) by Category", label="tab:category_pass", results=results, key_function=_category_from_result, display_function=_category_name)
    _write_group_analysis_table(report, main_dir / "stage_pass.tex", report_key="by_stage", caption="Pass@2 (\\%) by Target Stage", label="tab:stage_pass", results=results, key_function=_stage_from_result, display_function=str)
    _write_group_analysis_table(report, main_dir / "model_pass.tex", report_key="by_model", caption="Pass@2 (\\%) by Model", label="tab:model_pass2", results=results, key_function=lambda result: result.model, display_function=str)
    _write_overall_metrics(report, main_dir / "overall_metrics.tex")
    _write_error_table(report, error_dir / "error_taxonomy.tex")
    _write_budget_table(report, difficulty_dir / "budget_sensitivity.tex")

    # Keep the short flat paths used by the first exporter as paper-friendly
    # aliases, while the nested tree mirrors DaVinciBench's organization.
    aliases = {
        main_dir / "main_results.tex": table_dir / "main_results.tex",
        main_dir / "model_category_pass.tex": table_dir / "model_category_pass_rate.tex",
        main_dir / "category_pass.tex": table_dir / "category_results.tex",
        main_dir / "stage_pass.tex": table_dir / "stage_results.tex",
        main_dir / "model_pass.tex": table_dir / "model_results.tex",
        main_dir / "overall_metrics.tex": table_dir / "overall_metrics.tex",
        error_dir / "error_taxonomy.tex": table_dir / "error_taxonomy.tex",
        difficulty_dir / "budget_sensitivity.tex": table_dir / "budget_sensitivity.tex",
    }
    for source, destination in aliases.items():
        destination.write_text(source.read_text(encoding="utf-8"), encoding="utf-8")

    # These are derived files from the previous generic exporter.  Remove only
    # those exact names so stale plots cannot be mistaken for current paper
    # artifacts; user-created files with other names are untouched.
    for legacy_name in ("pass_rate_by_attempt", "score_by_attempt"):
        for extension in ("pdf", "png"):
            (figure_dir / f"{legacy_name}.{extension}").unlink(missing_ok=True)

    plt = _load_matplotlib()
    if plt is None:
        return {"tables": 8, "figures": 0, "figure_error": "matplotlib is not installed"}
    _plot_pass_by_model(plt, report, results, figure_dir)
    _plot_pass_by_category(plt, report, results, figure_dir)
    _plot_model_category_heatmap(plt, report, results, figure_dir)
    _plot_stage_degradation(plt, report, results, figure_dir)
    _plot_discovery_and_score(plt, report, figure_dir)
    _plot_error_combined(plt, report, figure_dir)
    _plot_budget_sensitivity(plt, report, figure_dir)
    return {"tables": 8, "figures": 7}

#!/usr/bin/env python3
"""
LTSP Dashboard — Unicode box-drawing two-column layout with braille phase chart.

No external dependencies. Imported by ltsp_integration_test.py.
"""

import math
import os
import shutil

# ---------------------------------------------------------------------------
# Braille chart renderer
# ---------------------------------------------------------------------------

# Unicode braille block: 2 columns x 4 rows per character cell
# Dot numbering (standard braille):
#   1 4
#   2 5
#   3 6
#   7 8
BRAILLE_BASE = 0x2800
BRAILLE_DOTS = [
    [0x01, 0x08],  # row 0: dots 1, 4
    [0x02, 0x10],  # row 1: dots 2, 5
    [0x04, 0x20],  # row 2: dots 3, 6
    [0x40, 0x80],  # row 3: dots 7, 8
]


def render_braille_chart(data, width=35, height=4):
    """Render a scatter plot using Unicode braille characters.

    Args:
        data: list of float values (phase in µs)
        width: chart width in character cells
        height: chart height in character cells

    Returns:
        list of strings (one per line), including Y-axis labels and bottom axis.
        Each line is exactly (6 + width) characters wide:
          6 chars for Y-axis label + `width` chars for braille.
    """
    if not data or len(data) < 2:
        return [" " * (6 + width)] * height + [" " * (6 + width)]

    # Pixel grid: each cell is 2 wide x 4 tall
    px_w = width * 2
    px_h = height * 4

    y_min = min(data)
    y_max = max(data)
    y_range = y_max - y_min
    if y_range < 1e-9:
        y_range = 1.0
        y_min -= 0.5
        y_max += 0.5

    # Map data to pixel positions
    n = len(data)
    grid = [[0] * px_w for _ in range(px_h)]

    for i, val in enumerate(data):
        px_x = int(i * (px_w - 1) / (n - 1)) if n > 1 else 0
        # Y: top of grid = y_max, bottom = y_min
        py = (val - y_min) / y_range
        px_y = int((1.0 - py) * (px_h - 1))
        px_x = max(0, min(px_w - 1, px_x))
        px_y = max(0, min(px_h - 1, px_y))
        grid[px_y][px_x] = 1

    # Convert pixel grid to braille characters
    lines = []
    for cy in range(height):
        row_chars = []
        for cx in range(width):
            code = BRAILLE_BASE
            for dy in range(4):
                for dx in range(2):
                    py = cy * 4 + dy
                    px = cx * 2 + dx
                    if py < px_h and px < px_w and grid[py][px]:
                        code |= BRAILLE_DOTS[dy][dx]
            row_chars.append(chr(code))
        lines.append("".join(row_chars))

    # Y-axis labels (top = max, bottom = min)
    result = []
    for i, line in enumerate(lines):
        if i == 0:
            label = f"{y_max:+.0f} "
        elif i == height - 1:
            label = f"{y_min:+.0f} "
        else:
            label = "     "
        result.append(f"{label:>6}{line}")

    # Bottom axis line
    duration_str = _format_duration(n)
    axis = f"{'':>6}" + f"\u2500" * width
    # Replace end with duration label
    if len(duration_str) + 2 < width:
        axis = axis[:-(len(duration_str) + 1)] + " " + duration_str
    result.append(axis)

    return result


def _format_duration(n_samples):
    """Format duration string for chart axis."""
    if n_samples < 120:
        return f"{n_samples} s"
    elif n_samples < 7200:
        return f"{n_samples // 60} min"
    else:
        return f"{n_samples / 3600:.1f} hr"


# ---------------------------------------------------------------------------
# Box drawing helpers
# ---------------------------------------------------------------------------

# Box chars: ┌ ┐ └ ┘ ─ │ ├ ┤ ┬ ┴ ┼
BOX_H = "\u2500"
BOX_V = "\u2502"
BOX_TL = "\u250c"
BOX_TR = "\u2510"
BOX_BL = "\u2514"
BOX_BR = "\u2518"
BOX_LT = "\u251c"  # left tee (├)
BOX_RT = "\u2524"  # right tee (┤)
BOX_TT = "\u252c"  # top tee (┬)
BOX_BT = "\u2534"  # bottom tee (┴)
BOX_CROSS = "\u253c"  # cross (┼)


def _hline(width, left, right, fill=BOX_H):
    """Horizontal line: left + fill*(width-2) + right"""
    return left + fill * (width - 2) + right


def _hline_title(width, left, right, title, fill=BOX_H):
    """Horizontal line with centered title."""
    title_str = f" {title} "
    pad = width - 2 - len(title_str)
    left_pad = pad // 2
    right_pad = pad - left_pad
    return left + fill * left_pad + title_str + fill * right_pad + right


def _pad_line(text, width):
    """Pad or truncate text to exactly width chars (between box borders)."""
    inner = width - 2  # space between │ and │
    text = text[:inner]
    return BOX_V + text.ljust(inner) + BOX_V


def _pad_left(text, lw):
    """Pad text for left panel (no right border)."""
    inner = lw - 1  # space after │
    text = text[:inner]
    return BOX_V + text.ljust(inner)


def _pad_right(text, rw):
    """Pad text for right panel (no left border)."""
    inner = rw - 1  # space before │
    text = text[:inner]
    return text.ljust(inner) + BOX_V


# ---------------------------------------------------------------------------
# Panel builders
# ---------------------------------------------------------------------------

def _build_left_lines(harness, phase_measurer, elapsed, duration):
    """Build left panel content lines (no box borders)."""
    lines = []
    gs = harness.gm_stats

    # GM section
    lines.append(f" GM  GPS: {'YES' if gs.locked else 'NO'}  "
                 f"PPS: {gs.pps_count}  "
                 f"Crystal: {gs.crystal_err_ns:+d} ns")
    if gs.regression_valid:
        lines.append(f"     Regression: a1={gs.a1_ppb:+.3f} ppb  "
                     f"sigma={gs.sigma_ns:.1f} ns")
    tx = harness.tx_latency_stats
    if tx.n > 0:
        lines.append(f"     TX latency: mean={tx.mean:+.0f} ns  std={tx.std:.0f} ns")

    lines.append(None)  # separator

    # RX section
    total_rx = harness.d_total_stats.n
    lines.append(f" RX  Packets: {total_rx}  "
                 f"Gaps: {harness.rx_seq_gaps}  "
                 f"State: {harness.last_sync_state}")
    drift_ppm = harness.last_drift_ns_per_s / 1000.0
    lines.append(f"     Drift: {drift_ppm:+.1f} ppm  "
                 f"sigma={harness.last_drift_sigma_ns:.0f} ns")

    lines.append(None)  # separator

    # Jitter section
    if harness.converged:
        ds = harness.d_detrended_stats
        os_ = harness.offset_stats
        lines.append(f" JIT Detrended: mean={ds.mean:+.0f} ns  "
                     f"std={ds.std:.0f} ns  n={ds.n}")
        lines.append(f"     Offset:    mean={os_.mean:+.0f} ns  "
                     f"std={os_.std:.0f} ns  n={os_.n}")
    else:
        lines.append(f" JIT Waiting for convergence (~60 samples)...")

    lines.append(None)  # separator

    # Phase section
    if phase_measurer:
        meas, mean_ns, sigma_ns, min_ns, max_ns = phase_measurer.get_stats()
        n = len(meas)
        if n > 0:
            pp = (max_ns - min_ns) / 1000.0
            lines.append(f" PHASE  n={n}  "
                         f"mean={mean_ns/1000:+.1f} us  "
                         f"sigma={sigma_ns/1000:.1f} us  "
                         f"P-P={pp:.1f} us")

            # Braille chart — last 300 samples (~5 min at 1 Hz)
            phases_us = [m[1] / 1000.0 for m in meas[-300:]]
            chart_lines = render_braille_chart(phases_us, width=35, height=4)
            for cl in chart_lines:
                lines.append(f" {cl}")
        else:
            lines.append(f" PHASE  No measurements yet "
                         f"(no-edge: {phase_measurer.no_edge_count}, "
                         f"errors: {phase_measurer.errors})")

    lines.append(None)  # separator

    # Verdicts
    verdicts = _build_verdicts(harness)
    for v in verdicts:
        lines.append(f" {v}")

    return lines


def _build_verdicts(harness):
    """Build verdict strings."""
    gs = harness.gm_stats
    verdicts = []

    if not harness.converged or harness.d_detrended_stats.n < 10:
        return ["... collecting samples ..."]

    jitter_us = harness.d_detrended_stats.std / 1000.0
    if jitter_us < 100:
        verdicts.append(f"PASS  Jitter sigma = {jitter_us:.1f} us")
    else:
        verdicts.append(f"WARN  Jitter sigma = {jitter_us:.1f} us (high)")

    if harness.rx_seq_gaps == 0:
        verdicts.append("PASS  Zero packet loss")
    else:
        verdicts.append(f"WARN  {harness.rx_seq_gaps} sequence gaps")

    if gs.regression_valid and gs.sigma_ns < 50:
        verdicts.append(f"PASS  GM regression sigma = {gs.sigma_ns:.1f} ns")

    if harness.last_sync_state == "LOCKED":
        verdicts.append("PASS  Sync state = LOCKED")
    elif harness.last_sync_state == "ACQUIRING":
        verdicts.append(f"INFO  Sync state = ACQUIRING")
    else:
        verdicts.append(f"WARN  Sync state = {harness.last_sync_state}")

    return verdicts


def _build_right_lines(gm_lines, rx_lines, max_per_section=10):
    """Build right panel content lines from recent serial output."""
    lines = []

    lines.append(" GM")
    if gm_lines:
        for line in gm_lines[-max_per_section:]:
            lines.append(f" {line}")
    else:
        lines.append(" (no data)")

    lines.append(None)  # separator

    lines.append(" RX")
    if rx_lines:
        for line in rx_lines[-max_per_section:]:
            lines.append(f" {line}")
    else:
        lines.append(" (no data)")

    return lines


# ---------------------------------------------------------------------------
# Main render function
# ---------------------------------------------------------------------------

def render_dashboard(harness, phase_measurer, gm_lines, rx_lines,
                     elapsed, duration, gm_line_count, rx_line_count,
                     run_dir):
    """Render the full dashboard as a string.

    Args:
        harness: TestHarness instance
        phase_measurer: PhaseMeasurer instance (or None)
        gm_lines: list of recent GM serial lines (strings)
        rx_lines: list of recent RX serial lines (strings)
        elapsed: seconds since test start
        duration: total test duration in seconds
        gm_line_count: total GM lines received
        rx_line_count: total RX lines received
        run_dir: output directory path

    Returns:
        Multi-line string for display.
    """
    term_width = shutil.get_terminal_size((160, 40)).columns
    total_w = min(term_width, 160)

    # Single-column mode for narrow terminals
    if total_w < 120:
        return _render_single_column(harness, phase_measurer, elapsed, duration,
                                     gm_line_count, rx_line_count, run_dir, total_w)

    return _render_two_column(harness, phase_measurer, gm_lines, rx_lines,
                              elapsed, duration, gm_line_count, rx_line_count,
                              run_dir, total_w)


def _render_two_column(harness, phase_measurer, gm_lines, rx_lines,
                       elapsed, duration, gm_line_count, rx_line_count,
                       run_dir, total_w):
    """Render two-column layout with box drawing."""
    lw = total_w // 2
    rw = total_w - lw

    out = []

    # Top border with titles
    left_top = BOX_TL + _center_title("Stats", lw - 2) + BOX_TT
    right_top = _center_title("Raw Data", rw - 2) + BOX_TR
    out.append(left_top + right_top)

    # Header line
    title = " LTSP Test Harness v0.5"
    elapsed_str = f"{elapsed:.0f}s elapsed "
    inner_lw = lw - 2  # content width inside left panel
    gap = inner_lw - len(title) - len(elapsed_str)
    header_left = title + " " * max(gap, 1) + elapsed_str
    out.append(_pad_left(header_left, lw) + BOX_V + _pad_right("", rw))

    # Divider under header
    out.append(BOX_LT + BOX_H * (lw - 2) + BOX_CROSS + BOX_H * (rw - 2) + BOX_RT)

    # Build content for both panels
    left_content = _build_left_lines(harness, phase_measurer, elapsed, duration)
    right_content = _build_right_lines(gm_lines, rx_lines)

    # Determine how many right-panel lines we can show
    # (fill to match left panel height)
    max_rows = max(len(left_content), len(right_content))

    # Pad both to same length
    while len(left_content) < max_rows:
        left_content.append("")
    while len(right_content) < max_rows:
        right_content.append("")

    for i in range(max_rows):
        lc = left_content[i]
        rc = right_content[i]

        if lc is None and rc is None:
            # Both separators
            out.append(BOX_LT + BOX_H * (lw - 2) + BOX_CROSS + BOX_H * (rw - 2) + BOX_RT)
        elif lc is None:
            # Left separator, right content continues
            rc_inner = _trunc(rc, rw - 2).ljust(rw - 2)
            out.append(BOX_LT + BOX_H * (lw - 2) + BOX_RT + rc_inner + BOX_V)
        elif rc is None:
            # Left content continues, right separator
            lc_inner = _trunc(lc, lw - 2).ljust(lw - 2)
            out.append(BOX_V + lc_inner + BOX_LT + BOX_H * (rw - 2) + BOX_RT)
        else:
            out.append(_pad_left(lc, lw) + BOX_V + _pad_right(rc, rw))

    # Footer divider (merge columns)
    out.append(BOX_LT + BOX_H * (lw - 2) + BOX_BT + BOX_H * (rw - 2) + BOX_RT)

    # Status bar
    short_dir = _short_path(run_dir)
    status = (f" [{elapsed:.0f}s / {duration}s]  "
              f"GM: {gm_line_count} lines  "
              f"RX: {rx_line_count} lines  "
              f"Output: {short_dir}")
    out.append(_pad_line(status, total_w))

    # Bottom border
    out.append(BOX_BL + BOX_H * (total_w - 2) + BOX_BR)

    return "\n".join(out)


def _render_single_column(harness, phase_measurer, elapsed, duration,
                          gm_line_count, rx_line_count, run_dir, total_w):
    """Fallback single-column layout for narrow terminals."""
    out = []

    out.append(_hline(total_w, BOX_TL, BOX_TR))

    title = " LTSP Test Harness v0.5"
    elapsed_str = f"{elapsed:.0f}s elapsed "
    inner_w = total_w - 2
    gap = inner_w - len(title) - len(elapsed_str)
    header = title + " " * max(gap, 1) + elapsed_str
    out.append(_pad_line(header, total_w))

    out.append(BOX_LT + BOX_H * (total_w - 2) + BOX_RT)

    left_content = _build_left_lines(harness, phase_measurer, elapsed, duration)
    for lc in left_content:
        if lc is None:
            out.append(BOX_LT + BOX_H * (total_w - 2) + BOX_RT)
        else:
            out.append(_pad_line(lc, total_w))

    # Footer
    out.append(BOX_LT + BOX_H * (total_w - 2) + BOX_RT)
    short_dir = _short_path(run_dir)
    status = (f" [{elapsed:.0f}s / {duration}s]  "
              f"GM: {gm_line_count} lines  "
              f"RX: {rx_line_count} lines  "
              f"Output: {short_dir}")
    out.append(_pad_line(status, total_w))
    out.append(BOX_BL + BOX_H * (total_w - 2) + BOX_BR)

    return "\n".join(out)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _center_title(title, width):
    """Center a title in a horizontal line of given width."""
    title_str = f" {title} "
    pad = width - len(title_str)
    left_pad = pad // 2
    right_pad = pad - left_pad
    return BOX_H * left_pad + title_str + BOX_H * right_pad


def _trunc(text, maxlen):
    """Truncate text to maxlen."""
    if len(text) <= maxlen:
        return text
    return text[:maxlen - 1] + "\u2026"


def _short_path(path):
    """Shorten a path for display (show last 2 components)."""
    if not path:
        return ""
    parts = path.rstrip("/").split("/")
    if len(parts) >= 2:
        return "/".join(parts[-2:])
    return path

#!/usr/bin/env python3
"""
mrac_plot.py — figures from a session recorded by mrac_record
════════════════════════════════════════════════════════════════════════
Writes into <out_dir> (default: the recording folder):

Whole session, against time
  mrac_e_mrac.png      tracking error q − q_m per joint
  mrac_Kx.png          position gain Kp and velocity gain Kd per joint
  mrac_Kr.png          reference gain Kr per joint
  mrac_W_hat.png       robust weight norm ‖Ŵ_i‖ per joint
  mrac_W_gravity.png   learned gravity and offset weights of joints 2 and 3
  mrac_lyapunov.png    tracking-error energy eᵀPe per joint
  mrac_tau.png         commanded torque per joint

Training (if the session contains mrac_trainer random poses)
  train_error.png          mean and max |e| per joint for every training pose
  train_error_overall.png  the same, combined over the four joints

Evaluation (evaluation loops from mrac_trainer eval_at or from test_moveit)
  eval_error.png       error over the evaluation loop, one line per evaluation
  eval_summary.png     mean and max |e| per joint for each evaluation
  summary.md           the same numbers as tables

Camera (only if frames were recorded with record_camera:=true)
  <kind>_<n>.gif       video of the first and last evaluation

Parameters (defaults in brackets):
  in_dir     (required)  folder written by mrac_record
  out_dir    [in_dir]
  gif_speed  [8.0]   playback speed relative to simulation time
  gif_fps    [12.5]
  gif_width  [480]

Only reads files; no simulation needs to be running.
════════════════════════════════════════════════════════════════════════
"""

import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402

N_JOINTS = 4
JOINT_LABELS = [f"joint{i+1}" for i in range(N_JOINTS)]
JOINT_COLORS = ["#3b82f6", "#f97316", "#22c55e", "#a855f7"]
BASIS = ["1", "sin(q2)", "sin(q2+q3)", "tanh(q̇1)", "tanh(q̇2)", "tanh(q̇3)", "tanh(q̇4)"]
EVAL_KINDS = ("eval", "eval_loop")
EVAL_COLORS = ["#94a3b8", "#f59e0b", "#dc2626", "#7c3aed", "#059669", "#0ea5e9", "#be185d", "#365314"]


# ── helpers ─────────────────────────────────────────────────────────────────
def repair_time(d):
    """Return the recording as a dict with a usable time axis.

    Time stamps come from the simulation clock (/clock). If that clock stopped
    being delivered part-way through a long session, the stamps freeze. The
    diagnostics are published at a fixed rate, so time is rebuilt from the
    sample count, using the sample period measured while the clock still ran.
    """
    out = {k: d[k] for k in d.files}
    t = out["t_error"]
    steps = np.diff(t)
    if len(steps) == 0 or (steps > 0).mean() > 0.98:
        return out
    period = float(np.median(steps[steps > 0]))
    for key in [k for k in out if k.startswith("t_") and k[2:] in out and len(out[k]) > 1]:
        out[key] = out[key][0] + period * np.arange(len(out[key]))
    n = out["seg_samples"].astype(float)
    good = np.where((out["seg_t1"] - out["seg_t0"] > 1.0) & (n > 100))[0]
    if len(good):
        raw = float(np.median((out["seg_t1"][good] - out["seg_t0"][good]) / n[good]))
        start = out["seg_t0"][0] + raw * np.concatenate(([0.0], np.cumsum(n)[:-1]))
        out["seg_t0"], out["seg_t1"] = start, start + raw * n
    print(f"  note: the simulation clock stopped advancing in this recording; "
          f"time was rebuilt from the sample count ({period*1000:.0f} ms per stored sample)")
    return out


class Session:
    def __init__(self, d):
        self.d = d
        self.t0 = float(d["t_error"][0])
        dur = float(d["t_error"][-1]) - self.t0
        self.scale, self.unit = (60.0, "min") if dur > 900 else (1.0, "s")
        kinds = [str(k) for k in d["seg_kind"]]
        self.segs = [
            dict(kind=kinds[i], number=int(d["seg_number"][i]), t0=float(d["seg_t0"][i]),
                 t1=float(d["seg_t1"][i]), mean=d["seg_mean_e"][i], max=d["seg_max_e"][i],
                 mean_traj=d["seg_mean_traj"][i], max_traj=d["seg_max_traj"][i],
                 final=d["seg_final_e"][i], max_tau=d["seg_max_tau"][i], n=int(d["seg_samples"][i]))
            for i in range(len(kinds))
        ]
        self.train = [s for s in self.segs if s["kind"] == "train" and s["n"] > 0]
        self.evals = [s for s in self.segs if s["kind"] in EVAL_KINDS and s["n"] > 0]

    def x(self, t):
        return (np.asarray(t) - self.t0) / self.scale

    def xlabel(self):
        return f"Time ({self.unit})"

    def eval_label(self, s):
        if s["kind"] == "eval_loop":
            return f"loop {s['number']}"
        return "before training" if s["number"] == 0 else f"after {s['number']} training poses"

    def mark(self, ax):
        """Shade evaluation segments and label them."""
        for s in self.evals:
            a, b = self.x(s["t0"]), self.x(s["t1"])
            if s["kind"] == "eval":
                ax.axvspan(a, b, color="#fde68a", alpha=0.35, lw=0)
                text = f" eval @{s['number']}"
            else:
                ax.axvline(a, color="k", ls=":", lw=0.8, alpha=0.5)
                text = f" loop {s['number']}"
            ax.text(a, 1.0, text, transform=ax.get_xaxis_transform(), va="top", fontsize=7, alpha=0.75)


def stats_box(ax, lines, color, loc="lower left"):
    x, y, va, ha = {"lower left": (0.02, 0.03, "bottom", "left"),
                    "upper right": (0.98, 0.90, "top", "right")}[loc]
    ax.text(x, y, "\n".join(lines), transform=ax.transAxes, va=va, ha=ha, fontsize=8,
            family="monospace", bbox=dict(boxstyle="round", fc=color, alpha=0.85))


def grid4(title):
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    fig.suptitle(title, fontsize=13, fontweight="bold")
    return fig, list(axes.flat)


def save(fig, out_dir, name):
    fig.tight_layout()
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("  wrote", path)


def rolling_median(y, window):
    y = np.asarray(y, dtype=float)
    half = window // 2
    return np.array([np.median(y[max(0, i - half): i + half + 1]) for i in range(len(y))])


# ── whole-session traces ────────────────────────────────────────────────────
def session_figures(S, out_dir):
    d = S.d
    t = S.x(d["t_error"])
    e = d["error"][:, 0::2]
    fig, axes = grid4("MRAC tracking error   e = q − q_m   (plant vs reference model)")
    for i, ax in enumerate(axes):
        ax.plot(t, e[:, i], color=JOINT_COLORS[i], lw=0.9)
        ax.axhline(0, color="k", ls="--", lw=0.7, alpha=0.4)
        S.mark(ax)
        a = np.abs(e[:, i])
        stats_box(ax, [f"max |e| = {a.max():.4f} rad", f"mean|e| = {a.mean():.4f} rad",
                       f"final   = {e[-1, i]:+.4f} rad"], "#fde68a")
        ax.set_title(JOINT_LABELS[i], fontweight="bold", color=JOINT_COLORS[i])
        ax.set_xlabel(S.xlabel()); ax.set_ylabel("e (rad)"); ax.grid(True, alpha=0.25)
    save(fig, out_dir, "mrac_e_mrac.png")

    t = S.x(d["t_K_hat"])
    K = d["K_hat"]
    fig, axes = grid4("Adaptive feedback gain  K̂x = [Kp, Kd]\n"
                      "K̂̇x = −Γx · s · [q, q̇],   s = p12·e + p22·ė")
    for i, ax in enumerate(axes):
        kp, kd = K[:, 3 * i], K[:, 3 * i + 1]
        ax.plot(t, kp, color=JOINT_COLORS[i], lw=1.2, label="Kp (left axis)")
        ax.set_ylabel("Kp  [Nm/rad]")
        ax2 = ax.twinx()
        ax2.plot(t, kd, color="#475569", lw=1.0, ls="--", label="Kd (right axis)")
        ax2.set_ylabel("Kd  [Nm·s/rad]")
        S.mark(ax)
        stats_box(ax, [f"Kp {kp[0]:+.2f} → {kp[-1]:+.2f}", f"Kd {kd[0]:+.2f} → {kd[-1]:+.2f}"], "#dbeafe")
        h1, l1 = ax.get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, loc="lower right", fontsize=8)
        ax.set_title(JOINT_LABELS[i], fontweight="bold", color=JOINT_COLORS[i])
        ax.set_xlabel(S.xlabel()); ax.grid(True, alpha=0.25)
    save(fig, out_dir, "mrac_Kx.png")

    fig, axes = grid4("Adaptive reference gain  K̂r\n"
                      "K̂̇r = −Γr · s · r   (adapts only while a trajectory executes)")
    for i, ax in enumerate(axes):
        kr = K[:, 3 * i + 2]
        ax.plot(t, kr, color=JOINT_COLORS[i], lw=1.2)
        S.mark(ax)
        stats_box(ax, [f"init  = {kr[0]:.3f}", f"final = {kr[-1]:.3f}", f"Δ     = {kr[-1]-kr[0]:+.3f}"], "#dcfce7")
        ax.set_title(JOINT_LABELS[i], fontweight="bold", color=JOINT_COLORS[i])
        ax.set_xlabel(S.xlabel()); ax.set_ylabel("Kr  [Nm/rad]"); ax.grid(True, alpha=0.25)
    save(fig, out_dir, "mrac_Kr.png")

    t = S.x(d["t_W_hat"])
    W = d["W_hat"]
    nb = W.shape[1] // N_JOINTS
    fig, axes = grid4("Robust weight norm  ‖Ŵ_i‖\nŴ̇ = ΓW · α(q, q̇) · s")
    for i, ax in enumerate(axes):
        wn = np.linalg.norm(W[:, i * nb:(i + 1) * nb], axis=1)
        ax.plot(t, wn, color=JOINT_COLORS[i], lw=1.2)
        S.mark(ax)
        stats_box(ax, [f"final = {wn[-1]:.3f}", f"max   = {wn.max():.3f}"], "#f3e8ff")
        ax.set_title(JOINT_LABELS[i], fontweight="bold", color=JOINT_COLORS[i])
        ax.set_xlabel(S.xlabel()); ax.set_ylabel("‖Ŵ‖  [Nm]"); ax.grid(True, alpha=0.25)
    save(fig, out_dir, "mrac_W_hat.png")

    if nb == len(BASIS):
        fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
        fig.suptitle("Learned gravity and offset weights (joints 2 and 3)", fontsize=13, fontweight="bold")
        palette = {0: "#64748b", 1: "#ef4444", 2: "#0ea5e9"}
        for ax, j, terms in ((axes[0], 1, [0, 1, 2]), (axes[1], 2, [0, 2])):
            for b in terms:
                ax.plot(t, W[:, j * nb + b], lw=1.2, color=palette[b], label=f"weight on {BASIS[b]}")
            S.mark(ax)
            ax.set_title(JOINT_LABELS[j], fontweight="bold", color=JOINT_COLORS[j])
            ax.set_xlabel(S.xlabel()); ax.set_ylabel("weight  [Nm]"); ax.grid(True, alpha=0.25)
            ax.legend(fontsize=8, loc="lower right")
        save(fig, out_dir, "mrac_W_gravity.png")

    t = S.x(d["t_lyapunov_V"])
    V = d["lyapunov_V"]
    fig, ax = plt.subplots(figsize=(11, 5))
    fig.suptitle("Tracking-error energy  eᵀPe  per joint", fontsize=13, fontweight="bold")
    for i in range(N_JOINTS):
        ax.semilogy(t, np.maximum(V[:, i], 1e-12), color=JOINT_COLORS[i], lw=0.8, label=JOINT_LABELS[i])
    S.mark(ax)
    ax.set_xlabel(S.xlabel()); ax.set_ylabel("eᵀPe  (log scale)"); ax.grid(True, which="both", alpha=0.25)
    ax.legend(loc="lower right", fontsize=9)
    save(fig, out_dir, "mrac_lyapunov.png")

    t = S.x(d["t_tau"])
    tau = d["tau"]
    fig, axes = grid4("Commanded joint torque  τ")
    for i, ax in enumerate(axes):
        ax.plot(t, tau[:, i], color=JOINT_COLORS[i], lw=0.8)
        S.mark(ax)
        stats_box(ax, [f"max |τ| = {np.abs(tau[:, i]).max():.2f} Nm"], "#fee2e2")
        ax.set_title(JOINT_LABELS[i], fontweight="bold", color=JOINT_COLORS[i])
        ax.set_xlabel(S.xlabel()); ax.set_ylabel("τ  [Nm]"); ax.grid(True, alpha=0.25)
    save(fig, out_dir, "mrac_tau.png")


# ── training: error per random pose ─────────────────────────────────────────
def training_figures(S, out_dir):
    if not S.train:
        return
    n = np.array([s["number"] for s in S.train])
    mean = np.array([s["mean"] for s in S.train])
    mx = np.array([s["max"] for s in S.train])
    window = max(5, len(n) // 20) | 1
    checkpoints = [s["number"] for s in S.evals if s["kind"] == "eval" and s["number"] > 0]

    fig, axes = grid4("Training on random poses — tracking error per pose\n"
                      f"points: one training pose;  lines: rolling median over {window} poses")
    for i, ax in enumerate(axes):
        ax.semilogy(n, mx[:, i], ".", ms=3, color=JOINT_COLORS[i], alpha=0.30)
        ax.semilogy(n, mean[:, i], ".", ms=3, color="#334155", alpha=0.30)
        ax.semilogy(n, rolling_median(mx[:, i], window), color=JOINT_COLORS[i], lw=1.8, label="max |e|")
        ax.semilogy(n, rolling_median(mean[:, i], window), color="#334155", lw=1.8, label="mean |e|")
        for c in checkpoints:
            ax.axvline(c, color="k", ls=":", lw=0.8, alpha=0.5)
        k = max(1, len(n) // 10)
        stats_box(ax, [f"first {k:3d} poses: mean {mean[:k, i].mean():.4f}  max {mx[:k, i].max():.4f}",
                       f"last  {k:3d} poses: mean {mean[-k:, i].mean():.4f}  max {mx[-k:, i].max():.4f}"],
                  "#f1f5f9")
        ax.set_title(JOINT_LABELS[i], fontweight="bold", color=JOINT_COLORS[i])
        ax.set_xlabel("Training pose number"); ax.set_ylabel("|e|  (rad, log scale)")
        ax.grid(True, which="both", alpha=0.25); ax.legend(loc="upper right", fontsize=8)
    save(fig, out_dir, "train_error.png")

    all_mean, all_max = mean.mean(axis=1), mx.max(axis=1)
    fig, ax = plt.subplots(figsize=(11, 5))
    fig.suptitle("Training on random poses — error over all four joints", fontsize=13, fontweight="bold")
    ax.semilogy(n, all_max, ".", ms=3, color="#dc2626", alpha=0.30)
    ax.semilogy(n, all_mean, ".", ms=3, color="#334155", alpha=0.30)
    ax.semilogy(n, rolling_median(all_max, window), color="#dc2626", lw=2.0, label="max |e| (worst joint)")
    ax.semilogy(n, rolling_median(all_mean, window), color="#334155", lw=2.0, label="mean |e| (average of joints)")
    for c in checkpoints:
        ax.axvline(c, color="k", ls=":", lw=0.8, alpha=0.5)
        ax.text(c, 1.0, f" eval @{c}", transform=ax.get_xaxis_transform(), va="top", fontsize=7, alpha=0.75)
    ax.set_xlabel("Training pose number"); ax.set_ylabel("|e|  (rad, log scale)")
    ax.grid(True, which="both", alpha=0.25); ax.legend(loc="upper right", fontsize=9)
    save(fig, out_dir, "train_error_overall.png")


# ── evaluation loops ────────────────────────────────────────────────────────
def evaluation_figures(S, out_dir):
    if not S.evals:
        return
    d = S.d
    t_all, e_all = d["t_error"], d["error"][:, 0::2]

    fig, axes = grid4("Evaluation loop  Home → Pose 1 → Pose 2 → Home — tracking error  e = q − q_m")
    for i, ax in enumerate(axes):
        for k, s in enumerate(S.evals):
            sel = (t_all >= s["t0"]) & (t_all <= s["t1"])
            last = k == len(S.evals) - 1
            ax.plot(t_all[sel] - s["t0"], e_all[sel, i], lw=1.5 if last else 1.0,
                    color=EVAL_COLORS[k % len(EVAL_COLORS)], alpha=1.0 if last else 0.8,
                    label=S.eval_label(s), zorder=3 if last else 2)
        ax.axhline(0, color="k", ls="--", lw=0.7, alpha=0.4)
        ax.set_title(JOINT_LABELS[i], fontweight="bold", color=JOINT_COLORS[i])
        ax.set_xlabel("Time since start of the loop (s)"); ax.set_ylabel("e (rad)")
        ax.grid(True, alpha=0.25); ax.legend(loc="best", fontsize=8)
    save(fig, out_dir, "eval_error.png")

    labels = [S.eval_label(s) for s in S.evals]
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    fig.suptitle("Evaluation loop — error per joint", fontsize=13, fontweight="bold")
    width = 0.8 / len(S.evals)
    for ax, key, title in ((axes[0], "mean", "mean |e|"), (axes[1], "max", "max |e|")):
        for k, s in enumerate(S.evals):
            xs = np.arange(N_JOINTS) + (k - (len(S.evals) - 1) / 2) * width
            bars = ax.bar(xs, s[key], width * 0.92, color=EVAL_COLORS[k % len(EVAL_COLORS)], label=labels[k])
            for b, v in zip(bars, s[key]):
                ax.text(b.get_x() + b.get_width() / 2, v, f"{v:.4f}", ha="center", va="bottom",
                        fontsize=6.5, rotation=90)
        ax.set_xticks(np.arange(N_JOINTS)); ax.set_xticklabels(JOINT_LABELS)
        ax.set_ylabel(f"{title}  (rad)"); ax.set_title(title, fontweight="bold")
        ax.margins(y=0.22); ax.grid(True, axis="y", alpha=0.25); ax.legend(fontsize=8)
    save(fig, out_dir, "eval_summary.png")

    lines = ["# Evaluation loop results", "",
             "Error is plant position minus reference model, in rad.", ""]
    for key, title in (("mean", "Mean |e|"), ("max", "Max |e|"), ("final", "Final e (end of the loop)")):
        lines += [f"## {title}", "", "| Evaluation | Joint 1 | Joint 2 | Joint 3 | Joint 4 |", "|---|---|---|---|---|"]
        for s, label in zip(S.evals, labels):
            fmt = "{:+.4f}" if key == "final" else "{:.4f}"
            lines.append(f"| {label} | " + " | ".join(fmt.format(v) for v in s[key]) + " |")
        lines.append("")
    if S.train:
        lines += ["## Training", "",
                  f"{len(S.train)} random poses, "
                  f"{sum(s['t1'] - s['t0'] for s in S.train) / 60:.1f} min of simulation time.", ""]
    path = os.path.join(out_dir, "summary.md")
    with open(path, "w") as f:
        f.write("\n".join(lines))
    print("  wrote", path)
    print("\n".join(lines[4:]))


# ── camera GIFs ─────────────────────────────────────────────────────────────
def make_gifs(S, in_dir, out_dir, speed, fps, width):
    csv_path = os.path.join(in_dir, "frames.csv")
    if not os.path.exists(csv_path) or not S.evals:
        return
    from PIL import Image, ImageDraw, ImageFont

    with open(csv_path) as f:
        rows = [(int(r["index"]), float(r["sim_time"]), r["kind"], int(r["number"])) for r in csv.DictReader(f)]
    font_path = os.path.join(os.path.dirname(matplotlib.__file__), "mpl-data", "fonts", "ttf", "DejaVuSans-Bold.ttf")
    step = speed / fps
    chosen = [S.evals[0]] + ([S.evals[-1]] if len(S.evals) > 1 else [])
    for s in chosen:
        sel = [(i, t) for i, t, kind, number in rows if kind == s["kind"] and number == s["number"]]
        if not sel:
            continue
        picked, next_t = [], sel[0][1]
        for i, t in sel:
            if t >= next_t:
                picked.append(i); next_t += step
        frames = []
        for i in picked:
            im = Image.open(os.path.join(in_dir, "frames", f"{i:06d}.jpg")).convert("RGB")
            im = im.resize((width, round(im.height * width / im.width)), Image.LANCZOS)
            draw = ImageDraw.Draw(im)
            big = ImageFont.truetype(font_path, max(14, width // 12))
            small = ImageFont.truetype(font_path, max(10, width // 26))
            label = f"x{speed:g}"
            draw.text((width - draw.textlength(label, font=big) - 12, 8), label, fill=(0, 0, 0), font=big)
            draw.text((12, 10), S.eval_label(s), fill=(0, 0, 0), font=small)
            frames.append(im.quantize(colors=128, method=Image.MEDIANCUT, dither=Image.NONE))
        path = os.path.join(out_dir, f"{s['kind']}_{s['number']}.gif")
        frames[0].save(path, save_all=True, append_images=frames[1:], duration=round(1000 / fps), loop=0, optimize=True)
        print(f"  wrote {path}  ({len(frames)} frames, {os.path.getsize(path)/1e6:.1f} MB)")


def main(args=None):
    rclpy.init(args=args)
    node = Node("mrac_plot")
    in_dir = os.path.expanduser(node.declare_parameter("in_dir", "").value)
    out_dir = os.path.expanduser(node.declare_parameter("out_dir", "").value) or in_dir
    gif_speed = float(node.declare_parameter("gif_speed", 8.0).value)
    gif_fps = float(node.declare_parameter("gif_fps", 12.5).value)
    gif_width = int(node.declare_parameter("gif_width", 480).value)
    node.destroy_node()
    rclpy.shutdown()

    if not in_dir or not os.path.exists(os.path.join(in_dir, "data.npz")):
        print("Give the recording folder:  --ros-args -p in_dir:=<folder containing data.npz>")
        return
    os.makedirs(out_dir, exist_ok=True)
    d = np.load(os.path.join(in_dir, "data.npz"))
    if len(d["t_error"]) == 0:
        print("The recording contains no /mrac/error samples.")
        return
    d = repair_time(d)

    S = Session(d)
    print(f"Session: {(d['t_error'][-1] - S.t0) / 60:.1f} min, {len(S.train)} training poses, "
          f"{len(S.evals)} evaluation loops")
    session_figures(S, out_dir)
    training_figures(S, out_dir)
    evaluation_figures(S, out_dir)
    make_gifs(S, in_dir, out_dir, gif_speed, gif_fps, gif_width)


if __name__ == "__main__":
    main()

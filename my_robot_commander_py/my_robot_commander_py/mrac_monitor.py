#!/usr/bin/env python3
"""
MRAC Monitor — Comprehensive Parameter & Error Plotter
=======================================================
Subscribes to all MRAC diagnostic topics and produces separate figures:

  Figure 1 — MRAC error  e_mrac = q − q_m  (plant vs reference model, driven by r_vec)
  Figure 2 — Adaptive gains Kx      (K_hat_x per joint)
  Figure 3 — Adaptive gains Kr      (K_hat_r per joint)
  Figure 4 — Robust weights W_hat   (‖Ŵ_i‖ norm per joint)
  Figure 5 — Lyapunov function V    (stability check, all joints)
  Figure 6 — Per-joint overlay: reference model q_m vs actual joint q  [4 subplots]

Topics consumed:
  /mrac/error        Float64MultiArray  [e0,ė0, e1,ė1, e2,ė2, e3,ė3]
  /mrac/K_hat        Float64MultiArray  [Kx0,Kr0, Kx1,Kr1, ...]
  /mrac/W_hat        Float64MultiArray  [Ŵ_0(22), Ŵ_1(22), ...]
  /mrac/lyapunov_V   Float64MultiArray  [V0, V1, V2, V3]
  /mrac/ref_model    Float64MultiArray  [qm0,q̇m0, qm1,q̇m1, ...]
  /joint_states      sensor_msgs/JointState  (actual joint positions)

Error semantics:
  e_mrac  q − q_m    Controller-internal: what the MRAC adaptive law minimises.
                      Reflects how well the plant tracks the reference model.
                      Now reference model is driven by r_vec (trajectory), so
                      e_mrac = q − q_m already captures trajectory tracking error.

Usage:
  Terminal 1: ros2 launch my_robot_bringup my_robot_gz.launch.py
  Terminal 2: ros2 run my_robot_commander_py mrac_monitor
  Terminal 3: ros2 run my_robot_commander_py test_moveit

Commands while running:
  'p'  — show all plots and print summary
  's'  — save all figures to /tmp/
  'r'  — reset / clear all buffers
  'q'  — quit
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import threading
import time
import os

# ── Constants must match mrac_controller.cpp ────────────────────────────────
N_JOINTS = 4
N_BASIS   = 22
JOINT_LABELS = [f"joint{i+1}" for i in range(N_JOINTS)]

# Colour palette — one colour per joint
JOINT_COLORS = ["#3b82f6", "#f97316", "#22c55e", "#a855f7"]
ALPHA_FILL   = 0.12
MAX_SAMPLES  = 700_000   # ~15 s at 1 kHz

SAVE_DIR = "/tmp"


class MRACMonitor(Node):
    def __init__(self):
        super().__init__("mrac_monitor")

        self.t = deque(maxlen=MAX_SAMPLES)   # wall-clock seconds

        # /mrac/error  — e_mrac: [e0,ė0, e1,ė1, ...]
        #   e_mrac = q − q_m  (plant vs reference model)
        self.e  = [deque(maxlen=MAX_SAMPLES) for _ in range(N_JOINTS)]
        self.de = [deque(maxlen=MAX_SAMPLES) for _ in range(N_JOINTS)]

        # /mrac/K_hat  — [Kx0,Kr0, Kx1,Kr1, ...]
        self.Kx = [deque(maxlen=MAX_SAMPLES) for _ in range(N_JOINTS)]
        self.Kr = [deque(maxlen=MAX_SAMPLES) for _ in range(N_JOINTS)]

        # /mrac/W_hat  — ‖Ŵ_i‖ norm (22 values per joint)
        self.W_norm = [deque(maxlen=MAX_SAMPLES) for _ in range(N_JOINTS)]

        # /mrac/lyapunov_V
        self.V = [deque(maxlen=MAX_SAMPLES) for _ in range(N_JOINTS)]

        # /mrac/ref_model — reference model positions [qm0,q̇m0, qm1,q̇m1, ...]
        self.q_ref = [deque(maxlen=MAX_SAMPLES) for _ in range(N_JOINTS)]

        # /joint_states — actual joint positions
        self.q_actual = [deque(maxlen=MAX_SAMPLES) for _ in range(N_JOINTS)]
        # timing buffer shared with joint_states (separate from /mrac/error timing)
        self.t_js = deque(maxlen=MAX_SAMPLES)

        self._t0   = time.time()
        self._lock = threading.Lock()
        self._n    = 0   # samples recorded

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(
            Float64MultiArray, "/mrac/error",
            self._cb_error, 10)
        self.create_subscription(
            Float64MultiArray, "/mrac/K_hat",
            self._cb_khat, 10)
        self.create_subscription(
            Float64MultiArray, "/mrac/W_hat",
            self._cb_what, 10)
        self.create_subscription(
            Float64MultiArray, "/mrac/lyapunov_V",
            self._cb_lyap, 10)

        self.create_subscription(
            JointState, "/joint_states",
            self._cb_joint_states, 10)

        self.get_logger().info("🎯 MRAC Monitor started — waiting for data…")

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _cb_error(self, msg: Float64MultiArray):
        """e_mrac = q − q_m — plant vs reference model (MRAC internal error)."""
        now = time.time() - self._t0
        with self._lock:
            self.t.append(now)
            self._n += 1
            for i in range(N_JOINTS):
                self.e[i].append( msg.data[2*i]   if len(msg.data) > 2*i   else 0.0)
                self.de[i].append(msg.data[2*i+1] if len(msg.data) > 2*i+1 else 0.0)

    def _cb_khat(self, msg: Float64MultiArray):
        with self._lock:
            for i in range(N_JOINTS):
                self.Kx[i].append(msg.data[2*i]   if len(msg.data) > 2*i   else 0.0)
                self.Kr[i].append(msg.data[2*i+1] if len(msg.data) > 2*i+1 else 0.0)

    def _cb_what(self, msg: Float64MultiArray):
        with self._lock:
            for i in range(N_JOINTS):
                start = i * N_BASIS
                if len(msg.data) >= start + N_BASIS:
                    w = np.array(msg.data[start:start + N_BASIS])
                    self.W_norm[i].append(float(np.linalg.norm(w)))
                else:
                    self.W_norm[i].append(0.0)

    def _cb_lyap(self, msg: Float64MultiArray):
        with self._lock:
            for i in range(N_JOINTS):
                self.V[i].append(msg.data[i] if len(msg.data) > i else 0.0)

    def _cb_ref_model(self, msg: Float64MultiArray):
        """Reference model positions: [qm0,q̇m0, qm1,q̇m1, ...]."""
        with self._lock:
            for i in range(N_JOINTS):
                idx = 2 * i
                self.q_ref[i].append(msg.data[idx] if len(msg.data) > idx else 0.0)

    def _cb_joint_states(self, msg: JointState):
        """Actual joint positions from /joint_states."""
        now = time.time() - self._t0
        # Build a name→index map on first call
        if not hasattr(self, '_js_map'):
            self._js_map = {name: k for k, name in enumerate(msg.name)}
        with self._lock:
            self.t_js.append(now)
            for i in range(N_JOINTS):
                label = JOINT_LABELS[i]  # e.g. "joint1"
                k = self._js_map.get(label, -1)
                if k >= 0 and k < len(msg.position):
                    self.q_actual[i].append(msg.position[k])
                else:
                    self.q_actual[i].append(0.0)

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _snap(self):
        """Return a consistent snapshot of all buffers under lock."""
        with self._lock:
            t       = np.array(self.t)
            e       = [np.array(q) for q in self.e]
            de      = [np.array(q) for q in self.de]
            Kx      = [np.array(q) for q in self.Kx]
            Kr      = [np.array(q) for q in self.Kr]
            W       = [np.array(q) for q in self.W_norm]
            V       = [np.array(q) for q in self.V]
            t_js    = np.array(self.t_js)
            q_ref   = [np.array(q) for q in self.q_ref]
            q_actual = [np.array(q) for q in self.q_actual]
        return t, e, de, Kx, Kr, W, V, t_js, q_ref, q_actual

    # ── Plot helpers ──────────────────────────────────────────────────────────
    @staticmethod
    def _trim(t, arr):
        """Trim arr to the same length as t."""
        n = min(len(t), len(arr))
        return t[:n], arr[:n]

    def _stats(self, arr):
        if len(arr) == 0:
            return 0., 0., 0.
        a = np.abs(arr)
        return float(np.max(a)), float(np.mean(a)), float(a[-1])

    # ─────────────────────────────────────────────────────────────────────────
    #  FIGURE 1 — MRAC error  e_mrac = q − q_m  (plant vs reference model)
    # ─────────────────────────────────────────────────────────────────────────
    def _fig_errors(self, t, e, de):
        fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=False)
        fig.suptitle(
            "Fig 1 — MRAC Controller Error   e_mrac = q − q_m  (plant vs reference model)\n"
            "This is the error the MRAC adaptive law drives to zero  (§17.3)",
            fontsize=12, fontweight="bold")

        for i, ax in enumerate(axes.flat):
            c = JOINT_COLORS[i]
            tt, ei = self._trim(t, e[i])
            if len(ei) == 0:
                ax.text(0.5, 0.5, "No data", ha="center", va="center",
                        transform=ax.transAxes)
            else:
                ax.plot(tt, ei, color=c, lw=1.4, label="e_mrac (rad)")
                ax.fill_between(tt, ei, alpha=ALPHA_FILL, color=c)
                ax.axhline(0, color="k", ls="--", lw=0.7, alpha=0.4)
                mx, mn, fn = self._stats(ei)
                ax.text(0.02, 0.97,
                    f"max |e| = {mx:.4f} rad\nmean|e| = {mn:.4f} rad\nfinal   = {fn:.4f} rad",
                    transform=ax.transAxes, va="top", fontsize=8,
                    family="monospace",
                    bbox=dict(boxstyle="round", fc="wheat", alpha=0.8))
            ax.set_title(f"{JOINT_LABELS[i]}", fontweight="bold", color=c)
            ax.set_xlabel("Time (s)"); ax.set_ylabel("e_mrac (rad)")
            ax.grid(True, alpha=0.25); ax.legend(fontsize=8)

        fig.tight_layout()
        return fig

    # ─────────────────────────────────────────────────────────────────────────
    #  FIGURE 2 — Adaptive gain K_hat_x
    # ─────────────────────────────────────────────────────────────────────────
    def _fig_Kx(self, t, Kx):
        fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=False)
        fig.suptitle("Fig 2 — Adaptive Feedback Gain  K̂_x  (§17.3)\n"
                     "K̂̇_x = −Γ_Kx · e · sign(b) · q − σ · K̂_x",
                     fontsize=13, fontweight="bold")

        for i, ax in enumerate(axes.flat):
            c = JOINT_COLORS[i]
            tt, kxi = self._trim(t, Kx[i])
            if len(kxi):
                ax.plot(tt, kxi, color=c, lw=1.4)
                ax.axhline(0, color="k", ls="--", lw=0.7, alpha=0.4)
                ax.text(0.02, 0.97,
                    f"init   = {kxi[0]:.3f}\nfinal  = {kxi[-1]:.3f}\nΔ      = {kxi[-1]-kxi[0]:.3f}",
                    transform=ax.transAxes, va="top", fontsize=8,
                    family="monospace",
                    bbox=dict(boxstyle="round", fc="#dbeafe", alpha=0.85))
            else:
                ax.text(0.5, 0.5, "No data", ha="center", va="center",
                        transform=ax.transAxes)
            ax.set_title(f"{JOINT_LABELS[i]}  K̂_x", fontweight="bold", color=c)
            ax.set_xlabel("Time (s)"); ax.set_ylabel("K̂_x  [Nm/rad]")
            ax.grid(True, alpha=0.25)

        fig.tight_layout()
        return fig

    # ─────────────────────────────────────────────────────────────────────────
    #  FIGURE 3 — Adaptive gain K_hat_r
    # ─────────────────────────────────────────────────────────────────────────
    def _fig_Kr(self, t, Kr):
        fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=False)
        fig.suptitle("Fig 3 — Adaptive Feedforward Gain  K̂_r  (§17.3)\n"
                     "K̂̇_r = −Γ_Kr · e · sign(b) · r − σ · K̂_r",
                     fontsize=13, fontweight="bold")

        for i, ax in enumerate(axes.flat):
            c = JOINT_COLORS[i]
            tt, kri = self._trim(t, Kr[i])
            if len(kri):
                ax.plot(tt, kri, color=c, lw=1.4)
                ax.axhline(0, color="k", ls="--", lw=0.7, alpha=0.4)
                ax.text(0.02, 0.97,
                    f"init   = {kri[0]:.3f}\nfinal  = {kri[-1]:.3f}\nΔ      = {kri[-1]-kri[0]:.3f}",
                    transform=ax.transAxes, va="top", fontsize=8,
                    family="monospace",
                    bbox=dict(boxstyle="round", fc="#dcfce7", alpha=0.85))
            else:
                ax.text(0.5, 0.5, "No data", ha="center", va="center",
                        transform=ax.transAxes)
            ax.set_title(f"{JOINT_LABELS[i]}  K̂_r", fontweight="bold", color=c)
            ax.set_xlabel("Time (s)"); ax.set_ylabel("K̂_r  [Nm/rad]")
            ax.grid(True, alpha=0.25)

        fig.tight_layout()
        return fig

    # ─────────────────────────────────────────────────────────────────────────
    #  FIGURE 4 — ‖Ŵ_i‖ robust weight norms
    # ─────────────────────────────────────────────────────────────────────────
    def _fig_W(self, t, W):
        fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=False)
        fig.suptitle("Fig 4 — Robust Weight Norm  ‖Ŵ_i‖  (§19.2)\n"
                     "Ŵ̇ = Γ_W · (e^T P B) · α(q, q̇) − σ_W · Ŵ",
                     fontsize=13, fontweight="bold")

        for i, ax in enumerate(axes.flat):
            c = JOINT_COLORS[i]
            tt, wi = self._trim(t, W[i])
            if len(wi):
                ax.plot(tt, wi, color=c, lw=1.4)
                ax.fill_between(tt, wi, alpha=ALPHA_FILL, color=c)
                ax.text(0.02, 0.97,
                    f"final ‖Ŵ‖ = {wi[-1]:.4f}\nmax   ‖Ŵ‖ = {np.max(wi):.4f}",
                    transform=ax.transAxes, va="top", fontsize=8,
                    family="monospace",
                    bbox=dict(boxstyle="round", fc="#fae8ff", alpha=0.85))
            else:
                ax.text(0.5, 0.5, "No data (robust OFF?)", ha="center",
                        va="center", transform=ax.transAxes)
            ax.set_title(f"{JOINT_LABELS[i]}  ‖Ŵ‖", fontweight="bold", color=c)
            ax.set_xlabel("Time (s)"); ax.set_ylabel("‖Ŵ_i‖")
            ax.grid(True, alpha=0.25)

        fig.tight_layout()
        return fig

    # ─────────────────────────────────────────────────────────────────────────
    #  FIGURE 5 — Per-joint overlay: Reference Model q_m vs Actual q
    # ─────────────────────────────────────────────────────────────────────────
    def _fig_joint_vs_ref(self, t_js, q_ref, q_actual):
        """4 subplots — each shows reference model output q_m and actual joint
        position q overlaid, so the user can directly assess tracking quality."""
        fig, axes = plt.subplots(2, 2, figsize=(14, 9), sharex=False)
        fig.suptitle(
            "Fig 5 — Per-Joint Tracking: Reference Model q_m  vs  Actual Joint q\n"
            "Ideal: q (solid) closely follows q_m (dashed)",
            fontsize=13, fontweight="bold")

        for i, ax in enumerate(axes.flat):
            c = JOINT_COLORS[i]

            # Trim to matching length for q_ref vs t
            n_ref = min(len(t_js), len(q_ref[i]))
            n_act = min(len(t_js), len(q_actual[i]))
            tt_ref = t_js[:n_ref]
            qm_i   = q_ref[i][:n_ref]
            tt_act = t_js[:n_act]
            qa_i   = q_actual[i][:n_act]

            if len(qm_i) == 0 and len(qa_i) == 0:
                ax.text(0.5, 0.5,
                        "No data — check /mrac/ref_model & /joint_states",
                        ha="center", va="center", transform=ax.transAxes,
                        fontsize=9, color="gray")
            else:
                if len(qm_i):
                    ax.plot(tt_ref, qm_i, color=c, lw=1.8,
                            ls="--", label="q_m (ref model)", alpha=0.85)
                if len(qa_i):
                    ax.plot(tt_act, qa_i, color=c, lw=1.4,
                            ls="-",  label="q (actual)", alpha=1.0)
                    ax.fill_between(tt_act, qa_i,
                                    np.interp(tt_act, tt_ref, qm_i)
                                    if len(qm_i) else 0,
                                    alpha=ALPHA_FILL * 1.5, color=c)

                # Stats: RMS tracking error over the common range
                n_common = min(len(qm_i), len(qa_i))
                if n_common > 5:
                    err = qa_i[:n_common] - qm_i[:n_common]
                    rms = float(np.sqrt(np.mean(err**2)))
                    peak = float(np.max(np.abs(err)))
                    ax.text(0.02, 0.97,
                        f"RMS  |q−q_m| = {rms:.4f} rad\n"
                        f"Peak |q−q_m| = {peak:.4f} rad",
                        transform=ax.transAxes, va="top", fontsize=8,
                        family="monospace",
                        bbox=dict(boxstyle="round", fc="#fef9c3", alpha=0.85))

            ax.axhline(0, color="k", ls=":", lw=0.6, alpha=0.3)
            ax.set_title(f"{JOINT_LABELS[i]}  —  q_m vs q",
                         fontweight="bold", color=c)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Position (rad)")
            ax.grid(True, alpha=0.25)
            ax.legend(fontsize=8, loc="upper right")

        fig.tight_layout()
        return fig

    # ─────────────────────────────────────────────────────────────────────────
    #  FIGURE 6 — Lyapunov function V (stability check)
    # ─────────────────────────────────────────────────────────────────────────
    def _fig_lyap(self, t, V):
        fig, ax = plt.subplots(figsize=(11, 5))
        fig.suptitle("Fig 6 — Lyapunov Function V(e, Ŵ, K̃)\n"
                     "V = e^T P e + ‖Ŵ‖²/Γ_W   (sigma > 0 → convergence to a residual set)",
                     fontsize=13, fontweight="bold")

        for i in range(N_JOINTS):
            tt, vi = self._trim(t, V[i])
            if len(vi):
                ax.plot(tt, vi, color=JOINT_COLORS[i], lw=1.6,
                        label=JOINT_LABELS[i])

        ax.set_xlabel("Time (s)"); ax.set_ylabel("V")
        ax.legend(); ax.grid(True, alpha=0.25)

        # Check monotonicity for joint 0 as a proxy
        tt, v0 = self._trim(t, V[0])
        if len(v0) > 1:
            dv = np.diff(v0)
            pct = np.sum(dv > 1e-6) / len(dv) * 100
            color = "#ef4444" if pct > 10 else "#22c55e"
            ax.text(0.75, 0.96,
                f"[joint1] V increasing: {pct:.1f}%\n"
                f"(target < 10% for stability)",
                transform=ax.transAxes, va="top", fontsize=9,
                family="monospace",
                bbox=dict(boxstyle="round", fc=color, alpha=0.25))

        fig.tight_layout()
        return fig

    # ─────────────────────────────────────────────────────────────────────────
    #  PUBLIC: show all plots
    # ─────────────────────────────────────────────────────────────────────────
    def plot_all(self, save=False):
        if self._n == 0:
            self.get_logger().warn("❌ No data collected yet.")
            return

        t, e, de, Kx, Kr, W, V, t_js, q_ref, q_actual = self._snap()

        figs = []
        figs.append(("e_mrac",        self._fig_errors(t, e, de)))
        figs.append(("Kx",            self._fig_Kx(t, Kx)))
        figs.append(("Kr",            self._fig_Kr(t, Kr)))
        figs.append(("W_hat",         self._fig_W(t, W)))
        figs.append(("lyapunov",      self._fig_lyap(t, V)))

        if save:
            for name, fig in figs:
                path = os.path.join(SAVE_DIR, f"mrac_{name}.png")
                fig.savefig(path, dpi=150, bbox_inches="tight")
                self.get_logger().info(f"📄 Saved → {path}")

        plt.show()

    # ─────────────────────────────────────────────────────────────────────────
    def print_summary(self):
        t, e, de, Kx, Kr, W, V, t_js, q_ref, q_actual = self._snap()
        lines = ["\n" + "="*70, "📈 MRAC SUMMARY", "="*70]
        for i in range(N_JOINTS):
            if len(e[i]) < 10:
                continue
            ei  = np.abs(np.array(e[i]))
            e0  = float(np.mean(ei[:50])) if len(ei) >= 50 else float(np.mean(ei))
            ef  = float(np.mean(ei[-50:])) if len(ei) >= 50 else float(np.mean(ei))
            conv = (e0 - ef) / max(e0, 1e-6) * 100
            Kxf = Kx[i][-1] if len(Kx[i]) else 0.
            Krf = Kr[i][-1] if len(Kr[i]) else 0.
            Wf  = W[i][-1]  if len(W[i])  else 0.
            lines.append(f"\n  {JOINT_LABELS[i]}:")
            lines.append(f"    e_mrac  initial = {e0:.5f} rad   (q − q_m, driven by r_vec)")
            lines.append(f"    e_mrac  final   = {ef:.5f} rad")
            lines.append(f"    convergence     = {conv:.1f}%")
            lines.append(f"    K̂_x final = {Kxf:.4f}   K̂_r final = {Krf:.4f}")
            lines.append(f"    ‖Ŵ‖ final  = {Wf:.4f}")
            if ef > 0.10:
                lines.append("    ⚠️  Large e_mrac — increase gamma_Kx or gamma_W")
            elif ef > 0.05:
                lines.append("    ⚡ Moderate e_mrac — consider increasing wn")
            elif conv < 30:
                lines.append("    🐢 Slow convergence — increase wn")
            else:
                lines.append("    ✅ Good tracking")
        lines.append("="*70)
        msg = "\n".join(lines)
        self.get_logger().info(msg)

    def reset_buffers(self):
        with self._lock:
            self.t.clear()
            self.t_js.clear()
            for q in [*self.e, *self.de,
                      *self.Kx, *self.Kr, *self.W_norm, *self.V,
                      *self.q_ref, *self.q_actual]:
                q.clear()
            self._n = 0
            self._t0 = time.time()
        self.get_logger().info("🔄 Buffers cleared.")


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    monitor = MRACMonitor()

    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(monitor), daemon=True)
    spin_thread.start()

    print("\n" + "="*70)
    print("🎯 MRAC MONITOR  —  Separate graphs per parameter")
    print("="*70)
    print("Commands:")
    print("  p   — show all 6 figures (incl. per-joint q_m vs q overlay, Lyapunov)")
    print("  s   — save figures to /tmp/mrac_*.png  and show")
    print("  r   — reset / clear all buffers")
    print("  q   — quit\n")
    print("Waiting for MRAC data… (run test_moveit in another terminal)\n")

    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd == "p":
                monitor.plot_all(save=False)
                monitor.print_summary()
            elif cmd == "s":
                monitor.plot_all(save=True)
                monitor.print_summary()
            elif cmd == "r":
                monitor.reset_buffers()
            elif cmd == "q":
                break
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
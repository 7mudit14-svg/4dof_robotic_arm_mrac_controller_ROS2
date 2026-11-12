#!/usr/bin/env python3
"""
mrac_record.py — record an MRAC session for plots  (ROS2 Jazzy)
════════════════════════════════════════════════════════════════════════
Records the /mrac/* diagnostic topics, stamped with simulation time, and
splits the session into segments using /mrac/segment, which test_moveit
and mrac_trainer publish ("eval_loop:2", "train:17", "eval:50", "idle:0").

For every segment it keeps the exact max and mean of |e| per joint
(computed from every sample), so a long training session can be stored
with decimated traces without losing the peaks.

Start it BEFORE test_moveit / mrac_trainer. It stops by itself when the
session goes back to idle (or on Ctrl+C) and writes:

  <out_dir>/data.npz        traces and per-segment statistics
  <out_dir>/frames/*.jpg    camera frames, only if the simulation was
  <out_dir>/frames.csv      launched with record_camera:=true

Then make the figures with:
  ros2 run my_robot_commander_py mrac_plot --ros-args -p in_dir:=<out_dir>

Parameters (defaults in brackets):
  out_dir         [~/mrac_recordings/<date-time>]
  decimate        [1]     keep every N-th sample of each trace (use 5–10 for long sessions)
  stop_when_idle  [true]  stop when /mrac/segment returns to idle
  autosave_sec    [300]   write data.npz this often while recording
════════════════════════════════════════════════════════════════════════
"""

import os
import time
from array import array

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Int32, String

ARRAY_TOPICS = ["error", "traj_error", "K_hat", "W_hat", "tau", "u_ad", "ref_model", "lyapunov_V"]
N_JOINTS = 4


class Trace:
    """Memory-efficient growing table of (time, values)."""

    def __init__(self, decimate):
        self.t = array("d")
        self.v = array("d")
        self.width = 0
        self.decimate = max(1, decimate)
        self.count = 0

    def add(self, t, values):
        self.count += 1
        if (self.count - 1) % self.decimate:
            return
        if self.width == 0:
            self.width = len(values)
        if len(values) != self.width:
            return
        self.t.append(t)
        self.v.extend(values)

    def arrays(self):
        t = np.frombuffer(self.t, dtype=np.float64).copy()
        v = np.frombuffer(self.v, dtype=np.float64).copy()
        return t, v.reshape(len(t), self.width) if self.width else v.reshape(0, 0)


class Segment:
    def __init__(self, kind, number, t0):
        self.kind, self.number, self.t0, self.t1 = kind, number, t0, t0
        self.n = 0
        self.sum_e = np.zeros(N_JOINTS)
        self.max_e = np.zeros(N_JOINTS)
        self.sum_d = np.zeros(N_JOINTS)
        self.max_d = np.zeros(N_JOINTS)
        self.nd = 0
        self.last_e = np.zeros(N_JOINTS)
        self.max_tau = np.zeros(N_JOINTS)


class MRACRecorder(Node):
    def __init__(self):
        super().__init__(
            "mrac_record",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        default_dir = os.path.join(
            os.path.expanduser("~"), "mrac_recordings", time.strftime("%Y-%m-%d_%H-%M-%S")
        )
        self.out_dir = os.path.expanduser(self.declare_parameter("out_dir", default_dir).value)
        decimate = int(self.declare_parameter("decimate", 1).value)
        self.stop_when_idle = self.declare_parameter("stop_when_idle", True).value
        self.autosave_sec = float(self.declare_parameter("autosave_sec", 300.0).value)
        self.frames_dir = os.path.join(self.out_dir, "frames")
        os.makedirs(self.out_dir, exist_ok=True)

        self.traces = {name: Trace(decimate) for name in ARRAY_TOPICS}
        self.mode = Trace(decimate)
        self.segments = []
        self.current = None
        self.frame_rows = []
        self.seen_activity = False
        self.done = False
        self.last_save = time.time()

        for name in ARRAY_TOPICS:
            self.create_subscription(
                Float64MultiArray, "/mrac/" + name, lambda m, n=name: self._on_array(n, m), 500
            )
        self.create_subscription(Int32, "/mrac/mode", self._on_mode, 100)
        self.create_subscription(
            String, "/mrac/segment", self._on_segment,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(Image, "/rec_cam", self._on_image, 10)
        self.create_timer(5.0, self._maybe_autosave)

        self.get_logger().info(f"Recording to {self.out_dir} — start test_moveit or mrac_trainer now")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ── callbacks ────────────────────────────────────────────────────────
    def _on_array(self, name, msg):
        t = self.now()
        self.traces[name].add(t, msg.data)
        seg = self.current
        if seg is None:
            return
        if name == "error" and len(msg.data) >= 2 * N_JOINTS:
            e = np.array(msg.data[0:2 * N_JOINTS:2])
            a = np.abs(e)
            seg.n += 1
            seg.sum_e += a
            seg.max_e = np.maximum(seg.max_e, a)
            seg.last_e = e
            seg.t1 = t
        elif name == "traj_error" and len(msg.data) >= N_JOINTS:
            a = np.abs(np.array(msg.data[:N_JOINTS]))
            seg.nd += 1
            seg.sum_d += a
            seg.max_d = np.maximum(seg.max_d, a)
        elif name == "tau" and len(msg.data) >= N_JOINTS:
            seg.max_tau = np.maximum(seg.max_tau, np.abs(np.array(msg.data[:N_JOINTS])))

    def _on_mode(self, msg):
        self.mode.add(self.now(), [float(msg.data)])

    def _on_segment(self, msg):
        kind, _, number = msg.data.partition(":")
        try:
            number = int(number)
        except ValueError:
            number = 0
        t = self.now()
        if self.current is not None:
            self.current.t1 = t
            self.segments.append(self.current)
            self.current = None
        if kind == "idle":
            if self.seen_activity and self.stop_when_idle:
                self.get_logger().info("Session finished — stopping")
                self.done = True
            return
        if not self.seen_activity:
            self.get_logger().info("Session started")
        self.seen_activity = True
        self.current = Segment(kind, number, t)
        if kind != "train" or number % 25 == 0:
            self.get_logger().info(f"segment {kind}:{number}")

    def _on_image(self, msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            return
        from PIL import Image as PilImage

        os.makedirs(self.frames_dir, exist_ok=True)
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
        arr = arr[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            arr = arr[:, :, ::-1]
        idx = len(self.frame_rows)
        PilImage.fromarray(arr).save(os.path.join(self.frames_dir, f"{idx:06d}.jpg"), quality=88)
        seg = self.current
        self.frame_rows.append((idx, self.now(), seg.kind if seg else "idle", seg.number if seg else 0))

    def _maybe_autosave(self):
        if time.time() - self.last_save >= self.autosave_sec:
            self.save(quiet=True)

    # ── output ───────────────────────────────────────────────────────────
    def save(self, quiet=False):
        self.last_save = time.time()
        out = {}
        for name, trace in self.traces.items():
            out["t_" + name], out[name] = trace.arrays()
        out["t_mode"], mode = self.mode.arrays()
        out["mode"] = mode.reshape(-1)

        segs = self.segments + ([self.current] if self.current is not None else [])
        out["seg_kind"] = np.array([s.kind for s in segs], dtype="U16")
        out["seg_number"] = np.array([s.number for s in segs], dtype=np.int64)
        out["seg_t0"] = np.array([s.t0 for s in segs])
        out["seg_t1"] = np.array([s.t1 for s in segs])
        out["seg_samples"] = np.array([s.n for s in segs], dtype=np.int64)
        z = np.zeros((0, N_JOINTS))
        out["seg_mean_e"] = np.array([s.sum_e / max(s.n, 1) for s in segs]) if segs else z
        out["seg_max_e"] = np.array([s.max_e for s in segs]) if segs else z
        out["seg_mean_traj"] = np.array([s.sum_d / max(s.nd, 1) for s in segs]) if segs else z
        out["seg_max_traj"] = np.array([s.max_d for s in segs]) if segs else z
        out["seg_final_e"] = np.array([s.last_e for s in segs]) if segs else z
        out["seg_max_tau"] = np.array([s.max_tau for s in segs]) if segs else z

        tmp = os.path.join(self.out_dir, "data.tmp.npz")
        np.savez_compressed(tmp, **out)
        os.replace(tmp, os.path.join(self.out_dir, "data.npz"))
        if self.frame_rows:
            with open(os.path.join(self.out_dir, "frames.csv"), "w") as f:
                f.write("index,sim_time,kind,number\n")
                for idx, t, kind, number in self.frame_rows:
                    f.write(f"{idx},{t:.4f},{kind},{number}\n")
        if not quiet:
            print(
                f"\nSaved {len(out['t_error'])} samples in {len(segs)} segments"
                f"{' and ' + str(len(self.frame_rows)) + ' camera frames' if self.frame_rows else ''} to\n"
                f"  {self.out_dir}\n"
                f"Next:  ros2 run my_robot_commander_py mrac_plot --ros-args -p in_dir:={self.out_dir}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = MRACRecorder()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
grasp_test_interactive.py
--------------------------
An interactive keyboard-driven tester for the pick-and-place system.
Lets you fire individual arm moves one step at a time, with confirmation
before each motion — useful for safely verifying each offset before running
the full autonomous sequence.

You DON'T need the vision nodes or the grasp node running.
This script talks directly to MoveIt2 and the gripper.

Controls
--------
  o  — open gripper
  c  — close gripper
  h  — move to HOME position
  1  — move to PRE-GRASP  (above cube test position)
  2  — move to GRASP      (at cube test position)
  3  — move to LIFT       (lift after grasp)
  4  — move to PRE-PLACE  (above box test position)
  5  — move to PLACE      (inside box)
  p  — run full PICK sequence (steps 1-2-close-3)
  d  — run full DROP sequence (steps 4-5-open)
  f  — run full cycle     (pick + drop)
  q  — quit

All moves prompt "Ready? [y/N]" before executing.
Set your test positions at the top of the file.
"""

import sys
import termios
import tty
import threading

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.duration import Duration

from pymoveit2 import MoveIt2


# ===========================================================================
# TEST POSITIONS — edit these before running
# All values are in the robot base frame (g_base), in metres.
# ===========================================================================

# Cube position (wherever you place the test cube on the table)
CUBE_X, CUBE_Y, CUBE_Z = 0.40, 0.00, 0.05   # CHANGE

# Box position (wherever the drop box is)
BOX_X,  BOX_Y,  BOX_Z  = 0.35, 0.10, 0.00   # CHANGE

# Offsets — match these to the values in cube_grasp_node.py
PRE_GRASP_ABOVE  = 0.10   # metres above cube for pre-grasp hover
GRASP_ABOVE      = 0.07   # metres above cube for actual grasp
LIFT_ABOVE       = 0.20   # metres above cube after closing gripper
PRE_PLACE_ABOVE  = 0.15   # metres above box for safe hover
PLACE_ABOVE      = 0.08   # metres above box to open gripper

# EEF orientation (tool pointing down — verify in RViz)
# CHANGE if your EEF convention is different
DOWN_QUAT = [1.0, 0.0, 0.0, 0.0]   # [x, y, z, w]

# Home joint angles (radians) — CHANGE to your actual safe home
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# MoveIt2 settings — CHANGE to match your robot
JOINT_NAMES       = [                          # CHANGE
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]
BASE_LINK         = "g_base"          # CHANGE
END_EFFECTOR_LINK = "joint6_flange"   # CHANGE
GROUP_NAME        = "arm"             # CHANGE

MAX_VEL  = 0.15   # keep slow during testing — increase when confident
MAX_ACCEL = 0.15


# ===========================================================================
# Keyboard helper (single keypress without Enter)
# ===========================================================================
def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def confirm(prompt: str) -> bool:
    print(f"\n  {prompt}  Ready? [y/N] ", end="", flush=True)
    k = get_key()
    print(k)
    return k.lower() == "y"


# ===========================================================================
# Test node
# ===========================================================================
class GraspTester(Node):
    def __init__(self):
        super().__init__("grasp_tester")

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR_LINK,
            group_name=GROUP_NAME,
            execute_via_moveit=True,
        )
        self.moveit2.max_velocity_scaling_factor     = MAX_VEL
        self.moveit2.max_acceleration_scaling_factor = MAX_ACCEL

        self.get_logger().info("GraspTester ready")

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------
    def _move(self, position, label: str):
        print(f"  → Planning to {label} {[f'{v:.3f}' for v in position]} …")
        self.moveit2.move_to_pose(position=position, quat_xyzw=DOWN_QUAT)
        self.moveit2.execute()
        print(f"  ✓ {label} done")

    def move_home(self):
        print("  → Planning to HOME …")
        self.moveit2.move_to_configuration(HOME_JOINTS)
        self.moveit2.execute()
        print("  ✓ HOME done")

    # ------------------------------------------------------------------
    # Gripper — CHANGE the body of these two methods to your gripper driver
    # (same options as in cube_grasp_node.py)
    # ------------------------------------------------------------------
    def open_gripper(self):
        print("  → OPEN gripper")
        # TODO: add your gripper open command here
        # e.g. publish to /gripper_control or call a service
        self.get_logger().warn("open_gripper() not implemented — add your command")

    def close_gripper(self):
        print("  → CLOSE gripper")
        # TODO: add your gripper close command here
        self.get_logger().warn("close_gripper() not implemented — add your command")

    # ------------------------------------------------------------------
    # Individual step targets
    # ------------------------------------------------------------------
    def step_pre_grasp(self):
        self._move([CUBE_X, CUBE_Y, CUBE_Z + PRE_GRASP_ABOVE], "PRE-GRASP")

    def step_grasp(self):
        self._move([CUBE_X, CUBE_Y, CUBE_Z + GRASP_ABOVE], "GRASP")

    def step_lift(self):
        self._move([CUBE_X, CUBE_Y, CUBE_Z + LIFT_ABOVE], "LIFT")

    def step_pre_place(self):
        self._move([BOX_X, BOX_Y, BOX_Z + PRE_PLACE_ABOVE], "PRE-PLACE")

    def step_place(self):
        self._move([BOX_X, BOX_Y, BOX_Z + PLACE_ABOVE], "PLACE")

    # ------------------------------------------------------------------
    # Sequences
    # ------------------------------------------------------------------
    def run_pick(self):
        print("\n  === PICK SEQUENCE ===")
        self.open_gripper()
        import time; time.sleep(0.4)
        self.step_pre_grasp()
        self.step_grasp()
        self.close_gripper()
        time.sleep(0.6)
        self.step_lift()
        print("  === PICK COMPLETE ===\n")

    def run_drop(self):
        print("\n  === DROP SEQUENCE ===")
        self.step_pre_place()
        self.open_gripper()
        import time; time.sleep(0.5)
        self.step_pre_place()   # retreat
        print("  === DROP COMPLETE ===\n")

    def run_full(self):
        self.run_pick()
        self.run_drop()
        self.move_home()


# ===========================================================================
# Interactive menu (runs in a separate thread so ROS2 keeps spinning)
# ===========================================================================
MENU = """
╔══════════════════════════════════════╗
║         Grasp tester controls        ║
╠══════════════════════════════════════╣
║  o  Open gripper                     ║
║  c  Close gripper                    ║
║  h  Home position                    ║
║  ─── individual steps ───            ║
║  1  Pre-grasp  (above cube)          ║
║  2  Grasp      (at cube)             ║
║  3  Lift       (after grasp)         ║
║  4  Pre-place  (above box)           ║
║  5  Place      (inside box)          ║
║  ─── sequences ───                   ║
║  p  Full PICK sequence (1→2→c→3)     ║
║  d  Full DROP sequence (4→o)         ║
║  f  Full cycle (pick + drop + home)  ║
║  ─────────────────────────────────── ║
║  q  Quit                             ║
╚══════════════════════════════════════╝
"""


def menu_thread(node: GraspTester):
    print(MENU)
    print("Current test positions:")
    print(f"  Cube: ({CUBE_X}, {CUBE_Y}, {CUBE_Z})")
    print(f"  Box:  ({BOX_X},  {BOX_Y},  {BOX_Z})")
    print()

    actions = {
        "o": ("Open gripper",       node.open_gripper),
        "c": ("Close gripper",      node.close_gripper),
        "h": ("Home",               node.move_home),
        "1": ("Pre-grasp",          node.step_pre_grasp),
        "2": ("Grasp",              node.step_grasp),
        "3": ("Lift",               node.step_lift),
        "4": ("Pre-place",          node.step_pre_place),
        "5": ("Place",              node.step_place),
        "p": ("Full PICK sequence", node.run_pick),
        "d": ("Full DROP sequence", node.run_drop),
        "f": ("Full cycle",         node.run_full),
    }

    while rclpy.ok():
        print("Key: ", end="", flush=True)
        k = get_key()
        print(k)

        if k == "q":
            print("Quitting …")
            rclpy.shutdown()
            break

        if k not in actions:
            print(f"  Unknown key '{k}'")
            continue

        label, fn = actions[k]
        if confirm(label):
            try:
                fn()
            except Exception as e:
                print(f"  ERROR: {e}")
        else:
            print("  Skipped.")


def main():
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()
    node     = GraspTester()
    executor.add_node(node)

    # Run menu in background thread, ROS2 spins on main thread
    t = threading.Thread(target=menu_thread, args=(node,), daemon=True)
    t.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

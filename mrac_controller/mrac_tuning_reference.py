#!/usr/bin/env python3
"""
MRAC Parameter Tuning Cheat Sheet
Interactive guide for quick parameter adjustments
"""

def show_tuning_grid():
    """Display a quick reference grid for parameter tuning"""
    
    grid = """
╔════════════════════════════════════════════════════════════════════════════╗
║                   MRAC PARAMETER TUNING DECISION TREE                      ║
╚════════════════════════════════════════════════════════════════════════════╝

PROBLEM: What's wrong with tracking?
═══════════════════════════════════════════════════════════════════════════════

┌─ Motion too SLOW
│  ├─ Solution 1: Increase wn (natural frequency)
│  │  └─ robust_direct_mrac_params.yaml:
│  │     wn: 5.0 → 7.0  or  7.0 → 9.0
│  │
│  └─ Solution 2: Decrease zeta (damping ratio)
│     └─ zeta: 0.70 → 0.60
│        ⚠️  May cause overshoot

├─ Motion too JERKY or OSCILLATES
│  ├─ Solution 1: Increase zeta
│  │  └─ zeta: 0.70 → 0.85
│  │
│  └─ Solution 2: Decrease wn
│     └─ wn: 5.0 → 3.5

├─ Final STEADY-STATE error too HIGH (> 0.1 rad)
│  ├─ Solution 1: Increase gamma_Kx
│  │  └─ gamma_Kx: 0.50 → 0.75 → 0.95
│  │
│  └─ Solution 2: Increase gamma_W
│     └─ gamma_W: 2.00 → 3.00 → 4.00

├─ Motion is NOISY or CHATTERING
│  ├─ Solution 1: Decrease gamma_W
│  │  └─ gamma_W: 2.00 → 1.50 → 1.00
│  │
│  └─ Solution 2: Decrease gamma_Kx
│     └─ gamma_Kx: 0.50 → 0.35 → 0.20

├─ Lyapunov V INCREASES (❌ unstable)
│  └─ CRITICAL: Decrease ALL gamma values
│     ├─ wn: 5.0 → 4.0
│     ├─ gamma_Kx: 0.50 → 0.25
│     ├─ gamma_W: 2.00 → 1.00
│     └─ Rebuild and retest!

└─ Nothing seems to help
   └─ Check these first:
      ├─ ✓ Gazebo and Gazebo bridge running?
      ├─ ✓ All controllers active?
      │  ros2 control list_controllers
      ├─ ✓ /joint_states publishing?
      │  ros2 topic echo /joint_states
      ├─ ✓ MRAC action server visible?
      │  ros2 action list | grep mrac
      └─ ✓ moveit_controllers.yaml has correct params?
         ros2 param get /move_group mrac_arm_controller.type


PARAMETER RANGES (Safe Operating Window)
═══════════════════════════════════════════════════════════════════════════════

Parameter      Current   Minimum   Maximum   Effect of Increase
─────────────  ───────   ───────   ───────   ──────────────────────────
wn             5.0       2.0       10.0      Faster response (but jerky)
zeta           0.70      0.50      1.20      Less overshoot (but slower)
gamma_Kx       0.50      0.10      2.00      Better steady-state (noise)
gamma_Kr       0.50      0.10      2.00      Better reference tracking
gamma_W        2.00      0.50      10.0      Handle more uncertainty


QUICK TUNING RECIPES
═══════════════════════════════════════════════════════════════════════════════

🎯 Aggressive (fast but potentially unstable):
   wn: 8.0
   zeta: 0.65
   gamma_Kx: 0.85
   gamma_W: 3.00
   → Use if: You want speed and robustness is okay

🎯 Balanced (recommended starting point):
   wn: 5.0
   zeta: 0.70
   gamma_Kx: 0.50
   gamma_W: 2.00
   → Use if: General good tracking with smooth motion

🎯 Conservative (slow but very stable):
   wn: 3.5
   zeta: 0.85
   gamma_Kx: 0.30
   gamma_W: 1.00
   → Use if: Stability is critical, speed doesn't matter

🎯 Heavy Load (payload_mass > 0.5 kg):
   wn: 4.0
   zeta: 0.75
   gamma_Kx: 0.75
   gamma_W: 4.00
   → Use if: Carrying heavy objects


TUNING WORKFLOW
═══════════════════════════════════════════════════════════════════════════════

RUN 1: Baseline
└─ Use default or "Balanced" recipe
   └─ MEASURE: Final error, convergence speed

RUN 2: Adjust ONE parameter only
├─ Pick problem from decision tree
├─ Make smallest recommended change
├─ MEASURE: Same metrics as RUN 1
└─ Compare to RUN 1

RUN 3-N: Iterate
└─ Repeat RUN 2 until satisfied

✅ BEST PRACTICE: Change ONE parameter by the SMALLEST amount


WHAT TO CHECK IN PLOTS
═════════════════════════════════════════════════════════════════════════════

For each joint error plot:

1. Initial error (first 0.5s)
   ├─ Should be ~0.3-0.8 rad (MoveIt planning range)
   └─ If zero: arm already at target

2. Convergence slope
   ├─ Fast (reaches 0.05 in 1-2s): Good tracking
   ├─ Slow (takes 5+ seconds): Increase wn or gamma_Kx
   └─ Oscillates: Increase zeta

3. Final steady-state (last 0.5s)
   ├─ < 0.01 rad: Excellent ✅
   ├─ 0.01-0.05 rad: Good ✓
   ├─ 0.05-0.10 rad: Acceptable (increase gamma_Kx)
   └─ > 0.10 rad: Poor (need big change)

4. Lyapunov V function
   ├─ Always decreasing: Stable ✅
   ├─ Flat then increases: Problem ❌
   └─ Spikes up: Discontinuity (normal at transitions)


PERFORMANCE TARGETS
═════════════════════════════════════════════════════════════════════════════

Characteristic        Target           Status
──────────────────    ────────────    ──────────
Rise time            < 1.0 s          ✅ If achieved
Steady-state error   < 0.03 rad       ✅ Excellent
Overshoot            < 5°             ✅ Smooth motion
Oscillation damping  Damp in 1s       ✅ No chatter
Lyapunov monotonic   Always decrease  ✅ Critical


RECORD-KEEPING
═══════════════════════════════════════════════════════════════════════════════

Create file: ~/mrac_tuning_log.txt

Format for each run:

Date: 2026-04-13
Run ID: run_001
Parameters:
  wn: 5.0
  zeta: 0.70
  gamma_Kx: 0.50
  gamma_W: 2.00
  
Results:
  Max error (joint1): 0.12 rad
  Final error (joint1): 0.023 rad
  Convergence time: 1.8 s
  Notes: Good tracking, slight oscillation
  
Next experiment: Increase gamma_Kx to 0.75


COMMON MISTAKES TO AVOID
═════════════════════════════════════════════════════════════════════════════

❌ MISTAKE: Changing multiple parameters at once
   → You won't know which one helped/hurt
   → Change ONE parameter per run

❌ MISTAKE: Not rebuilding after editing YAML
   → MRAC only reads params at startup
   → Always: colcon build, then relaunch

❌ MISTAKE: Running test while parameters are being searched
   → Let monitor settle for 5 seconds first
   → Start monitor BEFORE starting test

❌ MISTAKE: Dismissing noisy Lyapunov
   → Some noise is OK (measurement noise)
   → But V should never INCREASE overall
   → If it does, your adaptation is diverging

❌ MISTAKE: Trying to get zero error
   → 0.01-0.03 rad is realistic
   → Perfect tracking requires infinite gains (unstable)


DEBUGGING: "Lyapunov V is increasing!"
═══════════════════════════════════════════════════════════════════════════════

If you see V(t) going UP, the controller is DIVERGING.

Fix it:
  1. Stop the test (Ctrl+C)
  
  2. Reduce ALL gains by 50%:
     gamma_Kx: 0.50 → 0.25
     gamma_W: 2.00 → 1.00
     
  3. Also reduce wn:
     wn: 5.0 → 4.0
     
  4. Rebuild and retest:
     colcon build
     ros2 launch my_robot_bringup my_robot_gz.launch.py
     
  5. Check if V now decreases smoothly
  
  6. If YES: Slowly increase wn and gammas back up
     If NO: Reduce more aggressively

Root causes:
• Measurement noise too high
• Adaptation learning rate too fast
• Gazebo simulation not giving clean dynamics
"""
    
    print(grid)


def show_parameter_adjustment():
    """Show specific parameter change amounts"""
    
    adjustments = """
PARAMETER ADJUSTMENT REFERENCE
═══════════════════════════════════════════════════════════════════════════════

When to make a SMALL change (±0.5):    When to make a MEDIUM change (±1.0):
├─ You're near your target              ├─ Error is off by factor of 2-3
├─ Fine-tuning convergence speed        ├─ Need significant improvement
└─ Last run was very close             └─ Trying a different recipe

When to make a BIG change (±2.0+):
├─ Complete failure (error > 0.5 rad)
├─ Unstable (Lyapunov increasing)
└─ Need emergency backup to known params


SAFE CHANGE SEQUENCE
═══════════════════════════════════════════════════════════════════════════════

Try this order. Stop when you're happy:

Step 1. wn: 5.0 → 6.0         [ natural frequency up 20% ]
       Test. If better: continue. If worse: revert and skip to Step 2

Step 2. gamma_Kx: 0.50 → 0.65 [ direct adaptation gain up 30% ]
        Test. If better: continue. If worse: revert to 0.50

Step 3. zeta: 0.70 → 0.75     [ damping ratio up slightly ]
        Test. If better: keep. If worse: revert

Step 4. gamma_W: 2.00 → 2.5   [ W adaptation gain up slightly ]
        Test and verify Lyapunov still decreasing


RECOVERY SEQUENCE (if things go wrong)
═══════════════════════════════════════════════════════════════════════════════

If Lyapunov increases or motion becomes unstable:

  IMMEDIATE: Set to CONSERVATIVE recipe
  └─ wn: 3.5, zeta: 0.85, gamma_Kx: 0.30, gamma_W: 1.00
  
  THEN: colcon build && relaunch && test
  
  RESULT: Very slow but stable motion
  
  NEXT: Use Step 1-4 above to improve from here


COPY-PASTE PARAMETER TEMPLATES
═══════════════════════════════════════════════════════════════════════════════

For editing robust_direct_mrac_params.yaml:

# FAST (aggressive):
wn: 8.0
zeta: 0.65
gamma_Kx: 0.85
gamma_Kr: 0.85
gamma_W: 3.00

# BALANCED (recommended):
wn: 5.0
zeta: 0.70
gamma_Kx: 0.50
gamma_Kr: 0.50
gamma_W: 2.00

# STABLE (conservative):
wn: 3.5
zeta: 0.85
gamma_Kx: 0.30
gamma_Kr: 0.30
gamma_W: 1.00

# HEAVY_LOAD (payload > 0.5 kg):
wn: 4.0
zeta: 0.75
gamma_Kx: 0.75
gamma_Kr: 0.75
gamma_W: 4.00
"""
    
    print(adjustments)


if __name__ == '__main__':
    import sys
    
    print("\n" + "="*80)
    print("MRAC PARAMETER TUNING CHEAT SHEET")
    print("="*80 + "\n")
    
    if len(sys.argv) > 1 and sys.argv[1] == 'adjust':
        show_parameter_adjustment()
    else:
        show_tuning_grid()
    
    print("\n" + "="*80)
    print("Usage:")
    print("  python3 mrac_tuning_reference.py           # Show tuning grid")
    print("  python3 mrac_tuning_reference.py adjust    # Show adjustment sizes")
    print("="*80 + "\n")

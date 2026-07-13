#!/usr/bin/env python3
"""
FRICTION AND DAMPING IN GAZEBO + ROS2
======================================

This document explains the damping/friction settings now applied to the robot.


1. WHAT WAS CHANGED
====================

File: my_robot_description/urdf/arm.xacro

Each joint now has realistic damping and friction values:

Joint   Damping   Friction   Why
-----   -------   --------   ---
j1      1.0       0.3        Base joint - supports full arm (~6kg)
j2      0.8       0.25       Shoulder - heavy load
j3      0.6       0.15       Elbow - medium load
j4      0.2       0.08       Wrist - light load

The rebuild is required:
cd ~/moveit_series/robot_arm
colcon build


2. UNDERSTANDING JOINT DYNAMICS
================================

In URDF, each joint has:

  <dynamics damping="D" friction="F"/>

Where:
• D = damping (Nм·s/rad) - VISCOUS damping
  - Proportional to velocity
  - Acts like oil in hydraulics
  - Formula: τ_damping = -D * ω (opposes motion)
  
• F = friction (Nm) - COULOMB friction
  - Constant opposing motion
  - Acts like mechanical friction
  - Formula: τ_friction = -F * sign(ω)


3. EFFECTS OF DAMPING
======================

High damping (D = 1.0):
✓ Smooth, stable motion
✓ No overshoot or oscillation
✗ Slower convergence
✗ Higher energy consumption
→ Use for base joints supporting heavy loads

Low damping (D = 0.2):
✓ Faster motion
✓ More responsive
✗ Can overshoot or oscillate
✗ Requires stronger controller
→ Use for lightweight End-Effector joints


4. EFFECTS OF FRICTION
======================

High friction (F = 0.3):
✓ Prevents drift from gravity (slows creep)
✓ Realistic for heavy motors
✗ Requires more input torque to move
- Typical for large industrial arms

Low friction (F = 0.08):
✓ Easier to move (less resistance)
✓ Lower power consumption
✗ Joint might slowly creep under gravity
- Typical for lightweight arms


5. TUNING FOR YOUR ROBOT
=========================

After implementing new damping, you may notice:

Case A: "Motion is now very smooth but slower"
   → Damping is working. This is expected.
   → If too slow for your needs:
   
   Option 1: Increase wn (natural frequency) in MRAC
      robust_direct_mrac_params.yaml:
      wn: 5.0  →  wn: 8.0
   
   Option 2: Reduce damping slightly
      arm.xacro:
      joint1: damping="1.0" → damping="0.7"

Case B: "Arm no longer freely falls under gravity"
   → Friction is now active. This is realistic.
   → Real motors have friction too.
   → MRAC should overcome it automatically.
   
   To verify friction compensation working:
   ros2 topic echo /mrac/K_hat
   Should see constant values after convergence.

Case C: "Test fails with START_STATE_INVALID error"
   → Not related to damping. See moveit_controllers.yaml


6. GAZEBO SURFACE PROPERTIES (Advanced)
========================================

For even more realism, you can add surface contact properties.
Add this to gripper.xacro for each link:

  <gazebo reference="left_finger_link">
    <mu1>0.7</mu1>      <!-- Friction coefficient with other objects -->
    <mu2>0.7</mu2>
    <restitution>0.0</restitution>  <!-- Bounciness (0 = no bounce) -->
    <maxContacts>1</maxContacts>
    <surface>
      <friction>
        <ode>
          <mu>0.7</mu>
          <mu2>0.7</mu2>
          <fdir1>1.0 0 0</fdir1>
        </ode>
      </friction>
    </surface>
  </gazebo>

But for MRAC tuning, the joint damping/friction is sufficient.


7. COMPARING REAL ROBOT PARAMETERS
===================================

Real Collaborative Robot (UR10e):
• Damping: ~0.5 - 2.0 Nm·s/rad
• Friction: ~0.2 - 0.8 Nm
• Max velocity: 3.0 rad/s

Real Industrial Robot (KUKA KR10):
• Damping: ~1.0 - 3.0 Nm·s/rad
• Friction: ~0.5 - 1.5 Nm
• Max velocity: 2.0 rad/s

Your simulated robot:
• Damping: 0.2 - 1.0 Nm·s/rad  ✓ (reasonable for medium arm)
• Friction: 0.08 - 0.3 Nm      ✓ (reasonable)


8. WORKFLOW WITH NEW DAMPING
=============================

RUN 1: Default MRAC params with NEW damping
   ros2 launch my_robot_bringup my_robot_gz.launch.py
   ros2 run my_robot_commander_py test_moveit
   python3 mrac_monitor.py  (type 'p' to plot)
   → Baseline with realistic physics

RUN 2: Compare to old behavior
   Temporarily reduce damping back:
   joint1: damping="0.5" → damping="1.0"
   colcon build
   Re-run test
   → Should be slower but smoother

RUN 3: Optimize MRAC for the damping
   If convergence is now slow:
   Increase gamma_Kx: 0.50 → 0.75
   Increase wn: 5.0 → 7.0
   colcon build
   Re-run test
   → Find sweet spot


9. KEY LEARNINGS FOR MRAC
==========================

MRAC works by learning model uncertainties:
• When you add damping/friction, MRAC sees this as "unknown dynamics"
• MRAC's W matrix learns to compensate
• The /mrac/W_hat topic shows what's being learned

If you increase damping too much, MRAC must work harder:
✓ Test with current settings first (damping = 1.0, 0.8, 0.6, 0.2)
✓ If MRAC can't keep up, reduce damping slightly
✓ Then increase MRAC adaptation gains (gamma_W, gamma_Kx)

Best practice:
  Model uncertainty from damping/friction ≈ MRAC learning capacity
  
Hence:
  - Realistic damping (1.0, 0.8, 0.6, 0.2) with
  - Moderate MRAC gains (gamma_W = 2.0, gamma_Kx = 0.5)
  - Usually works better than
  - High damping with very high MRAC gains


10. RESET DAMPING BACK (if needed)
====================================

If you want to go back to minimal damping:

arm.xacro:
  joint1: damping="1.0" → damping="0.3"
  joint2: damping="0.8" → damping="0.3"
  joint3: damping="0.6" → damping="0.2"
  joint4: damping="0.2" → damping="0.1"

  And reduce friction by 50%:
  joint1: friction="0.3" → friction="0.1"
  joint2: friction="0.25" → friction="0.08"
  etc.

Then:
  colcon build
  Re-tune MRAC with lower gains (divide by 2)
"""

# This is a reference document. Read it to understand the damping changes.

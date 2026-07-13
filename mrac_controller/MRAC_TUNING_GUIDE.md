"""
MRAC Tuning Guide for ROS2 Jazzy
==================================

This file explains each MRAC parameter and how to tune them based on test results.

MRAC Parameters (in robust_direct_mrac_params.yaml):
====================================================

1. wn (Natural Frequency) - rad/s
   Default: 5.0
   Range: 2.0 - 15.0
   
   What it does: Higher wn = faster reference model response
   
   Tuning:
   • Too low (< 3.0): Arm moves too slowly, controller looks sluggish
   • Too high (> 10.0): Jerky motion, increased overshoot, higher torque demands
   • Sweet spot: 5-7 rad/s for most 4-DOF arms
   
   ✅ How to adjust:
      If: Tracking error decreases very slowly
      Then: Increase wn from 5.0 to 7.0
      
      If: Arm oscillates or jerks
      Then: Decrease wn from 5.0 to 3.5


2. zeta (Damping Ratio) - pure ratio
   Default: 0.70
   Range: 0.5 - 1.2
   
   What it does: Controls overshoot in reference model
   Formula: wn and zeta determine pole locations via ζ = damping/critical_damping
   
   Tuning:
   • zeta = 0.707: Butterworth response (fastest with no overshoot)
   • zeta < 0.707: Underdamped (oscillatory but fast)
   • zeta > 0.707: Overdamped (slow but no overshoot)
   
   ✅ How to adjust:
      If: Desired trajectory overshoots then settles
      Then: Increase zeta from 0.70 to 0.85
      
      If: Settling takes forever
      Then: Decrease zeta from 0.70 to 0.65 (trade off some overshoot for speed)


3. gamma_Kx (Adaptation Gain for K matrix) - rad/s²
   Default: 0.50
   Range: 0.1 - 2.0
   
   What it does: Speed of learning direct adaptive controller gains
   
   Tuning:
   • Too low (< 0.2): Doesn't learn fast enough, steady-state error remains
   • Too high (> 1.5): Noisy adaptation, instability on disturbances
   • Sweet spot: 0.5 - 0.8
   
   ✅ How to adjust:
      If: Final tracking error > 0.05 rad after convergence
      Then: Increase gamma_Kx from 0.50 to 0.75
      
      If: Kalman filter outputs jittery (see /mrac/K_hat topic)
      Then: Decrease gamma_Kx from 0.50 to 0.30


4. gamma_Kr (Adaptation Gain for r gain) - rad/s²
   Default: 0.50
   Range: 0.1 - 2.0
   
   What it does: Speed of learning feedforward (tracking) gain
   
   Tuning:
   • gamma_Kr ≈ gamma_Kx usually works best
   • Can be lower if only direct learning is needed
   
   ✅ How to adjust:
      Similar to gamma_Kx. If reference tracking is poor:
      Increase gamma_Kr to 0.75


5. gamma_W (Adaptation Gain for W matrix) - rad/s²
   Default: 2.00
   Range: 0.5 - 10.0
   
   What it does: Speed of learning unmodeled dynamics basis weights
   
   Tuning:
   • Controls robustness against model uncertainty
   • Higher gamma_W = can handle more unknown friction/damping
   • Too high (> 5): Noisy, can amplify measurement noise
   
   ✅ How to adjust:
      If: Arm stutters or jerks unexpectedly (due to friction)
      Then: Increase gamma_W from 2.0 to 4.0
      
      If: Adaptation is too aggressive (lots of chatter)
      Then: Decrease gamma_W from 2.0 to 1.0


TUNING WORKFLOW
===============

After each `ros2 run my_robot_commander_py test_moveit` run:

Step 1: View plots
   python3 mrac_monitor.py
   (type 'p' to show plots, 'q' to quit)

Step 2: Read the summary
   Look at error statistics for each joint:
   • Initial error: How bad was the first second?
   • Final error: Did it converge?
   • Convergence rate: How fast did error shrink?

Step 3: Apply ONE change only
   Example sequence:
   
   RUN 1: Default params
   → Check initial error (should be ~0.2-0.5 rad)
   → If final error is high, go to RUN 2
   
   RUN 2: Increase gamma_Kx to 0.75
   → If error improved but too oscillatory, go to RUN 3
   
   RUN 3: Increase zeta to 0.80
   → If now smooth, keep these changes
   
   RUN 4: Try increasing gamma_W to 3.0
   → Check Lyapunov function (should always decrease)

Step 4: Update robust_direct_mrac_params.yaml
   Example edit:
   
   # Before:
   wn: 5.0
   
   # After:
   wn: 7.0
   
   Then rebuild:
   cd ~/moveit_series/robot_arm
   colcon build

Step 5: Rerun test_moveit and compare


COMMON SCENARIOS AND FIXES
===========================

❌ "Arm overshoots desired position then oscillates"
   → Try: Increase zeta from 0.70 to 0.85
   → Or: Decrease wn from 5.0 to 3.5

❌ "Arm approaches position very slowly"
   → Try: Increase wn from 5.0 to 8.0
   → Or: Decrease zeta from 0.70 to 0.60

❌ "Final steady-state error is large (> 0.1 rad)"
   → Try: Increase gamma_Kx from 0.50 to 0.85
   → Or: Increase gamma_W from 2.0 to 3.5

❌ "Motion is jerky or chattery (noisy)"
   → Try: Decrease gamma_W from 2.0 to 1.0
   → Or: Decrease gamma_Kx from 0.50 to 0.30

❌ "Lyapunov function increases (goes up) even briefly"
   → STOP. This means controller not converging.
   → Try: Decrease ALL gamma values by 0.2
   → Or: Decrease wn by 1.0 rad/s

✅ "Good tracking with smooth motion"
   → You've found good baseline parameters!
   → Now can increase payloads (modify mass in launch args)


MONITORING TOPICS
==================

Real-time monitoring during test:

Terminal 3:
ros2 topic echo /mrac/error                    # Joint tracking errors
ros2 topic echo /mrac/K_hat                    # Estimated controller gains
ros2 topic echo /mrac/W_hat                    # Estimated NN weights
ros2 topic echo /mrac/lyapunov_V              # Stability metric


PAYLOAD TUNING
===============

After tuning for zero payload, test with different masses:

cd ~/moveit_series/robot_arm
ros2 launch my_robot_bringup my_robot_gz.launch.py payload_mass:=0.5
# Test results show worse tracking?
# Then increase gamma_W to compensate for added uncertainty


FINAL TIPS
===========

1. Always rebuild after changing params:
   colcon build
   
2. Give system ~3 seconds to stabilize after startup

3. Lyapunov function MUST always decrease:
   If V ever increases, something is wrong

4. Start with conservative gains, slowly increase
   Better to be slow and stable than fast and unstable

5. Use `ros2 bag` to record topics for offline analysis:
   ros2 bag record /mrac/error /mrac/K_hat /mrac/lyapunov_V
"""

# This file is meant to be read, not executed

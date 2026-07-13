#!/bin/bash
# Quick reference for MRAC tuning workflow
# Usage: source mrac_tuning_workflow.sh

set -e

WORKSPACE="/home/gg/moveit_series/robot_arm"

echo "╔════════════════════════════════════════════════════════╗"
echo "║           MRAC TUNING WORKFLOW - QUICK START            ║"
echo "╚════════════════════════════════════════════════════════╝"
echo

# 1. Build
echo "📦 STEP 1: Build workspace (run once after changes)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
cat << 'EOF'

cd ~/moveit_series/robot_arm
colcon build

EOF

# 2. Launch
echo "🚀 STEP 2: Launch simulation"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
cat << 'EOF'

Terminal 1:
  ros2 launch my_robot_bringup my_robot_gz.launch.py
  
  (let it run for 30 seconds until RViz opens)

EOF

# 3. Monitor
echo "📊 STEP 3: Start error monitor"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
cat << 'EOF'

Terminal 2:
  cd ~/moveit_series/robot_arm/src/my_robot_commander_py
  python3 -m my_robot_commander_py.mrac_monitor
  
  (it will wait for data)

EOF

# 4. Run test
echo "🧪 STEP 4: Run test"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
cat << 'EOF'

Terminal 3:
  ros2 run my_robot_commander_py test_moveit
  
  (watch arm move in RViz)

EOF

# 5. Plot
echo "📈 STEP 5: Analyze results"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
cat << 'EOF'

Back in Terminal 2:
  Press 'p' to generate plots
  
  Plots saved to: /tmp/mrac_error_analysis.png
  Summary saved to: /tmp/mrac_tuning_summary.txt
  
  View plots:
    display /tmp/mrac_error_analysis.png
    cat /tmp/mrac_tuning_summary.txt

EOF

# 6. Tune
echo "🔧 STEP 6: Adjust parameters"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
cat << 'EOF'

Edit: src/mrac_controller/robust_direct_mrac_params.yaml

Example changes:

If final error > 0.1 rad:
  gamma_Kx: 0.50 → 0.75
  gamma_W: 2.00 → 3.00

If motion is jerky/noisy:
  gamma_W: 2.00 → 1.00
  gamma_Kx: 0.50 → 0.30

If motion is too slow:
  wn: 5.0 → 7.0
  zeta: 0.70 → 0.65

Then rebuild:
  colcon build

EOF

# 7. Loop
echo "♻️  STEP 7: Repeat"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
cat << 'EOF'

Go back to STEP 4 and run test_moveit again
Compare new plots to previous run
Make ONE parameter change at a time

EOF

# 8. Advanced
echo "🎓 Advanced topics"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
cat << 'EOF'

Test with payload:
  ros2 launch my_robot_bringup my_robot_gz.launch.py payload_mass:=0.5
  
  After test, if error increases:
  gamma_W: 2.0 → 4.0  (learn heavier dynamics)

Record all topics:
  ros2 bag record -a -o ~/bag_mrac_run1
  
  Replay anytime:
  ros2 bag play ~/bag_mrac_run1

View real-time topics:
  ros2 topic echo /mrac/error
  ros2 topic echo /mrac/lyapunov_V
  
Review damping/friction:
  cat src/my_robot_description/DAMPING_FRICTION_GUIDE.md

Review MRAC parameters:
  cat src/mrac_controller/MRAC_TUNING_GUIDE.md

EOF

echo "════════════════════════════════════════════════════════"
echo "✅ Workflow ready! Follow the steps above."
echo "════════════════════════════════════════════════════════"

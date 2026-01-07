# Quick Reference: Testing the Gripper Fix

## Pre-Test Verification (Run Once)

```bash
cd /home/ros/aoc_hunter_ws

# Rebuild the package
colcon build --packages-select mobile_manipulator_pltf_description --symlink-install

# Run automated tests
bash src/.ai-docs/test_gripper.sh
```

Expected output:
```
✓ Gripper found in URDF (56 references)
✓ Found 9 gripper links
✓ Mimic joints configured (5 found)
✓ Gazebo ros2_control plugin found
✓ Position command interfaces configured (6)
✓ Self-collision enabled for mimic joints
✓ All checks passed!
```

---

## Launch Simulation

```bash
# Terminal 1: Start Gazebo and RViz
ros2 launch mobile_manipulator_pltf_description sim_bringup.launch.py \
    use_gazebo:=true use_rviz:=true

# Output should show:
# [spawn_entity.py-X] entity name: mobile_manipulator
# [robot_state_publisher-X] Publishing transforms for [...robotiq_85...]
```

---

## Test Gripper Motion

```bash
# Terminal 2: Open gripper
ros2 action send_goal /gripper_position_controller/gripper_cmd \
    control_msgs/action/GripperCommand \
    "{command: {position: 0.0, max_effort: 100.0}}"

# Terminal 2: Close gripper
ros2 action send_goal /gripper_position_controller/gripper_cmd \
    control_msgs/action/GripperCommand \
    "{command: {position: 0.4, max_effort: 100.0}}"
```

---

## Verify in Gazebo

**What you should see:**

1. **Gripper visible in 3D view**
   - Before: Fingers missing (gripper invisible)
   - After: All gripper links visible and properly positioned

2. **Gripper moves when commanded**
   - Before: Fingers locked in place
   - After: Fingers open/close smoothly

3. **Gripper can interact with objects**
   - Before: Objects fall through gripper
   - After: Gripper can grasp and manipulate objects

---

## Monitor Joint States

```bash
# Terminal 3: Watch gripper joint positions
ros2 topic echo /joint_states --field message.name message.position | \
    grep -A 1 robotiq

# Expected: When gripper closes (position: 0.4)
#   robotiq_85_left_knuckle_joint:     0.4
#   robotiq_85_right_knuckle_joint:   -0.4  (multiplied!)
#   robotiq_85_left_inner_knuckle_joint: 0.4
#   robotiq_85_right_inner_knuckle_joint: -0.4
#   robotiq_85_left_finger_tip_joint: -0.4
#   robotiq_85_right_finger_tip_joint: 0.4
```

---

## Success Criteria

✅ **Gripper is fixed when:**
1. All 9 gripper links visible in Gazebo 3D view
2. Fingers move when gripper commands sent
3. `ros2 topic echo /joint_states` shows all robotiq joints moving
4. Gripper can grasp objects (collision working)
5. No errors in Gazebo console about robotiq joints

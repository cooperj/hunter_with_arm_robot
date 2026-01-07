# Robotiq 85 Gripper - Gazebo Classic Visibility Fix

## Problem Analysis

The Robotiq 85 gripper was fully visible and functional in RViz but **not appearing in Gazebo Classic** client. Root cause: **Gazebo Classic does not natively support mimic joints** defined in URDF `<mimic>` tags.

### Error Pattern
```
Joint 'robotiq_85_right_finger_tip_joint_mimic' not found in model 'mobile_manipulator_pltf'
Joint 'robotiq_85_left_inner_knuckle_joint_mimic' not found in model 'mobile_manipulator_pltf'
```
These "_mimic" suffix errors indicate MoveIt expected mimic joints to be explicitly handled by Gazebo, but they were being ignored.

---

## Root Cause Analysis

1. **URDF has the gripper correctly defined** - 60 references to `robotiq_85` elements, with all 9 gripper links present and proper `<mimic>` tags on 5 joints

2. **RViz works** because:
   - `robot_state_publisher` uses the URDF directly and honors `<mimic>` tags
   - MoveIt/`ros2_control` handles mimic joints via the `<param name="mimic">` parameters in the `<ros2_control>` block

3. **Gazebo Classic fails** because:
   - It parses URDF `<mimic>` tags but **does not simulate them**
   - Mimic joint links remain at position 0, making them appear invisible or stuck
   - The `gazebo_ros2_control/GazeboSystem` plugin has limited mimic joint support by default

---

## Solution: Option 2 - ros2_control Configuration Fixes

### Problem 1: Plugin Conflict
The gripper's ros2_control had **both** `gazebo_ros2_control/GazeboSystem` AND `mock_components/GenericSystem` plugins.

**Fix:** Modified conditional to prevent MockSystem when using Gazebo:
```xml
<!-- Before -->
<xacro:if value="${use_fake_hardware}">
    <plugin>mock_components/GenericSystem</plugin>

<!-- After -->
<xacro:if value="${use_fake_hardware and not sim_gazebo}">
    <plugin>mock_components/GenericSystem</plugin>
```

### Problem 2: Missing Gazebo Support for Mimic Mechanics
Mimic joints don't have proper collision/physics configuration for Gazebo.

**Fix:** Added `<selfCollide>true</selfCollide>` for mimic joint child links
```xml
<xacro:if value="${sim_gazebo}">
    <gazebo reference="${prefix}robotiq_85_right_knuckle_link">
        <selfCollide>true</selfCollide>
    </gazebo>
    <!-- ... same for other mimic joint links ... -->
</xacro:if>
```

---

## Why This Works

1. **ros2_control manages the mimic relationship** - The controller applies position commands to ALL joints (including mimic) through the hardware interface

2. **gazebo_ros2_control plugin propagates commands** - Acts as the physics interface, allowing Gazebo to move all controlled joints

3. **selfCollide enables physics response** - Mimic joint links can now collide with objects and each other, making them "real" in the simulation

4. **No special Gazebo plugins needed** - Uses only standard ros2_control + Gazebo infrastructure

---

## Troubleshooting

### Gripper still not visible in Gazebo?

1. **Check if links are in the scene tree:**
   - Gazebo GUI → View → Translate/rotate → Click on robot
   - Left panel should expand showing all links including gripper

2. **Check joint states are publishing:**
   ```bash
   ros2 topic echo /joint_states | grep robotiq
   ```
   Should show all `robotiq_85_*` joints

3. **Verify collision meshes load:**
   - Check console output for "Error loading mesh" messages
   - Ensure `robotiq_description` package is installed

### Gripper moves in RViz but not in Gazebo?

1. **Check controller activation:**
   ```bash
   ros2 control list_controllers
   ```
   Should show `gripper_position_controller` as active

2. **Check ros2_control hardware interface:**
   - Verify only `gazebo_ros2_control/GazeboSystem` is loaded (not mock_components)

3. **Monitor Gazebo console for errors:**
   - Look for failed joint name resolution

---

## Testing Procedure

1. **Rebuild:**
   ```bash
   colcon build --packages-select mobile_manipulator_pltf_description
   ```

2. **Verify fix:**
   ```bash
   bash /home/ros/aoc_hunter_ws/src/.ai-docs/test_gripper.sh
   ```

3. **Launch simulation:**
   ```bash
   ros2 launch mobile_manipulator_pltf_description sim_bringup.launch.py use_gazebo:=true
   ```

4. **Test gripper:**
   ```bash
   ros2 action send_goal /gripper_position_controller/gripper_cmd \
       control_msgs/action/GripperCommand \
       "{command: {position: 0.4, max_effort: 100.0}}"
   ```

---

## Success Criteria

✅ **Gripper is fixed when:**
1. All 9 gripper links visible in Gazebo 3D view
2. Fingers move when gripper commands sent
3. `ros2 topic echo /joint_states` shows all robotiq joints moving
4. Gripper can grasp objects (collision working)
5. No errors in Gazebo console about robotiq joints

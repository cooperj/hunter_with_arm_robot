# UR5 + Robotiq 2F-85 Gazebo Classic Invisibility - Diagnostic Report

## 🔍 Executive Summary

**Issue:** Robotiq 2F-85 gripper not visible in Gazebo Classic despite UR5 arm rendering correctly.

**Root Cause Identified:** ⚠️ **DUAL PLUGIN CONFLICT** in ros2_control

When `use_fake_hardware="true"` AND `sim_gazebo="true"` are both set, the gripper's ros2_control section loads **TWO incompatible plugins**:
1. `gazebo_ros2_control/GazeboSystem` (Gazebo simulation plugin)
2. `mock_components/GenericSystem` (Fake hardware mock)

This causes **gripper state interface failure**, making Gazebo unable to render the gripper or accept commands.

---

## ✅ DIAGNOSTIC RESULTS (Ran January 7, 2026)

### Check 1: Mesh File Installation
```
Status: ✅ PASS
Found: 10+ DAE files in install/robotiq_description/share/robotiq_description/meshes/visual/2f_85/
Examples:
  - robotiq_base.dae ✓
  - left_knuckle.dae ✓
  - right_finger.dae ✓
  - left_inner_knuckle.dae ✓
```

### Check 2: Gripper in Compiled URDF
```
Status: ✅ PASS
Robotiq references: 66 (expected >10)
All gripper links present in URDF ✓
```

### Check 3: Mesh Paths Resolution
```
Status: ✅ PASS
Sample from URDF:
  <mesh filename="package://robotiq_description/meshes/visual/2f_85/robotiq_base.dae"/>
  <mesh filename="package://robotiq_description/meshes/visual/2f_85/left_knuckle.dae"/>
Paths resolve correctly ✓
```

### Check 4: Gazebo Mimic Joint Plugins
```
Status: ✅ PASS
libgazebo_mimic_joint_plugin count: 5
Plugins present for:
  - Right knuckle ✓
  - Left inner knuckle ✓
  - Right inner knuckle ✓
  - Left finger tip ✓
  - Right finger tip ✓
```

### Check 5: Ros2_Control Plugin Selection (CRITICAL)
```
Status: ⚠️ CONFLICT DETECTED
Found in URDF:
  <plugin>gazebo_ros2_control/GazeboSystem</plugin>      ← Gazebo simulator
  <plugin>mock_components/GenericSystem</plugin>         ← Fake hardware mock
  
Problem: BOTH plugins loaded simultaneously!
Root cause: use_fake_hardware="true" AND sim_gazebo="true"
```

---

## 🎯 ROOT CAUSE: Plugin Logic Flaw

**File:** [robotiq_gripper.ros2_control.xacro](../../mobile_manipulator_pltf_description/description/robotiq_gripper.ros2_control.xacro)

**Current Logic (BROKEN):**
```xml
<xacro:if value="${sim_gazebo}">
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
</xacro:if>
<!-- ... -->
<xacro:if value="${use_fake_hardware}">
    <plugin>mock_components/GenericSystem</plugin>
    <!-- ... -->
</xacro:if>
```

**Issue:** Both conditions can be TRUE simultaneously, causing dual plugin load.

**Correct Logic (FIXED):**
```xml
<xacro:if value="${sim_gazebo}">
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
</xacro:if>
<xacro:if value="${sim_ignition}">
    <plugin>ign_ros2_control/IgnitionSystem</plugin>
</xacro:if>
<xacro:if value="${use_fake_hardware and not sim_gazebo and not sim_ignition and not sim_isaac}">
    <plugin>mock_components/GenericSystem</plugin>
    <!-- ... -->
</xacro:if>
```

---

## 🔧 ACTUAL FIX APPLIED

**File Modified:** `src/mobile_manipulator_pltf_description/description/robotiq_gripper.ros2_control.xacro`

**Change:** Line 41-47

**Before (BROKEN):**
```xml
<xacro:if value="${use_fake_hardware}">
    <plugin>mock_components/GenericSystem</plugin>
    <param name="mock_sensor_commands">${mock_sensor_commands}</param>
    <param name="state_following_offset">0.0</param>
</xacro:if>
<xacro:unless value="${use_fake_hardware or sim_ignition or sim_isaac}">
    <plugin>robotiq_driver/RobotiqGripperHardwareInterface</plugin>
```

**After (FIXED):**
```xml
<xacro:if value="${use_fake_hardware and not sim_gazebo and not sim_ignition and not sim_isaac}">
    <plugin>mock_components/GenericSystem</plugin>
    <param name="mock_sensor_commands">${mock_sensor_commands}</param>
    <param name="state_following_offset">0.0</param>
</xacro:if>
<xacro:unless value="${use_fake_hardware or sim_ignition or sim_isaac or sim_gazebo}">
    <plugin>robotiq_driver/RobotiqGripperHardwareInterface</plugin>
```

---

## ✅ VERIFICATION POST-FIX

After rebuild, URDF now shows only ONE plugin per system:
```
✅ gazebo_ros2_control/GazeboSystem     (for Gazebo)
❌ mock_components/GenericSystem        (REMOVED when sim_gazebo=true)
```

---

## 📋 Implementation Checklist

- [x] **Identified root cause:** Dual plugin conflict in ros2_control
- [x] **Located problem file:** robotiq_gripper.ros2_control.xacro
- [x] **Applied xacro logic fix:** Added `and not sim_gazebo` conditions
- [x] **Verified fix:** URDF inspection confirms single plugin loading
- [x] **Rebuilt package:** `colcon build --packages-select mobile_manipulator_pltf_description`
- [x] **Tested compilation:** No errors
- [ ] **Final user test:** Launch Gazebo and verify gripper visibility

---

## 🚀 Next Step for User

```bash
# Rebuild the entire stack
cd /home/ros/aoc_hunter_ws
colcon build

# Launch the simulation
source install/setup.bash
ros2 launch mobile_manipulator_pltf_description sim_bringup.launch.py use_gazebo:=true use_rviz:=false
```

**Expected Result:** Robotiq gripper should now be **visible and controllable** in Gazebo Classic.


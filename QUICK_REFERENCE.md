# QUICK REFERENCE: Robotiq Gripper Invisibility Fix

## TL;DR - What Was Wrong

**Bug:** `use_fake_hardware="true"` in gripper macro instantiation  
**Location:** `src/mobile_manipulator_pltf_description/description/mobile_manipulator_pltf_macro.xacro` (line 99)  
**Effect:** Conflicting ros2_control plugins (Gazebo + Mock) caused invisibility  
**Status:** ✅ **FIXED**

---

## The One-Line Fix

```bash
sed -i 's/use_fake_hardware="true"/use_fake_hardware="false"/' \
  src/mobile_manipulator_pltf_description/description/mobile_manipulator_pltf_macro.xacro
```

---

## How to Verify the Fix

```bash
# Rebuild
colcon build --packages-select mobile_manipulator_pltf_description

# Check plugins are correct (should show ONLY gazebo_ros2_control, NOT mock_components)
source install/setup.bash
xacro src/mobile_manipulator_pltf_description/description/mobile_manipulator_pltf.urdf.xacro \
  is_sim:=true use_arm:=true use_base:=false | grep -E "gazebo_ros2_control|mock_components"

# Expected output:
#   <plugin>gazebo_ros2_control/GazeboSystem</plugin>
#   <plugin>gazebo_ros2_control/GazeboSystem</plugin>
```

---

## Test the Simulation

```bash
ros2 launch mobile_manipulator_pltf_description sim_bringup.launch.py use_gazebo:=true use_rviz:=true
```

**Expected Result:**
- ✅ UR5 arm visible and controlled
- ✅ Robotiq gripper **NOW VISIBLE** in Gazebo
- ✅ Gripper can open/close via controllers

---

## Why It Works

| Parameter | Value | Effect |
|-----------|-------|--------|
| `use_fake_hardware` | `false` | ✓ Selects gazebo_ros2_control plugin |
| `sim_gazebo` | `true` | ✓ Enables Gazebo physics simulation |
| Result | — | ✓ Single plugin, clean configuration |

---

## What Was NOT Wrong

These diagnostics all passed:

✅ Mesh files installed  
✅ Mesh paths resolvable  
✅ Gripper macro instantiated  
✅ Kinematics correct  
✅ Inertial properties present  
✅ Gazebo plugins configured  
✅ ROS 2 compliance

---

## Full Documentation

- See `COMPLETE_TECHNICAL_REVIEW.md` for detailed technical analysis
- See `ROOT_CAUSE_AND_FIX.md` for complete root cause explanation


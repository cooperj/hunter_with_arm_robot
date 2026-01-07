# Quick Fix: Robotiq Gripper Plugin Conflict

## The Problem

When `sim_gazebo="true"` and `use_fake_hardware="true"` are both set, the gripper ros2_control section loads **TWO conflicting plugins**:
- `gazebo_ros2_control/GazeboSystem` 
- `mock_components/GenericSystem`

This prevents Gazebo from rendering and controlling the gripper.

---

## The Solution

**File:** `src/mobile_manipulator_pltf_description/description/robotiq_gripper.ros2_control.xacro`

**Line 41:** Change the condition from:
```xml
<xacro:if value="${use_fake_hardware}">
```

To:
```xml
<xacro:if value="${use_fake_hardware and not sim_gazebo and not sim_ignition and not sim_isaac}">
```

**Line 47:** Change the unless from:
```xml
<xacro:unless value="${use_fake_hardware or sim_ignition or sim_isaac}">
```

To:
```xml
<xacro:unless value="${use_fake_hardware or sim_ignition or sim_isaac or sim_gazebo}">
```

---

## Rebuild & Test

```bash
cd /home/ros/aoc_hunter_ws
colcon build --packages-select mobile_manipulator_pltf_description
source install/setup.bash
ros2 launch mobile_manipulator_pltf_description sim_bringup.launch.py use_gazebo:=true
```

Gripper should now be visible in Gazebo Classic ✅


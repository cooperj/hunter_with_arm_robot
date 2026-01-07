# Detailed Change Log

## File 1: robotiq_gripper.ros2_control.xacro

### Change: Fix Plugin Conflict

**Location:** Hardware plugin selection section  
**Lines affected:** 28-40

### Before:
```xml
<xacro:if value="${use_fake_hardware}">
    <plugin>mock_components/GenericSystem</plugin>
    <param name="mock_sensor_commands">${mock_sensor_commands}</param>
    <param name="state_following_offset">0.0</param>
</xacro:if>
<xacro:unless value="${use_fake_hardware or sim_ignition or sim_isaac}">
    <plugin>robotiq_driver/RobotiqGripperHardwareInterface</plugin>
    ...
</xacro:unless>
```

### After:
```xml
<xacro:if value="${use_fake_hardware and not sim_gazebo}">
    <plugin>mock_components/GenericSystem</plugin>
    <param name="mock_sensor_commands">${mock_sensor_commands}</param>
    <param name="state_following_offset">0.0</param>
</xacro:if>
<xacro:unless value="${use_fake_hardware or sim_gazebo or sim_ignition or sim_isaac}">
    <plugin>robotiq_driver/RobotiqGripperHardwareInterface</plugin>
    ...
</xacro:unless>
```

### Reason:
- **Before:** When `use_fake_hardware:=true` and `sim_gazebo:=true` (both true in simulation), BOTH plugins were loaded
- This caused priority issues where mock hardware took precedence
- **After:** MockSystem only loads if `use_fake_hardware` AND NOT Gazebo simulation
- This ensures pure Gazebo simulation path is used with gazebo_ros2_control/GazeboSystem

---

## File 2: robotiq_gripper_macro.xacro

### Change: Add Gazebo Collision Configuration for Mimic Joints

**Location:** End of macro, before `</xacro:macro>`  
**Lines affected:** After joint definitions (around line 308+)

### Added:
```xml
<!-- Gazebo configuration for mimic joint simulation in Gazebo Classic -->
<!-- We add selfCollide to help Gazebo handle the coupling -->
<xacro:if value="${sim_gazebo}">
    <gazebo reference="${prefix}robotiq_85_right_knuckle_link">
        <selfCollide>true</selfCollide>
    </gazebo>
    <gazebo reference="${prefix}robotiq_85_left_inner_knuckle_link">
        <selfCollide>true</selfCollide>
    </gazebo>
    <gazebo reference="${prefix}robotiq_85_right_inner_knuckle_link">
        <selfCollide>true</selfCollide>
    </gazebo>
    <gazebo reference="${prefix}robotiq_85_left_finger_tip_link">
        <selfCollide>true</selfCollide>
    </gazebo>
    <gazebo reference="${prefix}robotiq_85_right_finger_tip_link">
        <selfCollide>true</selfCollide>
    </gazebo>
</xacro:if>
```

### Why:
- **selfCollide:** Enables Gazebo physics to compute collisions within the gripper structure
- **Conditional on sim_gazebo:** Only applies when simulating (not in hardware mode)
- **Targets only mimic joint children:** These are the links whose motion is derived from primary joints
- **Helps Gazebo physics:** Even though gazebo_ros2_control applies position commands, enabling self-collision helps the physics engine maintain proper link configurations

---

## Configuration Already Present (No Changes Needed)

### robotiq_gripper.ros2_control.xacro - Joint Interface Configuration

These were already correctly configured:

```xml
<joint name="${prefix}robotiq_85_left_knuckle_joint">
    <command_interface name="position" />
    <state_interface name="position">
        <param name="initial_value">${gripper_closed_position}</param>
    </state_interface>
    <state_interface name="velocity"/>
</joint>

<!-- When simulating we need to include the rest of the gripper joints -->
<xacro:if value="${sim_gazebo or use_fake_hardware or sim_isaac or sim_ignition}">
    <joint name="${prefix}robotiq_85_right_knuckle_joint">
        <param name="mimic">${prefix}robotiq_85_left_knuckle_joint</param>
        <param name="multiplier">-1</param>
        <xacro:unless value="${sim_ignition}">
            <command_interface name="position"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </xacro:unless>
    </joint>
    <!-- ... other mimic joints with proper multipliers ... -->
</xacro:if>
```

---

## Summary of Changes

| File | Change Type | Purpose | Impact |
|------|------------|---------|--------|
| robotiq_gripper.ros2_control.xacro | Logic Fix | Prevent plugin conflict | Ensures gazebo_ros2_control is active in simulation |
| robotiq_gripper_macro.xacro | Addition | Add selfCollide config | Enables Gazebo physics to handle mimic joint coupling |

**Total lines changed:** ~15 new lines, 2 conditionals modified  
**Build time impact:** None (pure Xacro conditionals)  
**Runtime impact:** Better Gazebo simulation, no RViz/hardware changes

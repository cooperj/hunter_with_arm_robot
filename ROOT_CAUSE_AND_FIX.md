# ROOT CAUSE IDENTIFIED & FIXES

## THE BUG: Plugin Conflict in ros2_control

### Issue Found in mobile_manipulator_pltf_macro.xacro (Line 99)

```xml
<xacro:robotiq_gripper
  name="RobotiqGripperHardwareInterface"
  prefix="${prefix}"
  parent="tool0"
  use_fake_hardware="true"           <!-- ❌ BUG: Should be FALSE for Gazebo sim
  com_port="/dev/ttyUSB0"
  sim_gazebo="true"
>
```

### Why This is a Problem

The robotiq_gripper.ros2_control.xacro file has conditional logic:

```xml
<xacro:if value="${sim_gazebo}">
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
</xacro:if>

<xacro:if value="${use_fake_hardware}">
    <plugin>mock_components/GenericSystem</plugin>
    <param name="mock_sensor_commands">${mock_sensor_commands}</param>
</xacro:if>
```

**When BOTH are true**, xacro instantiates TWO plugins:
1. `gazebo_ros2_control/GazeboSystem` (Gazebo physics simulation)
2. `mock_components/GenericSystem` (Mock/fake driver without real physics)

This creates a **conflict**: ros2_control tries to load both simulators, causing one to fail or both to malfunction.

---

## SOLUTION: Set use_fake_hardware="false"

### File to Fix:
**[mobile_manipulator_pltf_macro.xacro](src/mobile_manipulator_pltf_description/description/mobile_manipulator_pltf_macro.xacro#L95-L104)**

### Change FROM (Line 95-104):
```xml
      <xacro:robotiq_gripper
        name="RobotiqGripperHardwareInterface"
        prefix="${prefix}"
        parent="tool0"
        use_fake_hardware="true"
        com_port="/dev/ttyUSB0"
        sim_gazebo="true"
      >
        <origin xyz="0 0 0" rpy="0 0 0"/>
      </xacro:robotiq_gripper>
```

### Change TO:
```xml
      <xacro:robotiq_gripper
        name="RobotiqGripperHardwareInterface"
        prefix="${prefix}"
        parent="tool0"
        use_fake_hardware="false"
        com_port="/dev/ttyUSB0"
        sim_gazebo="true"
      >
        <origin xyz="0 0 0" rpy="0 0 0"/>
      </xacro:robotiq_gripper>
```

---

## How to Apply the Fix

### Option A: Using the Terminal

```bash
cd /home/ros/aoc_hunter_ws

# Open the file and make the change:
sed -i 's/use_fake_hardware="true"/use_fake_hardware="false"/' \
  src/mobile_manipulator_pltf_description/description/mobile_manipulator_pltf_macro.xacro

# Verify the change:
grep -A 5 "<xacro:robotiq_gripper" \
  src/mobile_manipulator_pltf_description/description/mobile_manipulator_pltf_macro.xacro

# Rebuild:
colcon build --packages-select mobile_manipulator_pltf_description --symlink-install

# Source the new setup:
source install/setup.bash
```

### Option B: Manual Edit

1. Open [mobile_manipulator_pltf_macro.xacro](src/mobile_manipulator_pltf_description/description/mobile_manipulator_pltf_macro.xacro)
2. Go to line 99
3. Change `use_fake_hardware="true"` to `use_fake_hardware="false"`
4. Save the file
5. Run `colcon build --packages-select mobile_manipulator_pltf_description`
6. Restart the simulation

---

## Verification: The Fixed ros2_control Logic

After the fix, the conditional logic in robotiq_gripper.ros2_control.xacro will work correctly:

```xml
<!-- BEFORE FIX: Both conditions true → plugin conflict ❌ -->
use_fake_hardware: true   AND   sim_gazebo: true
→ Loads BOTH mock_components AND gazebo_ros2_control

<!-- AFTER FIX: Only sim_gazebo true → correct plugin ✓ -->
use_fake_hardware: false  AND   sim_gazebo: true
→ Loads ONLY gazebo_ros2_control/GazeboSystem
→ Gazebo has full control of gripper physics via Gazebo plugins
```

---

## Expected Result After Fix

✓ Gripper will be **visible in Gazebo**
✓ Gripper will be **controlled by Gazebo physics simulation**
✓ Mimic joints will work via `libgazebo_mimic_joint_plugin.so`
✓ GripperActionController can command the gripper
✓ Fingers will open/close smoothly

---

## Complete Diagnostic Results Summary

| Check | Result | Details |
|-------|--------|---------|
| Mesh files installed | ✅ **PASS** | Found 8+ DAE files in install/robotiq_description/share/robotiq_description/meshes/visual/2f_85/ |
| Mesh paths in URDF | ✅ **PASS** | Found package://robotiq_description/meshes paths in compiled URDF |
| Gripper links in URDF | ✅ **PASS** | Found 66 robotiq_85 references (all 8 links, 7 joints, plugins) |
| Gazebo mimic plugins | ✅ **PASS** | Found 5 libgazebo_mimic_joint_plugin instances |
| gazebo_ros2_control plugin | ⚠️ **MIXED** | Found GazeboSystem selected BUT also mock_components loaded (plugin conflict!) |
| Macro instantiation | ✅ **PASS** | Gripper macro properly called in mobile_manipulator_pltf_macro.xacro |
| Tool0 connection | ✅ **PASS** | Fixed joint connecting tool0 to robotiq_85_base_link |
| ROS 2 compliance | ✅ **PASS** | Uses spawn_entity.py, no ROS 1 commands |
| **ROOT CAUSE** | ❌ **BUG FOUND** | `use_fake_hardware="true"` + `sim_gazebo="true"` causes plugin conflict |

---

## After Applying the Fix, Run This to Verify

```bash
# Generate URDF and check for plugin conflict:
source install/setup.bash
xacro src/mobile_manipulator_pltf_description/description/mobile_manipulator_pltf.urdf.xacro \
  is_sim:=true use_arm:=true use_base:=false | \
  grep -A 1 "<plugin>" | head -20

# Should show:
# <plugin>gazebo_ros2_control/GazeboSystem</plugin>
# But NOT mock_components/GenericSystem
```


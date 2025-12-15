**Tested commit:** `v0.1-virtual-wall`

Touch Virtual Wall Force Feedback

Reproducible Haptic Virtual Wall for Touch / Phantom Devices (ROS 2)

1. Project Overview

This repository provides a ROS 2–based force feedback implementation for Touch / Phantom haptic devices, focusing on a stable, non-rebounding virtual wall interaction.

The implementation is designed for experimental reproducibility, and has been validated through real-device interaction, not only simulation.

Key design goals

Distance-based force field before contact

Hard virtual wall with finite thickness

Stable contact without oscillation or rebound

Explicit contact onset cue (click feedback)

Hardware-aware force design (respecting device limits)

2. System Architecture

The virtual wall interaction is structured into three force zones along the wall normal direction:

Zone 1 – Distance-based Force Field

Active when the end-effector approaches the wall but is still far away

Generates a weak repulsive force proportional to distance

Purpose: provide early haptic awareness of the wall

Zone 2 – Wall Surface Layer

Active within a finite wall thickness

Combines:

Distance-based repulsion

Strong wall stiffness

Velocity-dependent damping

Purpose: create the sensation of a rigid but stable wall

Zone 3 – Contact / Penetration Region

Active after reaching the wall surface

Force no longer increases with penetration depth

Strong damping dominates to stabilize contact

Optional impulse (“click”) at contact onset

Purpose: prevent rebound and ensure steady contact

This layered design avoids common issues such as:

Sudden force discontinuities

Hard rebound

Excessive reliance on large force magnitudes

3. Software Environment
Tested Environment

OS: Ubuntu 22.04

ROS 2: Humble

Python: 3.10

Haptic device: Touch / Phantom series

⚠️ Note: The device driver internally limits continuous output force.
Large force values (>10 N) do not lead to proportionally stronger sensations.

4. Repository Structure
touch_ros2_ws/
├── src/
│   └── virtual_environment/
│       ├── package.xml
│       ├── setup.py
│       ├── resource/
│       └── virtual_environment/
│           ├── __init__.py
│           └── force_feedback_node.py
├── build/        (ignored)
├── install/      (ignored)
├── log/          (ignored)
└── README.md


Only the src/ directory is required for reproduction.

5. Build Instructions

From the workspace root:

cd ~/touch_ros2_ws
colcon build --packages-select virtual_environment
source install/setup.bash

6. Running the Experiment

Start the force feedback node:

ros2 run virtual_environment force_feedback_node


Ensure that:

The Touch / Phantom device driver is running

The device is properly calibrated before use

7. Visualization (Optional)

The force vector can be visualized in RViz:

Topic: /virtual_env/force_vector

Marker type: ARROW

Fixed Frame: world

The arrow:

Direction: wall normal

Length: proportional to force magnitude

Color: green (can be changed for debugging)

8. Key Parameters

The following parameters strongly affect the haptic sensation:

Parameter	Description	Typical Range
K_field	Distance-field repulsion (N)	0.5 – 2.0
field_range	Force field activation distance (m)	0.05 – 0.20
K_wall	Wall stiffness (N/m)	800 – 1500
wall_thickness	Virtual wall thickness (m)	0.01 – 0.02
B_min / B_max	Damping coefficients	20 – 80
click_amp	Contact impulse amplitude (N)	~1.0
click_tau	Impulse decay time (s)	~0.05

Increasing force magnitude alone does not necessarily improve perceived wall hardness due to hardware saturation.

9. Experimental Notes

The perceived wall hardness is dominated by:

Force gradient

Damping behavior

Contact onset cue

Simply increasing stiffness or force limits leads to diminishing returns

Stable contact requires structural force design, not extreme force values

10. Reproducibility Statement

All results reported using this repository:

Were obtained using a real Touch / Phantom device

Are reproducible using the parameters listed above

Do not rely on undocumented hardware features

11. License

Specify your license here (e.g., MIT, BSD, GPL).

12. Citation (Optional)

If you use this code in academic work, please cite:

@misc{touch_virtual_wall,
  title   = {Touch Virtual Wall Force Feedback},
  author  = {Your Name},
  year    = {2025},
  note    = {ROS 2 implementation for haptic virtual wall interaction}
}

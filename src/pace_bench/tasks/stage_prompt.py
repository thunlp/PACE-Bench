"""Shared prompt helpers for benchmark curriculum stages."""

from __future__ import annotations

import re
from collections.abc import Iterable


_NUMERIC_LITERAL = re.compile(r"(?<![A-Za-z])[-+]?\d+(?:\.\d+)?")

_TASK_VARIABLES: dict[str, tuple[str, ...]] = {
    "S_01": (
        "Structural Joint Force Limit",
        "Structural Joint Torque Limit",
        "Cliff Anchor Force Limit",
        "Cliff Anchor Torque Limit",
        "Structure Mass Limit",
        "Terrain Gap Width",
        "Gravitational Acceleration",
        "Atmospheric Wind Force",
    ),
    "S_02": (
        "Joint Force Limit",
        "Joint Torque Limit",
        "Gravitational Acceleration",
        "Earthquake Displacement Amplitude",
        "Earthquake Frequency",
        "Earthquake Amplitude Evolution",
        "Wind Force",
        "Wind Application Altitude",
        "Wind Shear",
        "Wind Oscillation Frequency",
    ),
    "S_03": (
        "Regional Anchor Response",
        "Payload Release Geometry",
        "Wall Attachment Availability",
        "Gravitational Acceleration",
        "Payload Mass",
        "Payload Delivery Mode",
        "Wall Anchor Force Limit",
        "Wall Anchor Torque Limit",
        "Internal Joint Force Limit",
        "Internal Joint Torque Limit",
        "Structure Mass Limit",
        "Tip Height Requirement",
        "Obstacle Presence",
        "Obstacle Geometry",
        "Localized Force Field",
        "Target Reach Requirement",
        "Atmospheric Wind Force",
    ),
    "S_04": (
        "Pivot Connection Mode",
        "Pivot Joint Torque Limit",
        "Pivot Friction",
        "Balance Angle Tolerance",
        "Balance Duration Requirement",
        "Payload Mass",
        "Gravitational Acceleration",
        "Obstacle Geometry",
        "Angular Damping",
        "Atmospheric Wind Force",
    ),
    "S_05": (
        "Joint Force Limit",
        "Joint Torque Limit",
        "Atmospheric Wind Force",
        "Structure Mass Limit",
        "Meteor Density",
        "Meteor Restitution",
        "Meteor Velocity Distribution",
        "Structure Restitution",
        "Lateral Containment Walls",
        "Gravitational Acceleration",
        "Protected Core Position",
        "Protected Core Force Limit",
        "Randomized Meteor Layout",
    ),
    "S_06": (
        "Horizontal Reach Requirement",
        "Block Spawn Zone",
        "Gravitational Acceleration",
        "Table Friction",
        "Block Friction",
        "Atmospheric Wind Force",
        "Total Mass Limit",
        "Table Oscillation",
        "Table Length",
    ),
    "K_01": (
        "Ground Friction",
        "Joint Angular Limits",
        "Gravitational Acceleration",
        "Body Friction Limit",
        "Linear Damping",
        "Angular Damping",
        "Structure Mass Limit",
    ),
    "K_02": (
        "Build Zone Height",
        "Joint Force Limit",
        "Joint Torque Limit",
        "Gravitational Acceleration",
        "Wall Adhesion Zones",
        "Structure Mass Bounds",
        "Atmospheric Wind Force",
        "Vortex Force Field",
        "Angular Damping",
    ),
    "K_03": (
        "Target Object Geometry",
        "Target Object Friction",
        "Target Object Mass",
        "Gravitational Acceleration",
        "Linear Damping",
        "Angular Damping",
    ),
    "K_04": (
        "Target Distance",
        "Structure Mass Limit",
        "Payload Geometry",
        "Payload Mass",
        "Payload Friction",
        "Payload Center Of Mass",
        "Ground Friction",
        "Gravitational Acceleration",
    ),
    "K_05": (
        "Target Height",
        "Ceiling Clearance",
        "Payload Geometry",
        "Payload Mass",
        "Payload Center Of Mass",
        "Structure Mass Limit",
        "Joint Force Limit",
        "Surface Friction",
        "Gravitational Acceleration",
        "Atmospheric Wind Force",
    ),
    "K_06": (
        "Particle Count",
        "Particle Distribution",
        "Particle Friction",
        "Particle Mass",
        "Particle Radius",
        "Structure Mass Limit",
        "Motor Torque Limit",
        "Glass Friction",
        "Gravitational Acceleration",
    ),
    "D_01": (
        "Target Zone Geometry",
        "Gravitational Acceleration",
        "Linear Damping",
        "Angular Damping",
    ),
    "D_02": (
        "Barrier Slot Geometry",
        "Gravitational Acceleration",
        "Atmospheric Wind Force",
        "Linear Damping",
    ),
    "D_03": (
        "Impulse Zone Magnitude",
        "Ambient Damping",
        "Deceleration Zone Damping",
        "Mud Zone Damping",
        "Gravitational Acceleration",
    ),
    "D_04": (
        "Actuator Dead Zone",
        "Dead Zone Speed Threshold",
        "Quadratic Damping",
        "Directional Actuator Fault",
        "Atmospheric Wind Strength",
        "Atmospheric Wind Period",
    ),
    "D_05": (
        "Shell Break Force",
        "Slot Bar Oscillation",
        "Angular Damping",
        "Gravitational Acceleration",
    ),
    "D_06": (
        "Legal Build Footprint",
        "Joint Force Limit",
        "Joint Fatigue Threshold",
        "Projectile Density",
        "Projectile Launch Velocity",
        "Projectile Launch Schedule",
        "Projectile Damping",
        "Projectile Restitution",
        "Gravity Modulation",
        "Structural Wind Coupling",
    ),
    "F_01": (
        "Weld Force Limit",
        "Weld Failure Persistence",
        "Gravitational Acceleration",
        "Fluid Particle Restitution",
        "Fluid Particle Friction",
        "Downstream Boundary Oscillation",
        "Debris Impact Velocity",
        "Seismic Horizontal Impulse",
        "Vertical Fluid Surge Impulse",
        "Reverse Fluid Surge Impulse",
        "Forward Fluid Surge Impulse",
        "Structure Mass Limit",
        "Leakage Rate Limit",
    ),
    "F_02": (
        "Water Current Intensity",
        "Electromagnetic Dead Zone",
        "Corrosive Altitude",
        "Whirlpool Force Field",
        "Joint Force Limit",
        "Thrust Cooldown Duration",
    ),
    "F_03": (
        "Particle Count",
        "Particle Friction",
        "Ambient Damping",
        "Transfer Requirement",
        "Pit Drift Force",
        "Scoop Capacity",
        "Structure Mass Limit",
        "Gravitational Acceleration",
        "Time Limit",
    ),
    "F_04": (
        "Baffle Geometry",
        "Sweeper Kinematics",
        "Beam Count Limit",
        "Structure Mass Limit",
        "Lateral Wind Force",
        "Gust Force",
        "Gravitational Acceleration",
        "Gravity Oscillation",
        "Ambient Damping",
        "Particle Mixture",
        "Wave Release Schedule",
        "Feed Zone Geometry",
        "Beam Friction",
        "Purity Requirement",
    ),
    "F_05": (
        "Boat Placement",
        "Build Zone Height",
        "Cargo Geometry",
        "Cargo Contact Behavior",
        "Cargo Loss Window",
        "Cargo Waterline",
        "Current Force",
        "Deck Friction",
        "Wave Forcing",
        "Wind Forcing",
        "Gust Forcing",
        "Lateral Impulse",
        "Rogue Wave Forcing",
        "Hull Roll Impulse",
        "Restoring Force",
        "Obstacle Geometry",
        "Joint Force Limit",
        "Structure Mass Limit",
        "Gravitational Acceleration",
        "Linear Damping",
        "Angular Damping",
    ),
    "F_06": (
        "Fluid Particle Count",
        "Fluid Particle Radius",
        "Fluid Particle Density",
        "Fluid Viscosity",
        "Gravitational Acceleration",
        "Gravity Well Force",
        "Force Budget",
        "Episode Duration",
        "Target Zone Height",
        "Delivery Ratio Requirement",
    ),
    "C_01": (
        "Sensor Delay",
        "Gravitational Acceleration",
        "Cart Mass",
        "Pole Mass",
        "Pole Length",
        "Track Center Position",
        "Safe Zone Width",
        "Episode Duration",
        "Actuator Force Limit",
        "Pole Initial Angle",
        "Rail Height",
    ),
    "C_02": (
        "Touchdown Vertical Speed Limit",
        "Landing Angle Limit",
        "Landing Zone Width",
        "Actuation Latency",
        "Flight Corridor Geometry",
        "Engine Thrust Limit",
        "Gravitational Acceleration",
        "Fuel Impulse Budget",
        "Landing Fuel Requirement",
        "Atmospheric Wind Force",
        "Atmospheric Gust Force",
    ),
    "C_03": (
        "Gravitational Acceleration",
        "Linear Damping",
        "Angular Damping",
        "Impulse Budget",
        "Track Distance Requirement",
        "Rendezvous Distance Limit",
        "Rendezvous Relative Speed Limit",
        "Rendezvous Heading Limit",
        "Rendezvous Time Windows",
        "Obstacle Geometry",
        "Spawn Position",
        "Ground Friction",
        "Low Friction Zones",
        "Thrust Limit",
        "Cooldown Constraints",
        "Sensor Blind Zones",
        "Target Motion",
    ),
    "C_04": (
        "Control Latency",
        "Whisker Blind Zones",
        "Fluid Drag",
        "Turbulence",
        "Control Reversal Zone",
        "Magnetic Floor Force",
        "Structural Impulse Limit",
        "Obstacle Geometry",
    ),
    "C_05": (
        "Regional Speed Limit",
        "Repulsive Field Force",
        "Input Sensitivity Threshold",
        "Ground Friction",
        "Ramp Friction",
        "Barrier Actuation Latency",
        "Cooldown Duration",
        "Activation Duration",
        "Temporal Sequencing Windows",
        "State Persistence Requirements",
    ),
    "C_06": (
        "Measurement Latency",
        "Motor Torque Limit",
        "Sustained Load Onset",
        "Torque Dead Zone",
        "Rotational Drag",
        "Cogging Torque",
        "Static Friction",
    ),
    "E_01": (
        "Arena Height",
        "Build Zone Height",
        "Gravitational Acceleration",
        "Linear Damping",
        "Angular Damping",
        "Joint Force Limit",
        "Surface Friction",
        "Beam Count Limit",
        "Structure Mass Limit",
        "Beam Density Scale",
    ),
    "E_02": (
        "Linear Damping",
        "Momentum Drain",
        "Thermal Limit",
        "Ambient Body Force",
        "Time Varying Wind Force",
        "Backward Slip Force",
    ),
    "E_03": (
        "Ground Friction",
        "Sled Friction",
        "Gravitational Acceleration",
        "Linear Damping",
        "Momentum Retention",
        "Thrust Delivery Scale",
        "Speed Penalty Threshold",
        "Speed Penalty Scale",
    ),
    "E_04": (
        "Mass Variation",
        "Joint Force Limit",
        "Joint Torque Limit",
        "Fatigue Time Constant",
        "Atmospheric Wind Pressure",
        "Gravitational Acceleration",
    ),
    "E_05": (
        "Magnet Layout",
        "Magnetic Field Strength",
        "Gravitational Acceleration",
        "Linear Damping",
        "Angular Damping",
        "Thrust Limit",
    ),
    "E_06": (
        "Gravitational Acceleration",
        "Linear Damping",
        "Angular Damping",
        "Distributed Disturbance Strength",
        "Coherent Pulse Timing",
        "Coherent Pulse Force",
        "Joint Force Limit",
        "Joint Torque Limit",
        "Damage Thresholds",
        "Damage Limit",
        "Beam Spin Limit",
        "Structure Mass Limit",
        "Anchor Zone",
        "Fatigue Activation",
        "Cascade Damage",
        "Storm Timing",
        "Storm Intensity",
    ),
}


def build_uniform_suffix(variable_names: Iterable[str]) -> str:
    """Build the canonical value-free mutation warning used by every task."""

    names: list[str] = []
    seen: set[str] = set()
    for raw_name in variable_names:
        name = str(raw_name).strip()
        if not name:
            raise ValueError("UNIFORM_SUFFIX variable names must be non-empty")
        if "\n" in name:
            raise ValueError(
                f"UNIFORM_SUFFIX variable name must fit on one line: {name!r}"
            )
        if _NUMERIC_LITERAL.search(name):
            raise ValueError(
                f"UNIFORM_SUFFIX variable name must not contain a value: {name!r}"
            )
        if name not in seen:
            names.append(name)
            seen.add(name)
    if not names:
        raise ValueError("UNIFORM_SUFFIX requires at least one variable name")

    bullets = "\n".join(f"- {name}" for name in names)
    return (
        "\n\n## Possible Environment Variations\n\n"
        "Sensors indicate that this region may have non-standard physical "
        "properties. Not every property listed below necessarily differs in "
        "this stage:\n\n"
        f"{bullets}\n\n"
        "This list intentionally provides no values, change directions, "
        "severity, or stage-to-property mapping. Current numeric values are "
        "stated in the task description only for hard constraints and directly "
        "visible geometry or state.\n\n"
        "Use interaction and diagnostic feedback to determine which latent "
        "properties matter in this environment."
    )


def uniform_suffix_for_task(task_name: str) -> str:
    """Return the canonical suffix for a benchmark task identifier."""

    try:
        variable_names = _TASK_VARIABLES[task_name]
    except KeyError as exc:
        raise ValueError(f"No UNIFORM_SUFFIX variable inventory for {task_name}") from exc
    return build_uniform_suffix(variable_names)


def uniform_suffix_variables(task_name: str) -> tuple[str, ...]:
    """Return the immutable variable-name inventory used by a task suffix."""

    try:
        return _TASK_VARIABLES[task_name]
    except KeyError as exc:
        raise ValueError(f"No UNIFORM_SUFFIX variable inventory for {task_name}") from exc

"""Reference agent for the single-environment tutorial demo."""


def build_agent(sandbox):
    """Build a small two-wheel vehicle using only the public sandbox API."""
    ground_top = 1.0
    wheel_radius = 1.5
    wheel_y = ground_top + wheel_radius

    chassis = sandbox.add_beam(
        x=5.0,
        y=wheel_y + 0.2,
        width=5.0,
        height=0.4,
        density=3.0,
    )
    front_wheel = sandbox.add_wheel(
        x=3.2,
        y=wheel_y,
        radius=wheel_radius,
        friction=4.0,
        density=1.0,
    )
    rear_wheel = sandbox.add_wheel(
        x=6.8,
        y=wheel_y,
        radius=wheel_radius,
        friction=4.0,
        density=1.0,
    )
    sandbox.connect(
        chassis,
        front_wheel,
        anchor_x=3.2,
        anchor_y=wheel_y,
        motor_speed=-6.0,
        max_torque=1800.0,
    )
    sandbox.connect(
        chassis,
        rear_wheel,
        anchor_x=6.8,
        anchor_y=wheel_y,
        motor_speed=-6.0,
        max_torque=1800.0,
    )

    is_valid, errors = sandbox.validate_design(chassis)
    if not is_valid:
        raise ValueError(f"Design validation failed: {errors}")
    return chassis


def agent_action(sandbox, agent_body, step_count):
    """Keep the motors configured by ``build_agent`` running."""

from typing import Dict, Any, List

def format_task_metrics(metrics: Dict[str, Any]) -> List[str]:
    metric_parts = []
    if 'distance_traveled' in metrics:
        metric_parts.append(f"**Distance traveled**: {metrics['distance_traveled']:.2f}m")
        metric_parts.append(f"**Current position**: x={metrics.get('current_x', 0):.2f}m, y={metrics.get('current_y', 0):.2f}m")
        if 'target_x' in metrics:
            metric_parts.append(f"**Target position**: x={metrics['target_x']:.2f}m")
        if 'progress' in metrics:
            metric_parts.append(f"**Progress**: {metrics['progress']:.1f}%")
        if 'max_distance' in metrics:
            metric_parts.append(f"**Maximum distance reached**: {metrics['max_distance']:.2f}m")
        if 'step_count' in metrics:
            metric_parts.append(f"**Simulation steps**: {metrics['step_count']}")
        metric_parts.append("\n**Physical State Information**:")
        if 'current_x' in metrics and 'current_y' in metrics:
            metric_parts.append(f"- Agent position: ({metrics['current_x']:.3f}, {metrics['current_y']:.3f})")
        if 'velocity' in metrics:
            metric_parts.append(f"- Agent velocity: {metrics['velocity']:.3f} m/s")
        if 'velocity_x' in metrics and 'velocity_y' in metrics:
            metric_parts.append(f"- Agent velocity components: vx={metrics['velocity_x']:.3f} m/s, vy={metrics['velocity_y']:.3f} m/s")
        if 'angular_velocity' in metrics:
            metric_parts.append(f"- Agent angular velocity: {metrics['angular_velocity']:.3f} rad/s")
        if 'angle' in metrics:
            metric_parts.append(f"- Agent angle: {metrics['angle']:.3f} rad ({metrics['angle'] * 180 / 3.14159:.1f}°)")
    excluded_keys = ['distance_traveled', 'current_x', 'current_y', 'target_x', 'progress',
                    'max_distance', 'step_count', 'success', 'failed', 'failure_reason',
                    'velocity', 'velocity_x', 'velocity_y', 'angular_velocity', 'angle']
    other_metrics = {k: v for k, v in metrics.items() if k not in excluded_keys}
    if other_metrics:
        metric_parts.append("\n**Additional Metrics**:")
        for key, value in other_metrics.items():
            if isinstance(value, (int, float)):
                metric_parts.append(f"- {key}: {value:.3f}" if isinstance(value, float) else f"- {key}: {value}")
            else:
                metric_parts.append(f"- {key}: {value}")
    return metric_parts

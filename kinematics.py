import math
from dataclasses import dataclass


@dataclass(frozen=True)
class SwerveParams:
    velocity: float = 0.0
    alpha_deg: float = 0.0
    yaw_deg: float = 0.0
    omega: float = 2.0
    wheelbase: float = 1.2
    track_width: float = 1.2


@dataclass(frozen=True)
class WheelState:
    key: str
    label: str
    local_x: float
    local_y: float
    world_x: float
    world_y: float
    vx_robot: float
    vy_robot: float
    speed: float
    steering_deg: float
    world_angle_deg: float


@dataclass(frozen=True)
class SwerveResult:
    params: SwerveParams
    vx_linear: float
    vy_linear: float
    wheels: dict[str, WheelState]
    chassis_corners_world: list[tuple[float, float]]


def rotate_2d(x: float, y: float, angle_rad: float) -> tuple[float, float]:
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    return (x * cos_a - y * sin_a, x * sin_a + y * cos_a)


def world_to_screen(
    x: float, y: float, cx: float, cy: float, scale: float
) -> tuple[float, float]:
    return (cx + x * scale, cy - y * scale)


def calculate_swerve(params: SwerveParams) -> SwerveResult:
    alpha_rad = math.radians(params.alpha_deg)
    yaw_rad = math.radians(params.yaw_deg)
    heading_rad = math.radians(90.0 + params.yaw_deg)

    half_wb = params.wheelbase / 2.0
    half_tw = params.track_width / 2.0

    vx_linear = params.velocity * math.cos(alpha_rad - yaw_rad)
    vy_linear = params.velocity * math.sin(alpha_rad - yaw_rad)

    wheel_positions = [
        ("FL", "Wheel A (FL)", half_wb, half_tw),
        ("FR", "Wheel B (FR)", half_wb, -half_tw),
        ("BL", "Wheel C (BL)", -half_wb, half_tw),
        ("BR", "Wheel D (BR)", -half_wb, -half_tw),
    ]

    wheels: dict[str, WheelState] = {}
    for key, label, lx, ly in wheel_positions:
        vx_robot = vx_linear - params.omega * ly
        vy_robot = vy_linear + params.omega * lx
        speed = math.hypot(vx_robot, vy_robot)
        steering_rad = math.atan2(vy_robot, vx_robot)
        steering_deg = math.degrees(steering_rad)

        wx, wy = rotate_2d(lx, ly, heading_rad)
        world_angle_deg = 90.0 + params.yaw_deg + steering_deg

        wheels[key] = WheelState(
            key=key,
            label=label,
            local_x=lx,
            local_y=ly,
            world_x=wx,
            world_y=wy,
            vx_robot=vx_robot,
            vy_robot=vy_robot,
            speed=speed,
            steering_deg=steering_deg,
            world_angle_deg=world_angle_deg,
        )

    local_corners = [
        (half_wb, half_tw),
        (half_wb, -half_tw),
        (-half_wb, -half_tw),
        (-half_wb, half_tw),
    ]
    chassis_corners_world = [
        rotate_2d(cx, cy, heading_rad) for cx, cy in local_corners
    ]

    return SwerveResult(
        params=params,
        vx_linear=vx_linear,
        vy_linear=vy_linear,
        wheels=wheels,
        chassis_corners_world=chassis_corners_world,
    )

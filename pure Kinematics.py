import math

Velocity = 10.0
Alpha = 0.0
Yaw = 45.0
Omega = 0.0

wheelbase = 1.2  # distance between front and rear wheels
track_width = 1.3  # distance between left and right wheels


Front_Left_wheelbase = wheelbase / 2.0
Front_Left_track_width = track_width / 2.0

Front_Right_wheelbase = wheelbase / 2.0
Front_Right_track_width = -(track_width / 2.0)

Back_Left_wheelbase = -(wheelbase / 2.0)
Back_Left_track_width = track_width / 2.0

Back_Right_wheelbase = -(wheelbase / 2.0)
Back_Right_track_width = -(track_width / 2.0)


Velocity_Linear_X = Velocity * math.cos(Alpha - Yaw)
Velocity_Linear_Y = Velocity * math.sin(Alpha - Yaw)

Velocity_Robot_X_Front_Left = Velocity_Linear_X - Omega * Front_Left_track_width
Velocity_Robot_Y_Front_Left = Velocity_Linear_Y + Omega * Front_Left_wheelbase

Velocity_Robot_X_Front_Right = Velocity_Linear_X - Omega * Front_Right_track_width
Velocity_Robot_Y_Front_Right = Velocity_Linear_Y + Omega * Front_Right_wheelbase

Velocity_Robot_X_Back_Left = Velocity_Linear_X - Omega * Back_Left_track_width
Velocity_Robot_Y_Back_Left = Velocity_Linear_Y + Omega * Back_Left_wheelbase

Velocity_Robot_X_Back_Right = Velocity_Linear_X - Omega * Back_Right_track_width
Velocity_Robot_Y_Back_Right = Velocity_Linear_Y + Omega * Back_Right_wheelbase


Velocity_Front_Left = math.sqrt(
    Velocity_Robot_X_Front_Left**2 + Velocity_Robot_Y_Front_Left**2
)
Velocity_Front_Right = math.sqrt(
    Velocity_Robot_X_Front_Right**2 + Velocity_Robot_Y_Front_Right**2
)
Velocity_Back_Left = math.sqrt(
    Velocity_Robot_X_Back_Left**2 + Velocity_Robot_Y_Back_Left**2
)
Velocity_Back_Right = math.sqrt(
    Velocity_Robot_X_Back_Right**2 + Velocity_Robot_Y_Back_Right**2
)

Steering_Angle_Front_Left = math.atan2(
    Velocity_Robot_Y_Front_Left, Velocity_Robot_X_Front_Left
)
Steering_Angle_Front_Right = math.atan2(
    Velocity_Robot_Y_Front_Right, Velocity_Robot_X_Front_Right
)
Steering_Angle_Back_Left = math.atan2(
    Velocity_Robot_Y_Back_Left, Velocity_Robot_X_Back_Left
)
Steering_Angle_Back_Right = math.atan2(
    Velocity_Robot_Y_Back_Right, Velocity_Robot_X_Back_Right
)

print(
    "Wheel A | ",
    "Velocity: ",
    Velocity_Front_Left,
    " | Steering: ",
    math.degrees(Steering_Angle_Front_Left),
)

print(
    "Wheel B | ",
    "Velocity: ",
    Velocity_Front_Right,
    " | Steering: ",
    math.degrees(Steering_Angle_Front_Right),
)

print(
    "Wheel C | ",
    "Velocity: ",
    Velocity_Back_Left,
    " | Steering: ",
    math.degrees(Steering_Angle_Back_Left),
)

print(
    "Wheel D | ",
    "Velocity: ",
    Velocity_Back_Right,
    " | Steering: ",
    math.degrees(Steering_Angle_Back_Right),
)
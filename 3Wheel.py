import math

# =====================================================================
# Motion Parameters
# =====================================================================
Velocity = 10.0
Alpha = 0.0
Yaw = 45.0
Omega = 0.0

# =====================================================================
# 3-Wheel Geometry & Wheel Angle Parameters
# =====================================================================
R = 0.65  # รัศมีจากจุดศูนย์กลางหุ่นถึงแต่ละล้อ (m)

# องศาตำแหน่งการจัดวางล้อรอบจุดศูนย์กลางหุ่น (deg)
Angle_Front = 0.0         # ล้อหน้า (ค่าเริ่มต้น 0°)
Angle_Back_Left = 60.0   # ล้อหลังซ้าย (ค่าเริ่มต้น 120°)
Angle_Back_Right = 300.0  # ล้อหลังขวา (ค่าเริ่มต้น 240° หรือ -120°)

# องศาชดเชยการติดตั้งหัวล้อ Swerve (Steering Offset Angle, deg)
Offset_Angle_Front = 0.0
Offset_Angle_Back_Left = 0.0
Offset_Angle_Back_Right = 0.0

# =====================================================================
# คำนวณพิกัด (x_i, y_i) ของแต่ละล้อจากรัศมี R และองศาล้อ
# =====================================================================
Front_wheelbase = R * math.cos(math.radians(Angle_Front))
Front_track_width = R * math.sin(math.radians(Angle_Front))

Back_Left_wheelbase = R * math.cos(math.radians(Angle_Back_Left))
Back_Left_track_width = R * math.sin(math.radians(Angle_Back_Left))

Back_Right_wheelbase = R * math.cos(math.radians(Angle_Back_Right))
Back_Right_track_width = R * math.sin(math.radians(Angle_Back_Right))

# =====================================================================
# คำนวณเวกเตอร์ความเร็วและมุมเลี้ยวของแต่ละล้อ
# v_ix = v_x - omega * y_i
# v_iy = v_y + omega * x_i
# =====================================================================
Velocity_Linear_X = Velocity * math.cos(math.radians(Alpha - Yaw))
Velocity_Linear_Y = Velocity * math.sin(math.radians(Alpha - Yaw))

Velocity_Robot_X_Front = Velocity_Linear_X - Omega * Front_track_width
Velocity_Robot_Y_Front = Velocity_Linear_Y + Omega * Front_wheelbase

Velocity_Robot_X_Back_Left = Velocity_Linear_X - Omega * Back_Left_track_width
Velocity_Robot_Y_Back_Left = Velocity_Linear_Y + Omega * Back_Left_wheelbase

Velocity_Robot_X_Back_Right = Velocity_Linear_X - Omega * Back_Right_track_width
Velocity_Robot_Y_Back_Right = Velocity_Linear_Y + Omega * Back_Right_wheelbase


Velocity_Front = math.sqrt(
    Velocity_Robot_X_Front**2 + Velocity_Robot_Y_Front**2
)
Velocity_Back_Left = math.sqrt(
    Velocity_Robot_X_Back_Left**2 + Velocity_Robot_Y_Back_Left**2
)
Velocity_Back_Right = math.sqrt(
    Velocity_Robot_X_Back_Right**2 + Velocity_Robot_Y_Back_Right**2
)

Steering_Angle_Front = (
    math.degrees(math.atan2(Velocity_Robot_Y_Front, Velocity_Robot_X_Front))
    - Offset_Angle_Front
)
Steering_Angle_Back_Left = (
    math.degrees(math.atan2(Velocity_Robot_Y_Back_Left, Velocity_Robot_X_Back_Left))
    - Offset_Angle_Back_Left
)
Steering_Angle_Back_Right = (
    math.degrees(math.atan2(Velocity_Robot_Y_Back_Right, Velocity_Robot_X_Back_Right))
    - Offset_Angle_Back_Right
)

print(
    f"Wheel A (Front @ {Angle_Front}°)      | "
    f"Pos: ({Front_wheelbase:+.3f}, {Front_track_width:+.3f}) | "
    f"Velocity: {Velocity_Front:.4f} | Steering: {Steering_Angle_Front:+.2f}°"
)

print(
    f"Wheel B (Back Left @ {Angle_Back_Left}°)  | "
    f"Pos: ({Back_Left_wheelbase:+.3f}, {Back_Left_track_width:+.3f}) | "
    f"Velocity: {Velocity_Back_Left:.4f} | Steering: {Steering_Angle_Back_Left:+.2f}°"
)

print(
    f"Wheel C (Back Right @ {Angle_Back_Right}°) | "
    f"Pos: ({Back_Right_wheelbase:+.3f}, {Back_Right_track_width:+.3f}) | "
    f"Velocity: {Velocity_Back_Right:.4f} | Steering: {Steering_Angle_Back_Right:+.2f}°"
)

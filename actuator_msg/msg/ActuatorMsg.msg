# 共通
int32 actuator_id          # アクチュエータの種別ごと（例：MD1とSERVO1は共存可能）
int32 actuator_type        # アクチュエータ種別 (0:デバッグ, 1:モタドラ, 2:サーボ, 3:ソレノイド)
string actuator_type_name  # "MD", "SERVO", "SV"
bool enable                # 有効・無効

# --- モタドラ用 ---
int32 motor_duty           # [%]
int32 motor_target_rpm     # [RPM]
int32 motor_target_pos     # 
int32 motor_target_torque  # 

# --- サーボ用 ---
int32 servo_angle_degree   # [deg]
int32 servo_speed          # 

# --- ソレノイドバルブ用 ---
bool solenoid_state        # [True/False]

# --- ロボマス用 ---
int32 robomas_target_angle # [degree]
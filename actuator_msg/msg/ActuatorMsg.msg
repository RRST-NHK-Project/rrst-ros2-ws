# アクチュエータ駆動用の自作メッセージ

int32 id # アクチュエータ種別(0:デバッグなど, 1:モタドラ, 2:サーボ, 3:ソレノイドバルブ)
string type # "MD", "SERVO", "SV" 

# 有効・無効
bool enable

# モタドラ
int32 duty # [%]
int32 target_rpm # [RPM]
int32 target_pos 
int32 target_torque 

# サーボ
int32 angle_degree # [deg]
int32 servo_speed #

# ソレノイドバルブ
bool state # [True/False]

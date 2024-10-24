from control import tf, feedback
from matplotlib import pyplot as plt
import numpy as np

# PID制御器のパラメータ
Kp = 0.6  # 比例
Ki = 0.03 # 積分
Kd = 0.03 # 微分
num = [Kd, Kp, Ki]
den = [1, 0]
K = tf(num, den)

# 制御対象
Kt = 1
J = 0.01
C = 0.1
num = [Kt]
den = [J, C, 0]
G = tf(num, den)

# フィードバックループ
sys = feedback(K*G, 1)

# 時間応答の計算
t = np.linspace(0, 3, 1000)
t, y = sys.step(T=t)

# プロット
plt.figure(figsize=(10, 6))
plt.plot(t, y)
plt.grid(True)
plt.axhline(1, color="b", linestyle="--")
plt.xlim(0, 3)
plt.ylim(0, max(y)*1.1)
plt.title('Step Response of PID Control System')
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.show()
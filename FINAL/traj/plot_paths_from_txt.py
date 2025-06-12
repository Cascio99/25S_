# plot_paths_from_txt.py
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def load_path_from_txt(filepath):
    """
    Load x, y, z positions from the custom FasterLIO .txt file
    """
    data = []
    with open(filepath, 'r') as f:
        for line in f:
            if line.startswith("#") or line.strip() == "":
                continue  # Skip comments or blank lines
            parts = line.strip().split()
            if len(parts) < 4:
                continue
            # Extract x, y, z
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            data.append([x, y, z])
    return np.array(data)

# 파일 경로 지정
default_txt = '/home/ysw/ws/25S_/FINAL/traj/traj_0.txt'
custom_txt = '/home/ysw/ws/25S_/FINAL/traj/traj_1.txt'

# 데이터 로딩
default_path = load_path_from_txt(default_txt)
custom_path = load_path_from_txt(custom_txt)

# 시각화
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Plot
ax.plot(default_path[:, 0], default_path[:, 1], default_path[:, 2], label='Default', color='blue')
ax.plot(custom_path[:, 0], custom_path[:, 1], custom_path[:, 2], label='Customized', color='red', linestyle='--')

# 시작점 / 끝점 강조 (옵션)
ax.scatter(*default_path[0], color='green', s=50, label='Start (Default)')
ax.scatter(*custom_path[0], color='orange', s=50, label='Start (Custom)')
ax.scatter(*default_path[-1], color='black', s=50, label='End (Default)')
ax.scatter(*custom_path[-1], color='purple', s=50, label='End (Custom)')

# 라벨링
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Trajectory Comparison (FasterLIO Default vs Custom)')
ax.legend()
ax.grid(True)

# 저장 및 표시
plt.savefig('fasterlio_path_compare.png', dpi=300)
plt.show()

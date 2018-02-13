import matplotlib.pyplot as plt
import numpy as np

lidar = []
radar = []

with open('./output.log') as f:
    lines = [l.strip().split(' ') for l in f.readlines()]

lines = lines[3:]

for line in lines:
    if line[1] == 'lidar':
        lidar.append(float(line[0]))
    else:
        radar.append(float(line[0]))

nis_90pct_lidar = 5.991
nis_90pct_radar = 7.815

fig, (ax1, ax2) = plt.subplots(2, 1)

ax1.plot(np.arange(len(lidar)), np.array(lidar))
ax1.plot(np.arange(len(lidar)), np.array([nis_90pct_lidar] * len(lidar)))
ax1.set_title('NIS, 95%, LIDAR', fontsize=32)

ax2.plot(np.arange(len(radar)), np.array(radar))
ax2.plot(np.arange(len(radar)), np.array([nis_90pct_radar] * len(radar)))
ax2.set_title('NIS, 95%, RADAR', fontsize=32)

plt.tight_layout()
plt.show()

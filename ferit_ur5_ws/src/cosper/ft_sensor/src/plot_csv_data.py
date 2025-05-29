import matplotlib.pyplot as plt
import numpy as np
import csv
import pandas as pd

# file = '/home/RVLuser/ferit_ur5_ws/unsuccessful_ft_sensor_data_static_part.csv'
# file = '/home/RVLuser/ferit_ur5_ws/unsuccessful_ft_sensor_data_door_panel.csv'
file = '/home/RVLuser/ferit_ur5_ws/ft_sensor_data/unsuccessful_6.csv'
image_save = file[:-3] + 'png'
print(image_save)
data = pd.read_csv(file)
num_points = data.shape[0]
print(data)

max_val = max(data.max(axis=0).values)
min_val = min(data.min(axis=0).values)
fig, (ax1, ax2, ax3) = plt.subplots(3)


# vert_axis_ticks = np.arange(min_val, max_val, step=1)
vert_axis_ticks = np.linspace(min_val, max_val, num=20)

# Add lines for better understanding of the graph
ax1.axvline(x = 1071, color='b', ls='--', label='press to slightly open')
ax2.axvline(x = 1071, color='b', ls='--', label='press to slightly open')
ax3.axvline(x = 1071, color='b', ls='--', label='press to slightly open')
ax1.axvline(x = 1726, color='c', ls='--', label='rotating gripper')
ax2.axvline(x = 1726, color='c', ls='--', label='rotating gripper')
ax3.axvline(x = 1726, color='c', ls='--', label='rotating gripper')
# ax1.axvline(x = 2324, color='g', ls='--', label='sensor zeroing before door contact')
# ax2.axvline(x = 2324, color='g', ls='--', label='sensor zeroing before door contact')
# ax3.axvline(x = 2324, color='g', ls='--', label='sensor zeroing before door contact')
ax1.axvline(x = 3055, color='y', ls='--', label='static part collision')
ax2.axvline(x = 3055, color='y', ls='--', label='static part collision')
ax3.axvline(x = 3055, color='y', ls='--', label='static part collision')
# ax1.axvline(x = 3226, color='r', ls='--', label='door contact')
# ax2.axvline(x = 3226, color='r', ls='--', label='door contact')
# ax3.axvline(x = 3226, color='r', ls='--', label='door contact')
ax1.set_ylabel('Force in x [N]')
ax2.set_ylabel('Force in y [N]')
ax3.set_ylabel('Force in z [N]')
ax3.set_xlabel('Time ticks')

# ax1.set_ylim([min_val, max_val])
# ax2.set_ylim([min_val, max_val])
# ax3.set_ylim([min_val, max_val])

ax1.plot(range(num_points), data['x'])
ax2.plot(range(num_points), data['y'])
ax3.plot(range(num_points), data['z'])
ax1.legend(loc = 'upper left')

manager = plt.get_current_fig_manager()
manager.window.showMaximized()

plt.show()
fig.savefig(image_save, dpi=300)
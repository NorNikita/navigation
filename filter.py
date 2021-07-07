import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

csv = pd.read_csv("a_w_file.csv")
frame = pd.DataFrame(csv)

previous_accel = np.array([frame['a_x'][0], frame['a_y'][1], frame['a_z'][0]])
previous_gyro = np.array([frame['w_x'][0], frame['w_y'][0], frame['w_z'][0]])
result_filter = pd.DataFrame(columns=["a_x", "a_y", "a_z", "w_x", "w_y", "w_z"])
alpha = 0.17

# фильтр низких частот
for index, row in frame.iterrows():
    accel = previous_accel + alpha * (np.array([[row['a_x']], [row['a_y']], [row['a_z']]]) - previous_accel)
    previous_accel = accel

    gyro = previous_gyro + alpha * (np.array([[row['w_x']], [row['w_y']], [row['w_z']]]) - previous_gyro)
    previous_gyro = gyro

    result_filter.loc[index] = [accel[0][0], accel[1][0], accel[2][0], gyro[0][0], gyro[1][0], gyro[2][0]]

# получение скорости и координат, в предположении что на итервале дескритизации ускорение кусочно постоянно
dt = 0.01
to_angle = 180 / np.pi
previous_coordinate = np.array([[0], [0], [0]])
previous_velocity = np.array([[result_filter['a_x'][0] * dt], [result_filter['a_y'][0] * dt], [result_filter['a_z'][0] * dt]])
# previous_velocity = np.array([[frame['a_x'][0] * dt], [frame['a_y'][0] * dt], [frame['a_z'][0] * dt]])

previous_angle = np.array([[0], [0], [np.pi / 2]])  # yaw pitch roll гол рысканья, тангажа, крена

res = pd.DataFrame(columns=["yaw", "pitch", "roll", "x", "y", "z"])


def norm(a, b):
    return np.sqrt(a * a + b * b)


for index, row in result_filter.iterrows():
    (a_x, a_y, a_z) = row['a_x'], row['a_y'], row['a_z']

    gyro_angles = previous_angle + dt * np.array([[row['w_x']], [row['w_y']], [row['w_z']]])
    accel_angles = np.array([[np.arctan(a_x / norm(a_y, a_z))], [np.arctan(a_y / norm(a_x, a_z))], [np.arctan(a_z / norm(a_x, a_y))]])
    previous_angle = gyro_angles

    angles = 0.5 * gyro_angles + 0.5 * accel_angles

    x1, x2, x3 = angles[0][0]*to_angle, angles[1][0]*to_angle, angles[2][0]*to_angle
    # x1, x2, x3 = gyro_angles[0][0], gyro_angles[1][0], gyro_angles[2][0]

    # матрица перехода в момент времени i
    matrix_i = np.array([[np.cos(x1) * np.cos(x3) - np.sin(x1) * np.sin(x2) * np.sin(x3), -np.sin(x1) * np.cos(x2),
                          np.cos(x1) * np.sin(x3) + np.cos(x3) * np.sin(x1) * np.sin(x2)],
                         [np.sin(x1) * np.cos(x3) + np.cos(x1) * np.sin(x2) * np.sin(x3), np.cos(x1) * np.cos(x2),
                          np.sin(x1) * np.sin(x3) - np.cos(x1) * np.sin(x2) * np.cos(x3)],
                         [-np.cos(x2) * np.sin(x3), np.sin(x2), np.cos(x2) * np.cos(x3)]])

    accel_abs = np.dot(matrix_i, np.array([[a_x], [a_y], [a_z]]))
    accel_abs[2][0] = accel_abs[2][0] - 9.81

    v_abs = previous_velocity + dt * accel_abs
    coordinate_abs = previous_coordinate + dt * previous_velocity

    previous_velocity = v_abs
    previous_coordinate = coordinate_abs

    res.loc[index] = [x1, x2, x3, coordinate_abs[0][0], coordinate_abs[1][0], coordinate_abs[2][0]]

# построение графиков
# fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 14))
# fig1.subplots_adjust(hspace=0.5)
# t = np.arange(0, len(result_filter.index))
#
# ax1.plot(t, result_filter["w_x"], t, frame["w_x"])
# ax1.set_xlabel('discrete time')
# ax1.set_ylabel('raw and filtered aссeleration a_x')
# ax1.grid(True)
#
# ax2.plot(t, result_filter["w_y"], t, frame["w_y"])
# ax2.set_xlabel('discrete time')
# ax2.set_ylabel('raw and filtered aссeleration a_y')
# ax2.grid(True)
#
# ax3.plot(t, result_filter["w_z"], t, frame["w_z"])
# ax3.set_xlabel('discrete time')
# ax3.set_ylabel('raw and filtered aссeleration a_z')
# ax3.grid(True)

fig, (x, y, z) = plt.subplots(3, 1, figsize=(14, 14))
fig.subplots_adjust(hspace=0.5)
t = np.arange(0, len(result_filter.index))

x.plot(t, res['yaw'])
x.set_xlabel('discrete time')
x.set_ylabel('yaw')
x.grid(True)

y.plot(t, res['pitch'])
y.set_xlabel('discrete time')
y.set_ylabel('pitch')
y.grid(True)

z.plot(t, res['roll'])
z.set_xlabel('discrete time')
z.set_ylabel('roll')
z.grid(True)

f = plt.figure().gca(projection='3d')
f.scatter(res['x'], res['y'], res['z'])


print(res)
plt.show()

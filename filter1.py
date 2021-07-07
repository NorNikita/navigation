import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from madgwik import MadgwickAHRS

dt = 0.01

csv = pd.read_csv("a_w_file.csv")
frame = pd.DataFrame(csv)

previous_coordinate = np.array([0, 0, 0])
previous_velocity = np.array([frame['a_x'][0] * dt, frame['a_y'][0] * dt, frame['a_z'][0] * dt])

ahrs = MadgwickAHRS(sampleperiod=0.01)
res = pd.DataFrame(columns=['x', 'y', 'z', 'yaw', 'pitch', 'roll'])

for index, row in frame.iterrows():
    gyro = np.array([row['w_x'], row['w_y'], row['w_z']])
    accel = np.array([row['a_x'], row['a_y'], row['a_z']])

    ahrs.update_imu(gyro, accel)

    (x1, x2, x3) = ahrs.quaternion.to_euler_angles()
    matrix_i = np.array([[np.cos(x1) * np.cos(x3) - np.sin(x1) * np.sin(x2) * np.sin(x3), -np.sin(x1) * np.cos(x2),
                          np.cos(x1) * np.sin(x3) + np.cos(x3) * np.sin(x1) * np.sin(x2)],
                         [np.sin(x1) * np.cos(x3) + np.cos(x1) * np.sin(x2) * np.sin(x3), np.cos(x1) * np.cos(x2),
                          np.sin(x1) * np.sin(x3) - np.cos(x1) * np.sin(x2) * np.cos(x3)],
                         [-np.cos(x2) * np.sin(x3), np.sin(x2), np.cos(x2) * np.cos(x3)]])

    accel_earth = np.dot(matrix_i, accel)
    accel_earth[2] = accel_earth[2] - 9.81

    velocity = previous_velocity + dt * accel_earth
    coordinate = previous_coordinate + dt * previous_velocity

    previous_velocity = velocity
    previous_coordinate = coordinate

    res.loc[index] = [coordinate[0], coordinate[1], coordinate[2], x1, x2, x3]

f = plt.figure().gca(projection='3d')
f.scatter(res['x'], res['y'], res['z'])

fig, (x, y, z) = plt.subplots(3, 1, figsize=(14, 14))
fig.subplots_adjust(hspace=0.5)
t = np.arange(0, len(res.index))

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

plt.show()

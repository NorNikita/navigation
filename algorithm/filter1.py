import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from madgwik import MadgwickAHRS
from quaternion import Quaternion
from scipy import signal

pd.options.mode.chained_assignment = None  # отключить warnings для pandas

samplePeriod = 0.03  # 1 / 256

# csv = pd.read_csv("./algorithm/straightLine.csv")
csv = pd.read_csv("acc_gyro.csv")
frame = pd.DataFrame(csv)

# чувсвительность настроена на диапазон -2g до 2g
frame["a_x"] = frame["a_x"].apply(lambda x: (x / 16384))
frame["a_y"] = frame["a_y"].apply(lambda y: (y / 16384))
frame["a_z"] = frame["a_z"].apply(lambda z: (z / 16384))

# чувсвительность настроена на диапазон -250 до 250 град/с
frame["w_x"] = frame["w_x"].apply(lambda x: 125 * (x / 16384))
frame["w_y"] = frame["w_y"].apply(lambda y: 125 * (y / 16384))
frame["w_z"] = frame["w_z"].apply(lambda z: 125 * (z / 16384))

startTime = 2  # 2
stopTime = 14  # 26

frame['time'] = [samplePeriod * index for index, val in enumerate(frame['Packet number'])]
frame = frame[(frame['time'] > startTime) & (frame['time'] < stopTime)]

LENGTH = len(frame['time'])

accX = frame['a_x']
accY = frame['a_y']
accZ = frame['a_z']
gyrX = frame['w_x']
gyrY = frame['w_y']
gyrZ = frame['w_z']
frame['accMagnitude'] = [np.sqrt(a['a_x'] ** 2 + a['a_y'] ** 2 + a['a_z'] ** 2)
                         for i, a in frame[['a_x', 'a_y', 'a_z']].iterrows()]

filterCutOff = 0.001
b, a = signal.butter(1, (2 * filterCutOff) / (1 / samplePeriod), btype='high')
accMagnituteFilt = np.abs(signal.filtfilt(b, a, frame['accMagnitude']))

filterCutOff = 6
b, a = signal.butter(1, (2 * filterCutOff) / (1 / samplePeriod), btype='low')
accMagnituteFilt = signal.filtfilt(b, a, accMagnituteFilt)

frame['accMagnitude_filter'] = accMagnituteFilt

# интервалы где приборный трехгранник покоится относительно земли. 1 - покой, 0 - движение
frame['stationary'] = np.where(frame['accMagnitude_filter'] < 0.05, 1, 0)

# строим график величины ускорения
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
t = np.arange(len(frame['a_x']))
ax1.plot(t, frame['accMagnitude'], label='норма вектора ускорения')
ax1.set_title('without filter')
ax1.legend()
ax1.axis([0, LENGTH, 0, 6])

# график индикаторной функции где тело покоится, а где нет
ax2.plot(t, accMagnituteFilt, label='фильрованая норма ускорения (полосовой фильтр Баттерворта)')
ax2.plot(t, frame['stationary'], label='1 - покой, 0 - движение (индикаторная функция)')
ax2.set_title('with filter')
ax2.legend()
ax2.axis([0, LENGTH, 0, 4])
plt.tight_layout()

initPeriod = 2
firstIndex = frame['time'].head(1).index[0]
initPeriodIndexes = [(x + firstIndex) for x in np.where(frame['time'] < frame['time'][firstIndex] + initPeriod)][0]

accX_init = frame['a_x'][initPeriodIndexes]
accY_init = frame['a_y'][initPeriodIndexes]
accZ_init = frame['a_z'][initPeriodIndexes]

# вычисляем "начальный" кватернион
ahrs = MadgwickAHRS(beta=1)
for i in range(1, 2000):
    ahrs.update_imu([0, 0, 0], [np.mean(accX_init), np.mean(accY_init), np.mean(accZ_init)])

# считаем массив кватернионов для каждого момента времени
quat = [ahrs.quaternion]
for index in range(firstIndex, len(frame['time']) + firstIndex):
    if frame['stationary'][index] == 1:
        ahrs.beta = 0.5
    else:
        ahrs.beta = 0

    gyro = [frame['w_x'][index], frame['w_y'][index], frame['w_z'][index]]
    acc = [frame['a_x'][index], frame['a_y'][index], frame['a_z'][index]]
    ahrs.update_imu(np.deg2rad(gyro), acc)

    quat.append(ahrs.quaternion)

    # преобразуем ускорение к с.к. связанной с землёй
    acc_earth = ahrs.quaternion.rotate(Quaternion(0, acc[0], acc[1], acc[2]))
    frame['a_x'][index] = acc_earth[1] * 9.81
    frame['a_y'][index] = acc_earth[2] * 9.81
    frame['a_z'][index] = acc_earth[3] * 9.81

# строим график ускорения в абс. с.к.
fig1, (accel) = plt.subplots(1, 1, sharex=True)
t = np.arange(len(frame['a_x']))
accel.plot(t, frame['a_x'], label='a_x', linewidth=0.5)
accel.plot(t, frame['a_y'], label='a_y', linewidth=0.5)
accel.plot(t, frame['a_z'], label='a_z', linewidth=0.5)
accel.set_title('acceleration')
accel.legend()
plt.tight_layout()

# вычитаем g из 3й компоненты ускорения
frame['a_z'] = frame['a_z'] - 9.81

# вычисляем скорость
vX_prev = vY_prev = vZ_prev = 0
vX = [vX_prev]
vY = [vY_prev]
vZ = [vZ_prev]
for index in range(firstIndex + 1, len(frame['time']) + firstIndex):
    if frame['stationary'][index] == 1:
        vX_curr = vY_curr = vZ_curr = 0
    else:
        vX_curr = vX_prev + frame['a_x'][index]
        vY_curr = vY_prev + frame['a_y'][index]
        vZ_curr = vZ_prev + frame['a_z'][index]

    vX.append(vX_curr)
    vY.append(vY_curr)
    vZ.append(vZ_curr)

    vX_prev = vX_curr
    vY_prev = vY_curr
    vZ_prev = vZ_curr

frame['v_x'] = vX
frame['v_y'] = vY
frame['v_z'] = vZ

# вычисляем дрифт гироскопа
frame['driftX'] = np.zeros(len(frame['stationary']))
frame['driftY'] = np.zeros(len(frame['stationary']))
frame['driftZ'] = np.zeros(len(frame['stationary']))
start = np.where(np.diff(frame['stationary']) == -1)[0]
end = np.where(np.diff(frame['stationary']) == 1)[0]

for index in range(len(start)):
    indexStart = start[index] + firstIndex
    indexEnd = end[index] + firstIndex

    driftRateX = (frame['v_x'][indexEnd] - frame['v_x'][indexStart]) / (indexEnd - indexStart)
    driftRateY = (frame['v_y'][indexEnd] - frame['v_y'][indexStart]) / (indexEnd - indexStart)
    driftRateZ = (frame['v_z'][indexEnd] - frame['v_z'][indexStart]) / (indexEnd - indexStart)

    enum = np.array(range(1, indexEnd - indexStart))
    curr = [enum.T * driftRateX, enum.T * driftRateY, enum.T * driftRateZ]

    # TODO отрефачить
    for i in range(indexStart, indexEnd - 1):
        frame['driftX'][i] = curr[0][i - indexStart]
        frame['driftY'][i] = curr[1][i - indexStart]
        frame['driftZ'][i] = curr[2][i - indexStart]

# вычитаем дрейф гироскопа
frame['v_x_d'] = frame['v_x'] - frame['driftX']
frame['v_y_d'] = frame['v_y'] - frame['driftY']
frame['v_z_d'] = frame['v_z'] - frame['driftZ']

# вычисляем координаты
x_prev = y_prev = z_prev = 0
x = []
y = []
z = []
for index in range(firstIndex, len(frame['time']) + firstIndex):
    x_curr = x_prev + frame['v_x_d'][index] * samplePeriod
    y_curr = y_prev + frame['v_y_d'][index] * samplePeriod
    z_curr = z_prev + frame['v_z_d'][index] * samplePeriod

    x.append(x_curr)
    y.append(y_curr)
    z.append(z_curr)

    x_prev = x_curr
    y_prev = y_curr
    z_prev = z_curr

fig2, (velocity) = plt.subplots(1, 1, sharex=True)
t = np.arange(LENGTH)
velocity.plot(t, frame['v_x'], label='v_x')
velocity.plot(t, frame['v_y'], label='v_y')
velocity.plot(t, frame['v_z'], label='v_z')
velocity.set_title('velocity')
velocity.set_xlabel('discrete time')
velocity.legend()
plt.tight_layout()

trajectory = plt.figure().add_subplot(projection='3d')
trajectory.plot(x, y, z, label='parametric curve')
trajectory.set_xlabel('x')
trajectory.set_ylabel('y')
trajectory.set_zlabel('z')
trajectory.set_title('trajectory')

fig3, (coord) = plt.subplots(1, 1, sharex=True)
t = range(len(frame['a_x']))
coord.plot(t, x, label='x', linewidth=0.5)
coord.plot(t, y, label='y', linewidth=0.5)
coord.plot(t, z, label='z', linewidth=0.5)
coord.set_title('coordinates')
coord.set_xlabel('discrete time')
coord.legend()
plt.tight_layout()

plt.show()

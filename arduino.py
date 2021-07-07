import re
import time as t
from array import *

import numpy as np
import pandas as pd
import serial as ser

arduino = ser.Serial('/dev/ttyACM0', 9600, timeout=1)

x_arr = array('d')
y_arr = array('d')
z_arr = array('d')

wx_arr = array('d')
wy_arr = array('d')
wz_arr = array('d')

times = array('d')

x_measure = b'ax'
y_measure = b'ay'
z_measure = b'az'

wx_measure = b'wx'
wy_measure = b'wy'
wz_measure = b'wz'

previousByte = x_measure
count = 0

begin = t.time()
while True:
    byte = arduino.readline()
    measure = re.findall(r'-?\d+', str(byte))

    if previousByte.startswith(x_measure) and len(measure) > 0:
        x_arr.append(float(measure[0]))

    elif previousByte.startswith(y_measure) and len(measure) > 0:
        y_arr.append(float(measure[0]))

    elif previousByte.startswith(z_measure) and len(measure) > 0:
        z_arr.append(float(measure[0]))

    elif previousByte.startswith(wx_measure) and len(measure) > 0:
        wx_arr.append(float(measure[0]))

    elif previousByte.startswith(wy_measure) and len(measure) > 0:
        wy_arr.append(float(measure[0]))

    elif previousByte.startswith(wz_measure) and len(measure) > 0:
        wz_arr.append(float(measure[0]))
    else:
        previousByte = byte

    count += 1

    if count == 4000:
        break
print("time: ", t.time() - begin)

size = np.min([len(x_arr), len(y_arr), len(z_arr), len(wx_arr), len(wy_arr), len(wz_arr)]) - 1

matrix = np.array([x_arr[0:size], y_arr[0:size], z_arr[0:size], wx_arr[0:size], wy_arr[0:size], wz_arr[0:size]])
matrix = matrix.transpose()

frame = pd.DataFrame(data=matrix, columns=["a_x", "a_y", "a_z", "w_x", "w_y", "w_z"])

# чувсвительность настроена на диапазон -2g до 2g
frame["a_x"] = frame["a_x"].apply(lambda x: 9.82 * (x / 16384))
frame["a_y"] = frame["a_y"].apply(lambda y: 9.82 * (y / 16384))
frame["a_z"] = frame["a_z"].apply(lambda z: 9.82 * (z / 16384))

# чувсвительность настроена на диапазон -250 до 250 град/с
frame["w_x"] = frame["w_x"].apply(lambda x: 125 * (x / 16384))
frame["w_y"] = frame["w_y"].apply(lambda y: 125 * (y / 16384))
frame["w_z"] = frame["w_z"].apply(lambda z: 125 * (z / 16384))

# запишем результаты измерений в csv файл чтобы потом не ждать записи с датчиков
frame.to_csv("a_w_file.csv", index=False)

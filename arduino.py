import serial as ser

"""
скрипт для записи измерений акселерометров и гироскопов в файл ./algorithm/acc_gyro.csv
"""

filename = 'algorithm/acc_gyro.csv' # для построения траектории
filename1 = 'algorithm/acc_gyro1.csv'  # для построения углов
filename2 = 'algorithm/acc_gyro2.csv'

port = '/dev/ttyACM0' #'/dev/cu.usbmodem141101'
arduino = ser.Serial(port, 9600, timeout=1)

file = open(filename2, "a")
file.write("Packet number,a_x,a_y,a_z,w_x,w_y,w_z\n")

after = 0  # to 100
while True:
    after += 1
    if after >= 100:
        inStr = arduino.readline().decode('utf-8').strip('\r\n') + '\n'
        file.write(inStr)
        print(inStr)

import serial
import time
import matplotlib.pyplot as plt
from collections import deque

def read_imu_data(port='COM11', baudrate=115200):
    ser = serial.Serial(port, baudrate)
    time.sleep(2)

    x_data, y_data = deque(maxlen=100), deque(maxlen=100)
    fig, ax = plt.subplots()
    plt.ion()

    while True:
        start_time = time.time()
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            try:
                parts = line.split(',')
                x = float(parts[0])
                y = float(parts[1])

                x_data.append(x)
                y_data.append(y)

                ax.clear()
                ax.plot(x_data, y_data, 'bo-')
                ax.set_xlabel('X Position (m)')
                ax.set_ylabel('Y Position (m)')
                ax.set_title('Planar Position (Map)')
                plt.draw()
                plt.pause(0.01)
            except Exception as e:
                print("Error parsing:", line, e)

        elapsed_time = time.time() - start_time
        sleep_time = max(0, 0.0067 - elapsed_time)  # Match the loop time (150 Hz)
        time.sleep(sleep_time)

if __name__ == "__main__":
    read_imu_data(port='COM11', baudrate=115200)

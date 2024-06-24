import cv2
import numpy as np
from lineactuator import *
from threading import Thread
import os 
import time
from loguru import logger
import math

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

class Controller:
    def __init__(self) -> None:
        logger.add(os.getcwd()  + '\\test.log')
        logger.info('Initialization Two Line Actuators: X_controller and Y_controller')
        self.X_controller = X_LineActuator("COM12", 57600, 0, "CURRENT-BASED-POSITION", 80, "COM12", 57600, 2, "CURRENT-BASED-POSITION", 80)
        self.Y_controller = Y_LineActuator("COM12", 57600, 1, "CURRENT-BASED-POSITION", 80, "COM12", 57600, 3, "CURRENT-BASED-POSITION", 80)
        self.X = None
        self.Y = None
        self.p1 = None
        self.p2 = None
        self.p3 = None
        self.p4 = None

        self.pid_x = PID(1.0, 0.1, 0)
        self.pid_y = PID(1.0, 0.1, 0)

    def calibration(self):
        current = 50
        p1, p2 = self.X_controller.calibration(-current, -current)
        p3, p4 = self.Y_controller.calibration(-current, -current)
        logger.info(f"Calibration Result: X_controller Initial Position ({p1}, {p2}); Y_controller Initial Position ({p3},{p4})")
        logger.info(f"Calibration Finished")
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.p4 = p4
        self.X = 0
        self.Y = 0

    def move(self, x, y):
        theta = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2)
        xy_2_m = 4096/(2*math.pi*2.84)
        p1 = self.p1 - r*math.cos(theta)*xy_2_m
        p2 = self.p2 + r*math.cos(theta)*xy_2_m
        p3 = self.p3 + r*math.sin(theta)*xy_2_m
        p4 = self.p4 - r*math.sin(theta)*xy_2_m
        
        self.X_controller.move(int(p1), int(p2))
        self.Y_controller.move(int(p3), int(p4))

    def disable(self):
        self.X_controller.disable()
        self.Y_controller.disable()

def draw_trajectory(frame, center_points):
    for i in range(1, len(center_points)):
        if center_points[i - 1] is None or center_points[i] is None:
            continue
        cv2.line(frame, center_points[i - 1], center_points[i], (0, 0, 255), 1)

def calculate_rmse(center_points, frame_center):
    if not center_points:
        return 0
    squared_distances = [(p[0] - frame_center[0])**2 + (p[1] - frame_center[1])**2 for p in center_points]
    mean_squared_distance = np.mean(squared_distances)
    rmse = np.sqrt(mean_squared_distance)
    return rmse

def write_rmse(rmse, filename="666.csv"):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        for e in rmse:
            writer.writerow(rmse)

def write_to_csv(center_points, filename="6.csv"):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["X", "Y"])
        for point in center_points:
            writer.writerow(point)

def main():
    controller = Controller()
    controller.calibration()

    rmses = []
    center_points = []
    cap = cv2.VideoCapture(1,cv2.CAP_DSHOW)
    prev_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_height, frame_width = frame.shape[:2]
        frame_center = (frame_width // 2, frame_height // 2)
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)

        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)

            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                center = (cX, cY)

                center_points.append(center)

                cv2.circle(frame, center, 2, (0, 255, 0), -1)

                error_x = frame_center[0] - cX
                error_y = frame_center[1] - cY

                current_time = time.time()
                dt = current_time - prev_time
                prev_time = current_time


                control_x = controller.pid_x.compute(error_x, dt)
                control_y = controller.pid_y.compute(error_y, dt)

                control_x = control_x / 5
                control_y = control_y / 5

                if control_x > 3:
                    control_x = 3
                elif control_y > 3:
                    control_y = 3

                controller.move(int(-control_x), int(control_y))

        draw_trajectory(frame, center_points)
        cv2.circle(frame, frame_center, 2, (255, 0, 0), -1)

        rmse = calculate_rmse(center_points, frame_center)
        cv2.putText(frame, f'RMSE: {rmse:.2f}', (10, frame_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        rmses.append(rmse)

        cv2.imshow("Rmse test (holding)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    controller.disable()
    write_to_csv(center_points)
    write_rmse(rmses)

if __name__ == "__main__":
    main()

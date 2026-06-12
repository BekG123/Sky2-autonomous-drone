import time
from time import sleep
from imu import MPU6050
from machine import Pin, PWM, UART, ADC, I2C
import math

led = Pin(2, Pin.OUT) # receiver indicator LED
uart = UART(0, baudrate = 115200, tx=Pin(0), rx=Pin(1)) # serial connection
motor1 = PWM(Pin(3)) # front left CW
motor2 = PWM(Pin(4)) # back left CCW
motor3 = PWM(Pin(5)) # back right CW
motor4 = PWM(Pin(6)) # front right CCW
power = ADC(Pin(28))
motor1.freq(200) # 200 Hz PWM frequency
motor2.freq(200)
motor3.freq(200)
motor4.freq(200)
i2c = I2C(0, sda = Pin(16), scl = Pin(17), freq = 400000)
imu = MPU6050(i2c)

POWER = 0

# Internal state variables (must persist between loop calls)
angle_pitch = 0.0
angle_roll = 0.0
angle_yaw = 0.0

x1 = 0
y1 = 0
x2 = 0
y2 = 0
t = 0

# PID State
prev_error = [0, 0, 0]  # Pitch, Roll, Yaw
integral = [0, 0, 0]

dt = 0.03

def serial(message):
    uart.write(message.encode('ascii') + b"\r\n") # send message to serial
    while uart.any():
        response = uart.read()
        if response:
            print(response.decode('utf-8', 'ignore')) # print message

def setup(): # sets up LoRa module
    print("Setup started")
    motor1.duty_u16(25000)
    motor2.duty_u16(25000)
    motor3.duty_u16(25000)
    motor4.duty_u16(25000)
    serial("AT+FACTORY") # reset to default settings
    sleep(0.5)
    serial("AT+ADDRESS=2") # set receiver address to 2
    sleep(0.5)
    serial("AT+PARAMETER=9,9,1,4") # parameters: spreading factor = 9, bandwidth = 9 (500 kHz), coding rate = 1 (4/5), preamble length = 4
    sleep(0.5)
    print("LoRa Setup complete")
    sleep(1.5)
    print("motor low")
    motor1.duty_u16(15000)
    motor2.duty_u16(15000)
    motor3.duty_u16(15000)
    motor4.duty_u16(15000)
    sleep(9)
    print("motor setup complete")
    
def pid():
    global angle_pitch, angle_roll, angle_yaw, x1, x2, y1, y2, t, dt
    # PID Constants (These require manual tuning)
    # Pitch/Roll usually share gains; Yaw is usually different
    Kp, Ki, Kd = 0.5, 0.00, 0.00
    Kp_y, Ki_y, Kd_y = 0.0, 0.0, 0.0

    # --- 1. SENSOR FUSION (Estimate current state) ---
    # Calculate pitch and roll from accelerometer (g)
    # Pitch: Rotation around Y axis. Roll: Rotation around X axis.
    accel_pitch = math.degrees(math.atan2(imu.accel.x, math.sqrt(imu.accel.y**2 + imu.accel.z**2)))
    accel_roll = math.degrees(math.atan2(imu.accel.y, imu.accel.z))

    # Complementary Filter: 98% Gyro (deg/s), 2% Accel
    # This filters out vibration noise and prevents long-term drift
    angle_pitch = 0.98 * (angle_pitch - imu.gyro.y * dt) + 0.02 * accel_pitch
    angle_roll = 0.98 * (angle_roll + imu.gyro.x * dt) + 0.02 * accel_roll
    angle_yaw += imu.gyro.z * dt # Pure integration for yaw (will drift without a magnetometer)

    current_angles = [angle_pitch, angle_roll, angle_yaw]
    dt = min((time.ticks_us() - t) / 1000000, 0.03)
    t = time.ticks_us()
    targets = [y2, x2, x1]
    corrections = [0, 0, 0]

    # --- 2. PID CALCULATION (Generate corrections) ---
    for i in range(3):
        error = targets[i] - current_angles[i]
        
        # Proportional
        P = (Kp_y if i == 2 else Kp) * error
        
        # Integral (with basic clamping to prevent 'windup')
        integral[i] = max(-10, min(10, integral[i] + error * dt))
        I = (Ki_y if i == 2 else Ki) * integral[i]
        
        # Derivative (Rate of change)
        D = (Kd_y if i == 2 else Kd) * (error - prev_error[i]) / dt
        
        corrections[i] = P
        prev_error[i] = error
    motor1.duty_u16(round(15000 + min(10000, max(0, (y1 + corrections[0] + corrections[1]) * 100)))); # motor 1 speed
    motor2.duty_u16(round(15000 + min(10000, max(0, (y1 - corrections[0] + corrections[1]) * 100)))); # motor 2 speed
    motor3.duty_u16(round(15000 + min(10000, max(0, (y1 - corrections[0] - corrections[1]) * 100)))); # motor 3 speed
    motor4.duty_u16(round(15000 + min(10000, max(0, (y1 + corrections[0] - corrections[1]) * 100)))); # motor 4 speed
    print(y1 + corrections[0] + corrections[1], y1 - corrections[0] + corrections[1], y1 - corrections[0] - corrections[1], y1 + corrections[0] - corrections[1])
    '''Sets all motor speeds using conventional drone RC controls
    y1 = throttle, x1 = yaw, y2 = pitch, x2 = roll
    High y1/throttle = all four motors high
    High x1 = right/CW yaw = CCW motors (2 and 4) high
    High y2 = forward pitch = back motors (2 and 3) high
    High x2 = right roll = left motors (1 and 2) high
    Each individual control (throttle, yaw, pitch, or roll) can drive motors to full or no power
    min(65535, max(0, motor speed)) keeps motor speed between 0 and 65535
    Otherwise, it may overflow with multiple simultaneous controls
    Numerical constant in each output "centers" motor to half speed with no joystick input'''

def receive():
    global x1, y1, x2, y2
    if uart.any():
        data = uart.read().decode('utf-8', 'ignore').strip()
        if "+RCV" in data: # data received from serial
            joystick = data.split(',')[2] # extract message from received data
            x1 = int(joystick[:2]) / 5.0 - 10.0 # split into two-digit numbers
            y1 = int(joystick[2:4])
            x2 = int(joystick[4:6]) / 5.0 - 10.0
            y2 = int(joystick[6:]) / 5.0 - 10.0
            led.high() # blink indicator LED
            sleep(0.001)
            led.low()


power_count = 0
while power.read_u16() < POWER:
    sleep(0.01)
setup()
while power_count < 300 and abs(angle_pitch) < 45 and abs(angle_roll) < 45:
    receive()
    pid()
    if power.read_u16() < POWER:
        power_count += 1
    else:
        power_count = 0
motor1.duty_u16(0)
motor2.duty_u16(0)
motor3.duty_u16(0)
motor4.duty_u16(0)


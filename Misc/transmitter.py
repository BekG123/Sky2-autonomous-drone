from time import sleep
from machine import Pin, PWM, UART, ADC
import sys

led = Pin(2, Pin.OUT) # transmit indicator LED
switch = ADC(Pin(28))
switch_count = 0
left = Pin(14, Pin.OUT) # left joystick power pin
right = Pin(15, Pin.OUT) # right joystick power pin
uart = UART(0, baudrate = 115200, tx=Pin(0), rx=Pin(1)) # serial communication object
x = ADC(Pin(26)) # analog input x
y = ADC(Pin(27)) # analog input y

def serial(message):
    uart.write(message.encode('ascii') + b"\r\n") # write to serial
    while uart.any():
        response = uart.read().decode('utf-8', 'ignore').strip()
        print(response)
        if "+OK" in response: # blink indicator LED of response is OK
            led.value(1)
            sleep(0.005)
            led.value(0)

def setup(): # sets up LoRa module
    print("Setup started")
    serial("AT+FACTORY") # reset to default settings
    sleep(0.5)
    serial("AT+ADDRESS=1") # set address to 1
    sleep(0.5)
    serial("AT+PARAMETER=9,9,1,4") # parameters: spreading factor = 9, bandwidth = 9 (500 kHz), coding rate = 1 (4/5), preamble length = 4
    sleep(0.5)
    
def transmit(message):
    length = len(message)
    serial(f"AT+SEND=2,{length},{message}") # sends message to receiver at address 2 via serial

setup()
while switch_count < 30:
    left.value(1) # power left joystick
    right.value(0) # ground right joystick
    sleep(0.025) # wait 25 ms (half of minimum latency of LoRa transmissions)
    y1 = max(0, min(100 - round(x.read_u16() / 270 - 12), 99)) # read y1 and transform to 0 to 99 scale
    x1 = max(0, min(round(y.read_u16() / 270 - 10), 99)) # read & transform x1
    if switch.read_u16() < 1000:
        switch_count += 1
    else:
        switch_count = 0
    right.value(1) # power right joystick
    left.value(0) # ground left joystick
    sleep(0.025) # wait 25 ms
    y2 = max(0, min(100 - round(x.read_u16() / 270 - 13), 99)) # read & transform y2
    x2 = max(0, min(round(y.read_u16() / 270 - 12), 99)) # read & transform x2
    joystick = "0" * (x1 < 10) + str(x1) + "0" * (y1 < 10) + str(y1) + "0" * (x2 < 10) + str(x2) + "0" * (y2 < 10) + str(y2)
    print(joystick)
    transmit(joystick) # add leading zeroes and convert to string of length 8
while True:
    transmit("50005050")

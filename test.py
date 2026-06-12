# file: BootPicoW.py
# description: this file is intended to test whether the Pico is fully functional upon start
# to allow succesful testing of othe modules

from machine import UART, Pin, PWM, ADC
import time

# flash the led upon boot
status_led = Pin(2, Pin.OUT) # Bug 1: Software - check correct pin 

# function to flash led 
while True:
    
    status_led.value(1) # on
    print("Led On")
    time.sleep(0.1) # Bug 2: Hardware - check for proper connections  
    status_led.value(0) # off
    print("Led Off")
    time.sleep(0.1)


    
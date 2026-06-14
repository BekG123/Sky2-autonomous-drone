from machine import ADC
import time

# Create ADC objects for each axis
x_adc = ADC(26)  # GP26
y_adc = ADC(27)  # GP27
z_adc = ADC(28)  # GP28

VREF = 3.3  # Pico ADC reference voltage

while True:
    x_raw = x_adc.read_u16()
    y_raw = y_adc.read_u16()
    z_raw = z_adc.read_u16()

    # Convert to voltage
    x_v = x_raw * VREF / 65535
    y_v = y_raw * VREF / 65535
    z_v = z_raw * VREF / 65535

    print("X:", x_v, "V   Y:", y_v, "V   Z:", z_v, "V")

    time.sleep(0.2)
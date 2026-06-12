from machine import Pin, SPI, ADC
from time import sleep
from nrf24l01 import NRF24L01
import struct

led = Pin(1, Pin.OUT)
spi = SPI(0, sck=Pin(2), mosi=Pin(3), miso=Pin(4), baudrate=100000)
ce = Pin(6, Pin.OUT, value=0)
csn = Pin(5, Pin.OUT, value=1)
nrf = NRF24L01(spi, csn, ce, payload_size=8)
x_acc = ADC(28)  # ADC2
y_acc = ADC(27)  # ADC1
z_acc = ADC(26)  # ADC0

nrf.open_tx_pipe(b'\xe1\xf0\xf0\xf0\xf0')

while True:
    x = int( ((x_acc.read_u16() - 2**(15)) * (3.3 / (2**16)) / 0.8) )
    y = int( ((y_acc.read_u16() - 2**(15)) * (3.3 / (2**16)) / 0.8) )
    z = int( ((z_acc.read_u16() - 2**(15)) * (3.3 / (2**16)) / 0.8) )
    payload = struct.pack("HHH", x, y, z) + b'\x00\x00'
    try:
        nrf.send(payload)
        print(f"Sent: x={x} y={y} z={z}")
    except OSError as e:
        print("Send failed:", e)

led.value(1)
sleep(0.01)
led.value(0)
sleep(0.04)

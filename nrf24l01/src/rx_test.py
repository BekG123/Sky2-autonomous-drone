from machine import Pin, SPI
from time import sleep
from nrf24l01 import NRF24L01

# pin wiring
led = Pin(1, Pin.OUT)
spi = SPI(0, sck = 2, mosi = 3, miso = 4, baudrate = 100000)
ce = Pin(5, Pin.OUT, value = 0)
csn = Pin(6, Pin.OUT, value = 1)
nrf = NRF24L01(spi, csn, ce, payload_size=8)

nrf.open_rx_pipe(1, b'\xe1\xf0\xf0\xf0\xf0')
nrf.start_listening()
while True:
    led.value(0)
    if nrf.any():
        while nrf.any():
            data = nrf.recv().decode('utf-8', 'ignore').strip()
        print(data)
        led.value(1)
        sleep(0.01) 
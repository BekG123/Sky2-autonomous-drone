import network
import socket
from time import sleep
from picozero import pico_temp_sensor, pico_led
import machine
import rp2
import sys

ssid = "SkyRunners"
password = "livelaughlockheed"

def connect():
    #Connect to WLAN
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while wlan.isconnected() == False:
        if rp2.bootsel_button() == 1:
            sys.exit()
        print('Waiting for connection...')
        pico_led.on()
        sleep(0.5)
        pico_led.off()
        sleep(0.5)
    ip = wlan.ifconfig()[0]
    print(f'Connected on {ip}')
    pico_led.on()
    return ip

connect()
while True:
    ai = socket.getaddrinfo("192.168.8.170", 80)
    addr = ai[0][-1]
    s = socket.socket()
    s.connect(addr)
    s.send(b"Anything")
    ss = str(s.recv(512))
    print(ss)
    
print('Connected!')
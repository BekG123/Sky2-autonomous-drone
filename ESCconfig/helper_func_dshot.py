# Author: Bekhruz Malikov
import matplotlib.pyplot as plt
import numpy as np

# fetch packet bits that were created by Pico CPU
# put them into a list 
# convert integer type packet to list type packet 
def fetch_sample_packet_bits(packet, length=16):
    """

    """
    fetched_packet = []
    for i in range(length):
        # add MSB first 
        fetched_bit = (packet >> (length - 1 - i)) & 1
        fetched_packet.append(fetched_bit)
    return fetched_packet

# iterate through the list of bit packet and add them to a new list 
# list will be used to simulate dshot state machine (PIO)
# convert the bit packet into ESC bit packet 
def simulate_dshot_sm(packet):
    """

    """
    # fetch_packet = fetch_sample_packet_bits(packet)
    track_esc_packet = []
    track_esc_packet.append(0) # pull(noblock).side(0)
    track_esc_packet.append(0) # set(x, 15).side(0)
    for i in range(len(packet)):
        track_esc_packet.append(1) # out(y,1).side(1)
        track_esc_packet.append(1) # jmp(not_y, "zero").side(1)
        # "one" state
        if packet[i] == 1:
            track_esc_packet.append(1)
            track_esc_packet.append(1)
        #  "zero" state 
        elif packet[i] == 0:
            track_esc_packet.append(0)
            track_esc_packet.append(0)
        track_esc_packet.append(0) # jmp(x_dec, "bitloop").side(0)
    # "done" state
    track_esc_packet.append(0)
    return track_esc_packet

# Just plotting 
def plot_ESC_waveform(packet):
    """

    """
    # timing information just in case 
    sys_clock = 250_000_000 # cyc/sec - Pico internal system runs on 250 MHz
    PIO_clock = 2_400_000 # State Machine runs its instructions at 2.4 MHz 
    cycles_per_bit = 5 
    cycles = 83

    time_axis = []
    for i in range(len(packet)):
        # divide x-axis into microsecond timings
        time_axis.append( (i / PIO_clock) * 1e6) # µs

    plt.figure(figsize=(20, 3))
    plt.step(time_axis, packet, where="post")
    plt.title("ESC Bit Packet")
    plt.axis((0, time_axis[-1], -0.1, 1.1))
    plt.xlabel("Time (µs)")
    plt.ylabel("Bit Level")
    plt.show()
    

# internal to this file :) 
def main():
    sample_packet = 0b1111111111111111 # full throttle 
    fetched_packet = fetch_sample_packet_bits(sample_packet, length=16)
    res_esc_packet = simulate_dshot_sm(fetched_packet)
    print(res_esc_packet)
    plot_ESC_waveform(res_esc_packet)

if __name__ == "__main__":
    main()
# Author: Bekhruz Malikov
import rp2
from rp2 import DMA
import machine
from machine import Pin, mem32
import time

machine.freq(120_000_000) # downclock MCU by 5 MHz (125 MHz) 
MOTOR_PINS = [6, 5, 4, 3] # GPIO on PicoW

################################################################
### @rp2.asm_pio(...) takes the Python function defined,
### wraps it with a decorator, and converts it into a low-level
### PIO program that the RP2040 can run directly on its hardware
### state machine, helpful for sending packets to the ESC ###
################################################################

@rp2.asm_pio(
    out_shiftdir=rp2.PIO.SHIFT_LEFT, #Read bits left to right (MSB first) — DShot requires this
    autopull=False, # don't grab data yet 
    set_init=rp2.PIO.OUT_LOW, # SET pin starts LOW
    out_init=rp2.PIO.OUT_LOW, # The OUTPUT pin starts LOW
    sideset_init=rp2.PIO.OUT_LOW  # Signal wire to ESC starts LOW
    )

def dshot():
    """
    RP2040 PIO program that transmits a single 16-bit DShot packet.

    This state machine pulls a packet from the TX FIFO, shifts the
    packet out one bit at a time, and generates the precise HIGH/LOW
    pulse timings required by the DShot protocol. The packet consists
    of an 11-bit throttle value, a 1-bit telemetry request bit, and a
    4-bit CRC checksum.

    Notes:
        - Runs entirely on the RP2040 PIO hardware.
        - Uses side-set operations to control the ESC signal line. (reference documentation)
        - Expects a 16-bit DShot packet left-aligned in a 32-bit word.
        - Timing is configured for DShot600 when the state machine
          frequency is set to 4.8 MHz.
    """
    #  ESC signal must stay low during fetching/pull of bits from CPU
    pull(block)
 
    # 16 loop iterations of 81 cycles =  cycles -> 5.5 seconds expected  
    set(x, 15) # set length of packet to 16
    
    label("BITLOOP")
    
    # 1024 -> 1 00000000000 1 000 
    # 0xFF00 -> 11111111 00000000 
    out(y, 1).side(1) [1] # pass into Y register to shift the packet by 1 bit, then check if bit is 0 or 1
    jmp(not_y, "ZERO").side(1) 

    nop().side(1) [2]

    jmp(x_dec, "BITLOOP").side(0) [1]
    jmp("DONE")

    label("ZERO")
    nop().side(0) [2]
    
    jmp(x_dec, "BITLOOP").side(0) [1]
    
    label("DONE")
    nop().side(0) [1]


def make_packet(throttle, telemetry=False):
    """
    Construct a valid 16-bit DShot packet.

    The packet format is:

        [11-bit throttle][1-bit telemetry][4-bit CRC]

    The CRC is calculated using the standard DShot checksum algorithm
    and is used by the ESC to detect transmission errors.

    Args:
        throttle (int):
            Throttle value in the DShot range (typically 0-2047).

        telemetry (bool, optional):
            Whether to request telemetry data from the ESC.
            Defaults to False.

    Returns:
        int:
            Complete 16-bit DShot packet containing throttle,
            telemetry flag, and CRC checksum.
    """
    packet = (throttle << 1) | (1 if telemetry else 0) # shift left by 1 to make space for telemetry flag bit
    crc = ((packet ^ (packet >> 4) ^ (packet >> 8))) & 0x0F # unique fingerprint; 0x0F = 15 
    return (packet << 4) | crc # make room for crc bits 


####
# may need FGPA to handle the parallel latency bottleneck 
####
def send_dshot(throttle):

    # physical addresses of state machines transmitter FIFOs on Pico Chip 
    # Sequential 32-bit hardware slots inside the RP2040 chip
    SM0_TXFIFO = 0x50200010  # Motor on SM0 (GPIO 6)
    SM1_TXFIFO = 0x50200014  # Motor on SM1 (GPIO 5)
    SM2_TXFIFO = 0x50200018  # Motor on SM2 (GPIO 4)
    SM3_TXFIFO = 0x5020001c  # Motor on SM3 (GPIO 3)
    
    # make the dshot packet and send it simultenously to selected motors 
    packet = make_packet(throttle)
    dshot_packet = packet << 16 # shift 16 bits left from 32 to be read by sm
    mem32[SM0_TXFIFO] = dshot_packet 
    mem32[SM1_TXFIFO] = dshot_packet 
    mem32[SM2_TXFIFO] = dshot_packet 
    mem32[SM3_TXFIFO] = dshot_packet


def _dshot_ticker(timer):
    """
    Periodic timer callback used to maintain continuous DShot output.

    This function is invoked by a hardware timer and repeatedly
    transmits the current global throttle value. Continuous packet
    transmission is required to keep many ESCs armed and responsive.

    Args:
        timer (machine.Timer):
            Timer instance that triggered the callback.
    """
    send_dshot(dshot_throttle)

def setup_dshot_sm():
    """
    Initialize and reset all DShot PIO state machines.

    Creates four state machines, one per motor output pin, and
    configures each to execute the DShot PIO program at the
    selected DShot clock frequency.

    Returns:
        None

    Notes:
        - State machine 0 controls GPIO 6.
        - State machine 1 controls GPIO 5.
        - State machine 2 controls GPIO 4.
        - State machine 3 controls GPIO 3.
        - A frequency of 4.8 MHz corresponds to DShot600 timing.
    """

    # 8 cycles per bit @ 4.8 MHz = DShot600 (1.67 microseconds per bit)
    # 8 cycles per bit @ 2.4 MHz = DShot300 (3.33 microseconds per bit)
    DSHOT_FREQUENCY = 4_800_000  
    state_machines = [
        rp2.StateMachine(
            0, # 0th state machine 
            dshot, # the sequence of actions to run
            freq=DSHOT_FREQUENCY, 
            sideset_base=Pin(MOTOR_PINS[0])), 
        rp2.StateMachine(
            1, 
            dshot,
            freq=DSHOT_FREQUENCY,
            sideset_base=Pin(MOTOR_PINS[1])), 
        rp2.StateMachine(
            2, 
            dshot,
            freq=DSHOT_FREQUENCY,
            sideset_base=Pin(MOTOR_PINS[2])), 
        rp2.StateMachine(
            3,
            dshot,
            freq=DSHOT_FREQUENCY,
            sideset_base=Pin(MOTOR_PINS[3]))
        ]
    # reset state machines to ensure no packets are sent to ESCs yet 
    for sm in state_machines:
            sm.restart()
   
def main():
    """
    Entry point for ESC initialization and motor testing.

    This function performs the following steps:

        1. Initializes all DShot state machines.
        2. Starts a periodic 1 kHz transmission timer.
        3. Activates all PIO state machines simultaneously.
        4. Arms connected ESCs at zero throttle.
        5. Gradually ramps throttle for testing.
        6. Returns motors to zero throttle.

    The continuous timer-based DShot transmission ensures ESCs
    remain armed even while the main application performs other
    operations.

    Raises:
        KeyboardInterrupt:
            Triggers an emergency stop sequence when execution
            is interrupted by the user.
    """

    global dshot_throttle
    dshot_throttle = 0

    ##### Initialize Timer ####
    dshot_timer = machine.Timer()

    # state machines ready to work 
    setup_dshot_sm()
    
    # initialize 1kHz dshot timer to synchronize each packet 
    dshot_timer.init(freq=1000, 
                     mode = machine.Timer.PERIODIC,
                     callback=_dshot_ticker)

    # activate all state machines at once 
    machine.mem32[0x50200000] = 0b1111

    #####################################
    ### Arm the ESCs  ###
    # Uses the state machine, then dshot()  
    # for sending pulses to ESC         
    #####################################

    print("Arming ESC for 2 seconds")
    try:
        for _ in range(200):
            dshot_throttle = 0 # test for 200 milliseconds = 2 seconds 
            time.sleep_ms(10) # 1s delay

        print("Armed: ")

        print("------------------------------")
        
        repeat_counter = 48
        MAX_THROTTLE = 200
        STEP_SIZE = 1
        while (repeat_counter < MAX_THROTTLE):
            
            throttle_percent = ((repeat_counter - 48) / (2047 - 48)) * 100
            print(f"Incrementing throttle: {throttle_percent} % ")
            # send_dshot(repeat_counter)
            dshot_throttle = repeat_counter
            time.sleep_ms(50)
            
            repeat_counter += STEP_SIZE 

        print("------------------------------")

        while True: 
            machine.mem32[0x50200000] = 0b0000
        
        # Bring them down safely
        print("Ramp complete.")
        # for _ in range(100):
        #     send_dshot(0)
        #     time.sleep_ms(10)

    except KeyboardInterrupt:
        print("!!!EMERGENCY STOP!!!")

if __name__  == '__main__' :
    main()

# GOAL: 1. Motors should be controlled by the transmitter, so function that takes in data from transmitter and adjusts appropriate motors 
# 2. Activate desired motors based on user input, will be helpful for transitioning to transmitter side of things 
# 3. Stability test with throttle 
# 4. Integrate VLA
#   4a. Vision - 360 Lidar for terrain mapping 
#   4b. Language - voice synthesis with Whispr Flow or any conversational AI built in 
#   4c. Action - based on language and vision, AI on board takes the approriate move 














# FOR TIME_INSTRUCTIONS: ignore first loop - it is setup of first loop 

# FOR DEBUGGING PICO DATA PINS AND MOTORS: 
# p = [Pin(MOTOR_PINS[0], Pin.OUT), Pin(MOTOR_PINS[1], Pin.OUT), Pin(MOTOR_PINS[2], Pin.OUT), Pin(MOTOR_PINS[3], Pin.OUT)]
# while True: 
#     p[3].high()
#     time.sleep_ms(500)
#     p[3].low()
#     time.sleep_ms(500)
#     p[0].high()
#     time.sleep_ms(500)
#     p[0].low()
#     time.sleep_ms(500)
#     p[1].high()
#     time.sleep_ms(500)
#     p[1].low()
#     time.sleep_ms(500)
#     p[2].high()
#     time.sleep_ms(500)
#     p[2].low()
#     time.sleep_ms(500)



#####################################
####    REFERENCES (PIO)          ###
####   Assembly instructions      ###
#####################################

# Instruction  | What it does
# -------------|--------------------------------------------------------------
# pull         | Fetches the next packet from Python into the PIO holding buffer (FIFO)
# block        | Modifier for pull - Nothing transmitted until a real packet arrives
# noblock      | Modifier for pull - Reuse last packet if no new one is ready
# .side(0)     | Simultaneously sets ESC/GPIO signal wire LOW during the instruction (important!)
# .side(1)     | Simultaneously sets ESC/GPIO signal wire HIGH during the instruction (important!)
# jmp          | Jump to a label, can be conditional e.g. jmp(not_y, "zero")
# nop          | No Operation - waste exactly one clock cycle to control timing
# label        | Marks a named spot in code that jmp can jump back to
# set          | Load a number into a register e.g. set(x, 15) sets counter to 15
# out          | Shift bits from holding buffer into a register e.g. out(y, 1)
# active       | Sets or indicates whether state machine is on or not 

# Functions: 
    # dshot()
        # is the middleman toolchain that:
        # Pulls the packet out of the CPU queue
        # Breaks it apart bit by bit
        # Translates each bit into precise electrical HIGH/LOW timing on the wire
        # Repeats until all 16 bits are delivered


# Visuals:
    # Visual 1.1: Dshot() 
        # 1. Pull 16-bit packet from CPU queue
           # ↓
        # 2. If no new packet, reuse last one
            # ↓
        # 3. Set counter to 15 (will count down to 0 = 16 bits total)
           #  ↓
        # 4. Pull ESC wire HIGH (start of bit)
           #  ↓
        # 5. Shift 1 bit into Y register
           #  ↓
        # 6. Was that bit a 1 or 0?
           #   /        \
        #   bit=1       bit=0
       #   stay HIGH   go LOW
       #   longer      sooner
       #      \        /
       # 7. Counter - 1, loop back to step 4
           #     ↓
        # 8. Counter hit 0? All 16 bits sent → done
        
    # Visual 1.2: Dshot()
    ####### 
        # Pico RP2040 CPU
        #         ↓
        # runs make_packet()      ← Pico CPU does this math
        #         ↓
        # sm.put(packet << 16)    ← Pico CPU pushes to FIFO
        #         ↓
        # PIO state machine       ← Pico PIO takes over
        #         ↓
        # .side() toggles wire    ← physical signal to ESC
        #         ↓
        # ESC receives DShot
        #         ↓
        # Motor spins 
    ####### 

    # Visual 1.3: Dshot()
    #######
    # 1 DShot packet
        #├── 16 bits
        #│   ├── 11 bits = throttle value (0–2047)
        #│   ├── 1 bit  = telemetry flag
        #│   └── 4 bits = CRC checksum
        #│
        #└── each bit = 8 PIO cycles (.side() commands)
                    #├── 2 cycles HIGH + 6 cycles LOW = bit "0" (for 8 cycles, different for 5 cycles)
                    #└── 6 cycles HIGH + 2 cycles LOW = bit "1"


# Terminology
    # CRC
        # stands for Cyclic Redundancy Check— it's a checksum, basically a fingerprint of your packet.
        # prevents making corrupted packets of data, so a unique fingerprint is assigned
    # Telemetry
        # a request flag to send back all data about motors
    # ^
        # XOR operation: Exclusive OR
        # Output is 1 if the bits are DIFFERENT, 0 if they are the SAME

# Important Explanations
    # 1. This means if the ESC receives the packet and XORs the same chunks together,
    # it should get the same CRC fingerprint back. If even one bit changed during
    # transmission, the fingerprint won't match and the ESC knows something went wrong.


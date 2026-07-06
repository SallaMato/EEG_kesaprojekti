import asyncio
import struct
import time
import math
from collections import deque

from bleak import BleakScanner
from bleak import BleakClient

# ==========================================================
# BLE
# ==========================================================

DEVICE_NAME = "OpenEEG"

SERVICE_UUID = "0000FFF0-0000-1000-8000-00805F9B34FB"
CHAR_UUID    = "0000FFF1-0000-1000-8000-00805F9B34FB"

# ==========================================================
# ADC
# ==========================================================

VREF = 1.2
GAIN = 4
FULL_SCALE = 8388608

EXPECTED_SPS = 250

WINDOW_SECONDS = 4

MAINS_FREQ = 50.0

# ==========================================================
# Buffers
# ==========================================================

packet_buffer = deque()

max_points = EXPECTED_SPS * WINDOW_SECONDS

time_buffer = deque(maxlen=max_points)

ch = [
    deque(maxlen=max_points)
    for _ in range(8)
]

# ==========================================================
# Statistics
# ==========================================================

received_packets = 0
bad_packets = 0

first_time = None

# ==========================================================
# Helpers
# ==========================================================

def raw_to_uv(raw):

    return (
        raw
        * VREF
        / GAIN
        / FULL_SCALE
        * 1_000_000
    )

def remove_mean(values):

    if len(values) == 0:
        return []

    m = sum(values) / len(values)

    return [
        x - m
        for x in values
    ]

def peak_to_peak(values):

    if len(values) == 0:
        return 0

    return max(values) - min(values)

def rms(values):

    if len(values) == 0:
        return 0

    return math.sqrt(

        sum(

            x * x

            for x in values

        ) / len(values)

    )

# ==========================================================
# Find OpenEEG
# ==========================================================

async def find_device():

    print("Scanning for OpenEEG...\n")

    devices = await BleakScanner.discover(timeout=5)

    for d in devices:

        if d.name == DEVICE_NAME:

            print(f"Found: {d.name}")

            return d

    return None

# ==========================================================
# BLE Notification
# ==========================================================

def notification_handler(sender, data):

    global received_packets
    global bad_packets

    #
    # Packet format:
    #
    # uint32 sample
    # int32 ch1
    # int32 ch2
    # ...
    # int32 ch8
    #

    if len(data) != 36:

        bad_packets += 1

        return

    packet = struct.unpack("<I8i", data)

    packet_buffer.append(packet)

    received_packets += 1

# ==========================================================
# Connect
# ==========================================================

async def connect():

    device = await find_device()

    if device is None:

        print("OpenEEG not found.")

        return None

    print("Connecting...\n")

    client = BleakClient(device)

    await client.connect()

    print("Connected.\n")

    await client.start_notify(

        CHAR_UUID,

        notification_handler

    )

    return client

# ==========================================================
# Store packet into analysis buffers
# ==========================================================

def process_packet(packet):

    global first_time

    sample = packet[0]

    channels = packet[1:]

    if first_time is None:

        first_time = time.time()

    t = sample / EXPECTED_SPS

    time_buffer.append(t)

    for i in range(8):

        ch[i].append(

            raw_to_uv(channels[i])

        )

# ==========================================================
# Analysis helpers
# ==========================================================

def estimate_sps(times):

    if len(times) < 2:
        return None

    dt = times[-1] - times[0]

    if dt <= 0:
        return None

    return (len(times) - 1) / dt

def sine_component_vpp(times, values, freq):

    values = remove_mean(values)

    n = len(values)

    if n < 20:
        return None

    re = 0.0
    im = 0.0

    for t, y in zip(times, values):

        angle = 2.0 * math.pi * freq * t

        re += y * math.cos(angle)
        im -= y * math.sin(angle)

    peak = (2.0 / n) * math.sqrt(re * re + im * im)

    return 2.0 * peak

def dominant_frequency(times, values):

    values = remove_mean(values)

    best_freq = 0
    best_power = 0

    freq = 0.5

    while freq <= 100:

        re = 0.0
        im = 0.0

        for t, y in zip(times, values):

            angle = 2.0 * math.pi * freq * t

            re += y * math.cos(angle)
            im -= y * math.sin(angle)

        power = re * re + im * im

        if power > best_power:

            best_power = power
            best_freq = freq

        freq += 0.5

    return best_freq

def channel_text(name,
                 times,
                 values):

    values_ac = remove_mean(values)

    total = peak_to_peak(values_ac)

    rms_uv = rms(values_ac)

    dom = dominant_frequency(

        times,

        values_ac

    )

    signal = sine_component_vpp(

        times,

        values_ac,

        dom

    )

    mains = sine_component_vpp(

        times,

        values_ac,

        MAINS_FREQ

    )

    return (

        f"{name}: "

        f"dom={dom:.1f} Hz, "

        f"total={total/1000:.3f} mVpp, "

        f"@{dom:.1f}Hz={signal/1000:.3f} mVpp, "

        f"@50Hz={mains/1000:.3f} mVpp, "

        f"rms={rms_uv:.1f} µVrms"

    )

# ==========================================================
# Print analyzer output
# ==========================================================

def print_statistics():

    if len(time_buffer) < EXPECTED_SPS:

        return

    times = list(time_buffer)

    sps = estimate_sps(times)

    print()

    print(

        f"SPS={sps:.1f}"

    )

    print(

        channel_text(

            "CH1",

            times,

            list(ch[0])

        )

    )

    print(

        channel_text(

            "CH2",

            times,

            list(ch[1])

        )

    )

    print()

# ==========================================================
# Main
# ==========================================================

async def main():

    client = await connect()

    if client is None:
        return

    print("Receiving EEG data...\n")

    last_print = time.time()

    try:

        while True:

            #
            # Process every received packet
            #
            while packet_buffer:

                packet = packet_buffer.popleft()

                process_packet(packet)

            #
            # Print once per second
            #
            now = time.time()

            if now - last_print >= 1.0:

                last_print = now

                print_statistics()

                print(
                    f"Packets={received_packets}   "
                    f"Bad={bad_packets}"
                )

            await asyncio.sleep(0.01)

    except KeyboardInterrupt:

        print("\nStopping...")

    finally:

        try:
            await client.stop_notify(CHAR_UUID)
        except Exception:
            pass

        try:
            await client.disconnect()
        except Exception:
            pass

        print("Disconnected.")

# ==========================================================
# Entry point
# ==========================================================

if __name__ == "__main__":

    asyncio.run(main())
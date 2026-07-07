import asyncio
import struct
import time
from collections import deque

import numpy as np
#import matplotlib.pyplot as plt

from bleak import BleakScanner
from bleak import BleakClient



# ==========================================================
# BLE
# ==========================================================

DEVICE_NAME = "OpenEEG"

SERVICE_UUID = "0000FFF0-0000-1000-8000-00805F9B34FB"
CHAR_UUID    = "0000FFF1-0000-1000-8000-00805F9B34FB"



# ==========================================================
# ADS131M08
# ==========================================================

EXPECTED_SPS = 250

VREF = 1.2
GAIN = 4

ADC_FS = 8388608.0



# ==========================================================
# Plot
# ==========================================================

WINDOW_SECONDS = 4

BUFFER_SIZE = EXPECTED_SPS * WINDOW_SECONDS



# ==========================================================
# Buffers
# ==========================================================

sample_buffer = deque(maxlen=BUFFER_SIZE)

ch2_buffer = deque(maxlen=BUFFER_SIZE)
ch3_buffer = deque(maxlen=BUFFER_SIZE)

arrival_times = deque(maxlen=BUFFER_SIZE)

# ==========================================================
# Statistics
# ==========================================================

received_packets = 0
bad_packets = 0

def raw_to_uv(raw):

    return (
        raw
        * VREF
        / GAIN
        / ADC_FS
        * 1_000_000.0
    )

# ==========================================================
# BLE notification
# ==========================================================

def notification_handler(sender, data):

    print("RX", len(data))

    global received_packets
    global bad_packets

    # Yksi näyte on 36 tavua. Koska ESP lähettää useamman näytteen putkeen,
    # datan pituuden pitää olla jaollinen 36:lla.
    if len(data) % 36 != 0:
        bad_packets += 1
        return

    # Käydään data läpi 36 tavun pätkissä
    for offset in range(0, len(data), 36):
        single_sample_bytes = data[offset : offset + 36]
        
        try:
            packet = struct.unpack("<I8i", single_sample_bytes)

            sample = packet[0]
            ch2_raw = packet[1]
            ch3_raw = packet[2]

            # Tallennetaan näytteet ja reaaliaika puskureihin
            sample_buffer.append(sample)
            arrival_times.append(time.time()) 
            
            ch2_buffer.append(raw_to_uv(ch2_raw))
            ch3_buffer.append(raw_to_uv(ch3_raw))

            received_packets += 1

        except struct.error:
            bad_packets += 1
            return

    if received_packets % 250 == 0:
        print("Packets received:", received_packets)

# ==========================================================
# Device search
# ==========================================================

async def find_device():

    print("Scanning for OpenEEG...")

    device = await BleakScanner.find_device_by_name(
        DEVICE_NAME,
        timeout=10.0,
    )

    return device

# ==========================================================
# Analysis
# ==========================================================

def remove_mean(x):

    x = np.asarray(x)

    return x - np.mean(x)


def rms(x):

    x = np.asarray(x)

    return np.sqrt(np.mean(x * x))


def peak_to_peak(x):

    x = np.asarray(x)

    return np.max(x) - np.min(x)   


def fft_analysis(values, sps):

    values = remove_mean(values)

    n = len(values)

    if n < 32:
        return None

    spectrum = np.fft.rfft(values)

    freqs = np.fft.rfftfreq(n, d=1.0 / sps)

    magnitude = np.abs(spectrum) * 2.0 / n

    return freqs, magnitude

def dominant_frequency(values, sps):

    result = fft_analysis(values, sps)

    if result is None:
        return None, None

    freqs, mag = result

    #
    # Ohitetaan DC
    #

    mag[0] = 0

    idx = np.argmax(mag)

    return freqs[idx], mag[idx] * 2.0

def amplitude_at(values, sps, target_freq):

    result = fft_analysis(values, sps)

    if result is None:
        return None

    freqs, mag = result

    idx = np.argmin(np.abs(freqs - target_freq))

    return mag[idx] * 2.0

def estimate_sps(times):
    if len(times) < 2:
        return EXPECTED_SPS
    duration = times[-1] - times[0]
    if duration <= 0:
        return EXPECTED_SPS
    return (len(times) - 1) / duration

# ==========================================================
# Plot
# ==========================================================
'''
plt.ion()

fig, ax = plt.subplots(figsize=(12, 5))

line_ch2, = ax.plot([], [], label="CH2")
line_ch3, = ax.plot([], [], label="CH3")

ax.set_xlabel("Samples")
ax.set_ylabel("uV")

ax.set_title("OpenEEG BLE")

ax.grid(True)

ax.legend()

fig.tight_layout()

def update_plot():

    if len(ch2_buffer) < 20:
        return

    x = np.arange(len(ch2_buffer))

    y2 = np.asarray(ch2_buffer)
    y3 = np.asarray(ch3_buffer)

    line_ch2.set_data(x, y2)
    line_ch3.set_data(x, y3)

    ax.set_xlim(0, len(x))

    ymin = min(np.min(y2), np.min(y3))
    ymax = max(np.max(y2), np.max(y3))

    margin = max((ymax - ymin) * 0.10, 50)

    ax.set_ylim(
        ymin - margin,
        ymax + margin
    )

    fig.canvas.draw_idle()
    fig.canvas.flush_events()

    plt.pause(0.001)'''

# ==========================================================
# Statistics print
# ==========================================================

last_print = time.time()


def print_statistics():

    global last_print

    if len(ch2_buffer) < BUFFER_SIZE:
        return

    now = time.time()

    if now - last_print < 1.0:
        return

    last_print = now

    sps = estimate_sps(arrival_times)

    ch2 = np.asarray(ch2_buffer)
    ch3 = np.asarray(ch3_buffer)

    print(
    "DEBUG:",
    ch2[0],
    ch2[1],
    ch2[2],
    ch2[3],
    ch2[4]
    )

    print(
        "MIN MAX:",
        np.min(ch2),
        np.max(ch2)
    )

    #
    # CH2
    #

    dom2, _ = dominant_frequency(ch2, sps)

    total2 = peak_to_peak(ch2)

    sig2 = amplitude_at(ch2, sps, dom2)

    hum2 = amplitude_at(ch2, sps, 50.0)

    rms2 = rms(ch2)

    #
    # CH3
    #

    dom3, _ = dominant_frequency(ch3, sps)

    total3 = peak_to_peak(ch3)

    sig3 = amplitude_at(ch3, sps, dom3)

    hum3 = amplitude_at(ch3, sps, 50.0)

    rms3 = rms(ch3)

    print()

    print(f"SPS={sps:.1f}")

    print(
        f"CH2: "
        f"dom={dom2:.1f} Hz, "
        f"total={total2/1000:.3f} mVpp, "
        f"@{dom2:.1f}Hz={sig2/1000:.3f} mVpp, "
        f"@50Hz={hum2/1000:.3f} mVpp, "
        f"rms={rms2:.1f} µVrms"
    )

    print(
        f"CH3: "
        f"dom={dom3:.1f} Hz, "
        f"total={total3/1000:.3f} mVpp, "
        f"@{dom3:.1f}Hz={sig3/1000:.3f} mVpp, "
        f"@50Hz={hum3/1000:.3f} mVpp, "
        f"rms={rms3:.1f} µVrms"
    )

    print()

    print(
        f"Packets={received_packets}   "
        f"Bad={bad_packets}"
    )

# ==========================================================
# Main
# ==========================================================

async def main():

    device = await find_device()

    if device is None:
        print("OpenEEG not found")
        return

    print("Connecting...")

    async with BleakClient(device) as client:

        print("Connected")

        await client.start_notify(
            CHAR_UUID,
            notification_handler
        )

        print("Receiving data...")
        print()

        try:

            while True:

                await asyncio.sleep(0.02)

                #update_plot()

                print_statistics()

        except KeyboardInterrupt:

            print()

            print("Stopping...")

        finally:

            await client.stop_notify(CHAR_UUID)

    print("Disconnected")


if __name__ == "__main__":

    asyncio.run(main())
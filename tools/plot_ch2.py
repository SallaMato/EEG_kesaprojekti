import sys
from collections import deque

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


BAUDRATE = 115200

MAX_POINTS = 1000
PLOT_INTERVAL_MS = 200
MAX_LINES_PER_UPDATE = 200
AC_MEAN_WINDOW = 100

VREF = 1.2
PGA_GAIN = 4
ADC_FULL_SCALE = 8388608


def raw_to_uv(raw: int) -> float:
    return raw * VREF / ADC_FULL_SCALE / PGA_GAIN * 1_000_000


def parse_data_line(line: str):
    """
    Expected:
    DATA,sample,t_us,ch2_raw,ch3_raw
    """
    line = line.strip()

    if not line.startswith("DATA,"):
        return None

    parts = line.split(",")

    if len(parts) != 5:
        return None

    try:
        sample = int(parts[1])
        t_us = int(parts[2])
        ch2_raw = int(parts[3])
        ch3_raw = int(parts[4])
        return sample, t_us, ch2_raw, ch3_raw
    except ValueError:
        return None


def moving_ac(values, window):
    if len(values) < 2:
        return 0.0

    recent = list(values)[-window:]
    mean_value = sum(recent) / len(recent)
    return values[-1] - mean_value

def estimate_signal_stats(times_s, values_uv):
    """
    Estimates frequency and peak-to-peak amplitude from the visible window.
    Very simple zero-crossing based estimate.
    """
    if len(times_s) < 10 or len(values_uv) < 10:
        return None, None

    times = list(times_s)
    values = list(values_uv)

    v_min = min(values)
    v_max = max(values)
    v_pp = v_max - v_min

    # Zero-crossings with positive slope
    crossings = []

    for i in range(1, len(values)):
        if values[i - 1] < 0 and values[i] >= 0:
            t0 = times[i - 1]
            t1 = times[i]
            y0 = values[i - 1]
            y1 = values[i]

            if y1 != y0:
                # Linear interpolation for a better crossing time
                frac = -y0 / (y1 - y0)
                t_cross = t0 + frac * (t1 - t0)
            else:
                t_cross = t1

            crossings.append(t_cross)

    if len(crossings) < 2:
        return None, v_pp

    periods = []
    for i in range(1, len(crossings)):
        periods.append(crossings[i] - crossings[i - 1])

    avg_period = sum(periods) / len(periods)

    if avg_period <= 0:
        return None, v_pp

    freq_hz = 1.0 / avg_period

    return freq_hz, v_pp

def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  py tools/plot_ch2.py COM_PORT")
        print("")
        print("Example:")
        print("  py tools/plot_ch2.py COM4")
        sys.exit(1)

    port = sys.argv[1]

    print(f"Opening serial port {port} at {BAUDRATE} baud...")
    ser = serial.Serial(port=port, baudrate=BAUDRATE, timeout=0)
    ser.reset_input_buffer()

    # x-axis: aika sekunteina mittauksen alusta
    times_s = deque(maxlen=MAX_POINTS)
    first_t_us = None

    # raaka-arvoista muunnetut mikrovoltit
    ch2_uv_values = deque(maxlen=MAX_POINTS)
    ch3_uv_values = deque(maxlen=MAX_POINTS)

    # AC = mikrovolttiarvo, josta on poistettu liukuva keskiarvo
    ch2_ac_uv_values = deque(maxlen=MAX_POINTS)
    ch3_ac_uv_values = deque(maxlen=MAX_POINTS)

    fig, ax = plt.subplots()

    ch2_ac_line, = ax.plot([], [], label="CH2 AC µV")
    ch3_ac_line, = ax.plot([], [], label="CH3 AC µV")

    ax.set_title("ADS131M08 CH2 / CH3 AC signal")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Amplitude (µV)")
    ax.grid(True)
    ax.legend(loc="upper right")

    update_counter = 0

    def update(_frame):
        nonlocal update_counter, first_t_us

        lines_read = 0

        while ser.in_waiting > 0 and lines_read < MAX_LINES_PER_UPDATE:
            raw = ser.readline()
            lines_read += 1

            if not raw:
                break

            text = raw.decode("utf-8", errors="ignore")
            parsed = parse_data_line(text)

            if parsed is None:
                continue

            sample, t_us, ch2_raw, ch3_raw = parsed

            if first_t_us is None:
                first_t_us = t_us

            t_s = (t_us - first_t_us) / 1_000_000.0

            ch2_uv = raw_to_uv(ch2_raw)
            ch3_uv = raw_to_uv(ch3_raw)

            times_s.append(t_s)

            ch2_uv_values.append(ch2_uv)
            ch3_uv_values.append(ch3_uv)

            ch2_ac_uv_values.append(moving_ac(ch2_uv_values, AC_MEAN_WINDOW))
            ch3_ac_uv_values.append(moving_ac(ch3_uv_values, AC_MEAN_WINDOW))

        if len(times_s) < 2:
            return ch2_ac_line, ch3_ac_line

        ch2_ac_line.set_data(times_s, ch2_ac_uv_values)
        ch3_ac_line.set_data(times_s, ch3_ac_uv_values)

        update_counter += 1

        if update_counter % 5 == 0:
            xmin = times_s[0]
            xmax = times_s[-1]
            ax.set_xlim(xmin, xmax)

            all_values = list(ch2_ac_uv_values) + list(ch3_ac_uv_values)

            ymin = min(all_values)
            ymax = max(all_values)

            if ymin == ymax:
                ymin -= 1
                ymax += 1

            margin = max(10, (ymax - ymin) * 0.1)
            ax.set_ylim(ymin - margin, ymax + margin)

            ch2_freq_hz, ch2_vpp_uv = estimate_signal_stats(times_s, ch2_ac_uv_values)
            ch3_freq_hz, ch3_vpp_uv = estimate_signal_stats(times_s, ch3_ac_uv_values)

            title = "ADS131M08 CH2 / CH3 AC signal"

            if ch2_freq_hz is not None and ch2_vpp_uv is not None:
                title += f" | CH2: {ch2_freq_hz:.2f} Hz, {ch2_vpp_uv / 1000:.2f} mVpp"

            if ch3_freq_hz is not None and ch3_vpp_uv is not None:
                title += f" | CH3: {ch3_freq_hz:.2f} Hz, {ch3_vpp_uv / 1000:.2f} mVpp"

            ax.set_title(title)

        return ch2_ac_line, ch3_ac_line

    ani = FuncAnimation(
        fig,
        update,
        interval=PLOT_INTERVAL_MS,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.show()
    finally:
        ser.close()
        print("Serial port closed.")


if __name__ == "__main__":
    main()
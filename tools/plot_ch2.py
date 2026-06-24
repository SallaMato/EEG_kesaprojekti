import sys
from collections import deque

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


BAUDRATE = 115200

MAX_POINTS = 300
PLOT_INTERVAL_MS = 100
MAX_LINES_PER_UPDATE = 30
AC_MEAN_WINDOW = 100


def parse_data_line(line: str):
    """
    Expected:
    DATA,sample,ch2_raw,ch3_raw
    """
    line = line.strip()

    if not line.startswith("DATA,"):
        return None

    parts = line.split(",")

    if len(parts) != 4:
        return None

    try:
        sample = int(parts[1])
        ch2 = int(parts[2])
        ch3 = int(parts[3])
        return sample, ch2, ch3
    except ValueError:
        return None


def moving_ac(values, window):
    if len(values) < 2:
        return 0

    recent = list(values)[-window:]
    mean_value = sum(recent) / len(recent)
    return values[-1] - mean_value


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  py tools/plot_ch2_ch3.py COM_PORT")
        print("")
        print("Example:")
        print("  py tools/plot_ch2_ch3.py COM5")
        sys.exit(1)

    port = sys.argv[1]

    print(f"Opening serial port {port} at {BAUDRATE} baud...")
    ser = serial.Serial(port=port, baudrate=BAUDRATE, timeout=0)

    samples = deque(maxlen=MAX_POINTS)

    ch2_raw_values = deque(maxlen=MAX_POINTS)
    ch3_raw_values = deque(maxlen=MAX_POINTS)

    ch2_ac_values = deque(maxlen=MAX_POINTS)
    ch3_ac_values = deque(maxlen=MAX_POINTS)

    fig, ax = plt.subplots()

    ch2_raw_line, = ax.plot([], [], label="CH2 raw")
    ch3_raw_line, = ax.plot([], [], label="CH3 raw")
    ch2_ac_line, = ax.plot([], [], label="CH2 AC")
    ch3_ac_line, = ax.plot([], [], label="CH3 AC")

    ax.set_title("ADS131M08 CH2 / CH3")
    ax.set_xlabel("Sample")
    ax.set_ylabel("Raw code")
    ax.grid(True)
    ax.legend(loc="upper right")

    update_counter = 0

    def update(_frame):
        nonlocal update_counter

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

            sample, ch2, ch3 = parsed

            samples.append(sample)
            ch2_raw_values.append(ch2)
            ch3_raw_values.append(ch3)

            ch2_ac_values.append(moving_ac(ch2_raw_values, AC_MEAN_WINDOW))
            ch3_ac_values.append(moving_ac(ch3_raw_values, AC_MEAN_WINDOW))

        if len(samples) < 2:
            return ch2_raw_line, ch3_raw_line, ch2_ac_line, ch3_ac_line

        ch2_raw_line.set_data(samples, ch2_raw_values)
        ch3_raw_line.set_data(samples, ch3_raw_values)
        ch2_ac_line.set_data(samples, ch2_ac_values)
        ch3_ac_line.set_data(samples, ch3_ac_values)

        update_counter += 1

        if update_counter % 5 == 0:
            xmin = samples[0]
            xmax = samples[-1]
            ax.set_xlim(xmin, xmax)

            all_values = (
                list(ch2_raw_values)
                + list(ch3_raw_values)
                + list(ch2_ac_values)
                + list(ch3_ac_values)
            )

            ymin = min(all_values)
            ymax = max(all_values)

            if ymin == ymax:
                ymin -= 1
                ymax += 1

            margin = max(1000, int((ymax - ymin) * 0.1))
            ax.set_ylim(ymin - margin, ymax + margin)

        return ch2_raw_line, ch3_raw_line, ch2_ac_line, ch3_ac_line

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
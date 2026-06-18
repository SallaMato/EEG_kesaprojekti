import sys
import time
from collections import deque

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


BAUDRATE = 115200
MAX_POINTS = 500


def parse_data_line(line: str):
    """
    Expected line:
    DATA,sample,ch2_raw
    """
    line = line.strip()

    if not line.startswith("DATA,"):
        return None

    parts = line.split(",")

    if len(parts) != 3:
        return None

    try:
        sample = int(parts[1])
        ch2 = int(parts[2])
        return sample, ch2
    except ValueError:
        return None


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python tools/plot_ch2.py COM4")
        print("")
        print("Example:")
        print("  python tools/plot_ch2.py COM4")
        sys.exit(1)

    port = sys.argv[1]

    print(f"Opening serial port {port} at {BAUDRATE} baud...")
    ser = serial.Serial(port=port, baudrate=BAUDRATE, timeout=1)

    samples = deque(maxlen=MAX_POINTS)
    values = deque(maxlen=MAX_POINTS)

    fig, ax = plt.subplots()
    line_plot, = ax.plot([], [])

    ax.set_title("ADS131M08 CH2 raw data")
    ax.set_xlabel("Sample")
    ax.set_ylabel("CH2 raw code")
    ax.grid(True)

    def update(_frame):
        # Read as many waiting lines as available, but do not block forever.
        for _ in range(50):
            raw = ser.readline()

            if not raw:
                break

            try:
                text = raw.decode("utf-8", errors="ignore")
            except UnicodeDecodeError:
                continue

            parsed = parse_data_line(text)

            if parsed is None:
                continue

            sample, ch2 = parsed
            samples.append(sample)
            values.append(ch2)

        if len(samples) > 1:
            line_plot.set_data(samples, values)

            ax.set_xlim(min(samples), max(samples))

            ymin = min(values)
            ymax = max(values)

            if ymin == ymax:
                ymin -= 1
                ymax += 1

            margin = max(1000, int((ymax - ymin) * 0.1))
            ax.set_ylim(ymin - margin, ymax + margin)

        return line_plot,

    ani = FuncAnimation(fig, update, interval=50, blit=False)

    try:
        plt.show()
    finally:
        ser.close()
        print("Serial port closed.")


if __name__ == "__main__":
    main()
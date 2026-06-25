import sys
import time
import serial


DEFAULT_BAUDRATE = 115200


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
        ch2 = int(parts[3])
        ch3 = int(parts[4])
        return sample, t_us, ch2, ch3
    except ValueError:
        return None


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  py tools/check_stream_rate.py COM_PORT [BAUDRATE]")
        print("")
        print("Example:")
        print("  py tools/check_stream_rate.py COM4")
        print("  py tools/check_stream_rate.py COM4 115200")
        print("  py tools/check_stream_rate.py COM4 921600")
        sys.exit(1)

    port = sys.argv[1]
    baudrate = int(sys.argv[2]) if len(sys.argv) >= 3 else DEFAULT_BAUDRATE

    print(f"Opening {port} at {baudrate} baud...")
    ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.2)
    ser.reset_input_buffer()

    print("Counting DATA lines. Press Ctrl+C to stop.")

    total_lines = 0
    data_lines = 0
    parsed_lines = 0
    parse_errors = 0

    last_print_time = time.time()
    last_parsed_lines = 0
    last_sample = None
    last_t_us = None

    try:
        while True:
            raw = ser.readline()

            if not raw:
                now = time.time()
                if now - last_print_time >= 1.0:
                    print("No data...")
                    last_print_time = now
                continue

            total_lines += 1

            text = raw.decode("utf-8", errors="ignore").strip()

            if not text.startswith("DATA,"):
                continue

            data_lines += 1
            parsed = parse_data_line(text)

            if parsed is None:
                parse_errors += 1
                continue

            sample, t_us, ch2, ch3 = parsed
            parsed_lines += 1

            now = time.time()

            if now - last_print_time >= 1.0:
                elapsed = now - last_print_time
                python_sps = (parsed_lines - last_parsed_lines) / elapsed

                if last_sample is not None and last_t_us is not None:
                    sample_delta = sample - last_sample
                    t_delta_s = (t_us - last_t_us) / 1_000_000.0

                    if t_delta_s > 0:
                        device_sps = sample_delta / t_delta_s
                    else:
                        device_sps = 0.0
                else:
                    device_sps = 0.0

                print(
                    f"parsed={parsed_lines} "
                    f"python_sps={python_sps:.1f} "
                    f"device_sps={device_sps:.1f} "
                    f"sample={sample} "
                    f"ch2={ch2} "
                    f"ch3={ch3} "
                    f"errors={parse_errors}"
                )

                last_print_time = now
                last_parsed_lines = parsed_lines
                last_sample = sample
                last_t_us = t_us

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        ser.close()
        print("Serial port closed.")


if __name__ == "__main__":
    main()
import sys
import time
import math
import serial
from collections import deque


DEFAULT_BAUDRATE = 115200

# Vastaa tämänhetkistä ESP-asetusta:
# ADS_STREAM_PERIOD_US 4000 = 250 SPS
EXPECTED_SPS = 250

# Kuinka pitkä analyysi-ikkuna pidetään muistissa sekunteina.
WINDOW_SECONDS = 4.0

# ADS / muunnos
VREF = 1.2
PGA_GAIN = 4
ADC_FULL_SCALE = 8388608

# Oletuksena tutkitaan generaattorin 2 Hz signaalia.
# Voit vaihtaa komentoriviltä esim:
#   py tools/analyze_stream.py COM4 115200 10
DEFAULT_TARGET_FREQ_HZ = 2.0

# Suomessa verkkohäiriö on 50 Hz.
MAINS_FREQ_HZ = 50.0

# Dominantin taajuuden haku
FREQ_MIN_HZ = 0.5
FREQ_MAX_HZ = 100.0
FREQ_STEP_HZ = 0.5


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


def remove_mean(values):
    if not values:
        return []

    mean_value = sum(values) / len(values)
    return [v - mean_value for v in values]


def estimate_sps(times_s):
    if len(times_s) < 2:
        return None

    duration = times_s[-1] - times_s[0]

    if duration <= 0:
        return None

    return (len(times_s) - 1) / duration


def peak_to_peak(values):
    if not values:
        return None

    return max(values) - min(values)


def rms(values):
    if not values:
        return None

    return math.sqrt(sum(v * v for v in values) / len(values))


def sine_component_vpp(times_s, values_uv, freq_hz):
    """
    Arvioi tietyn taajuuskomponentin peak-to-peak amplitudin.

    Tämä ei ole suodatin. Tämä vain kysyy:
    "Kuinka suuri osa signaalista näyttää olevan juuri tällä taajuudella?"

    Palauttaa arvon mikrovoltteina peak-to-peak.
    """
    if len(times_s) < 20 or len(values_uv) < 20:
        return None

    values = remove_mean(values_uv)
    n = len(values)

    re = 0.0
    im = 0.0

    for t, y in zip(times_s, values):
        angle = 2.0 * math.pi * freq_hz * t
        re += y * math.cos(angle)
        im -= y * math.sin(angle)

    # Sinin peak-amplitudi.
    peak_uv = (2.0 / n) * math.sqrt(re * re + im * im)

    # Peak-to-peak = 2 * peak.
    return 2.0 * peak_uv


def dominant_frequency_dft(times_s, values_uv):
    """
    Yksinkertainen DFT-haku.
    Tämä kertoo, mikä taajuus on vahvin näkyvässä ikkunassa.
    """
    if len(times_s) < 20 or len(values_uv) < 20:
        return None, None

    values = remove_mean(values_uv)

    best_freq = None
    best_power = 0.0

    freq = FREQ_MIN_HZ

    while freq <= FREQ_MAX_HZ:
        re = 0.0
        im = 0.0

        for t, y in zip(times_s, values):
            angle = 2.0 * math.pi * freq * t
            re += y * math.cos(angle)
            im -= y * math.sin(angle)

        power = re * re + im * im

        if power > best_power:
            best_power = power
            best_freq = freq

        freq += FREQ_STEP_HZ

    return best_freq, best_power


def print_channel_stats(name, times_s, values_uv, target_freq_hz):
    values_ac = remove_mean(values_uv)

    total_vpp_uv = peak_to_peak(values_ac)
    rms_uv = rms(values_ac)

    target_vpp_uv = sine_component_vpp(times_s, values_ac, target_freq_hz)
    mains_vpp_uv = sine_component_vpp(times_s, values_ac, MAINS_FREQ_HZ)

    dominant_hz, _ = dominant_frequency_dft(times_s, values_ac)

    if total_vpp_uv is None:
        return f"{name}: not enough data"

    return (
        f"{name}: "
        f"dom={dominant_hz:.1f} Hz, "
        f"total={total_vpp_uv / 1000:.3f} mVpp, "
        f"@{target_freq_hz:.1f}Hz={target_vpp_uv / 1000:.3f} mVpp, "
        f"@50Hz={mains_vpp_uv / 1000:.3f} mVpp, "
        f"rms={rms_uv:.1f} µVrms"
    )


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  py tools/analyze_stream.py COM_PORT [BAUDRATE] [TARGET_FREQ_HZ]")
        print("")
        print("Examples:")
        print("  py tools/analyze_stream.py COM4")
        print("  py tools/analyze_stream.py COM4 115200")
        print("  py tools/analyze_stream.py COM4 115200 2")
        print("  py tools/analyze_stream.py COM4 115200 10")
        sys.exit(1)

    port = sys.argv[1]
    baudrate = int(sys.argv[2]) if len(sys.argv) >= 3 else DEFAULT_BAUDRATE
    target_freq_hz = float(sys.argv[3]) if len(sys.argv) >= 4 else DEFAULT_TARGET_FREQ_HZ

    print(f"Opening {port} at {baudrate} baud...")
    print(f"Target frequency: {target_freq_hz:.1f} Hz")
    print(f"PGA_GAIN used for conversion: {PGA_GAIN}")
    print("Press Ctrl+C to stop.")

    ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.2)
    ser.reset_input_buffer()

    max_points = int(EXPECTED_SPS * WINDOW_SECONDS * 1.5)

    times_s = deque(maxlen=max_points)
    ch2_uv = deque(maxlen=max_points)
    ch3_uv = deque(maxlen=max_points)

    first_t_us = None

    parsed_lines = 0
    errors = 0
    last_print_time = time.time()

    try:
        while True:
            raw = ser.readline()

            if not raw:
                continue

            text = raw.decode("utf-8", errors="ignore")
            parsed = parse_data_line(text)

            if parsed is None:
                if text.startswith("DATA,"):
                    errors += 1
                continue

            sample, t_us, ch2_raw, ch3_raw = parsed

            if first_t_us is None:
                first_t_us = t_us

            t_s = (t_us - first_t_us) / 1_000_000.0

            times_s.append(t_s)
            ch2_uv.append(raw_to_uv(ch2_raw))
            ch3_uv.append(raw_to_uv(ch3_raw))

            parsed_lines += 1

            now = time.time()

            if now - last_print_time >= 1.0:
                last_print_time = now

                if len(times_s) < EXPECTED_SPS:
                    print(f"Collecting... parsed={parsed_lines}, errors={errors}")
                    continue

                times_list = list(times_s)

                sps = estimate_sps(times_list)

                ch2_text = print_channel_stats(
                    "CH2",
                    times_list,
                    list(ch2_uv),
                    target_freq_hz,
                )

                ch3_text = print_channel_stats(
                    "CH3",
                    times_list,
                    list(ch3_uv),
                    target_freq_hz,
                )

                print(
                    f"SPS={sps:.1f} | "
                    f"{ch2_text} | "
                    f"{ch3_text} | "
                    f"errors={errors}"
                )

    except KeyboardInterrupt:
        print("\nStopped.")

    except serial.SerialException as exc:
        print("")
        print("Serial port error:")
        print(exc)
        print("")
        print("Jos tämä tapahtui kun laitoit generaattorin output OFF,")
        print("ESP todennäköisesti resetoi tai USB-yhteys katkesi transientin takia.")

    finally:
        try:
            ser.close()
        except Exception:
            pass

        print("Serial port closed.")


if __name__ == "__main__":
    main()
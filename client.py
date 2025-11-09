import os
import glob
import time
import json
import requests
import RPi.GPIO as GPIO
import adafruit_dht
import math
import board
import serial
import re
from adafruit_ads1x15 import ADS1015, AnalogIn, ads1x15

# === Configuration ===
API_SENSOR_URL = "http://192.168.0.230:5050/api/sensors"
API_RELAY_URL  = "http://192.168.0.230:5050/api/relay-state"
API_MODE_URL   = "http://192.168.0.230:5050/api/mode"
API_CONFIG_URL = "http://192.168.0.230:5050/api/config"
STATE_FILE = "system_state.json"
RELAY_PINS = {"relay1": 26, "relay2": 20, "relay3": 21}
current_state = {
    "mode": "auto",
    "relay_states": {"relay1": 0, "relay2": 0, "relay3": 0},
    "config": {"temp_start_compressor": 4.5, "temp_stop_compressor": 3.5}
}

# Intervals
SENSOR_READ_INTERVAL = 2        # s
UPLOAD_INTERVAL = 5            # s
CHECK_POLL_INTERVAL = 4         # s
SAVE_STATE_INTERVAL = 120       # s
MANUAL_ROUTE_BACKOFF = 5        # s
MIN_ON_SEC = 120                # compressor min ON
MIN_OFF_SEC = 90                # compressor min OFF

# === Sensor setup (absolute paths) ===
os.system("modprobe w1-gpio")
os.system("modprobe w1-therm")
W1_BASE = "/sys/bus/w1/devices"
port = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=3)
i2c = board.I2C()
ads = ADS1015(i2c)

def _find_ds18b20_device_files():
    ds18b20_files = []
    try:
        for p in glob.glob(os.path.join(W1_BASE, "28*")):
            if os.path.isdir(p):
                ds18b20_files.append(os.path.join(p, "w1_slave"))
        return ds18b20_files
    except Exception:
        pass
    print("DS18B20 not found")
    return None

DS18B20_FILES = _find_ds18b20_device_files()
print(DS18B20_FILES)
dht22 = adafruit_dht.DHT22(board.D4, use_pulseio=False)

# === Helpers ===
def _now_ms():
    return int(time.time() * 1000)

def save_state(data):
    data["timestamp"] = int(time.time())
    tmp = STATE_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, STATE_FILE)
    print(f"State saved at {time.ctime(data['timestamp'])}")

def load_state():
    if not os.path.exists(STATE_FILE):
        print("No saved state found, starting fresh.")
        return None
    with open(STATE_FILE, "r") as f:
        state = json.load(f)
    print(f"Restored state from {time.ctime(state['timestamp'])}")
    return state

def read_scale():
    raw = port.read_until(b'\x03')
    if not raw:
        return None

    message = raw.decode('ascii', errors='ignore').strip()
    match = re.search(r'W= ([-+]?[0-9]*\.?[0-9]+)\s*kg', message)
    if not match:
        return None

    weight = float(match.group(1))
    return weight

def read_scale_ohaus():
    port.write(b'P\r\n')
    time.sleep(0.5)
    raw = port.read_all()
    if not raw:
        return None

    message = raw.decode('ascii', errors='ignore').strip()
    match = re.search(r"[-+]?\d*\.\d+|\d+", message)
  
    if not match:
        return None
    print(match.group())
    weight = float(match.group())
    return weight

def read_temp_raw():
    lines = []
    if not DS18B20_FILES:
        return None
    for file in DS18B20_FILES:
        with open(file, "r") as f:
            lines.append(f.readlines())
            f.close()

    return lines

def read_temp_ds18b20():
    files_lines = read_temp_raw()
    temps = []

    if files_lines is None:
        return None

    tries = 0
    for lines in files_lines:
        while lines[0].strip().endswith("YES") is False and tries < 5:
            time.sleep(0.2)
            lines = read_temp_raw()
            tries += 1
            if lines is None:
                return None
        equals_pos = lines[1].find("t=")
        if equals_pos != -1:
            temp_string = lines[1][equals_pos + 2:]
            temps.append(float(temp_string) / 1000.0)

    return [None, None] if temps is None else temps

def read_measurements():
    try:
        h = dht22.humidity
    except RuntimeError as err:
        print(f"DHT22 read error: {err.args[0]}")
        h =  None
    temp_inside = read_temp_ds18b20()[0]
    temp_outside = read_temp_ds18b20()[1]
    v = measure_rms()
    return temp_inside, h, temp_outside, v

def send_measurements(temp_inside, humidity, temp_outside, v, w):
    try:
        if (temp_inside is not None) and (humidity is not None) and (temp_outside is not None) and (v is not None) and (w is not None):
            data = {"temp_inside": temp_inside, "humidity": humidity, "temp_outside": temp_outside, "voltage": v, "weight": w}
            r = requests.post(API_SENSOR_URL, json=data, timeout=5)
            print("Sensor data sent:", r.status_code)
    except requests.RequestException as e:
        print("Error contacting server:", e)

def setup_gpio():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    for pin in RELAY_PINS.values():
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.HIGH)  # default off (active-low)

def set_relay_states(states):
    global _last_state

    for relay, value in states.items():
        if relay not in RELAY_PINS:
            continue

        if relay == 'relay1':
            _last_state = value

        GPIO.output(RELAY_PINS[relay], GPIO.LOW if value == 1 else GPIO.HIGH)
        current_state["relay_states"][relay] = 1 if value == 1 else 0

def fetch_relay_states():
    try:
        r = requests.get(API_RELAY_URL, timeout=5)
        if r.status_code == 200:
            data = r.json()
            print("Received relay state:", data)
            set_relay_states(data)
        else:
            print("Server returned status:", r.status_code, " not changing outputs.")
    except requests.RequestException as e:
        print("Error contacting server:", e)

def get_mode():
    try:
        r = requests.get(API_MODE_URL, timeout=3)
        return r.json().get("mode", "auto"), 0
    except Exception:
        print("Error fetching mode")
        return "auto", 1

_last_change_ms = 0
_last_state = 0  # 0=off, 1=on

def hysteresis_control(temp, start_on=4.5, stop_off=3.5):
    global _last_change_ms, _last_state
    if temp is None:
        return None
    now_ms = _now_ms()
    elapsed = (now_ms - _last_change_ms) / 1000.0
    if _last_state == 1:
        if temp < stop_off and elapsed >= MIN_ON_SEC:
            _last_state = 0
            _last_change_ms = now_ms
            return 0
    else:
        if temp > start_on and elapsed >= MIN_OFF_SEC:
            _last_state = 1
            _last_change_ms = now_ms
            return 1
    return None

def post_relay_states(mode="auto"):
    try:
        relay_states = {relay: (1 if GPIO.input(pin) == 0 else 0) for relay, pin in RELAY_PINS.items()}
        relay_states.update({'timestamp': _now_ms(), 'mode': mode})
        r = requests.post(API_RELAY_URL, json=relay_states, timeout=3)
        print("Synced relay states:", r.status_code, relay_states)
        return 1
    except Exception as e:
        print("Failed to sync relay states:", e)
        return 0

def get_config():
    try:
        r = requests.get(API_CONFIG_URL, timeout=3)
        if r.status_code == 200:
            data = r.json()
            ts = data.get("temp_start_compressor", 4.5)
            tf = data.get("temp_stop_compressor", 3.5)
            current_state['config']['temp_start_compressor'] = ts
            current_state['config']['temp_stop_compressor'] = tf
            return ts, tf
    except requests.RequestException as e:
        print("Error fetching config:", e)
    return current_state['config']['temp_start_compressor'], current_state['config']['temp_stop_compressor']

def emergency():
    print("Emergency fail-safe: turning ALL relays OFF")
    set_relay_states({"relay1": 0, "relay2": 0, "relay3": 0})

def post_config_once():
    try:
        r = requests.post(API_CONFIG_URL, json=current_state['config'], timeout=3)
        return r.ok
    except requests.RequestException:
        return False

def read_ads():
    chan = AnalogIn(ads, ads1x15.Pin.A0)
    return chan.voltage

def measure_rms(sample_time=1.0):
    samples = []
    t0 = time.time()

    while (time.time() - t0) < sample_time:
        v = read_ads()
        samples.append(v)

    # Compute DC offset (bias)
    offset = sum(samples) / len(samples)

    # Remove offset  ^f^r isolate AC
    ac = [v - offset for v in samples]

    # RMS = sqrt(average(square))
    squares = [v * v for v in ac]
    vrms_adc = math.sqrt(sum(squares) / len(squares))

    # Undo voltage divider (2:1)
    vrms_module = vrms_adc * 2.0
    
    # Constant for calibration
    K = 757

    mains_rms = vrms_module * K
    return mains_rms


def main():
    global current_state
    print("Starting relay client...")
    setup_gpio()

    prev = load_state()
    if prev:
        # shallow merge: keep current keys if not present in file
        current_state["mode"] = prev.get("mode", current_state["mode"])
        current_state["relay_states"].update(prev.get("relay_states", {}))
        current_state["config"].update(prev.get("config", {}))

    last_mode_check = time.monotonic()
    last_upload = time.monotonic()
    last_save = time.monotonic()
    last_handshake_try = 0.0
    last_measurements_read = 0
    control_mode = 'auto'
    first_connection = True

    try:
        while True:
            now_mono = time.monotonic()
            w = read_scale_ohaus()
            if now_mono - last_measurements_read > SENSOR_READ_INTERVAL:
                temp_outside, hum, temp_inside, v = read_measurements()
                last_measurements_read = now_mono

            # One-time config push on first (re)connection with backoff
            if first_connection and (now_mono - last_handshake_try) >= MANUAL_ROUTE_BACKOFF:
                if post_config_once():
                    post_relay_states()
                    first_connection = False
                last_handshake_try = now_mono

            # Mode check
            if (now_mono - last_mode_check) >= CHECK_POLL_INTERVAL:
                control_mode, conn_err = get_mode()
                last_mode_check = now_mono
                if conn_err == 1 and first_connection == False:
                    emergency()
                    first_connection = True
                print("Mode:", control_mode)

            # Control
            if control_mode == "auto":
                if first_connection == False:
                    ts, tf = get_config()
                    print(ts, tf)
                else:
                    ts = current_state["config"]["temp_start_compressor"]
                    tf = current_state["config"]["temp_stop_compressor"]
                change = hysteresis_control(temp_outside, ts, tf)
                if change is not None:
                    set_relay_states({"relay1": change})
                    post_relay_states(mode="auto")
            else:
                fetch_relay_states()

            # Upload sensors
            if (now_mono - last_upload) >= UPLOAD_INTERVAL:
                send_measurements(temp_inside, hum, temp_outside, v, w)
                last_upload = now_mono

            # Persist state
            if (now_mono - last_save) >= SAVE_STATE_INTERVAL:
                save_state(current_state)
                last_save = now_mono
            print(temp_inside, hum, temp_outside, v, w)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()

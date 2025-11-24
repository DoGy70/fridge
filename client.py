import os
import glob
import time
import json
import requests
import RPi.GPIO as GPIO
import adafruit_dht
import math
import board
from adafruit_ads1x15 import ADS1015, AnalogIn, ads1x15

# === Configuration ===
API_SENSOR_URL = "http://192.168.100.170:5050/api/sensors"
STATE_FILE = "system_state.json"
RELAY_PINS = {"compressor": 26, "ventilation": 20, "heater": 21}
current_state = {
    "humidity": 0,
    "auto_mode": True,
    "relay_states": {"compressor": 0, "ventilation": 0, "heater": 0},
    "target_temperature": 0,
    "defrost_threshold_temperature": 10,
    "status": "OFF",
    "defrost_type": "AUTO",
    "problem": False
}

# Intervals
SENSOR_READ_INTERVAL = 4        # s
UPLOAD_INTERVAL = 5            # s
CHECK_POLL_INTERVAL = 4         # s
SAVE_STATE_INTERVAL = 120       # s
MANUAL_ROUTE_BACKOFF = 3        # s
MIN_ON_SEC = 1                # compressor min ON
MIN_OFF_SEC = 9                # compressor min OFF

# === Sensor setup (absolute paths) ===
os.system("modprobe w1-gpio")
os.system("modprobe w1-therm")
W1_BASE = "/sys/bus/w1/devices"

# Default values
id = 1
hysteresis_defrost = 2
hysteresis_compressor_stop = 3
i2c = board.I2C()
ads = ADS1015(i2c)
_last_state_compressor = 0
_last_change_compressor = 0
_last_state_defrost = 0
_last_change_defrost = 0
_dht22_tries = 0

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

    if not files_lines:
        return [None, None]

    temps = []
    for lines in files_lines:
        tries = 0
        while lines[0].strip().endswith("YES") is False and tries < 5:
            time.sleep(0.2)
            lines = read_temp_raw()
            if not lines:
                return [None, None]
            tries += 1

        equals_pos = lines[1].find("t=")
        if equals_pos != -1:
            temp_string = lines[1][equals_pos + 2:]
            temps.append(float(temp_string) / 1000.0)
        else:
            temps.append(None)

    if len(temps) < 2:
        temps += [None] * (2 - len(temps))

    return temps

def read_measurements():
    global _dht22_tries
    try:
        humidity = dht22.humidity
        current_state["humidity"] = humidity
        _dht22_tries = 0
    except RuntimeError as err:
        print(f"DHT22 read error: {err.args[0]}")
        _dht22_tries += 1
        if _dht22_tries >= 3:
            current_state["humidity"] = 0
        humidity = current_state["humidity"]

    temperatures = read_temp_ds18b20()
    temperature = temperatures[0]
    evaporator_temperature = temperatures[1]
    supply_voltage = measure_rms()
    return temperature, humidity, evaporator_temperature, supply_voltage

def send_measurements(temperature, humidity, evaporator_temperature, supply_voltage):
        payload = {
        "temperature": temperature,
        "humidity": humidity,
        "evaporator_temperature": evaporator_temperature,
        "supply_voltage": supply_voltage,
        "target_temperature": current_state.get("target_temperature"),
        "defrost_threshold_temperature": current_state.get("defrost_threshold_temperature"),
        "defrost_type": current_state.get("defrost_type"),
        "compressor_on": current_state["relay_states"].get("compressor") == 1,
        "ventilation_on": current_state["relay_states"].get("ventilation") == 1,
        "heater_on": current_state["relay_states"].get("heater") == 1,
        "auto_mode": current_state.get("auto_mode"),
        "status": current_state.get("status"),
        "problem": current_state.get("problem"),
        }
        try:
            response = requests.post(API_SENSOR_URL, json=payload, auth=("raspberries", "rapanarapana") ,timeout=5)
            response.raise_for_status()
            r = response.json()
            current_state.update({
                "auto_mode": r.get("auto_mode", current_state["auto_mode"]),
                "target_temperature": r.get("target_temperature", current_state["target_temperature"]),
                "defrost_type": r.get("defrost_type", current_state["defrost_type"]),
                "defrost_threshold_temperature": r.get("defrost_threshold_temperature", current_state["defrost_threshold_temperature"])
            })
            if not current_state["auto_mode"]:
                relays = current_state["relay_states"]
                relays["compressor"] = 1 if r.get("compressor_on") == True else 0
                relays["ventilation"] = 1 if r.get("ventilation_on") == True else 0
                relays["heater"] = 1 if r.get("heater_on") == True else 0

            return 0
        except requests.RequestException as e:
            print("Error contacting server:", e)
            return 1

def setup_gpio():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    for pin in RELAY_PINS.values():
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.HIGH)  # default off (active-low)

def set_relay_states(states):
    global _last_state_compressor, _last_change_compressor, _last_state_defrost, _last_change_defrost
    for relay, value in states.items():
        if relay not in RELAY_PINS:
            continue

        if relay == 'compressor':
            if value != _last_state_compressor:
                _last_state_compressor = value
                _last_change_compressor = _now_ms()

        if relay == "heater":
            if value != _last_state_defrost:
                _last_state_defrost = value
                _last_change_defrost = _now_ms()

        GPIO.output(RELAY_PINS[relay], GPIO.LOW if value == 1 else GPIO.HIGH)
        current_state["relay_states"][relay] = 1 if value == 1 else 0

def hysteresis_control(temp, start_on=4.5, stop_off=3.5, mode="auto"):
    global _last_state_compressor, _last_change_compressor

    if temp is None:
        return None
    now_ms = _now_ms()
    elapsed = (now_ms - _last_change_compressor) / 1000.0
    if _last_state_compressor == 1:
        if temp < stop_off and elapsed >= MIN_ON_SEC:
            _last_state_compressor = 0
            _last_change_compressor = now_ms
            return 0
    else:
        if temp > start_on and elapsed >= MIN_OFF_SEC:
            _last_state_compressor = 1
            _last_change_compressor = now_ms
            return 1
    return None

def hysteresis_control_defrost(temp, start_on, stop_off):
    global _last_state_defrost, _last_change_defrost

    if temp is None:
        return None

    now_ms = _now_ms()
    elapsed = (now_ms - _last_change_defrost) / 1000.0

    if temp > stop_off and elapsed >= MIN_ON_SEC:
        _last_state_defrost = 0
        _last_change_defrost = now_ms
        return 0

    if temp < start_on and elapsed >= MIN_OFF_SEC:
        _last_state_defrost = 1
        _last_change_defrost = now_ms
        return 1

def emergency():
    print("Emergency fail-safe: turning ALL relays OFF")
    set_relay_states({"compressor": 0, "ventilation": 0, "heater": 0})

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
    print("Starting relay client...")
    setup_gpio()

    prev = load_state()
    if prev:
        # shallow merge: keep current keys if not present in file
        current_state["target_temperature"] = prev.get("target_temperature", current_state["target_temperature"])
        current_state["defrost_threshold_temperature"] = prev.get("defrost_threshold_temperature", current_state["defrost_threshold_temperature"])
        current_state["status"] = prev.get("status", current_state["status"])
        current_state["defrost_type"] = prev.get("defrost_type", current_state["defrost_type"])

    last_upload = time.monotonic()
    last_save = time.monotonic()
    last_measurements_read = 0
    stable_sensor_reads = 0
    STABLE_REQUIRED = 10
    ever_connected = False
    connection_fault_triggered = False
    compressor_and_ventilation_forbidden = False

    try:
        while True:
            now_mono = time.monotonic()

            # Reead sensors
            if now_mono - last_measurements_read > SENSOR_READ_INTERVAL:
                temperature, humidity, evaporator_temperature, supply_voltage = read_measurements()
                last_measurements_read = now_mono
                print(f"temperature: {temperature}\thumidity: {humidity}\tevaporator_temperature:{evaporator_temperature}\tsupply_voltage: {supply_voltage}")
                print(current_state)

            # Check whether sensors work
            if temperature is None or evaporator_temperature is None:
                print("Temperature sensors do not work!")
                emergency()
                stable_sensor_reads = 0
                current_state["problem"] = True

                if (now_mono - last_upload) >= UPLOAD_INTERVAL:
                    send_measurements(temperature, humidity, evaporator_temperature, supply_voltage)
                    last_upload = now_mono
                continue

            # If sensors work -> no problem
            if current_state["problem"]:
                stable_sensor_reads += 1
                print(f"Sensor stable counter: {stable_sensor_reads}/{STABLE_REQUIRED}")
                if stable_sensor_reads >= STABLE_REQUIRED:
                    print("Sensors stable again - exiting fail-safe mode.")
                    current_state["problem"] = False

                if (now_mono - last_upload) >= UPLOAD_INTERVAL:
                    send_measurements(temperature, humidity, evaporator_temperature, supply_voltage)
                    last_upload = now_mono
                continue

            # Normal operation mode
            if current_state["auto_mode"]:
                    temperature_start_defrost = current_state["defrost_threshold_temperature"]
                    temperature_stop_defrost = temperature_start_defrost + hysteresis_defrost
                    change_defrost = hysteresis_control_defrost(evaporator_temperature, temperature_start_defrost, temperature_stop_defrost)
                    if change_defrost is not None:
                        print("change_defrost:", change_defrost)
                        if change_defrost == 1:
                            current_state["status"] = "DEFROST"
                            compressor_and_ventilation_forbidden = True
                            print(current_state["defrost_type"])
                            if current_state["defrost_type"] == "AUTO":
                                print("Hello from auto!")
                                set_relay_states({"compressor": 0, "ventilation": 0, "heater": 0})
                            else:
                                set_relay_states({"compressor": 0, "ventilation": 0, "heater": change_defrost})
                        else:
                            compressor_and_ventilation_forbidden = False
                            set_relay_states({"heater": 0})

                    if not compressor_and_ventilation_forbidden:
                        current_state["status"] = "OFF" if current_state["relay_states"]["compressor"] == 0 else "ON"
                        temperature_stop_compressor = current_state["target_temperature"]
                        temperature_start_compressor = temperature_stop_compressor + hysteresis_compressor_stop
                        change_compressor = hysteresis_control(temperature, temperature_start_compressor, temperature_stop_compressor)
                        if change_compressor is not None:
                            set_relay_states({"compressor": change_compressor})
                            current_state["relay_states"]["compressor"] = change_compressor
            else:
                relays = current_state["relay_states"]
                set_relay_states({"compressor": relays["compressor"], "ventilation": relays["ventilation"], "heater": relays["heater"]})
                current_state["status"] = "OFF" if current_state["relay_states"]["compressor"] == 0 else "ON"
            # Upload sensors
            if (now_mono - last_upload) >= UPLOAD_INTERVAL:
                if send_measurements(temperature, humidity, evaporator_temperature, supply_voltage) == 0:
                    last_upload = now_mono
                    connection_fault_triggered = False
                    ever_connected = True
                else:
                    if ever_connected and not connection_fault_triggered:
                        emergency()
                        connection_fault_triggered = True
            # Persist state
            if (now_mono - last_save) >= SAVE_STATE_INTERVAL:
                save_state(current_state)
                last_save = now_mono

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()
        current_state["status"] = "OFF"
        send_measurements(temperature, humidity, evaporator_temperature, supply_voltage)

if __name__ == "__main__":
    main()
	

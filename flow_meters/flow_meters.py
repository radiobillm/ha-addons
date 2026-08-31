"""
Flow Meters add-on
Counts GPIO pulses from open-collector hall-effect flow sensors and
publishes flow rate / cumulative volume / raw pulse count to MQTT with
Home Assistant MQTT-discovery config, so entities appear automatically.

Each sensor's `ml_per_pulse` is an add-on option, editable from the HA UI
with no code change or rebuild needed. It defaults to 9.26 mL/pulse (108
pulses/liter, per the YF-DN32-S-SM stainless datasheet) as a starting
placeholder — UNVERIFIED for the actual installed units. Use the raw
"Pulse Count" diagnostic entity (which is NOT scaled by ml_per_pulse) to
confirm or correct it:

    ml_per_pulse = (known fill volume in mL) / (pulses counted during that fill)

Then edit the add-on's ml_per_pulse option for that sensor and restart.
"""

import json
import signal
import sys
import threading
import time

import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO

OPTIONS_PATH = "/data/options.json"
TOTALS_PATH = "/data/totals.json"

DISCOVERY_PREFIX = "homeassistant"
BASE_TOPIC = "flow_meters"
DEVICE = {
    "identifiers": ["flow_meters_addon"],
    "name": "Water Flow",
    "manufacturer": "DIY",
    "model": "RPi GPIO Flow Meters",
}


def slug(name):
    return name.lower().replace(" ", "_").replace("-", "_")


def load_json(path, default):
    try:
        with open(path) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return default


def save_totals(totals):
    try:
        with open(TOTALS_PATH, "w") as f:
            json.dump(totals, f)
    except OSError as e:
        print(f"WARN: could not save totals: {e}", flush=True)


def main():
    opts = load_json(OPTIONS_PATH, {})
    mqtt_host = opts.get("mqtt_host", "core-mosquitto")
    mqtt_port = int(opts.get("mqtt_port", 1883))
    mqtt_user = opts.get("mqtt_username") or None
    mqtt_pass = opts.get("mqtt_password") or None
    debounce_ms = int(opts.get("debounce_ms", 20))
    publish_interval = int(opts.get("publish_interval_s", 5))
    sensors = opts.get("sensors", [])

    if not sensors:
        print("ERROR: no sensors configured", flush=True)
        sys.exit(1)

    pulse_counts = {s["name"]: 0 for s in sensors}
    totals_l = load_json(TOTALS_PATH, {})
    for s in sensors:
        totals_l.setdefault(s["name"], 0.0)
    latest_rate_lpm = {s["name"]: 0.0 for s in sensors}

    lock = threading.Lock()
    latest_lock = threading.Lock()

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    def make_callback(name):
        def _cb(_channel):
            with lock:
                pulse_counts[name] += 1
        return _cb

    for s in sensors:
        pin = int(s["gpio"])
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(pin, GPIO.FALLING, callback=make_callback(s["name"]), bouncetime=debounce_ms)
        print(f"Watching '{s['name']}' on GPIO{pin} (ml_per_pulse={s.get('ml_per_pulse', 1.0)})", flush=True)

    client = mqtt.Client(client_id="flow_meters_addon")
    if mqtt_user:
        client.username_pw_set(mqtt_user, mqtt_pass)
    client.connect(mqtt_host, mqtt_port, keepalive=60)
    client.loop_start()

    def publish_discovery():
        for s in sensors:
            sid = slug(s["name"])
            client.publish(
                f"{DISCOVERY_PREFIX}/sensor/flow_meters/{sid}_pulses/config",
                json.dumps({
                    "name": f"{s['name']} Pulse Count",
                    "unique_id": f"flow_meters_{sid}_pulses",
                    "state_topic": f"{BASE_TOPIC}/{sid}/pulses",
                    "unit_of_measurement": "pulses",
                    "icon": "mdi:pulse",
                    "entity_category": "diagnostic",
                    "device": DEVICE,
                }),
                retain=True,
            )
            client.publish(
                f"{DISCOVERY_PREFIX}/sensor/flow_meters/{sid}_rate/config",
                json.dumps({
                    "name": f"{s['name']} Flow Rate",
                    "unique_id": f"flow_meters_{sid}_rate",
                    "state_topic": f"{BASE_TOPIC}/{sid}/rate_lpm",
                    "unit_of_measurement": "L/min",
                    "icon": "mdi:water",
                    "device": DEVICE,
                }),
                retain=True,
            )
            client.publish(
                f"{DISCOVERY_PREFIX}/sensor/flow_meters/{sid}_total/config",
                json.dumps({
                    "name": f"{s['name']} Total Volume",
                    "unique_id": f"flow_meters_{sid}_total",
                    "state_topic": f"{BASE_TOPIC}/{sid}/total_l",
                    "unit_of_measurement": "L",
                    "device_class": "water",
                    "state_class": "total_increasing",
                    "icon": "mdi:water-pump",
                    "device": DEVICE,
                }),
                retain=True,
            )

    publish_discovery()

    lcd_ui = None
    if opts.get("enable_lcd", True):
        def get_reading(name):
            with latest_lock:
                return latest_rate_lpm[name]

        try:
            from lcd_ui import LcdUi  # lazy import: LCD deps aren't installed in the lean default build
            lcd_ui = LcdUi(
                sensor_names=[s["name"] for s in sensors],
                get_reading=get_reading,
                color=opts.get("lcd_color", "red"),
                refresh_s=float(opts.get("lcd_refresh_s", 0.5)),
                poll_s=float(opts.get("button_poll_s", 0.05)),
            )
            lcd_ui.start()
            print("LCD UI started", flush=True)
        except Exception as e:  # noqa: BLE001 - LCD is optional hardware
            print(f"WARN: LCD not available, continuing without it: {e}", flush=True)
            lcd_ui = None

    running = True

    def handle_sig(*_args):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, handle_sig)
    signal.signal(signal.SIGINT, handle_sig)

    while running:
        time.sleep(publish_interval)
        with lock:
            snapshot = dict(pulse_counts)
            for k in pulse_counts:
                pulse_counts[k] = 0

        for s in sensors:
            name = s["name"]
            sid = slug(name)
            ml_per_pulse = float(s.get("ml_per_pulse", 1.0))
            pulses = snapshot[name]
            volume_l = (pulses * ml_per_pulse) / 1000.0
            rate_lpm = (volume_l / publish_interval) * 60.0
            totals_l[name] += volume_l

            client.publish(f"{BASE_TOPIC}/{sid}/pulses", pulses)
            client.publish(f"{BASE_TOPIC}/{sid}/rate_lpm", round(rate_lpm, 3))
            client.publish(f"{BASE_TOPIC}/{sid}/total_l", round(totals_l[name], 3))

            with latest_lock:
                latest_rate_lpm[name] = rate_lpm

        save_totals(totals_l)

    if lcd_ui is not None:
        lcd_ui.stop()
        lcd_ui.join(timeout=2)
    GPIO.cleanup()
    client.loop_stop()
    client.disconnect()


if __name__ == "__main__":
    main()

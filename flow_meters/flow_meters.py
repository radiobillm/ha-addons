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

Flow-rate curve (v0.2.13): these paddlewheel sensors are not linear - they
under-read at low flow and over-read at high flow, so one ml_per_pulse can't
fit. A sensor may carry an optional `curve` option, a string of
"hz:ml,hz:ml,..." knots (pulse rate in Hz -> mL per pulse at that rate).
Each publish interval's pulses are valued at the mL/pulse interpolated
linearly between knots for that interval's pulse rate (flat beyond the
first/last knot). With no curve the flat ml_per_pulse applies, as before.
Curves are fitted against the city meter: water/calibration/curve/.

v0.3.1: the LCD rotates through the pressure sensors after the flow view, and
its buttons step through one sensor at a time (see lcd_ui.py).

v0.3.0 (interposer board):
- `pulse_edge` / `pulse_pull`: the board's 74LVC14 buffer inverts and drives
  the GPIO, so count RISING edges with no internal pull.
- per-sensor `stray_pulse_max`: an interval with at most that many pulses,
  right after a silent interval, is held back; it is added to the next
  interval if flow continues and dropped if the next interval is silent too.
  Removes the lone rotor-twitch pulses a dead leg shows when the pump starts.
  It also drops real trickle flow, so use it only on a sensor that should
  read zero. The Pulse Count entity stays raw.
- `pressure_sensors`: ratiometric 0.5-4.5 V transducers on ADS1115 0x48
  through the board's x0.6 dividers, corrected by the 5 V analog rail
  measured on ADS1115 0x49 A0. The flow rail is on 0x49 A1.
- `aux_inputs`: dry-contact inputs (pump RUN / FAULT), HIGH = contact closed,
  published as MQTT binary_sensors.
"""

import json
import os
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


def parse_curve(text):
    """ "0.2:323,1:49.6,8:39" -> [(0.2, 323.0), (1.0, 49.6), (8.0, 39.0)], or None if empty/invalid."""
    if not text or not str(text).strip():
        return None
    try:
        pts = sorted((float(h), float(m)) for h, m in (p.split(":") for p in str(text).split(",") if p.strip()))
    except ValueError:
        print(f"WARN: bad curve '{text}' - using flat ml_per_pulse", flush=True)
        return None
    if len(pts) < 2 or any(m <= 0 for _, m in pts):
        print(f"WARN: bad curve '{text}' - using flat ml_per_pulse", flush=True)
        return None
    return pts


def ml_per_pulse_at(curve, hz, flat):
    """mL per pulse at this pulse rate: linear between curve knots, flat beyond the ends."""
    if not curve:
        return flat
    if hz <= curve[0][0]:
        return curve[0][1]
    if hz >= curve[-1][0]:
        return curve[-1][1]
    for (h0, m0), (h1, m1) in zip(curve, curve[1:]):
        if hz <= h1:
            return m0 + (m1 - m0) * (hz - h0) / (h1 - h0)
    return curve[-1][1]


def load_json(path, default):
    try:
        with open(path) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return default


def save_totals(totals):
    # Write to a temp file and rename, so a power cut mid-write can't leave an
    # empty totals.json (that zeroed every total on 2026-10-12).
    try:
        tmp = TOTALS_PATH + ".tmp"
        with open(tmp, "w") as f:
            json.dump(totals, f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, TOTALS_PATH)
    except OSError as e:
        print(f"WARN: could not save totals: {e}", flush=True)


def scan_i2c(bus=1):
    """Addresses that ACK a 1-byte read on /dev/i2c-<bus> (startup log only)."""
    import fcntl
    found = []
    try:
        fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
    except OSError as e:
        return f"unavailable ({e})"
    try:
        for addr in range(0x08, 0x78):
            try:
                fcntl.ioctl(fd, 0x0703, addr)  # I2C_SLAVE
                os.read(fd, 1)
                found.append(f"0x{addr:02x}")
            except OSError:
                pass
    finally:
        os.close(fd)
    return " ".join(found) or "none"


class Ads1115:
    """Single-shot reads from an ADS1115 over /dev/i2c-<bus>, no extra libraries."""
    LSB_V = 4.096 / 32768.0  # PGA +-4.096 V

    def __init__(self, addr, bus=1):
        import fcntl
        self.fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        fcntl.ioctl(self.fd, 0x0703, addr)  # I2C_SLAVE

    def volts(self, ch, samples=4):
        cfg = 0x8000 | ((4 + ch) << 12) | (1 << 9) | (1 << 8) | (4 << 5) | 3  # AINch-GND, single-shot, 128 SPS
        total = 0
        for _ in range(samples):
            os.write(self.fd, bytes([0x01, cfg >> 8, cfg & 0xFF]))
            time.sleep(0.011)
            os.write(self.fd, b"\x00")
            raw = int.from_bytes(os.read(self.fd, 2), "big", signed=True)
            total += raw
        return total / samples * self.LSB_V


DIVIDER = 0.6  # board divider on every 5 V-referenced analog input


def pressure_psi(v_adc, v5, full_scale):
    """0.5-4.5 V ratiometric transducer -> psi, or None if the sensor looks absent."""
    k = v5 / 5.0
    vs = v_adc / DIVIDER
    if k < 0.8 or vs < 0.25 * k:  # rail down, or below any valid output: unplugged / broken wire
        return None
    return (vs - 0.5 * k) / (4.0 * k) * full_scale


def main():
    print(f"I2C bus 1 devices: {scan_i2c()}", flush=True)
    opts = load_json(OPTIONS_PATH, {})
    mqtt_host = opts.get("mqtt_host", "core-mosquitto")
    mqtt_port = int(opts.get("mqtt_port", 1883))
    mqtt_user = opts.get("mqtt_username") or None
    mqtt_pass = opts.get("mqtt_password") or None
    debounce_ms = int(opts.get("debounce_ms", 20))
    publish_interval = int(opts.get("publish_interval_s", 5))
    sensors = opts.get("sensors", [])
    edge = GPIO.RISING if opts.get("pulse_edge", "falling") == "rising" else GPIO.FALLING
    pull = GPIO.PUD_OFF if opts.get("pulse_pull", "up") == "off" else GPIO.PUD_UP
    pressure_sensors = opts.get("pressure_sensors", [])
    aux_inputs = opts.get("aux_inputs", [])
    analog_interval = int(opts.get("analog_interval_s", 5))

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
    latest_psi = {}  # pressure sensor name -> psi or None, for the LCD

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    def make_callback(name):
        def _cb(_channel):
            with lock:
                pulse_counts[name] += 1
        return _cb

    for s in sensors:
        pin = int(s["gpio"])
        GPIO.setup(pin, GPIO.IN, pull_up_down=pull)
        GPIO.add_event_detect(pin, edge, callback=make_callback(s["name"]), bouncetime=debounce_ms)
        s["_curve"] = parse_curve(s.get("curve"))
        s["_stray_max"] = int(s.get("stray_pulse_max") or 0)
        s["_held"] = 0
        s["_prev_raw"] = 0
        print(f"Watching '{s['name']}' on GPIO{pin} (ml_per_pulse={s.get('ml_per_pulse', 1.0)}, "
              f"curve={s['_curve'] or 'none'}, stray_pulse_max={s['_stray_max']})", flush=True)
    print(f"Pulse inputs: edge={opts.get('pulse_edge', 'falling')}, pull={opts.get('pulse_pull', 'up')}, "
          f"debounce={debounce_ms} ms", flush=True)

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

    # ---- dry-contact inputs (pump RUN / FAULT): HIGH = contact closed ----
    def publish_aux(a):
        client.publish(f"{BASE_TOPIC}/aux/{slug(a['name'])}/state",
                       "ON" if GPIO.input(int(a["gpio"])) else "OFF", retain=True)

    for a in aux_inputs:
        pin = int(a["gpio"])
        aid = slug(a["name"])
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        cfg = {
            "name": a["name"],
            "unique_id": f"flow_meters_aux_{aid}",
            "state_topic": f"{BASE_TOPIC}/aux/{aid}/state",
            "device": DEVICE,
        }
        if a.get("device_class"):
            cfg["device_class"] = a["device_class"]
        client.publish(f"{DISCOVERY_PREFIX}/binary_sensor/flow_meters/{aid}/config", json.dumps(cfg), retain=True)
        GPIO.add_event_detect(pin, GPIO.BOTH, callback=lambda _ch, a=a: publish_aux(a), bouncetime=20)
        publish_aux(a)
        print(f"Aux input '{a['name']}' on GPIO{pin}: {'closed' if GPIO.input(pin) else 'open'}", flush=True)

    # ---- analog: pressure transducers + 5 V rails ----
    def analog_sensor_config(key, name, unit, device_class, diagnostic=False):
        cfg = {
            "name": name,
            "unique_id": f"flow_meters_{key}",
            "state_topic": f"{BASE_TOPIC}/analog/{key}",
            "unit_of_measurement": unit,
            "device_class": device_class,
            "state_class": "measurement",
            "availability_topic": f"{BASE_TOPIC}/analog/{key}/available",
            "device": DEVICE,
        }
        if diagnostic:
            cfg["entity_category"] = "diagnostic"
        client.publish(f"{DISCOVERY_PREFIX}/sensor/flow_meters/{key}/config", json.dumps(cfg), retain=True)

    def analog_loop():
        try:
            ads1, ads2 = Ads1115(0x48), Ads1115(0x49)
        except OSError as e:
            print(f"WARN: analog disabled, cannot open I2C: {e}", flush=True)
            return
        analog_sensor_config("rail_5v_analog", "5V Analog Rail", "V", "voltage", diagnostic=True)
        analog_sensor_config("rail_5v_flow", "5V Flow Rail", "V", "voltage", diagnostic=True)
        for ps in pressure_sensors:
            pid = slug(ps["name"])
            analog_sensor_config(f"pressure_{pid}", f"Pressure {ps['name']}", "psi", "pressure")
            analog_sensor_config(f"pressure_{pid}_volts", f"Pressure {ps['name']} Sensor Voltage", "V", "voltage",
                                 diagnostic=True)
        last = {}  # key -> (value, time published)

        def put(key, value, digits):
            now = time.time()
            if value is None:
                if last.get(key, (0, 0))[0] is not None:
                    client.publish(f"{BASE_TOPIC}/analog/{key}/available", "offline", retain=True)
                    last[key] = (None, now)
                return
            value = round(value, digits)
            prev = last.get(key)
            if prev is None or prev[0] is None:
                client.publish(f"{BASE_TOPIC}/analog/{key}/available", "online", retain=True)
            if prev is None or prev[0] != value or now - prev[1] >= 60:  # on change, and once a minute
                client.publish(f"{BASE_TOPIC}/analog/{key}", value)
                last[key] = (value, now)

        first = True
        while running:
            try:
                v5a = ads2.volts(0) / DIVIDER
                v5f = ads2.volts(1) / DIVIDER
                put("rail_5v_analog", v5a, 2)
                put("rail_5v_flow", v5f, 2)
                msg = [f"5VA={v5a:.3f} V", f"5VF={v5f:.3f} V"]
                for ps in pressure_sensors:
                    pid = slug(ps["name"])
                    v = ads1.volts(int(ps["channel"]))
                    psi = pressure_psi(v, v5a, float(ps.get("full_scale_psi", 100)))
                    latest_psi[ps["name"]] = psi
                    put(f"pressure_{pid}_volts", v / DIVIDER, 3)
                    put(f"pressure_{pid}", psi, 1)
                    msg.append(f"{ps['name']}={v / DIVIDER:.3f} V -> "
                               + (f"{psi:.1f} psi" if psi is not None else "no sensor"))
                if first:
                    print("Analog first read: " + ", ".join(msg), flush=True)
                    first = False
            except OSError as e:
                print(f"WARN: analog read failed: {e}", flush=True)
            time.sleep(analog_interval)

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
                pressure_names=[ps["name"] for ps in pressure_sensors],
                get_pressure=latest_psi.get,
                get_total=lambda name: totals_l[name] * 0.264172,
                moving_pressure=opts.get("lcd_moving_pressure", "House"),
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

    if pressure_sensors or opts.get("enable_analog", False):
        threading.Thread(target=analog_loop, daemon=True).start()
    aux_tick = 0

    while running:
        time.sleep(publish_interval)
        with lock:
            snapshot = dict(pulse_counts)
            for k in pulse_counts:
                pulse_counts[k] = 0

        for s in sensors:
            name = s["name"]
            sid = slug(name)
            raw = snapshot[name]
            pulses = raw
            if s["_stray_max"]:
                if s["_held"]:  # held from the last interval: keep it only if flow continued
                    pulses = raw + s["_held"] if raw else 0
                    s["_held"] = 0
                elif 0 < raw <= s["_stray_max"] and s["_prev_raw"] == 0:
                    s["_held"], pulses = raw, 0
                s["_prev_raw"] = raw
            ml_per_pulse = ml_per_pulse_at(s.get("_curve"), pulses / publish_interval,
                                           float(s.get("ml_per_pulse", 1.0)))
            volume_l = (pulses * ml_per_pulse) / 1000.0
            rate_lpm = (volume_l / publish_interval) * 60.0
            totals_l[name] += volume_l

            client.publish(f"{BASE_TOPIC}/{sid}/pulses", raw)
            client.publish(f"{BASE_TOPIC}/{sid}/rate_lpm", round(rate_lpm, 3))
            client.publish(f"{BASE_TOPIC}/{sid}/total_l", round(totals_l[name], 3))

            with latest_lock:
                latest_rate_lpm[name] = rate_lpm

        if any(snapshot.values()):  # only touch the SD card when something flowed
            save_totals(totals_l)

        aux_tick += 1
        if aux_tick * publish_interval >= 60:  # re-assert contact states once a minute
            aux_tick = 0
            for a in aux_inputs:
                publish_aux(a)

    if lcd_ui is not None:
        lcd_ui.stop()
        lcd_ui.join(timeout=2)
    GPIO.cleanup()
    client.loop_stop()
    client.disconnect()


if __name__ == "__main__":
    main()

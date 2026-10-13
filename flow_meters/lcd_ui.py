"""
Adafruit i2c 16x2 RGB LCD Pi Plate driver + page UI.

Display logic (rewritten 2026-09-02, replacing the old fixed-page scheme):
- 0 sensors flowing: shows "No flow" centered on row 1, row 2 blank.
- 1 sensor flowing: that sensor on row 1, row 2 blank.
- 2 sensors flowing: one per row, both shown at once (whichever two are
  actually active, not tied to a fixed City/Cistern-style pairing).
- 3 or 4 sensors flowing: not enough rows for all of them at once, so the
  display scrolls through them two at a time, advancing to the next pair
  every SCROLL_INTERVAL_S (2s). Non-overlapping windows in sensor order,
  wrapping around; with an odd count (3) one sensor repeats each cycle.
Row format unchanged from the original port of the old rpi-flow-meters
code: "<Label>:" left-aligned, flow rate right-aligned to fill 16 columns,
e.g. "Fr City:       3gm" (whole gallons/min, floored). The label shown is
a DISPLAY-ONLY mapping (see _DISPLAY_LABELS below) to the same Fr:/To:
directional names used on the HA dashboard card — the underlying sensor
name (config.yaml, MQTT topics, HA entity ids) is untouched.

Pressure pages (v0.3.1): when pressure sensors are configured, the automatic
display rotates: the flow view above for ROTATE_FLOW_S, then the pressure
sensors two per page for ROTATE_PRESSURE_S each ("Source:  51.0psi"), in the
order they are listed in the add-on's `pressure_sensors` option.

Buttons (v0.3.1, replacing the old UP/DOWN fixed-pair paging): any press
stops the rotation and shows ONE sensor on its own - first the flow sensors
in configured order (rate on row 1, total gallons on row 2), then the
pressure sensors. DOWN / RIGHT / SELECT step to the next one, UP / LEFT to
the previous one. After MANUAL_HOLD_S (30 s) without a press the rotation
resumes.

Uses adafruit_character_lcd.character_lcd_rgb_i2c.Character_LCD_RGB_I2C —
the SAME class the old code used (see its cistern.py) — which already
knows this specific plate's MCP23017 wiring internally, including the
.up_button/.down_button/.select_button/.left_button/.right_button and
.color/.display convenience properties. No hand-mapped pin numbers here.
"""

import threading
import time

import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import board
import busio

GALLON_PER_LITER = 0.264172
MANUAL_HOLD_S = 30  # seconds of no button press before the rotation resumes
ROTATE_FLOW_S = 4.0  # how long the flow view stays up in the rotation
ROTATE_PRESSURE_S = 3.0  # how long each pressure page stays up in the rotation
SCROLL_INTERVAL_S = 2.0  # how often the display advances when 3+ sensors are flowing
FLOW_EPSILON_LPM = 0.05  # below this, a sensor counts as "not flowing"

# Display-only labels (2026-09-02), matching the HA dashboard card's
# directional Fr:/To: naming (per Bill's own pin labels — see the pin/GPIO
# table in the brett-water-rpi-mqtt-pivot memory). DISPLAY TEXT ONLY: keys are
# the config.yaml sensor `name` values, which stay unchanged everywhere
# else (MQTT topic slugs, HA entity unique_ids) — renaming those would
# orphan the existing entities rather than relabel them (see the earlier
# "entity_id stays sticky" device-rename lesson in project memory). Falls
# back to the raw name for any sensor not in this map.
_DISPLAY_LABELS = {
    "House": "To House",
    "Yard": "To Yard",  # 2026-09-19: was "Fill-Cistern": "To Cistern" - that
    # line now feeds the yard, not the cistern (real plumbing change, see
    # config.yaml's dated comment + brett-water-rpi-mqtt-pivot memory).
    # NOT YET DEPLOYED to the running image as of this edit - this file
    # is baked into the Docker image (ghcr.io/radiobillm/flow-meters-addon),
    # unlike config.yaml's `sensors` list which is a runtime option. Until
    # a rebuild+push+update cycle happens, the LCD falls back to the raw
    # name ("Yard", via _display_label()'s .get(name, name) below) instead
    # of "To Yard" - harmless (still correct, just missing the directional
    # prefix), not worth a rebuild on its own.
    "City": "Fr City",
    "Cistern": "Fr Cistern",
}

# Pressure sensors get their own map: one of them is also called "House", and
# it must not pick up the flow sensor's "To House".
_PRESSURE_LABELS = {
    "Mid-filter": "Mid",  # the full name does not fit beside a reading
}


def _display_label(name):
    return _DISPLAY_LABELS.get(name, name)


def _pressure_label(name):
    return _PRESSURE_LABELS.get(name, name)


_COLORS = {
    # Values >1 (0-100 scale), not 0/1: Character_LCD_RGB.color's non-PWM
    # fallback does `pin.value = not color[number] > 1`, so a channel value
    # of 1 fails that `>1` check and reads as "off" - despite the library's
    # own docstring claiming "non-zero is on". Confirmed empirically
    # 2026-09-27: every entry below evaluated identically to "off" when
    # this dict used 0/1, which is why the backlight looked dead on both
    # the old AND the replacement board - a software bug, not a solder
    # fault. Any value >1 works; 100 matches the library's documented scale.
    "off": [0, 0, 0],
    "red": [100, 0, 0],
    "green": [0, 100, 0],
    "blue": [0, 0, 100],
    "yellow": [100, 100, 0],
    "cyan": [0, 100, 100],
    "magenta": [100, 0, 100],
    "white": [100, 100, 100],
}


def _format_row(label, rate_lpm, num_cols):
    """Same layout as the old FlowMetersPage.create_flow_meter_row_str()."""
    prefix = label + ":"
    if rate_lpm is None or rate_lpm < 0:
        rate_str = "---"
    else:
        gpm = rate_lpm * GALLON_PER_LITER
        # One decimal place below 10 gpm (2026-09-02 fix): the original
        # floor-to-whole-gpm format showed "0gm" for any real flow under
        # 1 gpm — which is most of the realistic range here, especially
        # during bench-test taps and the placeholder ml_per_pulse=9.26
        # calibration, so it looked like nothing was registering at all.
        # Falls back to a floored integer at 10+ gpm to stay inside the
        # width budget (worst case 5 display columns after the longest
        # label, e.g. "Fr Cistern:" — "0.3gm" fits, "12.3gm" wouldn't).
        if gpm < 10:
            rate_str = f"{gpm:.1f}gm"
        else:
            rate_str = f"{int(gpm)}gm"
    return (prefix + rate_str.rjust(max(0, num_cols - len(prefix))))[:num_cols]


def _format_text_row(label, text, num_cols):
    """"<label>:" left, text right-aligned to fill the row."""
    prefix = label + ":"
    return (prefix + text.rjust(max(0, num_cols - len(prefix))))[:num_cols]


def _format_pressure_row(label, psi, num_cols):
    return _format_text_row(label, "---" if psi is None else f"{psi:.1f}psi", num_cols)


class LcdUi(threading.Thread):
    def __init__(self, sensor_names, get_reading, color="red", refresh_s=0.5, poll_s=0.05,
                 pressure_names=(), get_pressure=None, get_total=None):
        super().__init__(daemon=True)
        self._sensor_names = list(sensor_names)
        self._get_reading = get_reading  # callable(name) -> rate_lpm (float) or None
        self._pressure_names = list(pressure_names) if get_pressure else []
        self._get_pressure = get_pressure  # callable(name) -> psi (float) or None
        self._get_total = get_total  # callable(name) -> total gallons (float) or None
        self._color = _COLORS.get(color, _COLORS["red"])
        self._refresh_s = refresh_s
        self._poll_s = poll_s
        self._stop_event = threading.Event()

        i2c = busio.I2C(board.SCL, board.SDA)
        self._lcd = character_lcd.Character_LCD_RGB_I2C(i2c, 16, 2)
        self._num_cols = 16
        self._num_rows = 2

        # one entry per sensor for the button-stepped single-sensor view:
        # flow sensors first, then pressure sensors
        self._items = ([("flow", n) for n in self._sensor_names]
                       + [("pressure", n) for n in self._pressure_names])
        self._item_num = -1  # first press lands on item 0
        self._last_manual_s = -MANUAL_HOLD_S  # rotation active from the start
        # pressure sensors two per page for the rotation
        self._pressure_pages = [
            self._pressure_names[i:i + self._num_rows]
            for i in range(0, len(self._pressure_names), self._num_rows)
        ]

        self._lcd.color = self._color
        self._lcd.display = True
        self._lcd.clear()

    def stop(self):
        self._stop_event.set()

    def _step(self, delta, now):
        if not self._items:
            return
        if now - self._last_manual_s >= MANUAL_HOLD_S:
            # coming from the rotation: start at the first sensor (or the last, going backwards)
            self._item_num = 0 if delta > 0 else len(self._items) - 1
        else:
            self._item_num = (self._item_num + delta) % len(self._items)
        self._last_manual_s = now

    def _active_sensors(self):
        """Sensor names currently flowing, in configured order."""
        active = []
        for name in self._sensor_names:
            rate = self._get_reading(name)
            if rate is not None and rate >= FLOW_EPSILON_LPM:
                active.append(name)
        return active

    def _flow_rows(self, now):
        active = self._active_sensors()
        n = len(active)

        if n == 0:
            return ["No flow".center(self._num_cols), " " * self._num_cols]

        if n == 1:
            return [_format_row(_display_label(active[0]), self._get_reading(active[0]), self._num_cols),
                    " " * self._num_cols]

        if n == 2:
            shown = active
        else:
            # 3 or 4 active: scroll through non-overlapping pairs, wrapping.
            num_windows = (n + 1) // 2
            window_idx = int(now // SCROLL_INTERVAL_S) % num_windows
            start = window_idx * 2
            shown = [active[start % n], active[(start + 1) % n]]

        return [_format_row(_display_label(shown[0]), self._get_reading(shown[0]), self._num_cols),
                _format_row(_display_label(shown[1]), self._get_reading(shown[1]), self._num_cols)]

    def _pressure_rows(self, names):
        rows = [_format_pressure_row(_pressure_label(n), self._get_pressure(n), self._num_cols) for n in names]
        while len(rows) < self._num_rows:
            rows.append(" " * self._num_cols)
        return rows

    def _auto_rows(self, now):
        """Rotation: flow view, then each pressure page."""
        if not self._pressure_pages:
            return self._flow_rows(now)
        cycle = ROTATE_FLOW_S + ROTATE_PRESSURE_S * len(self._pressure_pages)
        pos = now % cycle
        if pos < ROTATE_FLOW_S:
            return self._flow_rows(now)
        page = int((pos - ROTATE_FLOW_S) // ROTATE_PRESSURE_S)
        return self._pressure_rows(self._pressure_pages[min(page, len(self._pressure_pages) - 1)])

    def _manual_rows(self):
        kind, name = self._items[self._item_num]
        if kind == "flow":
            label = _display_label(name)
            total = self._get_total(name) if self._get_total else None
            row2 = " " * self._num_cols if total is None else _format_text_row("Total", f"{total:,.0f}gal", self._num_cols)
            return [_format_row(label, self._get_reading(name), self._num_cols), row2]
        psi = self._get_pressure(name)
        return [(_pressure_label(name) + " pressure")[:self._num_cols].ljust(self._num_cols),
                ("---" if psi is None else f"{psi:.1f} psi").center(self._num_cols)]

    def _draw(self, now):
        if self._items and now - self._last_manual_s < MANUAL_HOLD_S:
            rows = self._manual_rows()
        else:
            rows = self._auto_rows(now)
        # No .clear() here (unlike the original port): every row above is
        # always padded to exactly _num_cols, so .message fully overwrites
        # the previous content on its own — clearing first just produced a
        # visible blank-then-redraw flicker on every refresh for no benefit.
        self._lcd.message = "\n".join(rows)

    def run(self):
        now = time.monotonic()
        self._draw(now)
        last_redraw = now
        was_next = was_prev = False
        while not self._stop_event.is_set():
            now = time.monotonic()

            lcd = self._lcd
            nxt = lcd.down_button or lcd.right_button or lcd.select_button
            prev = lcd.up_button or lcd.left_button
            if nxt and not was_next:
                self._step(+1, now)
                self._draw(now)
                last_redraw = now
            elif prev and not was_prev:
                self._step(-1, now)
                self._draw(now)
                last_redraw = now
            was_next, was_prev = nxt, prev

            if now - last_redraw >= self._refresh_s:
                self._draw(now)
                last_redraw = now

            time.sleep(self._poll_s)

        self._lcd.clear()
        self._lcd.color = _COLORS["off"]
        self._lcd.display = False

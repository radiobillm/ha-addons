"""
Adafruit i2c 16x2 RGB LCD Pi Plate driver + page UI.

Matches the old rpi-flow-meters (ui/cistern_ui.py, ui/flow_meters_page.py,
ui/input_manager.py) behavior exactly:
- UP/DOWN buttons move between pages (verified against that code — SELECT/
  LEFT/RIGHT are not bound to anything, same as before).
- Each page shows up to 2 sensors, one per LCD row, as "<Label>:" left-
  aligned with the flow rate right-aligned to fill the 16 columns, e.g.
  "City:          3gm" (whole gallons/min, floored — same
  format_flow_rate()/create_flow_meter_row_str() logic as the old code).
  With 4 sensors that's 2 pages of 2.

On top of that faithful port, this adds auto-follow: after
AUTO_FOLLOW_IDLE_S seconds with no button press, the display
automatically jumps to whichever page has a sensor currently flowing
(typically only one does at a time), so you don't have to page-hunt to
see what's running. Any button press immediately takes back manual
control and pauses auto-follow for another AUTO_FOLLOW_IDLE_S seconds.

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
AUTO_FOLLOW_IDLE_S = 10  # seconds of no button press before auto-follow kicks in
FLOW_EPSILON_LPM = 0.05  # below this, a sensor counts as "not flowing"

_COLORS = {
    "off": [0, 0, 0],
    "red": [1, 0, 0],
    "green": [0, 1, 0],
    "blue": [0, 0, 1],
    "yellow": [1, 1, 0],
    "cyan": [0, 1, 1],
    "magenta": [1, 0, 1],
    "white": [1, 1, 1],
}


def _format_row(label, rate_lpm, num_cols):
    """Same layout as the old FlowMetersPage.create_flow_meter_row_str()."""
    prefix = label + ":"
    if rate_lpm is None or rate_lpm < 0:
        rate_str = "---"
    else:
        gpm = rate_lpm * GALLON_PER_LITER
        rate_str = f"{int(gpm)}gm"  # floored, matches old format_flow_rate()
    return (prefix + rate_str.rjust(max(0, num_cols - len(prefix))))[:num_cols]


class LcdUi(threading.Thread):
    def __init__(self, sensor_names, get_reading, color="red", refresh_s=0.5, poll_s=0.05):
        super().__init__(daemon=True)
        self._get_reading = get_reading  # callable(name) -> rate_lpm (float) or None
        self._color = _COLORS.get(color, _COLORS["red"])
        self._refresh_s = refresh_s
        self._poll_s = poll_s
        self._stop_event = threading.Event()

        i2c = busio.I2C(board.SCL, board.SDA)
        self._lcd = character_lcd.Character_LCD_RGB_I2C(i2c, 16, 2)
        self._num_cols = 16
        self._num_rows = 2

        # group sensors into pages of `num_rows` each, same grouping as
        # the old CisternUI.create_pages()
        self._pages = [
            sensor_names[i:i + self._num_rows]
            for i in range(0, len(sensor_names), self._num_rows)
        ]
        self._page_num = 0
        self._auto_rotate_idx = 0
        self._last_manual_s = -AUTO_FOLLOW_IDLE_S  # auto-follow active from the start

        self._lcd.color = self._color
        self._lcd.display = True
        self._lcd.clear()

    def stop(self):
        self._stop_event.set()

    def _go_next(self):
        if len(self._pages) > 1:
            self._page_num = (self._page_num + 1) % len(self._pages)

    def _go_prev(self):
        if len(self._pages) > 1:
            self._page_num = (self._page_num - 1) % len(self._pages)

    def _pages_with_flow(self):
        result = []
        for i, page in enumerate(self._pages):
            for name in page:
                rate = self._get_reading(name)
                if rate is not None and rate >= FLOW_EPSILON_LPM:
                    result.append(i)
                    break
        return result

    def _maybe_auto_follow(self, now):
        if now - self._last_manual_s < AUTO_FOLLOW_IDLE_S:
            return
        flowing = self._pages_with_flow()
        if not flowing:
            return
        if self._page_num not in flowing:
            self._auto_rotate_idx = 0
            self._page_num = flowing[0]
        elif len(flowing) > 1:
            # more than one page has flow at once (unusual) - rotate among them
            self._auto_rotate_idx = (self._auto_rotate_idx + 1) % len(flowing)
            self._page_num = flowing[self._auto_rotate_idx]

    def _draw(self):
        page = self._pages[self._page_num]
        rows = [_format_row(name, self._get_reading(name), self._num_cols) for name in page]
        while len(rows) < self._num_rows:
            rows.append(" " * self._num_cols)
        self._lcd.clear()
        self._lcd.message = "\n".join(rows)

    def run(self):
        self._draw()
        last_redraw = time.monotonic()
        was_up = was_down = False
        while not self._stop_event.is_set():
            now = time.monotonic()

            up = self._lcd.up_button
            down = self._lcd.down_button
            if up and not was_up:
                self._go_prev()
                self._last_manual_s = now
                self._draw()
                last_redraw = now
            elif down and not was_down:
                self._go_next()
                self._last_manual_s = now
                self._draw()
                last_redraw = now
            was_up, was_down = up, down

            if now - last_redraw >= self._refresh_s:
                self._maybe_auto_follow(now)
                self._draw()
                last_redraw = now

            time.sleep(self._poll_s)

        self._lcd.clear()
        self._lcd.color = _COLORS["off"]
        self._lcd.display = False

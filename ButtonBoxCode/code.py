import board
from keybow2040 import Keybow2040

import usb_hid
from joystick_xl.joystick import Joystick

js = Joystick()

# Set up Keybow
i2c = board.I2C()
keybow = Keybow2040(i2c)
keys = keybow.keys

# Colors
ACTIVE_COLOR = (0, 55, 255)  # Yellow (active button)
INACTIVE_COLOR = (0, 0, 0)    # Off (inactive buttons)

# Track which button is active
active_button = None  

# Attach handler functions to all of the keys
for key in keys:
    @keybow.on_press(key)
    def press_handler(key):
        global active_button

        # If pressing the currently active button, turn it off
        if active_button == key:
            key.led_off()  # Turn off LED
            active_button = None  # No active button
            js.update_button((key.number, False))
        else:
            # Turn off all other buttons
            for k in keys:
                k.led_off() # Turn off LEDs
                js.update_button((k.number, False))

            # Activate the newly pressed button
            key.set_led(*ACTIVE_COLOR)  # Set LED to active color
            active_button = key  # Store the new active button
            js.update_button((key.number, True))

while True:
    keybow.update()
    js.update()

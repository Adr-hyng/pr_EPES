import RPi.GPIO as GPIO
from time import sleep
#Set warnings off (optional)
#Set Button and LED pins

class ButtonHandler:
    def __init__(
        self, 
        button_pin, 
        output_pin_short, 
        output_pin_long, 
        long_press_time=3, 
        short_press_callback=None, 
        long_press_callback=None,
        short_output_condition=lambda: True,  # Default to always execute
        long_output_condition=lambda: True   # Default to always execute
    ):
        self.button_pin = button_pin
        self.led_pin_short = output_pin_short
        self.led_pin_long = output_pin_long
        self.long_press_time = long_press_time
        self.short_press_callback = short_press_callback
        self.long_press_callback = long_press_callback
        self.short_output_condition = short_output_condition
        self.long_output_condition = long_output_condition

        self.press_start_time = 0
        self.button_state = False
        self.last_button_state = False
        self.long_press_handled = False

        # Blink sequence for long press
        self.long_blink_stage = 0
        self.long_blink_timer = 0

        # Setup GPIO
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.led_pin_short, GPIO.OUT)

        if self.led_pin_long != -1:  # Only setup if valid
            GPIO.setup(self.led_pin_long, GPIO.OUT)

    def update(self, current_time):
        """Update the button state and handle actions."""
        self.button_state = GPIO.input(self.button_pin) == GPIO.HIGH

        if self.button_state and not self.last_button_state:
            # Button just pressed
            self.press_start_time = current_time
            self.long_press_handled = False
            self.long_blink_stage = 0  # Reset blink stage
        elif not self.button_state and self.last_button_state:
            # Button just released
            press_duration = current_time - self.press_start_time

            if not self.long_press_handled:
                # Short press action
                if self.short_output_condition():  # Check short press condition
                    print(f"Button {self.button_pin}: Short press detected!")
                    GPIO.output(self.led_pin_short, GPIO.HIGH)
                    sleep(0.1)  # Keep LED on for 0.1 seconds
                    GPIO.output(self.led_pin_short, GPIO.LOW)

                    # Execute short press callback if provided
                    if self.short_press_callback:
                        self.short_press_callback()

        # Handle long press action only if led_pin_long is valid
        if self.led_pin_long != -1:
            if (
                self.button_state
                and current_time - self.press_start_time >= self.long_press_time
                and not self.long_press_handled
            ):
                # Long press action
                if self.long_output_condition():  # Check long press condition
                    print(f"Button {self.button_pin}: Long press detected!")
                    self.long_press_handled = True
                    self.long_blink_stage = 1  # Start blink sequence
                    self.long_blink_timer = current_time  # Initialize timer

                    # Execute long press callback if provided
                    if self.long_press_callback:
                        self.long_press_callback()

            # Handle long press blink sequence
            if self.long_blink_stage > 0:
                if self.long_blink_stage == 1 and current_time >= self.long_blink_timer:
                    GPIO.output(self.led_pin_long, GPIO.HIGH)  # Turn LED on
                    self.long_blink_timer = current_time + 0.001  # On duration
                    self.long_blink_stage = 2
                elif self.long_blink_stage == 2 and current_time >= self.long_blink_timer:
                    GPIO.output(self.led_pin_long, GPIO.LOW)  # Turn LED off
                    self.long_blink_timer = current_time + 0.005  # Off duration
                    self.long_blink_stage = 3
                elif self.long_blink_stage == 3 and current_time >= self.long_blink_timer:
                    GPIO.output(self.led_pin_long, GPIO.HIGH)  # Turn LED on
                    self.long_blink_timer = current_time + 0.001  # On duration
                    self.long_blink_stage = 4
                elif self.long_blink_stage == 4 and current_time >= self.long_blink_timer:
                    GPIO.output(self.led_pin_long, GPIO.LOW)  # Turn LED off
                    self.long_blink_stage = 0  # Blink sequence completed

        self.last_button_state = self.button_state  # Update last button state

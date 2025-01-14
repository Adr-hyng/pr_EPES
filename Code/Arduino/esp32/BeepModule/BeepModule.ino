// Quick Beeping Sound for button either held or not.

// Pin definitions
const int buttonPin = 3;  // Pushbutton pin
const int buzzerPin = 7;     // LED pin

// Variables
int buzzerState = LOW;          // Current state of the LED
int buttonState;             // Current reading from the button pin
int lastButtonState = HIGH;   // Previous reading from the button pin

bool buttonHandled = false; // Tracks if the button press has been handled

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, buzzerState); // Initialize LED state
}

void beepSound() {
  unsigned long lastDebounceTime = 0; // Last time the button state changed
  unsigned long debounceDelay = 50;   // Debounce time in milliseconds
  unsigned long beepDelay = 100;
  // Read the state of the button
  int reading = digitalRead(buttonPin);

  // Check for button state change (debounce)
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  Serial.println(reading);
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the button state has stabilized
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == HIGH && !buttonHandled) {
        // Turn LED on, then off
        digitalWrite(buzzerPin, HIGH);
        // Don't use this delay, instead just rely on the delay 50
        delay(beepDelay); // Small delay to simulate brief ON state 
        digitalWrite(buzzerPin, LOW);
        buttonHandled = true; // Mark this press as handled
      }
    }

    // Reset buttonHandled when the button is released
    if (buttonState == LOW) {
      buttonHandled = false;
    }
  }

  // Save the current reading as the last state for the next loop
  lastButtonState = reading;
  delay(50);
}

void loop() {
  beepSound();
}

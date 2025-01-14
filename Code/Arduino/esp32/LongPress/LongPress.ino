const int buttonPin = 2;  // Pin connected to the button
const int ledPin1 = 9;   // Pin for short press feedback
const int ledPin2 = 10;  // Pin for long press feedback

unsigned long pressStartTime = 0; // Variable to store when the button press started
bool buttonState = false;         // Current button state
bool lastButtonState = false;     // Previous button state
bool longPressHandled = false;    // Tracks if the long press action has been handled

void setup() {
  pinMode(buttonPin, INPUT); // Configure button pin as input with pull-up resistor
  pinMode(ledPin1, OUTPUT);         // Configure LED pin 1 as output
  pinMode(ledPin2, OUTPUT);         // Configure LED pin 2 as output
  Serial.begin(9600);              // Initialize serial communication
}

void loop() {
  buttonState = digitalRead(buttonPin) == HIGH; // Read button state (active LOW)

  if (buttonState && !lastButtonState) {
    // Button just pressed
    pressStartTime = millis();
    longPressHandled = false; // Reset the long press flag
  } else if (!buttonState && lastButtonState) {
    // Button just released
    unsigned long pressDuration = millis() - pressStartTime;

    if (!longPressHandled) {
      // Short press action (if long press wasn't triggered)
      Serial.println("Short press detected!");
      digitalWrite(ledPin1, HIGH);
      delay(500); // Feedback for short press
      digitalWrite(ledPin1, LOW);
    }
  } else if (buttonState && millis() - pressStartTime >= 3000 && !longPressHandled) {
    // Long press action (after 3 seconds)
    Serial.println("Long press detected!");
    digitalWrite(ledPin2, HIGH);
    delay(500); // Feedback for long press
    digitalWrite(ledPin2, LOW);

    longPressHandled = true; // Mark long press as handled
  }

  lastButtonState = buttonState; // Update last button state
}

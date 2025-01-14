#include "HX711.h"
#include <vector>
#include <math.h>

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 33;

HX711 scale;

// Use your calibrated factor (e.g., from calibrating with 100g)
float calibrationFactor = -3084.7;

// Variables for standard deviation
std::vector<float> weightReadings; // Store the weight readings
const int maxReadings = 20;        // Number of readings to calculate standard deviation

// Lifting control variables
bool isStabilized = false;  // Track stabilization state
float targetWeight = 100.0; // Initial weight to lift
float weightReduction = 0.05;

void setup() {
  Serial.begin(57600);
  Serial.println("HX711 scale demo");

  // Initialize the scale
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibrationFactor); // Set your calibration factor
  scale.tare(); // Reset the scale to 0 assuming no weight is on it

  Serial.println("Place an object to measure its weight.");
}

void loop() {
  if (scale.is_ready()) {
    // Continuously read weight in grams
    float weight = scale.get_units(10); // Average of 10 readings for stability
    weightReadings.push_back(weight);   // Store the weight reading

    // Maintain the maximum number of readings
    if (weightReadings.size() > maxReadings) {
      weightReadings.erase(weightReadings.begin()); // Remove the oldest reading
    }

    // Calculate mean and standard deviation
    if (weightReadings.size() == maxReadings) {
      float mean = calculateMean(weightReadings);
      float stddev = calculateStdDev(weightReadings, mean);

      if (!isStabilized) {
        // Wait until standard deviation is less than 0.5
        if (stddev < 0.3) {
          isStabilized = true; // Stabilization achieved
          Serial.println("Stabilization complete. Starting lifting process.");
        }
      } else {
        // Lifting logic
        if (stddev >= 0.5) {
          targetWeight -= weightReduction; // Reduce weight
          Serial.print("Lifting... Target Weight: ");
          Serial.print(targetWeight, 2);
          Serial.println(" g");
        } 
        // else {
        //   Serial.println("Lifting paused. Standard deviation exceeded 1.");
        //   isStabilized = false; // Reset stabilization to wait again
        // }
      }

      // Display results
      Serial.print("Current Weight: ");
      Serial.print(weight, 2);
      Serial.println(" g");

      Serial.print("Mean: ");
      Serial.print(mean, 2);
      Serial.println(" g");

      Serial.print("Standard Deviation: ");
      Serial.print(stddev, 2);
      Serial.println(" g");
    }
  } else {
    Serial.println("HX711 not found. Check connections.");
  }

  delay(100); // Add a delay to stabilize readings
}

// Function to calculate mean
float calculateMean(const std::vector<float>& readings) {
  float sum = 0.0;
  for (float value : readings) {
    sum += value;
  }
  return sum / readings.size();
}

// Function to calculate standard deviation
float calculateStdDev(const std::vector<float>& readings, float mean) {
  float sumSqDiff = 0.0;
  for (float value : readings) {
    sumSqDiff += pow(value - mean, 2);
  }
  return sqrt(sumSqDiff / readings.size());
}

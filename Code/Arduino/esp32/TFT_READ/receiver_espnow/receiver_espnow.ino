//<App !Start!>
// FILE: [receiver_espnow.ino]
// Created by GUIslice Builder version: [0.17.b34]
//
// GUIslice Builder Generated File
//
// For the latest guides, updates and support view:
// https://github.com/ImpulseAdventure/GUIslice
//
//<App !End!>

// ------------------------------------------------
// Headers to include
// ------------------------------------------------
#include "receiver_espnow_GSLC.h"
#include <esp_now.h>
#include <WiFi.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include "HX711.h"
#include <vector>
#include <math.h>
#include <functional>

// ------------------------------------------------
// Program Globals
#include <functional>

class Timer {
public:
  Timer(unsigned long interval, std::function<void()> callback)
    : interval(interval), callback(callback), lastExecutionTime(0) {}

  void check() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastExecutionTime >= interval) {
      lastExecutionTime = currentMillis;
      callback();
    }
  }

private:
  unsigned long interval;
  std::function<void()> callback;
  unsigned long lastExecutionTime;
};

class RunTimeout {
public:
  RunTimeout(unsigned long timeout, std::function<void()> callback)
    : timeout(timeout), callback(callback), startTime(millis()), executed(false) {}

  void check() {
    if (!executed && millis() - startTime >= timeout) {
      executed = true;
      callback();
    }
  }

  void reset(unsigned long newTimeout = 0) {
    startTime = millis();
    executed = false;
    if (newTimeout > 0) {
      timeout = newTimeout;
    }
  }

private:
  unsigned long timeout;
  std::function<void()> callback;
  unsigned long startTime;
  bool executed;
};



// Must match the sender structure
uint8_t broadcastAddress[] = {0x34, 0x5f, 0x45, 0xa9, 0xc1, 0x08}; // New ESP
esp_now_peer_info_t peerInfo;
typedef struct struct_message {
  short ContainerCap; // Send to Raspberry Pi
  short Mode; // Send to Raspberry Pi
  short CurTemperature; // Send to Raspberry Pi
  bool isPushed; // Send to Raspberry Pi
  short SelTemp_MRange;
  bool heaterActivated;
  bool isStablized;
  bool childLockActivated;
  bool resetMode;
} struct_message;
struct_message myData;
bool redrawRequired = false;

// Temperature Sensor
const int oneWireBus = 22;  
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// Analog Joystick
const int joystick_y_pin = 34;

// Weight Sensor
// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 33;
HX711 scale;
float calibrationFactor = -3084.7;
// ------------------------------------------------

// Save some element references for direct access
//<Save_References !Start!>
gslc_tsElemRef* m_pElemOutCurTemp = NULL;
gslc_tsElemRef* m_pElemOutSelTemp = NULL;
gslc_tsElemRef* m_pElemOutThermometerIMG= NULL;
gslc_tsElemRef* m_pElemXRingGauge1= NULL;
gslc_tsElemRef* m_pElemHeaterState= NULL;
//<Save_References !End!>

// Define debug message function
static int16_t DebugOut(char ch) { if (ch == (char)'\n') Serial.println(""); else Serial.write(ch); return 0; }

// ------------------------------------------------
// Callback Methods
// ------------------------------------------------
void initializeScale() {
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibrationFactor);
  scale.tare();
}
void execEveryMillis(unsigned long interval, std::function<void()> callback) {
  static unsigned long lastExecutionTime = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastExecutionTime >= interval) {
    lastExecutionTime = currentMillis;
    callback();
  }
}
RunTimeout goBackToDefault(2000, []() {
  gslc_ElemXRingGaugeSetColorActiveFlat(&m_gui, m_pElemXRingGauge1, gslc_tsColor(255, 255, 255));
  myData.isStablized = false;
});
// Function to handle weight reading, stabilization, and lifting
void handleWeightMeasurement(float& targetWeight, bool& isStabilized, float weightReduction = 0.05, float stabilizationThreshold = 0.3, int maxReadings = 20) {
  static std::vector<float> weightReadings;
  float initialVal = -1;
  
  if (scale.is_ready()) {
    float weight = scale.get_units(10); // Average of 10 readings
    weightReadings.push_back(weight);

    // Maintain the maximum number of readings
    if (weightReadings.size() > maxReadings) {
      weightReadings.erase(weightReadings.begin());
    }

    if (weightReadings.size() == maxReadings) {
      // Calculate mean and standard deviation
      float mean = calculateMean(weightReadings);
      float stddev = calculateStdDev(weightReadings, mean);

      if (!isStabilized && stddev < stabilizationThreshold) {
        // Turn green the gauge
        isStabilized = true;
        // Serial.println("Stabilization complete.");
        gslc_ElemXRingGaugeSetColorActiveFlat(&m_gui, m_pElemXRingGauge1, gslc_tsColor(0, 255, 0));
        myData.isStablized = true;
        
      } else if (isStabilized && stddev >= stabilizationThreshold) {
        goBackToDefault.check();
        targetWeight -= weightReduction; // Adjust target weight
        // Serial.print("Lifting... Target Weight: ");
        // Serial.print(targetWeight, 2);
        // Serial.println(" g");
        myData.isStablized = false;
        myData.ContainerCap = targetWeight;
      }

      if(!(initialVal >= mean - 50) && isStabilized) { // is Empty and stablized 
      // Not working
        targetWeight = 0;
      }

      // // Display results
      // Serial.print("Initial Weight: ");
      // Serial.print(initialVal, 2);
      // Serial.println(" g");
      // Serial.print("Mean: ");
      // Serial.print(mean, 2);
      // Serial.println(" g");
      // Serial.print("Standard Deviation: ");
      // Serial.print(stddev, 2);
      // Serial.println(" g");
    }
  } else {
    Serial.println("HX711 not found. Check connections.");
  }
}

// Helper functions
float calculateMean(const std::vector<float>& readings) {
  float sum = 0.0;
  for (float value : readings) {
    sum += value;
  }
  return sum / readings.size();
}

float calculateStdDev(const std::vector<float>& readings, float mean) {
  float sumSqDiff = 0.0;
  for (float value : readings) {
    sumSqDiff += pow(value - mean, 2);
  }
  return sqrt(sumSqDiff / readings.size());
}
void sendData()
{
  // Set values to send
  myData.Mode = myData.Mode;
  myData.childLockActivated = myData.childLockActivated;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  gslc_ElemXRingGaugeSetVal(&m_gui, m_pElemXRingGauge1, myData.ContainerCap);
  // if (result == ESP_OK) {
  //   Serial.println("Sent with success");
  // } else {
  //   Serial.println("Error sending the data");
  // }
}
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  // Serial.print("Capacity: ");
  // Serial.println(myData.ContainerCap);
  // Serial.print("Mode: ");
  // Serial.println(myData.Mode);
  // Serial.print("Current Temp.: ");
  // Serial.println(myData.CurTemperature);
  // Serial.println();
  
  gslc_ElemXRingGaugeSetVal(&m_gui, m_pElemXRingGauge1, myData.ContainerCap);
  redrawRequired = true;
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  return;
}
bool onStateChange(bool& currentState, bool newValue) {
  static bool previousState = false;
  if (currentState != newValue) {
    previousState = currentState;
    currentState = newValue;
    return true; // Change detected
  }
  return false; // No change
}
void UpdateThermometerImage() {
    static short lastSelTemp_MRange = -1; // Initialize with an invalid value

    if (myData.SelTemp_MRange != lastSelTemp_MRange) {
        lastSelTemp_MRange = myData.SelTemp_MRange; // Update the last known value

        if (myData.SelTemp_MRange == 85) {
            UpdateBitmapImage(&m_gui, m_pElemOutThermometerIMG, THERMOMETER);
        } else if (myData.SelTemp_MRange == 75) {
            UpdateBitmapImage(&m_gui, m_pElemOutThermometerIMG, THERMOMETER_1);
        } else {
            UpdateBitmapImage(&m_gui, m_pElemOutThermometerIMG, THERMOMETER_2);
        }

        static char TSelTemp[4] = ""; // Ensure TCurTemp is properly declared
        snprintf(TSelTemp, sizeof(TSelTemp), "%d", myData.SelTemp_MRange);
        if (m_pElemOutSelTemp != NULL) {
          gslc_ElemSetTxtStr(&m_gui, m_pElemOutSelTemp, TSelTemp);
        } 

        // gslc_ElemSetRedraw(&m_gui, m_pElemOutSelTemp, GSLC_REDRAW_FOCUS);
    }
}
void UpdateBitmapImage(gslc_tsGui* pGui, gslc_tsElemRef* pElemRef, const unsigned short* newBitmap) 
{
    if (pElemRef == nullptr) {
        return; // Element reference is invalid
    }

    // Clear the element's background
    // gslc_ElemSetRedraw(pGui, pElemRef, GSLC_REDRAW_FOCUS);

    // Update the image reference
    gslc_tsImgRef updImgRef = gslc_GetImageFromProg((const unsigned char*)newBitmap, GSLC_IMGREF_FMT_BMP24);
    gslc_ElemSetImage(pGui, pElemRef, updImgRef, updImgRef);

    // Trigger a redraw of the element to display the new image
    gslc_ElemSetRedraw(pGui, pElemRef, GSLC_REDRAW_FOCUS);
}
// Common Button callback
bool CbBtnCommon(void* pvGui,void *pvElemRef,gslc_teTouch eTouch,int16_t nX,int16_t nY)
{
  // Typecast the parameters to match the GUI and element types
  gslc_tsGui*     pGui     = (gslc_tsGui*)(pvGui);
  gslc_tsElemRef* pElemRef = (gslc_tsElemRef*)(pvElemRef);
  gslc_tsElem*    pElem    = gslc_GetElemFromRef(pGui,pElemRef);

  if ( eTouch == GSLC_TOUCH_UP_IN ) {
    // From the element's ID we can determine which button was pressed.
    switch (pElem->nId) {
//<Button Enums !Start!>
      case HeaterIndicator:
        break;
//<Button Enums !End!>
      default:
        break;
    }
  }
  return true;
}
//<Checkbox Callback !Start!>
//<Checkbox Callback !End!>
//<Keypad Callback !Start!>
//<Keypad Callback !End!>
//<Spinner Callback !Start!>
//<Spinner Callback !End!>
//<Listbox Callback !Start!>
//<Listbox Callback !End!>
//<Draw Callback !Start!>
//<Draw Callback !End!>
//<Slider Callback !Start!>
//<Slider Callback !End!>
//<Tick Callback !Start!>
//<Tick Callback !End!>

void setup()
{
  // ----------------------4--------------------------
  // Initialize
  // ------------------------------------------------
  myData.ContainerCap = 0;
  Serial.begin(115200);
  sensors.begin();
  initializeScale();
  // Wait for USB Serial 
  //delay(1000);  // NOTE: Some devices require a delay after Serial.begin() before serial port can be used

  gslc_InitDebug(&DebugOut);

  // ------------------------------------------------
  // Create graphic elements
  // ------------------------------------------------
  InitGUIslice_gen();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    // Serial.println("Error initializing ESP-NOW")
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

   // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    // Serial.println("Failed to add peer");
    return;
  }

  myData.resetMode = 1;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  delay(50);
  myData.resetMode = 0;
}

Timer timer1(500, []() {
  static float currentWaterCapacity = 50.0;
  static bool isStabilized = false;
  handleWeightMeasurement(currentWaterCapacity, isStabilized, 0.2);
  Serial.print(1);
  Serial.print(",");
  Serial.print(myData.ContainerCap);
  Serial.print(",");
  Serial.print(myData.Mode);
  Serial.print(",");
  Serial.print(myData.CurTemperature);
  Serial.print(",");
  Serial.print(myData.isPushed);
  Serial.println();
});

Timer timer2(100, []() {
  int y_adc_val; 
  float y_volt;
  y_adc_val = analogRead(joystick_y_pin);
  y_volt = ( (y_adc_val * 3.3 ) / 4095 );  /*Convert digital value to voltage */
  myData.isPushed = (y_volt <= 1.3 && y_volt >= 0.6) || y_volt <= 0.4;
});
Timer timer3(10000, []() {
  sensors.requestTemperatures(); 
  myData.CurTemperature = sensors.getTempCByIndex(0);
  static char TCurTemp[4] = ""; // Ensure TCurTemp is properly declared
  snprintf(TCurTemp, sizeof(TCurTemp), "%d", myData.CurTemperature);
  if (m_pElemOutCurTemp != NULL) {
    gslc_ElemSetTxtStr(&m_gui, m_pElemOutCurTemp, TCurTemp);
  } 
  // redrawRequired = true;
});

Timer timer4(1000, []() {
  sendData();
});

// -----------------------------------
// Main event loop
// -----------------------------------
void loop()
{
  // ------------------------------------------------
  if (Serial.available() > 0) { // Check if data is available
    String receivedData = Serial.readStringUntil('\n'); // Read until newline
    // Parse the data
    int dummy; // Placeholder for the first value (0)
    int tempSelTemp_MRange, tempHeaterActivated, tempChildLockActivated, tempResetMode; // Temporary variables for parsing
    int parsedValues = sscanf(receivedData.c_str(), "%d,%d,%d,%d,%d",
                               &dummy,
                               &tempSelTemp_MRange,
                               &tempHeaterActivated,
                               &tempChildLockActivated,
                               &tempResetMode);
    if (parsedValues == 5) {
      if (dummy == 0) { // Only assign values if dummy is 0
        myData.SelTemp_MRange = tempSelTemp_MRange;

        if (onStateChange(myData.resetMode, tempResetMode)) {
          ESP.restart();
        }

        UpdateThermometerImage();

        if(onStateChange(myData.heaterActivated, tempHeaterActivated)) {
          m_pElemHeaterState = gslc_PageFindElemById(&m_gui, gslc_GetPageCur(&m_gui), HeaterIndicator);
          gslc_ElemSetGlowEn(&m_gui, m_pElemHeaterState, myData.heaterActivated);
          gslc_ElemSetGlow(&m_gui, m_pElemHeaterState, myData.heaterActivated);
        }

        if (onStateChange(myData.childLockActivated, tempChildLockActivated)) {
          sendData();
        }
      }
    }
  }
  timer1.check();
  timer2.check();
  timer3.check();
  timer4.check();

  // Update GUI Elements
  if (redrawRequired) {
    gslc_ElemSetRedraw(&m_gui, m_pElemOutCurTemp, GSLC_REDRAW_FOCUS);
    redrawRequired = false; // Reset flag after redraw
  }
  // ------------------------------------------------
  
  //TODO - Add update code for any text, gauges, or sliders
  
  // ------------------------------------------------
  // Periodically call GUIslice update function
  // ------------------------------------------------
  gslc_Update(&m_gui);
}


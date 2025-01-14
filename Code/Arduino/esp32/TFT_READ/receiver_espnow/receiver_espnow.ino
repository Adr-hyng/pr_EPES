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

// ------------------------------------------------
// Program Globals
// Must match the sender structure
uint8_t broadcastAddress[] = {0x34, 0x5f, 0x45, 0xa9, 0xc1, 0x08}; // New ESP
esp_now_peer_info_t peerInfo;
typedef struct struct_message {
  short ContainerCap;
  short Mode;
  short CLstate; // child lock
  short Hstate; // heat lock
  short TLstate; // temp lock
  short CurTemperature;
  short SelTemp_MRange;
  bool isPushed;
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
// Function to handle weight reading, stabilization, and lifting
void handleWeightMeasurement(float& targetWeight, bool& isStabilized, float weightReduction = 0.05, float stabilizationThreshold = 0.3, int maxReadings = 20) {
  static std::vector<float> weightReadings;
  
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
        isStabilized = true;
        Serial.println("Stabilization complete.");
      } else if (isStabilized && stddev >= stabilizationThreshold) {
        targetWeight -= weightReduction; // Adjust target weight
        Serial.print("Lifting... Target Weight: ");
        Serial.print(targetWeight, 2);
        Serial.println(" g");
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
  // myData.CLstate = load_from_nvs(NVS_KEY_LOCK_MODE, 0);
  // myData.Hstate = load_from_nvs(NVS_KEY_LOCK_MODE, 0);
  // myData.TLstate = load_from_nvs(NVS_KEY_LOCK_MODE, 0);
  sensors.requestTemperatures(); 
  myData.CurTemperature = sensors.getTempCByIndex(0);

  myData.SelTemp_MRange = random(50, 75);


  int y_adc_val; 
  float y_volt;
  y_adc_val = analogRead(joystick_y_pin);
  y_volt = ( (y_adc_val * 3.3 ) / 4095 );  /*Convert digital value to voltage */
  myData.isPushed = y_volt <= 1.3;

  Serial.println(y_volt);

  static float currentWaterCapacity = 100.0;
  static bool isStabilized = false;
  handleWeightMeasurement(currentWaterCapacity, isStabilized);
  myData.ContainerCap = currentWaterCapacity;
  
  static char TCurTemp[4] = ""; // Ensure TCurTemp is properly declared
  snprintf(TCurTemp, sizeof(TCurTemp), "%d", myData.CurTemperature);
  if (m_pElemOutCurTemp != NULL) {
    gslc_ElemSetTxtStr(&m_gui, m_pElemOutCurTemp, TCurTemp);
    // gslc_ElemSetRedraw(&m_gui, m_pElemOutCurTemp, GSLC_REDRAW_FULL);
  } else {
      Serial.println("Error: m_pElemOutCurTemp is NULL");
  }
  // redrawRequired = true;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  gslc_ElemXRingGaugeSetVal(&m_gui, m_pElemXRingGauge1, myData.ContainerCap);
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Capacity: ");
  Serial.println(myData.ContainerCap);
  Serial.print("Mode: ");
  Serial.println(myData.Mode);
  Serial.print("Child-Lock: ");
  Serial.println(myData.CLstate);
  Serial.print("Heater: ");
  Serial.println(myData.Hstate);
  Serial.print("Temperature Lock: ");
  Serial.println(myData.TLstate);
  Serial.print("Current Temp.: ");
  Serial.println(myData.CurTemperature);
  Serial.print("Selected Max Temp: ");
  Serial.println(myData.SelTemp_MRange);
  Serial.println();

  gslc_ElemXRingGaugeSetVal(&m_gui, m_pElemXRingGauge1, myData.ContainerCap);
  static char TCurTemp[4] = ""; // Ensure TCurTemp is properly declared
  snprintf(TCurTemp, sizeof(TCurTemp), "%d", myData.CurTemperature);
  if (m_pElemOutCurTemp != NULL) {
    gslc_ElemSetTxtStr(&m_gui, m_pElemOutCurTemp, TCurTemp);
    // gslc_ElemSetRedraw(&m_gui, m_pElemOutCurTemp, GSLC_REDRAW_FULL);
  } else {
      Serial.println("Error: m_pElemOutCurTemp is NULL");
  }
  redrawRequired = true;
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
void execEveryMillis(unsigned long interval, void (*callback)())
{
  static unsigned long lastExecutionTime = 0;
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastExecutionTime >= interval) {
    lastExecutionTime = currentMillis;
    callback();
  }
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
  // ------------------------------------------------
  // Initialize
  // ------------------------------------------------
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
    Serial.println("Error initializing ESP-NOW");
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
    Serial.println("Failed to add peer");
    return;
  }
}

// -----------------------------------
// Main event loop
// -----------------------------------
void loop()
{

  // ------------------------------------------------

  execEveryMillis(1000, sendData);

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

